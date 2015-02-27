/**
********************************************************************************
\file   drvintf.c

\brief  Interface module for application interface to kernel daemon in Windows

This module handles all the application request forwarded to the daemon
in Windows kernel. It uses dualprocshm and circbuf libraries to manage PDO
memory, error objects shared memory, event and DLL queues.

The module also implements mapping of kernel memory into user space to provide
direct access to user application for specific shared memory regions.

\ingroup module_driver_ndispcie
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <ndis.h>
#include "drvintf.h"

#include <ndis-intf.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PROC_ID                        0xFA
#define BENCHMARK_OFFSET               0x00001000 //TODO: Get this value from PCIe header files
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
UINT8*               benchmarkBase_g;          ///< Pointer to user benchmark

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Mapped memory information

This structure stores the information for a memory shared between user and
kernel space.
*/
typedef struct
{
    PMDL      pMdl;                 ///< Memory descriptor list describing the memory.
    size_t    memSize;              ///< Size of memory
    void*     pKernelVa;            ///< Pointer to memory in kernel space.
    void*     pUserVa;              ///< Pointer to memory mapped in user space.
} tMemInfo;

/**
\brief Driver interface instance

Local parameters used by PCIe driver instance
*/
typedef struct
{
    tMemInfo                benchmarkMem;                       ///< Benchmark memory information mapped to user space.
    tMemInfo                kernel2UserMem;                     ///< Kernel to user mapped memory.
    BOOLEAN                 fDriverActive;                      ///< Flag to identify status of driver interface.
}tDriverInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDriverInstance    drvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int        mapMemory(tMemInfo* pMemInfo_p);
static void       unmapMemory(tMemInfo* pMemInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get Benchmark Base

Retrieves the benchmark memory from NDIS driver and maps it into user virtual
address space for accessing for user layer.

\param  ppBenchmarkMem_p    Pointer to benchmark memory.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
int drv_getBenchmarkMem(UINT8** ppBenchmarkMem_p)
{
    UINT8*      pMem;
    tMemInfo*   pBenchmarkMemInfo = &drvInstance_l.benchmarkMem;

    // Check if memory is already allocated and mapped
    if (pBenchmarkMemInfo->pUserVa != NULL)
        goto Exit;

    pMem = (UINT8*) ndis_getBarAddr(1);

    if (pMem == NULL)
        return -1;

    pBenchmarkMemInfo->pKernelVa = pMem + BENCHMARK_OFFSET;
    pBenchmarkMemInfo->memSize = 4;

    if (mapMemory(pBenchmarkMemInfo) != 0)
    {
        TRACE("%s() error mapping memory\n", __FUNCTION__);
        return -1;
    }

    benchmarkBase_g = pBenchmarkMemInfo->pKernelVa;
Exit:
    *ppBenchmarkMem_p = pBenchmarkMemInfo->pUserVa;

    TRACE("%s() Benchmark memory address in user space %p\n", __FUNCTION__, pBenchmarkMemInfo->pUserVa);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Free Benchmark memory

Frees the benchmark memory previously allocated.

\param  pBenchmarkMem_p    Pointer to benchmark memory.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_freeBenchmarkMem(UINT8* pBenchmarkMem_p)
{
    tMemInfo*   pBenchmarkMemInfo = &drvInstance_l.benchmarkMem;

    unmapMemory(pBenchmarkMemInfo);

    pBenchmarkMem_p = NULL;
}

UINT8* drv_getUserBenchmarkBase(void)
{
    if (drvInstance_l.benchmarkMem.pKernelVa != NULL)
        return drvInstance_l.benchmarkMem.pKernelVa;

    return NULL;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{


//------------------------------------------------------------------------------
/**
\brief  Map memory to user space

Maps the specified memory into user space.

\param  pMemInfo_p          Pointer memory map information structure for the
                            memory to map.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static int mapMemory(tMemInfo* pMemInfo_p)
{
    if(pMemInfo_p->pKernelVa == NULL )
        return -1;

    // Allocate new MDL pointing to PDO memory
    pMemInfo_p->pMdl = IoAllocateMdl(pMemInfo_p->pKernelVa, pMemInfo_p->memSize,
                                      FALSE, FALSE, NULL);

    if (pMemInfo_p->pMdl == NULL)
    {
        TRACE("%s() Error allocating MDL !\n", __FUNCTION__);
        return -1;
    }

    // Update the MDL with physical addresses
    MmBuildMdlForNonPagedPool(pMemInfo_p->pMdl);
    // Map the memory in user space and get the address
    pMemInfo_p->pUserVa = MmMapLockedPagesSpecifyCache(pMemInfo_p->pMdl,        // MDL
                                                        UserMode,               // Mode
                                                        MmCached,               // Caching
                                                        NULL,                   // Address
                                                        FALSE,                  // Bugcheck?
                                                        NormalPagePriority);    // Priority

    if (pMemInfo_p->pUserVa == NULL)
    {
        MmUnmapLockedPages(pMemInfo_p->pUserVa, pMemInfo_p->pMdl);
        IoFreeMdl(pMemInfo_p->pMdl);
        TRACE("%s() Error mapping MDL !\n", __FUNCTION__);
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap memory for user space

Unmap the specified memory mapped into user space

\param  pMemInfo_p          Pointer memory map information structure for the
memory to map.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static void unmapMemory(tMemInfo* pMemInfo_p)
{
    if (pMemInfo_p->pMdl == NULL)
    {
        TRACE("%s() MDL already deleted !\n", __FUNCTION__);
        return;
    }

    if (pMemInfo_p->pUserVa != NULL)
    {
        MmUnmapLockedPages(pMemInfo_p->pUserVa, pMemInfo_p->pMdl);
        IoFreeMdl(pMemInfo_p->pMdl);
    }
}
///\}

