/**
********************************************************************************
\file   memmap-winpcie.c

\brief  Memory mapping implementation for Windows with an external PCIe POWERLINK
        processor.

This file contains the architecture specific memory mapping implementation
for Windows system with external PCIe POWERLINK processor. This module enables
the user application in Windows to map a specific memory on PCIe card into
Windows user space.

It use Windows IOCTL to request mapping of a specific memory in PCIe into
Windows user space.

\ingroup module_lib_memmap
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <common/oplkinc.h>
#include <common/memmap.h>
#include <common/target.h>
#include <common/driver.h>
#include <user/ctrlucal.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

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
\brief Local instance

Structure to hold all local parameters.
 
*/
typedef struct
{
    UINT8*      pKernelBase;        ///< Pointer to the kernel base address.
    UINT8*      pUserBase;          ///< Pointer to the user base address.
    HANDLE      pFileHandle;        ///< File handle for the POWERLINK driver.
    tMemStruc   memStruc;           ///< Shared memory structure to exchange addresses.
} tMemMapInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tMemMapInstance      memMapInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize memory mapping

The function initializes the memory mapping service.

\return The function returns a tMemMapReturn error code.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
tMemMapReturn memmap_init(void)
{
    ULONG       bytesReturned;
    tMemStruc   inMemStruc;
    tMemStruc*  pOutMemStruc = &memMapInstance_l.memStruc;

    memMapInstance_l.pFileHandle = (HANDLE)ctrlucal_getFd();

    if (memMapInstance_l.pFileHandle == NULL)
        return kMemMapNoResource;

    if (!DeviceIoControl(memMapInstance_l.pFileHandle, PLK_CMD_MAP_MEM,
        &inMemStruc, sizeof(tMemStruc), pOutMemStruc, sizeof(tMemStruc),
        &bytesReturned, NULL))
    {
        return kMemMapNoResource;
    }

    if (bytesReturned == 0 || pOutMemStruc->pUserAddr == NULL)
        return kMemMapNoResource;

    return kMemMapOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down memory mapping

The function shuts down the memory mapping service.

\return The function returns a tMemMapReturn error code.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
tMemMapReturn memmap_shutdown(void)
{
    tMemStruc*  pMemStruc = &memMapInstance_l.memStruc;
    ULONG       bytesReturned;

    if (!DeviceIoControl(memMapInstance_l.pFileHandle, PLK_CMD_UNMAP_MEM,
        pMemStruc, sizeof(tMemStruc), NULL, 0,
        &bytesReturned, NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to free mem %d\n", __func__, GetLastError());
        return kErrorGeneralError;
    }

    pMemStruc->pKernelAddr = NULL;
    pMemStruc->pUserAddr = NULL;
    pMemStruc->size = 0;
    memMapInstance_l.pFileHandle = NULL;

    return kMemMapOk;
}

//------------------------------------------------------------------------------
/**
\brief  Map kernel buffer

The function maps a kernel buffer address.

\param  pKernelBuffer_p     The pointer to the kernel buffer.

\return The functions returns the pointer to the mapped kernel buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void* memmap_mapKernelBuffer(void* pKernelBuffer_p, UINT bufferSize_p)
{
    INT32       offset;
    UINT8*      pUserAddr;
    tMemStruc*  pMemStruc = &memMapInstance_l.memStruc;

    offset = (UINT)((UINT8*)pKernelBuffer_p - (UINT8*)pMemStruc->pKernelAddr);

    // Negative offset not possible
    if (offset < 0 || (((UINT8*)pKernelBuffer_p + bufferSize_p) >
        ((UINT8*)pMemStruc->pKernelAddr + pMemStruc->size)))
       return NULL;

    pUserAddr = (UINT8*)pMemStruc->pUserAddr + offset;

    return pUserAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Disconnect from a memory mapping

The function disconnects from a memory mapping.

\param  pBuffer_p           The pointer the previously mapped buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void memmap_unmapKernelBuffer(void* pBuffer_p)
{
    UNUSED_PARAMETER(pBuffer_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
