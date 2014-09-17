/**
********************************************************************************
\file   pdoucalmem-winioctl.c

\brief  PDO user CAL shared-memory module using the Windows kernel driver

This file implements the PDO CAL memory module to acquire the memory resources
for PDO exchange between user-kernel layers for a Windows user-kernel interface
using ioctl.

The kernel driver is responsible for allocating the shared memory for the PDOs
and map virtual address in user space. The user layer then retrieves the virtual
address for the memory in a ioctl call.

\ingroup module_pdoucal
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
#include <oplk/oplkinc.h>

#include <common/pdo.h>
#include <user/ctrlucal.h>

#include <common/driver.h>
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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
HANDLE    pFileHandle_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Open PDO shared memory

The function performs all actions needed to setup the shared memory at the
start of the stack.

For the Linux kernel driver shared memory acces we need to get the device
descriptor of the kernel driver.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_openMem(void)
{
    pFileHandle_l = ctrlucal_getFd();
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close PDO shared memory

The function performs all actions needed to clean up the shared memory at
shutdown.

For the Linux kernel mmap implementation nothing needs to be done.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_closeMem(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate PDO shared memory

The function allocates shared memory for the user needed to transfer the PDOs.

\param  memSize_p               Size of PDO memory.
\param  ppPdoMem_p              Pointer to store the PDO memory pointer.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_allocateMem(size_t memSize_p, BYTE** ppPdoMem_p)
{
    ULONG      bytesReturned;
    tPdoMem    inPdoMem;
    tPdoMem    outPdoMem;

    inPdoMem.memSize = memSize_p;

    printf("Allocate PDO memory %d\n", memSize_p);
    if (!DeviceIoControl(pFileHandle_l, PLK_CMD_PDO_GET_MEM,
                         &inPdoMem, sizeof(tPdoMem), &outPdoMem, sizeof(tPdoMem),
                         &bytesReturned, NULL))
    {
        printf("Error in PDO memory allocation %x\n", GetLastError());
        *ppPdoMem_p = NULL;
        return kErrorNoResource;
    }

    if (bytesReturned != 0)
    {
        if (outPdoMem.pPdoAddr != NULL)
        {
            *ppPdoMem_p = outPdoMem.pPdoAddr;
            printf("Pdo Address %x\n", outPdoMem.pPdoAddr);
        }
        else
        {
            printf("Allocation of PDO memory Failed\n");
            *ppPdoMem_p = NULL;
            return kErrorNoResource;
        }
    }
    else
    {
        *ppPdoMem_p = NULL;
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO shared memory

The function frees shared memory which was allocated in the user layer for
transfering the PDOs.

\param  pMem_p                  Pointer to the shared memory segment.
\param  memSize_p               Size of PDO memory

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_freeMem(BYTE* pMem_p, size_t memSize_p)
{
    ULONG      bytesReturned;
    tPdoMem    PdoMem;

    PdoMem.memSize = memSize_p;

    if (!DeviceIoControl(pFileHandle_l, PLK_CMD_PDO_FREE_MEM,
                         &PdoMem, sizeof(tPdoMem), NULL, 0,
                         &bytesReturned, NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to free mem %d\n", __func__, GetLastError());
        return kErrorGeneralError;
    }

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}

