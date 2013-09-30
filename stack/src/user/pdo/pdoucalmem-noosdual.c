/**
********************************************************************************
\file   pdoucalmem-noosdual.c

\brief  PDO user CAL shared-memory module using dual processor library

This file contains an implementation for the user PDO CAL shared-memory
module which uses dual processor library to access it. The shared memory is used
to transfer PDO data between user and kernel layer. This implementation is used if
user and kernel layer are on two different processors using a common memory.

\ingroup module_pdoucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012 Kalycito Infotech Private Limited
              www.kalycito.com
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
#include <EplInc.h>

#include <pdo.h>
#include <user/ctrlucal.h>

#include <dualprocshm.h>

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
#define DUALPROCSHM_BUFF_ID_PDO     8
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tDualprocDrvInstance    pDrvInstance;
} tMemInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tMemInstance         memPdo_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Open PDO shared memory

The function performs all actions needed to setup the shared memory at
starting of the stack.

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_openMem(void)
{
    tDualprocDrvInstance pInstance = dualprocshm_getDrvInst(kDualProcHost);

    if(pInstance == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't get Host dual proc driver instance\n",
                __func__);
        return kEplNoResource;
    }

    memPdo_l.pDrvInstance = pInstance;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Close PDO shared memory

The function performs all actions needed to cleanup the shared memory at
shutdown.

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_closeMem(void)
{
    memPdo_l.pDrvInstance = NULL;
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate PDO shared memory

The function allocates shared memory for the kernel needed to transfer the PDOs.

\param  memSize_p               Size of PDO memory
\param  ppPdoMem_p              Pointer to store the PDO memory pointer.

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_allocateMem(size_t memSize_p, BYTE** ppPdoMem_p)
{
    tDualprocReturn dualRet;

    dualRet = dualprocshm_getMemory(memPdo_l.pDrvInstance, \
                                    DUALPROCSHM_BUFF_ID_PDO,ppPdoMem_p,&memSize_p,FALSE);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't allocate Pdo buffer (%d)\n",
                __func__, dualRet);
        return kEplNoResource;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO shared memory

The function frees shared memory which was allocated in the kernel layer for
transferring the PDOs.

\param  pMem_p                  Pointer to the shared memory segment.
\param  memSize_p               Size of PDO memory

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_freeMem(BYTE* pMem_p, size_t memSize_p)
{
    tDualprocReturn dualRet;

    UNUSED_PARAMETER(pMem_p);
    UNUSED_PARAMETER(memSize_p);

    TRACE("%s() try to free address %p\n",
            __func__, pMem_p);


    dualRet = dualprocshm_freeMemory(memPdo_l.pDrvInstance, DUALPROCSHM_BUFF_ID_PDO,FALSE);
    if(dualRet != kDualprocSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't free Pdo buffer (%d)\n",
                __func__, dualRet);
        return kEplNoResource;
    }

    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{



///\}
