/**
********************************************************************************
\file   drvintf.c

\brief  Interface module for application interface to kernel daemon in Windows

// TODO: Add description

\ingroup module_driver_ndisim
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
#include <oplk/oplk.h>

#include <common/driver.h>
#include <common/ctrl.h>
#include <common/ctrlcal-mem.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <errhndkcal.h>

#include <drvintf.h>

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Execute a control command from user application

This function parsed the control command from user and passes it to the control
module for processing. The return value is again passed to user by copying it
into the common control structure.

\param  ctrlCmd_p       Pointer to control command structure.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_executeCmd(tCtrlCmd* pCtrlCmd_p)
{
    tOplkError    ret;
    UINT16        status;

    ctrlk_executeCmd(pCtrlCmd_p->cmd,(UINT16*) &ret, &status, NULL);
    pCtrlCmd_p->cmd = 0;
    pCtrlCmd_p->retVal = ret;
    ctrlkcal_setStatus(status);
}

//------------------------------------------------------------------------------
/**
\brief  Read initialization parameters

Read the initialization parameters from the kernel stack.

\param  pInitParam_p       Pointer to initialization parameters structure.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_readInitParam(tCtrlInitParam* pInitParam_p)
{
    ctrlkcal_readInitParam(pInitParam_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write initialization parameters

Write the initialization parameters from the user into kernel memory.

\param  pInitParam_p       Pointer to initialization parameters structure.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    ctrlkcal_storeInitParam(pInitParam_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get kernel status

Return the current status of kernel stack.

\param  pStatus_p       Pointer to status variable to return.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_getStatus(UINT16* pStatus_p)
{
    *pStatus_p = ctrlkcal_getStatus();
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat

Return the current heartbeat value in kernel.

\param  pHeartbeat       Pointer to heartbeat variable to return.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_getHeartbeat(UINT16* pHeartbeat)
{
    *pHeartbeat = ctrlk_getHeartbeat();
}

//------------------------------------------------------------------------------
/**
\brief  Write asynchronous frame

This routines extracts the azynchronous frame from the IOCTL buffer and passes it
to DLL for processing.

\param  pArg_p       Pointer to IOCTL buffer.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_sendAsyncFrame(unsigned char* pArg_p)
{
    tIoctlDllCalAsync*    asyncFrameInfo;
    tFrameInfo            frameInfo;

    asyncFrameInfo = (tIoctlDllCalAsync*) pArg_p;
    frameInfo.frameSize = asyncFrameInfo->size;
    frameInfo.pFrame =    (tPlkFrame*)(pArg_p + sizeof(tIoctlDllCalAsync));

    dllkcal_writeAsyncFrame(&frameInfo, asyncFrameInfo->queue);
}

//------------------------------------------------------------------------------
/**
\brief  Write error object

This routines updates the error objects in kernel with the value passed by user.

\param  pWriteObject_p       Pointer to writeobject to update.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_writeErrorObject(tErrHndIoctl* pWriteObject_p)
{
    tErrHndObjects*   errorObjects;
    errorObjects = errhndkcal_getMemPtr();
    *((char*) errorObjects + pWriteObject_p->offset) = pWriteObject_p->errVal;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object

This routines fetches the error objects in kernel to be passed to user.

\param  pWriteObject_p       Pointer to pReadObject_p to fetch.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_readErrorObject(tErrHndIoctl* pReadObject_p)
{
    tErrHndObjects*   errorObjects;

    errorObjects = errhndkcal_getMemPtr();
    pReadObject_p->errVal = *((char*) errorObjects + pReadObject_p->offset);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}

