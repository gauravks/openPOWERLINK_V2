/**
********************************************************************************
\file   drv_ndis_intemediate/drvintf.h

\brief  Driver interface header file

// TODO: Add description here
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

#ifndef _INC_drvintf_H_
#define _INC_drvintf_H_

#include <common/driver.h>
#include <common/ctrl.h>
#include <common/target.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>
#include <common/ctrlcal-mem.h>
//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef struct
{
    //
    // Lock to rundown threads that are dispatching I/Os on a file handle
    // while the cleanup for that handle is in progress.
    //
    IO_REMOVE_LOCK    driverAccessLock;
} tFileContext;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
void  drv_executeCmd(tCtrlCmd* ctrlCmd_p);
void  drv_readInitParam(tCtrlInitParam* pInitParam_p);
void  drv_storeInitParam(tCtrlInitParam* pInitParam_p);
void  drv_getStatus(UINT16* status_p);
void  drv_getHeartbeat(UINT16* heartbeat);
void  drv_sendAsyncFrame(unsigned char* pArg_p);
void  drv_writeErrorObject(tErrHndIoctl* pWriteObject_p);
void  drv_readErrorObject(tErrHndIoctl* pReadObject_p);
tOplkError drv_initDualProcDrv(void);
void drv_exitDualProcDrv(void);
void drv_postEvent(void* pEvent_p);
void drv_getEvent(void* pEvent_p, size_t* pSize_p);
tOplkError drv_getPdoMem(UINT8** ppPdoMem_p, size_t memSize_p);
void drv_freePdoMem(UINT8* pPdoMem_p, size_t memSize_p);
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_drvintf_H_ */
