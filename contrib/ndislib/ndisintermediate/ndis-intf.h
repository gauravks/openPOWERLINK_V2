/**
********************************************************************************
\file   ndis-intf.h

\brief  NDIS driver interface header

This file contains the set for routines exported from the NDIS driver.
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

#ifndef _INC_ndis_intf_H_
#define _INC_ndis_intf_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief TODO:

*/
typedef enum eNdisBindingState
{
    NdisBindingPaused,              ///< Lower end binding is in paused state
    NdisBindingPausing,             ///< Lower end binding is entering into paused state
    NdisBindingReady,               ///< Lower end binding is running
    NdisBindingRunning
}tNdisBindingState;

/**
\brief TODO:

*/
typedef enum
{
    NdisStatusSuccess,              ///< Lower end binding is in paused state
    NdisStatusInit,
    NdisStatusResources,            ///< Lower end binding is entering into paused state
    NdisStatusTxError,              ///< Lower end binding is running
    NdisStatusRxError,
    NdisStatusInvalidParams
}tNdisErrorStatus;

typedef void (*tAppIntfRegister)(NDIS_HANDLE driverHandle_p);
typedef void (*tAppIntfDeRegister)(void);
/**
\brief TODO:

*/
typedef void (*tNdisReceiveCb)(void* pRxData_p, size_t size_p);

/**
\brief TODO:

*/
typedef void (*tVEthSendCb)(void* pVEthTxData_p, size_t size_p);

/**
\brief TODO:

*/
typedef void (*tNdisTransmitCompleteCb)(void* pTxBuffer_p);
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tNdisErrorStatus ndis_initDriver(PDRIVER_OBJECT pDriverObject_p, PUNICODE_STRING pRegistryPath_p);
void             ndis_registerAppIntf(tAppIntfRegister pAppIntfRegCb_p, tAppIntfDeRegister pAppIntfDeregCb_p);
BOOLEAN          ndis_checkBindingState(void);
void             ndis_setBindingState(ULONG state_p);
tNdisErrorStatus ndis_allocateTxRxBuff(UINT txBuffCount_p, UINT rxBuffCount_p);
void             ndis_freeTxRxBuff(void);
tNdisErrorStatus ndis_getTxBuff(void** ppData_p, size_t size_p, void** ppTxLink_p);
void             ndis_freeTxBuff(void* pTxLink_p);
tNdisErrorStatus ndis_sendPacket(void* pData_p, size_t size_p, void* pTxLink_p);
void             ndis_registerTxRxHandler(tNdisTransmitCompleteCb pfnTxCallback_p, tNdisReceiveCb pfnRxCallback_p);
void             ndis_createAppIntf(void);
void             ndis_closeAppIntf(void);
NDIS_HANDLE      ndis_getAdapterHandle(void);
void             ndis_getMacAddress(UCHAR*  pMac_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_ndis-intf_H_ */
