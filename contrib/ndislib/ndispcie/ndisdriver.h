/**
********************************************************************************
\file   ndisdriver.h

\brief  Internal header file for NDIS driver

This files contains the common types and constant declaration to be used across
the NDIS drivers.
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

#ifndef _INC_ndisdriver_H_
#define _INC_ndisdriver_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <ndis.h>
#include "ndis-intf.h"
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_MEM_TAG           'klpO'
#define OPLK_MAX_FRAME_SIZE    1546


#define OPLK_MTU_SIZE                       1500
#define OPLK_LINK_SPEED                     10000000
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief TODO:

*/
typedef struct
{
    tAppIntfRegister      pfnAppIntfRegCb;
    tAppIntfDeRegister    pfnAppIntfDeregisterCb;
    NDIS_HANDLE           pMiniportHandle;      ///< Miniport driver handle returned by OS
    NDIS_HANDLE           pProtocolHandle;      ///< Protocol driver handle returned by OS
}tNdisDriverInstance;

/**
\brief TODO:

*/
typedef struct
{
    NDIS_STATUS         status;                 // Completion status
    NDIS_EVENT          waitEvent;              // Used to block for completion.
    NDIS_OID_REQUEST    oidRequest;
} tNdisOidRequest;

typedef struct
{
    void*      pData;
    ULONG      maxLength;
    ULONG      length;
    BOOLEAN    free;
} tRxBufInfo;

typedef struct
{
    LIST_ENTRY          txLink;
    PNET_BUFFER_LIST    pNbl;
    PMDL                pMdl;
    BOOLEAN             free;
    ULONG               maxLength;
    ULONG               length;
    void*               pToken;
    void*               pData;
} tTxBufInfo;

typedef struct
{
    NDIS_HANDLE                bindingHandle;
    NDIS_HANDLE                sendNblPool;
    NDIS_EVENT                 adapterEvent;        ///<
    PNDIS_EVENT                pPauseEvent;
    PNDIS_EVENT                pOidCompleteEvent;
    NDIS_SPIN_LOCK             pauseEventLock;
    NDIS_SPIN_LOCK             driverLock;
    NDIS_STATUS                adapterInitStatus;
    NDIS_LINK_STATE            lastLinkState;
    tNdisBindingState          bindingState;
    NDIS_BIND_PARAMETERS       bindParameters;
    ULONG                      oidReq;
    ULONG                      sendRequest;
    void*                      pVEthInstance;
    tNdisReceiveCb             pfnReceiveCb;
    tNdisTransmitCompleteCb    pfnTransmitCompleteCb;
    tRxBufInfo*                pReceiveBufInfo;
    ULONG                      receiveHead;
    ULONG                      receiveBufCount;
    ULONG                      transmitBufCount;
    void*                      pTransmitBuf;
    tTxBufInfo*                pTxBuffInfo;
    void*                      pReceiveBuf;
    LIST_ENTRY                 txList;
    NDIS_SPIN_LOCK             txListLock;
}tProtocolInstance;

/**
\brief TODO:

*/
typedef struct
{
    NDIS_HANDLE                 miniportAdapterHandle;  ///< Adapter handle for NDIS up-calls related
    NDIS_HANDLE                 bindingHandle;          ///<
    NDIS_HANDLE                 interruptHandle;        ///<
    BOOLEAN                     miniportHalting;        ///< Has our Halt entry point been called?
    BOOLEAN                     miniportPaused;
    NDIS_STRING                 cfgDeviceName;          ///< Used as the unique ID for the VELAN
    // Some standard miniport parameters (OID values).
    ULONG                       packetFilter;
    ULONG                       lookAhead;
    ULONG64                     linkSpeed;
    ULONG                       maxBusySends;
    ULONG                       maxBusyRecvs;
    // Packet counts
    ULONG64                     goodTransmits;
    ULONG64                     goodReceives;
    ULONG                       numTxSinceLastAdjust;
    NDIS_LINK_STATE             lastPendingLinkState;
    NDIS_STATUS                 pendingStatusIndication;
    NDIS_STATUS                 lastLinkStatus;
    NDIS_LINK_STATE             lastLinkState;
    UCHAR                       permanentAddress[ETH_LENGTH_OF_ADDRESS];
    UCHAR                       currentAddress[ETH_LENGTH_OF_ADDRESS];
    ULONG                       state;
    NDIS_EVENT                  miniportInitEvent;
    BOOLEAN                     miniportInitPending;
    BOOLEAN                     oidRequestPending;
    NDIS_SPIN_LOCK              miniportLock;
    NDIS_SPIN_LOCK              pauseLock;
    tNdisOidRequest             ndisOidReq;
    NDIS_STATUS                 status;
    tProtocolInstance*          protocolInstance;
    NET_IFINDEX                 ifIndex;
    ULONG                       sendRequests;
    ULONG                       receiveIndication;
    tVEthSendCb                 pfnVEthSendCb;
    tSyncHandler                pfnSyncCb;
    NDIS_INTERRUPT_TYPE         interruptType;
    PIO_INTERRUPT_MESSAGE_INFO  intrMsgInfo;
    PHYSICAL_ADDRESS            phyAddrBar0;
    PHYSICAL_ADDRESS            phyAddrBar1;
    PHYSICAL_ADDRESS            phyAddrBar2;
    PULONG                      virtualAddrBar0;
    PULONG                      virtualAddrBar1;
    PULONG                      virtualAddrBar2;
    ULONG                       lengthBar0;
    ULONG                       lengthBar1;
    ULONG                       lengthBar2;
    ULONG                       msiVector;
}tVEthInstance;
//------------------------------------------------------------------------------
// global defines
//------------------------------------------------------------------------------

extern tNdisDriverInstance    driverInstance_l;
extern tVEthInstance          vethInstance_l;
//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------

#define TXINFO_FROM_NBL(_NBL)    ((tTxBufInfo*)((_NBL)->ProtocolReserved[0]))
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

// Miniport driver prototypes

DRIVER_DISPATCH                            miniportIoDispatch;
DRIVER_DISPATCH                            miniportDeviceIoControl;
MINIPORT_SET_OPTIONS                       miniportSetOptions;
MINIPORT_INITIALIZE                        miniportInitialize;
MINIPORT_HALT                              miniportHalt;
MINIPORT_UNLOAD                            miniportUnload;
MINIPORT_PAUSE                             miniportPause;
MINIPORT_RESTART                           miniportRestart;
MINIPORT_OID_REQUEST                       miniportOidRequest;
MINIPORT_SEND_NET_BUFFER_LISTS             miniportSendNetBufferLists;
MINIPORT_RETURN_NET_BUFFER_LISTS           miniportReturnNetBufferLists;
MINIPORT_CANCEL_SEND                       miniportCancelSendNetBufferLists;
MINIPORT_DEVICE_PNP_EVENT_NOTIFY           miniportPnPEventNotify;
MINIPORT_SHUTDOWN                          miniportShutdown;
MINIPORT_CANCEL_OID_REQUEST                miniportCancelOidRequest;
MINIPORT_CHECK_FOR_HANG                    miniportCheckForHang;
MINIPORT_RESET                             miniportReset;

// Protocol driver protoypes

PROTOCOL_SET_OPTIONS                       protocolSetOptions;
PROTOCOL_OPEN_ADAPTER_COMPLETE_EX          protocolOpenAdapterComplete;
PROTOCOL_CLOSE_ADAPTER_COMPLETE_EX         protocolCloseAdapterComplete;
PROTOCOL_OID_REQUEST_COMPLETE              protocolRequestComplete;
PROTOCOL_STATUS_EX                         protocolStatus;
PROTOCOL_BIND_ADAPTER_EX                   protocolBindAdapter;
PROTOCOL_UNBIND_ADAPTER_EX                 protocolUnbindAdapter;
PROTOCOL_NET_PNP_EVENT                     protocolPnpHandler;
PROTOCOL_RECEIVE_NET_BUFFER_LISTS          protocolReceiveNbl;
PROTOCOL_SEND_NET_BUFFER_LISTS_COMPLETE    protocolSendNblComplete;

// Protocol Global routine prototype
void        protocol_freeVEthInstance(tVEthInstance* pVEthInstance_p);
BOOLEAN     protocol_checkBindingState();
void        protocol_setBindingState(ULONG state_p);
void        protocol_registerTxRxHandler(tNdisTransmitCompleteCb pfnTxCallback_p,
                                         tNdisReceiveCb pfnRxCallback_p);
NDIS_STATUS protocol_allocateTxRxBuf(ULONG txBufCount_p, ULONG rxBufCount_p);
void        protocol_freeTxRxBuffers(void);
tTxBufInfo* protocol_getTxBuff(size_t size_p);
void        protocol_freeTxBuff(PVOID pTxLink_p);
NDIS_STATUS protocol_sendOidRequest(NDIS_REQUEST_TYPE requestType_p, NDIS_OID oid_p,
                                    PVOID oidReqBuffer_p, ULONG oidReqBufferLength_p);
NDIS_STATUS protocol_sendPacket(void* pToken_p, size_t size_p, void* pTxLink_p);
UCHAR*      protocol_getCurrentMac(void);

#ifndef NDEBUG
#define TRACE(...)      DbgPrint(__VA_ARGS__)
#else
#define TRACE(...)
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_ndisdriver_H_ */
