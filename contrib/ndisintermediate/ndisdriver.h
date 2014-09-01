/**
********************************************************************************
\file   /windows-split/contrib/ndisintermediate/ndisdriver.h

\brief  {BRIEF_DESCRIPTION_OF_THE_FILE}

{DETAILED_DESCRIPTION_OF_THE_FILE}
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, {DEVELOPER_NAME}
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
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief TODO:

*/
typedef void(*tNdisReceiveCb)(void* pRxData_p, size_t size_p);

/**
\brief TODO:

*/
typedef void(*tNdisTransmitCompleteCb)(void* pTxBuffer_p);

/**
\brief TODO:

*/
typedef enum eNdisBindingState
{
    NdisBindingPaused,          ///< Lower end binding is in paused state
    NdisBindingPausing,         ///< Lower end binding is entering into paused state
    NdisBindingRunning          ///< Lower end binding is running
}tNdisBindingState;

/**
\brief TODO:

*/
typedef struct
{
    NDIS_HANDLE     pMiniportHandle;        ///< Miniport driver handle returned by OS
    NDIS_HANDLE     pProtocolHandle;        ///< Protocol driver handle returned by OS
}tNdisDriverInstance;

/**
\brief TODO:

*/
typedef struct
{
    NDIS_STATUS                 status;     // Completion status
    NDIS_EVENT                  waitEvent;      // Used to block for completion.
    NDIS_OID_REQUEST            oidRequest;
} tNdisOidRequest;

/**
\brief TODO:

*/
typedef struct
{
    NDIS_HANDLE     miniportAdapterHandle; ///< Adapter handle for NDIS up-calls related
    NDIS_HANDLE     bindingHandle;         ///< 
                                           ///< to this virtual miniport.
    BOOLEAN         miniportHalting;       ///< Has our Halt entry point been called?
    NDIS_STRING     cfgDeviceName;         ///< Used as the unique ID for the VELAN
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
    tNdisOidRequest             ndisOidReq;
}tVEthInstance;
//------------------------------------------------------------------------------
// global defines
//------------------------------------------------------------------------------

extern tNdisDriverInstance driverInstance_l;
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

// Miniport driver prototypes

DRIVER_DISPATCH miniportIoDispatch;

DRIVER_DISPATCH miniportDeviceIoControl;

MINIPORT_SET_OPTIONS miniportSetOptions;

MINIPORT_INITIALIZE miniportInitialize;

MINIPORT_HALT miniportHalt;

MINIPORT_UNLOAD miniportUnload;

MINIPORT_PAUSE miniportPause;

MINIPORT_RESTART miniportRestart;

MINIPORT_CHECK_FOR_HANG miniportCheckForHang;

MINIPORT_OID_REQUEST miniportOidRequest;

MINIPORT_SEND_NET_BUFFER_LISTS miniportSendNetBufferLists;

MINIPORT_RETURN_NET_BUFFER_LISTS miniportReturnNetBufferLists;

MINIPORT_CANCEL_SEND miniportCancelSendNetBufferLists;

MINIPORT_DEVICE_PNP_EVENT_NOTIFY miniportPnPEventNotify;

MINIPORT_SHUTDOWN miniportShutdown;

MINIPORT_CANCEL_OID_REQUEST miniportCancelOidRequest;

MINIPORT_RESET miniportReset;


// Protocol driver protoypes

PROTOCOL_SET_OPTIONS protocolSetOptions;

PROTOCOL_OPEN_ADAPTER_COMPLETE_EX protocolOpenAdapterComplete;

PROTOCOL_CLOSE_ADAPTER_COMPLETE_EX protocolCloseAdapterComplete;

PROTOCOL_OID_REQUEST_COMPLETE protocolRequestComplete;

PROTOCOL_STATUS_EX protocolStatus;

PROTOCOL_BIND_ADAPTER_EX protocolBindAdapter;

PROTOCOL_UNBIND_ADAPTER_EX protocolUnbindAdapter;

PROTOCOL_NET_PNP_EVENT protocolPnpHandler;

PROTOCOL_RECEIVE_NET_BUFFER_LISTS protocolReceiveNbl;

PROTOCOL_SEND_NET_BUFFER_LISTS_COMPLETE protocolSendNblComplete;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_ndisdriver_H_ */
