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

#define OPLK_MTU_SIZE          1500
#define OPLK_LINK_SPEED        10000000
#define OPLK_MAX_BAR_COUNT     6
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief NDIS driver instance structure

The structure holds the parameters for the NDIS driver instance.

*/
typedef struct
{
    tDrvIntfRegister      pfnDrvIntfRegCb;          ///< Driver interface register callback.
    tDrvIntfDeRegister    pfnDrvIntfDeregisterCb;   ///< Driver interface de-register callback.
    NDIS_HANDLE           pMiniportHandle;          ///< Miniport driver handle returned by OS.
    NDIS_HANDLE           pProtocolHandle;          ///< Protocol driver handle returned by OS.
} tNdisDriverInstance;

/**
\brief NDIS OID request structure

The structure holds the parameters for an OID request.

*/
typedef struct
{
    NDIS_STATUS         status;                 ///< Completion status of the OID.
    NDIS_EVENT          waitEvent;              ///< Event handle to block completion.
    NDIS_OID_REQUEST    oidRequest;             ///< NDIS OID request for the request.
} tNdisOidRequest;

/**
\brief Receive buffer information

Receive buffer information structure for NDIS miniport.

*/
typedef struct
{
    void*      pData;           ///< Pointer to Receive buffer.
    ULONG      maxLength;       ///< Max length of the buffer.
    ULONG      length;          ///< Length of the buffer in receive frame.
    BOOLEAN    free;            ///< Flag to identify buffer access status.
} tRxBufInfo;

/**
\brief Transmit buffer information

Transmit buffer information structure for NDIS miniport.

*/
typedef struct
{
    LIST_ENTRY          txLink;         ///< List entry for the transmit buffer info.
    PNET_BUFFER_LIST    pNbl;           ///< Pointer to NET_BUFFER_LIST, the transmit buffer is mapped to.
    PMDL                pMdl;           ///< MDL describing the transmit buffer.
    BOOLEAN             free;           ///< Flag to identify buffer access status.
    ULONG               maxLength;      ///< Max length of the buffer.
    ULONG               length;         ///< Length of the buffer in transmit frame.
    void*               pToken;         ///< Identification token for the buffer.
    void*               pData;          ///< Pointer to transmit buffer.
} tTxBufInfo;

/**
\brief NDIS protocol instance structure

The structure holds all the parameters required by the NDIS protocol instance.

*/
typedef struct
{
    NDIS_HANDLE                bindingHandle;           ///< NDIS handle for lower binding to the protocol.
    NDIS_HANDLE                sendNblPool;             ///< Send NET_BUFFER_LIST pool handle.
    NDIS_EVENT                 adapterEvent;            ///< Adapter initialization event.
    PNDIS_EVENT                pPauseEvent;             ///< Adapter pause event.
    PNDIS_EVENT                pOidCompleteEvent;       ///< OID complete synchronization event.
    NDIS_SPIN_LOCK             pauseEventLock;          ///< NDIS lock for pause event.
    NDIS_SPIN_LOCK             driverLock;              ///< NDIS lock for adapter initialization and exit event.
    NDIS_STATUS                adapterInitStatus;       ///< Adapter initialization status.
    NDIS_LINK_STATE            lastLinkState;           ///< Last updated link state information.
    tNdisBindingState          bindingState;            ///< Binding state of the protocol instance.
    NDIS_BIND_PARAMETERS       bindParameters;          ///< NDIS bind parameters for the protocol to miniport binding.
    ULONG                      oidReq;                  ///< OID request count.
    ULONG                      sendRequest;             ///< Send request count.
    void*                      pVEthInstance;           ///< Pointer to virtual Ethernet context structure.
    tNdisReceiveCb             pfnReceiveCb;            ///< Function pointer for receive callback.
    tNdisTransmitCompleteCb    pfnTransmitCompleteCb;   ///< Function pointer for transmit complete callback.
    tRxBufInfo*                pReceiveBufInfo;         ///< Pointer to receive buffer information list.
    ULONG                      receiveHead;             ///< Current receive head.
    ULONG                      receiveBufCount;         ///< Total receive count for statistical information.
    ULONG                      transmitBufCount;        ///< Total transmit count for statistical information.
    void*                      pTransmitBuf;            ///< Pointer to transmit buffer.
    tTxBufInfo*                pTxBuffInfo;             ///< Pointer to list of transmit buffer information.
    void*                      pReceiveBuf;             ///< Pointer to receive buffer.
    LIST_ENTRY                 txList;                  ///< Transmit queue.
    NDIS_SPIN_LOCK             txListLock;              ///< Transmit queue access lock.
} tProtocolInstance;

/**
\brief PCIe BAR information

The structure holds the information of the PCIe BAR mapped by the NDIS miniport.

*/
typedef struct
{
    PHYSICAL_ADDRESS              phyAddr;          ///< Physical address of the BAR.
    PULONG                        virtualAddr;      ///< Virtual address of the BAR in kernel memory.
    ULONG                         length;           ///< Length of the BAR.
} tBarInfo;

/**
\brief TODO:

*/
typedef struct
{
    NDIS_HANDLE                   miniportAdapterHandle;    ///< Adapter handle for the NDIS miniport.
    NDIS_HANDLE                   bindingHandle;            ///< NDIS handle to the upper binding to NDIS miniport.
    NDIS_HANDLE                   interruptHandle;          ///< NDIS handle for interrupts.
    BOOLEAN                       miniportHalting;          ///< Flag to identify miniport halting status.
    BOOLEAN                       miniportPaused;           ///< Flag to identify paused status.
    NDIS_STRING                   cfgDeviceName;            ///< Used as the unique ID for the VETH.
    // Some standard miniport parameters (OID values).
    ULONG                         packetFilter;             ///< Current packet filter for the miniport.
    ULONG                         lookAhead;                ///< Lookahead flags for the miniport.
    ULONG64                       linkSpeed;                ///< Link speed of the miniport to broadcast.
    ULONG                         maxBusySends;             ///< Max busy send count.
    ULONG                         maxBusyRecvs;             ///< Max busy receive count.
    // Packet counts (Used for handling OIDs)
    ULONG64                       goodTransmits;            ///< Total count of good transmits.
    ULONG64                       goodReceives;             ///< Total count of good receives.
    NDIS_LINK_STATE               lastPendingLinkState;     ///< Last pending link state request.
    NDIS_STATUS                   pendingStatusIndication;  ///< Last pending status indication request.
    NDIS_STATUS                   lastLinkStatus;           ///< Last link status received.
    NDIS_LINK_STATE               lastLinkState;            ///< Last link state change.
    UCHAR                         permanentAddress[ETH_LENGTH_OF_ADDRESS];  ///< Permanent MAC address of the NIC.
    UCHAR                         currentAddress[ETH_LENGTH_OF_ADDRESS];    ///< Current MAC used by miniport.
    ULONG                         state;                    ///< Current miniport state.
    NDIS_EVENT                    miniportInitEvent;        ///< Miniport initialization complete event.
    BOOLEAN                       miniportInitPending;      ///< Flag to identify miniport initialization completion.
    BOOLEAN                       oidRequestPending;        ///< Flag for pending OID request handling.
    NDIS_SPIN_LOCK                miniportLock;             ///< Global miniport lock.
    NDIS_SPIN_LOCK                pauseLock;                ///< Lock for miniport pause event.
    tNdisOidRequest               ndisOidReq;               ///< NDIS OID request information.
    NDIS_STATUS                   status;                   ///< Miniport status.
    tProtocolInstance*            protocolInstance;         ///< Pointer to protocol instance in a NDIS intermediate driver.
    NET_IFINDEX                   ifIndex;                  ///< Interface index in the Windows network stack.
    ULONG                         sendRequests;             ///< Total send requests handled.
    ULONG                         receiveIndication;        ///< Total receive indications handled.
    tVEthSendCb                   pfnVEthSendCb;            ///< Function pointer to VEth send complete callback.
    tSyncHandler                  pfnSyncCb;                ///< Function pointer to synchronization callback.
    NDIS_INTERRUPT_TYPE           interruptType;            ///< NDIS interrupt type for the registered interrupt.
    PIO_INTERRUPT_MESSAGE_INFO    intrMsgInfo;              ///< Message interrupt information.
    tBarInfo                      barInfo[OPLK_MAX_BAR_COUNT];  ///< PCIe BAR information for all available BARs.
    ULONG                         msiVector;                    ///< MSI vector assigned for the miniport.
} tVEthInstance;

//------------------------------------------------------------------------------
// global defines
//------------------------------------------------------------------------------
extern tNdisDriverInstance                 driverInstance_g;
extern tVEthInstance                       vethInstance_g;

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

// Protocol driver prototypes

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


#ifndef NDEBUG
#define TRACE(...)    DbgPrint(__VA_ARGS__)
#else
#define TRACE(...)
#endif

#ifdef __cplusplus
extern "C" {
#endif
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

#ifdef __cplusplus
}
#endif

#endif /* _INC_ndisdriver_H_ */
