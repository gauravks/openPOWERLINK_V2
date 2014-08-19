/**
********************************************************************************
\file   edrv-ndisintermediate.c

\brief  Implementation of edrv module using NDIS intermediate driver interface

This file contains the implementation of Ethernet driver in windows kernel which
uses NDIS interface to communicate to native miniport driver. This driver will
use the protocol access points of NDIS intermediate driver to communicate with
lower drivers and exchange packets.

//TODO:gks complete description for the driver

\ingroup module_edrv
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
#include <oplk/ami.h>
#include <kernel/edrv.h>


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS     42
#endif

#ifndef EDRV_MAX_TX_DESCS
#define EDRV_MAX_TX_DESCS           16
#define EDRV_TX_DESC_MASK           (EDRV_MAX_TX_DESCS - 1)
#endif

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS         256
#endif

#ifndef EDRV_MAX_RX_DESCS
#define EDRV_MAX_RX_DESCS           16
#define EDRV_RX_DESC_MASK           (EDRV_MAX_RX_DESCS - 1)
#endif

#define EDRV_MAX_FRAME_SIZE         0x600

#define EDRV_TX_BUFFER_SIZE         (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)

#define EDRV_NDIS_MEM_TAG           'klpO'
#define EDRV_ALLOCATED_NBL          0x10000000

#define EDRV_MALLOC(_pVar, _Size)   NdisAllocateMemoryWithTag((PVOID *)(&_pVar),\
                                    (_Size), EDRV_NDIS_MEM_TAG)

#define EDRV_FREE(_pMem)            NdisFreeMemory(_pMem, 0, 0)
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
typedef enum eNdisBindingState
{
    NdisBindingPaused,          ///< Lower end binding is in paused state
    NdisBindingPausing,         ///< Lower end binding is entering into paused state
    NdisBindingRunning          ///< Lower end binding is running
}tNdisBindingState;

typedef struct
{
    PMDL                        pMdl;
    // TODO@gks: do we need a duplicate pointer?
    void*                       pBuffer;
    PNET_BUFFER_LIST            pNetBufList;
    tEdrvTxBuffer*              pEdrvBuffer;
}tNdisTxBuffInfo;

typedef struct
{
    NDIS_HANDLE                     pProtocolHandle;       ///< From NdisMRegisterProtocolDriver
    NDIS_HANDLE                     pDeviceHandle;       ///< From NdisMRegisterDeviceEx
    NDIS_HANDLE                     pBindingHandle;      ///< Lower end miniport binding handle
    NDIS_STATUS                     adapterInitStatus;
    NDIS_EVENT                      adapterEvent;
    void*                           pIoAddr;             ///< Pointer to register space of Ethernet controller
    UINT                            ioLength;

    tEdrvRxDesc*                    pRxDesc;             ///< Pointer to Rx descriptors
    tRxUsedBuff                     aRxBuffUsed[EDRV_MAX_RX_DESCS];
    NDIS_PHYSICAL_ADDRESS           pRxDescDma;          ///< Dma pointer to Rx descriptors
    void*                           pRxBuf;              ///< Pointer to Rx buffer
    NDIS_PHYSICAL_ADDRESS           pRxBufDma;           ///< Pointer to Rx buffer
    INT                             rxBufFreeTop;
    PNDIS_SPIN_LOCK*                pSpinLockRxBufRelease;
    INT                             pageAllocations;

    void*                           pTxBuf;             ///< Pointer to Tx buffer
    NDIS_PHYSICAL_ADDRESS           pTxBufDma;          ///< Physical address of Tx buffers
    tEdrvTxDesc*                    pTxDesc;            ///< Pointer to Tx descriptors
    tEdrvTxBuffer*                  apTxBuffer[EDRV_MAX_TX_BUFFERS];
    NDIS_PHYSICAL_ADDRESS           pTxDescDma;
    BOOLEAN                         afTxBufUsed[EDRV_MAX_TX_BUFFERS];

    UINT                            headTxDesc;
    UINT                            tailTxDesc;
    UINT                            headRxDesc;
    UINT                            tailRxDesc;
    tNdisTxBuffInfo                 pTxBuffInfo[EDRV_MAX_TX_BUFFERS];


    // NDIS Intermediate driver handling parameters
    NDIS_BIND_PARAMETERS            bindParameters;
    NDIS_PNP_CAPABILITIES           powerManagementCap;
    NDIS_RECEIVE_SCALE_CAPABILITIES rcvScaleCap;
    NDIS_LINK_STATE                 lastIndicatedLinkState;
    // Power state of the underlying adapter
    tNdisBindingState               bindingState;
    // TODO:gks To be created as a part of VEth module
    tVEthInstance*                  pVEthInstance;
    NDIS_SPIN_LOCK                  pauseEventLock;
    PNDIS_EVENT                     pauseEvent;
    ULONG                           sendRequestCount;
    NDIS_HANDLE                     sendNblPool;
    tEdrvInitParam                  initParam;

#if CONFIG_EDRV_USE_DIAGNOSTICS != FALSE
    ULONGLONG           interruptCount;
    INT                 rxBufFreeMin;
    UINT                rxCount[EDRV_SAMPLE_NUM];
    UINT                txCount[EDRV_SAMPLE_NUM];
    UINT                pos;
#endif
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

tEdrvInstance   edrvInstance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void edrvTxHandler(ULONG buffId_p);
static void edrvRxHandler(void* pRxBuffer_p, size_t size_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver initialization

This function initializes the Ethernet driver.

\param  pEdrvInitParam_p    Edrv initialization parameters

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_init(tEdrvInitParam* pEdrvInitParam_p)
{
    tOplkError          ret = kErrorOk;

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    // save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    // Check if NDIS intermediate driver is ready
    if (ndis_checkBindingState())
    {
        // The ndis driver has not initialized
        DbgPrint("%s() NDIS Driver not initialized\n",__func__);
        return kErrorEdrvInit;
    }

    // Allocate and prepare Transmit and receive Net Buffer Lists
    ret = ndis_allocateTxRxNbl(EDRV_MAX_TX_BUFFERS, EDRV_MAX_RX_BUFFERS);
    if(ret != kErrorOk)
    {
        DbgPrint("%s() TX and RX buffer allocation failed 0x%X\n",__func__, ret);
        return ret;
    }

    // Register Tx and Rx callbacks
    ndis_registerTxRxHandler(edrvTxHandler, edrvRxHandler);

    // TODO@gks: Get the MAC address here and assign to init params
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver shutdown

This function shuts down the Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_shutdown(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\param  pMacAddr_p  Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(UINT8* pMacAddr_p)
{
    // TODO@gks check if this is possible?
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clear multicast address entry

This function removes the multicast entry from the Ethernet controller.

\param  pMacAddr_p  Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(UINT8* pMacAddr_p)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Change Rx filter setup

This function changes the Rx filter setup. The parameter entryChanged_p
selects the Rx filter entry that shall be changed and \p changeFlags_p determines
the property.
If \p entryChanged_p is equal or larger count_p all Rx filters shall be changed.

\note Rx filters are not supported by this driver!

\param  pFilter_p           Base pointer of Rx filter array
\param  count_p             Number of Rx filter array entries
\param  entryChanged_p      Index of Rx filter entry that shall be changed
\param  changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_changeRxFilter(tEdrvFilter* pFilter_p, UINT count_p,
                               UINT entryChanged_p, UINT changeFlags_p)
{
    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(count_p);
    UNUSED_PARAMETER(entryChanged_p);
    UNUSED_PARAMETER(changeFlags_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This function releases the Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function sends the Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set Tx buffer ready

This function sets the Tx buffer buffer ready for transmission.

\param  pBuffer_p   Tx buffer buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setTxBufferReady(tEdrvTxBuffer* pBuffer_p)
{
    UNUSED_PARAMETER(pBuffer_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Start ready Tx buffer

This function sends the Tx buffer marked as ready.

\param  pBuffer_p   Tx buffer buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_startTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    UNUSED_PARAMETER(pBuffer_p);

    return kErrorOk;
}

#if CONFIG_EDRV_USE_DIAGNOSTICS != FALSE
//------------------------------------------------------------------------------
/**
\brief  Get Edrv module diagnostics

This function returns the Edrv diagnostics to a provided buffer.

\param  pBuffer_p   Pointer to buffer filled with diagnostics.
\param  size_p      Size of buffer

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
INT edrv_getDiagnostics(char* pBuffer_p, INT size_p)
{

}
#endif
//------------------------------------------------------------------------------
/**
\brief  Release Rx buffer

This function releases a late release Rx buffer.

\param  pRxBuffer_p     Rx buffer to be released

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_releaseRxBuffer(tEdrvRxBuffer* pRxBuffer_p)
{
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

static void edrvRxHandler(void* pRxBuffer_p, size_t size_p)
{

}

static void edrvTxHandler(ULONG buffId_p)
{

}

//------------------------------------------------------------------------------
/**
\brief  Protocol entry point for bind adapter

Called by NDIS to bind to a miniport below. This routine creates a binding by
calling NdisOpenAdapterEx, and then initiates creation of all configured
VELANs on this binding.

\param  protocolDriverContext_p     A pointer to the driver context
\param  bindContext_p               A pointer to the bind context
\param  bindParameters_p            Pointer to related information about
                                    this new binding.

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS ndisBindAdapter(NDIS_HANDLE protocolDriverContext_p,
                            NDIS_HANDLE bindContext_p,
                            PNDIS_BIND_PARAMETERS bindParameters_p)
{
    NDIS_STATUS                       status = NDIS_STATUS_SUCCESS;
    NDIS_OPEN_PARAMETERS              openParameters;
    NET_BUFFER_LIST_POOL_PARAMETERS   poolParameters;

    // TODO@gks: Get the protocol handle register during driver initialization
    edrvInstance_l.pProtocolHandle = getProtocolHandle();

    if(edrvInstance_l.pProtocolHandle == NULL)
    {
        // Driver not registered yet. Is this even possible?
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    NdisInitializeEvent(&edrvInstance_l.adapterEvent);
    NdisAllocateSpinLock(&edrvInstance_l.pauseEventLock);
    //NdisInitializeListHead(&pAdapt->VElanList);
    edrvInstance_l.bindingState = NdisBindingPaused;

    edrvInstance_l.lastIndicatedLinkState.Header.Revision = NDIS_LINK_STATE_REVISION_1;
    edrvInstance_l.lastIndicatedLinkState.Type = NDIS_OBJECT_TYPE_DEFAULT;
    edrvInstance_l.lastIndicatedLinkState.Size = sizeof(NDIS_LINK_STATE);
    edrvInstance_l.lastIndicatedLinkState.MediaConnectState = BindParameters->MediaConnectState;
    edrvInstance_l.lastIndicatedLinkState.MediaDuplexState = BindParameters->MediaDuplexState;
    edrvInstance_l.lastIndicatedLinkState.XmitLinkSpeed = BindParameters->XmitLinkSpeed;
    edrvInstance_l.lastIndicatedLinkState.RcvLinkSpeed = BindParameters->RcvLinkSpeed;

    //
    // Now open the adapter below and complete the initialization
    //
    NdisZeroMemory(&openParameters, sizeof(NDIS_OPEN_PARAMETERS));

    openParameters.Header.Type = NDIS_OBJECT_TYPE_OPEN_PARAMETERS;
    openParameters.Header.Revision = NDIS_OPEN_PARAMETERS_REVISION_1;
    openParameters.Header.Size = sizeof(NDIS_OPEN_PARAMETERS);
    openParameters.AdapterName = BindParameters->AdapterName;
    openParameters.MediumArray = MediumArray;
    openParameters.MediumArraySize = sizeof(MediumArray) / sizeof(NDIS_MEDIUM);
    openParameters.SelectedMediumIndex = &MediumIndex;

    openParameters.FrameTypeArray = NULL;
    openParameters.FrameTypeArraySize = 0;

    NDIS_DECLARE_PROTOCOL_OPEN_CONTEXT(tEdrvInstance);
    status = NdisOpenAdapterEx(edrvInstance_l.pProtocolHandle,
                               &edrvInstance_l,
                               &openParameters,
                               bindContext_p,
                               &edrvInstance_l.pBindingHandle);

    if (status == NDIS_STATUS_PENDING)
    {
          // Wait for initialization of adapter to complete
          NdisWaitEvent(&edrvInstance_l.adapterEvent, 0);
          status = edrvInstance_l.adapterInitStatus;
    }

    if (status != NDIS_STATUS_SUCCESS)
    {
          edrvInstance_l.pBindingHandle = NULL;
          goto ExitFail;
    }

    edrvInstance_l.bindParameters = *bindParameters_p;

    if (bindParameters_p->RcvScaleCapabilities)
    {
        edrvInstance_l.bindParameters.RcvScaleCapabilities = (*bindParameters_p->RcvScaleCapabilities);
    }

    // Zeroing out fields that are not needed by the MUX driver
    edrvInstance_l.bindParameters.ProtocolSection= NULL;
    edrvInstance_l.bindParameters.AdapterName = NULL;
    edrvInstance_l.bindParameters.PhysicalDeviceObject = NULL;

    NdisZeroMemory(&poolParameters, sizeof(NET_BUFFER_LIST_POOL_PARAMETERS));

    poolParameters.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    poolParameters.Header.Revision = NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
    poolParameters.Header.Size = sizeof(poolParameters);
    poolParameters.ProtocolId = NDIS_PROTOCOL_ID_IPX;       // Sequenced Packet exchanger
    poolParameters.ContextSize = 0;
    poolParameters.fAllocateNetBuffer = TRUE;
    poolParameters.PoolTag = MUX_TAG;           //TODO@gks: Change this with a valid tag

    edrvInstance_l.sendNblPool = NdisAllocateNetBufferListPool(edrvInstance_l.pProtocolHandle,
                                                               &poolParameters);
    if (edrvInstance_l.sendNblPool == NULL)
    {
        DbgPrint("CreateBinding: failed to alloc send net buffer list pool\n");

        status = NDIS_STATUS_RESOURCES;
        break;
    }

    //
    // Start all VEth for this adapterr.
    //
    status = bootStrapVEth();

    if (status != NDIS_STATUS_SUCCESS)
    {
        goto ExitFail;
    }

ExitFail:
    if (status != NDIS_STATUS_SUCCESS)
    {
        if (edrvInstance_l.pBindingHandle != NULL)
        {
            closeAdapter();
        }
    }

Exit:
    return status;
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  status_p

*/
//------------------------------------------------------------------------------
VOID ndisOpenAdapterComplete(NDIS_HANDLE protocolBindingContext_p, NDIS_STATUS status_p)
{
    tEdrvInstance* edrvInstance = (tEdrvInstance*)protocolBindingContext_p;

    edrvInstance->adapterInitStatus = status_p;
    NdisSetEvent(&edrvInstance->adapterEvent);

}

//------------------------------------------------------------------------------
/**
\brief

\param  unbindContext_p
\param  protocolBindingContext_p

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS ndisUnbindAdapter(NDIS_HANDLE unbindContext_p,
                             NDIS_HANDLE protocolBindingContext_p)
{
    NDIS_STATUS     Status = NDIS_STATUS_SUCCESS;

    if(edrvInstance_l.pVEthInstance != NULL)
    {
        stopVEth();
    }

    if (edrvInstance_l.pVEthInstance->status != statusClosed)
    {
        // Wait for VEth module to close
        NdisMSleep(2000);
    }

    //
    // Close the binding to the lower adapter.
    //
    if (edrvInstance_l.pBindingHandle != NULL)
    {
        closeAdapter();
    }
    else
    {
        //
        // Binding Handle should not be NULL.
        //
        Status = NDIS_STATUS_FAILURE;
        ASSERT(0);
    }

}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p

*/
//------------------------------------------------------------------------------
VOID ndisCloseAdapterComplete(NDIS_HANDLE protocolBindingContext_p)
{
    NdisSetEvent(&edrvInstance_l->adapterEvent);
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  ndisRequest_p
\param  status_p

*/
//------------------------------------------------------------------------------
VOID ndisRequestComplete(NDIS_HANDLE protocolBindingContext_p,
                         PNDIS_OID_REQUEST ndisRequest_p,
                         NDIS_STATUS status_p)
{
    // Nothing to do now
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  statusIndication_p

*/
//------------------------------------------------------------------------------
VOID ndisStatus(NDIS_HANDLE protocolBindingContext_p, PNDIS_STATUS_INDICATION statusIndication_p)
{
    NDIS_STATUS GeneralStatus = statusIndication_p->StatusCode;
    NDIS_STATUS_INDICATION     NewStatusIndication;

    // TODO@gks: DO we need lock here to avoid re-entrance?
    if (GeneralStatus == NDIS_STATUS_LINK_STATE)
    {
        edrvInstance_l.lastIndicatedLinkState = *((PNDIS_LINK_STATE)(statusIndication_p->StatusBuffer));
    }

}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  pNetPnPEventNotification_p

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS ndisPnpHandler(NDIS_HANDLE protocolBindingContext_p,
                           PNET_PNP_EVENT_NOTIFICATION pNetPnPEventNotification_p)
{
    NDIS_STATUS         status  = NDIS_STATUS_SUCCESS;
    PLIST_ENTRY         p;
    NDIS_EVENT          pauseEvent;

    switch (pNetPnPEventNotification_p->NetPnPEvent.NetEvent)
    {
        case NetEventSetPower:

            // TODO@gks: We don't handle power state configuration now.
            status = NDIS_STATUS_SUCCESS
            break;

        case NetEventReconfigure:

            status = NDIS_STATUS_SUCCESS;
            break;
        case NetEventIMReEnableDevice:

                bootStrapVEth(pNetPnPEventNotification_p->NetPnPEvent.Buffer);

            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventPause:

            NdisAcquireSpinLock(&edrvInstance_l.pauseEventLock);

            ASSERT(edrvInstance_l.pauseEvent == NULL);
            // Wait for all the send requests to complete
            // TODO@gks : Shall we wait indefinitely?
            if (edrvInstance_l.sendRequestCount != 0)
            {
                NdisInitializeEvent(&pauseEvent);

                edrvInstance_l.pauseEvent = &pauseEvent;

                NdisReleaseSpinLock(&edrvInstance_l.pauseEventLock);

                NdisWaitEvent(&pauseEvent, 0);
            }
            edrvInstance_l.bindingState = NdisBindingPaused;
            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventRestart:
            edrvInstance_l.bindingState = NdisBindingRunning;
            status = NDIS_STATUS_SUCCESS;
            break;


        default:
            status = NDIS_STATUS_SUCCESS;

            break;
    }

    return status;
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  netBufferLists_p
\param  portNumber_p
\param  numberOfNbl_p
\param  receiveFlags_p

*/
//------------------------------------------------------------------------------
VOID ndisReceiveNbl(NDIS_HANDLE protocolBindingContext_p, PNET_BUFFER_LIST netBufferLists_p,
                    NDIS_PORT_NUMBER  portNumber_p,ULONG numberOfNbl_p,
                    ULONG receiveFlags_p)
{
    PVELAN                  pVElan = NULL;
    PNET_BUFFER_LIST        currentNbl = NULL;
    PNET_BUFFER_LIST        returnNbl = NULL;
    PNET_BUFFER_LIST        lastReturnNbl = NULL;
    LOCK_STATE              lockState;
    ULONG                   returnFlags;
    ULONG                   NewReceiveFlags;
    BOOLEAN                 bReturnNbl;
    PMDL                    pMdl;
    ULONG                   offset = 0;         // CurrentMdlOffset
    ULONG                   totalLength;

    UNREFERENCED_PARAMETER(numberOfNbl_p);

    returnFlags = 0;

    if (NDIS_TEST_RECEIVE_AT_DISPATCH_LEVEL(receiveFlags_p))
    {
        NDIS_SET_RETURN_FLAG(returnFlags, NDIS_RETURN_FLAGS_DISPATCH_LEVEL);
    }

    ASSERT(netBufferLists_p != NULL);

    if(edrvInstance_l.bindingState != NdisBindingRunning)
    {
        if (NDIS_TEST_RECEIVE_CAN_PEND(receiveFlags_p) == TRUE)
        {
            NdisReturnNetBufferLists(edrvInstance_l.pBindingHandle,
                                     netBufferLists_p,
                                     returnFlags);
        }
        return;
    }

    while (netBufferLists_p != NULL)
    {
        tEdrvRxBuffer           rxBuffer;
        ULONG                   bytesAvailable;
        PUCHAR                  pRxDataSrc;
        ULONG                   bytesToCopy = 0;

        currentNbl = netBufferLists_p;
        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);

        bReturnNbl = TRUE;
        pMdl = NET_BUFFER_CURRENT_MDL(NET_BUFFER_LIST_FIRST_NB(netBufferLists_p));
        rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;

        totalLength = NET_BUFFER_DATA_LENGTH(NET_BUFFER_LIST_FIRST_NB(netBufferLists_p));
        rxBuffer.rxFrameSize = totalLength;
        offset = NET_BUFFER_CURRENT_MDL_OFFSET(NET_BUFFER_LIST_FIRST_NB(netBufferLists_p));

        while ((pMdl != NULL) && totalLength >= 0)
        {
            pRxDataSrc = NULL;
            NdisQueryMdl(pMdl, &pRxDataSrc, &bytesAvailable, NormalPagePriority);
            if (pRxDataSrc == NULL)
            {
                break;
            }
            bytesToCopy = bytesAvailable - offset;
            bytesToCopy = min(bytesToCopy, totalLength);

            OPLK_MEMCPY(rxBuffer.pBuffer, pRxDataSrc, bytesToCopy);
            rxBuffer.pBuffer += bytesToCopy;
            totalLength -= bytesToCopy;

            offset = 0;
            NdisGetNextMdl(pMdl, &pMdl);
        }

        if (NDIS_TEST_RECEIVE_CAN_PEND(receiveFlags_p) == TRUE)
        {
            if (returnNbl == NULL)
            {
                returnNbl = currentNbl;
            }
            else
            {
                NET_BUFFER_LIST_NEXT_NBL(lastReturnNbl) = currentNbl;
            }

            lastReturnNbl = currentNbl;
            NET_BUFFER_LIST_NEXT_NBL(lastReturnNbl) = NULL;
        }
        else
        {
            //
            // Restore the NetBufferList chain
            //
            NET_BUFFER_LIST_NEXT_NBL(currentNbl) = netBufferLists_p;
        }
    }

    if (returnNbl != NULL)
    {
        NdisReturnNetBufferLists(edrvInstance_l.pBindingHandle,
                                 returnNbl,
                                 returnFlags);
    }
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  netBufferLists_p
\param  sendCompleteFlags_p

*/
//------------------------------------------------------------------------------
VOID ndisSendNblComplete(NDIS_HANDLE protocolBindingContext_p,PNET_BUFFER_LIST netBufferLists_p,
                         ULONG sendCompleteFlags_p)
{
    PNET_BUFFER_LIST        currentNbl;
    tNdisTxBuffInfo*        pTxBufInfo;
    while(netBufferLists_p)
    {
        UINT                TotalLength;
        UINT                BufferLength;
        PUCHAR              pCopyData = NULL;
        PUCHAR              protocolResv;
        currentNbl = netBufferLists_p;
        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);
        NET_BUFFER_LIST_NEXT_NBL(currentNbl) = NULL;

        if (NBL_TEST_PROT_RSVD_FLAG(currentNbl, EDRV_ALLOCATED_NBL))
        {
            pTxBufInfo = (tNdisTxBuffInfo*) NET_BUFFER_LIST_PROTOCOL_RESERVED(currentNbl);

            if(pTxBufInfo == NULL)
            {
                break;
            }

            if(pTxBufInfo->pEdrvBuffer->pfnTxHandler != NULL)
            {
                pTxBufInfo->pEdrvBuffer->pfnTxHandler(pTxBufInfo->pEdrvBuffer);
            }
            //DbgPrint("Free\n");
        }
    }
}

///\}

