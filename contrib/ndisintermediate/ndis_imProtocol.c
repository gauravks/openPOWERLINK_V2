/**
********************************************************************************
\file   ndis_imProtocol.c

\brief  Protocol driver implementation for NDIS intermediate driver

This file contains implementation of the protcol section of the intermediate
driver. Protocol section of the intermediate driver is responsible to interface
with the native miniport drivers of the NIC.

\ingroup module_ndis
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
#include <ndis.h>

#include "ndisdriver.h"


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define OPLK_ALLOCATED_NBL          0x10000000
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
NDIS_MEDIUM        aMediumArray[1] =
{
    NdisMedium802_3,    // Ethernet
};

tProtocolInstance   protocolInstance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
NDIS_STATUS bootStrapVEth(PNDIS_STRING instanceName_p);
void freeVEthInstance(tVEthInstance* pVEthInstance_p);
void stopVEth(tVEthInstance* pVEthInstance_p);
NDIS_STATUS allocateTxRxBuf(ULONG txBufCount, ULONG rxBufCount);
void closeBinding(void);
//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief



*/
//------------------------------------------------------------------------------
void protocol_freeVEthInstance(tVEthInstance* pVEthInstance_p)
{
    freeVEthInstance(pVEthInstance_p);
}

//------------------------------------------------------------------------------
/**
\brief



*/
//------------------------------------------------------------------------------
BOOLEAN protocol_checkBindingState()
{
    if (protocolInstance_l.bindingState == NdisBindingRunning)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

//------------------------------------------------------------------------------
/**
\brief



*/
//------------------------------------------------------------------------------
void protocol_registerTxRxHandler(tNdisTransmitCompleteCb pfnTxCallback_p,
    tNdisReceiveCb pfnRxCallback_p)
{
    protocolInstance_l.pfnReceiveCb = pfnRxCallback_p;
    protocolInstance_l.pfnTransmitCompleteCb = pfnTxCallback_p;
}

//------------------------------------------------------------------------------
/**
\brief

\param  txBufCount_p
\param  rxBufCount_p


*/
//------------------------------------------------------------------------------
NDIS_STATUS protocol_allocateTxRxBuf(ULONG txBufCount_p, ULONG rxBufCount_p)
{
    UINT                              index;
    tRxBufInfo*                       rxBufInfo = NULL;
    NDIS_STATUS                       status = NDIS_STATUS_SUCCESS;
    NET_BUFFER_LIST_POOL_PARAMETERS   poolParameters;

    NdisInitializeListHead(&protocolInstance_l.txList);
    NdisAllocateSpinLock(&protocolInstance_l.txListLock);

    NdisZeroMemory(&poolParameters, sizeof(NET_BUFFER_LIST_POOL_PARAMETERS));

    poolParameters.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    poolParameters.Header.Revision = NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
    poolParameters.Header.Size = sizeof(poolParameters);
    poolParameters.ProtocolId = NDIS_PROTOCOL_ID_IPX;       // Sequenced Packet exchanger
    poolParameters.ContextSize = 0;
    poolParameters.fAllocateNetBuffer = TRUE;
    poolParameters.PoolTag = OPLK_MEM_TAG;           //TODO@gks: Change this with a valid tag

    protocolInstance_l.sendNblPool = NdisAllocateNetBufferListPool(driverInstance_l.pProtocolHandle,
        &poolParameters);
    if (protocolInstance_l.sendNblPool == NULL)
    {
        DbgPrint("%s(): failed to alloc send net buffer list pool\n", __FUNCTION__);

        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }
    // Allocate Tx memory
    protocolInstance_l.pTransmitBuf = NdisAllocateMemoryWithTagPriority(driverInstance_l.pProtocolHandle,
        (OPLK_MAX_FRAME_SIZE * txBufCount_p),
        OPLK_MEM_TAG, NormalPoolPriority);

    if (protocolInstance_l.pTransmitBuf == NULL)
    {
        DbgPrint("%s() Failed to allocate Tx buffers\n", __FUNCTION__);
        return NDIS_STATUS_RESOURCES;
    }

    protocolInstance_l.pTxBuffInfo = NdisAllocateMemoryWithTagPriority(driverInstance_l.pProtocolHandle,
        (sizeof(tTxBufInfo) * txBufCount_p),
        OPLK_MEM_TAG, NormalPoolPriority);

    if (protocolInstance_l.pTxBuffInfo == NULL)
    {
        DbgPrint("%s() Failed to allocate Tx buffers info\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    for (index = 0; index < txBufCount_p; index++)
    {
        tTxBufInfo*     pTxInfo = &protocolInstance_l.pTxBuffInfo[index];

        if (pTxInfo != NULL)
        {
            pTxInfo->free = TRUE;
            pTxInfo->maxLength = OPLK_MAX_FRAME_SIZE;
            pTxInfo->pData = (void*) ((ULONG_PTR) protocolInstance_l.pTransmitBuf + (OPLK_MAX_FRAME_SIZE * index));

            pTxInfo->pMdl = NdisAllocateMdl(protocolInstance_l.bindingHandle, pTxInfo->pData, OPLK_MAX_FRAME_SIZE);

            if (pTxInfo->pMdl)
            {
                status = NDIS_STATUS_RESOURCES;
                goto Exit;
            }
            // Allocate empty NetBufferLists
            pTxInfo->pNbl = NdisAllocateNetBufferAndNetBufferList(protocolInstance_l.sendNblPool,
                0, 0, pTxInfo->pMdl, 0, 0);

            if (pTxInfo->pNbl != NULL)
            {
                DbgPrint("%s() Failed to allocate Tx NBL\n", __FUNCTION__);
                status = NDIS_STATUS_RESOURCES;
                goto Exit;
            }

            NBL_SET_PROT_RSVD_FLAG(pTxInfo->pNbl, OPLK_ALLOCATED_NBL);
            pTxInfo->pNbl->SourceHandle = protocolInstance_l.bindingHandle;
            TXINFO_FROM_NBL(pTxInfo->pNbl) = pTxInfo;
            NdisInterlockedInsertTailList(&protocolInstance_l.txList, &pTxInfo->txLink, &protocolInstance_l.txListLock);
        }

    }

    protocolInstance_l.pReceiveBuf = NdisAllocateMemoryWithTagPriority(driverInstance_l.pProtocolHandle,
        (OPLK_MAX_FRAME_SIZE * rxBufCount_p),
        OPLK_MEM_TAG, NormalPoolPriority);
    if (protocolInstance_l.pReceiveBuf == NULL)
    {
        DbgPrint("%s() Failed to allocate Rx buffers\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    protocolInstance_l.pReceiveBufInfo = NdisAllocateMemoryWithTagPriority(driverInstance_l.pProtocolHandle,
        (sizeof(tRxBufInfo) * rxBufCount_p),
        OPLK_MEM_TAG, NormalPoolPriority);
    if (protocolInstance_l.pReceiveBufInfo == NULL)
    {
        DbgPrint("%s() Failed to allocate Rx Info buffers\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Setup rx buffers
    // TODO: Change this to list entry
    for (index = 0; index < rxBufCount_p; index++)
    {
        rxBufInfo = &protocolInstance_l.pReceiveBufInfo[index];
        rxBufInfo->free = TRUE;
        rxBufInfo->length = 0;
        rxBufInfo->maxLength = OPLK_MAX_FRAME_SIZE;
        rxBufInfo->pData = (void*) ((ULONG_PTR) protocolInstance_l.pReceiveBuf + (index * OPLK_MAX_FRAME_SIZE));
    }

    protocolInstance_l.receiveBufCount = rxBufCount_p;
    protocolInstance_l.transmitBufCount = txBufCount_p;

Exit:
    if (status != NDIS_STATUS_SUCCESS)
    {
        protocol_freeTxRxBuffers();
    }
    return status;
}

//------------------------------------------------------------------------------
/**
\brief

\param  ppTxBuf_p


*/
//------------------------------------------------------------------------------
void protocol_freeTxRxBuffers(void)
{
    PLIST_ENTRY     pTxLink;
    tTxBufInfo*     pTxBufInfo;

    if (protocolInstance_l.txList.Flink)
    {
        pTxLink = NdisInterlockedRemoveHeadList(&protocolInstance_l.txList,
            &protocolInstance_l.txListLock);
        while (pTxLink != NULL)
        {
            pTxBufInfo = CONTAINING_RECORD(pTxLink, tTxBufInfo, txLink);
            if (pTxBufInfo->pNbl != NULL)
            {
                NdisFreeNetBufferList(pTxBufInfo->pNbl);
            }

            if (pTxBufInfo->pMdl != NULL)
            {
                NdisFreeMdl(pTxBufInfo->pMdl);
            }
        }
    }

    if (protocolInstance_l.sendNblPool != NULL)
    {
        NdisFreeNetBufferListPool(protocolInstance_l.sendNblPool);
        protocolInstance_l.sendNblPool = NULL;
    }

    if (protocolInstance_l.pTxBuffInfo != NULL)
    {
        NdisFreeMemory(protocolInstance_l.pTxBuffInfo, 0, 0);
    }

    if (protocolInstance_l.pTransmitBuf != NULL)
    {
        NdisFreeMemory(protocolInstance_l.pTransmitBuf,
            (protocolInstance_l.transmitBufCount * OPLK_MAX_FRAME_SIZE),
            0);
    }

    NdisFreeSpinLock(&protocolInstance_l.txListLock);

    if (protocolInstance_l.pReceiveBuf != NULL)
    {
        NdisFreeMemory(protocolInstance_l.pReceiveBuf,
            (protocolInstance_l.receiveBufCount * OPLK_MAX_FRAME_SIZE),
            0);
    }

    if (protocolInstance_l.pReceiveBufInfo != NULL)
    {
        NdisFreeMemory(protocolInstance_l.pReceiveBufInfo,
            (sizeof(tRxBufInfo) * OPLK_MAX_FRAME_SIZE),
            0);
    }
}

//------------------------------------------------------------------------------
/**
\brief

\param  ppTxBuf_p


*/
//------------------------------------------------------------------------------
void protocol_getTxBuff(void** ppTxBuf_p, size_t size_p, void* pTxLink_p)
{
    PLIST_ENTRY     pTxLink;
    tTxBufInfo*     pTxBufInfo;
    if (!IsListEmpty(&protocolInstance_l.txList))
    {
        pTxLink = NdisInterlockedRemoveHeadList(&protocolInstance_l.txList,
            &protocolInstance_l.txListLock);
        if (pTxLink)
        {
            pTxBufInfo = CONTAINING_RECORD(pTxLink, tTxBufInfo, txLink);

            if (pTxBufInfo && size_p <= OPLK_MAX_FRAME_SIZE)
            {
                *ppTxBuf_p = pTxBufInfo->pData;
                pTxLink_p = pTxLink;
            }
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief

\param  ppTxBuf_p


*/
//------------------------------------------------------------------------------
void protocol_freeTxBuff(PVOID pTxLink_p)
{
    PLIST_ENTRY     pTxLink = NULL;

    if (pTxLink_p != NULL)
    {
        pTxLink = (PLIST_ENTRY) pTxLink_p;
    }

    // Return the buffer to linked list the resources will be freed later
    NdisInterlockedInsertTailList(&protocolInstance_l.txList, pTxLink,
        &protocolInstance_l.txListLock);

}

//------------------------------------------------------------------------------
/**
\brief



*/
//------------------------------------------------------------------------------
NDIS_STATUS protocol_sendOidRequest(NDIS_REQUEST_TYPE requestType_p, NDIS_OID oid_p,
    PVOID oidReqBuffer_p, ULONG oidReqBufferLength_p)
{
    tNdisOidRequest*            pNdisOidReq = NULL;
    NDIS_STATUS                 status;

    // Allocate memory for the new request structure
    pNdisOidReq = NdisAllocateMemoryWithTagPriority(protocolInstance_l.bindingHandle, sizeof(tNdisOidRequest),
        OPLK_MEM_TAG, LowPoolPriority);

    if (pNdisOidReq == NULL)
    {
        return NDIS_STATUS_FAILURE;
    }

    // Create the oid request
    pNdisOidReq->oidRequest.Header.Type = NDIS_OBJECT_TYPE_OID_REQUEST;
    pNdisOidReq->oidRequest.Header.Revision = NDIS_OID_REQUEST_REVISION_1;
    pNdisOidReq->oidRequest.Header.Size = sizeof(NDIS_OID_REQUEST);

    pNdisOidReq->oidRequest.RequestType = requestType_p;
    pNdisOidReq->oidRequest.DATA.QUERY_INFORMATION.Oid = oid_p;
    pNdisOidReq->oidRequest.DATA.QUERY_INFORMATION.InformationBuffer = oidReqBuffer_p;
    pNdisOidReq->oidRequest.DATA.QUERY_INFORMATION.InformationBufferLength = oidReqBufferLength_p;

    NdisAcquireSpinLock(&protocolInstance_l.driverLock);
    protocolInstance_l.oidReq++;

    if (protocolInstance_l.bindingState != NdisBindingRunning)
    {
        status = NDIS_STATUS_CLOSING;
        NdisReleaseSpinLock(&protocolInstance_l.driverLock);
    }
    else
    {
        NdisReleaseSpinLock(&protocolInstance_l.driverLock);
        status = NdisOidRequest(protocolInstance_l.bindingHandle, &pNdisOidReq->oidRequest);
    }

    if (status != NDIS_STATUS_PENDING)
    {
        // Request completed or failed
        NdisAcquireSpinLock(&protocolInstance_l.driverLock);
        protocolInstance_l.oidReq--;

        if (protocolInstance_l.oidReq == 0 && protocolInstance_l.pOidCompleteEvent != NULL)
        {
            NdisSetEvent(protocolInstance_l.pOidCompleteEvent);
            protocolInstance_l.pOidCompleteEvent = NULL;
        }
        NdisReleaseSpinLock(&protocolInstance_l.driverLock);
    }
    else
    {
        // Wait for request to complete
        NdisWaitEvent(&pNdisOidReq->waitEvent, 0);
        status = pNdisOidReq->status;
    }

    if (pNdisOidReq != NULL)
    {
        NdisFreeMemory(pNdisOidReq, sizeof(tNdisOidRequest), 0);
    }

    return status;
}



//------------------------------------------------------------------------------
/**
\brief



*/
//------------------------------------------------------------------------------
NDIS_STATUS protocol_sendPacket(void* pToken_p, size_t size_p, void* pTxLink_p)
{
    PLIST_ENTRY     pTxLink = (PLIST_ENTRY) pTxLink_p;
    tTxBufInfo*     pTxBufInfo;
    PNET_BUFFER     netBuffer;
    ULONG           sendFlags = 0;

    pTxBufInfo = CONTAINING_RECORD(pTxLink, tTxBufInfo, txLink);

    if (pTxBufInfo == NULL)
    {
        return NDIS_STATUS_INVALID_DATA;
    }

    pTxBufInfo->pToken = pToken_p;
    // Update the size
    netBuffer = NET_BUFFER_LIST_FIRST_NB(pTxBufInfo->pNbl);
    NET_BUFFER_DATA_LENGTH(netBuffer) = size_p;

    sendFlags |= NDIS_SEND_FLAGS_CHECK_FOR_LOOPBACK;

    // Forward the packet to Lower binding
    NdisSendNetBufferLists(protocolInstance_l.bindingHandle,
        pTxBufInfo->pNbl,
        NDIS_DEFAULT_PORT_NUMBER,
        sendFlags);

    return NDIS_STATUS_SUCCESS;

}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

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
NDIS_STATUS protocolBindAdapter(NDIS_HANDLE protocolDriverContext_p,
                                NDIS_HANDLE bindContext_p,
                                PNDIS_BIND_PARAMETERS pBindParameters_p)
{
    NDIS_STATUS                       status = NDIS_STATUS_SUCCESS;
    NDIS_OPEN_PARAMETERS              openParameters;
    UINT                              mediumIndex = 0;
    NDIS_STRING                       deviceName;
    UNREFERENCED_PARAMETER(protocolDriverContext_p);
//    UNREFERENCED_PARAMETER(bindContext_p);

    if (driverInstance_l.pProtocolHandle == NULL)
    {
        // Driver not registered yet. Is this even possible?
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    NdisInitializeEvent(&protocolInstance_l.adapterEvent);
    NdisAllocateSpinLock(&protocolInstance_l.pauseEventLock);
    NdisAllocateSpinLock(&protocolInstance_l.driverLock);
    //NdisInitializeListHead(&pAdapt->VElanList);
    protocolInstance_l.bindingState = NdisBindingPaused;

    protocolInstance_l.lastLinkState.Header.Revision = NDIS_LINK_STATE_REVISION_1;
    protocolInstance_l.lastLinkState.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    protocolInstance_l.lastLinkState.Header.Size = sizeof(NDIS_LINK_STATE);
    protocolInstance_l.lastLinkState.MediaConnectState = pBindParameters_p->MediaConnectState;
    protocolInstance_l.lastLinkState.MediaDuplexState = pBindParameters_p->MediaDuplexState;
    protocolInstance_l.lastLinkState.XmitLinkSpeed = pBindParameters_p->XmitLinkSpeed;
    protocolInstance_l.lastLinkState.RcvLinkSpeed = pBindParameters_p->RcvLinkSpeed;

    //
    // Now open the adapter below and complete the initialization
    //
    NdisZeroMemory(&openParameters, sizeof(NDIS_OPEN_PARAMETERS));

    openParameters.Header.Type = NDIS_OBJECT_TYPE_OPEN_PARAMETERS;
    openParameters.Header.Revision = NDIS_OPEN_PARAMETERS_REVISION_1;
    openParameters.Header.Size = sizeof(NDIS_OPEN_PARAMETERS);
    openParameters.AdapterName = pBindParameters_p->AdapterName;
    openParameters.MediumArray = aMediumArray;
    openParameters.MediumArraySize = sizeof(aMediumArray) / sizeof(NDIS_MEDIUM);
    openParameters.SelectedMediumIndex = &mediumIndex;

    openParameters.FrameTypeArray = NULL;
    openParameters.FrameTypeArraySize = 0;

    NDIS_DECLARE_PROTOCOL_OPEN_CONTEXT(tProtocolInstance);
    status = NdisOpenAdapterEx(driverInstance_l.pProtocolHandle,
                                &protocolInstance_l,
                                &openParameters,
                                bindContext_p,
                                &protocolInstance_l.bindingHandle);

    if (status == NDIS_STATUS_PENDING)
    {
        // Wait for initialization of adapter to complete
        NdisWaitEvent(&protocolInstance_l.adapterEvent, 0);
        status = protocolInstance_l.adapterInitStatus;
    }

    if (status != NDIS_STATUS_SUCCESS)
    {
        protocolInstance_l.bindingHandle = NULL;
        goto ExitFail;
    }

    protocolInstance_l.bindParameters = *pBindParameters_p;

    if (pBindParameters_p->RcvScaleCapabilities)
    {
        protocolInstance_l.bindParameters.RcvScaleCapabilities = pBindParameters_p->RcvScaleCapabilities;
    }

    // Zeroing out fields that are not needed by the MUX driver
    protocolInstance_l.bindParameters.ProtocolSection = NULL;
    protocolInstance_l.bindParameters.AdapterName = NULL;
    protocolInstance_l.bindParameters.PhysicalDeviceObject = NULL;


    //
    // Start all VEth for this adapterr.
    //

    // TODO: Identify the lower bindings and select one before starting as we support only on lower binding
    status = bootStrapVEth(&deviceName);

    if (status != NDIS_STATUS_SUCCESS)
    {
        goto ExitFail;
    }

ExitFail:
    if (status != NDIS_STATUS_SUCCESS)
    {
        if (protocolInstance_l.bindingHandle != NULL)
        {
            closeBinding();
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
VOID protocolOpenAdapterComplete(NDIS_HANDLE protocolBindingContext_p, NDIS_STATUS status_p)
{
    tProtocolInstance* protInstance = (tProtocolInstance*) protocolBindingContext_p;

    protInstance->adapterInitStatus = status_p;
    NdisSetEvent(&protInstance->adapterEvent);

}

//------------------------------------------------------------------------------
/**
\brief

\param  unbindContext_p
\param  protocolBindingContext_p

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS protocolUnbindAdapter(NDIS_HANDLE unbindContext_p,
    NDIS_HANDLE protocolBindingContext_p)
{
    NDIS_STATUS     Status = NDIS_STATUS_SUCCESS;
    tVEthInstance*  pVEthInstance = NULL;


    if (protocolInstance_l.pVEthInstance != NULL)
    {
        pVEthInstance = (tVEthInstance*) protocolInstance_l.pVEthInstance;
        stopVEth(pVEthInstance);
    }

    if (pVEthInstance != NULL)
    {
        // Wait for VEth module to close
        NdisMSleep(2000);
    }

    //
    // Close the binding to the lower adapter.
    //
    if (protocolInstance_l.bindingHandle != NULL)
    {
        closeBinding();
    }
    else
    {
        //
        // Binding Handle should not be NULL.
        //
        Status = NDIS_STATUS_FAILURE;
        ASSERT(0);
    }
    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p

*/
//------------------------------------------------------------------------------
VOID protocolCloseAdapterComplete(NDIS_HANDLE protocolBindingContext_p)
{
    NdisSetEvent(&protocolInstance_l.adapterEvent);
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  ndisRequest_p
\param  status_p

*/
//------------------------------------------------------------------------------
VOID protocolRequestComplete(NDIS_HANDLE protocolBindingContext_p,
    PNDIS_OID_REQUEST ndisRequest_p,
    NDIS_STATUS status_p)
{
    tNdisOidRequest*        pNdisOidRequest = NULL;

    pNdisOidRequest = CONTAINING_RECORD(ndisRequest_p, tNdisOidRequest, oidRequest);

    // Complete the request
    pNdisOidRequest->status = status_p;
    NdisSetEvent(&pNdisOidRequest->waitEvent);

    NdisAcquireSpinLock(&protocolInstance_l.driverLock);
    // Decrement the request count
    protocolInstance_l.oidReq--;

    // If the no requests pending and driver was waiting for completion set the complete event
    if (protocolInstance_l.oidReq == 0 && protocolInstance_l.pOidCompleteEvent != NULL)
    {
        NdisSetEvent(protocolInstance_l.pOidCompleteEvent);
        protocolInstance_l.pOidCompleteEvent = NULL;
    }

    NdisReleaseSpinLock(&protocolInstance_l.driverLock);
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  statusIndication_p

*/
//------------------------------------------------------------------------------
VOID protocolStatus(NDIS_HANDLE protocolBindingContext_p, PNDIS_STATUS_INDICATION statusIndication_p)
{
    NDIS_STATUS                generalStatus = statusIndication_p->StatusCode;
    NDIS_STATUS_INDICATION     newStatusIndication;
    tVEthInstance*             pVEthinstance = (tVEthInstance*)protocolInstance_l.pVEthInstance;

    if (generalStatus != NDIS_STATUS_LINK_STATE)
    {
        // We only handle Link states here
        return;
    }

    // TODO@gks: DO we need lock here to avoid re-entrance?
    protocolInstance_l.lastLinkState = *((PNDIS_LINK_STATE) (statusIndication_p->StatusBuffer));

    if ((pVEthinstance->miniportHalting) || pVEthinstance->miniportAdapterHandle == NULL)
    {
        pVEthinstance->pendingStatusIndication = generalStatus;
        pVEthinstance->lastPendingLinkState = *((PNDIS_LINK_STATE) (statusIndication_p->StatusBuffer));
        return;
    }

    pVEthinstance->lastLinkStatus = generalStatus;
    if (generalStatus != NDIS_STATUS_LINK_STATE)
    {
        pVEthinstance->lastLinkState = *((PNDIS_LINK_STATE) (statusIndication_p->StatusBuffer));
    }

    // Allocate a new status indication and pass it protocols bind to VEth
    NdisZeroMemory(&newStatusIndication, sizeof(NDIS_STATUS_INDICATION));

    newStatusIndication.Header.Type = NDIS_OBJECT_TYPE_STATUS_INDICATION;
    newStatusIndication.Header.Revision = NDIS_STATUS_INDICATION_REVISION_1;
    newStatusIndication.Header.Size = sizeof(NDIS_STATUS_INDICATION);

    newStatusIndication.StatusCode = statusIndication_p->StatusCode;
    newStatusIndication.SourceHandle = pVEthinstance->miniportAdapterHandle;
    newStatusIndication.DestinationHandle = NULL;

    newStatusIndication.StatusBuffer = statusIndication_p->StatusBuffer;
    newStatusIndication.StatusBufferSize = statusIndication_p->StatusBufferSize;

    NdisMIndicateStatusEx(pVEthinstance->miniportAdapterHandle, &newStatusIndication);
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p
\param  pNetPnPEventNotification_p

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS protocolPnpHandler(NDIS_HANDLE protocolBindingContext_p,
                               PNET_PNP_EVENT_NOTIFICATION pNetPnPEventNotification_p)
{
    NDIS_STATUS         status = NDIS_STATUS_SUCCESS;
    NDIS_EVENT          pPauseEvent;

    switch (pNetPnPEventNotification_p->NetPnPEvent.NetEvent)
    {
        case NetEventSetPower:

            // TODO@gks: We don't handle power state configuration now.
            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventReconfigure:

            status = NDIS_STATUS_SUCCESS;
            break;
        case NetEventIMReEnableDevice:

            bootStrapVEth(pNetPnPEventNotification_p->NetPnPEvent.Buffer);

            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventPause:

            NdisAcquireSpinLock(&protocolInstance_l.driverLock);

            ASSERT(protocolInstance_l.pauseEvent == NULL);
            // Wait for all the send requests to complete
            // TODO@gks : Shall we wait indefinitely?
            if (protocolInstance_l.sendRequest != 0)
            {
                NdisInitializeEvent(&pPauseEvent);

                protocolInstance_l.pPauseEvent = &pPauseEvent;

                NdisReleaseSpinLock(&protocolInstance_l.driverLock);

                NdisWaitEvent(&pPauseEvent, 0);
                NdisAcquireSpinLock(&protocolInstance_l.driverLock);
            }
            protocolInstance_l.bindingState = NdisBindingPaused;

            NdisReleaseSpinLock(&protocolInstance_l.driverLock);
            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventRestart:
            protocolInstance_l.bindingState = NdisBindingRunning;
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
VOID protocolReceiveNbl(NDIS_HANDLE protocolBindingContext_p, PNET_BUFFER_LIST netBufferLists_p,
                        NDIS_PORT_NUMBER  portNumber_p, ULONG numberOfNbl_p,
                        ULONG receiveFlags_p)
{
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

    if (protocolInstance_l.bindingState != NdisBindingRunning)
    {
        if (NDIS_TEST_RECEIVE_CAN_PEND(receiveFlags_p) == TRUE)
        {
            NdisReturnNetBufferLists(protocolInstance_l.bindingHandle,
                netBufferLists_p,
                returnFlags);
        }
        return;
    }

    while (netBufferLists_p != NULL)
    {
        ULONG                   bytesAvailable = 0;
        PUCHAR                  pRxDataSrc;
        ULONG                   bytesToCopy = 0;
        tRxBufInfo*             rxBufInfo = NULL;

        currentNbl = netBufferLists_p;
        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);
        NET_BUFFER_LIST_NEXT_NBL(currentNbl) = NULL;

        rxBufInfo = &protocolInstance_l.pReceiveBufInfo[protocolInstance_l.receiveHead];
        bReturnNbl = TRUE;
        pMdl = NET_BUFFER_CURRENT_MDL(NET_BUFFER_LIST_FIRST_NB(currentNbl));
        totalLength = NET_BUFFER_DATA_LENGTH(NET_BUFFER_LIST_FIRST_NB(currentNbl));
        if (rxBufInfo->maxLength >= totalLength)
        {
            rxBufInfo->length = totalLength;
        }

        offset = NET_BUFFER_CURRENT_MDL_OFFSET(NET_BUFFER_LIST_FIRST_NB(currentNbl));

        while ((pMdl != NULL) && (totalLength > 0))
        {
            pRxDataSrc = NULL;
            NdisQueryMdl(pMdl, &pRxDataSrc, &bytesAvailable, NormalPagePriority);
            if (pRxDataSrc == NULL)
            {
                break;
            }
            bytesToCopy = bytesAvailable - offset;
            bytesToCopy = min(bytesToCopy, totalLength);
            
           
            NdisMoveMemory(rxBufInfo->pData, pRxDataSrc, bytesToCopy);
            rxBufInfo->pData = (PVOID)((ULONG_PTR)rxBufInfo->pData + bytesToCopy);
            totalLength -= bytesToCopy;

            offset = 0;
            NdisGetNextMdl(pMdl, &pMdl);
        }

        protocolInstance_l.receiveHead = (protocolInstance_l.receiveHead + 1) & (protocolInstance_l.receiveBufCount -1);

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
        NdisReturnNetBufferLists(protocolInstance_l.bindingHandle,
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
VOID protocolSendNblComplete(NDIS_HANDLE protocolBindingContext_p, PNET_BUFFER_LIST netBufferLists_p,
    ULONG sendCompleteFlags_p)
{
    PNET_BUFFER_LIST        currentNbl;
    PLIST_ENTRY             pTxLink;
    tTxBufInfo*             pTxBufInfo;

    while (netBufferLists_p)
    {
        UINT                TotalLength;
        UINT                BufferLength;
        PUCHAR              pCopyData = NULL;
        PUCHAR              protocolResv;
        currentNbl = netBufferLists_p;
        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);
        NET_BUFFER_LIST_NEXT_NBL(currentNbl) = NULL;

        if (NBL_TEST_PROT_RSVD_FLAG(currentNbl, OPLK_ALLOCATED_NBL))
        {
            pTxLink = (PLIST_ENTRY) TXINFO_FROM_NBL(currentNbl);

            if (pTxLink == NULL)
            {
                break;
            }

            pTxBufInfo = CONTAINING_RECORD(pTxLink, tTxBufInfo, txLink);
            if (protocolInstance_l.pfnTransmitCompleteCb != NULL)
            {
                protocolInstance_l.pfnTransmitCompleteCb(pTxBufInfo->pToken);
            }
            //DbgPrint("Free\n");
        }

    }
}

//------------------------------------------------------------------------------
/**
\brief

\param  protocolBindingContext_p


*/
//------------------------------------------------------------------------------
void closeBinding(void)
{
    NDIS_STATUS             status;
    NDIS_EVENT              oidCompleteEvent;
    ULONG                   packetFilter = 0;
    PVOID                   multicastAddrBuf = NULL;
    ULONG                   multicastAddrBufSize = 0;

    // Disable Multicast filter
    protocol_sendOidRequest(NdisRequestSetInformation, OID_GEN_CURRENT_PACKET_FILTER, &packetFilter,
                   sizeof(packetFilter));

    // Delete multicast list. We have already deleted individual entries before.
    protocol_sendOidRequest(NdisRequestSetInformation, OID_802_3_MULTICAST_LIST, &multicastAddrBuf,
                   multicastAddrBufSize);

    NdisAcquireSpinLock(&protocolInstance_l.driverLock);

    // Wait for all the oid request to be completed
    if (protocolInstance_l.oidReq != 0)
    {
        NdisInitializeEvent(&oidCompleteEvent);
        protocolInstance_l.pOidCompleteEvent = &oidCompleteEvent;
        NdisReleaseSpinLock(&protocolInstance_l.driverLock);
        NdisWaitEvent(protocolInstance_l.pOidCompleteEvent, 0);
        NdisAcquireSpinLock(&protocolInstance_l.driverLock);
    }

    // Free NBL pool
    NdisFreeNetBufferListPool(protocolInstance_l.sendNblPool);
    NdisReleaseSpinLock(&protocolInstance_l.driverLock);

    NdisResetEvent(&protocolInstance_l.adapterEvent);

    // Close binding with lower layer miniport
    status = NdisCloseAdapterEx(protocolInstance_l.bindingHandle);

    if (status == NDIS_STATUS_PENDING)
    {
        NdisWaitEvent(&protocolInstance_l.adapterEvent, 0);
    }

    // Binding closed
    protocolInstance_l.bindingHandle = NULL;

}


//------------------------------------------------------------------------------
/**
\brief



*/
//------------------------------------------------------------------------------
NDIS_STATUS bootStrapVEth(PNDIS_STRING instanceName_p)
{
    NDIS_STATUS                     status = NDIS_STATUS_SUCCESS;
    tVEthInstance*                  pVEthInstance = NULL;
    NDIS_HANDLE                     adapterConfigHandle;
    PNDIS_CONFIGURATION_PARAMETER   configParam;
    NDIS_STRING                     upperBindingStr = NDIS_STRING_CONST("UpperBindings");
    PWSTR                           devName;
    LOCK_STATE                      lockState;
    NDIS_CONFIGURATION_OBJECT       configObject;

    status = NDIS_STATUS_SUCCESS;

    // TODO: This peice of code shall be used to identify the lower binding
    adapterConfigHandle = NULL;

    // Get the configuration of lower binding
    configObject.Header.Type = NDIS_OBJECT_TYPE_CONFIGURATION_OBJECT;
    configObject.Header.Revision = NDIS_CONFIGURATION_OBJECT_REVISION_1;
    configObject.Header.Size = sizeof(NDIS_CONFIGURATION_OBJECT);
    configObject.NdisHandle = protocolInstance_l.bindingHandle;
    configObject.Flags = 0;

    status = NdisOpenConfigurationEx(&configObject, &adapterConfigHandle);

    if (status != NDIS_STATUS_SUCCESS)
    {
        adapterConfigHandle = NULL;
        DbgPrint("%s() Failed to open adapter configuration\n", __FUNCTION__);
        return NDIS_STATUS_OPEN_FAILED;
    }

    //
    // Read the "UpperBindings" reserved key that contains a list
    // of device names representing our miniport instances corresponding
    // to this lower binding. The UpperBindings is a 
    // MULTI_SZ containing a list of device names. We will loop through
    // this list and initialize the virtual miniports.
    //
    NdisReadConfiguration(&status,
                          &configParam,
                          adapterConfigHandle,
                          &upperBindingStr,
                          NdisParameterMultiString);

    if (status != NDIS_STATUS_SUCCESS)
    {
        DbgPrint("Unenable to read configuration\n");
        return status;
    }

    devName = configParam->ParameterData.StringData.Buffer;

    while (*devName != L'\0')
    {
        NDIS_STRING     devString;
        ULONG           length;

        NdisInitUnicodeString(&devString, devName);

        if (instanceName_p != NULL)
        {
            if (NdisEqualString(&devString, instanceName_p, TRUE))
            {
                length = sizeof(tVEthInstance) + devString.Length + sizeof(WCHAR);
                // Allocate a new VEth instance
                pVEthInstance = NdisAllocateMemoryWithTagPriority(protocolInstance_l.bindingHandle,
                                            length, OPLK_MEM_TAG, LowPoolPriority);

                if (pVEthInstance != NULL)
                {
                    DbgPrint("%s() Failed to allocate memory for VEth instance\n", __FUNCTION__);
                    goto ExitFail;
                }

                NdisZeroMemory(pVEthInstance, length);
                pVEthInstance->cfgDeviceName.Length = 0;
                pVEthInstance->cfgDeviceName.Buffer = (PWCHAR) ((PUCHAR) pVEthInstance + sizeof(tVEthInstance));
                pVEthInstance->cfgDeviceName.MaximumLength = devString.Length + sizeof(WCHAR);
                (void) NdisUpcaseUnicodeString(&pVEthInstance->cfgDeviceName, &devString);
                pVEthInstance->cfgDeviceName.Buffer[devString.Length / sizeof(WCHAR)] = ((WCHAR) 0);

                // Setup initial miniport parameters
                pVEthInstance->lastLinkStatus = NDIS_STATUS_LINK_STATE;

                pVEthInstance->lookAhead = protocolInstance_l.bindParameters.LookaheadSize;
                pVEthInstance->linkSpeed = protocolInstance_l.bindParameters.RcvLinkSpeed;

                if (protocolInstance_l.bindParameters.MacAddressLength == 6)
                {
                    NdisMoveMemory(pVEthInstance->permanentAddress,
                                   &protocolInstance_l.bindParameters.CurrentMacAddress,
                                   protocolInstance_l.bindParameters.MacAddressLength);

                    NdisMoveMemory(pVEthInstance->currentAddress,
                                   &protocolInstance_l.bindParameters.CurrentMacAddress,
                                   protocolInstance_l.bindParameters.MacAddressLength);
                }
                else
                {
                    status = NDIS_STATUS_NOT_SUPPORTED;
                    goto ExitFail;
                }

                NdisAllocateSpinLock(&pVEthInstance->miniportLock);
                NdisAllocateSpinLock(&pVEthInstance->pauseLock);

                pVEthInstance->miniportInitPending = TRUE;
                NdisInitializeEvent(&pVEthInstance->miniportInitEvent);

                // Initialize Miniport instance for the VEth
                status = NdisIMInitializeDeviceInstanceEx(driverInstance_l.pMiniportHandle,
                                                          &pVEthInstance->cfgDeviceName,
                                                          pVEthInstance);

                if (status != NDIS_STATUS_SUCCESS)
                {
                    if (!pVEthInstance->miniportHalting)
                    {
                        goto ExitFail;
                    }
                }
            }
        }
    }

    if (adapterConfigHandle != NULL)
    {
        NdisCloseConfiguration(adapterConfigHandle);
    }

    goto Exit;

ExitFail:
    if (status != NDIS_STATUS_SUCCESS)
    {
        if (pVEthInstance)
        {
            freeVEthInstance(pVEthInstance);
            pVEthInstance = NULL;
        }
    }

Exit:
    protocolInstance_l.pVEthInstance = (void*)pVEthInstance;
    return status;
}

//------------------------------------------------------------------------------
/**
\brief



*/
//------------------------------------------------------------------------------
void freeVEthInstance(tVEthInstance* pVEthInstance_p)
{
    if (pVEthInstance_p != NULL)
    {
        NdisFreeSpinLock(&pVEthInstance_p->miniportLock);
        NdisFreeMemory(pVEthInstance_p, 0, 0);
    }
}

//------------------------------------------------------------------------------
/**
\brief



*/
//------------------------------------------------------------------------------
void stopVEth(tVEthInstance* pVEthInstance_p)
{
    NDIS_STATUS     status;
    NDIS_HANDLE     miniportAdapterHandle;
    BOOLEAN         fMiniportInitCancelled;

    NdisAcquireSpinLock(&pVEthInstance_p->miniportLock);

    if (pVEthInstance_p->oidRequestPending)
    {
        pVEthInstance_p->oidRequestPending = FALSE;
        NdisReleaseSpinLock(&pVEthInstance_p->miniportLock);
        protocolRequestComplete(&protocolInstance_l, &pVEthInstance_p->ndisOidReq.oidRequest,
                                NDIS_STATUS_FAILURE);
    }
    else
    {
        NdisReleaseSpinLock(&pVEthInstance_p->miniportLock);
    }

    if (pVEthInstance_p->miniportInitPending)
    {
        status = NdisIMCancelInitializeDeviceInstance(
                 driverInstance_l.pMiniportHandle,
                 &pVEthInstance_p->cfgDeviceName);
        if (status != NDIS_STATUS_SUCCESS)
        {
            pVEthInstance_p->miniportInitPending = FALSE;
            fMiniportInitCancelled = TRUE;
        }
        else
        {
            NdisWaitEvent(&pVEthInstance_p->miniportInitEvent, 0);
        }
    }

    if (pVEthInstance_p->miniportAdapterHandle != NULL &&
        (!pVEthInstance_p->miniportHalting))
    {
        // Stop the miniport
        (void) NdisIMDeInitializeDeviceInstance(pVEthInstance_p->miniportAdapterHandle);
    }
    else
    {
        if (fMiniportInitCancelled)
        {
            freeVEthInstance(pVEthInstance_p);
        }
    }
}


    ///\}






