/**
********************************************************************************
\file   ndis_imMiniport.c

\brief  Miniport implementation of NDIS intermediate driver

// TODO@gks: Add description here

\ingroup ndis_intermediate
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
//#include <ndis.h>

#include "ndisDriver.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
//#define MAX_TX_FRAME_LENGTH     OPLK_MAX_FRAME_SIZE
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
NDIS_OID          VEthSupportedOids[] =
{
    OID_GEN_SUPPORTED_LIST,
    OID_GEN_HARDWARE_STATUS,
    OID_GEN_MEDIA_SUPPORTED,
    OID_GEN_MEDIA_IN_USE,
    OID_GEN_MAXIMUM_LOOKAHEAD,
    OID_GEN_MAXIMUM_FRAME_SIZE,
    OID_GEN_LINK_SPEED,
    OID_GEN_TRANSMIT_BUFFER_SPACE,
    OID_GEN_RECEIVE_BUFFER_SPACE,
    OID_GEN_TRANSMIT_BLOCK_SIZE,
    OID_GEN_RECEIVE_BLOCK_SIZE,
    OID_GEN_VENDOR_ID,
    OID_GEN_VENDOR_DESCRIPTION,
    OID_GEN_VENDOR_DRIVER_VERSION,
    OID_GEN_CURRENT_PACKET_FILTER,
    OID_GEN_CURRENT_LOOKAHEAD,
    OID_GEN_DRIVER_VERSION,
    OID_GEN_MAXIMUM_TOTAL_SIZE,
    OID_GEN_PROTOCOL_OPTIONS,
    OID_GEN_MAC_OPTIONS,
    OID_GEN_MEDIA_CONNECT_STATUS,
    OID_GEN_MAXIMUM_SEND_PACKETS,
    OID_GEN_XMIT_OK,
    OID_GEN_RCV_OK,
    OID_GEN_XMIT_ERROR,
    OID_GEN_RCV_ERROR,
    OID_GEN_RCV_NO_BUFFER,
    OID_GEN_RCV_CRC_ERROR,
    OID_GEN_TRANSMIT_QUEUE_LENGTH,
    OID_GEN_STATISTICS,
    OID_802_3_PERMANENT_ADDRESS,
    OID_802_3_CURRENT_ADDRESS,
    OID_802_3_MULTICAST_LIST,
    OID_802_3_MAXIMUM_LIST_SIZE,
    OID_802_3_RCV_ERROR_ALIGNMENT,
    OID_802_3_XMIT_ONE_COLLISION,
    OID_802_3_XMIT_MORE_COLLISIONS,
    OID_802_3_XMIT_DEFERRED,
    OID_802_3_XMIT_MAX_COLLISIONS,
    OID_802_3_RCV_OVERRUN,
    OID_802_3_XMIT_UNDERRUN,
    OID_802_3_XMIT_HEARTBEAT_FAILURE,
    OID_802_3_XMIT_TIMES_CRS_LOST,
    OID_802_3_XMIT_LATE_COLLISIONS,
    OID_PNP_CAPABILITIES,
    OID_PNP_SET_POWER,
    OID_PNP_QUERY_POWER,
    OID_PNP_ADD_WAKE_UP_PATTERN,
    OID_PNP_REMOVE_WAKE_UP_PATTERN,
    OID_PNP_ENABLE_WAKE_UP
};

static BOOLEAN    fInitialize = FALSE;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Miniport initialize routine

This is the Miniport Initialize routine which gets called as a result of our
call to NdisIMInitializeDeviceInstanceEx. The context parameter which we
pass there is the VEthInstance structure which we retrieve here.

\param  adapterHandle_p     NDIS handle for this miniport
\param  driverContext_p     Handle passed to NDIS when we registered the driver
\param  initParams_p        Miniport initialization parameters such
                            as our device context, resources, etc.

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportInitialize(NDIS_HANDLE adapterHandle_p,
                               NDIS_HANDLE driverContext_p,
                               PNDIS_MINIPORT_INIT_PARAMETERS initParams_p)
{
    tVEthInstance*                      pVEthInstance;
    NDIS_STATUS                         status = NDIS_STATUS_FAILURE;
    NDIS_MINIPORT_ADAPTER_ATTRIBUTES    miniportAttributes;
    NDIS_CONFIGURATION_OBJECT           configObject;
    NDIS_HANDLE    configHandle;
    PVOID          macAddress;
    UINT           macLength;
    NET_IFINDEX    highLayerIfIndex;
    NET_IFINDEX    lowerLayerIfIndex;
    ULONG          packetFilter = NDIS_PACKET_TYPE_PROMISCUOUS;

    UNREFERENCED_PARAMETER(driverContext_p);

    DbgPrint("%s()... \n", __FUNCTION__);

    if (fInitialize == TRUE)
    {
        return NDIS_STATUS_SUCCESS;
    }
    pVEthInstance = (tVEthInstance*) initParams_p->IMDeviceInstanceContext;

    NdisZeroMemory(&miniportAttributes, sizeof(NDIS_MINIPORT_ADAPTER_ATTRIBUTES));

    pVEthInstance->miniportAdapterHandle = adapterHandle_p;

    // Register IOCTL interface here
    ndis_createAppIntf();

    miniportAttributes.RegistrationAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES;
    miniportAttributes.RegistrationAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1;
    miniportAttributes.RegistrationAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES);

    miniportAttributes.RegistrationAttributes.MiniportAdapterContext = (NDIS_HANDLE) pVEthInstance;
    miniportAttributes.RegistrationAttributes.AttributeFlags = NDIS_MINIPORT_ATTRIBUTES_NO_HALT_ON_SUSPEND;
    miniportAttributes.RegistrationAttributes.CheckForHangTimeInSeconds = 0;
    miniportAttributes.RegistrationAttributes.InterfaceType = 0;

    NDIS_DECLARE_MINIPORT_ADAPTER_CONTEXT(tVEthInstance);
    status = NdisMSetMiniportAttributes(adapterHandle_p, &miniportAttributes);

    if (status != NDIS_STATUS_SUCCESS)
    {
        goto Exit;
    }

    // TODO@gks check if we need this for our VETH
    configObject.Header.Type = NDIS_OBJECT_TYPE_CONFIGURATION_OBJECT;
    configObject.Header.Revision = NDIS_CONFIGURATION_OBJECT_REVISION_1;
    configObject.Header.Size = sizeof(NDIS_CONFIGURATION_OBJECT);
    configObject.NdisHandle = pVEthInstance->miniportAdapterHandle;
    configObject.Flags = 0;

    status = NdisOpenConfigurationEx(&configObject, &configHandle);

    if (status != NDIS_STATUS_SUCCESS)
    {
        goto Exit;
    }

    NdisReadNetworkAddress(&status, &macAddress, &macLength, configHandle);

    if ((status == NDIS_STATUS_SUCCESS) && (macLength == ETH_LENGTH_OF_ADDRESS) &&
        (!ETH_IS_MULTICAST(macAddress)))
    {
        ETH_COPY_NETWORK_ADDRESS(pVEthInstance->currentAddress, macAddress);
    }
    else
    {
        ETH_COPY_NETWORK_ADDRESS(pVEthInstance->currentAddress, pVEthInstance->permanentAddress);
    }

    //
    status = NDIS_STATUS_SUCCESS;

    NdisCloseConfiguration(configHandle);

    miniportAttributes.GeneralAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES;
    miniportAttributes.GeneralAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1;
    miniportAttributes.GeneralAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES);
    miniportAttributes.GeneralAttributes.MediaType = NdisMedium802_3;

    miniportAttributes.GeneralAttributes.MtuSize = pVEthInstance->protocolInstance->bindParameters.MtuSize;
    miniportAttributes.GeneralAttributes.MaxXmitLinkSpeed = pVEthInstance->protocolInstance->bindParameters.MaxXmitLinkSpeed;
    miniportAttributes.GeneralAttributes.MaxRcvLinkSpeed = pVEthInstance->protocolInstance->bindParameters.MaxRcvLinkSpeed;
    miniportAttributes.GeneralAttributes.XmitLinkSpeed = pVEthInstance->protocolInstance->bindParameters.XmitLinkSpeed;
    miniportAttributes.GeneralAttributes.RcvLinkSpeed = pVEthInstance->protocolInstance->bindParameters.RcvLinkSpeed;

    miniportAttributes.GeneralAttributes.MediaConnectState = pVEthInstance->protocolInstance->lastLinkState.MediaConnectState;
    miniportAttributes.GeneralAttributes.MediaDuplexState = pVEthInstance->protocolInstance->lastLinkState.MediaDuplexState;
    miniportAttributes.GeneralAttributes.XmitLinkSpeed = pVEthInstance->protocolInstance->lastLinkState.XmitLinkSpeed;
    miniportAttributes.GeneralAttributes.RcvLinkSpeed = pVEthInstance->protocolInstance->lastLinkState.RcvLinkSpeed;

    pVEthInstance->lastLinkStatus = NDIS_STATUS_LINK_STATE;

    pVEthInstance->lastLinkState = pVEthInstance->protocolInstance->lastLinkState;

    miniportAttributes.GeneralAttributes.LookaheadSize = pVEthInstance->protocolInstance->bindParameters.LookaheadSize;
    miniportAttributes.GeneralAttributes.MaxMulticastListSize = pVEthInstance->protocolInstance->bindParameters.MaxMulticastListSize;
    miniportAttributes.GeneralAttributes.MacAddressLength = pVEthInstance->protocolInstance->bindParameters.MacAddressLength;

    miniportAttributes.GeneralAttributes.PhysicalMediumType = pVEthInstance->protocolInstance->bindParameters.PhysicalMediumType;
    miniportAttributes.GeneralAttributes.AccessType = pVEthInstance->protocolInstance->bindParameters.AccessType;
    miniportAttributes.GeneralAttributes.DirectionType = pVEthInstance->protocolInstance->bindParameters.DirectionType;
    miniportAttributes.GeneralAttributes.ConnectionType = pVEthInstance->protocolInstance->bindParameters.ConnectionType;
    miniportAttributes.GeneralAttributes.IfType = pVEthInstance->protocolInstance->bindParameters.IfType;
    miniportAttributes.GeneralAttributes.IfConnectorPresent = FALSE; // RFC 2665 TRUE if physical adapter
    miniportAttributes.GeneralAttributes.RecvScaleCapabilities = NULL;
    miniportAttributes.GeneralAttributes.MacOptions = NDIS_MAC_OPTION_NO_LOOPBACK;

    miniportAttributes.GeneralAttributes.SupportedPacketFilters = pVEthInstance->protocolInstance->bindParameters.SupportedPacketFilters;

    miniportAttributes.GeneralAttributes.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_OK_SUPPORTED |
                                                               NDIS_STATISTICS_XMIT_ERROR_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_ERROR_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_CRC_ERROR_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_NO_BUFFER_SUPPORTED |
                                                               NDIS_STATISTICS_TRANSMIT_QUEUE_LENGTH_SUPPORTED |
                                                               NDIS_STATISTICS_GEN_STATISTICS_SUPPORTED;

    NdisMoveMemory(&miniportAttributes.GeneralAttributes.CurrentMacAddress,
                   &pVEthInstance->currentAddress,
                   ETH_LENGTH_OF_ADDRESS);

    NdisMoveMemory(&miniportAttributes.GeneralAttributes.PermanentMacAddress,
                   &pVEthInstance->permanentAddress,
                   ETH_LENGTH_OF_ADDRESS);
    miniportAttributes.GeneralAttributes.PowerManagementCapabilities = NULL;
    miniportAttributes.GeneralAttributes.SupportedOidList = VEthSupportedOids;
    miniportAttributes.GeneralAttributes.SupportedOidListLength = sizeof(VEthSupportedOids);

    //Release lock
    status = NdisMSetMiniportAttributes(adapterHandle_p, &miniportAttributes);

    pVEthInstance->miniportInitPending = FALSE;

Exit:

    if (status == NDIS_STATUS_SUCCESS)
    {
        //
        // we should set this to FALSE only if we successfully initialized the adapter
        // otherwise the unbind routine will wait forever for this instance to go away
        //
        pVEthInstance->miniportInitPending = FALSE;
        //
        // Save the IfIndex for this VELAN
        //
        highLayerIfIndex = initParams_p->IfIndex;
        lowerLayerIfIndex = pVEthInstance->protocolInstance->bindParameters.BoundIfIndex;

        status = NdisIfAddIfStackEntry(highLayerIfIndex, lowerLayerIfIndex);

        if (status == NDIS_STATUS_SUCCESS)
        {
            pVEthInstance->ifIndex = highLayerIfIndex;
        }

        //
        // Ignore if the add fails
        //
        status = NDIS_STATUS_SUCCESS;
    }
    else
    {
        pVEthInstance->miniportAdapterHandle = NULL;
    }

    if (pVEthInstance->miniportInitPending == TRUE)
    {
        pVEthInstance->miniportInitPending = FALSE;
    }

    // TODO: check to see if we can set the init event in a failure case?

    protocol_sendOidRequest(NdisRequestSetInformation, OID_GEN_CURRENT_PACKET_FILTER, &packetFilter,
                            sizeof(packetFilter));

    NdisSetEvent(&pVEthInstance->miniportInitEvent);
    DbgPrint("%s() - OK \n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief MiniportRequest dispatch handler

\param  adapterContext_p        Pointer to the adapter structure.
\param  ndisRequest_p           Pointer to NDIS_OID_REQUEST sent down by NDIS.

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportOidRequest(NDIS_HANDLE adapterContext_p,
                               PNDIS_OID_REQUEST ndisRequest_p)
{
    //    tVEthInstance*          pVEthInstance = (tVEthInstance*) adapterContext_p;
    NDIS_REQUEST_TYPE    requestType;
    NDIS_STATUS          status;

    requestType = ndisRequest_p->RequestType;

    switch (requestType)
    {
        case NdisRequestMethod:
        {
            // Do Nothing for now
            status = NDIS_STATUS_SUCCESS;
            break;
        }
        case NdisRequestSetInformation:
        {
            // Do nothing for now
            status = NDIS_STATUS_SUCCESS;
            break;
        }
        case NdisRequestQueryInformation:
        case NdisRequestQueryStatistics:
        {
            // Do nothing for now
            status = NDIS_STATUS_SUCCESS;
            break;
        }
        default:
            status = NDIS_STATUS_NOT_SUPPORTED;
            break;
    }

    return status;
}

//------------------------------------------------------------------------------
/**
\brief Halt handler

Stop all pending I/O on the VEth and then unlink it from lower miniport.

\param  adapterContext_p    Pointer to the adapter structure.
\param  haltAction_p        The reason adapter is being halted.

*/
//------------------------------------------------------------------------------
VOID miniportHalt(NDIS_HANDLE adapterContext_p, NDIS_HALT_ACTION haltAction_p)
{
    tVEthInstance*   pVEthInstance = (tVEthInstance*) adapterContext_p;
    NET_IFINDEX      lowerLayerIfIndex;

    DbgPrint("%s()...\n", __FUNCTION__);

    pVEthInstance->miniportHalting = TRUE;

    while (pVEthInstance->sendRequests)
    {
        // Wait for completion of ASynchronous sends on VEth
        NdisMSleep(20000);
    }

    while (pVEthInstance->receiveIndication)
    {
        // Wait for all the receive packets to be returned back to us
        NdisMSleep(20000);
    }

    ndis_closeAppIntf();

    if (pVEthInstance->ifIndex != 0)
    {
        // Remove driver from the stack
        lowerLayerIfIndex = pVEthInstance->protocolInstance->bindParameters.BoundIfIndex;
        NdisIfDeleteIfStackEntry(pVEthInstance->ifIndex, lowerLayerIfIndex);

        pVEthInstance->ifIndex = 0;
    }

    // Free the VETh instance
    pVEthInstance->miniportAdapterHandle = NULL;

    protocol_freeVEthInstance(pVEthInstance);

    DbgPrint("%s() - OK\n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief Miniport PNP event handler

This handler is called to notify us of PnP events directed to our miniport
device object

\param  adapterContext_p    Pointer to the adapter structure.
\param  pnPEvent_p          Pointer to the PNP event.

*/
//------------------------------------------------------------------------------
VOID miniportPnPEventNotify(NDIS_HANDLE adapterContext_p, PNET_DEVICE_PNP_EVENT pnPEvent_p)
{
    // Not a real device
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(pnPEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief Miniport Shutdown Handler

This handler is called to notify us of an impending system shutdown.
Since this is not a hardware driver, there isn't anything specific
we need to do about this.

\param  adapterContext_p    Pointer to the adapter structure.
\param  shutdownAction_P    Specific reason to shut down the adapter.

*/
//------------------------------------------------------------------------------
VOID miniportShutdown(NDIS_HANDLE adapterContext_p, NDIS_SHUTDOWN_ACTION shutdownAction_P)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(shutdownAction_P);
}

//------------------------------------------------------------------------------
/**
\brief Miniport Unload Handler

This handler is used to unload the miniport

\param  driverObject_p  Pointer to the system's driver object structure
                        for this driver.

*/
//------------------------------------------------------------------------------
VOID miniportUnload(PDRIVER_OBJECT driverObject_p)
{
    DbgPrint("%s()...\n", __FUNCTION__);
    UNREFERENCED_PARAMETER(driverObject_p);
    if (driverInstance_l.pProtocolHandle != NULL)
    {
        NdisDeregisterProtocolDriver(driverInstance_l.pProtocolHandle);
    }

    NdisMDeregisterMiniportDriver(driverInstance_l.pMiniportHandle);
    DbgPrint("%s() - OK\n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief Miniport pause handler

This handler is used to pause the miniport. During which, no NET_BUFFER_LIST
will be indicated to the upper binding as well as status indications.

\param  adapterContext_p    Pointer to the adapter structure.
\param  pauseParams_p       Pause parameters.

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportPause(NDIS_HANDLE adapterContext_p,
                          PNDIS_MINIPORT_PAUSE_PARAMETERS pauseParams_p)
{
    tVEthInstance*   pVEthInstance = (tVEthInstance*) adapterContext_p;
    NDIS_STATUS      status = NDIS_STATUS_SUCCESS;

    DbgPrint("%s()...\n", __FUNCTION__);
    NdisAcquireSpinLock(&pVEthInstance->pauseLock);
    pVEthInstance->miniportPaused = TRUE;
    NdisReleaseSpinLock(&pVEthInstance->pauseLock);

    DbgPrint("%s() - OK\n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Miniport restart handler

This handler is used to restart the miniport.  When the miniport is
back in the restart state, it can indicate NET_BUFFER_LISTs to the
upper binding

\param  adapterContext_p    Pointer to the adapter structure.
\param  restartParams_p     Restart parameters for miniport.

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportRestart(NDIS_HANDLE adapterContext_p,
                            PNDIS_MINIPORT_RESTART_PARAMETERS restartParams_p)
{
    tVEthInstance*   pVEthInstance = (tVEthInstance*) adapterContext_p;
    NDIS_STATUS      status = NDIS_STATUS_SUCCESS;

    DbgPrint("%s()... \n", __FUNCTION__);
    NdisAcquireSpinLock(&pVEthInstance->pauseLock);
    pVEthInstance->miniportPaused = FALSE;
    NdisReleaseSpinLock(&pVEthInstance->pauseLock);
    DbgPrint("%s() - OK \n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Miniport send request handler

Send NET_BUFFER_LISTs to the Kernel layer of stack for ASync scheduling.

\param  adapterContext_p    Pointer to the adapter structure.
\param  netBufferLists_p    Set of NET_BUFFER_LISTs to send.
\param  portNumber_p        A port number that identifies a miniport adapter port.
\param  sendFlags_p         Flags that define attributes for the send operation.

*/
//------------------------------------------------------------------------------
VOID miniportSendNetBufferLists(NDIS_HANDLE adapterContext_p, PNET_BUFFER_LIST netBufferLists_p,
                                NDIS_PORT_NUMBER portNumber_p, ULONG sendFlags_p)
{
    tVEthInstance*      pVEthInstance = (tVEthInstance*) adapterContext_p;
    NDIS_STATUS         status = NDIS_STATUS_SUCCESS;
    PNET_BUFFER_LIST    currentNbl = netBufferLists_p;
    PNET_BUFFER_LIST    returnNbl = NULL;
    PNET_BUFFER_LIST    lastReturnNbl = NULL;
    ULONG               completeFlags = 0;
    PUCHAR              pVethTxData;
    PMDL                pMdl;
    ULONG               totalLength, txLength;
    ULONG               offset = 0;                 // CurrentMdlOffset

    UNREFERENCED_PARAMETER(portNumber_p);

    {
        PNET_BUFFER_LIST    TempNetBufferList;

        for (TempNetBufferList = currentNbl;
             TempNetBufferList != NULL;
             TempNetBufferList = NET_BUFFER_LIST_NEXT_NBL(TempNetBufferList))
        {
            NET_BUFFER_LIST_STATUS(TempNetBufferList) = status;
        }
        if (NDIS_TEST_SEND_AT_DISPATCH_LEVEL(sendFlags_p))
        {
            NDIS_SET_SEND_COMPLETE_FLAG(completeFlags, NDIS_SEND_COMPLETE_FLAGS_DISPATCH_LEVEL);
        }

        NdisMSendNetBufferListsComplete(pVEthInstance->miniportAdapterHandle,
                                        currentNbl,
                                        completeFlags);
    }
    /*
    pVethTxData = NdisAllocateMemoryWithTagPriority(driverInstance_l.pMiniportHandle,
                                                    (OPLK_MAX_FRAME_SIZE),
                                                    OPLK_MEM_TAG, NormalPoolPriority);

    if (pVethTxData == NULL)
    {
        DbgPrint("%s() Failed to allocate memory for VEth Tx frame ", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    while (netBufferLists_p != NULL)
    {
        ULONG                   bytesAvailable = 0;
        PUCHAR                  pRxDataSrc;
        ULONG                   bytesToCopy = 0;

        currentNbl = netBufferLists_p;
        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);
        NET_BUFFER_LIST_NEXT_NBL(currentNbl) = NULL;

        //pVEthInstance->sendRequests++;

        pMdl = NET_BUFFER_CURRENT_MDL(NET_BUFFER_LIST_FIRST_NB(currentNbl));
        txLength = totalLength = NET_BUFFER_DATA_LENGTH(NET_BUFFER_LIST_FIRST_NB(currentNbl));

        if (totalLength > OPLK_MAX_FRAME_SIZE)
        {
            break;
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


            NdisMoveMemory(pVethTxData, pRxDataSrc, bytesToCopy);
            pVethTxData = (PUCHAR) ((ULONG_PTR) pVethTxData + bytesToCopy);
            totalLength -= bytesToCopy;

            offset = 0;
            NdisGetNextMdl(pMdl, &pMdl);
        }

        pVEthInstance->pfnVEthSendCb(pVEthInstance, txLength);

        NET_BUFFER_LIST_STATUS(currentNbl) = status;

        if (NDIS_TEST_SEND_AT_DISPATCH_LEVEL(sendFlags_p))
        {
            NDIS_SET_SEND_COMPLETE_FLAG(completeFlags, NDIS_SEND_COMPLETE_FLAGS_DISPATCH_LEVEL);
        }

        NdisMSendNetBufferListsComplete(pVEthInstance->miniportAdapterHandle,
                                        currentNbl,
                                        completeFlags);
    }

    if (pVethTxData != NULL)
    {
        NdisFreeMemory(pVethTxData, 0, 0);
    }

    */
Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief Miniport receive complete handler

NDIS Miniport entry point called whenever protocols are done with
a packet that we had indicated up and they had queued up for returning later.

\param  adapterContext_p        Pointer to the adapter structure.
\param  netBufferLists_p        A pointer to a linked list of NET_BUFFER_LIST
                                structures that NDIS is returning to the
                                miniport driver.
\param  returnFlags_p           Return flags.

\return The function returns a NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
VOID miniportReturnNetBufferLists(NDIS_HANDLE adapterContext_p,
                                  PNET_BUFFER_LIST netBufferLists_p, ULONG returnFlags_p)
{
}

//------------------------------------------------------------------------------
/**
\brief Miniport cancel send handler

The miniport entry point to hanadle cancellation of all send packets that
match the given CancelId. If we have queued any packets that match this,
then we should dequeue them and call NdisMSendCompleteNetBufferLists for
all such packets, with a status of NDIS_STATUS_REQUEST_ABORTED.

We should also call NdisCancelSendPackets in turn, on each lower binding
that this adapter corresponds to. This is to let miniports below cancel
any matching packets.

\param  adapterContext_p    Pointer to the adapter structure.
\param  cancelId_p          ID of NetBufferLists to be cancelled.

*/
//------------------------------------------------------------------------------
VOID miniportCancelSendNetBufferLists(NDIS_HANDLE adapterContext_p, PVOID cancelId_p)
{
}

//------------------------------------------------------------------------------
/**
\brief Miniport cancel requests handler

The miniport entry point to hanadle cancellation of a request. This function
checks to see if the CancelRequest should be terminated at this level
or passed down to the next driver.

\param  adapterContext_p    Pointer to the adapter structure.
\param  requestId_p         RequestId to be cancelled.

*/
//------------------------------------------------------------------------------
VOID miniportCancelOidRequest(NDIS_HANDLE adapterContext_p, PVOID requestId_p)
{
}

///\}

