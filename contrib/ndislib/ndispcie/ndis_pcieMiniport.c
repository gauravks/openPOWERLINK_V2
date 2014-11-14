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
#include "ndisDriver.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
//#define READ_WRITE_PROTO

#ifdef READ_WRITE_PROTO
#define OCM_OFFSET          0x00004000
#define SSRAM_OFFSET        0x0c400000
#endif
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

tVEthInstance     vethInstance_l;
static BOOLEAN    fInitialize = FALSE;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
NDIS_STATUS     prepareHardware(PNDIS_RESOURCE_LIST resourceList_p);
void            releaseHardware(void);
BOOLEAN         interruptHandler(NDIS_HANDLE  interruptContext_p, ULONG messageId_p,
                         PBOOLEAN queueDefaultInterruptDpc_p, PULONG targetProcessors_p);
VOID            interruptDpc(NDIS_HANDLE  interruptContext_p, ULONG messageId_p,
                  PVOID dpcContext_p, PULONG reserved1_p, PULONG reserved2_p);
#ifdef READ_WRITE_PROTO
void            readWriteTest(void);
#endif
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
    NDIS_STATUS                                     status = NDIS_STATUS_SUCCESS;
    NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS         intrChars;
    NDIS_MINIPORT_ADAPTER_ATTRIBUTES                miniportAttributes;
    NDIS_PNP_CAPABILITIES                           pnpCaps;

    UNREFERENCED_PARAMETER(driverContext_p);

    TRACE("%s()... \n", __FUNCTION__);

    NdisZeroMemory(&miniportAttributes, sizeof(NDIS_MINIPORT_ADAPTER_ATTRIBUTES));

    NdisZeroMemory(&vethInstance_l, sizeof(tVEthInstance));
    // Register IOCTL interface here
#ifndef READ_WRITE_PROTO
    ndis_createAppIntf();
#endif

    // TODO: Initialize the miniport resources here
    //
    NdisAllocateSpinLock(&vethInstance_l.pauseLock);

    vethInstance_l.miniportAdapterHandle = adapterHandle_p;

    miniportAttributes.RegistrationAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES;
    miniportAttributes.RegistrationAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1;
    miniportAttributes.RegistrationAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES);

    miniportAttributes.RegistrationAttributes.MiniportAdapterContext = (NDIS_HANDLE)&vethInstance_l;
    miniportAttributes.RegistrationAttributes.AttributeFlags = NDIS_MINIPORT_ATTRIBUTES_HARDWARE_DEVICE;
    miniportAttributes.RegistrationAttributes.CheckForHangTimeInSeconds = 4;
    miniportAttributes.RegistrationAttributes.InterfaceType = NdisInterfacePci;

    status = NdisMSetMiniportAttributes(adapterHandle_p, &miniportAttributes);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Miniport registration atrribute configuration failed\n");
        goto ExitFail;
    }

    miniportAttributes.GeneralAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES;
    miniportAttributes.GeneralAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1;
    miniportAttributes.GeneralAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES);

    miniportAttributes.GeneralAttributes.MediaType = NdisMedium802_3;
    miniportAttributes.GeneralAttributes.MtuSize = OPLK_MTU_SIZE;
    miniportAttributes.GeneralAttributes.MaxXmitLinkSpeed = OPLK_LINK_SPEED;
    miniportAttributes.GeneralAttributes.MaxRcvLinkSpeed = OPLK_LINK_SPEED;
    miniportAttributes.GeneralAttributes.XmitLinkSpeed = OPLK_LINK_SPEED;
    miniportAttributes.GeneralAttributes.RcvLinkSpeed = OPLK_LINK_SPEED;
    miniportAttributes.GeneralAttributes.MediaConnectState = MediaConnectStateUnknown;
    miniportAttributes.GeneralAttributes.MediaDuplexState = MediaDuplexStateHalf;
    miniportAttributes.GeneralAttributes.LookaheadSize = OPLK_LINK_SPEED;

    miniportAttributes.GeneralAttributes.PowerManagementCapabilities = NULL;
    miniportAttributes.GeneralAttributes.MacOptions = NDIS_MAC_OPTION_TRANSFERS_NOT_PEND |
                                                      NDIS_MAC_OPTION_NO_LOOPBACK;

    miniportAttributes.GeneralAttributes.SupportedPacketFilters = NDIS_PACKET_TYPE_DIRECTED |
                                                                  NDIS_PACKET_TYPE_MULTICAST |
                                                                  NDIS_PACKET_TYPE_ALL_MULTICAST |
                                                                  NDIS_PACKET_TYPE_BROADCAST;
    miniportAttributes.GeneralAttributes.MaxMulticastListSize = 32;
    miniportAttributes.GeneralAttributes.MacAddressLength = ETH_LENGTH_OF_ADDRESS;
    miniportAttributes.GeneralAttributes.PhysicalMediumType = NdisPhysicalMedium802_3;
    miniportAttributes.GeneralAttributes.RecvScaleCapabilities = NULL;
    miniportAttributes.GeneralAttributes.AccessType = NET_IF_ACCESS_BROADCAST;
    miniportAttributes.GeneralAttributes.DirectionType = NET_IF_DIRECTION_SENDRECEIVE;
    miniportAttributes.GeneralAttributes.ConnectionType = NET_IF_CONNECTION_DEDICATED;
    miniportAttributes.GeneralAttributes.IfType = IF_TYPE_ETHERNET_CSMACD;
    miniportAttributes.GeneralAttributes.IfConnectorPresent = TRUE;
    miniportAttributes.GeneralAttributes.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_OK_SUPPORTED;
    miniportAttributes.GeneralAttributes.SupportedPauseFunctions = NdisPauseFunctionsUnsupported;
    miniportAttributes.GeneralAttributes.AutoNegotiationFlags = NDIS_LINK_STATE_DUPLEX_AUTO_NEGOTIATED;
    miniportAttributes.GeneralAttributes.SupportedOidList = VEthSupportedOids;
    miniportAttributes.GeneralAttributes.SupportedOidListLength = sizeof(VEthSupportedOids);

    status = NdisMSetMiniportAttributes(adapterHandle_p, &miniportAttributes);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() General attribute registration failed\n");
        goto ExitFail;
    }

    // Register Interrupt
    NdisZeroMemory(&intrChars, sizeof(NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS));

    intrChars.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_INTERRUPT;
    intrChars.Header.Revision = NDIS_MINIPORT_INTERRUPT_REVISION_1;
    intrChars.Header.Size = sizeof(NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS);;
    intrChars.MsiSyncWithAllMessages = TRUE;
    intrChars.MessageInterruptDpcHandler = interruptDpc;
    intrChars.MessageInterruptHandler = interruptHandler;
    intrChars.EnableMessageInterruptHandler = NULL;
    intrChars.DisableMessageInterruptHandler = NULL;

    status = NdisMRegisterInterruptEx(adapterHandle_p, &vethInstance_l, &intrChars,
                                      &vethInstance_l.interruptHandle);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Failed to register interrupt\n", __FUNCTION__);
        goto ExitFail;
    }

    vethInstance_l.interruptType = intrChars.InterruptType;
    if (intrChars.InterruptType == NDIS_CONNECT_MESSAGE_BASED)
    {
        vethInstance_l.intrMsgInfo = intrChars.MessageInfoTable;
    }

    status = prepareHardware(initParams_p->AllocatedResources);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Failed to parse resources\n", __FUNCTION__);
        status = NDIS_STATUS_FAILURE;
        goto ExitFail;
    }

    vethInstance_l.state = NdisBindingReady;

#ifdef READ_WRITE_PROTO
    readWriteTest();
#endif
    goto Exit;

ExitFail:

    if (status != NDIS_STATUS_SUCCESS)
    {
        releaseHardware();
    }
    

Exit:
    TRACE("%s() Return with status 0x%X\n", __FUNCTION__, status);
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
    TRACE("%s()...\n", __FUNCTION__);

    vethInstance_l.miniportHalting = TRUE;

    releaseHardware();
#ifndef READ_WRITE_PROTO
    ndis_closeAppIntf();
#endif
    vethInstance_l.miniportAdapterHandle = NULL;

    TRACE("%s() - OK\n", __FUNCTION__);
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
    TRACE("%s()...\n", __FUNCTION__);
    UNREFERENCED_PARAMETER(driverObject_p);

    NdisMDeregisterMiniportDriver(driverInstance_l.pMiniportHandle);

    TRACE("%s() - OK\n", __FUNCTION__);
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

    TRACE("%s()...\n", __FUNCTION__);
    NdisAcquireSpinLock(&pVEthInstance->pauseLock);
    pVEthInstance->miniportPaused = TRUE;
    NdisReleaseSpinLock(&pVEthInstance->pauseLock);

    TRACE("%s() - OK\n", __FUNCTION__);
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

    TRACE("%s()... \n", __FUNCTION__);
    NdisAcquireSpinLock(&pVEthInstance->pauseLock);
    pVEthInstance->miniportPaused = FALSE;
    NdisReleaseSpinLock(&pVEthInstance->pauseLock);
    TRACE("%s() - OK \n", __FUNCTION__);
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
    // TODO: Veth Intergration to be done later
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

//------------------------------------------------------------------------------
/**
\brief 

\param  adapterContext_p    Pointer to the adapter structure.

*/
//------------------------------------------------------------------------------
BOOLEAN miniportCheckForHang(NDIS_HANDLE adapterContext_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief

\param  adapterContext_p    Pointer to the adapter structure.
\param  addressingReset_p   
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportReset(NDIS_HANDLE adapterContext_p, PBOOLEAN addressingReset_p)
{

    NDIS_STATUS ndisStatus = NDIS_STATUS_SUCCESS;
    TRACE("%s() ....");
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(addressingReset_p);
    TRACE("OK\n");
    return(ndisStatus);
}
//------------------------------------------------------------------------------
/**
\brief Prepare hardware resources

This routine parses the hardware resources such as memory, interrupt etc
assigned by the OS during device enumeration and maps it into system space.


\param  resourceList_p    Pointer to resources allocated for device.

*/
//------------------------------------------------------------------------------
NDIS_STATUS prepareHardware(PNDIS_RESOURCE_LIST resourceList_p)
{
    NDIS_STATUS                         status = NDIS_STATUS_SUCCESS;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR     pResDescriptor;
    ULONG                               resCount;
    ULONG                               barCount = 0;

    PAGED_CODE();

    // Scan through the allocated resources and store them in device context
    for (resCount = 0; resCount < resourceList_p->Count; resCount++)
    {
        pResDescriptor = &resourceList_p->PartialDescriptors[resCount];

        if (pResDescriptor == NULL)
        {
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }

        switch (pResDescriptor->Type)
        {
            case CmResourceTypePort:
            {
                 // Device Doesnt have any I/O Port Resources so Ignore This
                 break;
            }
            case CmResourceTypeMemory:
            {
                switch (barCount)
                {
                    case 0:
                    {
                        // First BAR found, map the memory and store the details
                        if (pResDescriptor->u.Memory.Length <= 0)
                            break;

                        barCount++;
                        vethInstance_l.phyAddrBar0 = pResDescriptor->u.Memory.Start;
                        status = NdisMMapIoSpace(&vethInstance_l.virtualAddrBar0,
                                                vethInstance_l.miniportAdapterHandle,
                                                pResDescriptor->u.Memory.Start,
                                                pResDescriptor->u.Memory.Length);

                        if (status != NDIS_STATUS_SUCCESS)
                        {
                            TRACE("%s() BAR 0 not mapped\n", __FUNCTION__);
                            goto Exit;
                        }

                        vethInstance_l.lengthBar0 = pResDescriptor->u.Memory.Length;

                        TRACE("BAR 0 PhyAddr:%x VirtAddr: %x Size %d\n",
                                        vethInstance_l.phyAddrBar0,
                                        vethInstance_l.virtualAddrBar0,
                                        vethInstance_l.lengthBar0);
                        break;
                    }
                    case 1:
                    {
                        // Second BAR Found
                        if (pResDescriptor->u.Memory.Length <= 0)
                            break;

                        barCount++;
                        vethInstance_l.phyAddrBar1 = pResDescriptor->u.Memory.Start;
                        status = NdisMMapIoSpace(&vethInstance_l.virtualAddrBar1,
                                                vethInstance_l.miniportAdapterHandle,
                                                pResDescriptor->u.Memory.Start,
                                                pResDescriptor->u.Memory.Length);

                        if (status != NDIS_STATUS_SUCCESS)
                        {
                            TRACE("%s() BAR 1 not mapped\n", __FUNCTION__);
                            goto Exit;
                        }

                        vethInstance_l.lengthBar1 = pResDescriptor->u.Memory.Length;

                        TRACE("BAR 1 PhyAddr:%x VirtAddr: %x Size %d\n",
                                            vethInstance_l.phyAddrBar1,
                                            vethInstance_l.virtualAddrBar1,
                                            vethInstance_l.lengthBar1);
                        break;
                    }
                    case 2:
                    {
                        // Third BAR Found
                        if (pResDescriptor->u.Memory.Length <= 0)
                            break;

                        barCount++;
                        vethInstance_l.phyAddrBar2 = pResDescriptor->u.Memory.Start;
                        status = NdisMMapIoSpace(&vethInstance_l.virtualAddrBar2,
                                                vethInstance_l.miniportAdapterHandle,
                                                pResDescriptor->u.Memory.Start,
                                                pResDescriptor->u.Memory.Length);

                        if (status != NDIS_STATUS_SUCCESS)
                        {
                            TRACE("%s() BAR 2 not mapped\n", __FUNCTION__);
                            goto Exit;
                        }

                        vethInstance_l.lengthBar2 = pResDescriptor->u.Memory.Length;

                        TRACE("BAR 2 PhyAddr:%x VirtAddr: %x Size %d\n",
                                vethInstance_l.phyAddrBar2,
                                vethInstance_l.virtualAddrBar2,
                                vethInstance_l.lengthBar2);

                        break;
                    }
                    default:
                    {
                        // Only Three BARs available.
                        break;
                    }
                }
            }
            case CmResourceTypeInterrupt:
            {
                if (pResDescriptor->Flags & CM_RESOURCE_INTERRUPT_MESSAGE)
                {
                    vethInstance_l.msiVector = pResDescriptor->u.MessageInterrupt.Translated.Vector;
                }

                break;
            }
        }
    }

Exit:
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Release hardware resources

This routine frees resources allocated during intialization

*/
//------------------------------------------------------------------------------
void releaseHardware(void)
{
    vethInstance_l.state = NdisBindingPausing;
    if (vethInstance_l.virtualAddrBar0)
    {
        NdisMUnmapIoSpace(vethInstance_l.miniportAdapterHandle, vethInstance_l.virtualAddrBar0,
                          vethInstance_l.lengthBar0);
        vethInstance_l.virtualAddrBar0 = NULL;
    }

    if (vethInstance_l.virtualAddrBar1)
    {
        NdisMUnmapIoSpace(vethInstance_l.miniportAdapterHandle, vethInstance_l.virtualAddrBar1,
                          vethInstance_l.lengthBar1);
        vethInstance_l.virtualAddrBar1 = NULL;
    }

    // TODO release other resources
    if (vethInstance_l.interruptHandle)
    {
        NdisMDeregisterInterruptEx(vethInstance_l.interruptHandle);
        vethInstance_l.interruptHandle = NULL;
    }

    NdisFreeSpinLock(&vethInstance_l.pauseLock);
}

//------------------------------------------------------------------------------
/**
\brief Interrupt service routine

\param  interruptContext_p          Pointer to context memory of the driver.
\param  messageId_p                 Message ID of the interrupt.
\param  queueDefaultInterruptDpc_p  Flag to invoke DPC.
\param  targetProcessors_p          Target processor interrupted.

\return Returns TRUE if the interrupt is handled else FALSE.

*/
//------------------------------------------------------------------------------
BOOLEAN interruptHandler(NDIS_HANDLE  interruptContext_p, ULONG messageId_p,
                         PBOOLEAN queueDefaultInterruptDpc_p, PULONG targetProcessors_p)
{

    UNREFERENCED_PARAMETER(interruptContext_p);
    UNREFERENCED_PARAMETER(messageId_p);
    UNREFERENCED_PARAMETER(targetProcessors_p);

    *queueDefaultInterruptDpc_p = TRUE;

    return TRUE;
}

//------------------------------------------------------------------------------
/**
\brief Interrupt DPC routine


\param  resourceList_p    Pointer to resources allocated for device.

*/
//------------------------------------------------------------------------------
VOID interruptDpc(NDIS_HANDLE  interruptContext_p, ULONG messageId_p,
                  PVOID dpcContext_p, PULONG reserved1_p, PULONG reserved2_p)
{
    UNREFERENCED_PARAMETER(interruptContext_p);
    UNREFERENCED_PARAMETER(messageId_p);
    UNREFERENCED_PARAMETER(dpcContext_p);
    UNREFERENCED_PARAMETER(reserved1_p);
    UNREFERENCED_PARAMETER(reserved2_p);

    if (vethInstance_l.pfnSyncCb != NULL)
    {
        vethInstance_l.pfnSyncCb();
    }
}

#ifdef READ_WRITE_PROTO
void readWriteTest(void)
{
    UCHAR*      ocmBase = (UCHAR*)vethInstance_l.virtualAddrBar0 + OCM_OFFSET;
    UCHAR*      ssramBase = (UCHAR*) vethInstance_l.virtualAddrBar2 + SSRAM_OFFSET;
    UCHAR*      shareMemory = ssramBase + *((UINT*) ocmBase);
    UCHAR*      handshakeMemory = ssramBase + *((UINT*) (ocmBase + 4));
    int         index = 0;
    int count = 0;

    if (shareMemory == NULL || handshakeMemory == NULL)
    {
        DbgPrint("Error in initilaization\n");
        return;
    }
    DbgPrint("Handshake %p %x Sharedmemory %p &x\n", handshakeMemory, *handshakeMemory, shareMemory, shareMemory[1]);

    while (1)
    {
        while (1)
        {
            if (!(*handshakeMemory))
            {
                DbgPrint("%p %x\n", shareMemory, shareMemory[1]);
                break;
            }
            else
            {
                DbgPrint("%x-%d\n", *handshakeMemory, count);
            }
        }

        shareMemory[0] = count;

        count++;
        if (count == 256)
            break;

        *handshakeMemory = 1;
    }
/*    // Read and Write OCM memmory
    DbgPrint(" Begin OCM read Write at base %p\n", ocmBase);

    for (index = 0; index < 256; index++)
    {
        ocmBase[index] = (UCHAR)index;
        DbgPrint("Index %d Address %p Value %x\n", index, &ocmBase[index], ocmBase[index]);
    }

    DbgPrint(" Begin SSRAM read Write at base %p\n", ssramBase);

    for (index = 0; index < 256; index++)
    {
        ssramBase[index] = (UCHAR)index;
        DbgPrint("Index %d Address %p Value %x\n", index, &ssramBase[index], ssramBase[index]);
    }*/

}

#endif
///\}

