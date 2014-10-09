/**
********************************************************************************
\file   ndis_intermediate.c

\brief  Driver initialization and dispatch functions for the NDIS Intermediate driver

This file implements the initialization routines for NDIS intermediate drivers
and provides helper routines which can be used to access, modify, send and
receive frames.

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

#include "ndisdriver.h"
#include <ndis.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_MAJOR_NDIS_VERSION         6
#define OPLK_MINOR_NDIS_VERSION         0

#define OPLK_MAJOR_DRIVER_VERSION       3
#define OPLK_MINOR_DRIVER_VERSION       0

#define OPLK_PROT_MAJOR_NDIS_VERSION    6
#define OPLK_PROT_MINOR_NDIS_VERSION    0
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
tNdisDriverInstance    driverInstance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Initialization routine for NDIS driver

This routine registers the NDIS protocol/miniport characteristics and entry
routines to OS using NdisXRegisterXXXDriver.

\param  pDriverObject_p      Pointer to the system's driver object structure
                            for this driver.
\param  pRegistryPath_p     System's registry path for this driver.

\return The function returns a NDIS_STATUS error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_STATUS ndis_initDriver(PDRIVER_OBJECT pDriverObject_p, PUNICODE_STRING pRegistryPath_p)
{
    NDIS_STATUS                            ndisStatus = NDIS_STATUS_SUCCESS;
    NDIS_PROTOCOL_DRIVER_CHARACTERISTICS   protocolChars;
    NDIS_MINIPORT_DRIVER_CHARACTERISTICS   miniportChars;
    NDIS_HANDLE                            miniportDriverContext = NULL;
    NDIS_HANDLE                            protocolDriverContext = NULL;
    NDIS_STRING                            ndisDriverName;

    NdisZeroMemory(&driverInstance_l, sizeof(tNdisDriverInstance));

    NdisZeroMemory(&miniportChars, sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS));
    miniportChars.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    miniportChars.Header.Size = sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS);
    miniportChars.Header.Revision = NDIS_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_1;
    miniportChars.MajorNdisVersion = OPLK_MAJOR_NDIS_VERSION;
    miniportChars.MinorNdisVersion = OPLK_MINOR_NDIS_VERSION;
    miniportChars.MajorDriverVersion = OPLK_MAJOR_DRIVER_VERSION;
    miniportChars.MinorDriverVersion = OPLK_MINOR_DRIVER_VERSION;

    miniportChars.SetOptionsHandler = miniportSetOptions;
    miniportChars.InitializeHandlerEx = miniportInitialize;
    miniportChars.UnloadHandler = miniportUnload;
    miniportChars.HaltHandlerEx = miniportHalt;

    miniportChars.OidRequestHandler = miniportOidRequest;

    miniportChars.CancelSendHandler = miniportCancelSendNetBufferLists;
    miniportChars.DevicePnPEventNotifyHandler = miniportPnPEventNotify;
    miniportChars.ShutdownHandlerEx = miniportShutdown;
    miniportChars.CancelOidRequestHandler = miniportCancelOidRequest;

    // Disable the check for hang timeout so no need for a check for hang handler!
    miniportChars.CheckForHangHandlerEx = NULL;

    miniportChars.ReturnNetBufferListsHandler = miniportReturnNetBufferLists;
    miniportChars.SendNetBufferListsHandler = miniportSendNetBufferLists;

    miniportChars.PauseHandler = miniportPause;
    miniportChars.RestartHandler = miniportRestart;

    miniportChars.Flags = NDIS_INTERMEDIATE_DRIVER;

    ndisStatus = NdisMRegisterMiniportDriver(pDriverObject_p, pRegistryPath_p, miniportDriverContext,
                                             &miniportChars, &driverInstance_l.pMiniportHandle);
    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        DbgPrint("%s() Miniport driver registration failed 0x%X\n", __FUNCTION__, ndisStatus);
        return ndisStatus;
    }

    NdisZeroMemory(&protocolChars, sizeof(NDIS_PROTOCOL_DRIVER_CHARACTERISTICS));

    protocolChars.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    protocolChars.Header.Size = sizeof(NDIS_PROTOCOL_DRIVER_CHARACTERISTICS);
    protocolChars.Header.Revision = NDIS_PROTOCOL_DRIVER_CHARACTERISTICS_REVISION_1;
    protocolChars.MajorNdisVersion = OPLK_PROT_MAJOR_NDIS_VERSION;
    protocolChars.MinorNdisVersion = OPLK_PROT_MINOR_NDIS_VERSION;

    protocolChars.MajorDriverVersion = OPLK_MAJOR_DRIVER_VERSION;
    protocolChars.MinorDriverVersion = OPLK_MINOR_DRIVER_VERSION;

    protocolChars.SetOptionsHandler = protocolSetOptions;

    NdisInitUnicodeString(&ndisDriverName, L"MUXP");    // Protocol name
    protocolChars.Name = ndisDriverName;
    protocolChars.OpenAdapterCompleteHandlerEx = protocolOpenAdapterComplete;
    protocolChars.CloseAdapterCompleteHandlerEx = protocolCloseAdapterComplete;

    protocolChars.ReceiveNetBufferListsHandler = protocolReceiveNbl;
    protocolChars.SendNetBufferListsCompleteHandler = protocolSendNblComplete;
    protocolChars.OidRequestCompleteHandler = protocolRequestComplete;
    protocolChars.StatusHandlerEx = protocolStatus;
    protocolChars.BindAdapterHandlerEx = protocolBindAdapter;
    protocolChars.UnbindAdapterHandlerEx = protocolUnbindAdapter;
    protocolChars.NetPnPEventHandler = protocolPnpHandler;

    ndisStatus = NdisRegisterProtocolDriver(protocolDriverContext, &protocolChars,
                                            &driverInstance_l.pProtocolHandle);

    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        DbgPrint("%s() Protocol driver registration failed 0x%X\n", __FUNCTION__, ndisStatus);
        NdisMDeregisterMiniportDriver(driverInstance_l.pMiniportHandle);
        return ndisStatus;
    }

    // Create association between protocol and miniport driver
    NdisIMAssociateMiniport(driverInstance_l.pMiniportHandle, driverInstance_l.pProtocolHandle);

    return ndisStatus;
}

//------------------------------------------------------------------------------
/**
\brief  Get miniport driver handle

Get miniport driver handle returned by OS during miniport registration for this
driver.

\return The function returns miniport adapter handle.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_HANDLE ndis_getAdapterHandle(void)
{
    return driverInstance_l.pMiniportHandle;
}

//------------------------------------------------------------------------------
/**
\brief  Get Mac address for the device

Get MAC address of the ethernet controller.

\param pMac_p       Pointer to MAC address.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_getMacAddress(UCHAR*  pMac_p)
{
    UCHAR*   currentMac;
    currentMac = protocol_getCurrentMac();

    if (currentMac != NULL)
        NdisMoveMemory(pMac_p, currentMac, ETH_LENGTH_OF_ADDRESS);
}

//------------------------------------------------------------------------------
/**
\brief  Register application interface routines

Register routines for IOCTL interface registration and de-registration. NDIS
driver calls the registration routine from miniport initialization callback,
and de-registration from miniport halt callback.

\param pMac_p       Pointer to MAC address.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_registerAppIntf(tAppIntfRegister pAppIntfRegCb_p, tAppIntfDeRegister pAppIntfDeregCb_p)
{
    driverInstance_l.pfnAppIntfRegCb = pAppIntfRegCb_p;
    driverInstance_l.pfnAppIntfDeregisterCb = pAppIntfDeregCb_p;
}

//------------------------------------------------------------------------------
/**
\brief  Check driver binding state

Check the current state of lower edge binding of protocol.

\return Returns TRUE if ready else FALSE

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
BOOLEAN ndis_checkBindingState(void)
{
    return protocol_checkBindingState();
}

//------------------------------------------------------------------------------
/**
\brief  Set binding state for the protocol driver

Set the status of the protocol binding.

\param state_p  State to set

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_setBindingState(ULONG state_p)
{
    protocol_setBindingState(state_p);
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx and Rx buffers

This routines invokes the specific Tx and Rx buffers allocation routine of the
NDIS driver to initialize receive queue and transmit queue.

\param  txBuffCount_p       Tx buffer count.
\param  rxBuffCount_p       Rx buffer count.

\return Returns tNdisErrorStatus error code.
\retval NdisStatusSuccess If buffers allocated successfully.
\retval NdisStatusResources If some error occurred in allocation.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
tNdisErrorStatus ndis_allocateTxRxBuff(UINT txBuffCount_p, UINT rxBuffCount_p)
{
    NDIS_STATUS status;
    status = protocol_allocateTxRxBuf(txBuffCount_p, rxBuffCount_p);

    if (status != NDIS_STATUS_SUCCESS)
        return NdisStatusResources;

    return NdisStatusSuccess;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx and Rx buffer

\return Returns tNdisErrorStatus error code

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_freeTxRxBuff(void)
{
    protocol_freeTxRxBuffers();
}

//------------------------------------------------------------------------------
/**
\brief  Get Tx buffer

This routines gets a Tx buffer to be shared with the caller from the NDIS driver.

\param  pData_p      Pointer to buffer.
\param  size_p       Size of the buffer.
\param  pTxLink_p    Pointer to LIST_ENTRY to track the Tx buffer in next calls

\return Returns tNdisErrorStatus error code

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
tNdisErrorStatus ndis_getTxBuff(void** ppData_p, size_t size_p, void** ppTxLink_p)
{
    tTxBufInfo*   pTxBuffInfo = protocol_getTxBuff(size_p);

    if (pTxBuffInfo == NULL)
    {
        return NdisStatusResources;
    }

    *ppData_p = pTxBuffInfo->pData;
    *ppTxLink_p = (void*) &pTxBuffInfo->txLink;

    return NdisStatusSuccess;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This routines frees the previously allocated Tx buffer shared with the caller

\param  pTxLink_p      Pointer to LIST_ENTRY of the Txbuffer

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_freeTxBuff(void* pTxLink_p)
{
    protocol_freeTxBuff(pTxLink_p);
}

//------------------------------------------------------------------------------
/**
\brief  Send packet

Send a packet

\param  pData_p      Pointer to data buffer.
\param  size_p       Size of the packet to send.
\param  pTxLink_p    Pointer to link list entry for the Tx buffer.

\return Returns tNdisErrorStatus error code

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
tNdisErrorStatus ndis_sendPacket(void* pData_p, size_t size_p, void* pTxLink_p)
{
    NDIS_STATUS    ndisStatus;

    if (pData_p == NULL || pTxLink_p == NULL)
    {
        return NdisStatusInvalidParams;
    }

    ndisStatus = protocol_sendPacket(pData_p, size_p, pTxLink_p);

    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        return NdisStatusTxError;
    }

    return NdisStatusSuccess;
}

//------------------------------------------------------------------------------
/**
\brief  Register Tx and Rx callback

Register Tx and Rx callbacks to NDIS driver.

\param  pfnTxCallback_p      Pointer to Tx callback routine.
\param  pfnRxCallback_p      Pointer to Rx callback routine.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_registerTxRxHandler(tNdisTransmitCompleteCb pfnTxCallback_p,
                              tNdisReceiveCb pfnRxCallback_p)
{
    protocol_registerTxRxHandler(pfnTxCallback_p, pfnRxCallback_p);
}

//------------------------------------------------------------------------------
/**
\brief  Create Application interface device

This routines calls the IOCTL interface registration routine which initializes
a IOCTL interface for user application to interact with the driver.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_createAppIntf(void)
{
    driverInstance_l.pfnAppIntfRegCb(driverInstance_l.pMiniportHandle);
}

//------------------------------------------------------------------------------
/**
\brief  Close Application interface device

Close the IOCTL interface created before.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_closeAppIntf(void)
{
    driverInstance_l.pfnAppIntfDeregisterCb();
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Miniport set options routine for NDIS driver

This routine registers the optional handlers for the miniport section of driver
with NDIS.

\param  driverHandle_p      Miniport driver handle.
\param  driverContext_p     Specifies a handle to a driver-allocated context area
                            where the driver maintains state and configuration
                            information.

                            NOTE: Nothing to be done in this routine now.

\return The function returns a NDIS_STATUS error code.

*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportSetOptions(NDIS_HANDLE driverHandle_p, NDIS_HANDLE driverContext_p)
{
    UNREFERENCED_PARAMETER(driverHandle_p);
    UNREFERENCED_PARAMETER(driverContext_p);
    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  Protocol set options routine for NDIS driver

This routine registers the optional handlers for the protocol section of driver
with NDIS.

\param  driverHandle_p      Protocol driver handle.
\param  driverContext_p     Specifies a handle to a driver-allocated context area
                            where the driver maintains state and configuration
                            information.

                            NOTE: Nothing to be done in this routine now.

\return The function returns a NDIS_STATUS error code.

*/
//------------------------------------------------------------------------------
NDIS_STATUS protocolSetOptions(NDIS_HANDLE driverHandle_p, NDIS_HANDLE driverContext_p)
{
    UNREFERENCED_PARAMETER(driverHandle_p);
    UNREFERENCED_PARAMETER(driverContext_p);
    return NDIS_STATUS_SUCCESS;
}

///\}

