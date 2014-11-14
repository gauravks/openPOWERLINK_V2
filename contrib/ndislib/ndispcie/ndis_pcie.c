/**
********************************************************************************
\file   ndis_pcie.c

\brief  NDIS library interface routine for NDIS miniport driver for PCIe

This file implements the initialization routines for NDIS miniport driver for
PCIe card running kernel layer of stack. It also provides helper routines
for memory iniatization, interrupt registration and other communication.

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
    NDIS_MINIPORT_DRIVER_CHARACTERISTICS   miniportChars;
    NDIS_HANDLE                            miniportDriverContext = NULL;

    NdisZeroMemory(&driverInstance_l, sizeof(tNdisDriverInstance));

    NdisZeroMemory(&miniportChars, sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS));
    miniportChars.Header.Type                   = NDIS_OBJECT_TYPE_MINIPORT_DRIVER_CHARACTERISTICS;
    miniportChars.Header.Size                   = sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS);
    miniportChars.Header.Revision               = NDIS_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_1;
    miniportChars.MajorNdisVersion              = OPLK_MAJOR_NDIS_VERSION;
    miniportChars.MinorNdisVersion              = OPLK_MINOR_NDIS_VERSION;
    miniportChars.MajorDriverVersion            = OPLK_MAJOR_DRIVER_VERSION;
    miniportChars.MinorDriverVersion            = OPLK_MINOR_DRIVER_VERSION;

    miniportChars.SetOptionsHandler             = miniportSetOptions;
    miniportChars.InitializeHandlerEx           = miniportInitialize;
    miniportChars.UnloadHandler                 = miniportUnload;
    miniportChars.HaltHandlerEx                 = miniportHalt;
    miniportChars.OidRequestHandler             = miniportOidRequest;
    miniportChars.CancelSendHandler             = miniportCancelSendNetBufferLists;
    miniportChars.DevicePnPEventNotifyHandler   = miniportPnPEventNotify;
    miniportChars.ShutdownHandlerEx             = miniportShutdown;
    miniportChars.CancelOidRequestHandler       = miniportCancelOidRequest;
    // Disable the check for hang timeout so no need for a check for hang handler!
    miniportChars.CheckForHangHandlerEx         = miniportCheckForHang;
    miniportChars.ReturnNetBufferListsHandler   = miniportReturnNetBufferLists;
    miniportChars.SendNetBufferListsHandler     = miniportSendNetBufferLists;
    miniportChars.PauseHandler                  = miniportPause;
    miniportChars.RestartHandler                = miniportRestart;
    miniportChars.ResetHandlerEx                = miniportReset;

    ndisStatus = NdisMRegisterMiniportDriver(pDriverObject_p, pRegistryPath_p, miniportDriverContext,
                                             &miniportChars, &driverInstance_l.pMiniportHandle);
    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Miniport driver registration failed 0x%X\n", __FUNCTION__, ndisStatus);
        return ndisStatus;
    }

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

//------------------------------------------------------------------------------
/**
\brief  Get Bar 0 base address

\return Returns the address of PCIe BAR 0

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
PULONG ndis_getBar0Addr(void)
{
    if (vethInstance_l.state == NdisBindingReady)
    {
        return vethInstance_l.virtualAddrBar0;
    }
    else
    {
        return NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get Bar 1 base address

\return Returns the address of PCIe BAR 1

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
PULONG ndis_getBar1Addr(void)
{
    if (vethInstance_l.state == NdisBindingReady)
    {
        return vethInstance_l.virtualAddrBar1;
    }
    else
    {
        return NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get BAR 2 base address

\return Returns the address of PCIe BAR 2

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
PULONG ndis_getBar2Addr(void)
{
    if (vethInstance_l.state == NdisBindingReady)
    {
        return vethInstance_l.virtualAddrBar2;
    }
    else
    {
        return NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Register synchronization handler

Register application specific synchronization routine called from the ISR.

\param  pfnSyncCb_p     Pointer to synchronization routine.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_registerSyncHandler(tSyncHandler pfnSyncCb_p)
{
    vethInstance_l.pfnSyncCb = pfnSyncCb_p;
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

///\}

