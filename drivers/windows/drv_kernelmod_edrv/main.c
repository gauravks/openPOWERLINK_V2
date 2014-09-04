/**
********************************************************************************
\file   main.c

\brief  main file for Windows kernel module

This file contains the main part of the Windows kernel module implementation of
the openPOWERLINK kernel stack.

\ingroup module_driver_win_kernel
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
#include <ntddk.h>

#include <oplk/oplk.h>

#include <common/driver.h>
#include <common/ctrl.h>
#include <common/ctrlcal-mem.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <errhndkcal.h>
#include <ndis-intf.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_MEM_TAG     'klpO'
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
NDIS_HANDLE     heartbeatTimer_g;
//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------
#pragma NDIS_INIT_FUNCTION(DriverEntry)
//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    //
    // Lock to rundown threads that are dispatching I/Os on a file handle
    // while the cleanup for that handle is in progress.
    //
    IO_REMOVE_LOCK  driverAccessLock;
} tFileContext;

typedef struct
{
    PDEVICE_OBJECT      pAppDeviceObject;
    NDIS_HANDLE         pAppDeviceHandle;
    NDIS_HANDLE         driverHandle;
    LIST_ENTRY          eventQueue;
    NDIS_SPIN_LOCK      eventQueueLock;
    LIST_ENTRY          pdoSyncQueue;
    NDIS_SPIN_LOCK      pdoSyncLock;
}tPlkDriverInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tPlkDriverInstance plkDriverInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void registerAppIntf(NDIS_HANDLE driverHandle_p);
static void deRegisterAppIntf(void);
DRIVER_DISPATCH powerlinkCreate;
DRIVER_DISPATCH powerlinkCleanup;
DRIVER_DISPATCH powerlinkClose;
DRIVER_DISPATCH powerlinkIoctl;

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//---------------------------------------------------------------------------
//  Initailize driver
//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
/**
\brief  Driver initialization routine

The function implements openPOWERLINK kernel module initialization function.

\ingroup module_driver_windows_kernel
*/
//------------------------------------------------------------------------------
NTSTATUS DriverEntry(PDRIVER_OBJECT driverObject_p, PUNICODE_STRING registryPath_p)
{
    NDIS_STATUS ndisStatus;

    ndisStatus = ndis_initDriver(driverObject_p, registryPath_p);

    if(ndisStatus != NDIS_STATUS_SUCCESS)
    {
        DbgPrint("%s() Failed to initialize driver 0x%X\n", __FUNCTION__, ndisStatus);
    }

    // register application interface handlers
    ndis_registerAppIntf(registerAppIntf, deRegisterAppIntf);
    return ndisStatus;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

static void registerAppIntf(NDIS_HANDLE driverHandle_p)
{
    NDIS_STATUS                     status = NDIS_STATUS_SUCCESS;
    UNICODE_STRING                  deviceName;
    UNICODE_STRING                  deviceLinkUnicodeString;
    NDIS_DEVICE_OBJECT_ATTRIBUTES   deviceObjectAttributes;
    PDRIVER_DISPATCH                dispatchTable[IRP_MJ_MAXIMUM_FUNCTION+1];

    plkDriverInstance_l.driverHandle = driverHandle_p;
    NdisZeroMemory(dispatchTable, (IRP_MJ_MAXIMUM_FUNCTION + 1) * sizeof(PDRIVER_DISPATCH));

    dispatchTable[IRP_MJ_CREATE] = powerlinkCreate;
    dispatchTable[IRP_MJ_CLEANUP] = powerlinkCleanup;
    dispatchTable[IRP_MJ_CLOSE] = powerlinkClose;
    dispatchTable[IRP_MJ_DEVICE_CONTROL] = powerlinkIoctl;

    NdisInitUnicodeString(&deviceName, PLK_DEV_STRING);
    NdisInitUnicodeString(&deviceLinkUnicodeString, PLK_LINK_NAME);

    NdisZeroMemory(&deviceObjectAttributes, sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES));

    deviceObjectAttributes.Header.Type = NDIS_OBJECT_TYPE_DEFAULT; // type implicit from the context
    deviceObjectAttributes.Header.Revision = NDIS_DEVICE_OBJECT_ATTRIBUTES_REVISION_1;
    deviceObjectAttributes.Header.Size = sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES);
    deviceObjectAttributes.DeviceName = &deviceName;
    deviceObjectAttributes.SymbolicName = &deviceLinkUnicodeString;
    deviceObjectAttributes.MajorFunctions = &dispatchTable[0];
    deviceObjectAttributes.ExtensionSize = 0;
    deviceObjectAttributes.DefaultSDDLString = NULL;
    deviceObjectAttributes.DeviceClassGuid = 0;

    status = NdisRegisterDeviceEx(driverHandle_p,
                                  &deviceObjectAttributes,
                                  &plkDriverInstance_l.pAppDeviceObject,
                                  &plkDriverInstance_l.pAppDeviceHandle);

    plkDriverInstance_l.pAppDeviceObject->Flags |= DO_BUFFERED_IO;

    NdisInitializeListHead(&plkDriverInstance_l.eventQueue);
    NdisInitializeListHead(&plkDriverInstance_l.pdoSyncQueue);


    NdisAllocateSpinLock(&plkDriverInstance_l.eventQueueLock);
    NdisAllocateSpinLock(&plkDriverInstance_l.pdoSyncLock);

}

static void deRegisterAppIntf(void)
{
    NdisFreeSpinLock(&plkDriverInstance_l.eventQueueLock);
    NdisFreeSpinLock(&plkDriverInstance_l.pdoSyncLock);

    if (plkDriverInstance_l.pAppDeviceHandle != NULL)
    {
        NdisDeregisterDeviceEx(plkDriverInstance_l.pAppDeviceHandle);
    }
}
//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver create function

The function implements openPOWERLINK kernel module open function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCreate(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    NDIS_TIMER_CHARACTERISTICS        timerChars;
    tFileContext*                     pFileContext;
    PIO_STACK_LOCATION                irpStack;
    NDIS_STATUS                       status;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkCreate ...\n");

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = ExAllocatePoolWithQuotaTag(NonPagedPool, sizeof(tFileContext),
                                              PLK_MEM_TAG);

    if (pFileContext == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("PLK: Failed to create file context\n");
    }

    IoInitializeRemoveLock(&pFileContext->driverAccessLock, PLK_MEM_TAG, 0, 0);

    irpStack->FileObject->FsContext = (void*)pFileContext;

    NdisZeroMemory(&timerChars, sizeof(timerChars));

    {C_ASSERT(NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1 <= sizeof(timerChars)); }
    timerChars.Header.Type = NDIS_OBJECT_TYPE_TIMER_CHARACTERISTICS;
    timerChars.Header.Size = NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1;
    timerChars.Header.Revision = NDIS_TIMER_CHARACTERISTICS_REVISION_1;

    timerChars.TimerFunction = increaseHeartbeatCb;
    timerChars.FunctionContext = NULL;
    timerChars.AllocationTag = PLK_MEM_TAG;

    status = NdisAllocateTimerObject(
                                    plkDriverInstance_l.driverHandle,
                                    &timerChars,
                                    &heartbeatTimer_g);
    if (status != NDIS_STATUS_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Timer Creation Failed %x\n",__func__, status);
        return STATUS_SUCCESS;
    }

    if (ctrlk_init() != kErrorOk)
    {
        return NDIS_STATUS_RESOURCES;
    }

    startHeartbeatTimer(20);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkCreate - OK\n");

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver create function

The function implements openPOWERLINK kernel module open function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCleanup(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver create function

The function implements openPOWERLINK kernel module open function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkClose(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    tFileContext*                     pFileContext;
    PIO_STACK_LOCATION                irpStack;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkClose...\n");

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = irpStack->FileObject->FsContext;
    ExFreePoolWithTag(pFileContext, PLK_MEM_TAG);

    stopHeartbeatTimer();
    ctrlk_exit();

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkClose - OK\n");

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);
    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver create function

The function implements openPOWERLINK kernel module open function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkIoctl(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    PIO_STACK_LOCATION  irpStack;
    NTSTATUS            status = STATUS_SUCCESS;
    ULONG               inlen,outlen;
    void*               pInBuffer;
    void*               pOutBuffer;
    tFileContext*       pFileContext;


    UNREFERENCED_PARAMETER(DeviceObject);

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    // Acquire the IRP remove lock
    status = IoAcquireRemoveLock(&pFileContext->driverAccessLock, pIrp_p);

    if (!NT_SUCCESS(status)) {
        //
        // Lock is in a removed state. That means we have already received
        // cleaned up request for this handle.
        //
        pIrp_p->IoStatus.Status = status;
        IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);
        return status;
    }

    inlen = irpStack->Parameters.DeviceIoControl.InputBufferLength;
    outlen = irpStack->Parameters.DeviceIoControl.OutputBufferLength;

    switch (irpStack->Parameters.DeviceIoControl.IoControlCode)
    {
        case PLK_CMD_CTRL_EXECUTE_CMD:
            tCtrlCmd* pCtrlCmd = (tCtrlCmd*) pIrp_p->AssociatedIrp.SystemBuffer;
//          pInBuffer = pIrp_p->AssociatedIrp.SystemBuffer;
//          pOutBuffer = pIrp_p->AssociatedIrp.SystemBuffer;

            executeCmd(pCtrlCmd);
            pIrp_p->IoStatus.Information = sizeof(tCtrlCmd);
            break;

        case PLK_CMD_CTRL_STORE_INITPARAM:

            tCtrlInitParam* pCtrlInitCmd = (tCtrlInitParam*) pIrp_p->AssociatedIrp.SystemBuffer;

            storeInitParam(pCtrlInitCmd);

            pIrp_p->IoStatus.Information = sizeof(tCtrlInitParam);
            break;

        case PLK_CMD_CTRL_READ_INITPARAM:

            tCtrlInitParam* pCtrlInitCmd = (tCtrlInitParam*) pIrp_p->AssociatedIrp.SystemBuffer;

            readInitParam(pCtrlInitCmd);
            pIrp_p->IoStatus.Information = sizeof(tCtrlInitParam);
            break;

        case PLK_CMD_CTRL_GET_STATUS:
            UINT16*  pStatus = (UINT16*) pIrp_p->AssociatedIrp.SystemBuffer;

            getStatus(pStatus);

            pIrp_p->IoStatus.Information = sizeof(UINT16);
            break;

        case PLK_CMD_CTRL_GET_HEARTBEAT:

            UINT16*  pHeartBeat = (UINT16*) pIrp_p->AssociatedIrp.SystemBuffer;
            getHeartbeat(pHeartBeat);
            pIrp_p->IoStatus.Information = sizeof(UINT16);
            break;

        case PLK_CMD_POST_EVENT:

            pInBuffer = pIrp_p->AssociatedIrp.SystemBuffer;

            eventkcal_postEventFromUser(pInBuffer);

            break;

        case PLK_CMD_GET_EVENT:
            size_t     eventSize;
            pOutBuffer = pIrp_p->AssociatedIrp.SystemBuffer;

            eventkcal_getEventForUser(pOutBuffer, eventSize);

            if (!pIrp_p->Cancel)
            {
                // Mark Irp pending
                IoMarkIrpPending(pIrp_p);
                // Insert Irp to eventQueue list
                NdisInterlockedInsertTailList(&plkDriverInstance_l.eventQueue,
                                              &pIrp_p->Tail.Overlay.ListEntry,
                                              &plkDriverInstance_l.eventQueueLock);
            }
            else
            {
                pIrp_p->IoStatus.Information = 0;
                Status = STATUS_CANCELLED;
                break;
            }

            status = STATUS_PENDING;
            break;

        case PLK_CMD_DLLCAL_ASYNCSEND:
            ret = sendAsyncFrame(arg);
            break;

        case PLK_CMD_ERRHND_WRITE:
            ret = writeErrorObject(arg);
            break;

        case PLK_CMD_ERRHND_READ:
            ret = readErrorObject(arg);
            break;

        case PLK_CMD_PDO_SYNC:
            if ((oplRet = pdokcal_waitSyncEvent()) == kErrorRetry)
                ret = -ERESTARTSYS;
            else
                ret = 0;
            break;

        default:
            DEBUG_LVL_ERROR_TRACE("PLK: - Invalid cmd (cmd=%d type=%d)\n", _IOC_NR(cmd), _IOC_TYPE(cmd));
            ret = -ENOTTY;
            break;
    }

    IoReleaseRemoveLock(&pFileContext->driverAccessLock, pIrp_p);

    return status;
}

//------------------------------------------------------------------------------
/**
\brief  Start heartbeat timer

The function starts the timer used for updating the heartbeat counter.

\param  timeInMs_p          Timeout value in milliseconds

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
void startHeartbeatTimer(ULONG timeInMs_p)
{
    LARGE_INTEGER   dueTime;
    dueTime.QuadPart = -(timeInMs_p * 10000);
    NdisSetTimerObject(heartbeatTimer_g, dueTime, timeInMs_p, NULL);
}

//------------------------------------------------------------------------------
/**
\brief  Stop heartbeat timer

The function stops the timer used for updating the heartbeat counter.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
void stopHeartbeatTimer(void)
{
    NdisCancelTimerObject(heartbeatTimer_g);
}

//------------------------------------------------------------------------------
/**
\brief  Increase heartbeat

The function implements the timer callback function used to increase the
heartbeat counter.

\param  data_p          Not used, need for timer interface

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
void increaseHeartbeatCb(void* unusedParameter1_p, void* functionContext_p,
                         void* unusedParameter2_p, void* unusedParameter3_p)
{
    UNUSED_PARAMETER(unusedParameter1_p);
    UNUSED_PARAMETER(functionContext_p);
    UNUSED_PARAMETER(unusedParameter2_p);
    UNUSED_PARAMETER(unusedParameter3_p);

    ctrlk_updateHeartbeat();
}
///\}

