/**
********************************************************************************
\file   main.c

\brief  main file for Windows kernel module

This file contains the main part of the Windows kernel module implementation for
PCIe device.

\ingroup module_driver_ndispcie
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

#include <ndis-intf.h>

#include "drvintf.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_MEM_TAG       'klpO'
#define  SRAM_TEST
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
extern UINT8*               benchmarkBase_g;          ///< Pointer to user benchmark
static LARGE_INTEGER               startTime_l, endTime_l, frequency_l;
static ULONGLONG Rxdif_l, Txdif_l, TxTotal_l = 0, RxTotal_l = 0, txcount_l = 0, rxcount_l = 0;
static LONGLONG multiplier_l;
//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_IO_WR8(addr, val)      (*(volatile UINT8*)(addr)) = (val)
#define OPLK_IO_WR16(addr, val)     (*(volatile UINT16*)(addr)) = (val)
#define OPLK_IO_WR32(addr, val)     (*(volatile UINT32*)(addr)) = (val)
// - Read
#define OPLK_IO_RD8(addr)           (*(volatile UINT8*)(addr))
#define OPLK_IO_RD16(addr)          (*(volatile UINT16*)(addr))
#define OPLK_IO_RD32(addr)          (*(volatile UINT32*)(addr))


#define BENCHMARK_SET(base, x)        OPLK_IO_WR8(base,((OPLK_IO_RD8(base)) | (1 << (x))))
#define BENCHMARK_RESET(base, x)      OPLK_IO_WR8(base,((OPLK_IO_RD8(base)) & ~(1 << (x))))
#define BENCHMARK_TOGGLE(base, x)     OPLK_IO_WR8(base,((OPLK_IO_RD8(base)) ^ (1 << (x))))

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief  Instance for POWERLINK driver

The structure specifies the instance variable of the Windows kernel driver
*/
typedef struct
{
    PDEVICE_OBJECT        pAppDeviceObject;         ///< IOCTL interface device object
    NDIS_HANDLE           pAppDeviceHandle;         ///< IOCTL interface device handle
    NDIS_HANDLE           driverHandle;             ///< Miniport driver handle
    char                  fInitialized;             ///< Initialization status
    UINT                  instanceCount;            ///< Number of open instances
    UINT8*                pCommMemBase;             ///< Pointer to common memory
    UINT8*                pSharedMemBase;           ///< Pointer to shared memory
}tPlkDriverInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tPlkDriverInstance    plkDriverInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
DRIVER_DISPATCH       powerlinkCreate;
DRIVER_DISPATCH       powerlinkCleanup;
DRIVER_DISPATCH       powerlinkClose;
DRIVER_DISPATCH       powerlinkIoctl;

static void registerDrvIntf(NDIS_HANDLE driverHandle_p);
static void deregisterDrvIntf(void);

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//---------------------------------------------------------------------------
//  Initialize driver
//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
/**
\brief  Driver initialization routine

The function implements openPOWERLINK Windows kernel driver initialization callback.
OS calls this routine on driver registration.

\param  driverObject_p       Pointer to the system's driver object structure
                             for this driver.
\param  registryPath_p       System's registry path for this driver.

\return This routine returns a NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS DriverEntry(PDRIVER_OBJECT driverObject_p, PUNICODE_STRING registryPath_p)
{
    NDIS_STATUS    ndisStatus;

    TRACE("PLK: + Driver Entry Latency Test Driver\n");
    ndisStatus = ndis_initDriver(driverObject_p, registryPath_p);

    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Failed to initialize driver 0x%X\n", __FUNCTION__,
                              ndisStatus);
        return ndisStatus;
    }

    // register application interface handlers
    ndis_registerDrvIntf(registerDrvIntf, deregisterDrvIntf);
    plkDriverInstance_l.fInitialized = FALSE;
    plkDriverInstance_l.instanceCount = 0;


    TRACE("READ %x Write %x\n", PLK_CMD_READ, PLK_CMD_WRITE);
    TRACE("PLK: + Driver Entry - OK\n");
    return ndisStatus;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver create function

The function implements openPOWERLINK kernel module create callback. OS calls
this routine when a application tries to open an FILE interface to this driver
using CreateFile().

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns a NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCreate(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    NDIS_TIMER_CHARACTERISTICS    timerChars;
    tFileContext*                 pFileContext;
    PIO_STACK_LOCATION            irpStack;
    NDIS_STATUS                   status;
    ULONG                         baroffset;

    UNREFERENCED_PARAMETER(pDeviceObject_p);

    TRACE("PLK: + powerlinkCreate ...\n");

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = ExAllocatePoolWithQuotaTag(NonPagedPool, sizeof(tFileContext),
                                              PLK_MEM_TAG);

    if (pFileContext == NULL)
    {
        TRACE("PLK: Failed to create file context\n");
    }

    IoInitializeRemoveLock(&pFileContext->driverAccessLock, PLK_MEM_TAG, 0, 0);

    irpStack->FileObject->FsContext = (void*) pFileContext;

    plkDriverInstance_l.pCommMemBase = ndis_getBarAddr(1);

    if (plkDriverInstance_l.pCommMemBase == NULL)
    {
        TRACE("Unable to get common memory\n");
    }

    baroffset = *((ULONG*)plkDriverInstance_l.pCommMemBase);

    plkDriverInstance_l.pSharedMemBase = (UINT8*)ndis_getBarAddr(0) + baroffset;

    if (plkDriverInstance_l.pSharedMemBase == NULL)
    {
        TRACE("Unable to get shared memory\n");
    }

    plkDriverInstance_l.pCommMemBase = (plkDriverInstance_l.pCommMemBase + 4);

    startTime_l = KeQueryPerformanceCounter(&frequency_l);
    if (!plkDriverInstance_l.fInitialized)
    {
        plkDriverInstance_l.fInitialized = TRUE;
    }

    // Increase the count for open instances
    plkDriverInstance_l.instanceCount++;

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);



    TRACE("PLK: + powerlinkCreate - OK\n");

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver clean-up function

The function implements the clean-up callback. OS calls this when an application
closes the FILE interface to the IOCTL device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns a NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCleanup(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver close function

The function implements openPOWERLINK kernel driver close callback. OS calls
this function when the user application calls CloseHandle() for the device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns a NTSTATUS error code.
\retval Always return STATUS_SUCCESS.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkClose(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    tFileContext*         pFileContext;
    PIO_STACK_LOCATION    irpStack;
    UINT16                status;
    TRACE("PLK: + powerlinkClose...\n");

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = irpStack->FileObject->FsContext;
    ExFreePoolWithTag(pFileContext, PLK_MEM_TAG);

    plkDriverInstance_l.instanceCount--;

    // Close lower driver resources only if all open instances have closed.
    if (plkDriverInstance_l.fInitialized && plkDriverInstance_l.instanceCount == 0)
    {
        plkDriverInstance_l.fInitialized = FALSE;
    }

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);

    TRACE("PLK: + powerlinkClose - OK\n");

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver IOCTL handler

The function implements IOCTL callback. OS calls this routine when the user
application calls DeviceIoControl() for the device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns a NTSTATUS error code.
\retval Always return STATUS_SUCCESS.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkIoctl(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    PIO_STACK_LOCATION    irpStack;
    NTSTATUS              status = STATUS_SUCCESS;
    int                   ret;
    ULONG                 inlen, outlen;
    void*                 pInBuffer;
    void*                 pOutBuffer;
    tFileContext*         pFileContext;
#ifdef SRAM_TEST
    UINT8*      pTestBuffer = plkDriverInstance_l.pSharedMemBase;
#else
    UINT8*      pTestBuffer = plkDriverInstance_l.pCommMemBase;
#endif
    UNREFERENCED_PARAMETER(pDeviceObject_p);

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = irpStack->FileObject->FsContext;

    // Acquire the IRP remove lock
    status = IoAcquireRemoveLock(&pFileContext->driverAccessLock, pIrp_p);

    if (!NT_SUCCESS(status))
    {
        // Lock is in a removed state. That means we have already received
        // cleaned up request for this handle.
        pIrp_p->IoStatus.Status = status;
        IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);
        return status;
    }

    inlen = irpStack->Parameters.DeviceIoControl.InputBufferLength;
    outlen = irpStack->Parameters.DeviceIoControl.OutputBufferLength;

    switch (irpStack->Parameters.DeviceIoControl.IoControlCode)
    {
        case PLK_CMD_WRITE:
        {
            pInBuffer = pIrp_p->AssociatedIrp.SystemBuffer;

            if (pTestBuffer != NULL &&
                pInBuffer != NULL)
            {
                BENCHMARK_SET(benchmarkBase_g, 2);
                startTime_l = KeQueryPerformanceCounter(&frequency_l);
                NdisMoveMemory(pTestBuffer, pInBuffer, inlen);
                endTime_l = KeQueryPerformanceCounter(&frequency_l);
                BENCHMARK_RESET(benchmarkBase_g, 2);
            }
            Txdif_l = (endTime_l.QuadPart - startTime_l.QuadPart);
            TxTotal_l += Txdif_l;
            pIrp_p->IoStatus.Information = inlen;
            break;
        }
        case PLK_CMD_READ:
        {
            pOutBuffer = pIrp_p->AssociatedIrp.SystemBuffer;

            if (pTestBuffer != NULL &&
                pOutBuffer != NULL)
            {
                BENCHMARK_SET(benchmarkBase_g, 3);
                startTime_l = KeQueryPerformanceCounter(&frequency_l);
                NdisMoveMemory(pOutBuffer, pTestBuffer, outlen);
                endTime_l = KeQueryPerformanceCounter(&frequency_l);
                BENCHMARK_RESET(benchmarkBase_g, 3);
            }

            Rxdif_l = (endTime_l.QuadPart - startTime_l.QuadPart);
            RxTotal_l += Rxdif_l;
            pIrp_p->IoStatus.Information = outlen;
            break;
        }
        case PLK_CMD_GET_BENCHMARK:
        {
            tBenchmarkMem*   pBenchmarkMem = (tBenchmarkMem*) pIrp_p->AssociatedIrp.SystemBuffer;
            DbgPrint("Get benchmark\n");
            ret = drv_getBenchmarkMem((UINT8**) &pBenchmarkMem->pBaseAddr);
            if (ret != 0)
            {
                pIrp_p->IoStatus.Information = 0;
                DbgPrint("Error\\n");
            }
            else
            {
                pIrp_p->IoStatus.Information = sizeof(tBenchmarkMem);
                DbgPrint("Okay\\n");

            }
            status = STATUS_SUCCESS;
            break;
        }
        case PLK_CMD_PRINT_STATS:
        {
            ULONGLONG* count_l = (ULONGLONG*) pIrp_p->AssociatedIrp.SystemBuffer;
            multiplier_l = 1000000000LL / frequency_l.QuadPart;
            DbgPrint("Time taken for Rx %lld us  Tx %lld us in %lld TotalRx:%lld TTx:%lld\n", \
                     (((RxTotal_l) / *count_l)*multiplier_l) / 1000, \
                     (((TxTotal_l) / *count_l)*multiplier_l) / 1000, *count_l, \
                    (((RxTotal_l))*multiplier_l) / 1000, \
                    (((TxTotal_l))*multiplier_l) / 1000);
            break;
        }
        default:
            TRACE("PLK: - Invalid cmd (cmd=%x)\n",
                                  irpStack->Parameters.DeviceIoControl.IoControlCode);
            break;
    }

    if (status != STATUS_PENDING)
    {
        // complete the Irp if its not pended
        pIrp_p->IoStatus.Status = status;
        IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);
    }

    // Release lock
    IoReleaseRemoveLock(&pFileContext->driverAccessLock, pIrp_p);

    return status;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Register application interface device

This routine is called from the miniport initialize to register an IOCTL
interface for the driver. A user application can use this interfaces to
communicate with this driver.

A device object to be used for this purpose is created by NDIS when we call
NdisMRegisterDevice.

This routine is called whenever a new miniport instance is initialized.
However, we only create one global device object, when the first miniport
instance is initialized.

/param  driverHandle_p      Miniport driver handle returned by OS on registration

*/
//------------------------------------------------------------------------------
static void registerDrvIntf(NDIS_HANDLE driverHandle_p)
{
    NDIS_STATUS                      status = NDIS_STATUS_SUCCESS;
    UNICODE_STRING                   deviceName;
    UNICODE_STRING                   deviceLinkUnicodeString;
    NDIS_DEVICE_OBJECT_ATTRIBUTES    deviceObjectAttributes;
    PDRIVER_DISPATCH                 dispatchTable[IRP_MJ_MAXIMUM_FUNCTION + 1];

    TRACE("PLK %s()...\n", __FUNCTION__);
    plkDriverInstance_l.driverHandle = driverHandle_p;
    NdisZeroMemory(dispatchTable, (IRP_MJ_MAXIMUM_FUNCTION + 1) * sizeof(PDRIVER_DISPATCH));

    dispatchTable[IRP_MJ_CREATE] = powerlinkCreate;
    dispatchTable[IRP_MJ_CLEANUP] = powerlinkCleanup;
    dispatchTable[IRP_MJ_CLOSE] = powerlinkClose;
    dispatchTable[IRP_MJ_DEVICE_CONTROL] = powerlinkIoctl;

    NdisInitUnicodeString(&deviceName, PLK_DEV_STRING);
    NdisInitUnicodeString(&deviceLinkUnicodeString, PLK_LINK_NAME);

    NdisZeroMemory(&deviceObjectAttributes, sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES));

    // type implicit from the context
    deviceObjectAttributes.Header.Type = NDIS_OBJECT_TYPE_DEVICE_OBJECT_ATTRIBUTES;
    deviceObjectAttributes.Header.Revision = NDIS_DEVICE_OBJECT_ATTRIBUTES_REVISION_1;
    deviceObjectAttributes.Header.Size = sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES);
    deviceObjectAttributes.DeviceName = &deviceName;
    deviceObjectAttributes.SymbolicName = &deviceLinkUnicodeString;
    deviceObjectAttributes.MajorFunctions = &dispatchTable[0];
    deviceObjectAttributes.ExtensionSize = 0;

    status = NdisRegisterDeviceEx(driverHandle_p,
                                  &deviceObjectAttributes,
                                  &plkDriverInstance_l.pAppDeviceObject,
                                  &plkDriverInstance_l.pAppDeviceHandle);

    plkDriverInstance_l.pAppDeviceObject->Flags |= DO_BUFFERED_IO;

    TRACE("PLK %s() - OK\n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief De-register application interface device

De-register the IOCTL interface registered during initialization,

*/
//------------------------------------------------------------------------------
static void deregisterDrvIntf(void)
{
    if (plkDriverInstance_l.pAppDeviceHandle != NULL)
    {
        NdisDeregisterDeviceEx(plkDriverInstance_l.pAppDeviceHandle);
    }
}

///\}

