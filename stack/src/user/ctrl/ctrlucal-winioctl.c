/**
********************************************************************************
\file   ctrlucal-winioctl.c

\brief  User control CAL module using a ioctl calls to Windows kernel

This file contains the implementation of user CAL module which uses ioctl
to communicate to a openPOWERLINK kernel layer in Windows kernel.

\ingroup module_ctrlucal
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
#include <stddef.h>

#include <oplk/oplk.h>
#include <common/ctrl.h>
#include <common/ctrlcal.h>
#include <common/ctrlcal-mem.h>
#include <user/ctrlucal.h>
#include <common/target.h>

#include <common/driver.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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
static HANDLE    fileHandle_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user control CAL module

The function initializes the user control CAL module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_init(void)
{
    UINT32    errCode;

    fileHandle_l = CreateFile(PLK_DEV_FILE,                                                 // Name of the NT "device" to open
                              GENERIC_READ | GENERIC_WRITE,                                 // Access rights requested
                              FILE_SHARE_READ | FILE_SHARE_WRITE,                           // Share access - NONE
                              NULL,                                                         // Security attributes - not used!
                              OPEN_EXISTING,                                                // Device must exist to open it.
                              FILE_ATTRIBUTE_NORMAL,                                        // Open for overlapped I/O
                              NULL);                                                        // Extended attributes - not used!

    if (fileHandle_l == INVALID_HANDLE_VALUE)
    {
        errCode = GetLastError();
        TRACE("%s() CreateFile failed with error 0x%x\n", __func__, errCode);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up user control CAL module

The function cleans up the user control CAL module.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
void ctrlucal_exit(void)
{
    CloseHandle(fileHandle_l);
}

//------------------------------------------------------------------------------
/**
\brief  Process user control CAL module

This function provides processing time for the CAL module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_process(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Execute a ctrl command

The function executes a control command in the kernel stack.

\param  cmd_p            Command to execute

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_executeCmd(tCtrlCmdType cmd_p)
{
    tCtrlCmd    ctrlCmd;
    tCtrlCmd    ctrlCmdRes;
    ULONG       bytesReturned;

    ctrlCmd.cmd = cmd_p;
    ctrlCmd.retVal = 0;

    if (!DeviceIoControl(fileHandle_l, PLK_CMD_CTRL_EXECUTE_CMD,
                         &ctrlCmd, sizeof(tCtrlCmd),
                         &ctrlCmdRes, sizeof(tCtrlCmd),
                         &bytesReturned, NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n", __func__, GetLastError());
        return kErrorGeneralError;
    }

    return ctrlCmdRes.retVal;
}

//------------------------------------------------------------------------------
/**
\brief Check state of kernel stack

The function checks the state of the kernel stack. If it is already running
it tries to shutdown.

\return The function returns a tOplkError error code.
\retval kErrorOk             Kernel stack is initialized
\retval kErrorNoResource     Kernel stack is not running or in wrong state

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_checkKernelStack(void)
{
    UINT16        kernelStatus;
    tOplkError    ret;

    TRACE("Checking for kernel stack...\n");
    kernelStatus = ctrlucal_getStatus();

    switch (kernelStatus)
    {
        case kCtrlStatusReady:
            ret = kErrorOk;
            break;

        case kCtrlStatusRunning:
            /* try to shutdown kernel stack */
            ret = ctrlucal_executeCmd(kCtrlCleanupStack);
            if (ret != kErrorOk)
            {
                ret = kErrorNoResource;
                break;
            }

            target_msleep(1000);

            kernelStatus = ctrlucal_getStatus();
            if (kernelStatus != kCtrlStatusReady)
            {
                ret = kErrorNoResource;
            }
            break;

        default:
            ret = kErrorNoResource;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get status of kernel stack

The function gets the status of the kernel stack

\return The function returns the kernel status.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
UINT16 ctrlucal_getStatus(void)
{
    UINT16    status;
    ULONG     bytesReturned;

    if (!DeviceIoControl(fileHandle_l, PLK_CMD_CTRL_GET_STATUS, 0, 0,
                         &status, sizeof(UINT16), &bytesReturned, NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n", __func__, GetLastError());
        return kCtrlStatusUnavailable;
    }
    //TRACE("Status %x\n", status);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Get the heartbeat of the kernel stack

The function reads the heartbeat genereated by the kernel stack.

\return The function returns the heartbeat counter.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
UINT16 ctrlucal_getHeartbeat(void)
{
    UINT16    heartbeat;
    ULONG     bytesReturned;

    if (!DeviceIoControl(fileHandle_l, PLK_CMD_CTRL_GET_HEARTBEAT, 0, 0,
                         &heartbeat, sizeof(UINT16), &bytesReturned, NULL))
    {
        TRACE("%s() Error in DeviceIoControl : %d\n", __func__, GetLastError());
        return 0;
    }
    return heartbeat;
}

//------------------------------------------------------------------------------
/**
\brief  Store the init parameters for kernel use

The function stores the openPOWERLINK initialization parameter so that they
can be accessed by the kernel stack.

\param  pInitParam_p        Specifies where to read the init parameters.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
void ctrlucal_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    ULONG    bytesReturned;

    if (!DeviceIoControl(fileHandle_l, PLK_CMD_CTRL_STORE_INITPARAM,
                         pInitParam_p, sizeof(tCtrlInitParam),
                         0, 0, &bytesReturned, NULL))
    {
        TRACE("%s() Error in DeviceIoControl : %d\n", __func__, GetLastError());
    }
}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from kernel

The function reads the initialization parameter from the kernel stack.

\param  pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tOplkError error code. It returns always
        kErrorOk!

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_readInitParam(tCtrlInitParam* pInitParam_p)
{
    ULONG    bytesReturned;

    if (!DeviceIoControl(fileHandle_l, PLK_CMD_CTRL_READ_INITPARAM,
                         0, 0,
                         pInitParam_p, sizeof(tCtrlInitParam),
                         &bytesReturned, NULL))
    {
        TRACE("%s() Error in DeviceIoControl : %d\n", __func__, GetLastError());
        return kErrorGeneralError;
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Return the file descriptor of the kernel module

The function returns the file descriptor of the kernel module.

\return The function returns the file descriptor.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
HANDLE ctrlucal_getFd(void)
{
    //TODO: return value should be a ULONG_PTR type to handle pointers correctly in 64-bit system
    return fileHandle_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}

