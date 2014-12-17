/**
********************************************************************************
\file   eventucal-winioctl.c

\brief  User event CAL module for IOCTL interface on Windows

This file implements the user event CAL module for Windows user-kernel demo which
uses Windows IOCTL calls for communication to a external PCIe.

The event user module uses the circular memory library interface for the creation
and management for event queues. Separate thread routines to process events
are added to process events in background.

\ingroup module_eventucal
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
#include <oplk/debugstr.h>
#include <user/eventucal.h>
#include <user/eventucalintf.h>
#include <common/target.h>

#include <user/ctrlucal.h>
#include <common/driver.h>
#include <user/eventu.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DEVICE_CLOSE_IO    995
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
/**
\brief User event CAL instance type

The structure contains all necessary information needed by the user event
CAL module.
*/
typedef struct
{
    HANDLE    sendfileHandle;                ///< Handle to driver for sending events
    HANDLE    rcvfileHandle;                 ///< Handle to driver for receiving events
    HANDLE    eventProcThread;               ///< User event thread handle
    HANDLE    kernelThread;                  ///< Kernel event thread handle
    HANDLE    semUserData;                   ///< Semaphore to synchronize main and user thread access
    HANDLE    mutexK2U;                      ///< Mutex used for locking access to kernel to user queue
    BOOL      fStopThread;                   ///< Flag to identify thread exit
    char      eventBuf[sizeof(tEvent) +MAX_EVENT_ARG_SIZE]; ///< Event buffer
    BOOL      fKernelEvent;                  ///< Flag to indicate new kernel event
    BOOL      fInitialized;                  ///< Flag to indicate module initialization
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance    instance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
void              signalUserEvent(void);
static UINT32     eventProcess(void* arg_p);
static UINT32     kernelEventThread(void* arg_p);
static tOplkError postEvent(tEvent* pEvent_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize user event CAL module

The function initializes the user event CAL module. Depending on the
configuration it gets the function pointer interface of the used queue
implementations and calls the appropriate init functions.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init(void)
{
    tOplkError    ret = kErrorOk;
    TCHAR         mutexName[MAX_PATH];

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    instance_l.sendfileHandle = ctrlucal_getFd();
    instance_l.fStopThread = FALSE;

    if ((instance_l.semUserData = CreateSemaphore(NULL, 0, 100, "Local\\semUserEvent")) == NULL)
        goto Exit;

    sprintf(mutexName, "Local\\K2UMutex");
    if ((instance_l.mutexK2U = CreateMutex(NULL, FALSE, mutexName)) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() creating mutex failed!\n", __func__);
    }

    if (eventucal_initQueueCircbuf(kEventQueueUInt) != kErrorOk)
        goto Exit;

    eventucal_setSignalingCircbuf(kEventQueueUInt, signalUserEvent);

    instance_l.eventProcThread = CreateThread(NULL,      // Default security attributes
                                           0,            // Use Default stack size
                                           eventProcess, // Thread routine
                                           NULL,         // Argument to the thread routine
                                           0,            // Use default creation flags
                                           NULL          // Returned thread Id
                                          );

    if (instance_l.eventProcThread == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to create event Process thread with error: 0x%X\n",
                              __func__, GetLastError());
        return kErrorNoResource;
    }

    instance_l.kernelThread = CreateThread(NULL,                  // Default security attributes
                                              0,                  // Use Default stack size
                                              kernelEventThread,  // Thread routine
                                              NULL,               // Argument to the thread routine
                                              0,                  // Use default creation flags
                                              NULL                // Returned thread Id
                                              );

    if (instance_l.kernelThread == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to create kernel thread with error: 0x%X\n",
                              __func__, GetLastError());
        return kErrorNoResource;
    }

    if (!SetThreadPriority(instance_l.eventProcThread, THREAD_PRIORITY_TIME_CRITICAL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to boost thread priority with error: 0x%X\n",
                              __func__, GetLastError());
        return kErrorNoResource;
    }

    if (!SetThreadPriority(instance_l.kernelThread, THREAD_PRIORITY_TIME_CRITICAL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to boost kernel thread priority with error: 0x%X\n",
                              __func__, GetLastError());
        return kErrorNoResource;
    }

    instance_l.fInitialized = TRUE;
    return kErrorOk;
Exit:
    if (instance_l.semUserData != NULL)
        CloseHandle(instance_l.semUserData);

    if (instance_l.mutexK2U != NULL)
        CloseHandle(instance_l.mutexK2U);

    eventucal_exitQueueCircbuf(kEventQueueUInt);

    return kErrorNoResource;

}

//------------------------------------------------------------------------------
/**
\brief    Clean up user event CAL module

The function cleans up the user event CAL module. For clean-up it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit(void)
{
    UINT    i = 0;

    if (instance_l.fInitialized)
    {
        instance_l.fStopThread = TRUE;
        ReleaseSemaphore(instance_l.semUserData, 1, NULL);
        while (instance_l.fStopThread == TRUE)
        {
            target_msleep(10);
            if (i++ > 1000)
            {
                TRACE("Event Thread is not terminating, continue shutdown...!\n");
                break;
            }
        }

        eventucal_exitQueueCircbuf(kEventQueueUInt);

        if (instance_l.semUserData != NULL)
            CloseHandle(instance_l.semUserData);

        if (instance_l.mutexK2U != NULL)
            CloseHandle(instance_l.mutexK2U);
    }

    instance_l.fInitialized = FALSE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts an event to a queue. It is called from the generic user
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postUserEvent(tEvent* pEvent_p)
{
    return eventucal_postEventCircbuf(kEventQueueUInt, pEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to a queue. It is called from the generic user
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postKernelEvent(tEvent* pEvent_p)
{
    return postEvent(pEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief  Process function of user CAL module

This function will be called by the system process function.

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
void eventucal_process(void)
{
    // Nothing to do, because we use threads
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Post event

This function posts an event to a queue. It is called from the generic user
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred
*/
//------------------------------------------------------------------------------
static tOplkError postEvent(tEvent* pEvent_p)
{
    UINT8       eventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    UINT32      eventBufSize = sizeof(tEvent) + pEvent_p->eventArgSize;
    ULONG       bytesReturned;

    OPLK_MEMCPY(eventBuf, pEvent_p, sizeof(tEvent));

    if (pEvent_p->eventArgSize != 0)
    {
        OPLK_MEMCPY((eventBuf + sizeof(tEvent)), (void*)pEvent_p->pEventArg, pEvent_p->eventArgSize);
    }
    

    if (!DeviceIoControl(instance_l.sendfileHandle, PLK_CMD_POST_EVENT,
                         eventBuf, eventBufSize,
                         0, 0, &bytesReturned, NULL))
        return kErrorNoResource;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Event thread function

This function implements the event thread.

\param  arg_p                Thread argument.

*/
//------------------------------------------------------------------------------
static UINT32 eventProcess(void* arg_p)
{
    DWORD                   waitResult;
    tEvent*                 pEvent;
    DEBUG_LVL_EVENTU_TRACE("User event thread %d waiting for events...\n", GetCurrentThreadId());

    while (!instance_l.fStopThread)
    {
        waitResult = WaitForSingleObject(instance_l.semUserData, 5000);
        switch (waitResult)
        {
            case WAIT_OBJECT_0:
                /* first handle all kernel to user events --> higher priority! */
                if (instance_l.fKernelEvent)
                {
                    pEvent = (tEvent*) instance_l.eventBuf;
                    if (pEvent->eventArgSize != 0)
                        pEvent->pEventArg = (ULONGLONG)((char*) pEvent + sizeof(tEvent));
                    
                    eventu_process(pEvent);
                    instance_l.fKernelEvent = FALSE;
                    ReleaseMutex(instance_l.mutexK2U);
                }
                else
                {
                    if (eventucal_getEventCountCircbuf(kEventQueueUInt) > 0)
                    {
                        eventucal_processEventCircbuf(kEventQueueUInt);
                    }
                }
                break;

            case WAIT_TIMEOUT:
                break;

            default:
                DEBUG_LVL_ERROR_TRACE("%s() Semaphore wait unknown error! Error:%ld\n",
                                      __func__, GetLastError());
                if (GetLastError() == 6)
                    instance_l.fStopThread = TRUE;
                break;
        }
    }
    DEBUG_LVL_EVENTU_TRACE("User Event Thread is exiting!\n");
    return 0;

}

static UINT32 kernelEventThread(void* arg_p)
{
    BOOL      ret;
    UINT32    eventBufSize = sizeof(tEvent) + MAX_EVENT_ARG_SIZE;
    ULONG     bytesReturned;
    UINT      errNum = 0;

    UNUSED_PARAMETER(arg_p);
    instance_l.rcvfileHandle = CreateFile(PLK_DEV_FILE,                        // Name of the NT "device" to open
                                          GENERIC_READ | GENERIC_WRITE,        // Access rights requested
                                          FILE_SHARE_READ | FILE_SHARE_WRITE,  // Share access - NONE
                                          NULL,                                // Security attributes - not used!
                                          OPEN_EXISTING,                       // Device must exist to open it.
                                          FILE_ATTRIBUTE_NORMAL,               // Open for overlapped I/O
                                          NULL);

    if (instance_l.rcvfileHandle == INVALID_HANDLE_VALUE)
    {
        errNum = GetLastError();

        if (!(errNum == ERROR_FILE_NOT_FOUND ||
            errNum == ERROR_PATH_NOT_FOUND))
        {
            DEBUG_LVL_ERROR_TRACE("%s() createFile failed!  ERROR_FILE_NOT_FOUND = %d\n",
                                  __func__, errNum);
            return kErrorNoResource;
        }
    }

    while (!instance_l.fStopThread)
    {
        target_msleep(10);
        ret = DeviceIoControl(instance_l.rcvfileHandle, PLK_CMD_GET_EVENT,
                              NULL, 0, instance_l.eventBuf, eventBufSize,
                              &bytesReturned, NULL);
        if (!ret)
        {
            if (DEVICE_CLOSE_IO == GetLastError())
            {
                DEBUG_LVL_ALWAYS_TRACE("Closing Event Thread\n");
            }
            else
            {
                DEBUG_LVL_ERROR_TRACE("%s():Error in DeviceIoControl : %d\n", GetLastError());
            }

            break;
        }

        if (bytesReturned > 0)
        {
            instance_l.fKernelEvent = TRUE;
            ReleaseSemaphore(instance_l.semUserData, 1, NULL);
            WaitForSingleObject(instance_l.mutexK2U, 5000);
        }
    }

    CloseHandle(instance_l.rcvfileHandle);
    instance_l.fStopThread = FALSE;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Signal a user event

This function signals that a user event was posted. It will be registered in
the circular buffer library as signal callback function
*/
//------------------------------------------------------------------------------
void signalUserEvent(void)
{
    ReleaseSemaphore(instance_l.semUserData, 1, NULL);
}
///\}

