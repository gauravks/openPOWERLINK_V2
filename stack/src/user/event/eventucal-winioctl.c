/**
********************************************************************************
\file   eventucal-winioctl.c

\brief  User event CAL module for ioctl interface on Windows

This file implements the user event CAK module for Windows user-kernel demo which
uses Windows ioctl calls for communication.

The event user module uses the circular memory library interface for the creation
and management for event queues. Separated thread routines to process events
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
#include <common/target.h>

#include <user/ctrlucal.h>
#include <oplk/powerlink-module.h>

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
/**
\brief User event CAL instance type

The structure contains all necessary information needed by the user event
CAL module.
*/
typedef struct
{
    HANDLE                 fileHandle;
    HANDLE                 threadHandle;
    BOOL                   fStopThread;
    UINT32                 threadId;
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance    instance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static UINT32 eventThread(void* arg_p);
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
    tOplkError          ret = kErrorOk;

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    instance_l.fileHandle = (void*)ctrlucal_getFd();
    instance_l.fStopThread = FALSE;

    instance_l.threadHandle = CreateThread(
                                     NULL,                // Default security attributes
                                     0,                   // Use Default stack size
                                     eventThread,         // Thread routine
                                     NULL,                // Argum to the thread routine
                                     0,                   // Use default creation flags
                                     &instance_l.threadId // Returened thread Id
                                     );

    if (instance_l.threadHandle == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to create event thread with error: 0x%X\n",
                              __func__, GetLastError());
        return kErrorNoResource;
    }

    if (!SetPriorityClass(instance_l.threadHandle, REALTIME_PRIORITY_CLASS))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to boost thread priority with error: 0x%X\n",
                                __func__, GetLastError());
        return kErrorNoResource;
    }

    if (!SetThreadPriority(instance_l.threadHandle, THREAD_PRIORITY_TIME_CRITICAL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to boost thread priority with error: 0x%X\n",
                              __func__, GetLastError());
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up user event CAL module

The function cleans up the user event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit(void)
{
    UINT            i = 0;

    instance_l.fStopThread = TRUE;
    while (instance_l.fStopThread == TRUE)
    {
        target_msleep(10);
        if (i++ > 1000)
        {
            TRACE("Event Thread is not terminating, continue shutdown...!\n");
            break;
        }
    }

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
    return postEvent(pEvent_p);
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
    UINT8   eventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    size_t  eventBufSize = sizeof(tEvent) + pEvent_p->eventArgSize;
    ULONG   bytesReturned;

    OPLK_MEMCPY(eventBuf, pEvent_p, sizeof(tEvent));
    OPLK_MEMCPY((eventBuf + sizeof(tEvent)), pEvent_p->pEventArg, pEvent_p->eventArgSize);

    if (!DeviceIoControl(instance_l.fileHandle, PLK_CMD_POST_EVENT,
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
static UINT32 eventThread(void* arg_p)
{
    tEvent*     pEvent;
    BOOL        ret;
    char        eventBuf[sizeof(tEvent) +MAX_EVENT_ARG_SIZE];
    size_t      eventBufSize;
    ULONG       bytesReturned;

    UNUSED_PARAMETER(arg_p);

    pEvent = (tEvent*) eventBuf;

    while (!instance_l.fStopThread)
    {
        ret = DeviceIoControl(instance_l.fileHandle, PLK_CMD_GET_EVENT,
                              0, 0, eventBuf, eventBufSize,
                              &bytesReturned, NULL);
        if (ret != 0)
        {
            /*TRACE ("%s() User: got event type:%d(%s) sink:%d(%s)\n", __func__,
            pEvent->eventType, debugstr_getEventTypeStr(pEvent->eventType),
            pEvent->eventSink, debugstr_getEventSinkStr(pEvent->eventSink));*/
            if (pEvent->eventArgSize != 0)
                pEvent->pEventArg = (char*) pEvent + sizeof(tEvent);

            ret = eventu_process(pEvent);
        }
        /*else
        TRACE("%s() ret = %d\n", __func__, ret);*/
    }
    instance_l.fStopThread = FALSE;

    return 0;
}

///\}

