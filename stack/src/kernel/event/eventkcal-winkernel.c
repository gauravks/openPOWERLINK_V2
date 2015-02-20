/**
********************************************************************************
\file   eventkcal-winkernel.c

\brief  Kernel event CAL module for Windows kernelspace

This file implements the kernel event handler CAL module for the Windows
kernel. It uses the circular buffer interface for all event queues.

\see eventkcalintf-circbuf.c

\ingroup module_eventkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <oplk/oplkinc.h>
#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <kernel/eventkcal.h>
#include <kernel/eventkcalintf.h>
#include <common/circbuffer.h>

#include <ndis.h>
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
\brief Kernel event CAL instance type

The structure contains all necessary information needed by the kernel event
CAL module.
*/
typedef struct
{
    HANDLE        threadHandle;                     ///< Event thread handle.
    NDIS_EVENT    kernelWaitEvent;                  ///< NDIS event for kernel events.
    NDIS_EVENT    userWaitEvent;                    ///< NDIS event for user events.
    LONG          userEventCount;                   ///< User event counter.
    LONG          kernelEventCount;                 ///< Kernel event counter.
    BOOL          fThreadIsRunning;                 ///< Flag for thread status.
    BOOL          fInitialized;                     ///< Flag for event initialization status.
    BYTE          aUintRxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];   /// Array for user internal events.
    BYTE          aK2URxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];    /// Array for kernel to user events.
} tEventkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventkCalInstance    instance_l;            ///< Instance variable of kernel event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
KSTART_ROUTINE   eventThread;
static void      signalUserEvent(void);
static void      signalKernelEvent(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module. Depending on the
configuration it gets the function pointer interface of the used queue
implementations and calls the appropriate init functions.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_init(void)
{
    ULONG                desiredAccess = DELETE | SYNCHRONIZE;
    OBJECT_ATTRIBUTES    ObjectAttributes;

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventkCalInstance));

    NdisInitializeEvent(&instance_l.kernelWaitEvent);
    NdisInitializeEvent(&instance_l.userWaitEvent);

    instance_l.userEventCount = 0;
    instance_l.kernelEventCount = 0;

    if (eventkcal_initQueueCircbuf(kEventQueueK2U) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueU2K) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueKInt) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueUInt) != kErrorOk)
        goto Exit;

    eventkcal_setSignalingCircbuf(kEventQueueK2U, signalUserEvent);

    eventkcal_setSignalingCircbuf(kEventQueueU2K, signalKernelEvent);

    eventkcal_setSignalingCircbuf(kEventQueueUInt, signalUserEvent);

    eventkcal_setSignalingCircbuf(kEventQueueKInt, signalKernelEvent);

    InitializeObjectAttributes(&ObjectAttributes, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);

    // TODO@gks check the return value here
    PsCreateSystemThread(&instance_l.threadHandle, desiredAccess, &ObjectAttributes,
                         NULL, NULL, eventThread, &instance_l);

    instance_l.fInitialized = TRUE;

    return kErrorOk;

Exit:
    // TODO@gks Handle return value
    TRACE("%s() Initialization error!\n", __func__);
    eventkcal_exitQueueCircbuf(kEventQueueK2U);
    eventkcal_exitQueueCircbuf(kEventQueueU2K);
    eventkcal_exitQueueCircbuf(kEventQueueKInt);
    eventkcal_exitQueueCircbuf(kEventQueueUInt);

    return kErrorNoResource;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_exit(void)
{
    UINT    i = 0;

    instance_l.fInitialized = FALSE;

    NdisSetEvent(&instance_l.kernelWaitEvent);
    NdisSetEvent(&instance_l.userWaitEvent);

    while (instance_l.fThreadIsRunning)
    {
        NdisMSleep(10);
        if (i++ > 1000)
        {
            TRACE("Event Thread is not terminating, continue shutdown...!\n");
            break;
        }
    }

    ZwClose(instance_l.threadHandle);

    eventkcal_exitQueueCircbuf(kEventQueueK2U);
    eventkcal_exitQueueCircbuf(kEventQueueU2K);
    eventkcal_exitQueueCircbuf(kEventQueueUInt);
    eventkcal_exitQueueCircbuf(kEventQueueKInt);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post User event

This function posts a event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_postUserEvent(tEvent* pEvent_p)
{
    tOplkError    ret = kErrorOk;

    if (instance_l.fInitialized)
        ret = eventkcal_postEventCircbuf(kEventQueueK2U, pEvent_p);
    else
        ret = kErrorIllegalInstance;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_postKernelEvent(tEvent* pEvent_p)
{
    tOplkError    ret = kErrorOk;

    if (instance_l.fInitialized)
        ret = eventkcal_postEventCircbuf(kEventQueueKInt, pEvent_p);
    else
        ret = kErrorIllegalInstance;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process function of kernel CAL module

This function will be called by the systems process function.
*/
//------------------------------------------------------------------------------
void eventkcal_process(void)
{
    // Nothing to do, because we use threads
}

//------------------------------------------------------------------------------
/**
\brief    Post event from user

This function posts a event from the user layer to a queue.

\param  pEvent_p        Pointer to the event buffer to be posted.

\return The function returns Linux error code.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
void eventkcal_postEventFromUser(void* pEvent_p)
{
    tOplkError    ret = kErrorOk;
    tEvent        event;
    char*         pArg = NULL;

    if (!instance_l.fInitialized)
        return;

    OPLK_MEMCPY(&event, pEvent_p, sizeof(tEvent));

    if (event.eventArgSize != 0)
    {
        pArg = (char*)((UCHAR*) pEvent_p + sizeof(tEvent));
        event.eventArg.pEventArg = pArg;
    }

    switch (event.eventSink)
    {
        case kEventSinkSync:
        case kEventSinkNmtk:
        case kEventSinkDllk:
        case kEventSinkDllkCal:
        case kEventSinkPdok:
        case kEventSinkPdokCal:
        case kEventSinkErrk:
            /*TRACE("U2K  type:%s(%d) sink:%s(%d) size:%d!\n",
                   debugstr_getEventTypeStr(event.eventType), event.eventType,
                   debugstr_getEventSinkStr(event.eventSink), event.eventSink,
                   event.eventArgSize);*/
            ret = eventkcal_postEventCircbuf(kEventQueueU2K, &event);
            break;

        case kEventSinkNmtMnu:
        case kEventSinkNmtu:
        case kEventSinkSdoAsySeq:
        case kEventSinkApi:
        case kEventSinkDlluCal:
        case kEventSinkErru:
        case kEventSinkLedu:
            /*TRACE("UINT type:%s(%d) sink:%s(%d) size:%d!\n",
                   debugstr_getEventTypeStr(event.eventType), event.eventType,
                   debugstr_getEventSinkStr(event.eventSink), event.eventSink,
                   event.eventArgSize);*/
            ret = eventkcal_postEventCircbuf(kEventQueueUInt, &event);
            break;

        default:
            TRACE("UNKNOWN Event: Type:%s(%d) sink:%s(%d) size:%d!\n",
                  debugstr_getEventTypeStr(event.eventType), event.eventType,
                  debugstr_getEventSinkStr(event.eventSink), event.eventSink,
                  event.eventArgSize);
            break;
    }

    return;
}

//------------------------------------------------------------------------------
/**
\brief    Get an event for the user layer

This function waits for events to the user.

\param  pEvent_p         Buffer pointer to receive event.
\param  pSize_p          Size of the event.

\return The function returns Linux error code.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
void eventkcal_getEventForUser(void* pEvent_p, size_t* pSize_p)
{
    tOplkError    error;
    BOOL          fRet;
    size_t        readSize;
    UINT32        timeout = 500;

    if (!instance_l.fInitialized)
        return;

    fRet = NdisWaitEvent(&instance_l.userWaitEvent, timeout);

    if (fRet && (instance_l.userEventCount == 0))
    {
        NdisResetEvent(&instance_l.userWaitEvent);
        return;
    }

    if (!instance_l.fInitialized)
        return;

    NdisResetEvent(&instance_l.userWaitEvent);

    if (eventkcal_getEventCountCircbuf(kEventQueueK2U) > 0)
    {
        NdisInterlockedDecrement(&instance_l.userEventCount);

        error = eventkcal_getEventCircbuf(kEventQueueK2U, pEvent_p, pSize_p);
        if (error != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Error reading K2U events %d!\n", __func__, error);
            return;
        }

        if (pEvent_p == NULL)
        {
            return;
        }

        return;
    }
    else if (eventkcal_getEventCountCircbuf(kEventQueueUInt) > 0)
    {
        NdisInterlockedDecrement(&instance_l.userEventCount);

        error = eventkcal_getEventCircbuf(kEventQueueUInt, pEvent_p, pSize_p);
        if (error != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Error reading UINT events %d!\n", __func__, error);
            return;
        }
        //TRACE("%s() copy user event to user: %d Bytes\n", __func__, *pSize_p);
    }
    else
    {
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Event handler thread function

This function contains the main function for the event handler thread.

\param  pArg                     Thread parameter. Not used!

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static void eventThread(void* pArg)
{
    int         timeout = 50;
    PKTHREAD    thread;
    BOOL        fRet = FALSE;

    UNUSED_PARAMETER(pArg);

    // increase the priority of the thread
    thread = KeGetCurrentThread();
    KeSetPriorityThread(thread, HIGH_PRIORITY);

    instance_l.fThreadIsRunning = TRUE;

    while (instance_l.fInitialized)
    {
        fRet = NdisWaitEvent(&instance_l.kernelWaitEvent, timeout);
        if (!instance_l.fInitialized)
            break;

        NdisResetEvent(&instance_l.kernelWaitEvent);

        if (instance_l.kernelEventCount <= 0)
            continue;

        /* first handle all kernel internal events --> higher priority! */
        while (eventkcal_getEventCountCircbuf(kEventQueueKInt) > 0)
        {
            eventkcal_processEventCircbuf(kEventQueueKInt);
            NdisInterlockedDecrement(&instance_l.kernelEventCount);
        }

        if (eventkcal_getEventCountCircbuf(kEventQueueU2K) > 0)
        {
            eventkcal_processEventCircbuf(kEventQueueU2K);
            NdisInterlockedDecrement(&instance_l.kernelEventCount);
        }
    }

    instance_l.fThreadIsRunning = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Signal a user event

This function signals that a user event was posted. It will be registered in
the circular buffer library as signal callback function.
*/
//------------------------------------------------------------------------------
void signalUserEvent(void)
{
    NdisInterlockedIncrement(&instance_l.userEventCount);
    NdisSetEvent(&instance_l.userWaitEvent);
}

//------------------------------------------------------------------------------
/**
\brief  Signal a kernel event

This function signals that a kernel event was posted. It will be registered in
the circular buffer library as signal callback function.
*/
//------------------------------------------------------------------------------
void signalKernelEvent(void)
{
    NdisInterlockedIncrement(&instance_l.kernelEventCount);
    NdisSetEvent(&instance_l.kernelWaitEvent);
}

/// \}
