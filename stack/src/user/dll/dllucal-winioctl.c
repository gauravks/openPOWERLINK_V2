/**
********************************************************************************
\file   dllucal-winioctl.c

\brief  Source file for user DLL CAL module - Windows User-Kernel

This file contains the implementation of dll CAL module on a Windows which
uses ioctl to communicate between user and kernel.

\ingroup module_dllucal
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
#include <common/dllcal.h>
#include <user/ctrlucal.h>
#include <common/driver.h>

//#include "{LOCAL_INCLUDE_FILE}"


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
\brief DLL CAL ioctl instance type

This struct is used for every event queue using the Windows ioctl for
event posting.
*/
typedef struct
{
    tDllCalQueue                dllCalQueue;        ///< DLL CAL queue
    tDllAsyncReqPriority        priority;           ///< Request priority
    HANDLE                      fileHandle;         ///< File handle of openPOWERLINK driver
} tDllCalIoctlInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError addInstance(tDllCalQueueInstance* ppDllCalQueue_p, tDllCalQueue DllCalQueue_p);
static tOplkError delInstance(tDllCalQueueInstance pDllCalQueue_p);
static tOplkError insertDataBlock(tDllCalQueueInstance pDllCalQueue_p, UINT8* pData_p, UINT* pDataSize_p);

/* define external function interface */
static tDllCalFuncIntf funcintf_l =
{
    addInstance,
    delInstance,
    insertDataBlock,
    NULL,
    NULL,
    NULL
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Return pointer to function interface

This function returns a pointer to the function interface structure which
is used to access the dllcal functions of the shared buffer implementation.

\return Returns a pointer to the local function interface

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tDllCalFuncIntf* dllcalioctl_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Add DLL CAL instance

Add an instance for TX packet forwarding in DLL CAL.

\param  ppDllCalQueue_p         Double-pointer to DllCal Queue instance
\param  dllCalQueue_p           Parameter that determines the queue

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other                   Error
*/
//------------------------------------------------------------------------------
static tOplkError addInstance(tDllCalQueueInstance* ppDllCalQueue_p,
                              tDllCalQueue dllCalQueue_p)
{
    tDllCalIoctlInstance*       pInstance;

    pInstance = (tDllCalIoctlInstance*) OPLK_MALLOC(sizeof(tDllCalIoctlInstance));
    if (pInstance == NULL)
    {
        return kErrorNoResource;
    }

    //store parameters in instance
    pInstance->dllCalQueue = dllCalQueue_p;
    pInstance->fileHandle = ctrlucal_getFd();

    *ppDllCalQueue_p = (tDllCalQueueInstance*) pInstance;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Delete DLL CAL instance

Delete the DLL CAL instance.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other                   Error
*/
//------------------------------------------------------------------------------
static tOplkError delInstance(tDllCalQueueInstance pDllCalQueue_p)
{
    tDllCalIoctlInstance*     pInstance = (tDllCalIoctlInstance*) pDllCalQueue_p;

    OPLK_FREE(pInstance);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Insert data block into queue

Inserts a data block into the DLL CAL queue.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pData_p                 Pointer to the data block to be inserted
\param  pDataSize_p             Pointer to the size of the data block to be
                                insert

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other                   Error
*/
//------------------------------------------------------------------------------
static tOplkError insertDataBlock(tDllCalQueueInstance pDllCalQueue_p,
                                  UINT8* pData_p, UINT* pDataSize_p)
{
    tOplkError                      ret = kErrorOk;
    tDllCalIoctlInstance*           pInstance =
                                         (tDllCalIoctlInstance*) pDllCalQueue_p;
    UINT8*                          ioctlAsyncBuf;
    tIoctlDllCalAsync               ioctlAsyncFrame;
    BOOLEAN                         ioctlRet;
    ULONG                           bytesReturned;

    if (pInstance == NULL)
    {
        return kErrorInvalidInstanceParam;
    }

    ioctlAsyncBuf = OPLK_MALLOC(sizeof(tIoctlDllCalAsync) + *pDataSize_p);

    if (ioctlAsyncBuf == NULL)
    {
        return kErrorNoResource;
    }

    ioctlAsyncFrame.size = *pDataSize_p;
    ioctlAsyncFrame.queue = pInstance->dllCalQueue;

    OPLK_MEMCPY(ioctlAsyncBuf, &ioctlAsyncFrame, sizeof(tIoctlDllCalAsync));
    OPLK_MEMCPY((ioctlAsyncBuf + sizeof(tIoctlDllCalAsync)), pData_p, *pDataSize_p);
    //TRACE ("%s() send async frame: size:%d\n", __func__, pFrameInfo_p->frameSize);
    ioctlRet = DeviceIoControl(pInstance->fileHandle, PLK_CMD_DLLCAL_ASYNCSEND,
                               ioctlAsyncBuf, (sizeof(tIoctlDllCalAsync) +*pDataSize_p),
                          0, 0,
                          &bytesReturned, NULL);
    if (ioctlRet == 0)
        return kErrorDllAsyncTxBufferFull;

    return kErrorOk;
}

///\}

