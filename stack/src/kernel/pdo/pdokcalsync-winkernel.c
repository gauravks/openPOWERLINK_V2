/**
********************************************************************************
\file   pdokcalsync-winkernel.c

\brief  PDO CAL kernel sync module using the Windows kernel driver

This file contains the implementation for kernel PDO CAL synchronization module
for Windows kernel. The synchronization module is responsible for notifying
the user layer of presence of new data.

The module uses NDIS events to get notification of new data and pass it to the
user layer by completing pended IOCTLs.

\ingroup module_pdokcal
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
#include <common/pdo.h>

#include <ndisintermediate/ndis-intf.h>
#include <ndis.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PDO_TAG    'odpO'

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
/brief  PDO synchronization module instance structure

Local variables and flags used by PDO synchronization module.

*/
typedef struct
{
    NDIS_EVENT        syncWaitEvent;    ///< NDIS event for synchronization events.
    BOOL              fInitialized;     ///< Flag to identify initialization status of module.
} tPdokCalSyncInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdokCalSyncInstance*   instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel PDO CAL sync module

The function initializes the kernel PDO CAL sync module.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_initSync(void)
{
    NDIS_HANDLE    adapterHandle = ndis_getAdapterHandle();

    instance_l = NdisAllocateMemoryWithTagPriority(adapterHandle, sizeof(tPdokCalSyncInstance),
                                                   PDO_TAG, NormalPoolPriority);

    if (instance_l == NULL)
        return kErrorNoResource;

    NdisInitializeEvent(&instance_l->syncWaitEvent);

    instance_l->fInitialized = TRUE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO CAL sync module

The function cleans up the PDO CAL sync module

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
void pdokcal_exitSync(void)
{
    if (instance_l != NULL)
        NdisFreeMemory(instance_l, 0, 0);

    instance_l->fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Send a sync event

The function sends a sync event

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_sendSyncEvent(void)
{
    if (instance_l == NULL)
        return kErrorNoResource;

    if (instance_l->fInitialized == TRUE)
    {
        NdisSetEvent(&instance_l->syncWaitEvent);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_waitSyncEvent(void)
{
    int        timeout = 1000;
    BOOLEAN    fRet;

    if (!instance_l->fInitialized)
    {
        return kErrorNoResource;
    }

    fRet = NdisWaitEvent(&instance_l->syncWaitEvent, timeout);

    if (fRet)
    {
        NdisResetEvent(&instance_l->syncWaitEvent);
    }
    else
    {
        return kErrorRetry;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Enable sync events

The function enables sync events

\param  fEnable_p        enable/disable sync event.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_controlSync(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}

