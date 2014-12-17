/**
********************************************************************************
\file   windows/lock-dualprocnoos.c

\brief  Locks for dual processor system with Windows

This target depending module provides lock functionality in dual processor
system with shared memory where one of the processor hosts Windows operating
system and the other as non-OS.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited.

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
#include "lock.h"

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
#define LOCK_LOCAL_ID    (2)        // The non-OS processor should use 1 as the ID

#if (LOCK_LOCAL_ID == LOCK_UNLOCKED_C)
#error "Change the to LOCK_LOCAL_ID to some unique BYTE value unequal 0x0!"
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static LOCK_T*   pLock_l = NULL;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initializes given lock

This function initializes the lock instance.

\param  pLock_p                Reference to lock

\return The function returns 0 when successful.

\ingroup module_target
*/
//------------------------------------------------------------------------------
int target_initLock(LOCK_T* pLock_p)
{
    if (pLock_p == NULL)
        return -1;

    pLock_l = pLock_p;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief    Lock the given lock

This function tries to lock the given lock, otherwise it spins until the
lock is freed.

\return The function returns 0 when successful.

\ingroup module_target
*/
//------------------------------------------------------------------------------
int target_lock(void)
{
    USHORT    val;

    if (pLock_l == NULL)
        return -1;

    // spin if id is not written to shared memory
    do
    {
        val = READ_UCHAR(pLock_l);
        // write local id if unlocked
        if (val == LOCK_UNLOCKED_C)
        {
            WRITE_UCHAR(pLock_l, LOCK_LOCAL_ID);
            continue; // return to top of loop to check again
        }
    } while (val != LOCK_LOCAL_ID);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief    Unlock the given lock

This function frees the given lock.

\return The function returns 0 when successful.

\ingroup module_target
*/
//------------------------------------------------------------------------------
int target_unlock(void)
{
    if (pLock_l == NULL)
        return -1;
    WRITE_UCHAR(pLock_l, LOCK_UNLOCKED_C);
    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
