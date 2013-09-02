/**
********************************************************************************
\file   target-arm.c

\brief  target specific functions for ARM on Zynq without OS

This target depending module provides several functions that are necessary for
systems without shared buffer and any OS.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Kalycito Infotech Pvt Ltd, Coimbatore
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
/*
 *  Created on: May 17, 2013       Author: Chidrupaya S
 */

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <global.h>
#include <xscugic.h>
#include <xtime_l.h>
#include "systemComponents.h"
#include "xil_cache.h"
#include "xil_types.h"
#include "xscugic.h"
#include "xil_io.h"
#include "xil_exception.h"
#include <unistd.h>


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
#define TGTCONIO_MS_IN_US(x)    (x*1000U)

#define HOSTIF_MAGIC            0x504C4B00
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    returns current system tick

This function returns the current system tick determined by the system timer.

\return system tick
\retval DWORD

\ingroup module_target
*/
//------------------------------------------------------------------------------
DWORD PUBLIC EplTgtGetTickCountMs(void)
{
    DWORD dwTicks;
    XTime* ticks;
    /*Uses global timer functions*/

    XTime_GetTime(ticks);
    /*Select the lower 32 bit of the timer value*/
    dwTicks = (DWORD)(((2000 * (*ticks))/XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ));

    return dwTicks;
}

//------------------------------------------------------------------------------
/**
\brief    enables global interrupt

This function enabels/disables global interrupts.

\param  fEnable_p               TRUE = enable interrupts
                                FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void EplTgtEnableGlobalInterrupt (BYTE fEnable_p)
{
    if(fEnable_p == TRUE)
    {
        SysComp_enableInterrupts();
    }
    else
    {
        SysComp_disableInterrupts();
    }
}

//------------------------------------------------------------------------------
/**
\brief    checks if CPU is in interrupt context

This function obtains if the CPU is in interrupt context.

\return CPU in interrupt context
\retval TRUE                    CPU is in interrupt context
\retval FALSE                   CPU is NOT in interrupt context

\ingroup module_target
*/
//------------------------------------------------------------------------------
BYTE EplTgtIsInterruptContext (void)
{
    // No real interrupt context check is performed.
    // This would be possible with a flag in the ISR, only.
    // For now, the global interrupt enable flag is checked.

    // Read the distributor state
    u32 Distributor_state = Xil_In32(XPAR_PS7_SCUGIC_0_DIST_BASEADDR + XSCUGIC_DIST_EN_OFFSET);
    // Read the DP (Distributor) and CP (CPU interface) state
    u32 CPUif_state = Xil_In32(XPAR_SCUGIC_0_CPU_BASEADDR + XSCUGIC_CONTROL_OFFSET);

    if(Distributor_state && CPUif_state)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
//------------------------------------------------------------------------------
/**
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel target_init(void)
{
u32 version = 0;
#if defined(__arm__)

    Xil_DCacheFlush();
    SysComp_initPeripheral();

    return kEplSuccessful;
#else
    // Add Here any other platform specific code
	return kEplSuccessful;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup target specific stuff

The function cleans-up target specific stuff.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel target_cleanup(void)
{
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  mseconds_p              Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep (unsigned int milliSecond_p)
{
    usleep(TGTCONIO_MS_IN_US(milliSecond_p));
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//