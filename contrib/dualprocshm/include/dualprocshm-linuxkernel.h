/**
********************************************************************************
\file   dualprocshm-linuxkernel.h

\brief  Dual Processor Library Target support Header - For Linux Kernel target

This header file provides specific macros for Linux Kernel CPU.

*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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


#ifndef _INC_dualprocshm_linuxkernel_H_
#define _INC_dualprocshm_linuxkernel_H_


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <asm/io.h>
//#include <linux/delay.h>
#include <linux/slab.h>

// include section header file for special functions in
// tightly-coupled memory
//#include <section-arm.h>//TODO@gks: Check if this is needed with no TCM

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


/// memory
#define DUALPROCSHM_MALLOC(size)            kzalloc(size,GFP_KERNEL)
#define DUALPROCSHM_FREE(ptr)               kfree(ptr)

/// sleep
#define DUALPROCSHM_USLEEP(x)               usleep((UINT32)x)

/// IO operations
#define DPSHM_READ8(base)                   readb((UINT8*)base);
#define DPSHM_WRITE8(base,val)              writeb(val,(UINT8*)base);
#define DPSHM_READ16(base)                  readw((UINT8*)base);
#define DPSHM_WRITE16(base,val)             writew(val,(UINT8*)base);

/// cache handling
#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base,range)

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base,range)

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------


#endif /* _INC_DUALPROCSHM_ARM_H_ */

