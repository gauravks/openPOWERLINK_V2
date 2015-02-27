/**
********************************************************************************
\file   drv_ndis_pcie/drvintf.h

\brief  Driver interface header file

Driver interface for the kernel daemon - Header file

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

#ifndef _INC_drvintf_H_
#define _INC_drvintf_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include "driver.h"

//------------------------------------------------------------------------------
// module global defines
//------------------------------------------------------------------------------
extern UINT8*               benchmarkBase_g;          ///< Pointer to user benchmark

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/*
/brief File context for user application

This struture contains lock to rundown threads that are dispatching I/Os on
driver file handle while the cleanup is in progress.
*/
typedef struct
{
    IO_REMOVE_LOCK    driverAccessLock;     ///< Driver lock for IO access.
} tFileContext;


//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef NDEBUG
#define TRACE(...)    DbgPrint(__VA_ARGS__)
#else
#define TRACE(...)
#endif

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
    int drv_getBenchmarkMem(UINT8** ppBenchmarkMem_p);
#ifdef __cplusplus
}
#endif

#endif /* _INC_drvintf_H_ */
