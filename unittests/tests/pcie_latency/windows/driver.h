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

#ifndef _INC_driver_H_
#define _INC_driver_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#ifndef _KERNEL_MODE
#include <winioctl.h>
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_DEV_FILE      "\\\\.\\plk"
#define PLK_DEV_STRING    L"\\Device\\plk"
#define PLK_LINK_NAME     L"\\DosDevices\\Global\\plk"
//------------------------------------------------------------------------------
//  Commands for <ioctl>
//------------------------------------------------------------------------------

#define PLK_IO_TYPE                    50001

#define PLK_CMD_READ          CTL_CODE(PLK_IO_TYPE, 0x901, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_WRITE         CTL_CODE(PLK_IO_TYPE, 0x902, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_GET_BENCHMARK CTL_CODE(PLK_IO_TYPE, 0x903, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PLK_CMD_PRINT_STATS   CTL_CODE(PLK_IO_TYPE, 0x904, METHOD_BUFFERED, FILE_ANY_ACCESS)

//------------------------------------------------------------------------------
// module global defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Benchmark memory struture

The structure is used to retrieve the benchmark address.
*/
typedef struct
{
    void*                   pBaseAddr;       ///< Pointer to the benchmark address returned by kernel
} tBenchmarkMem;
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_driver_H_ */
