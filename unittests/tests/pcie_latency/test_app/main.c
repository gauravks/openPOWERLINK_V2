/**
********************************************************************************
\file   main.c

\brief  main file for Windows kernel module

This file contains the main part of the Windows kernel module implementation for
PCIe device.

\ingroup module_driver_ndispcie
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
#include <windows.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <limits.h>
#include <strsafe.h>

#include <conio.h>

#include "driver.h"
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
// Target IO functions
// - Write
#define OPLK_IO_WR8(addr, val)      (*(volatile UINT8*)(addr)) = (val)
#define OPLK_IO_WR16(addr, val)     (*(volatile UINT16*)(addr)) = (val)
#define OPLK_IO_WR32(addr, val)     (*(volatile UINT32*)(addr)) = (val)
// - Read
#define OPLK_IO_RD8(addr)           (*(volatile UINT8*)(addr))
#define OPLK_IO_RD16(addr)          (*(volatile UINT16*)(addr))
#define OPLK_IO_RD32(addr)          (*(volatile UINT32*)(addr))


#define BENCHMARK_SET(base, x)        OPLK_IO_WR8(base,((OPLK_IO_RD8(base)) | (1 << (x))))
#define BENCHMARK_RESET(base, x)      OPLK_IO_WR8(base,((OPLK_IO_RD8(base)) & ~(1 << (x))))
#define BENCHMARK_TOGGLE(base, x)     OPLK_IO_WR8(base,((OPLK_IO_RD8(base)) ^ (1 << (x))))
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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tBenchmarkMem    benchmarkMem_l;
static LARGE_INTEGER    startTime_l, endTime_l, frequency_l;
static ULONGLONG        Rxdif_l, Txdif_l, TxTotal_l = 0, RxTotal_l = 0, count_l = 0;
static LONGLONG         multiplier_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void DoIoctls(HANDLE fileHandle_p);
static int getUserBenchmarkBase(HANDLE fileHandle_p);

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
void main(__in ULONG argc,__in_ecount(argc) PCHAR argv[])
{
    HANDLE   fileHandle;
    LONG     errCode;
    BOOLEAN  fExit = FALSE;
    char     cKey;
    ULONG bytesReturned;

    UNREFERENCED_PARAMETER(argc);
    UNREFERENCED_PARAMETER(argv);
    // open POWERLINK device
    fileHandle = CreateFile(PLK_DEV_FILE,                         // Name of the NT "device" to open
                              GENERIC_READ | GENERIC_WRITE,         // Access rights requested
                              FILE_SHARE_READ | FILE_SHARE_WRITE,   // Share access - NONE
                              NULL,                                 // Security attributes - not used!
                              OPEN_EXISTING,                        // Device must exist to open it.
                              FILE_ATTRIBUTE_NORMAL,                // Open for overlapped I/O
                              NULL);                                // Extended attributes - not used!

    if (fileHandle == INVALID_HANDLE_VALUE)
    {
        errCode = GetLastError();
        printf("%s() CreateFile failed with error 0x%x\n", __FUNCTION__, errCode);
        return;
    }

    getUserBenchmarkBase(fileHandle);

    QueryPerformanceFrequency(&frequency_l);
    multiplier_l = 1000000000LL / frequency_l.QuadPart;

    printf("Frequency %lld Mult:%lld\n", frequency_l.QuadPart, multiplier_l);

    while (!fExit)
    {
        if (_kbhit())
        {
            cKey = (char) _getch();
            if (cKey == 0x1B)
                break;
        }

        DoIoctls(fileHandle);
        count_l++;
    }

    printf("Time taken for Rx %lld us  Tx %lld us in %lld TotalRx:%lld TTx:%lld\n", \
           (((RxTotal_l) / count_l)*multiplier_l) / 1000, \
           (((TxTotal_l) / count_l)*multiplier_l) / 1000, count_l, \
           (((RxTotal_l))*multiplier_l) / 1000, \
           (((TxTotal_l))*multiplier_l) / 1000);

    DeviceIoControl(fileHandle,
                          (DWORD) PLK_CMD_PRINT_STATS,
                          &count_l,
                          sizeof(count_l),
                          NULL,
                          0,
                          &bytesReturned,
                          NULL
                          );

    CloseHandle(fileHandle);
    return;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{
static void DoIoctls(HANDLE fileHandle_p)
{
    char OutputBuffer[1500];
    char InputBuffer[1500];
    BOOL bRc;
    ULONG bytesReturned;

    // Printing Input & Output buffer pointers and size

    //printf("InputBuffer Pointer = %p, BufLength = %d\n", InputBuffer,
    //       sizeof(InputBuffer));
    //printf("OutputBuffer Pointer = %p BufLength = %d\n", OutputBuffer,
    ///       sizeof(OutputBuffer));

    //
    // Performing METHOD_TX
    //

   // printf("\nCalling DeviceIoControl METHOD_TX\n");

    //if (FAILED(StringCchCopy(InputBuffer, sizeof(InputBuffer),
        //"this String is from User Application; using METHOD_TX"))) {
        //return;
    //}

    memset(InputBuffer, 0xFA, 1500);
    //printf("Data in TX:%s\n", InputBuffer);

    BENCHMARK_SET(benchmarkMem_l.pBaseAddr, 0);
    QueryPerformanceCounter(&startTime_l);
    bRc = DeviceIoControl(fileHandle_p,
                          (DWORD) PLK_CMD_WRITE,
                          InputBuffer,
                          1500,
                          OutputBuffer,
                          1500,
                          &bytesReturned,
                          NULL
                          );
    BENCHMARK_RESET(benchmarkMem_l.pBaseAddr, 0);
    QueryPerformanceCounter(&endTime_l);

    Txdif_l = (endTime_l.QuadPart - startTime_l.QuadPart);
    TxTotal_l += Txdif_l;

    if (!bRc)
    {
        printf("Error in DeviceIoControl : : %d\n", GetLastError());
        return;
    }

   // printf("    Number of bytes transfered from OutBuffer: %d\n",
    //       bytesReturned);

    //
    // Performing METHOD_RX
    //

    //printf("\nCalling DeviceIoControl METHOD_RX\n");

    memset(OutputBuffer, 0, sizeof(OutputBuffer));
    QueryPerformanceCounter(&startTime_l);
    BENCHMARK_SET(benchmarkMem_l.pBaseAddr, 1);
    bRc = DeviceIoControl(fileHandle_p,
                          (DWORD) PLK_CMD_READ,
                          InputBuffer,
                          1500,
                          OutputBuffer,
                          1500,
                          &bytesReturned,
                          NULL
                          );
    QueryPerformanceCounter(&endTime_l);

    BENCHMARK_RESET(benchmarkMem_l.pBaseAddr, 1);

    Rxdif_l = (endTime_l.QuadPart - startTime_l.QuadPart);
    RxTotal_l += Rxdif_l;

    if (!bRc)
    {
        printf("Error in DeviceIoControl : : %d", GetLastError());
        return;
    }

    //printf("    OutBuffer (%d): \n", bytesReturned);

    return;

}

//------------------------------------------------------------------------------
/**
\brief  Return the user benchmark base address

The funtion returns the base address for user benchamark situated on external
device such as FPGA.

\return The function returns the base address for benchmark.

*/
//------------------------------------------------------------------------------
static int getUserBenchmarkBase(HANDLE fileHandle_p)
{
    ULONG   bytesReturned;
    if (benchmarkMem_l.pBaseAddr != NULL)
        return 0;

    if (!DeviceIoControl(fileHandle_p, PLK_CMD_GET_BENCHMARK,
        0, 0, &benchmarkMem_l, sizeof(tBenchmarkMem),
        &bytesReturned, NULL))
    {
        printf("Failed to get benchmark base\n");
        return -1;
    }

    if (bytesReturned == 0 || benchmarkMem_l.pBaseAddr == NULL)
        return -1;

    return 0;
}
///\}