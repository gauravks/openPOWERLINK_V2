/**
********************************************************************************
\file   pcie-driver.h

\brief  Xilinx AXI PCIe IP driver - Header file

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

#ifndef _INC_pcie-driver_H_
#define _INC_pcie-driver_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <xparameters.h>
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PCIE_DEVICE_ID              XPAR_AXIPCIE_0_DEVICE_ID

// Offsets
#define PCIE_CFG_ID_REG             0x0000      // Vendor ID/Device ID
#define PCIE_CFG_CMD_STATUS_REG     0x0001      // Command/Status Register
#define PCIE_CFG_CAH_LAT_HD_REG     0x0003      // Cache Line/Latency
#define PCIE_CFG_BAR_ZERO_REG       0x0004      // Bar 0 offset
#define PCIE_CFG_BAR_ONE_REG        0x0005      // Bar 1 offset
#define PCIE_CFG_CMD_BUSM_EN        0x00000004  // Bus master enable
#define PCIE_CFG_CMD_MEM_EN         0x00000002  // Memory access enable
#define PCIE_CFG_CMD_IO_EN          0x00000001  // I/O access enable
#define PCIE_CFG_CMD_PARITY         0x00000040  // parity errors response
#define PCIE_CFG_CMD_SERR_EN        0x00000100  // SERR report enable */
#define PCIE_CFG_PRIM_SEC_BUS       0xFFFF0100  //
#define PCIE_CFG_PRI_SEC_BUS_REG    0x0006
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
int     pcie_initialize(void);
#ifdef __cplusplus
}
#endif

#endif /* _INC_pcie-driver_H_ */
