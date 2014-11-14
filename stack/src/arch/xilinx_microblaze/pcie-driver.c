/**
********************************************************************************
\file   pcie-driver.c

\brief  PCIe driver for Xilinx AXI PCIe IP core

This file implements the initialization routines for Xilinx AXI PCIe IP core.

_NOTE_: The initialization logic is based on the xaxipcie_ep_enable_example.c
        which gives an example application to initialize and use AXI PCIe IP.

\ingroup module_target
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
#include <xaxipcie.h>
#include <stdio.h>
#include <xgpio.h>
#include <xintc.h>

#include "pcie-driver.h"


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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static XAxiPcie endPoint;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
int pcie_initialize(void)
{
    int                 status;
    XAxiPcie_Config*    pPcieConfig;

    // Initialize the driver
    pPcieConfig = XAxiPcie_LookupConfig(PCIE_DEVICE_ID);

    if(pPcieConfig == NULL)
    {
        printf("%s() Unable to initialize PCIe End Point Instance\n");
        return XST_FAILURE;
    }

    status = XAxiPcie_CfgInitialize(&endPoint, pPcieConfig, pPcieConfig->BaseAddress);
    if(status != XST_SUCCESS)
    {
        printf("%s() Failed to Configure End Point\n");
        return XST_FAILURE;
    }

    // Disable all the Interrupts
    XAxiPcie_DisableInterrupts(&endPoint, XAXIPCIE_IM_ENABLE_ALL_MASK);

    printf("Check for PCIe LINK....");

    // Make sure link is up.
    status = XAxiPcie_IsLinkUp(&endPoint);
    if(status)
    {
        // Link is Up, check if device configured
        printf("PCIe Link is UP\n");
        configureEndPoint();
        return XST_SUCCESS;
    }

    // Link is not UP
    printf("\n***********REBOOT HOST***********\n");
    printf("Check for PCIe LINK Again....");
    while(!status)
    {
        status = XAxiPcie_IsLinkUp(&endPoint);
    }

    printf("PCIe Link is UP\n");
    configureEndPoint();
    return XST_SUCCESS;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

void configureEndPoint(void)
{
    int     status;
    u32     configData;
    u8      busNum, devNum, funcNum, portNum;

    printf("Checking End Point configuration Status.....\n");

    XAxiPcie_ReadLocalConfigSpace(&endPoint, PCIE_CFG_CMD_STATUS_REG,
                                                    &configData);

    if ((configData & PCIE_CFG_CMD_BUSM_EN) == 0)
    {
        printf("End point Not configured yet by root complex, REBOOT THE HOST\n");

        while(!(configData & PCIE_CFG_CMD_BUSM_EN))
        {
            XAxiPcie_ReadLocalConfigSpace(&endPoint, PCIE_CFG_CMD_STATUS_REG,
                                                                &configData);
        }
    }

    printf("End point configured Successfully\n");

    XAxiPcie_GetRequesterId(&endPoint, &busNum, &devNum, &funcNum, &portNum);

    printf("PCIe device configured on Bus Number %02X\r\n"
                "with Device Number %02X\r\n"
                "as Function Number %02X\r\n"
                "at Port Number %02X\r\n",
                busNum, devNum, funcNum, portNum);

    // Read the configuration space
    XAxiPcie_ReadLocalConfigSpace(&endPoint, PCIE_CFG_ID_REG, &configData);
    printf("PCIe Vendor ID/Device ID Register is %08X\r\n", configData);

    XAxiPcie_ReadLocalConfigSpace(&endPoint, PCIE_CFG_CMD_STATUS_REG, &configData);
    printf("PCIe Command/Status Register is %08X\r\n", configData);

    XAxiPcie_ReadLocalConfigSpace(&endPoint, PCIE_CFG_CAH_LAT_HD_REG, &configData);
    printf("PCIe Header Type/Latency Timer Register is %08X\r\n", configData);


    XAxiPcie_ReadLocalConfigSpace(&endPoint, PCIE_CFG_BAR_ZERO_REG, &configData);
    printf("PCIe BAR 0 is at physical address %08X\r\n", configData);

    XAxiPcie_ReadLocalConfigSpace(&endPoint, PCIE_CFG_BAR_ONE_REG, &configData);
    printf("PCIe BAR 1 is at physical address %08X\r\n", configData);

}
///\}

