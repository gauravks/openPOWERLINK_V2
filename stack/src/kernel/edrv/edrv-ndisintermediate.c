/**
********************************************************************************
\file   edrv-ndisintermediate.c

\brief  Implementation of edrv module using NDIS intermediate driver interface

This file contains the implementation of Ethernet driver in windows kernel which
uses NDIS interface to communicate to native miniport driver. This driver will
use the protocol access points of NDIS intermediate driver to communicate with
lower drivers and exchange packets.

//TODO:gks complete description for the driver

\ingroup module_edrv
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
#include <oplk/oplkinc.h>
#include <common/ami.h>
#include <kernel/edrv.h>

#include <ndisintermediate/ndis-intf.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS    42
#endif

#ifndef EDRV_MAX_TX_QUEUE
#define EDRV_MAX_TX_QUEUE      16
#define EDRV_TX_QUEUE_MASK     (EDRV_MAX_TX_QUEUE - 1)
#endif

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS    256
#endif

#ifndef EDRV_MAX_RX_DESCS
#define EDRV_MAX_RX_DESCS      16
#define EDRV_RX_DESC_MASK      (EDRV_MAX_RX_DESCS - 1)
#endif

#define EDRV_MAX_FRAME_SIZE    0x600

#define EDRV_TX_BUFFER_SIZE    (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE)      // n * (MTU + 14 + 4)

#define EDRV_MIN_FRAME_SIZE    60

#define EDRV_NDIS_MEM_TAG      'klpO'
#define EDRV_ALLOCATED_NBL     0x10000000

#define EDRV_MALLOC(_pVar, _Size)    NdisAllocateMemoryWithTag((PVOID*)(&_pVar), \
                                                               (_Size), EDRV_NDIS_MEM_TAG)

#define EDRV_FREE(_pMem)             NdisFreeMemory(_pMem, 0, 0)
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

typedef struct
{
    void*             pTxBuff;
    UINT              headTxIndex;
    UINT              tailTxIndex;
    tEdrvTxBuffer*    apTxBuffer[EDRV_MAX_TX_BUFFERS];              ///< Array of TX buffers
    BOOL              afTxBufUsed[EDRV_MAX_TX_BUFFERS];             ///< Array indicating the use of a specific TX buffer
    tEdrvInitParam    initParam;

#if CONFIG_EDRV_USE_DIAGNOSTICS != FALSE
    ULONGLONG         interruptCount;
    INT               rxBufFreeMin;
    UINT              rxCount[EDRV_SAMPLE_NUM];
    UINT              txCount[EDRV_SAMPLE_NUM];
    UINT              pos;
#endif
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

tEdrvInstance    edrvInstance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void edrvTxHandler(void* txBuff_p);
static void edrvRxHandler(void* pRxBuffer_p, size_t size_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver initialization

This function initializes the Ethernet driver.

\param  pEdrvInitParam_p    Edrv initialization parameters

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_init(tEdrvInitParam* pEdrvInitParam_p)
{
    tOplkError    ret = kErrorOk;

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    DbgPrint("Edrv Init\n");
    // save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    // Check if NDIS intermediate driver is ready
    if (!ndis_checkBindingState())
    {
        // The ndis driver has not initialized
        DbgPrint("%s() NDIS Driver not initialized\n", __func__);
        return kErrorEdrvInit;
    }

    // Allocate and prepare Transmit and receive Net Buffer Lists
    ret = ndis_allocateTxRxBuff(EDRV_MAX_TX_BUFFERS, EDRV_MAX_RX_BUFFERS);
    if (ret != kErrorOk)
    {
        DbgPrint("%s() TX and RX buffer allocation failed 0x%X\n", __func__, ret);
        return ret;
    }
    //    edrvInstance_l.pTxBuff =  ndis_getTxBuf();
    // Register Tx and Rx callbacks
    ndis_registerTxRxHandler(edrvTxHandler, edrvRxHandler);

    ndis_getMacAddress(pEdrvInitParam_p->aMacAddr);

    ndis_setBindingState(NdisBindingRunning);
    // TODO@gks: Get the MAC address here and assign to init params
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver shutdown

This function shuts down the Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_shutdown(void)
{
    DbgPrint("%s() \n", __func__);
    ndis_setBindingState(NdisBindingReady);
    ndis_freeTxRxBuff();
    ndis_registerTxRxHandler(NULL, NULL);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\param  pMacAddr_p  Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(UINT8* pMacAddr_p)
{
    // TODO@gks check if this is possible?
    UNUSED_PARAMETER(pMacAddr_p);
    // USE OID_802_3_ADD_MULTICAST_ADDRESS along with OID_GEN_CURRENT_PACKET_FILTER
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clear multicast address entry

This function removes the multicast entry from the Ethernet controller.

\param  pMacAddr_p  Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(UINT8* pMacAddr_p)
{
    UNUSED_PARAMETER(pMacAddr_p);
    // Use  OID_802_3_DELETE_MULTICAST_ADDRESS along with OID_GEN_CURRENT_PACKET_FILTER
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Change Rx filter setup

This function changes the Rx filter setup. The parameter entryChanged_p
selects the Rx filter entry that shall be changed and \p changeFlags_p determines
the property.
If \p entryChanged_p is equal or larger count_p all Rx filters shall be changed.

\note Rx filters are not supported by this driver!

\param  pFilter_p           Base pointer of Rx filter array
\param  count_p             Number of Rx filter array entries
\param  entryChanged_p      Index of Rx filter entry that shall be changed
\param  changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_changeRxFilter(tEdrvFilter* pFilter_p, UINT count_p,
                               UINT entryChanged_p, UINT changeFlags_p)
{
    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(count_p);
    UNUSED_PARAMETER(entryChanged_p);
    UNUSED_PARAMETER(changeFlags_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    UINT                bufIndex;
    tNdisErrorStatus    ndisStatus;
    void*               pTxBuffer = NULL;
    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        return kErrorEdrvNoFreeBufEntry;
    }

    //if (edrvInstance_l.pTxBuff == NULL)
    //{
    //   DEBUG_LVL_ERROR_TRACE("%s Tx buffers currently not allocated\n", __FUNCTION__);
    //   return kErrorEdrvNoFreeBufEntry;

    //}
    ndisStatus = ndis_getTxBuff(&pTxBuffer, pBuffer_p->maxBufferSize,
                                &pBuffer_p->txBufferNumber.pArg);
    if (ndisStatus != NdisStatusSuccess)
    {
        DEBUG_LVL_ERROR_TRACE("%s Tx buffers currently not allocated %x\n", __func__, ndisStatus);
        return kErrorEdrvNoFreeBufEntry;
    }

    if (pTxBuffer != NULL && pBuffer_p->txBufferNumber.pArg)
    {
        pBuffer_p->pBuffer = (UINT8*) pTxBuffer;
    }
    else
    {
        DbgPrint("Now What Happened\n");
    }
    /*    for(bufIndex = 0; bufIndex < EDRV_MAX_TX_BUFFERS; bufIndex++)
        {
            if(!edrvInstance_l.afTxBufUsed[bufIndex])
            {
                edrvInstance_l.afTxBufUsed[bufIndex] = TRUE;
                pBuffer_p->pBuffer = (UINT8*)edrvInstance_l.pTxBuff + (bufIndex * EDRV_MAX_FRAME_SIZE);
                pBuffer_p->maxBufferSize = EDRV_MAX_FRAME_SIZE;
                pBuffer_p->txBufferNumber.value = bufIndex;
            }
        }

        if (bufIndex >= EDRV_MAX_TX_BUFFERS)
        {
            return kErrorEdrvNoFreeBufEntry;
        }
    */
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This function releases the Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    ndis_freeTxBuff(pBuffer_p->txBufferNumber.pArg);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function sends the Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError    ret = kErrorOk;
    UINT          bufferNumber;

    //DbgPrint("%s() \n", __func__);
    //bufferNumber = pBuffer_p->txBufferNumber.value;

    if ((pBuffer_p->txFrameSize > EDRV_MAX_FRAME_SIZE))
    {
        return kErrorEdrvInvalidParam;
    }

    // DbgPrint("1\n");
    if (((edrvInstance_l.tailTxIndex + 1) & EDRV_TX_QUEUE_MASK) == edrvInstance_l.headTxIndex)
    {
        return kErrorEdrvNoFreeTxDesc;
    }

    //DbgPrint("2\n");
    edrvInstance_l.apTxBuffer[edrvInstance_l.tailTxIndex] = pBuffer_p;

    if (pBuffer_p->txFrameSize < EDRV_MIN_FRAME_SIZE)
    {
        pBuffer_p->txFrameSize = EDRV_MIN_FRAME_SIZE;
    }

    //DbgPrint("Send Packet\n");
    ndis_sendPacket(pBuffer_p, pBuffer_p->txFrameSize, pBuffer_p->txBufferNumber.pArg);

    // increment Tx queue tail pointer
    edrvInstance_l.tailTxIndex = (edrvInstance_l.tailTxIndex + 1) & EDRV_TX_QUEUE_MASK;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set Tx buffer ready

This function sets the Tx buffer buffer ready for transmission.

\param  pBuffer_p   Tx buffer buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setTxBufferReady(tEdrvTxBuffer* pBuffer_p)
{
    UNUSED_PARAMETER(pBuffer_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Start ready Tx buffer

This function sends the Tx buffer marked as ready.

\param  pBuffer_p   Tx buffer buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_startTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    UNUSED_PARAMETER(pBuffer_p);

    return kErrorOk;
}

#if CONFIG_EDRV_USE_DIAGNOSTICS != FALSE
//------------------------------------------------------------------------------
/**
\brief  Get Edrv module diagnostics

This function returns the Edrv diagnostics to a provided buffer.

\param  pBuffer_p   Pointer to buffer filled with diagnostics.
\param  size_p      Size of buffer

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
INT edrv_getDiagnostics(char* pBuffer_p, INT size_p)
{
}

#endif
//------------------------------------------------------------------------------
/**
\brief  Release Rx buffer

This function releases a late release Rx buffer.

\param  pRxBuffer_p     Rx buffer to be released

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_releaseRxBuffer(tEdrvRxBuffer* pRxBuffer_p)
{
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{
//------------------------------------------------------------------------------
/**
\brief  Receive Handler routine

Receive handler registered with NDIS intermediate driver. Intermediate driver
invokes receive from the NetBufferLists Receive handler.

\param  pRxBuffer_p         A pointer to the Receive buffer.
\param  size_p              Size of the received data.

*/
//------------------------------------------------------------------------------
static void edrvRxHandler(void* pRxData_p, size_t size_p)
{
    tEdrvRxBuffer           rxBuffer;
    tEdrvReleaseRxBuffer    retReleaseRxBuffer;
    //DbgPrint("%s() \n", __func__);
    rxBuffer.pBuffer = pRxData_p;
    rxBuffer.rxFrameSize = size_p;
    rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;

    retReleaseRxBuffer = edrvInstance_l.initParam.pfnRxHandler(&rxBuffer);
}

//------------------------------------------------------------------------------
/**
\brief  Transmit complete Handler routine

Transmit complete handler regstered with NDIS intermediate driver.

\param  buffindex_p         Index of the last buffer marked as sent.

*/
//------------------------------------------------------------------------------
static void edrvTxHandler(void* txBuff_p)
{
    tEdrvTxBuffer*   pBuffer = (tEdrvTxBuffer*) txBuff_p;
    //DbgPrint("%s() \n", __func__);
    // Process all the frames till specified index
    if (pBuffer != NULL)
    {
        //DbgPrint("%s() \n", __func__);
        if (pBuffer->pfnTxHandler != NULL)
        {
            //DbgPrint("1 \n");
            pBuffer->pfnTxHandler(pBuffer);
        }
    }
    edrvInstance_l.headTxIndex = (edrvInstance_l.headTxIndex + 1) & EDRV_TX_QUEUE_MASK;
}

///\}

