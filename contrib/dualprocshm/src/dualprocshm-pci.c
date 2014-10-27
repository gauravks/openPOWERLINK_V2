/**
********************************************************************************
\file   dualprocshm-pci.c

\brief  Dual Processor Library Support File - sp605eb

This file provides specific function definition for Zynq SoC to support shared
memory interface using dual processor library.

\ingroup module_dualprocshm
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <dualprocshm-target.h>
#include <dualprocshm-pcp.h>
#include <ndis-intf.h>
#include <common/target.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DEFAULT_LOCK_ID             0x00    ///< Default lock Id
#define DYN_MEM_TABLE_ENTRY_SIZE    4       ///< Size of Dynamic table entry
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
// get the address to store address mapping table

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------
void target_regSyncIrqHdl(void* callback_p,void* pArg_p);
void target_enableSyncIrq(BOOL fEnable_p);
//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Get common memory address for platform

Target specific routine to retrieve the base address of common memory between
two processors.

\param  pSize_p      Minimum size of the common memory, returns the
                     actual size of common memory.

\return Pointer to base address of common memory.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
UINT8* dualprocshm_getCommonMemAddr(UINT16* pSize_p)
{
    UINT8* pAddr;

    if (*pSize_p > MAX_COMMON_MEM_SIZE )
    {
//        printk("%s Common memory not available\n",__FUNCTION__);
        return NULL;
    }

    pAddr = (UINT8*)ndis_getBar1Addr();

    if(pAddr == NULL)
    {
    //	printk("%s Memory not found\n",__FUNCTION__); 	
        return NULL;
    }

    *pSize_p = MAX_COMMON_MEM_SIZE - 1;
    DbgPrint("%s() Addr:%p\n", __FUNCTION__,pAddr);
    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free common memory address

Target specific to routine to release the base address of
common memory.

\param  pSize_p      Size of the common memory

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseCommonMemAddr(UINT16 pSize_p)
{
    UNUSED_PARAMETER(pSize_p);
    // nothing to do be done on zynq
}

//------------------------------------------------------------------------------
/**
\brief  Get dynamic mapping table base address

Target specific routine to retrieve the base address for storing
dynamic mapping table

\return Pointer to base address of dynamic mapping table

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT8* dualprocshm_getDynMapTableAddr(void)
{
    UINT8* pAddr = (UINT8*)ndis_getBar1Addr();
//printk("Dynamic map tab:%x\n",pAddr);
    pAddr = (UINT8*) (pAddr + MEM_ADDR_TABLE_OFFSET);
    DbgPrint("%s() Addr:%p\n", __FUNCTION__, pAddr);
    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free dynamic mapping table base address

Target specific routine to free the base address used for storing
dynamic mapping table

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseDynMapTableAddr(void)
{
    // nothing to be done on zynq
}

//------------------------------------------------------------------------------
/**
\brief  Get interrupt synchronization base address

Target specific routine to retrieve the base address for storing
interrupt synchronization registers.

\return Pointer to base address of interrupt synchronization registers

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT8* dualprocshm_getIntrMemAddr(void)
{
    UINT8* pAddr = (UINT8*)ndis_getBar1Addr();

    if (pAddr == NULL)
        return NULL;
    pAddr = (UINT8*) (pAddr + MEM_INTR_OFFSET);
    DbgPrint("%s() Addr:%p\n", __FUNCTION__,pAddr);
    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free interrupt synchronization base address

Target specific routine to free the base address used for storing
interrupt synchronization registers.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseIntrMemAddr()
{
    // nothing to be done on zynq
}

//------------------------------------------------------------------------------
/**
\brief  Read data from memory

Target specific memory read routine

\param  pBase_p    Base address to be read
\param  size_p     No of bytes to be read
\param  pData_p    Pointer to receive the read data

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetReadData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p)
{

    if (pBase_p == NULL || pData_p == NULL)
    {
//        printk("%s Invalid parameters\n",__FUNCTION__);
        return;
    }

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(pBase_p,size_p);

    DUALPROCSHM_MEMCPY(pData_p, pBase_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write data to memory

Target specific routine used to write data to the specified memory address.

\param  pBase_p      Base address to be written
\param  size_p       No of bytes to write
\param  pData_p      Pointer to memory containing data to written

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetWriteData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p)
{
    if (pBase_p == NULL || pData_p == NULL)
    {
//        printk("%s Invalid parameters\n",__FUNCTION__);
        return;
    }

    DUALPROCSHM_MEMCPY(pBase_p, pData_p, size_p);

    DUALPROCSHM_FLUSH_DCACHE_RANGE(pBase_p,size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory lock routine(acquire)

This routine provides support for a token based lock using the common memory.
The caller needs to pass the base address and the token for locking a resource
such as memory buffers

\param  pBase_p         Base address of the lock memory
\param  lockToken_p     Token to be used for locking

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetAcquireLock(UINT8* pBase_p, UINT8 lockToken_p)
{
    UINT8 lock = 0;

    if (pBase_p == NULL)
    {
        return;
    }
//printk("Lock Address %x\n",pBase_p);
    // spin till the passed token is written into memory
    do{
        //DUALPROCSHM_INVALIDATE_DCACHE_RANGE((UINT32)pBase_p,1);
        lock = DPSHM_READ8(pBase_p);

        if (lock == DEFAULT_LOCK_ID)
        {
            DPSHM_WRITE8(pBase_p, lockToken_p);
            //DUALPROCSHM_FLUSH_DCACHE_RANGE((UINT32)pBase_p,1);
            continue;
        }
      }while (lock != lockToken_p);

}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory lock routine(release)

This routine is used to release a lock acquired before at a address specified

\param  pBase_p         Base address of the lock memory

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetReleaseLock(UINT8* pBase_p)
{
    UINT8   defaultlock = DEFAULT_LOCK_ID;

    if (pBase_p == NULL)
    {
        return;
    }

    DPSHM_WRITE8(pBase_p,defaultlock);

    DUALPROCSHM_FLUSH_DCACHE_RANGE(pBase_p,sizeof(UINT8));
}

//------------------------------------------------------------------------------
/**
\brief Register synchronization interrupt handler

The function registers the ISR for target specific synchronization interrupt
used by the application for PDO and event synchronization.

\param  callback_p              Interrupt handler
\param  pArg_p                  Argument to be passed while calling the handler

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_regSyncIrqHdl(targetSyncHdl callback_p,void* pArg_p)
{
    //target_regSyncIrqHdl(callback_p,pArg_p);
}

//------------------------------------------------------------------------------
/**
\brief Sync interrupt control routine

The function is used to enable or disable the sync interrupt

\param  fEnable_p              enable if TRUE, disable if FALSE

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_enableSyncIrq(BOOL fEnable_p)
{
    //target_enableSyncIrq(fEnable_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write the buffer address in dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.
\param  addr_p       Address of the buffer.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_targetSetDynBuffAddr(UINT8* pMemTableBase , UINT16 index_p, UINT32 addr_p)
{
    UINT32          tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32          offset;

    if (addr_p != 0)
    {
        offset = (UINT32) (addr_p - (UINT32)ndis_getBar0Addr());
    }
    DPSHM_WRITE32(pMemTableBase + tableEntryOffs, offset);
    DUALPROCSHM_FLUSH_DCACHE_RANGE((UINT32) (pMemTableBase + tableEntryOffs), sizeof(UINT32));
    DbgPrint("%s() I %d Addr:%x\n", __FUNCTION__, index_p, offset);
}

//------------------------------------------------------------------------------
/**
\brief  Read the buffer address from dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.

\return Address of the buffer requested.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
// TODO test if UINT32 is correct data specifier to hold address in 64 bit systems
UINT8* dualprocshm_targetGetDynBuffAddr(UINT8* pMemTableBase, UINT16 index_p)
{
    UINT32          tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32          buffoffset = 0x00;
    UINT8*          Bar0Addr = (UINT8*) ndis_getBar0Addr();
    UINT8*          bufAddr;
    UINT8*          memAddr = (pMemTableBase + tableEntryOffs);
    UINT            count = 0;
    DUALPROCSHM_INVALIDATE_DCACHE_RANGE((pMemTableBase + tableEntryOffs), sizeof(UINT32));
    buffoffset = DPSHM_READ32(memAddr);

    while (buffoffset == 0)
    {
        if (count == 20)
        {
            DbgPrint("Unable to Get Address\n");
            return NULL;
        }
        target_msleep(1000);
        buffoffset = DPSHM_READ32(memAddr);
        count++;
    }
    bufAddr = (Bar0Addr + buffoffset);
    DbgPrint("%s() I %d Addr:%p-%p-%x-%p\n", __FUNCTION__, index_p, memAddr, pMemTableBase, buffoffset, bufAddr);
    return bufAddr;
}
