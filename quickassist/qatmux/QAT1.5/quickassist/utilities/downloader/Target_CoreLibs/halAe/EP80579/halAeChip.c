/**
 **************************************************************************
 * @file halAeChip.c
 *
 * @description
 *      This file provides Implementation of Ucode AE Library
 *
 * @par 
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify 
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but 
 *   WITHOUT ANY WARRANTY; without even the implied warranty of 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License 
 *   along with this program; if not, write to the Free Software 
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution 
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE 
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without 
 *   modification, are permitted provided that the following conditions 
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided with the 
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *  version: QAT1.5.L.1.11.0-36
 *
 **************************************************************************/ 

#include "core_io.h"
#include "halAeApi.h"
#include "halAeDrv.h"
#include "icp_firml_interface.h"

#define HWID_ICP_AEPERCLUSTER  4
#define HWID_ICP_AE_MAX_NUM    3

int halAe_GetProdSetting(icp_firml_handle_t *handle);
int halAe_GetChipSetting(icp_firml_handle_t *handle);
int halAe_checkSINT(icp_firml_handle_t *handle);
void initVirAddr(icp_firml_handle_t * handle);
void disableQatIntr(icp_firml_handle_t *handle);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the CPU Type and product maj/min revisions, and return
 *      the CPU type, and the combined maj and min revision.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param prodType - IN A pointer to the location of product type
 * @param prodRev - IN A pointer to the location of product revision
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/
int 
halAe_GetProdInfo(icp_firml_handle_t *handle, 
                  unsigned int *prodType, 
                  unsigned int *prodRev)
{
    if(!handle || !prodType || !prodRev) 
    {
        ERRINFO(("handle=0x%x, prodType=0x%x, prodRev=0x%x\n", 
                  (unsigned int)handle, (unsigned int)prodType, 
                  (unsigned int)prodRev)); 
        return (HALAE_BADARG);
    }    
    HALAE_VERIFY_LIB(handle);

    switch(handle->halHandle->PrdMinType)
    {
    case HWID_ICP:
        if (handle->halHandle->PrdMajRev < 0x2)
        {
            /* EP80579 */
            *prodType = EP80579_CPU_TYPE; 
            *prodRev = 
                    (handle->halHandle->PrdMajRev << PID_MAJOR_REV_BITPOS) 
                    | handle->halHandle->PrdMinRev;
        }
        else
        {
            ERRINFO((" PrdMajRev =0x%x\n", handle->halHandle->PrdMajRev)); 
            return (HALAE_FAIL);
        }
        break;

    default: 
        ERRINFO(("PrdMinType=0x%x\n", handle->halHandle->PrdMinType)); 
        return (HALAE_FAIL);
    }
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Get chip specific settings
 *
 * @param 
 * @param 
 *
 * @retval  HALAE_SUCCESS, HALAE_FAIL
 * 
 * 
 *****************************************************************************/
int halAe_GetProdSetting(icp_firml_handle_t *handle)
{
    const unsigned int ustore8k=0x2000;
    switch(handle->halHandle->PrdMinType)
    {
    case HWID_ICP:
        if (handle->halHandle->PrdMajRev < 0x2)
        {   /* EP80579 */
            /* helps map AE number to number within the cluster */
            handle->halHandle->AeMask = 0x3;
            handle->halHandle->AePerCluster = HWID_ICP_AEPERCLUSTER;
            handle->halHandle->AeBadMask = 0xfffffff0;
            handle->halHandle->AeMaxNum = HWID_ICP_AE_MAX_NUM;
        }
        else
        {
            ERRINFO((" PrdMajRev =0x%x\n", handle->halHandle->PrdMajRev)); 
            return (HALAE_FAIL);
        }
        handle->halHandle->UpcMask = 0x1ffff;
        handle->halHandle->MaxUstore = ustore8k;
        handle->halHandle->MaxLmemReg = MAX_ICP_LMEM_REG;
        break;

    default:
        ERRINFO(("PrdMinType=0x%x\n", handle->halHandle->PrdMinType)); 
        return (HALAE_FAIL);
    }

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Get chip-specific settings.
 *
 * @param 
 * @param 
 *
 * @retval  0 if successful or -1 for failure
 * 
 * 
 *****************************************************************************/
int 
halAe_GetChipSetting(icp_firml_handle_t *handle)
{
    
    handle->halHandle->prodId = 
            HWID_ICP << PID_MINOR_PROD_TYPE_BITPOS; /* EP80579 */
    if(handle->sysMemInfo.revisionId == EP80579_A0_RID) /* EP80579 A0 */
    {
        handle->halHandle->prodId |= 
                (PID_MAJOR_REV | PID_MINOR_REV) 
                & ((EP80579_A_MAJOR_REV << PID_MAJOR_REV_BITPOS));
    }
    else if((handle->sysMemInfo.revisionId >= EP80579_B0_RID)
            && (handle->sysMemInfo.revisionId <= EP80579_B2_RID)) 
    { /* EP80579 B Stepping */
        handle->halHandle->prodId |= 
                ((EP80579_B_MAJOR_REV << PID_MAJOR_REV_BITPOS) & PID_MAJOR_REV)
                | ((handle->sysMemInfo.revisionId - EP80579_B0_RID)
                    & PID_MINOR_REV);
    }
    else
    {
        handle->halHandle->prodId = 0;
        return (-1);
    }
    
    return (0);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read sINT on EP to check interrupt source
 *
 * @param 
 * @param 
 *
 * @retval  0 if successful or -1 for failure
 * 
 * 
 *****************************************************************************/
int
halAe_checkSINT(icp_firml_handle_t *handle)
{
    return (0);
}

void 
initVirAddr(icp_firml_handle_t * handle)
{
    if(handle->Hal_dram_ch0_virtAddr == 0 ) 
    {
        handle->Hal_dram_ch0_virtAddr = 
                handle->sysMemInfo.dramDesc[0].dramBaseAddr_v;
    }
    if(handle->Hal_dram_ch1_virtAddr == 0 ) 
    {
        handle->Hal_dram_ch1_virtAddr = 
                handle->sysMemInfo.dramDesc[1].dramBaseAddr_v;
    }    
    if(handle->Hal_sram_virtAddr == 0 ) 
    {
        handle->Hal_sram_virtAddr = 
                handle->sysMemInfo.sramDesc.sramBaseAddr_v;           
    }
    if(handle->Hal_cap_global_ctl_csr_virtAddr == 0 ) 
    {
        handle->Hal_cap_global_ctl_csr_virtAddr = 
                handle->sysMemInfo.pciBars[ICP_AE_CLUSTER_CAP_BAR].virtAddr;
    }
    if(handle->Hal_cap_ae_xfer_csr_virtAddr == 0 ) 
    {
        handle->Hal_cap_ae_xfer_csr_virtAddr = 
                handle->sysMemInfo.pciBars[ICP_AE_CLUSTER_AE_BAR].virtAddr;
    }
    if(handle->Hal_cap_ae_local_csr_virtAddr == 0 ) 
    {
        handle->Hal_cap_ae_local_csr_virtAddr = 
                SADD(handle->Hal_cap_ae_xfer_csr_virtAddr, 
                     LOCAL_TO_XFER_REG_OFFSET);
    }
    /* to be initialized later while icp_hal_debug.ko is loaded */
    handle->Hal_cap_pmu_csr_virtAddr = 0;     
    if(handle->Hal_cap_hash_csr_virtAddr == 0 ) 
    {
        handle->Hal_cap_hash_csr_virtAddr = 
                SADD(handle->Hal_cap_global_ctl_csr_virtAddr, 
                     HASH_TO_GLOBAL_REG_OFFSET);
    }
    if(handle->Hal_scratch_rd_wr_swap_virtAddr == 0 ) 
    {
        handle->Hal_scratch_rd_wr_swap_virtAddr = 
                handle->sysMemInfo.pciBars[ICP_AE_CLUSTER_SP_BAR].virtAddr;
    }
    /* to be initialized for whole chip */
    handle->Hal_ae_fastaccess_csr_virtAddr = 0;
    
    if(handle->Hal_ssu_csr_virtAddr == 0 ) 
    {
        handle->Hal_ssu_csr_virtAddr = 
                handle->sysMemInfo.pciBars[ICP_AE_CLUSTER_SSU_BAR].virtAddr;
    } 
    if(handle->Hal_ep_csr_virtAddr == 0 ) 
    {
        handle->Hal_ep_csr_virtAddr = 0;
    }
    if(handle->Hal_eagletail_ring_csr_virtAddr == 0 ) 
    {
        handle->Hal_eagletail_ring_csr_virtAddr = 
                handle->sysMemInfo.pciBars[ICP_AE_CLUSTER_SSU_BAR+1].virtAddr 
                + 0x3000;        
    } 
}

void disableQatIntr(icp_firml_handle_t *handle)
{
    return;
}
