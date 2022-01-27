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
#include "halAe.h"
#include "icp_firml_interface.h"
#include "halMemScrub.h"

int halAe_GetProdSetting(icp_firml_handle_t *handle);
int halAe_GetChipSetting(icp_firml_handle_t *handle);
int halAe_checkSINT(icp_firml_handle_t *handle);
void initVirAddr(icp_firml_handle_t * handle);
void disableQatIntr(icp_firml_handle_t *handle);
int getAeCsr(icp_firml_handle_t *handle, 
         unsigned char ae, 
         unsigned int csr, 
         unsigned int *value);
extern int waitNumCycles(icp_firml_handle_t *handle,
         unsigned char ae,
         unsigned int cycles,
         int chkInactive);

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
        ERRINFO(("handle=0x%p, prodType=0x%p, prodRev=0x%p\n",
                  handle, prodType, prodRev)); 
        return (HALAE_BADARG);
    }    
    HALAE_VERIFY_LIB(handle);

    switch(handle->halHandle->PrdMinType)
    {
    case HWID_ACCEL_COMP:
        *prodType = ACCEL_COMP_TYPE; 
        *prodRev = 
                (handle->halHandle->PrdMajRev << PID_MAJOR_REV_BITPOS) 
                | handle->halHandle->PrdMinRev;
        break;

    case HWID_ACCEL_COMP_C:
        *prodType = ACCELCOMP_C_CPU_TYPE; 
        *prodRev =
                (handle->halHandle->PrdMajRev << PID_MAJOR_REV_BITPOS)
                | handle->halHandle->PrdMinRev;
        break;

    case HWID_ACCEL_COMP_B:
        *prodType = ACCELCOMP_B_CPU_TYPE; 
        *prodRev =
                (handle->halHandle->PrdMajRev << PID_MAJOR_REV_BITPOS)
                | handle->halHandle->PrdMinRev;
        break;

    case HWID_ACCEL_COMP_RS:
        *prodType = ACCELCOMP_RS_CPU_TYPE; 
        *prodRev =
                (handle->halHandle->PrdMajRev << PID_MAJOR_REV_BITPOS)
                | handle->halHandle->PrdMinRev;
        break;

    case HWID_ACCEL_COMP_R:
        *prodType = ACCELCOMP_R_CPU_TYPE; 
        *prodRev = 
                (handle->halHandle->PrdMajRev << PID_MAJOR_REV_BITPOS) 
                | handle->halHandle->PrdMinRev;
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
    const unsigned int ustore16k=0x4000;
    unsigned int maxEnabledAeNum = 0, ii;
    switch(handle->halHandle->PrdMinType)
    {
    case HWID_ACCEL_COMP:
        for(ii = 0; ii < MAX_AE; ii++)
        {
            if((1<<ii) & handle->sysMemInfo.aeMask) 
            {
                maxEnabledAeNum = ii;
            }
        }
        /* helps map AE number to number within the cluster */
        handle->halHandle->AeMask = 0x7;
        handle->halHandle->AePerCluster = maxEnabledAeNum + 1;
        handle->halHandle->AeBadMask = ~(handle->sysMemInfo.aeMask);
        handle->halHandle->AeMaxNum = maxEnabledAeNum;
        handle->halHandle->UpcMask = 0x1ffff;
        handle->halHandle->MaxUstore = ustore16k;
        handle->halHandle->MaxLmemReg = MAX_ICP_LMEM_REG;
        break;

    case HWID_ACCEL_COMP_C:
    case HWID_ACCEL_COMP_B:
    case HWID_ACCEL_COMP_RS:
        for(ii = 0; ii < MAX_AE; ii++)
        {
            if((1<<ii) & handle->sysMemInfo.aeMask) 
            {
                maxEnabledAeNum = ii;
            }
        }
        
        /* helps map AE number to number within the cluster */
        handle->halHandle->AeMask = 0xf;
        handle->halHandle->AePerCluster = maxEnabledAeNum + 1;
        handle->halHandle->AeBadMask = ~(handle->sysMemInfo.aeMask);
        handle->halHandle->AeMaxNum = maxEnabledAeNum;
        handle->halHandle->UpcMask = 0x1ffff;
        handle->halHandle->MaxUstore = ustore16k;
        handle->halHandle->MaxLmemReg = MAX_ICP_LMEM_REG;
        break;

    case HWID_ACCEL_COMP_R:
        for(ii = 0; ii < MAX_AE; ii++)
        {
            if((1<<ii) & handle->sysMemInfo.aeMask) 
            {
                maxEnabledAeNum = ii;
            }
        }
        
        /* helps map AE number to number within the cluster */
        handle->halHandle->AeMask = 0x1;
        handle->halHandle->AePerCluster = maxEnabledAeNum + 1;
        handle->halHandle->AeBadMask = ~(handle->sysMemInfo.aeMask);
        handle->halHandle->AeMaxNum = maxEnabledAeNum;
        handle->halHandle->UpcMask = 0x1ffff;
        handle->halHandle->MaxUstore = ustore16k;
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
    switch(handle->sysMemInfo.deviceId)
    {
    case ICP_ACCELCOMP_PCIE_DEVICE_ID_AC:
        handle->halHandle->prodId = 
                HWID_ACCEL_COMP << PID_MINOR_PROD_TYPE_BITPOS;
        break;

    case ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC:
        handle->halHandle->prodId = 
                HWID_ACCEL_COMP_C << PID_MINOR_PROD_TYPE_BITPOS;
        break;

    case ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC:
        handle->halHandle->prodId = 
                HWID_ACCEL_COMP_R << PID_MINOR_PROD_TYPE_BITPOS;
        break;

    case ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC:
        handle->halHandle->prodId = 
                HWID_ACCEL_COMP_B << PID_MINOR_PROD_TYPE_BITPOS;
        break;

    case ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC:
        handle->halHandle->prodId = 
                HWID_ACCEL_COMP_RS << PID_MINOR_PROD_TYPE_BITPOS;
        break;

    default:
        handle->halHandle->prodId = 0;
        return (-1);
    }
    if(handle->sysMemInfo.revisionId > ACCELCOMP_D3_RID)
    {
        handle->halHandle->prodId = 0;
        return (-1);
    }
    handle->halHandle->prodId |= 
            (PID_MAJOR_REV | PID_MINOR_REV) 
            & handle->sysMemInfo.revisionId;
    
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
    unsigned int sMIA = 0, sINT = 0, sERRSOU3 = 0, sERRMSK3 = 0;
    
    if (handle == NULL)
    {
        return (-1);
    }
    
    if ((handle->sysMemInfo.deviceId == ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
    {
        /* GET SMIAPF1, SINTPF1 */
        sMIA = READ_LWORD(handle->Hal_ep_csr_virtAddr + EP_SMIAPF1);
        sINT = READ_LWORD(handle->Hal_ep_csr_virtAddr + EP_SINTPF1);
        
        if(((sMIA & sINT) & SINTPF1_MISC_BITPOS) == 0)
        {
            return (-1);
        }
    }
    else
    {
        /* GET SMIA, SINT*/
        sMIA = READ_LWORD(handle->Hal_ep_csr_virtAddr + EP_SMIAPF);
        sINT = READ_LWORD(handle->Hal_ep_csr_virtAddr + EP_SINTPF);
        
        if(((sMIA & sINT) & SINTPF_MISC_BITPOS) == 0)
        {
            return (-1);
        }    
    }

    /*ERRSOU3 bit 1 SHaC1 */
    sERRSOU3 = READ_LWORD(handle->Hal_ep_csr_virtAddr + EP_ERRSOU3);
    if ((sERRSOU3 & ERRSOU3_SHAC1_BITPOS) == 0)
    {
        return (-1);
    }

    /*ERRMSK3 bit 1 SHaC1 */
    sERRMSK3 = READ_LWORD(handle->Hal_ep_csr_virtAddr + EP_ERRMSK3);
    if ((sERRMSK3 & ERRMSK3_SHAC1_BITPOS) == ERRMSK3_SHAC1_BITPOS)
    {
        return (-1);
    }    

    return (0);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Enable eSram auto initialization
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
 * @param handle - IN
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/
int
halAe_enableAutoInitSram(icp_firml_handle_t *handle, unsigned int aeMask)
{
    int status = HALAE_SUCCESS;
    unsigned int debugB = 0, csr = ESRAM_DEBUGB;
    int times = SRAMAUTO_INIT_TIMES;
    int ae;

    /* if on BEK, just return, no secure RAM in Roslin */
    if ((handle->sysMemInfo.deviceId == ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
    {
        return HALAE_SUCCESS;
    }

    if (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC)
    {
        csr = ESRAM_DEBUGB_ACCELCOMP_C;
    }
    /* return if init is done */ 
    debugB = READ_LWORD(handle->Hal_ep_csr_virtAddr + csr);
    if ((debugB & DEBUGB_SRAMAUTO_TINIT) &&
        (debugB & DEBUGB_SRAMAUTO_TINITDONE))
    {
        return HALAE_SUCCESS;
    }

    /* get an available AE */
    for (ae=0; ae<MAX_AE; ae++) {
        if(((1<<ae)& aeMask) && 
           (halAe_GetAeState(handle, (unsigned char)ae) == HALAE_CLR_RST)) {
            break;
        }
    }  
    if (ae>=MAX_AE) {
         PRINTF("could not find right ae\n");
         return (HALAE_FAIL);
    }

    /* enable eSram Auto Initialization */
    debugB = READ_LWORD(handle->Hal_ep_csr_virtAddr + csr);
    debugB |= DEBUGB_SRAMAUTO_TINIT;
    WRITE_LWORD(handle->Hal_ep_csr_virtAddr + csr, debugB);

    /* check if eSram auto initialization is done */
    do
    {
        /* All 4 SRAM are initialized in parallel 
         * and take 8200 p_clk (~10 usecs) */
        waitNumCycles(handle, (unsigned char)ae, SRAMAUTO_INIT_USECS, 0);
        debugB = READ_LWORD(handle->Hal_ep_csr_virtAddr + csr);
        times --;
        DBGINFO(("times=%d, debugB=0x%x\n", times, debugB));
    } while(!(debugB & DEBUGB_SRAMAUTO_TINITDONE) && (times>0));
    if((times == 0) && (!(debugB & DEBUGB_SRAMAUTO_TINITDONE)))
    {
        PRINTF("Fail to enable eSram auto initialization!!\n");
        status = HALAE_FAIL;
    }
    return status;
}

void 
initVirAddr(icp_firml_handle_t * handle)
{
    unsigned int cap_offset, ae_offset;
    unsigned int pmu_offset, scratch_offset, ssu_offset, ep_offset;
    GET_OFFSET(handle->sysMemInfo.deviceId, cap_offset, ae_offset, 
               pmu_offset, scratch_offset, ssu_offset, ep_offset);
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
                SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, 
                     cap_offset);
    }
    if(handle->Hal_cap_ae_xfer_csr_virtAddr == 0 ) 
    {
        handle->Hal_cap_ae_xfer_csr_virtAddr = 
                SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, 
                     ae_offset);
    }
    if(handle->Hal_cap_ae_local_csr_virtAddr == 0 ) 
    {
        handle->Hal_cap_ae_local_csr_virtAddr = 
                SADD(handle->Hal_cap_ae_xfer_csr_virtAddr, 
                     LOCAL_TO_XFER_REG_OFFSET);
    }
    /* to be initialized later while icp_hal_debug.ko is loaded */
    if(handle->Hal_cap_pmu_csr_virtAddr == 0 ) 
    {
        if(handle->sysMemInfo.deviceId == ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC)
        {
            handle->Hal_cap_pmu_csr_virtAddr = 0;
        }
        else
        {
            handle->Hal_cap_pmu_csr_virtAddr = 
                    SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, 
                         pmu_offset);
        }
    }
    if(handle->Hal_cap_hash_csr_virtAddr == 0 ) 
    {
        handle->Hal_cap_hash_csr_virtAddr = 
                SADD(handle->Hal_cap_global_ctl_csr_virtAddr, 
                     HASH_TO_GLOBAL_REG_OFFSET);
    }
    if(handle->Hal_scratch_rd_wr_swap_virtAddr == 0 ) 
    {
        if((handle->sysMemInfo.deviceId == 
           ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC) ||
           (handle->sysMemInfo.deviceId == 
           ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) ||
           (handle->sysMemInfo.deviceId == 
           ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
        {
            handle->Hal_scratch_rd_wr_swap_virtAddr = 0;
        }
        else
        {
            handle->Hal_scratch_rd_wr_swap_virtAddr = 
                    SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, 
                         scratch_offset);
        }
    }
    handle->Hal_ae_fastaccess_csr_virtAddr = 0;
    
    if(handle->Hal_ssu_csr_virtAddr == 0 ) 
    {
        handle->Hal_ssu_csr_virtAddr = 
                SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, 
                     ssu_offset);
    } 
    if(handle->Hal_ep_csr_virtAddr == 0 ) 
    {
        handle->Hal_ep_csr_virtAddr = 
                SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, 
                     ep_offset);
    }
    if(handle->Hal_eagletail_ring_csr_virtAddr == 0 ) 
    {
        handle->Hal_eagletail_ring_csr_virtAddr = 
                handle->sysMemInfo.pciBars[ICP_AC_PETRINGCSR_BAR].virtAddr;
    } 
}

void disableQatIntr(icp_firml_handle_t *handle)
{
    unsigned int qat;
    unsigned int csr = 0;
    /* disable Notify and Job Done interrupt
     * from CPR/XLT/RegEx for Accelcomp Stepping A */
    if((handle->sysMemInfo.deviceId == ICP_ACCELCOMP_PCIE_DEVICE_ID_AC)
      && (handle->sysMemInfo.revisionId == ACCELCOMP_A0_RID))
    {
        for (qat=0; qat < MAX_QAT; qat++) 
        {
            if ((1 << qat) & handle->sysMemInfo.qatMask ) 
            {
                csr = READ_LWORD(((handle->Hal_ssu_csr_virtAddr
                      + 0x4000*(qat & 0x1))
                      +((QAT_NB_IA_EVENTMSK_CMP) & 0x3FFF)));
                csr = csr | CMP_MASK_NOTIFY_JOBDONE;
                WRITE_LWORD(((handle->Hal_ssu_csr_virtAddr
                      + 0x4000*(qat & 0x1))
                      +((QAT_NB_IA_EVENTMSK_CMP) & 0x3FFF)), (csr));

                csr = READ_LWORD(((handle->Hal_ssu_csr_virtAddr
                      + 0x4000*(qat & 0x1))
                      +((QAT_NB_IA_EVENTMSK_RE) & 0x3FFF)));
                csr = csr | REGEX_MASK_NOTIFY_JOBDONE;
                WRITE_LWORD(((handle->Hal_ssu_csr_virtAddr 
                      + 0x4000*(qat & 0x1))
                      +((QAT_NB_IA_EVENTMSK_RE) & 0x3FFF)), (csr));

                csr = READ_LWORD(((handle->Hal_ssu_csr_virtAddr 
                      + 0x4000*(qat & 0x1))
                      +((QAT_NB_IA_EVENTMSK_XLT) & 0x3FFF)));
                csr = csr | XLT_MASK_NOTIFY_JOBDONE;
                WRITE_LWORD(((handle->Hal_ssu_csr_virtAddr
                      + 0x4000*(qat & 0x1))
                      +((QAT_NB_IA_EVENTMSK_XLT) & 0x3FFF)), (csr));
            }
        }
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      
 * @param
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
int
halAe_checkAes(icp_firml_handle_t *handle, 
               unsigned int aeMask)
{
    unsigned int cnt, pre_cnt;
    unsigned char ae;
    int times = MAX_RETRY_TIMES;
    unsigned int stat = HALAE_SUCCESS;
    for(ae = 0; ae < MAX_AE; ae++)
    {
        if(!(aeMask & (1<<ae)))
        {
            continue;
        }
        times = MAX_RETRY_TIMES;
        stat = getAeCsr(handle, ae, PROFILE_COUNT, (unsigned int *)&cnt);
        if(stat != HALAE_SUCCESS)
        {
            PRINTF("AE%d CSR is unaccessible!!\n", ae);
            return HALAE_FAIL;
        }
        pre_cnt = cnt & 0xffff;

        do {
            stat = getAeCsr(handle, ae, PROFILE_COUNT, (unsigned int *)&cnt); 
            if(stat != HALAE_SUCCESS)
            {
                PRINTF("AE%d CSR is unaccessible!!\n", ae);
                return HALAE_FAIL;
            }
            cnt &= 0xffff;
    
            if(cnt == pre_cnt)
            {
                times --;
                DBGINFO(("times=%d\n", times));
            }
            else
            {
                break;
            }
            if(times <= 0)
            {
                PRINTF("AE%d is useless!!\n", ae);
                return HALAE_FAIL;
            }
        } while(1);
    }
    return HALAE_SUCCESS;
}

