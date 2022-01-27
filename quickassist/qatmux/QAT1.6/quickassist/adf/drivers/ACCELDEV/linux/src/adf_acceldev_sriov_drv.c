/*****************************************************************************
 *
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
 *  version: QAT1.6.L.2.6.0-65
 *
 *****************************************************************************/

/*****************************************************************************
 * @file adf_acceldev_sriov_drv.c
 *
 * @description
 *
 *      This file contains sriov related functions
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <asm/io.h>

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_platform.h"
#include "adf_platform.h"
#include "adf_ETring_ap.h"
#include "adf_drv_sriov.h"
#include "adf_init.h"
#include "adf_pfvfcomms.h"
#include "icp_sal_versions.h"

#if LINUX_VERSION_CODE <  KERNEL_VERSION(2,6,26)
#include <asm/semaphore.h>
#else
#include <linux/semaphore.h>
#endif

#define RESTARTING_MAX_LOOPS  1000
#define RESTARTING_SLEEP_TIME 2

static struct workqueue_struct *putMsg_wq = NULL;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
static DEFINE_SEMAPHORE(ring_info_sema);
#else
static DECLARE_MUTEX(ring_info_sema);
#endif

/* Offset within the BAR for Table A */
#define REG_OFFSET_TABLE_A_START    (0x3A400 + 0x190)
/* Offset within the BAR for Table B */
#define REG_OFFSET_TABLE_B_START    (0x3A400 + 0x310)
/* Register size of Ae2Function */
#define REG_SIZE            0x4
/* Number of Ae2Function registers (for table A) */
#define AE2FUNCTION_NUM_TABLE_A     96
/* Number of Ae2Function registers (for table B) */
#define AE2FUNCTION_NUM_TABLE_B     12
/* Valid bit within these registers */
#define VALID_BIT           7
/* Mask to unset AE function valid bit */
#define AE_INVALID_BIT_MSK 0x7F
/* Mask for enabling/disabling ints for all VFs */
#define ICP_VFTOPF_MSK_32VFS      0xFFFFFFFF

#define READ_CSR_AE2FUNCTION_TABLE_A(pmisc_bar_addr, index)                  \
        ICP_ADF_CSR_RD(pmisc_bar_addr, REG_OFFSET_TABLE_A_START +            \
                REG_SIZE * index)

#define WRITE_CSR_AE2FUNCTION_TABLE_A(pmisc_bar_addr, index, value)          \
        ICP_ADF_CSR_WR(pmisc_bar_addr, REG_OFFSET_TABLE_A_START +            \
                REG_SIZE * index, value)

#define READ_CSR_AE2FUNCTION_TABLE_B(pmisc_bar_addr, index)                  \
        ICP_ADF_CSR_RD(pmisc_bar_addr, REG_OFFSET_TABLE_B_START +            \
                REG_SIZE * index)

#define WRITE_CSR_AE2FUNCTION_TABLE_B(pmisc_bar_addr, index, value)          \
        ICP_ADF_CSR_WR(pmisc_bar_addr, REG_OFFSET_TABLE_B_START +            \
                REG_SIZE * index, value)

/*
 * Lock to synchronize access to autopush registers
 */
static spinlock_t lock;

typedef struct adf_vf_ring_update_info_s {
        icp_adf_ringInfoService_t inf;
        Cpa32U ring;
        Cpa32U flags;
} adf_ring_update_info_t;

static adf_ring_update_info_t *info_tbl[ADF_MAX_DEVICES];

typedef struct adf_putMsg_param_s {
        icp_accel_pci_vf_info_t *vf ;
        Cpa32U msg;
        struct work_struct putMsg_work;

}adf_putMsg_param_t;


#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
/*
 * adf_vf_ring_info_update
 * Call ring info callback
 */
static int adf_vf_ring_info_update(icp_accel_dev_t *accel_dev, Cpa32U ring,
                            icp_adf_ringInfoOperation_t op,
                            icp_adf_ringInfoService_t info)
{
        int ret = SUCCESS;
        CpaStatus stat = CPA_STATUS_SUCCESS;
        ringInfoCb cb = (ringInfoCb) accel_dev->ringInfoCallBack;
        if (NULL != cb) {
                /* Serialize calls to the callback */
                down(&ring_info_sema);
                stat = cb(accel_dev, ring, op, info);
                up(&ring_info_sema);
                if (CPA_STATUS_SUCCESS != stat) {
                        ADF_ERROR("Ring Info Callback failed, "
                                  "ring number %d will be disabled\n", ring);
                        ret = FAIL;
                }
        } else {
                ADF_ERROR("Ring Info Callback is NULL, "
                          "ring number %d will be disabled\n", ring);
                ret = FAIL;
        }
        return ret;
}

/*
 * adf_sendResponseToVF
 * sends a message to the VF
 */
static void adf_sendResponseToVF(struct work_struct *work)
{
        adf_hw_device_data_t  *hw_data = NULL;
        adf_putMsg_param_t *putMsg = NULL;
        icp_accel_pci_vf_info_t *vf = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        CpaStatus status = CPA_STATUS_SUCCESS;

        ICP_CHECK_FOR_NULL_PARAM_VOID(work);
        putMsg = container_of(work, adf_putMsg_param_t , putMsg_work);
        ICP_CHECK_FOR_NULL_PARAM_VOID(putMsg);
        vf = putMsg->vf;
        ICP_CHECK_FOR_NULL_PARAM_VOID(vf);
        accel_dev = vf->accel_dev;
        ICP_CHECK_FOR_NULL_PARAM_VOID(accel_dev);
        hw_data = accel_dev->pHwDeviceData;
        ICP_CHECK_FOR_NULL_PARAM_VOID(hw_data);

        if (NOT_NULL_FUNCTION(hw_data->adf_systemMsg_PF2VF_putMsg)) {
                status = hw_data->adf_systemMsg_PF2VF_putMsg(
                        (icp_accel_pci_vf_info_t*)vf, putMsg->msg);
        }else{
                ADF_ERROR(" PF2VFcomms: put message not supported on "
                          "this device");
        }
        
        if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("PF failed to send msg to VF %d", status);
        }

        ICP_FREE(putMsg);
        return;
}

STATIC void
adf_remove_device_from_ring_info(icp_accel_dev_t *accel_dev)
{
        ICP_CHECK_FOR_NULL_PARAM_VOID(accel_dev);
        ICP_FREE_NUMA(info_tbl[accel_dev->accelId]);
        info_tbl[accel_dev->accelId] = NULL;
        return;
}

STATIC CpaStatus
adf_add_device_to_ring_info(icp_accel_dev_t *accel_dev)
{
        adf_ring_update_info_t *info = NULL;
        Cpa32U numRingsInDevice = 0;
        Cpa32U sizeOfRingInfoArray = 0;

        ICP_CHECK_FOR_NULL_PARAM(accel_dev);
        numRingsInDevice = GET_NUM_RINGS_PER_BANK(accel_dev)
                                            * GET_MAX_BANKS(accel_dev);
        sizeOfRingInfoArray = sizeof (adf_ring_update_info_t)*numRingsInDevice;
        info = ICP_MALLOC_GEN_NUMA(sizeOfRingInfoArray, accel_dev->pkg_id);
        if (!info) {
                return CPA_STATUS_FAIL;
        }
        ICP_MEMSET(info, '\0', sizeOfRingInfoArray);
        info_tbl[accel_dev->accelId] = info;
        info = NULL;
        return CPA_STATUS_SUCCESS;
}

/*
 * adf_vf_clean_ring_info
 * Unset autopush and send ring info update to AE
 */
int adf_vf_clean_ring_info(icp_accel_pci_vf_info_t *vf)
{
        int ret = SUCCESS, i = 0;
        adf_ring_update_info_t **tbl = vf->vf_ring_info_tbl;
        adf_ring_update_info_t *ring_info = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        Cpa32U numRingsPerBank = 0;

        accel_dev = vf->accel_dev;
        numRingsPerBank = GET_NUM_RINGS_PER_BANK(accel_dev);
        for (i = 0; i < numRingsPerBank; i++, tbl++) {
                ring_info = *tbl;
                if (ring_info) {
                        if (ring_info->flags) {
                                /* Send ring info msg to AE */
                                ret = adf_vf_ring_info_update(vf->accel_dev,
                                              ring_info->ring,
                                              ICP_ADF_RING_DISABLE,
                                              ring_info->inf);
                                if (SUCCESS != ret)
                                        ADF_ERROR("Failed to send ring info"
                                                   " update to AE\n");
                        }
                        /* Now after AE is updated we can remove the ring from
                         * autopush */
                        spin_lock_bh(&lock);
                        adf_etrAeApClrRingRegister(vf->accel_dev,
                                            ring_info->ring, ring_info->flags);
                        spin_unlock_bh(&lock);
                        *tbl = NULL;
                }
        }
        return SUCCESS;
}

/*
 * adf_vfbh_handler
 * Bottom half handles for VF2PF irq
 */
void adf_vf_bh_handler(void* handle)
{
        Cpa32U *pmisc_bar_addr = NULL, msg = 0;
        icp_accel_pci_vf_info_t *vf = (icp_accel_pci_vf_info_t*) handle;
        icp_accel_dev_t *accel_dev = NULL;
        icp_accel_pci_info_t *pci_info = NULL;
        adf_putMsg_param_t *putMsg_param = NULL;
        adf_hw_device_data_t  *hw_data = NULL;
        Cpa32U pmisc_bar_id = 0;
        Cpa32U compat_response_msg = 0;
        CpaStatus status = CPA_STATUS_SUCCESS;

        ICP_CHECK_FOR_NULL_PARAM_VOID(vf);
        accel_dev = vf->accel_dev;
        ICP_CHECK_FOR_NULL_PARAM_VOID(accel_dev);
        pci_info = &accel_dev->pciAccelDev;
        ICP_CHECK_FOR_NULL_PARAM_VOID(pci_info);
        hw_data = accel_dev->pHwDeviceData;
        ICP_CHECK_FOR_NULL_PARAM_VOID(hw_data);

        pmisc_bar_id = hw_data->getMiscBarId(hw_data);
        pmisc_bar_addr = (void*)
                pci_info->pciBars[pmisc_bar_id].virtAddr;

        /* Read the message from VF */
        msg = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        hw_data->getPf2VfDbIntOffset(vf->deviceId));


        if (ICP_PFVF_MSGORIGIN_SYSTEM ==
                adf_vf2pf_get_msg_origin(msg)) {

                /* Get msg type */
                switch (adf_vf2pf_get_msg_type(msg)) {

                case ICP_VF2PF_MSGTYPE_INIT:  
                {
                    ADF_DEBUG("Init Msg received from Dev%d VF%d 0x%x\n",
                            accel_dev->accelId, vf->deviceId+1, msg );
                    vf->init = 1;
                }
                break;
                case ICP_VF2PF_MSGTYPE_SHUTDOWN:
                {
                    ADF_DEBUG("Shutdown Msg received from Dev%d VF%d 0x%x\n",
                            accel_dev->accelId, vf->deviceId+1,msg);
                    vf->init = 0;
                }
                break;
                case ICP_VF2PF_MSGTYPE_VERSION_REQ:
                {
                        Cpa32U vfMajorVersion = 0;
                        Cpa32U vfMinorVersion = 0;
                        Cpa32U pf_lowestMajorVersionOfVFSupported = 
                         hw_data->icp_version.device_lowestSupportMajorVersion;
                        Cpa32U pf_lowestMinorVersionOfVFSupported = 
                         hw_data->icp_version.device_lowestSupportMinorVersion;

                        ADF_DEBUG("Version Msg received frm Dev%d,VF%d 0x%x\n",
                                accel_dev->accelId, vf->deviceId+1,msg);
                        vfMajorVersion = adf_vf2pf_get_major_version(msg);
                        vfMinorVersion = adf_vf2pf_get_minor_version(msg);
                        
                        /* check if we support the major version */
                        if(vfMajorVersion<pf_lowestMajorVersionOfVFSupported){
                                adf_pf2vf_set_vf_notSupported(
                                                    &compat_response_msg);
                                ADF_DEBUG("VF major number not supported\n");
                                status = CPA_STATUS_FAIL;
                        }
                        /* see if the minor number is supported */
                        if(CPA_STATUS_SUCCESS == status && vfMajorVersion ==
                                pf_lowestMajorVersionOfVFSupported ){
                                /*if we support the major - check the 
                                 * the minor is supported */
                                if(vfMinorVersion
                                        < pf_lowestMinorVersionOfVFSupported){
                                          adf_pf2vf_set_vf_notSupported(
                                                        &compat_response_msg);
                                          ADF_DEBUG("VF minor number "
                                                           "not supported\n");
                                          status = CPA_STATUS_FAIL;
                                  }
                        }
                        if (CPA_STATUS_FAIL == status ) {
                                ADF_ERROR("PF (%d.%d) QA driver received "
                                    "message from incompatible VF (%d.%d)"
                                    "on Device%d, VF%d."
                                    " Please upgrade VF to version supported"
                                    " by PF. See release notes for details\n",
                                    SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
                                    SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER,
                                    vfMajorVersion,vfMinorVersion,
                                    accel_dev->accelId, vf->deviceId+1);
                        }

            
                        if(CPA_STATUS_SUCCESS == status ){
                               /* The VF version is not too old. Now see if
                                * it's newer than PF */
                               if(vfMajorVersion >=
                                        pf_lowestMajorVersionOfVFSupported
                                               &&
                                  vfMajorVersion <
                                     SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER){
                                       adf_pf2vf_set_vf_supported(
                                                         &compat_response_msg);
                               }else if(vfMajorVersion == 
                                       SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER
                                               &&
                                       vfMinorVersion <=
                                       SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER){
                                       adf_pf2vf_set_vf_supported(
                                                         &compat_response_msg);
                                       
                               }else{
                                       /* The VF version is newer than PF.
                                          PF doesn't know if it's compatible,
                                          let VF decide */
                                       adf_pf2vf_set_vf_unknown(
                                                         &compat_response_msg);
                                       ADF_DEBUG("PF(%d.%d) QA driver received"
                                           " message from unknown"
                                           " VF version (%d.%d)."
                                           " VF will decide if compatible\n",
                                        SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
                                        SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER,
                                        vfMajorVersion,vfMinorVersion);
                               }
                        }

                        adf_pf2vf_set_version(&compat_response_msg,
                                SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
                                SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER);

                        adf_pf2vf_set_msg_type(&compat_response_msg,
                                ICP_PF2VF_MSGTYPE_VERSION_RESP);
                        adf_pf2vf_set_msg_origin(&compat_response_msg,
                                ICP_PFVF_MSGORIGIN_SYSTEM);

                        putMsg_param =
                                ICP_MALLOC_ATOMIC(sizeof(adf_putMsg_param_t));
                        if (putMsg_param) {

                                putMsg_param->vf = vf;
                                putMsg_param->msg = compat_response_msg;

                                INIT_WORK(&putMsg_param->putMsg_work,
                                                adf_sendResponseToVF);
                                queue_work(putMsg_wq,
                                                   &putMsg_param->putMsg_work);
                        } else {
                                ADF_ERROR("Failed to alloc mem for putMsg\n");
                        }
                }
                break;
                default: 
                    ADF_ERROR("Unknown Msg received from Dev%d VF%d 0x%x\n",
                                        accel_dev->accelId,vf->deviceId+1,msg);

                        break;
                } 
        } else {
                /*USER SPACE MSG*/
                Cpa16U userSpaceMsg = 0;

                ADF_DEBUG("User Msg received from Dev%d VF%d 0x%08x\n",
                                    accel_dev->accelId,vf->deviceId+1,msg);
                userSpaceMsg  = adf_vf2pf_get_userSpace_msg(msg);
                adf_vfpf_storeMsgForUser(accel_dev, vf->deviceId,userSpaceMsg);
        }

        /* Send ack to VF */
        adf_pf2vf_ack(&msg);
        ADF_DEBUG("writing to doorbell CSR on Dev%d VF%d: 0x%08x\n",
                                accel_dev->accelId,vf->deviceId+1,msg);
        ICP_ADF_CSR_WR(pmisc_bar_addr,
                              hw_data->getPf2VfDbIntOffset(vf->deviceId), msg);
        /* re-enable interrupt on PF from this VF */
        if (hw_data->enableVf2PfInterrupts) {
                hw_data->enableVf2PfInterrupts(accel_dev, (1 << vf->deviceId));
        }
}

/*
 * Notify VF about restarting event
 */
CpaStatus adf_PF2VF_notify_restarting(icp_accel_dev_t *accel_dev)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        Cpa32U val = 0;
        adf_hw_device_data_t *hw_data = NULL;
        Cpa32U pf2vf_bar_id = 0;
        Cpa32U pf2vf_dbint_offset = 0;
        icp_accel_pci_vf_info_t *vf = NULL;
        Cpa32U dev_nr = 0;

        ICP_CHECK_FOR_NULL_PARAM(accel_dev);
        ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);

        hw_data = accel_dev->pHwDeviceData;
        pf2vf_bar_id = hw_data->getMiscBarId(hw_data);

        /* Build a notify message to VF */
        adf_pf2vf_set_msg_type(&val, ICP_PF2VF_MSGTYPE_RESTARTING);
        adf_pf2vf_set_msg_origin(&val,ICP_PFVF_MSGORIGIN_SYSTEM);

        for(dev_nr = 0; dev_nr < hw_data->maxNumVf; dev_nr++) {
                if(NULL == accel_dev->pci_vfs[dev_nr]) continue;
                vf = (icp_accel_pci_vf_info_t *)accel_dev->pci_vfs[dev_nr];
                if(1 == vf->attached) continue;
                if(0 == vf->init) continue;

                pf2vf_dbint_offset = hw_data->getPf2VfDbIntOffset(dev_nr);

                /* Send the message */
                status = hw_data->adf_systemMsg_PF2VF_putMsg(vf, val);
                
                if (CPA_STATUS_FAIL == status)
                {
                        ADF_ERROR(" Failed to send restarting "
                                        "message to Device%d vf:%d\n",
                                        accel_dev->accelId,dev_nr+1);
                }
        }
        
        ADF_DEBUG("%s: status = %d\n", __FUNCTION__, status);

        return status;
}

/*
 * adf_sriov_enable
 * Enable accel dev virtualization
 */
int adf_enable_sriov(icp_accel_dev_t *accel_dev)
{
        int ret = SUCCESS, i = 0;
        uint32_t *pmisc_bar_addr = NULL;
        uint32_t reg = 0, vfs = 0;
        icp_accel_pci_info_t *pci_info = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        Cpa32U miscBarId = 0;
        Cpa32U ae2Function = 0;
        CpaStatus status = CPA_STATUS_FAIL;

        if (!accel_dev) {
                ADF_ERROR("Invalid accel device\n");
                return FAIL;
        }

        if (!accel_dev->pHwDeviceData) {
                ADF_ERROR("Invalid accel device config data\n");
                return FAIL;
        }
        hw_data = accel_dev->pHwDeviceData;

        /*
         * Check number of VF for lower SKUs with one Accelerator.
         * However, if we have an arbiter, we have the same number of VFs on
         * all SKUs.
         */
        if(hw_data->isArbitrationSupported) {
                vfs = hw_data->maxNumVf;
        } else {
                vfs = (hw_data->maxNumAccel ==
                        hw_data->getNumAccelerators(hw_data,
                                                    accel_dev->accelMask))
                        ? hw_data->maxNumVf : hw_data->maxNumVf >> 1;
        }

        if (!BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                  ADF_STATUS_SYSTEM_RESTARTING)) {
                ret = pci_enable_sriov(accel_dev->pciAccelDev.pDev, vfs);
                if (SUCCESS != ret) {
                        ADF_ERROR("Could not enable sriov for device %d\n",
                                                       accel_dev->accelId);
                        return FAIL;
                }
        }
        accel_dev->virtualization.enabled = 1;
        miscBarId = hw_data->getMiscBarId(hw_data);
        pci_info = &accel_dev->pciAccelDev;
        pmisc_bar_addr = (void*)
                       pci_info->pciBars[miscBarId].virtAddr;

        if (CPA_FALSE == hw_data->isArbitrationSupported) {
                /*
                 * Turn off remapping for VF0 and VF8
                 */
                memcpy_fromio(&reg, ((uint8_t*)pmisc_bar_addr) +
                        hw_data->getTiMiscIntCtlOffset(), sizeof(reg));
                reg |= 1<<1;
                memcpy_toio(((uint8_t*)pmisc_bar_addr) +
                        hw_data->getTiMiscIntCtlOffset(), &reg, sizeof(reg));

                accel_dev->vf_rings_info = ICP_ZALLOC_GEN(sizeof(Cpa16U) *
                        GET_MAX_BANKS(accel_dev));

                if(!accel_dev->vf_rings_info) {
                        ADF_ERROR("Could not allocate memory\n");
                        return FAIL;
                }

                /* Add device to ring info table */
                status = adf_add_device_to_ring_info(accel_dev);
                if (CPA_STATUS_FAIL == status) {
                        ADF_ERROR("Could not add device to ring_info table\n");
                        ICP_FREE(accel_dev->vf_rings_info);
                        return FAIL;
                }
                
        } else {
                for (i = 0; i < AE2FUNCTION_NUM_TABLE_A; i++) {
                        ae2Function = READ_CSR_AE2FUNCTION_TABLE_A(
                                        pmisc_bar_addr, i);
                        ae2Function |= 1<< VALID_BIT;
                        WRITE_CSR_AE2FUNCTION_TABLE_A(pmisc_bar_addr, i,
                                        ae2Function);
                }
                for (i = 0; i < AE2FUNCTION_NUM_TABLE_B; i++) {
                        ae2Function = READ_CSR_AE2FUNCTION_TABLE_B(
                                        pmisc_bar_addr, i);
                        ae2Function |= 1<< VALID_BIT;
                        WRITE_CSR_AE2FUNCTION_TABLE_B(pmisc_bar_addr, i,
                                        ae2Function);
                }
        }

        /* Enable VF to PF interrupts */
        if (hw_data->enableVf2PfInterrupts) {
                hw_data->enableVf2PfInterrupts(accel_dev,
                                    ICP_VFTOPF_MSK_32VFS);
        }

        return ret;
}

/*
 * adf_disable_sriov
 * Disable accel dev virtualization
 */
void adf_disable_sriov(icp_accel_dev_t *accel_dev)
{
        uint32_t *pmisc_bar_addr = NULL;
        int i = 0;
        icp_accel_pci_info_t *pci_info = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        Cpa32U miscBarId = 0;
        Cpa32U ae2Function = 0;

        if (!accel_dev) {
                ADF_ERROR("Invalid accel device\n");
                return;
        }

        if (!accel_dev->pHwDeviceData) {
                ADF_ERROR("Invalid accel device config data\n");
                return;
        }

        hw_data = accel_dev->pHwDeviceData;
        miscBarId = hw_data->getMiscBarId(hw_data);
        pci_info = &accel_dev->pciAccelDev;
        pmisc_bar_addr = (void*)pci_info->pciBars[miscBarId].virtAddr;

        if (accel_dev->virtualization.enabled) {
                if (!BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                          ADF_STATUS_SYSTEM_RESTARTING)) {
                        pci_disable_sriov(accel_dev->pciAccelDev.pDev);
                }

                if (CPA_FALSE == hw_data->isArbitrationSupported) {
                        /* Remove device from ring info table */
                        adf_remove_device_from_ring_info(accel_dev);
                        ICP_FREE(accel_dev->vf_rings_info);
                } else {
                        /* Unset the valid bit for the AE2Function map regs */
                        for (i = 0; i < AE2FUNCTION_NUM_TABLE_A; i++) {
                                ae2Function = READ_CSR_AE2FUNCTION_TABLE_A(
                                        pmisc_bar_addr, i);
                                ae2Function &= AE_INVALID_BIT_MSK;
                                WRITE_CSR_AE2FUNCTION_TABLE_A(pmisc_bar_addr, i,
                                        ae2Function);
                        }
                        for (i = 0; i < AE2FUNCTION_NUM_TABLE_B; i++) {
                                ae2Function = READ_CSR_AE2FUNCTION_TABLE_B(
                                        pmisc_bar_addr, i);
                                ae2Function &= AE_INVALID_BIT_MSK;
                                WRITE_CSR_AE2FUNCTION_TABLE_B(pmisc_bar_addr, i,
                                        ae2Function);
                        }
                }
                accel_dev->virtualization.enabled = 0;
        }

        /* Disable VF to PF interrupts */
        if (hw_data->disableVf2PfInterrupts)
                hw_data->disableVf2PfInterrupts(accel_dev,
                        ICP_VFTOPF_MSK_32VFS);
}

int adf_init_sriov(void)
{
        spin_lock_init(&lock);
        putMsg_wq  = create_workqueue("putMsg_wq");
        return (NULL == putMsg_wq) ? FAIL : SUCCESS;
}

void adf_exit_sriov(void)
{
        
        if (putMsg_wq)
                destroy_workqueue(putMsg_wq);
        putMsg_wq = NULL;
}
#else
int adf_vf_clean_ring_info(icp_accel_pci_vf_info_t *vf)
{
}
void adf_vf_bh_handler(void* handle)
{
}
int adf_enable_sriov(icp_accel_dev_t *accel_dev)
{
}
void adf_disable_sriov(icp_accel_dev_t *accel_dev)
{
}
int adf_init_sriov(void)
{
}

void adf_exit_sriov(void)
{
}
CpaStatus adf_PF2VF_notify_restarting(icp_accel_dev_t *accel_dev)
{
}
#endif
