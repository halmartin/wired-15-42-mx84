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
 *  version: QAT1.5.L.1.11.0-36
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

#define RESTARTING_MAX_LOOPS  1000
#define RESTARTING_SLEEP_TIME 2

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
static struct workqueue_struct *ring_info_wq = NULL;
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
static DEFINE_SEMAPHORE(ring_info_sema);
#else
static DECLARE_MUTEX(ring_info_sema);
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
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

typedef struct adf_ring_update_param_s {
        icp_accel_dev_t *accel_dev;
        Cpa32U deviceId;
        adf_ring_update_info_t info;
        icp_adf_ringInfoOperation_t op;
        uint32_t *pmisc_bar_addr;
        struct work_struct ring_info_work;
} adf_ring_update_param_t;

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
 * adf_vf_ring_info_update_send
 * Send ring info update to AE
 */
static void adf_vf_ring_info_update_send(struct work_struct *work)
{
        int ret = SUCCESS;
        Cpa32U flags = (1<<ICP_RESP_TYPE_NONE);
        adf_hw_device_data_t  *hw_data = NULL;

        adf_ring_update_param_t *ring_info =
                  container_of(work, adf_ring_update_param_t, ring_info_work);
        hw_data = ring_info->accel_dev->pHwDeviceData;
        ret = adf_vf_ring_info_update(ring_info->accel_dev,
                                      ring_info->info.ring,
                                      ring_info->op,
                                      ring_info->info.inf);
        if (SUCCESS == ret) {
                /* Send ring info update ACK to VF */
                ICP_ADF_CSR_WR(ring_info->pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(ring_info->deviceId),
                    ICP_PMISCBAR_VF2PFMSGRINGINFOACK);
        } else {
                /* Send ring info update error to VF */
                ICP_ADF_CSR_WR(ring_info->pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(ring_info->deviceId),
                    ICP_PMISCBAR_VF2PFMSGRINGINFOERR);
        }
        /* Enable IRQ for this VF */
        ICP_ADF_CSR_WR(ring_info->pmisc_bar_addr,
                    hw_data->getVIntMskOffset(ring_info->deviceId), 0);
        /* Now after ME is updated we can remove the ring from autopush */
        if (ICP_ADF_RING_DISABLE == ring_info->op) {
                CpaStatus stat;
                spin_lock_bh(&lock);
                stat = adf_etrApClrRingRegister(ring_info->accel_dev,
                                            ring_info->info.ring, flags);
                spin_unlock_bh(&lock);
                if (CPA_STATUS_SUCCESS != stat) {
                        ADF_ERROR("Autopush update failed\n");
                }
        }
        ICP_FREE(ring_info);
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
    adf_ring_update_info_t *info       = NULL;
    Cpa32U numRingsInDevice = 0;
    Cpa32U sizeOfRingInfoArray = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    numRingsInDevice = GET_NUM_RINGS_PER_BANK(accel_dev)
										* GET_MAX_BANKS(accel_dev);
    sizeOfRingInfoArray = sizeof (adf_ring_update_info_t) * numRingsInDevice;
    info = ICP_MALLOC_GEN_NUMA(sizeOfRingInfoArray, accel_dev->pkg_id);
    if (NULL == info){
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
        for(i = 0; i < numRingsPerBank; i++, tbl++)
        {
                ring_info = *tbl;
                if (ring_info) {
                        if (ring_info->flags) {
                                /* Send ring info msg to ME */
                                ret = adf_vf_ring_info_update(vf->accel_dev,
                                                      ring_info->ring,
                                                      ICP_ADF_RING_DISABLE,
                                                      ring_info->inf);
                                if (SUCCESS != ret) {
                                        ADF_ERROR("Failed to send ring info"
                                                   " update to AE\n");
                                }
                        }
                        /* Now after ME is updated we can remove the ring from
                         * autopush */
                        spin_lock_bh(&lock);
                        adf_etrApClrRingRegister(vf->accel_dev,
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
        uint32_t *pmisc_bar_addr = NULL, mask = 0;
        Cpa32U flags = 0, ring = 0;
        Cpa32U numRingsPerBank = 0;
        CpaStatus stat = CPA_STATUS_SUCCESS;
        icp_accel_pci_vf_info_t *vf = (icp_accel_pci_vf_info_t*) handle;
        icp_accel_dev_t *accel_dev = vf->accel_dev;
        icp_accel_pci_info_t *pci_info = &accel_dev->pciAccelDev;
        adf_ring_update_param_t *param = NULL;
        adf_hw_device_data_t  *hw_data = NULL;
        Cpa32U pmisc_bar_id = 0;
        Cpa32U defVfIntMask = 0;

        hw_data = accel_dev->pHwDeviceData;
        pmisc_bar_id = hw_data->getMiscBarId(hw_data);

        pmisc_bar_addr = (void*)
                         pci_info->pciBars[pmisc_bar_id].virtAddr;

        hw_data = accel_dev->pHwDeviceData;

        /* Read the message from VF */
        mask = ICP_ADF_CSR_RD(pmisc_bar_addr,
                hw_data->getPf2VfDbIntOffset(vf->deviceId));
        /* Clean interrupt */
        ICP_ADF_CSR_WR(pmisc_bar_addr,
                hw_data->getPf2VfDbIntOffset(vf->deviceId), 0);
        /* Disable IRQ for this VF */
        defVfIntMask = hw_data->getVIntMskDefault();
        ICP_ADF_CSR_WR(pmisc_bar_addr,
            hw_data->getVIntMskOffset(vf->deviceId), defVfIntMask);
        /* Enable VF2PF IRQs */
        ICP_ADF_CSR_WR(pmisc_bar_addr, hw_data->getVf2PfIntMaskOffset(), 0);
        numRingsPerBank = GET_NUM_RINGS_PER_BANK(accel_dev);

        /* Get msg type */
        if (ICP_PMISCBAR_VF2PFMSGTYPERESPONSE ==
                                     adf_vf2pf_get_msg_type(mask)) {
            ADF_ERROR("not request msg\n");
            /* Enable IRQ for this VF */
            ICP_ADF_CSR_WR(pmisc_bar_addr,
                hw_data->getVIntMskOffset(vf->deviceId), 0);
            /* Send ring info update error to VF */
            ICP_ADF_CSR_WR(pmisc_bar_addr,
                            hw_data->getPf2VfDbIntOffset(vf->deviceId),
                                ICP_PMISCBAR_VF2PFMSGRINGINFOERR);
            return;
        }
        /* Get msg type */
        if (ICP_PMISCBAR_VF2PFMSGTYPEINIT ==
                                     adf_vf2pf_get_msg_type(mask)) {
            vf->init = 1;
            /* this is a request for VF init
             * so ack and enable back the vf irq */
            mask = ICP_PMISCBAR_VF2PFMSGRINGINFOACK;

            ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(vf->deviceId), mask);
            /* Enable IRQ for this VF */
            ICP_ADF_CSR_WR(pmisc_bar_addr,
                hw_data->getVIntMskOffset(vf->deviceId), 0);
            return;
        }
        /* Get msg type */
        if (ICP_PMISCBAR_VF2PFMSGTYPESHUTDOWN ==
                                     adf_vf2pf_get_msg_type(mask)) {
            vf->init = 0;
            /* this is a request for VF shutdown
             * so ack and enable back the vf irq */
            mask = ICP_PMISCBAR_VF2PFMSGRINGINFOACK;

            ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(vf->deviceId), mask);
            /* Enable IRQ for this VF */
            ICP_ADF_CSR_WR(pmisc_bar_addr,
                hw_data->getVIntMskOffset(vf->deviceId), 0);
            return;
        }
        /* Get msg type */
        if (ICP_PMISCBAR_VF2PFMSGTYPERINGOFFSET ==
                                     adf_vf2pf_get_msg_type(mask)) {
            /* this is a request for VF ring offset
             * so write the offset to the reg, ack and
             * enable back the vf irq */
            mask = (vf->deviceId * numRingsPerBank)
                                  << ICP_PMISCBAR_VF2PFMSGRINGINFOACKSHIFT;
            mask |= ICP_PMISCBAR_VF2PFMSGRINGINFOACK;

            ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(vf->deviceId), mask);
            /* Enable IRQ for this VF */
            ICP_ADF_CSR_WR(pmisc_bar_addr,
                hw_data->getVIntMskOffset(vf->deviceId), 0);
            return;

        }
        /* this is a request for ring operation */
        /* Get ring number */
        ring = adf_vf2pf_get_ring_number(mask);
        /* Check if it is valid */
        if ((ring > numRingsPerBank - 1) || (ring < 0)) {
                ADF_ERROR("Invalid ring number: %d from VF: %d\n",
                                                 ring, vf->deviceId);
                /* enable IRQ from this VF */
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getVIntMskOffset(vf->deviceId), 0);
                /* Send ring info update error to VF */
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(vf->deviceId),
                        ICP_PMISCBAR_VF2PFMSGRINGINFOERR);
                return;
        }

        /* Calculate the real ring number */
        ring = ring + (vf->deviceId * numRingsPerBank);
        /* Set flag for response ring */
        if (!adf_vf2pf_get_response_ring(mask)) {
                flags = (1<<ICP_RESP_TYPE_NONE);
        }

        /* If this is ring enable operation */
        if (ICP_ADF_RING_ENABLE == adf_vf2pf_get_operation(mask)) {
                adf_ring_update_info_t **tbl = NULL;
                /* Set autopush register */
                spin_lock_bh(&lock);
                stat = adf_etrApSetRingRegister(accel_dev,
                                                ring, flags);
                spin_unlock_bh(&lock);
                /* Store ring info entry in vf info */
                info_tbl[accel_dev->accelId][ring].inf =
                                            adf_vf2pf_get_ring_info(mask);
                info_tbl[accel_dev->accelId][ring].flags = flags;
                info_tbl[accel_dev->accelId][ring].ring = ring;
                tbl = vf->vf_ring_info_tbl;
                tbl[ring % numRingsPerBank] =
                                      &info_tbl[accel_dev->accelId][ring];
        } else {
                adf_ring_update_info_t **tbl = NULL;
                /* Here can only remove the Rx rings from autopush
                 * for Tx rings it needs to be done after the ME
                 * is notified
                 */
                if (!flags) {
                        spin_lock_bh(&lock);
                        stat = adf_etrApClrRingRegister(accel_dev,
                                                ring, flags);
                        spin_unlock_bh(&lock);
                }
                /* Remove ring info entry from vf info */
                tbl = vf->vf_ring_info_tbl;
                tbl[ring % numRingsPerBank] = NULL;
        }
        if (stat != CPA_STATUS_SUCCESS) {
                ADF_ERROR("Autopush update failed\n");
                /* enable IRQ from this VF */
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getVIntMskOffset(vf->deviceId), 0);
                /* Send ring info update error to VF */
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(vf->deviceId),
                        ICP_PMISCBAR_VF2PFMSGRINGINFOERR);
                return;
        }
        if (!flags) {
                /* Ring info is for request rings only.
                 * This is a response ring so dont need to send
                 * info to firmware. Enable IRQ for this VF */
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getVIntMskOffset(vf->deviceId), 0);
                /* Send ring info update ACK to VF */
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(vf->deviceId),
                        ICP_PMISCBAR_VF2PFMSGRINGINFOACK);
                return;
        }

        /* Can not call the ring info callback from this context
         * because it wais. Need to enqueue the ring info update
         * operation */
        param = ICP_MALLOC_ATOMIC(sizeof(adf_ring_update_param_t));
        if (param) {
                memset(param, '\0', sizeof(adf_ring_update_param_t));
                param->info.ring = ring;
                param->info.inf = adf_vf2pf_get_ring_info(mask);
                param->op = adf_vf2pf_get_operation(mask);
                param->accel_dev = accel_dev;
                param->deviceId = vf->deviceId;
                param->pmisc_bar_addr = pmisc_bar_addr;
                INIT_WORK(&param->ring_info_work,
                           adf_vf_ring_info_update_send);
                queue_work(ring_info_wq, &param->ring_info_work);
                return;
        }
        /* Allocation failed print error msg, send error to the VF
         * and enable VF IRQ */
        ADF_ERROR("Failed to allocate memory for ring info parameter\n");
        /* Enable IRQ for this VF */
        ICP_ADF_CSR_WR(pmisc_bar_addr,
            hw_data->getVIntMskOffset(vf->deviceId), 0);
        /* Send ring info update error to VF */
        ICP_ADF_CSR_WR(pmisc_bar_addr,
                        hw_data->getPf2VfDbIntOffset(vf->deviceId),
                            ICP_PMISCBAR_VF2PFMSGRINGINFOERR);
}

/*
 * Notify VF about restarting event
 */
CpaStatus adf_PF2VF_notify(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U *reg = NULL, val = 0, ctr = 0;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U pf2vf_bar_id = 0;
    Cpa32U pf2vf_dbint_offset = 0;
    icp_accel_pci_vf_info_t *vf = NULL;
    Cpa32U dev_nr = 0;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);

    hw_data = accel_dev->pHwDeviceData;
    pf2vf_bar_id = hw_data->getMiscBarId(hw_data);

    reg = (void*)
        accel_dev->pciAccelDev.pciBars[pf2vf_bar_id].virtAddr;

    /* Build a notify message to VF */
    adf_pf2vf_set_msg_type(&val, ICP_PMISCBAR_PF2VFMSGTYPERESTARTING);
    adf_pf2vf_set_irq(&val);

    for(dev_nr = 0; dev_nr < hw_data->maxNumVf; dev_nr++)
    {
        if(NULL == accel_dev->pci_vfs[dev_nr]) continue;
        vf = (icp_accel_pci_vf_info_t *)accel_dev->pci_vfs[dev_nr];
        if(1 == vf->attached) continue;
        if(0 == vf->init) continue;

        pf2vf_dbint_offset = hw_data->getPf2VfDbIntOffset(dev_nr);
        /* Send the message */
        ICP_ADF_CSR_WR(reg, pf2vf_dbint_offset, val);

        /* And wait for confirmation */
        do{
            ICP_MSLEEP(RESTARTING_SLEEP_TIME);
            val = ICP_ADF_CSR_RD(reg, pf2vf_dbint_offset);
        } while((ICP_PMISCBAR_PF2VFMSGACK != val) &&
                                               ctr++ < RESTARTING_MAX_LOOPS);
        /* Return success only if we got ACK from VF */
        status = (ICP_PMISCBAR_PF2VFMSGACK == val) ? CPA_STATUS_SUCCESS :
                                                             CPA_STATUS_FAIL;
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
        int ret = FAIL, i = 0;
        uint32_t *pmisc_bar_addr = NULL;
        uint32_t reg = 0, vfs = 0;
        icp_accel_pci_info_t *pci_info = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        Cpa32U miscBarId = 0;
        CpaStatus status = CPA_STATUS_FAIL;

        if (!accel_dev) {
                ADF_ERROR("Invalid dev handle\n");
                return ret;
        }

        if (!accel_dev->pHwDeviceData) {
                ADF_ERROR("Invalid dev config data\n");
                return ret;
        }
        hw_data = accel_dev->pHwDeviceData;
        accel_dev->vf_rings_info =
                 ICP_ZALLOC_GEN(sizeof(Cpa16U) * GET_MAX_BANKS(accel_dev));
        if (!accel_dev->vf_rings_info) {
                ADF_ERROR("Could not allocate memory\n");
                return ret;
        }
        /* Add device to ring info table */
        status = adf_add_device_to_ring_info(accel_dev);
        if (CPA_STATUS_FAIL == status) {
            ADF_ERROR("Could not add device to ring_info table\n");
            ICP_FREE(accel_dev->vf_rings_info);
            return ret;
        }
        /*
         * Check number of VF for lower SKUs with one Accelelator.
         */
        vfs = (hw_data->maxNumAccel ==
                 hw_data->getNumAccelerators(hw_data, accel_dev->accelMask)) ?
                     hw_data->maxNumVf : hw_data->maxNumVf >> 1;

        if (!BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                  ADF_STATUS_SYSTEM_RESTARTING))
        {
            ret = pci_enable_sriov(accel_dev->pciAccelDev.pDev, vfs);
            if (SUCCESS != ret) {
                    ADF_ERROR("Could not enable sriov for device %d\n",
                                                   accel_dev->accelId);
                    ICP_FREE(accel_dev->vf_rings_info);
                    adf_remove_device_from_ring_info(accel_dev);
                    return ret;
            }
        }
        ret = 0;
        accel_dev->virtualization.enabled = 1;

        miscBarId = hw_data->getMiscBarId(hw_data);
        pci_info = &accel_dev->pciAccelDev;
        pmisc_bar_addr = (void*)
                       pci_info->pciBars[miscBarId].virtAddr;
        /*
         * Turn off remapping for VF0 abd VF8
         */
        memcpy_fromio(&reg, ((uint8_t*)pmisc_bar_addr) +
                hw_data->getTiMiscIntCtlOffset(), sizeof(reg));
        reg |= 1<<1;
        memcpy_toio(((uint8_t*)pmisc_bar_addr) +
                hw_data->getTiMiscIntCtlOffset(), &reg, sizeof(reg));
        /*
         * Enable VF2PF interrupts
         */
        ICP_ADF_CSR_WR(pmisc_bar_addr,
                hw_data->getVf2PfIntMaskOffset(), 0);
        for(i = 0; i < hw_data->maxNumVf; i++)
        {
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getVIntMskOffset(i), 0);
        }
        return ret;
}

/*
 * adf_sriov_disable
 * Disable accel dev virtualization
 */
void adf_disable_sriov(icp_accel_dev_t *accel_dev)
{
        uint32_t *pmisc_bar_addr = NULL;
        int i = 0;
        icp_accel_pci_info_t *pci_info = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        Cpa32U miscBarId = 0;
        Cpa32U defVfIntMask = 0;
        Cpa32U defVf2PfMask = 0;

        if (accel_dev->virtualization.enabled) {
                if (!BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                          ADF_STATUS_SYSTEM_RESTARTING))
                {
                    pci_disable_sriov(accel_dev->pciAccelDev.pDev);
                }
                /* Remove device from ring info table */
                adf_remove_device_from_ring_info(accel_dev);
                accel_dev->virtualization.enabled = 0;
                ICP_FREE(accel_dev->vf_rings_info);
                /* Disable VF2PF interrupts */
                hw_data = accel_dev->pHwDeviceData;
                miscBarId = hw_data->getMiscBarId(hw_data);
                pci_info = &accel_dev->pciAccelDev;
                pmisc_bar_addr = (void*)
                       pci_info->pciBars[miscBarId].virtAddr;
                defVf2PfMask = hw_data->getVf2PfIntMaskDefault();
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getVf2PfIntMaskOffset(), defVf2PfMask);
                hw_data = accel_dev->pHwDeviceData;
                defVfIntMask = hw_data->getVIntMskDefault();
                for(i = 0; i < hw_data->maxNumVf; i++)
                {
                        ICP_ADF_CSR_WR(pmisc_bar_addr,
                                hw_data->getVIntMskOffset(i), defVfIntMask);
                }
        }
}

int adf_init_sriov(void)
{
        spin_lock_init(&lock);
        ring_info_wq = create_workqueue("ring_info_wq");
        return (ring_info_wq == NULL) ? FAIL : SUCCESS;
}

void adf_exit_sriov(void)
{
        if (ring_info_wq)
                destroy_workqueue(ring_info_wq);
        ring_info_wq = NULL;
}
#else
int adf_vf_clean_ring_info(icp_accel_pci_vf_info_t *vf)
{
    return SUCCESS;
}
void adf_vf_bh_handler(void* handle)
{
}
int adf_enable_sriov(icp_accel_dev_t *accel_dev)
{
    return SUCCESS;
}
void adf_disable_sriov(icp_accel_dev_t *accel_dev)
{
}
int adf_init_sriov(void)
{
    return SUCCESS;
}

void adf_exit_sriov(void)
{
}
CpaStatus adf_PF2VF_notify(icp_accel_dev_t *accel_dev)
{
    return CPA_STATUS_SUCCESS;
}

#endif
