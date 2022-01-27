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
 * @file adf_transport_debug.c
 *
 * @description
 *      Transport debug code in which ADF uses the debug feature to show the
 *      contents of the rings in the proc filesystem
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_debug.h"
#include "adf_pfvfcomms.h"
#include "adf_platform_common.h"
#include "adf_platform_acceldev_common.h"
#include "adf_drv_sriov.h"

#define ADF_DEBUG_PFVF "pfvf"

typedef struct debug_private_s{
    icp_accel_dev_t *accel_dev;
} debug_private_t;


/*
 * adf_debug_print_bank_rings
 * Print the ring info of a bank
 */
STATIC int adf_debug_vfpf_print(void *prv, char* data,
                                int size, int offset)
{
        Cpa32U len = 0;
        static Cpa16U vfNumber;
        icp_accel_dev_t *accel_dev;
        Cpa32U *reg = NULL;
        icp_accel_pci_info_t *pci_info = NULL;
        icp_accel_pci_vf_info_t *vf;
        Cpa32U pmisc_bar_id = 0;
        adf_hw_device_data_t  *hw_data = NULL;
        Cpa32U message = 0,  maxNumPfVf = 0;
        Cpa32U doorbell_csr_offset = 0;

        if (NULL == data) {
                /* We can't print the error message. */
                return 0;
        }

        if (NULL == prv) {
                len = snprintf(data+len, size-len,
                               "Internal error adf_debug_vfpf_print: prv NULL\n");
                return 0;
        }
        accel_dev = ((debug_private_t *)prv)->accel_dev;
        if (NULL == accel_dev) {
                len = snprintf(data+len, size-len,
                               "Internal error adf_debug_vfpf_print: accel_dev NULL\n");
                return 0;
        }
        if (NULL == accel_dev->pHwDeviceData) {
                len = snprintf(data+len, size-len,
                               "Internal error adf_debug_vfpf_print: accel_dev->pHwDeviceData NULL\n");
                return 0;
        }

        hw_data = accel_dev->pHwDeviceData;
        if (NULL == hw_data->getMiscBarId) {
                len = snprintf(data+len, size-len,
                               "Internal error adf_debug_vfpf_print: hw_data->getMiscBarId NULL\n");
                return 0;
        }
        pmisc_bar_id = hw_data->getMiscBarId(hw_data);
        pci_info = &accel_dev->pciAccelDev;

        reg = (void*)pci_info->pciBars[pmisc_bar_id].virtAddr;
        if (NULL == reg) {
                len = snprintf(data+len, size-len,
                               "Internal error adf_debug_vfpf_print: pci_info->pciBars[pmisc_bar_id].virtAddr NULL\n");
                return 0;
        }


        if (NULL == hw_data->getPf2VfDbIntOffset) {
                len = snprintf(data+len, size-len,
                               "Internal error adf_debug_vfpf_print: hw_data->getPf2VfDbIntOffset NULL\n");
                return 0;
        }

        if(offset == 0) {
                len = snprintf(data+len, size-len,
                               "------- PFVF CSR -------\n");
        }
#define ADF_DEBUG_LINE_LEN sizeof("VF 32, PF2VF CSR 0x%12345678: VF to PF system msg 0x1234 PF to VF system msg 0x1234\n")
        if (0 == offset) {
                vfNumber = 0;
        }

        if (accel_dev->virtualization.enabled) {
            /* on the Host(PF) */
            maxNumPfVf = hw_data->maxNumVf;
        } else {
            /* on a Guest (VM) */
            maxNumPfVf = ADF_PFVF_MAX_NUM_PF;
        }

        for(; vfNumber <  maxNumPfVf; vfNumber++) {
                if (len + ADF_DEBUG_LINE_LEN > size) {
                        break; /* Come back again for more data. */
                }
                /* VFs are reported as 1..32 not 0..31 */
                if (accel_dev->virtualization.enabled) {
                        /* On the PF */
                        len += snprintf(data+len, size-len,
                                "VF %02d, ", vfNumber+1);

                        if (NULL == accel_dev->pci_vfs[vfNumber]) {
                            len = snprintf(data+len, size-len,
                                    "Internal error adf_debug_vfpf_print: "
                                    "accel_dev->pci_vfs[%d] NULL\n", vfNumber);
                            return 0;
                        }
                        vf = accel_dev->pci_vfs[vfNumber];
                        doorbell_csr_offset = hw_data->getPf2VfDbIntOffset(vf->deviceId);
                } else {
                        doorbell_csr_offset = hw_data->getPf2VfDbIntOffset(0);
                }


                message = ICP_ADF_CSR_RD(reg, doorbell_csr_offset);

                len += snprintf(data+len, size-len,
                                "PF2VF CSR 0x%08X: ", message);
                if ((ICP_IN_USE_BY_PF != (message & ICP_IN_USE_BY_PF_MASK))
                            &&
                    (0 != (message & ICP_IN_USE_BY_PF_MASK))) {
                        len += snprintf(data+len, size-len,
                                        "VF to PF %s msg 0x%04X ",
                                        (ICP_PFVF_MSGORIGIN_SYSTEM ==
                                         adf_vf2pf_get_msg_origin(message))?"system":"user",
                                        adf_vf2pf_get_userSpace_msg(message));
                }
                if ((ICP_IN_USE_BY_VF != (message & ICP_IN_USE_BY_VF_MASK))
                            &&
                    (0 != (message & ICP_IN_USE_BY_VF_MASK))) {
                        len += snprintf(data+len, size-len,
                                        "PF to VF %s msg 0x%04X",
                                        (ICP_PFVF_MSGORIGIN_SYSTEM ==
                                         adf_pf2vf_get_msg_origin(message))?"system":"user",
                                        adf_pf2vf_get_userSpace_msg(message));
                }

                len += snprintf(data+len, size-len, "\n");
        }
        offset += len;

        if (vfNumber >= maxNumPfVf) {
                offset = 0;
        }

        return offset;
}


/*
 * adf_debug_create_pfvf
 * Create a debug file for pfvf
 */
CpaStatus adf_debug_create_pfvf(icp_accel_dev_t *accel_dev)
{
        debug_file_info_t *file;
        debug_private_t *private;


        file = ICP_MALLOC_GEN(sizeof(debug_file_info_t));
        if(NULL == file)
        {
                ADF_ERROR("Failed to allocate memory for debug file\n");
                return CPA_STATUS_FAIL;
        }
        file->name = ADF_DEBUG_PFVF;
        file->parent = NULL;
        file->seq_read = adf_debug_vfpf_print;
        private = ICP_MALLOC_GEN(sizeof(debug_private_t));
        if(NULL == private)
        {
                ADF_ERROR("Failed to allocate memory for private data\n");
                ICP_FREE(file);
                return CPA_STATUS_FAIL;
        }
        private->accel_dev = accel_dev;
        file->private_data = private;

        if( CPA_STATUS_SUCCESS != icp_adf_debugAddFile(accel_dev, file))
        {
                ADF_ERROR("Failed to create debug file for pfvf\n");
                ICP_FREE(file);
                return CPA_STATUS_FAIL;
        }

        accel_dev->pfvfDebugHandle = file;
        return CPA_STATUS_SUCCESS;
}


/*
 * adf_debug_remove_pfvf
 * Remove debug file for pfvf
 */
void adf_debug_remove_pfvf(icp_accel_dev_t *accel_dev)
{
        debug_file_info_t *file =
                (debug_file_info_t *)accel_dev->pfvfDebugHandle;

        if (NULL != file)
        {
                icp_adf_debugRemoveFile(file);
                ICP_FREE(file);
                accel_dev->pfvfDebugHandle = NULL;
        }
}

