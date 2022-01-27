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
 * @file adf_bh.c
 *
 * @description
 *    This file contains ADF code for setting up ISR BH handlers
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/interrupt.h>

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "adf_platform.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"


/*
 * adf_setup_bh
 * Setup & Initialise the tasklet structs for this device
 */
int adf_setup_bh(icp_accel_dev_t *accel_dev)
{
        int i = 0;
        icp_etr_priv_data_t *priv_data = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        bank_handler bankBhHandler = NULL;
        bank_handler bankBhHandlerPoll = NULL;

        if(NULL == accel_dev) {
                ADF_ERROR("adf_setup_bh invalid param accel_dev\n");
                return FAIL;
        }

        priv_data = (icp_etr_priv_data_t*) accel_dev->pCommsHandle;
        if(NULL == priv_data) {
                ADF_ERROR("adf_setup_bh invalid param priv_data\n");
                return FAIL;
        }

        if(NULL == accel_dev->pHwDeviceData) {
                ADF_ERROR("adf_setup_bh invalid param pHwDeviceData\n");
                return FAIL;
        }
        hw_data = accel_dev->pHwDeviceData;

        /*
         * Setup the tasklets
         */
        for (i = 0; i < GET_MAX_BANKS(accel_dev); i++) {
                priv_data->banks[i].ETR_bank_tasklet =
                                ICP_ZALLOC_GEN(sizeof(struct tasklet_struct));
                if (!priv_data->banks[i].ETR_bank_tasklet) {
                        ADF_ERROR(
                                "Failed to allocate ETR Bank tasklet\n");
                        return FAIL;
                }
                bankBhHandler = hw_data->getBankBhHandler(i);
                tasklet_init(priv_data->banks[i].ETR_bank_tasklet,
                                (void*)bankBhHandler,
                                (unsigned long)accel_dev);

                /*
                 * Setup the tasklets for the Ring Polling on a per bank
                 * basis as well
                 */
                priv_data->banks[i].ETR_bank_tasklet_poll =
                                ICP_ZALLOC_GEN(sizeof(struct tasklet_struct));
                if (!priv_data->banks[i].ETR_bank_tasklet_poll) {
                        ADF_ERROR(
                        "Failed to allocate ETR Bank Polling tasklet\n");
                        return FAIL;
                }
                bankBhHandlerPoll = hw_data->getBankBhPollingHandler(i);
                tasklet_init(priv_data->banks[i].ETR_bank_tasklet_poll,
                                (void*)bankBhHandlerPoll,
                                (unsigned long)accel_dev);

                /*Initialise tasklet flag to zero. */
                osalAtomicSet(0,
                        &(priv_data->banks[i].pollingTaskletScheduled));
        }
        return SUCCESS;
}

/*
 * adf_cleanup_bh
 * Cleanup Tasklet structures
 */
int adf_cleanup_bh(icp_accel_dev_t *accel_dev)
{
        int i = 0;
        icp_etr_priv_data_t *priv_data = NULL;


        if(NULL == accel_dev) {
                ADF_ERROR("adf_cleanup_bh invalid param accel_dev\n");
                return FAIL;
        }
        priv_data = (icp_etr_priv_data_t*) accel_dev->pCommsHandle;
        if(NULL == priv_data) {
                ADF_ERROR("adf_cleanup_bh invalid param priv_data\n");
                return FAIL;
        }

        if(NULL == accel_dev->pHwDeviceData) {
                ADF_ERROR("adf_cleanup_bh invalid param pHwDeviceData\n");
                return FAIL;
        }

        for (i = 0; i < GET_MAX_BANKS(accel_dev); i++) {
                if (priv_data->banks[i].ETR_bank_tasklet != NULL) {
                        tasklet_disable(priv_data->banks[i].ETR_bank_tasklet);
                        tasklet_kill(priv_data->banks[i].ETR_bank_tasklet);
                        ICP_FREE(priv_data->banks[i].ETR_bank_tasklet);
                        priv_data->banks[i].ETR_bank_tasklet = NULL;
                }
                /*Cleanup tasklets used for bank polling. */
                if (priv_data->banks[i].ETR_bank_tasklet_poll != NULL) {
                        tasklet_disable(
                                priv_data->banks[i].ETR_bank_tasklet_poll);
                        tasklet_kill(
                                priv_data->banks[i].ETR_bank_tasklet_poll);
                        ICP_FREE(
                                priv_data->banks[i].ETR_bank_tasklet_poll);
                        priv_data->banks[i].ETR_bank_tasklet_poll = NULL;
                }
        }
        return SUCCESS;
}
