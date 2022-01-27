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
 * @file adf_poll_tasklets.c
 *
 * @description
 *    This file contains ADF code for setting up tasklets for the Ring Polling
 *    These tasklets are setup on a per ring basis and called
 *    by the createhandle function. Also included here are the tasklet
 *    scheduling functions employed by the OS.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_poll_tasklets.h"

/*
 * Setup & Initialise the tasklet structs for this device
 */
int adf_setup_poll_tasklets(icp_accel_dev_t *accel_dev,
                                Cpa32U ring_num_in_bank,
                                Cpa32U bank_number)
{
        icp_etr_priv_data_t *priv_data = NULL;
        icp_et_ring_bank_data_t *bank=NULL;
        icp_et_ring_data_t *ring=NULL;

        ICP_CHECK_FOR_NULL_PARAM(accel_dev);

        priv_data = (icp_etr_priv_data_t*) accel_dev->pCommsHandle;
        ICP_CHECK_FOR_NULL_PARAM(priv_data);

        bank = &priv_data->banks[bank_number];

        ring = &bank->rings[ring_num_in_bank];

        ring->ETR_ring_tasklet_poll =
        ICP_ZALLOC_GEN(sizeof(struct tasklet_struct));
        if (!ring->ETR_ring_tasklet_poll) {
                ADF_ERROR("Failed to allocate ETR Ring poll tasklet\n");
                return CPA_STATUS_FAIL;
        }
        tasklet_init(ring->ETR_ring_tasklet_poll,
                        (void*)adf_ringResponsePolling,
                        (unsigned long)ring);
        /*Initialise the tasklet flag to one. */
        osalAtomicSet(1, &ring->pollingTaskletScheduled);
        return CPA_STATUS_SUCCESS;
}

/*
 * Cleanup Tasklet structures
 */
int adf_cleanup_poll_tasklets(icp_accel_dev_t *accel_dev,
                                Cpa32U ring_num_in_bank,
                                Cpa32U bank_number)
{
        icp_etr_priv_data_t *priv_data = NULL;
        icp_et_ring_bank_data_t *bank=NULL;
        icp_et_ring_data_t *ring=NULL;

        ICP_CHECK_FOR_NULL_PARAM(accel_dev);

        priv_data = (icp_etr_priv_data_t*) accel_dev->pCommsHandle;
        ICP_CHECK_FOR_NULL_PARAM(priv_data);

        bank = &priv_data->banks[bank_number];
        ring = &bank->rings[ring_num_in_bank];

        tasklet_disable(ring->ETR_ring_tasklet_poll);
        tasklet_kill(ring->ETR_ring_tasklet_poll);
        ICP_FREE(ring->ETR_ring_tasklet_poll);
        ring->ETR_ring_tasklet_poll = NULL;


        return CPA_STATUS_SUCCESS;
}


/* Schedule tasklets on a per bank basis for the Ring Polling. */
void adf_schedule_bank_tasklet(icp_et_ring_bank_data_t *bank)
{
        tasklet_hi_schedule(bank->ETR_bank_tasklet_poll);
}

/* Schedule tasklets on a per ring basis for the Ring Polling. */
void adf_schedule_ring_tasklet(icp_et_ring_data_t* ring)
{
        tasklet_hi_schedule(ring->ETR_ring_tasklet_poll);
}
