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
 * @file adf_poll_tasklets.h
 *
 * @description
 *      File contains function prototypes for initialising the tasklets for
 *      ring polling on a per ring basis.
 *
 *****************************************************************************/
#ifndef ADF_POLL_TASKLETS_H
#define ADF_POLL_TASKLETS_H

/******************************************************************************
 * @description
 * adf_setup_poll_tasklets
 * Allocate tasklet resources for Ring Polling
 * Returns: 0 Success, 1 Failure
 *****************************************************************************/
int adf_setup_poll_tasklets(icp_accel_dev_t *accel_dev,
                            Cpa32U ring_num_in_bank,
                            Cpa32U bank_num);

/******************************************************************************
 * @description
 * adf_cleanup_poll_tasklets
 * Cleanup any allocated tasklet resources
 * used for Ring Polling
 * Returns: 0 Success, 1 Failure
 *****************************************************************************/
int adf_cleanup_poll_tasklets(icp_accel_dev_t *accel_dev,
                              Cpa32U ring_num_in_bank,
                              Cpa32U bank_num);

/******************************************************************************
 * @description
 * adf_scheduleBankTasklet
 * Schedule a tasklet to poll the Bank
 *****************************************************************************/
void adf_schedule_bank_tasklet(icp_et_ring_bank_data_t *bank);

/******************************************************************************
 * @description
 * adf_scheduleRingTasklet
 * Schedule a tasklet to poll the particular ring.
 *****************************************************************************/
void adf_schedule_ring_tasklet(icp_et_ring_data_t* ring);

#endif /* ADF_POLL_TASKLETS_H */
