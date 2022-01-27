/******************************************************************************
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

/******************************************************************************
 * @file adf_dev_ring.h
 *
 * @description
 *         This is the header file for the ADF RING device driver
 *
 *****************************************************************************/
#ifndef ADF_DEV_RING_H
#define ADF_DEV_RING_H

#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"

/*
 * adf_ring_chrdev_register
 * Register the ADF RING character device driver
 * Returns: 0 Success, 1 Failure
 */
int adf_ring_chrdev_register(void);

/*
 * adf_ring_chrdev_unregister
 * Unregister the ADF RING character device driver
 * Returns: None
 */
void adf_ring_chrdev_unregister(void);

/*
 * adf_ring_chrdev_create
 * Creates an ADF RING character device
 * Returns: None
 */
int adf_ring_chrdev_create(int accel_id);

/*
 * adf_ring_process_notify
 * Notifies an user space process about message delivery
 * Returns: None
 */
void adf_ring_process_notify(icp_et_ring_data_t *ring, Cpa16U irq_mask);

/*
 * adf_ring_process_finish_notify
 * Notifies a user space process that is should exit reading thread.
 * Returns: None
 */
void adf_ring_process_finish_notify(icp_et_ring_data_t *ring);

/*
 * adf_ring_process_ring_del_notify
 * Notify functions that wakes up a userspace process
 * waiting on the blocking read and makes the reading thread exit.
 * Returns: None
 */
void adf_ring_process_ring_del_notify(icp_et_ring_data_t *ring);
#endif /* ADF_DEV_RING_H */
