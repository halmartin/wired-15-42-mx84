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
 * @file adf_dev_ring_ctl.h
 *
 * @description
 *         This is the header file containing the IOCTL interface and
 *         commands for accessing the rings
 *
 *****************************************************************************/
#ifndef ADF_DEV_RING_CTL_H
#define ADF_DEV_RING_CTL_H

#if (defined(__linux__))
#include <linux/ioctl.h>
#endif

typedef struct adf_dev_bank_handle_s {
        uint32_t bank_number;
        uint32_t interrupt_mask;
        uint32_t pollingMask;
        union {
            void *user_bank_lock;
            uint64_t padding_user_bank_lock;
        };
        uint32_t timed_coalesc_enabled :1;
        uint32_t number_msg_coalesc_enabled :1;
} adf_dev_bank_handle_t;

/* Structure used to contain the parameters for requesting ring handle */
#pragma pack(push) /*Push the current alignement on the stack*/
#pragma pack(1)    /*Force alignement on 1 byte to support 32
                     bits user space on 64 bits kernels*/
typedef struct adf_instance_assignment_s {
        union {
            adf_service_type_t stype;
            uint64_t padding_stype;
        };
        uint32_t val;
        uint32_t accelID;
}adf_instance_assignment_t;

typedef struct adf_dev_ring_handle_s {
        /* Request Parameters */
        union {
            icp_accel_dev_t *accel_dev;
            uint64_t padding_accel_dev;
        };
        union {
            icp_transport_type trans_type;
            uint64_t padding_trans_type;
        };
        union {
            char *service_name;
            uint64_t padding_service_name;
        };
        uint32_t service_name_len;
        union {
            char *section_name;
            uint64_t padding_section_name;
        };
        uint32_t section_name_len;
        uint32_t accel_num;
        uint32_t bank_num;
        uint32_t ring_num;
        uint32_t ring_size;
        uint32_t message_size;
        union {
            icp_adf_ringInfoService_t info;
            uint64_t padding_info;
        };
        union {
            icp_trans_callback callback;
            uint64_t padding_callback;
        };
        union {
            icp_resp_deliv_method resp;
            uint64_t padding_resp;
        };
        /* Result Parameters */
        union {
            void *ring_virt_addr;
            uint64_t padding_ring_virt_addr;
        };
        union {
            icp_accel_dev_t *k_accelhandle;
            uint64_t padding_accelhandle;
        };
        union {
            icp_comms_trans_handle k_comms_handle;
            uint64_t padding_k_comms_handle;
        };
        union {
            int *ring_kmalloc_area;
            uint64_t padding_ring_kmalloc_area;
        };
        uint64_t ring_phys_base_addr;
        uint32_t interrupt_kernel_mask;
        uint32_t interrupt_user_mask;
        uint32_t pollingMask;
        uint32_t timed_coalesc_enabled :1;
        uint32_t number_msg_coalesc_enabled :1;
        uint32_t is_wireless :1;
        uint32_t is_dyn :1;
        union {
            adf_dev_bank_handle_t *bank_data;
            uint64_t padding_bank_data;
        };
        /* userspace shadow values */
         union {
             void *user_lock;
             uint64_t padding_user_lock;
         };
        uint32_t head;
        uint32_t tail;
        uint32_t modulo;
        uint32_t ringResponseQuota;
        int64_t pollingInProgress;
        union {
            Cpa64U  *dp_in_flight;
            OsalAtomic *trad_in_flight;
            uint64_t padding_in_flight;
        };
        uint32_t max_requests_inflight;
        uint32_t coal_write_count;
        uint32_t min_resps_per_head_write;
} adf_dev_ring_handle_t;
#pragma pack(pop) /*Restore Previous alignement*/
/* Structure used to contain bank and ring masks */
typedef struct adf_dev_rings_mask_s {
        Cpa32U bank_mask;
        Cpa32U ring_mask;
        union {
            Cpa32U *bank_irq_mask;
            uint64_t padding_bank_irq_mask;
        };
} adf_dev_rings_mask_t;

/* IOCTL number for use between the kernel and the user space application */
#define ADF_DEV_RING_MAGIC               'r'
#define ADF_DEV_RING_CMD_CREATEHANDLE    (0)
#define ADF_DEV_RING_CMD_RELEASEHANDLE   (1)
#define ADF_DEV_RING_CMD_UPDATEIRQMASK   (2)
#define ADF_DEV_RING_CMD_DYNGETINSTANCE  (3)
#define ADF_DEV_RING_CMD_DYNPUTINSTANCE  (4)
#define ADF_DEV_RING_CMD_DYNGETNUMAVAIL  (5)

/* IOCTL commands for requesting kernel memory */
#define ADF_DEV_RING_IOC_CREATEHANDLE \
        _IOWR(ADF_DEV_RING_MAGIC, ADF_DEV_RING_CMD_CREATEHANDLE, \
                                  adf_dev_ring_handle_t)
#define ADF_DEV_RING_IOC_RELEASEHANDLE \
        _IOR(ADF_DEV_RING_MAGIC, ADF_DEV_RING_CMD_RELEASEHANDLE, \
                                 adf_dev_ring_handle_t)
#define ADF_DEV_RING_GET_DYNINSTANCE \
        _IOWR(ADF_DEV_RING_MAGIC, ADF_DEV_RING_CMD_DYNGETINSTANCE, \
                                  adf_instance_assignment_t)
#define ADF_DEV_RING_PUT_DYNINSTANCE \
        _IOR(ADF_DEV_RING_MAGIC, ADF_DEV_RING_CMD_DYNPUTINSTANCE, \
                                  adf_instance_assignment_t)
#define ADF_DEV_RING_GETNUM_AVAIL_DYNINSTANCE \
        _IOWR(ADF_DEV_RING_MAGIC, ADF_DEV_RING_CMD_DYNGETNUMAVAIL, \
                                  adf_instance_assignment_t)

#endif /* ADF_DEV_RNG_CTL_H */
