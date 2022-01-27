/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2014 Intel Corporation.
  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  qat-linux@intel.com

  BSD LICENSE
  Copyright(c) 2014 Intel Corporation.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef ADF_CFG_COMMON_H_
#define ADF_CFG_COMMON_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define ADF_CFG_MAX_STR_LEN 128
#define ADF_CFG_MAX_KEY_LEN_IN_BYTES ADF_CFG_MAX_STR_LEN
#define ADF_CFG_MAX_VAL_LEN_IN_BYTES ADF_CFG_MAX_STR_LEN
#define ADF_CFG_MAX_SECTION_LEN_IN_BYTES ADF_CFG_MAX_STR_LEN
#define ADF_CFG_BASE_DEC 10
#define ADF_CFG_BASE_HEX 16
#define ADF_CFG_ALL_DEVICES 0xFFFE
#define ADF_CFG_NO_DEVICE 0xFFFF
#define ADF_CFG_AFFINITY_WHATEVER 0xFF
#define MAX_DEVICE_NAME_SIZE 32
#define ADF_MAX_DEVICES (32 * 32)
#define ADF_CFG_STORAGE_ENABLED 1
#define ADF_DEVS_ARRAY_SIZE BITS_TO_LONGS(ADF_MAX_DEVICES)
#define ADF_SSM_WDT_DEFAULT_VALUE 0x3000000
#define ADF_MIN_HB_TIMER_MS 100
#define ADF_CFG_MAX_NUM_OF_SECTIONS 16
#define ADF_CFG_MAX_NUM_OF_TOKENS 16
#define ADF_CFG_MAX_TOKENS_IN_CONFIG 8
#define ADF_CFG_SERV_RING_PAIR_0_SHIFT 0
#define ADF_CFG_SERV_RING_PAIR_1_SHIFT 3
#define ADF_CFG_SERV_RING_PAIR_2_SHIFT 6
#define ADF_CFG_SERV_RING_PAIR_3_SHIFT 9
#define ADF_CFG_RESP_POLL 1
#define ADF_CFG_RESP_EPOLL 2
#define ADF_CFG_DEF_CY_RING_ASYM_SIZE 64
#define ADF_CFG_DEF_CY_RING_SYM_SIZE 512
#define ADF_CFG_DEF_DC_RING_SIZE 512
#define ADF_CFG_MAX_CORE_NUM 256
#define ADF_CFG_MAX_TOKENS ADF_CFG_MAX_CORE_NUM
#define ADF_CFG_MAX_TOKEN_LEN 10
#define ADF_CFG_ACCEL_DEF_COALES 1
#define ADF_CFG_ACCEL_DEF_COALES_TIMER 10000
#define ADF_CFG_ACCEL_DEF_COALES_NUM_MSG 0
#define ADF_CFG_ASYM_SRV_MASK 1
#define ADF_CFG_SYM_SRV_MASK 2
#define ADF_CFG_DC_SRV_MASK 8
#define ADF_CFG_UNKNOWN_SRV_MASK 0
#define ADF_CFG_DEF_ASYM_MASK 0x03
#define ADF_CFG_MAX_SERVICES 4

enum adf_cfg_bundle_type {
	FREE,
	KERNEL,
	USER
};

enum adf_cfg_service_type {
	NA = 0,
	CRYPTO,
	COMP,
	SYM,
	ASYM,
	USED
};

enum adf_cfg_val_type {
	ADF_DEC,
	ADF_HEX,
	ADF_STR
};

enum adf_device_type {
	DEV_UNKNOWN = 0,
	DEV_DH895XCC,
	DEV_DH895XCCVF,
	DEV_C62X,
	DEV_C62XVF,
	DEV_C3XXX,
	DEV_C3XXXVF,
	DEV_D15XX,
	DEV_D15XXVF
};

struct adf_dev_status_info {
	enum adf_device_type type;
	u32 accel_id;
	u32 instance_id;
	u32 kpt_achandle;
	uint8_t num_ae;
	uint8_t num_accel;
	uint8_t num_logical_accel;
	uint8_t banks_per_accel;
	uint8_t state;
	uint8_t bus;
	uint8_t dev;
	uint8_t fun;
	char name[MAX_DEVICE_NAME_SIZE];
	u32 node_id;
	int domain;
};

#define ADF_CFG_HB_DEFAULT_VALUE 500
#define ADF_CFG_HB_COUNT_THRESHOLD 3

enum adf_device_heartbeat_status {
	DEV_HB_UNRESPONSIVE = 0,
	DEV_HB_ALIVE,
	DEV_HB_UNSUPPORTED
};

struct adf_dev_heartbeat_status_ctl {
	u32 device_id;
	enum adf_device_heartbeat_status status;
};

#define ADF_CTL_IOC_MAGIC 'a'
#define IOCTL_CONFIG_SYS_RESOURCE_PARAMETERS _IOW(ADF_CTL_IOC_MAGIC, 0, \
		struct adf_user_cfg_ctl_data)
#define IOCTL_STOP_ACCEL_DEV _IOW(ADF_CTL_IOC_MAGIC, 1, \
		struct adf_user_cfg_ctl_data)
#define IOCTL_START_ACCEL_DEV _IOW(ADF_CTL_IOC_MAGIC, 2, \
		struct adf_user_cfg_ctl_data)
#define IOCTL_STATUS_ACCEL_DEV _IOW(ADF_CTL_IOC_MAGIC, 3, uint32_t)
#define IOCTL_GET_NUM_DEVICES _IOW(ADF_CTL_IOC_MAGIC, 4, int32_t)
#define IOCTL_HEARTBEAT_ACCEL_DEV _IOW(ADF_CTL_IOC_MAGIC, 15, \
				struct adf_dev_heartbeat_status_ctl)
#ifdef QAT_HB_FAIL_SIM
#define IOCTL_HEARTBEAT_SIM_FAIL _IOW(ADF_CTL_IOC_MAGIC, 99, uint32_t)
#endif


#define IOCTL_GET_CFG_VAL _IOW(ADF_CTL_IOC_MAGIC, 5, \
			       struct adf_user_cfg_ctl_data)
#define IOCTL_RESERVE_RING _IOW(ADF_CTL_IOC_MAGIC, 6, \
				struct adf_user_reserve_ring)
#define IOCTL_RELEASE_RING _IOW(ADF_CTL_IOC_MAGIC, 7, \
				struct adf_user_reserve_ring)
#define IOCTL_ENABLE_RING _IOW(ADF_CTL_IOC_MAGIC, 8, \
			       struct adf_user_reserve_ring)
#define IOCTL_DISABLE_RING _IOW(ADF_CTL_IOC_MAGIC, 9, \
		 		struct adf_user_reserve_ring)
#define IOCTL_RESET_ACCEL_DEV _IOW(ADF_CTL_IOC_MAGIC, 10, \
				struct adf_user_cfg_ctl_data)


#endif
