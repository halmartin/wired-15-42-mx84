/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 * Copyright(c) 2015 Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 * qat-linux@intel.com
 *
 * BSD LICENSE
 * Copyright(c) 2015 Intel Corporation.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/device.h>

#ifdef CONFIG_PCI_IOV
#include "adf_accel_devices.h"
#include "adf_common_drv.h"
#include "adf_pf2vf_msg.h"
#include "adf_cfg.h"

#define ADF_VF2PF_CAPABILITIES_V1_VERSION 1
#define ADF_VF2PF_CAPABILITIES_V1_LENGTH  4
#define ADF_VF2PF_CAPABILITIES_V2_VERSION 2
#define ADF_VF2PF_CAPABILITIES_CAP_OFFSET 4
#define ADF_VF2PF_CAPABILITIES_V2_LENGTH  8
#define ADF_VF2PF_CAPABILITIES_V3_VERSION 3
#define ADF_VF2PF_CAPABILITIES_FREQ_OFFSET 8
#define ADF_VF2PF_CAPABILITIES_V3_LENGTH 12


static int adf_pf_capabilities_msg_provider(struct adf_accel_dev *accel_dev,
					    u8 **buffer, u8 *length,
					    u8 *block_version, u8 compatibility,
					    u8 byte_num)
{
	static u8 data[ADF_VF2PF_CAPABILITIES_V3_LENGTH] = {0};
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	u32 ext_dc_caps = hw_data->extended_dc_capabilities;
	u32 capabilities = hw_data->accel_capabilities_mask;
	u32 frequency = hw_data->clock_frequency;
	u16 byte = 0;
	u16 index = 0;

	for (byte = 0; byte < sizeof(ext_dc_caps); byte++) {
		data[byte] =
			(ext_dc_caps >> (byte * ADF_PFVF_DATA_SHIFT))
				& ADF_PFVF_DATA_MASK;
	}

	for (byte = 0, index = ADF_VF2PF_CAPABILITIES_CAP_OFFSET;
	     byte < sizeof(capabilities);
	     byte++, index++) {
		data[index] =
			(capabilities >> (byte * ADF_PFVF_DATA_SHIFT))
			& ADF_PFVF_DATA_MASK;
	}

	if (frequency) {
		for (byte = 0, index = ADF_VF2PF_CAPABILITIES_FREQ_OFFSET;
		     byte < sizeof(frequency);
		     byte++, index++) {
			data[index] =
				(frequency >> (byte * ADF_PFVF_DATA_SHIFT))
				& ADF_PFVF_DATA_MASK;
		}
		*length = ADF_VF2PF_CAPABILITIES_V3_LENGTH;
		*block_version = ADF_VF2PF_CAPABILITIES_V3_VERSION;
	} else {
		*length = ADF_VF2PF_CAPABILITIES_V2_LENGTH;
		*block_version = ADF_VF2PF_CAPABILITIES_V2_VERSION;
	}

	*buffer = data;
	return 0;
}

int adf_pf_vf_capabilities_init(struct adf_accel_dev *accel_dev)
{
	u8 data[ADF_VF2PF_CAPABILITIES_V3_LENGTH] = {0};
	u8 len = ADF_VF2PF_CAPABILITIES_V3_LENGTH;
	u8 version = ADF_VF2PF_CAPABILITIES_V2_VERSION;
	u32 extended_dc_caps = 0;
	u32 capabilities  = 0;
	u32 frequency  = 0;
	u16 byte = 0;
	u16 index = 0;

	if (!accel_dev->is_vf) {
		/* on the pf */
		if (!adf_iov_is_block_provider_registered
		    (ADF_VF2PF_BLOCK_MSG_CAP_SUMMARY))
			adf_iov_block_provider_register
			(ADF_VF2PF_BLOCK_MSG_CAP_SUMMARY,
			adf_pf_capabilities_msg_provider);
	} else if (accel_dev->vf.pf_version >=
			ADF_PFVF_COMPATIBILITY_CAPABILITIES) {
		/* on the vf */
		if (adf_iov_block_get
			(accel_dev, ADF_VF2PF_BLOCK_MSG_CAP_SUMMARY,
			 &version, data, &len)) {
			dev_err(&GET_DEV(accel_dev),
				"QAT: Failed adf_iov_block_get\n");
			return -EFAULT;
		}

		if (len < ADF_VF2PF_CAPABILITIES_V1_LENGTH) {
			dev_err(&GET_DEV(accel_dev),
				"Capabilities message truncated to %d bytes\n",
				len);
			return -EFAULT;
		}

		for (byte = 0;
		     byte < sizeof(extended_dc_caps);
		     byte++) {
			extended_dc_caps |=
				data[byte] << (byte * ADF_PFVF_DATA_SHIFT);
		}
		accel_dev->hw_device->extended_dc_capabilities =
			extended_dc_caps;

		/* Get capabilities if provided by PF */
		if (len >= ADF_VF2PF_CAPABILITIES_V2_LENGTH) {
			for (byte = 0,
			     index = ADF_VF2PF_CAPABILITIES_CAP_OFFSET;
			     byte < sizeof(capabilities);
			     byte++, index++) {
				capabilities |=
				data[index] << (byte * ADF_PFVF_DATA_SHIFT);
			}
			accel_dev->hw_device->accel_capabilities_mask =
				capabilities;
		} else {
			dev_info(&GET_DEV(accel_dev),
				 "PF did not communicate capabilities\n");
		}

		/* Get frequency if provided by the PF */
		if (len >= ADF_VF2PF_CAPABILITIES_V3_LENGTH) {
			for (byte = 0,
			     index = ADF_VF2PF_CAPABILITIES_FREQ_OFFSET;
			     byte < sizeof(frequency);
			     byte++, index++) {
				frequency |=
				    data[index] << (byte * ADF_PFVF_DATA_SHIFT);
			}
			accel_dev->hw_device->clock_frequency = frequency;
		} else {
			dev_info(&GET_DEV(accel_dev),
				 "PF did not communicate frequency\n");
		}

	} else {
		/* The PF is too old to support the extended capabilities */
		accel_dev->hw_device->extended_dc_capabilities = 0;
	}
	return 0;
}
#endif /* CONFIG_PCI_IOV */
