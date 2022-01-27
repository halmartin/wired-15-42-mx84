/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2015 Intel Corporation.
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
  Copyright(c) 2015 Intel Corporation.
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
#include <adf_accel_devices.h>
#include <adf_pf2vf_msg.h>
#include <adf_common_drv.h>
#include "adf_c3xxxvf_hw_data.h"
#include "icp_qat_hw.h"
#include "adf_cfg.h"

static struct adf_hw_device_class c3xxxiov_class = {
	.name = ADF_C3XXXVF_DEVICE_NAME,
	.type = DEV_C3XXXVF,
	.instances = 0
};

static u32 get_accel_mask(struct adf_accel_dev *accel_dev)
{
	return ADF_C3XXXIOV_ACCELERATORS_MASK;
}

static u32 get_ae_mask(struct adf_accel_dev *accel_dev)
{
	return ADF_C3XXXIOV_ACCELENGINES_MASK;
}

static u32 get_num_accels(struct adf_hw_device_data *self)
{
	return ADF_C3XXXIOV_MAX_ACCELERATORS;
}

static u32 get_num_aes(struct adf_hw_device_data *self)
{
	return ADF_C3XXXIOV_MAX_ACCELENGINES;
}

static u32 get_misc_bar_id(struct adf_hw_device_data *self)
{
	return ADF_C3XXXIOV_PMISC_BAR;
}

static u32 get_etr_bar_id(struct adf_hw_device_data *self)
{
	return ADF_C3XXXIOV_ETR_BAR;
}

static enum dev_sku_info get_sku(struct adf_hw_device_data *self)
{
	return DEV_SKU_VF;
}

static u32 get_pf2vf_offset(u32 i)
{
	return ADF_C3XXXIOV_PF2VF_OFFSET;
}

static u32 get_vintmsk_offset(u32 i)
{
	return ADF_C3XXXIOV_VINTMSK_OFFSET;
}

static int adf_vf_int_noop(struct adf_accel_dev *accel_dev)
{
	return 0;
}

static void adf_vf_void_noop(struct adf_accel_dev *accel_dev)
{
}

static u32 c3xxxvf_get_hw_cap(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_dev->accel_pci_dev.pci_dev;
	u32 legfuses;
	u32 capabilities;

	/* Read accelerator capabilities mask */
	pci_read_config_dword(pdev, ADF_C3XXX_VFLEGFUSE_OFFSET,
			      &legfuses);
	capabilities =
		ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC +
		ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC +
		ICP_ACCEL_CAPABILITIES_CIPHER +
		ICP_ACCEL_CAPABILITIES_AUTHENTICATION +
		ICP_ACCEL_CAPABILITIES_COMPRESSION +
		ICP_ACCEL_CAPABILITIES_ZUC +
		ICP_ACCEL_CAPABILITIES_SHA3;
	if (legfuses & ICP_ACCEL_MASK_CIPHER_SLICE) {
		capabilities &= ~ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC;
		capabilities &= ~ICP_ACCEL_CAPABILITIES_CIPHER;
	}
	if (legfuses & ICP_ACCEL_MASK_AUTH_SLICE)
		capabilities &= ~ICP_ACCEL_CAPABILITIES_AUTHENTICATION;
	if (legfuses & ICP_ACCEL_MASK_PKE_SLICE)
		capabilities &= ~ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
	if (legfuses & ICP_ACCEL_MASK_COMPRESS_SLICE)
		capabilities &= ~ICP_ACCEL_CAPABILITIES_COMPRESSION;
	if (legfuses & ICP_ACCEL_MASK_EIA3_SLICE)
		capabilities &= ~ICP_ACCEL_CAPABILITIES_ZUC;

	return capabilities;
}

static u32 get_clock_speed(struct adf_hw_device_data *self)
{
	/* CPP clock is half high-speed clock */
	return self->clock_frequency / 2;
}

void adf_init_hw_data_c3xxxiov(struct adf_hw_device_data *hw_data)
{
	hw_data->dev_class = &c3xxxiov_class;
	hw_data->num_banks = ADF_C3XXXIOV_ETR_MAX_BANKS;
	hw_data->num_rings_per_bank = ADF_ETR_MAX_RINGS_PER_BANK;
	hw_data->num_accel = ADF_C3XXXIOV_MAX_ACCELERATORS;
	hw_data->num_logical_accel = 1;
	hw_data->num_engines = ADF_C3XXXIOV_MAX_ACCELENGINES;
	hw_data->tx_rx_gap = ADF_C3XXXIOV_RX_RINGS_OFFSET;
	hw_data->tx_rings_mask = ADF_C3XXXIOV_TX_RINGS_MASK;
	hw_data->alloc_irq = adf_vf_isr_resource_alloc;
	hw_data->free_irq = adf_vf_isr_resource_free;
	hw_data->enable_error_correction = adf_vf_void_noop;
	hw_data->init_admin_comms = adf_vf_int_noop;
	hw_data->exit_admin_comms = adf_vf_void_noop;
	hw_data->send_admin_init = adf_vf2pf_init;
	hw_data->init_arb = adf_vf_int_noop;
	hw_data->exit_arb = adf_vf_void_noop;
	hw_data->disable_iov = adf_vf2pf_shutdown;
	hw_data->get_accel_mask = get_accel_mask;
	hw_data->get_ae_mask = get_ae_mask;
	hw_data->get_num_accels = get_num_accels;
	hw_data->get_num_aes = get_num_aes;
	hw_data->get_etr_bar_id = get_etr_bar_id;
	hw_data->get_misc_bar_id = get_misc_bar_id;
	hw_data->get_pf2vf_offset = get_pf2vf_offset;
	hw_data->get_vintmsk_offset = get_vintmsk_offset;
	hw_data->get_clock_speed = get_clock_speed;
	hw_data->get_sku = get_sku;
	hw_data->enable_ints = adf_vf_void_noop;
	hw_data->enable_vf2pf_comms = adf_enable_vf2pf_comms;
	hw_data->disable_vf2pf_comms = adf_disable_vf2pf_comms;
	hw_data->min_iov_compat_ver = ADF_PFVF_COMPATIBILITY_VERSION;
	hw_data->clock_frequency = ADF_C3XXX_AE_FREQ;
	hw_data->get_accel_cap = c3xxxvf_get_hw_cap;
	hw_data->config_device = adf_config_device;
	hw_data->set_asym_rings_mask = adf_cfg_set_asym_rings_mask;
	hw_data->dev_class->instances++;
	adf_devmgr_update_class_index(hw_data);
}

void adf_clean_hw_data_c3xxxiov(struct adf_hw_device_data *hw_data)
{
	hw_data->dev_class->instances--;
	adf_devmgr_update_class_index(hw_data);
}
