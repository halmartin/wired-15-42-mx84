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
#include <adf_accel_devices.h>
#include <adf_pf2vf_msg.h>
#include <adf_common_drv.h>
#include <adf_dev_err.h>
#include "adf_dh895xcc_hw_data.h"
#include "adf_heartbeat.h"
#include "icp_qat_hw.h"
#include "adf_cfg.h"

/* Worker thread to service arbiter mappings based on dev SKUs */
static const u32 thrd_to_arb_map_sku4[] = {
	0x12222AAA, 0x11666666, 0x12222AAA, 0x11666666,
	0x12222AAA, 0x11222222, 0x12222AAA, 0x11222222,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
};

static const u32 thrd_to_arb_map_sku6[] = {
	0x12222AAA, 0x11666666, 0x12222AAA, 0x11666666,
	0x12222AAA, 0x11222222, 0x12222AAA, 0x11222222,
	0x12222AAA, 0x11222222, 0x12222AAA, 0x11222222
};

static const u32 thrd_to_arb_map_sku3[] = {
	0x00000888, 0x00000000, 0x00000888, 0x00000000,
	0x00000888, 0x00000000, 0x00000888, 0x00000000,
	0x00000888, 0x00000000, 0x00000888, 0x00000000
};

static u32 thrd_to_arb_map_gen[ADF_DH895XCC_MAX_ACCELENGINES] = {0};

static struct adf_hw_device_class dh895xcc_class = {
	.name = ADF_DH895XCC_DEVICE_NAME,
	.type = DEV_DH895XCC,
	.instances = 0
};

static u32 get_accel_mask(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_dev->accel_pci_dev.pci_dev;
	u32 fuse;

	pci_read_config_dword(pdev, ADF_DEVICE_FUSECTL_OFFSET,
			      &fuse);

	return (~fuse) >> ADF_DH895XCC_ACCELERATORS_REG_OFFSET &
		ADF_DH895XCC_ACCELERATORS_MASK;
}

static u32 get_ae_mask(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_dev->accel_pci_dev.pci_dev;
	u32 fuse;

	pci_read_config_dword(pdev, ADF_DEVICE_FUSECTL_OFFSET,
			      &fuse);

	return (~fuse) & ADF_DH895XCC_ACCELENGINES_MASK;
}

static uint32_t get_num_accels(struct adf_hw_device_data *self)
{
	uint32_t i, ctr = 0;

	if (!self || !self->accel_mask)
		return 0;

	for (i = 0; i < ADF_DH895XCC_MAX_ACCELERATORS; i++) {
		if (self->accel_mask & (1 << i))
			ctr++;
	}
	return ctr;
}

static uint32_t get_num_aes(struct adf_hw_device_data *self)
{
	uint32_t i, ctr = 0;

	if (!self || !self->ae_mask)
		return 0;

	for (i = 0; i < ADF_DH895XCC_MAX_ACCELENGINES; i++) {
		if (self->ae_mask & (1 << i))
			ctr++;
	}
	return ctr;
}

static uint32_t get_misc_bar_id(struct adf_hw_device_data *self)
{
	return ADF_DH895XCC_PMISC_BAR;
}

static uint32_t get_etr_bar_id(struct adf_hw_device_data *self)
{
	return ADF_DH895XCC_ETR_BAR;
}

static uint32_t get_sram_bar_id(struct adf_hw_device_data *self)
{
	return ADF_DH895XCC_SRAM_BAR;
}

static enum dev_sku_info get_sku(struct adf_hw_device_data *self)
{
	int sku = (self->fuses & ADF_DH895XCC_FUSECTL_SKU_MASK)
	    >> ADF_DH895XCC_FUSECTL_SKU_SHIFT;

	switch (sku) {
	case ADF_DH895XCC_FUSECTL_SKU_1:
		return DEV_SKU_1;
	case ADF_DH895XCC_FUSECTL_SKU_2:
		return DEV_SKU_2;
	case ADF_DH895XCC_FUSECTL_SKU_3:
		return DEV_SKU_3;
	case ADF_DH895XCC_FUSECTL_SKU_4:
		return DEV_SKU_4;
	default:
		return DEV_SKU_UNKNOWN;
	}
	return DEV_SKU_UNKNOWN;
}

#if defined(CONFIG_PCI_IOV)
static void get_vf2pf_mask(void __iomem *pmisc_bar,
			   u32 vf_mask_sets[ADF_MAX_VF2PF_SET])
{
	/* Get the interrupt sources triggered by VFs */
	vf_mask_sets[0] =
		ADF_DH895XCC_ERRSOU3_VF2PF_L(ADF_CSR_RD(pmisc_bar,
							ADF_ERRSOU3));
	vf_mask_sets[0] |=
		ADF_DH895XCC_ERRSOU5_VF2PF_U(ADF_CSR_RD(pmisc_bar,
							ADF_ERRSOU5));
	vf_mask_sets[1] = 0;
	vf_mask_sets[2] = 0;
	vf_mask_sets[3] = 0;
}

static void enable_vf2pf_interrupts(void __iomem *pmisc_addr,
				    u32 vf_mask, u8 vf2pf_set)
{
	if (vf2pf_set)
		return;

	/* Enable VF2PF Messaging Ints - VFs 1 through 16 per vf_mask[15:0] */
	if (vf_mask & 0xFFFF)
		adf_csr_fetch_and_and(pmisc_addr,
				      ADF_ERRMSK3,
				      ~ADF_DH895XCC_ERRMSK3_VF2PF_L(vf_mask));

	/* Enable VF2PF Messaging Ints - VFs 17 through 32 per vf_mask[31:16] */
	if (vf_mask >> 16) {
		adf_csr_fetch_and_and(pmisc_addr,
				      ADF_ERRMSK5,
				      ~ADF_DH895XCC_ERRMSK5_VF2PF_U(vf_mask));
	}
}

static void disable_vf2pf_interrupts(void __iomem *pmisc_addr,
				     u32 vf_mask, u8 vf2pf_set)
{
	if (vf2pf_set)
		return;

	/* Disable VF2PF interrupts for VFs 1 through 16 per vf_mask[15:0] */
	if (vf_mask & 0xFFFF)
		adf_csr_fetch_and_or(pmisc_addr,
				     ADF_ERRMSK3,
				     ADF_DH895XCC_ERRMSK3_VF2PF_L(vf_mask));

	/* Disable VF2PF interrupts for VFs 17 through 32 per vf_mask[31:16] */
	if (vf_mask >> 16)
		adf_csr_fetch_and_or(pmisc_addr,
				     ADF_ERRMSK5,
				     ADF_DH895XCC_ERRMSK5_VF2PF_U(vf_mask));
}
#endif /* CONFIG_PCI_IOV */

static void adf_get_arbiter_mapping(struct adf_accel_dev *accel_dev,
				    u32 const **arb_map_config)
{
	switch (accel_dev->accel_pci_dev.sku) {
	case DEV_SKU_1:
		adf_cfg_gen_dispatch_arbiter(accel_dev,
					     thrd_to_arb_map_sku4,
					     thrd_to_arb_map_gen,
					     ADF_DH895XCC_MAX_ACCELENGINES);
		*arb_map_config = thrd_to_arb_map_gen;
		break;

	case DEV_SKU_2:
	case DEV_SKU_4:
		adf_cfg_gen_dispatch_arbiter(accel_dev,
					     thrd_to_arb_map_sku6,
					     thrd_to_arb_map_gen,
					     ADF_DH895XCC_MAX_ACCELENGINES);
		*arb_map_config = thrd_to_arb_map_gen;
		break;

	case DEV_SKU_3:
		adf_cfg_gen_dispatch_arbiter(accel_dev,
					     thrd_to_arb_map_sku3,
					     thrd_to_arb_map_gen,
					     ADF_DH895XCC_MAX_ACCELENGINES);
		*arb_map_config = thrd_to_arb_map_gen;

		break;

	default:
		dev_err(&GET_DEV(accel_dev),
			"The configuration doesn't match any SKU");
		*arb_map_config = NULL;
	}
}

static uint32_t get_pf2vf_offset(uint32_t i)
{
	return ADF_DH895XCC_PF2VF_OFFSET(i);
}

static uint32_t get_vintmsk_offset(uint32_t i)
{
	return ADF_DH895XCC_VINTMSK_OFFSET(i);
}

static u32 get_clock_speed(struct adf_hw_device_data *self)
{
	/* CPP clock is half high-speed clock */
	return self->clock_frequency / 2;
}

static void adf_enable_error_interrupts(void __iomem *csr)
{
	ADF_CSR_WR(csr, ADF_ERRMSK0, ADF_DH895XCC_ERRMSK0_CERR); /* ME0-ME3  */
	ADF_CSR_WR(csr, ADF_ERRMSK1, ADF_DH895XCC_ERRMSK1_CERR); /* ME4-ME7  */
	ADF_CSR_WR(csr, ADF_ERRMSK4, ADF_DH895XCC_ERRMSK4_CERR); /* ME8-ME11 */

	/* Reset everything except VFtoPF1_16 */
	adf_csr_fetch_and_and(csr, ADF_ERRMSK3, ADF_DH895XCC_VF2PF1_16);

	/* Disable Secure RAM correctable error interrupt */
	adf_csr_fetch_and_or(csr, ADF_ERRMSK3, ADF_DH895XCC_ERRMSK3_CERR);

	/* Reset everything except VFtoPF17_32 */
	adf_csr_fetch_and_and(csr, ADF_ERRMSK5, ADF_DH895XCC_VF2PF17_32);

	/* RI CPP bus interface error detection and reporting. */
	ADF_CSR_WR(csr, ADF_DH895XCC_RICPPINTCTL, ADF_DH895XCC_RICPP_EN);

	/* TI CPP bus interface error detection and reporting. */
	ADF_CSR_WR(csr, ADF_DH895XCC_TICPPINTCTL, ADF_DH895XCC_TICPP_EN);

	/* Enable CFC Error interrupts and logging */
	ADF_CSR_WR(csr, ADF_DH895XCC_CPP_SHAC_ERR_CTRL,
		   ADF_DH895XCC_CPP_SHAC_UE);

	/* Enable SecureRAM to fix and log Correctable errors */
	ADF_CSR_WR(csr, ADF_DH895XCC_ESRAMCERR, ADF_DH895XCC_ESRAM_CERR);

	/* Enable SecureRAM Uncorrectable error interrupts and logging */
	ADF_CSR_WR(csr, ADF_DH895XCC_ESRAMUERR, ADF_DH895XCC_ESRAM_UERR);

	/* Enable Push/Pull Misc Uncorrectable error interrupts and logging */
	ADF_CSR_WR(csr, ADF_CPPMEMTGTERR, ADF_DH895XCC_TGT_UERR);
}

static void adf_disable_error_interrupts(struct adf_accel_dev *accel_dev)
{
	struct adf_bar *misc_bar = &GET_BARS(accel_dev)[ADF_DH895XCC_PMISC_BAR];
	void __iomem *csr = misc_bar->virt_addr;

	/* ME0-ME3 */
	ADF_CSR_WR(csr, ADF_ERRMSK0, ADF_DH895XCC_ERRMSK0_UERR |
		   ADF_DH895XCC_ERRMSK0_CERR);
	/* ME4-ME7 */
	ADF_CSR_WR(csr, ADF_ERRMSK1, ADF_DH895XCC_ERRMSK1_UERR |
		   ADF_DH895XCC_ERRMSK1_CERR);
	/* CPP Push Pull, RI, TI, CPM0-CPM1, CFC */
	ADF_CSR_WR(csr, ADF_ERRMSK3, ADF_DH895XCC_ERRMSK3_UERR |
		   ADF_DH895XCC_ERRMSK3_CERR);
	/* ME8-ME11 */
	ADF_CSR_WR(csr, ADF_ERRMSK4, ADF_DH895XCC_ERRMSK4_UERR |
		   ADF_DH895XCC_ERRMSK4_CERR);
	/* CPM2-CPM5 */
	ADF_CSR_WR(csr, ADF_ERRMSK5, ADF_DH895XCC_ERRMSK5_UERR |
		   ADF_DH895XCC_ERRMSK5_CERR);
}

static int adf_check_uncorrectable_error(struct adf_accel_dev *accel_dev)
{
	struct adf_bar *misc_bar = &GET_BARS(accel_dev)[ADF_DH895XCC_PMISC_BAR];
	void __iomem *csr = misc_bar->virt_addr;

	u32 errsou0 = ADF_CSR_RD(csr, ADF_ERRSOU0) & ADF_DH895XCC_ERRMSK0_UERR;
	u32 errsou1 = ADF_CSR_RD(csr, ADF_ERRSOU1) & ADF_DH895XCC_ERRMSK1_UERR;
	u32 errsou3 = ADF_CSR_RD(csr, ADF_ERRSOU3) & ADF_DH895XCC_ERRMSK3_UERR;
	u32 errsou4 = ADF_CSR_RD(csr, ADF_ERRSOU4) & ADF_DH895XCC_ERRMSK4_UERR;
	u32 errsou5 = ADF_CSR_RD(csr, ADF_ERRSOU5) & ADF_DH895XCC_ERRMSK5_UERR;

	return (errsou0 | errsou1 | errsou3 | errsou4 | errsou5);
}

static void adf_enable_mmps(void __iomem *csr,
			    unsigned int dev)
{
	unsigned int mmp;

	for (mmp = 0; mmp < ADF_MAX_MMP; ++mmp) {
		/*
		 * The device supports PKE,
		 * so enable error reporting from MMP memory
		 */
		adf_csr_fetch_and_or(csr, ADF_UERRSSMMMP(dev, mmp),
				     ADF_DH895XCC_UERRSSMMMP_EN);
		/*
		 * The device supports PKE,
		 * so enable error correction from MMP memory
		 */
		adf_csr_fetch_and_or(csr, ADF_CERRSSMMMP(dev, mmp),
				     ADF_DH895XCC_CERRSSMMMP_EN);
	}
}

static void adf_disable_mmps(void __iomem *csr,
			     unsigned int dev)
{
	unsigned int mmp;

	for (mmp = 0; mmp < ADF_MAX_MMP; ++mmp) {
		/*
		 * The device doesn't support PKE,
		 * so disable error reporting from MMP memory
		 */
		adf_csr_fetch_and_and(csr, ADF_UERRSSMMMP(dev, mmp),
				      ~ADF_DH895XCC_UERRSSMMMP_EN);
		/*
		 * The device doesn't support PKE,
		 * so disable error correction from MMP memory
		 */
		adf_csr_fetch_and_and(csr, ADF_CERRSSMMMP(dev, mmp),
				      ~ADF_DH895XCC_CERRSSMMMP_EN);
	}			}

static void adf_enable_mmp_error_correction(void __iomem *csr,
					    struct adf_hw_device_data *hw_data)
{
	unsigned int dev;
	unsigned int mask;

	/* Enable MMP Logging */
	for (dev = 0, mask = hw_data->accel_mask; mask; dev++, mask >>= 1) {
		if (!(mask & 1))
			continue;
		/* Set power-up */
		adf_csr_fetch_and_and(csr, ADF_DH895XCC_SLICEPWRDOWN(dev),
				      ~ADF_DH895XCC_MMP_PWR_UP_MSK);

		if (hw_data->accel_capabilities_mask &
		    ADF_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC)
			adf_enable_mmps(csr, dev);
		else
			adf_disable_mmps(csr, dev);

		/* Restore power-down value */
		adf_csr_fetch_and_or(csr, ADF_DH895XCC_SLICEPWRDOWN(dev),
				     ADF_DH895XCC_MMP_PWR_UP_MSK);

		/* Disabling correctable error interrupts. */
		ADF_CSR_WR(csr, ADF_DH895XCC_INTMASKSSM(dev),
			   ADF_DH895XCC_INTMASKSSM_UERR);
	}
}
static void adf_enable_error_correction(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_device = accel_dev->hw_device;
	struct adf_bar *misc_bar = &GET_BARS(accel_dev)[ADF_DH895XCC_PMISC_BAR];
	void __iomem *csr = misc_bar->virt_addr;
	unsigned int i;
	unsigned int mask;

	/* Enable Accel Engine error detection & correction */
	for (i = 0, mask = hw_device->ae_mask; mask; i++, mask >>= 1) {
		if (!(mask & 1))
			continue;
		adf_csr_fetch_and_or(csr, ADF_DH895XCC_AE_CTX_ENABLES(i),
				     ADF_DH895XCC_ENABLE_AE_ECC_ERR);
		adf_csr_fetch_and_or(csr, ADF_DH895XCC_AE_MISC_CONTROL(i),
				     ADF_DH895XCC_ENABLE_AE_ECC_PARITY_CORR);
	}

	/* Enable shared memory error detection & correction */
	for (i = 0, mask = hw_device->accel_mask; mask; i++, mask >>= 1) {
		if (!(mask & 1))
			continue;
		adf_csr_fetch_and_or(csr, ADF_UERRSSMSH(i),
				     ADF_DH895XCC_ERRSSMSH_EN);
		adf_csr_fetch_and_or(csr, ADF_CERRSSMSH(i),
				     ADF_DH895XCC_ERRSSMSH_EN);
		adf_csr_fetch_and_or(csr, ADF_PPERR(i),
				     ADF_DH895XCC_PPERR_EN);
	}

	adf_enable_error_interrupts(csr);
	adf_enable_mmp_error_correction(csr, hw_device);
}

static void adf_enable_ints(struct adf_accel_dev *accel_dev)
{
	void __iomem *addr;

	addr = (&GET_BARS(accel_dev)[ADF_DH895XCC_PMISC_BAR])->virt_addr;

	/* Enable bundle and misc interrupts */
	ADF_CSR_WR(addr, ADF_DH895XCC_SMIAPF0_MASK_OFFSET,
		   accel_dev->pf.vf_info ? 0 :
			GENMASK_ULL(GET_MAX_BANKS(accel_dev) - 1, 0));
	ADF_CSR_WR(addr, ADF_DH895XCC_SMIAPF1_MASK_OFFSET,
		   ADF_DH895XCC_SMIA1_MASK);
}

static u32 get_ae_clock(struct adf_hw_device_data *self)
{
	/*
	 * Clock update interval is <16> ticks for dh895xcc.
	 */
	return self->clock_frequency / 16;
}

static u32 dh895xcc_get_hw_cap(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_dev->accel_pci_dev.pci_dev;
	u32 legfuses;
	u32 capabilities;

	/* Read accelerator capabilities mask */
	pci_read_config_dword(pdev, ADF_DEVICE_LEGFUSE_OFFSET,
			      &legfuses);
	capabilities =
		ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC +
		ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC +
		ICP_ACCEL_CAPABILITIES_CIPHER +
		ICP_ACCEL_CAPABILITIES_AUTHENTICATION +
		ICP_ACCEL_CAPABILITIES_COMPRESSION +
		ICP_ACCEL_CAPABILITIES_LZS_COMPRESSION;

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
	if (legfuses & ICP_ACCEL_MASK_LZS_SLICE)
		capabilities &= ~ICP_ACCEL_CAPABILITIES_LZS_COMPRESSION;

	return capabilities;
}

void adf_init_hw_data_dh895xcc(struct adf_hw_device_data *hw_data)
{
	hw_data->dev_class = &dh895xcc_class;
	hw_data->instance_id = dh895xcc_class.instances++;
	hw_data->num_banks = ADF_DH895XCC_ETR_MAX_BANKS;
	hw_data->num_rings_per_bank = ADF_ETR_MAX_RINGS_PER_BANK;
	hw_data->num_accel = ADF_DH895XCC_MAX_ACCELERATORS;
	hw_data->num_logical_accel = 1;
	hw_data->num_engines = ADF_DH895XCC_MAX_ACCELENGINES;
	hw_data->tx_rx_gap = ADF_DH895XCC_RX_RINGS_OFFSET;
	hw_data->tx_rings_mask = ADF_DH895XCC_TX_RINGS_MASK;
	hw_data->alloc_irq = adf_isr_resource_alloc;
	hw_data->free_irq = adf_isr_resource_free;
	hw_data->enable_error_correction = adf_enable_error_correction;
	hw_data->check_uncorrectable_error = adf_check_uncorrectable_error;
	hw_data->print_err_registers = adf_print_err_registers;
	hw_data->disable_error_interrupts = adf_disable_error_interrupts;
	hw_data->get_accel_mask = get_accel_mask;
	hw_data->get_ae_mask = get_ae_mask;
	hw_data->get_num_accels = get_num_accels;
	hw_data->get_num_aes = get_num_aes;
	hw_data->get_etr_bar_id = get_etr_bar_id;
	hw_data->get_misc_bar_id = get_misc_bar_id;
	hw_data->get_pf2vf_offset = get_pf2vf_offset;
	hw_data->get_vintmsk_offset = get_vintmsk_offset;
	hw_data->get_clock_speed = get_clock_speed;
	hw_data->get_sram_bar_id = get_sram_bar_id;
	hw_data->get_sku = get_sku;
#if defined(CONFIG_PCI_IOV)
	hw_data->get_vf2pf_mask = get_vf2pf_mask;
	hw_data->enable_vf2pf_interrupts = enable_vf2pf_interrupts;
	hw_data->disable_vf2pf_interrupts = disable_vf2pf_interrupts;
#endif
	hw_data->fw_name = ADF_DH895XCC_FW;
	hw_data->fw_mmp_name = ADF_DH895XCC_MMP;
	hw_data->init_admin_comms = adf_init_admin_comms;
	hw_data->exit_admin_comms = adf_exit_admin_comms;
	hw_data->configure_iov_threads = adf_configure_iov_threads;
	hw_data->disable_iov = adf_disable_sriov;
	hw_data->send_admin_init = adf_send_admin_init;
	hw_data->init_arb = adf_init_arb;
	hw_data->exit_arb = adf_exit_arb;
	hw_data->disable_arb = adf_disable_arb;
	hw_data->get_arb_mapping = adf_get_arbiter_mapping;
	hw_data->enable_ints = adf_enable_ints;
	hw_data->enable_vf2pf_comms = adf_pf_enable_vf2pf_comms;
	hw_data->disable_vf2pf_comms = adf_pf_disable_vf2pf_comms;
	hw_data->reset_device = adf_reset_sbr;
	hw_data->min_iov_compat_ver = ADF_PFVF_COMPATIBILITY_VERSION;
	hw_data->get_heartbeat_status = adf_get_heartbeat_status;
	hw_data->get_ae_clock = get_ae_clock;
	hw_data->clock_frequency = ADF_DH895XCC_AE_FREQ;
	hw_data->get_accel_cap = dh895xcc_get_hw_cap;
	hw_data->quiesce_device = adf_dev_quiesce;
	hw_data->extended_dc_capabilities = 0;
	hw_data->config_device = adf_config_device;
	hw_data->set_asym_rings_mask = adf_cfg_set_asym_rings_mask;
}

void adf_clean_hw_data_dh895xcc(struct adf_hw_device_data *hw_data)
{
	hw_data->dev_class->instances--;
}
