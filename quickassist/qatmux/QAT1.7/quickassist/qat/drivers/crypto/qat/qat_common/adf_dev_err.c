/*
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 *  redistributing this file, you may do so under either license.
 *
 *  GPL LICENSE SUMMARY
 *  Copyright(c) 2017 Intel Corporation.
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Contact Information:
 *
 *  qat-linux@intel.com
 *
 *  BSD LICENSE
 *  Copyright(c) 2017 Intel Corporation.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "adf_dev_err.h"

struct reg_info {
	size_t	offs;
	char	*name;
};

static struct reg_info adf_err_regs[] = {
	{ADF_ERRSOU0, "ERRSOU0"},
	{ADF_ERRSOU1, "ERRSOU1"},
	{ADF_ERRSOU3, "ERRSOU3"},
	{ADF_ERRSOU4, "ERRSOU4"},
	{ADF_ERRSOU5, "ERRSOU5"},
	{ADF_RICPPINTSTS, "RICPPINTSTS"},
	{ADF_RIERRPUSHID, "RIERRPUSHID"},
	{ADF_RIERRPULLID, "RIERRPULLID"},
	{ADF_CPP_CFC_ERR_STATUS, "CPP_CFC_ERR_STATUS"},
	{ADF_CPP_CFC_ERR_PPID, "CPP_CFC_ERR_PPID"},
	{ADF_TICPPINTSTS, "TICPPINTSTS"},
	{ADF_TIERRPUSHID, "TIERRPUSHID"},
	{ADF_TIERRPULLID, "TIERRPULLID"},
	{ADF_SECRAMUERR, "SECRAMUERR"},
	{ADF_SECRAMUERRAD, "SECRAMUERRAD"},
	{ADF_CPPMEMTGTERR, "CPPMEMTGTERR"},
	{ADF_ERRPPID, "ERRPPID"},
};

static u32 adf_get_intstatsssm(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_INTSTATSSM(dev));
}

static u32 adf_get_pperr(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_PPERR(dev));
}

static u32 adf_get_pperrid(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_PPERRID(dev));
}

static u32 adf_get_uerrssmsh(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMSH(dev));
}

static u32 adf_get_uerrssmshad(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMSHAD(dev));
}

static u32 adf_get_uerrssmmmp0(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMP(dev, 0));
}

static u32 adf_get_uerrssmmmp1(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMP(dev, 1));
}

static u32 adf_get_uerrssmmmp2(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMP(dev, 2));
}

static u32 adf_get_uerrssmmmp3(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMP(dev, 3));
}

static u32 adf_get_uerrssmmmp4(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMP(dev, 4));
}

static u32 adf_get_uerrssmmmpad0(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMPAD(dev, 0));
}

static u32 adf_get_uerrssmmmpad1(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMPAD(dev, 1));
}

static u32 adf_get_uerrssmmmpad2(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMPAD(dev, 2));
}

static u32 adf_get_uerrssmmmpad3(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMPAD(dev, 3));
}

static u32 adf_get_uerrssmmmpad4(void __iomem *pmisc_bar_addr, size_t dev)
{
	return ADF_CSR_RD(pmisc_bar_addr, ADF_UERRSSMMMPAD(dev, 4));
}

struct reg_array_info {
	u32	(*read)(void __iomem *, size_t);
	char	*name;
};

static struct reg_array_info adf_accel_err_regs[] = {
	{adf_get_intstatsssm, "INTSTATSSM"},
	{adf_get_pperr, "PPERR"},
	{adf_get_pperrid, "PPERRID"},
	{adf_get_uerrssmsh, "UERRSSMSH"},
	{adf_get_uerrssmshad, "UERRSSMSHAD"},
	{adf_get_uerrssmmmp0, "UERRSSMMMP0"},
	{adf_get_uerrssmmmp1, "UERRSSMMMP1"},
	{adf_get_uerrssmmmp2, "UERRSSMMMP2"},
	{adf_get_uerrssmmmp3, "UERRSSMMMP3"},
	{adf_get_uerrssmmmp4, "UERRSSMMMP4"},
	{adf_get_uerrssmmmpad0, "UERRSSMMMPAD0"},
	{adf_get_uerrssmmmpad1, "UERRSSMMMPAD1"},
	{adf_get_uerrssmmmpad2, "UERRSSMMMPAD2"},
	{adf_get_uerrssmmmpad3, "UERRSSMMMPAD3"},
	{adf_get_uerrssmmmpad4, "UERRSSMMMPAD4"},
};

static char adf_printf_buf[128] = {0};
static size_t adf_printf_len;

static void adf_print_flush(struct adf_accel_dev *accel_dev)
{
	if (adf_printf_len > 0) {
		dev_err(&GET_DEV(accel_dev), "%.128s\n", adf_printf_buf);
		adf_printf_len = 0;
	}
}

static void adf_print_reg(struct adf_accel_dev *accel_dev,
			  const char *name, size_t idx, u32 val)
{
	adf_printf_len += snprintf(&adf_printf_buf[adf_printf_len],
			sizeof(adf_printf_buf) - adf_printf_len,
			"%s[%zu],%.8x,", name, idx, val);

	if (adf_printf_len >= 80)
		adf_print_flush(accel_dev);
}

void adf_print_err_registers(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct adf_bar *misc_bar =
		&GET_BARS(accel_dev)[hw_data->get_misc_bar_id(hw_data)];
	void __iomem *csr = misc_bar->virt_addr;
	size_t i;
	unsigned int mask;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(adf_err_regs); ++i) {
		val = ADF_CSR_RD(csr, adf_err_regs[i].offs);

		adf_print_reg(accel_dev, adf_err_regs[i].name, 0, val);
	}

	for (i = 0; i < ARRAY_SIZE(adf_accel_err_regs); ++i) {
		size_t accel;

		for (accel = 0, mask = hw_data->accel_mask; mask;
				accel++, mask >>= 1) {
			if (!(mask & 1))
				continue;
			val = adf_accel_err_regs[i].read(csr, accel);

			adf_print_reg(accel_dev, adf_accel_err_regs[i].name,
				      accel, val);
		}
	}

	adf_print_flush(accel_dev);
}
EXPORT_SYMBOL_GPL(adf_print_err_registers);

static void adf_log_slice_hang(struct adf_accel_dev *accel_dev,
			       u8 accel_num, char *unit_name, u8 unit_number)
{
	dev_err(&GET_DEV(accel_dev),
		"CPM #%x Slice Hang Detected unit: %s%d.\n",
		accel_num, unit_name, unit_number);
}

static bool adf_handle_slice_hang(struct adf_accel_dev *accel_dev,
				  u8 accel_num, void __iomem *csr)
{
	u32 slice_hang = ADF_CSR_RD(csr, ADF_SLICEHANGSTATUS(accel_num));

	if (!slice_hang)
		return false;

	if (slice_hang & ADF_SLICE_HANG_AUTH0_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "Auth", 0);
	if (slice_hang & ADF_SLICE_HANG_AUTH1_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "Auth", 1);
	if (slice_hang & ADF_SLICE_HANG_CPHR0_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "Cipher", 0);
	if (slice_hang & ADF_SLICE_HANG_CPHR1_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "Cipher", 1);
	if (slice_hang & ADF_SLICE_HANG_CMP0_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "Comp", 0);
	if (slice_hang & ADF_SLICE_HANG_CMP1_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "Comp", 1);
	if (slice_hang & ADF_SLICE_HANG_XLT0_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "Xlator", 0);
	if (slice_hang & ADF_SLICE_HANG_XLT1_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "Xlator", 1);
	if (slice_hang & ADF_SLICE_HANG_MMP0_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "MMP", 0);
	if (slice_hang & ADF_SLICE_HANG_MMP1_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "MMP", 1);
	if (slice_hang & ADF_SLICE_HANG_MMP2_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "MMP", 2);
	if (slice_hang & ADF_SLICE_HANG_MMP3_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "MMP", 3);
	if (slice_hang & ADF_SLICE_HANG_MMP4_MASK)
		adf_log_slice_hang(accel_dev, accel_num, "MMP", 4);

	/* Clear the associated interrupt - write 1 to clear */
	ADF_CSR_WR(csr, ADF_SLICEHANGSTATUS(accel_num), slice_hang);

	return true;
}

/**
 * adf_check_slice_hang() - Check slice hang status
 *
 * Return: true if a slice hange interrupt is serviced..
 */
bool adf_check_slice_hang(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct adf_bar *misc_bar =
		&GET_BARS(accel_dev)[hw_data->get_misc_bar_id(hw_data)];
	void __iomem *csr = misc_bar->virt_addr;
	u32 errsou3 = ADF_CSR_RD(csr, ADF_ERRSOU3);
	u32 errsou5 = ADF_CSR_RD(csr, ADF_ERRSOU5);
	u32 accel_num;
	bool handled = false;
	u32 errsou[] = {errsou3, errsou3, errsou5, errsou5, errsou5};
	u32 mask[] = {ADF_EMSK3_CPM0_MASK,
		      ADF_EMSK3_CPM1_MASK,
		      ADF_EMSK5_CPM2_MASK,
		      ADF_EMSK5_CPM3_MASK,
		      ADF_EMSK5_CPM4_MASK};
	unsigned int accel_mask;

	for (accel_num = 0, accel_mask = hw_data->accel_mask; accel_mask;
			accel_num++, accel_mask >>= 1) {
		if (!(accel_mask & 1))
			continue;
		if (accel_num >= ARRAY_SIZE(errsou)) {
			dev_err(&GET_DEV(accel_dev),
				"Invalid accel_num %d.\n", accel_num);
			break;
		}

		if (errsou[accel_num] & mask[accel_num]) {
			if (ADF_CSR_RD(csr, ADF_INTSTATSSM(accel_num)) &
				       ADF_INTSTATSSM_SHANGERR)
				handled |= adf_handle_slice_hang(accel_dev,
								 accel_num,
								 csr);
		}
	}

	return handled;
}
EXPORT_SYMBOL_GPL(adf_check_slice_hang);
