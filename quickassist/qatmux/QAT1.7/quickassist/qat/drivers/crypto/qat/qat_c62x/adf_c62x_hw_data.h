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
#ifndef ADF_C62X_HW_DATA_H_
#define ADF_C62X_HW_DATA_H_

/* PCIe configuration space */
#define ADF_C62X_SRAM_BAR 0
#define ADF_C62X_PMISC_BAR 1
#define ADF_C62X_ETR_BAR 2
#define ADF_C62X_RX_RINGS_OFFSET 8
#define ADF_C62X_TX_RINGS_MASK 0xFF
#define ADF_C62X_MAX_ACCELERATORS 5
#define ADF_C62X_MAX_ACCELENGINES 10
#define ADF_C62X_ACCELERATORS_REG_OFFSET 16
#define ADF_C62X_ACCELERATORS_MASK 0x1F
#define ADF_C62X_ACCELENGINES_MASK 0x3FF
#define ADF_C62X_ETR_MAX_BANKS 16
#define ADF_C62X_SMIAPF0_MASK_OFFSET (0x3A000 + 0x28)
#define ADF_C62X_SMIAPF1_MASK_OFFSET (0x3A000 + 0x30)
#define ADF_C62X_SMIA0_MASK 0xFFFF
#define ADF_C62X_SMIA1_MASK 0x1
#define ADF_C62X_SOFTSTRAP_CSR_OFFSET 0x2EC
#define ADF_C62X_POWERGATE_PKE     BIT(24)
#define ADF_C62X_POWERGATE_DC      BIT(23)

/* Error detection and correction */
#define ADF_C62X_AE_CTX_ENABLES(i) (i * 0x1000 + 0x20818)
#define ADF_C62X_AE_MISC_CONTROL(i) (i * 0x1000 + 0x20960)
#define ADF_C62X_ENABLE_AE_ECC_ERR BIT(28)
#define ADF_C62X_ENABLE_AE_ECC_PARITY_CORR (BIT(24) | BIT(12))
#define ADF_C62X_ERRSSMSH_EN (BIT(3))
/* BIT(2) enables the logging of push/pull data errors. */
#define ADF_C62X_PPERR_EN		(BIT(2))

/* Mask for VF2PF interrupts */
#define ADF_C62X_VF2PF1_16              (0xFFFF << 9)
#define ADF_C62X_ERRSOU3_VF2PF(errsou3) (((errsou3) & 0x01FFFE00) >> 9)
#define ADF_C62X_ERRMSK3_VF2PF(vf_mask) (((vf_mask) & 0xFFFF) << 9)

/* Masks for correctable error interrupts. */
#define ADF_C62X_ERRMSK0_CERR		(BIT(24) | BIT(16) | BIT(8) | BIT(0))
#define ADF_C62X_ERRMSK1_CERR		(BIT(24) | BIT(16) | BIT(8) | BIT(0))
#define ADF_C62X_ERRMSK3_CERR		(BIT(7))
#define ADF_C62X_ERRMSK4_CERR		(BIT(8) | BIT(0))
#define ADF_C62X_ERRMSK5_CERR		(0)

/* Masks for uncorrectable error interrupts. */
#define ADF_C62X_ERRMSK0_UERR		(BIT(25) | BIT(17) | BIT(9) | BIT(1))
#define ADF_C62X_ERRMSK1_UERR		(BIT(25) | BIT(17) | BIT(9) | BIT(1))
#define ADF_C62X_ERRMSK3_UERR		(BIT(8) | BIT(6) | BIT(5) | BIT(4) | \
					 BIT(3) | BIT(2) | BIT(0))
#define ADF_C62X_ERRMSK4_UERR		(BIT(9) | BIT(1))
#define ADF_C62X_ERRMSK5_UERR		(BIT(18) | BIT(17) | BIT(16))

/* RI CPP control */
#define ADF_C62X_RICPPINTCTL		(0x3A000 + 0x110)
/*
 * BIT(2) enables error detection and reporting on the RI Parity Error.
 * BIT(1) enables error detection and reporting on the RI CPP Pull interface.
 * BIT(0) enables error detection and reporting on the RI CPP Push interface.
 */
#define ADF_C62X_RICPP_EN		\
	(BIT(2) | BIT(1) | BIT(0))

/* TI CPP control */
#define ADF_C62X_TICPPINTCTL		(0x3A400 + 0x138)
/*
 * BIT(4) enables parity error detection and reporting on the Secure RAM.
 * BIT(3) enables error detection and reporting on the ETR Parity Error.
 * BIT(2) enables error detection and reporting on the TI Parity Error.
 * BIT(1) enables error detection and reporting on the TI CPP Pull interface.
 * BIT(0) enables error detection and reporting on the TI CPP Push interface.
 */
#define ADF_C62X_TICPP_EN		\
	(BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))

/* CFC Uncorrectable Errors */
#define ADF_C62X_CPP_CFC_ERR_CTRL	(0x30000 + 0xC00)
/*
 * BIT(1) enables interrupt.
 * BIT(0) enables detecting and logging of push/pull data errors.
 */
#define ADF_C62X_CPP_CFC_UE		(BIT(1) | BIT(0))

/* Correctable SecureRAM Error Reg */
#define ADF_C62X_SECRAMCERR		(0x3AC00 + 0x00)
/* BIT(3) enables fixing and logging of correctable errors. */
#define ADF_C62X_SECRAM_CERR		(BIT(3))

/* Uncorrectable SecureRAM Error Reg */
/*
 * BIT(17) enables interrupt.
 * BIT(3) enables detecting and logging of uncorrectable errors.
 */
#define ADF_C62X_SECRAM_UERR		(BIT(17) | BIT(3))

/* Miscellaneous Memory Target Errors Register */
/*
 * BIT(3) enables detecting and logging push/pull data errors.
 * BIT(2) enables interrupt.
 */
#define ADF_C62X_TGT_UERR		(BIT(3) | BIT(2))

#define ADF_C62X_SLICEPWRDOWN(i)	((i) * 0x4000 + 0x2C)
/* Enabling PKE4-PKE0. */
#define ADF_C62X_MMP_PWR_UP_MSK		\
	(BIT(20) | BIT(19) | BIT(18) | BIT(17) | BIT(16))

/* CPM Uncorrectable Errors */
#define ADF_C62X_INTMASKSSM(i)		((i) * 0x4000 + 0x0)
/* Disabling interrupts for correctable errors. */
#define ADF_C62X_INTMASKSSM_UERR	\
	(BIT(11) | BIT(9) | BIT(7) | BIT(5) | BIT(3) | BIT(1))

/* MMP */
/* BIT(3) enables correction. */
#define ADF_C62X_CERRSSMMMP_EN		(BIT(3))

/* BIT(3) enables logging. */
#define ADF_C62X_UERRSSMMMP_EN		(BIT(3))

#define ADF_C62X_PF2VF_OFFSET(i)	(0x3A000 + 0x280 + ((i) * 0x04))
#define ADF_C62X_VINTMSK_OFFSET(i)	(0x3A000 + 0x200 + ((i) * 0x04))

/* Firmware Binary */
#define ADF_C62X_FW "qat_c62x.bin"
#define ADF_C62X_MMP "qat_c62x_mmp.bin"

void adf_init_hw_data_c62x(struct adf_hw_device_data *hw_data);
void adf_clean_hw_data_c62x(struct adf_hw_device_data *hw_data);

#define ADF_C62X_AE_FREQ		(685 * 1000000)
#define ADF_C62X_MIN_AE_FREQ (533 * 1000000)
#define ADF_C62X_MAX_AE_FREQ (800 * 1000000)

#endif
