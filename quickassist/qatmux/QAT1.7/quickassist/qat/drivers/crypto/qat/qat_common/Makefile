########################################################################
#  This file is provided under a dual BSD/GPLv2 license.  When using or
#  redistributing this file, you may do so under either license.
#
#  GPL LICENSE SUMMARY
#  Copyright(c) 2014 Intel Corporation.
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of version 2 of the GNU General Public License as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  Contact Information:
#  qat-linux@intel.com
#
#  BSD LICENSE
#  Copyright(c) 2014 Intel Corporation.
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#    * Neither the name of Intel Corporation nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
########################################################################

obj-$(CONFIG_CRYPTO_DEV_QAT) += intel_qat.o
intel_qat-objs := adf_cfg.o \
	adf_isr.o \
	adf_ctl_drv.o \
	adf_dev_mgr.o \
	adf_dev_err.o \
	adf_init.o \
	adf_accel_engine.o \
	adf_aer.o \
	adf_transport.o \
	adf_admin.o \
	adf_hw_arbiter.o \
	qat_uclo.o \
	qat_hal.o \
	adf_cfg_dev_dbg.o \
	adf_heartbeat.o \
	adf_heartbeat_dbg.o \
	adf_ver_dbg.o \
	adf_clock.o \
	adf_pf2vf_dbg.o \
	adf_fw_counters.o \
	adf_cnvnr_freq_counters.o

ifeq ($(QAT_NO_LKCF), y)
intel_qat-objs += qat_crypto_stubs.o
else
intel_qat-objs += qat_crypto.o \
	qat_algs.o \
	qat_asym_algs.o
endif

intel_qat-$(CONFIG_DEBUG_FS) += adf_transport_debug.o
intel_qat-$(CONFIG_PCI_IOV) += adf_sriov.o adf_pf2vf_msg.o \
			       adf_vf2pf_msg.o adf_vf_isr.o \
			       adf_pf2vf_capabilities.o
