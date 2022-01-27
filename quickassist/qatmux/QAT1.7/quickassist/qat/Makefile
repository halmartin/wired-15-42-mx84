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
export CONFIG_CRYPTO_DEV_QAT=m
export CONFIG_CRYPTO_DEV_QAT_DH895xCC=m
export CONFIG_CRYPTO_DEV_QAT_DH895xCCVF=m
export CONFIG_CRYPTO_DEV_QAT_C3XXX=m
export CONFIG_CRYPTO_DEV_QAT_C62X=m
export CONFIG_CRYPTO_DEV_QAT_C3XXXVF=m
export CONFIG_CRYPTO_DEV_QAT_C62XVF=m
export CONFIG_CRYPTO_DEV_QAT_D15XX=m
export CONFIG_CRYPTO_DEV_QAT_D15XXVF=m
export QAT_UIO?=y
export ICP_HB_FAIL_SIM?=n

ifeq ($(ICP_HB_FAIL_SIM),y)
$(info Compiling with Heartbeat Failure Simulation feature)
endif

ifeq ($(KERNELRELEASE),)
KDIR ?= $(INSTALL_MOD_PATH)/lib/modules/$(shell uname -r)/build
ifneq ($(shell if [ -e $(KDIR)/include/config/auto.conf ]; then echo 1; fi),1)
  $(error ERROR: Kernel header files not found.  Install the appropriate \
    kernel development package necessary for building external kernel modules \
    or run 'make oldconfig && make modules_prepare' on kernel src to fix it)
endif
include $(KDIR)/include/config/auto.conf
INSTALL_FW_PATH ?= $(INSTALL_MOD_PATH)/lib/firmware

default: modules

# Error out on missing kernel config dependencies
CONFIG_%:
	$(if $($@), , $(error $@ not enabled in kernel configuration))

KCONFIG_DEP := CONFIG_X86 CONFIG_PCI CONFIG_CRYPTO CONFIG_CRYPTO_HW
KCONFIG_DEP += CONFIG_FW_LOADER
KCONFIG_DEP += CONFIG_CRYPTO_AEAD CONFIG_CRYPTO_AUTHENC CONFIG_CRYPTO_ALGAPI
KCONFIG_DEP += CONFIG_CRYPTO_AES CONFIG_CRYPTO_CBC
KCONFIG_DEP += CONFIG_CRYPTO_SHA1 CONFIG_CRYPTO_SHA256 CONFIG_CRYPTO_SHA512
ifeq ($(QAT_UIO),y)
KCONFIG_DEP += CONFIG_UIO
endif

modules: $(KCONFIG_DEP)
	$(MAKE) -C $(KDIR) M=$(CURDIR) $@

modules_install: modules
	@# Install the external/out-of-tree modules under the
	@# $(INSTALL_MOD_PATH)/lib/modules/$(KERNELRELEASE)/updates/
	@# directory in order to supersede any in-kernel modules with the
	@# same name.  Do not remove the latter modules as that could corrupt
	@# the package manager (e.g. rpm) database.
	$(MAKE) -C $(KDIR) M=$(CURDIR) INSTALL_MOD_DIR=updates $@

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) $@

help:
	@echo ''
	@echo '  Building external/out-of-tree Intel(R) QuickAssist Technology modules.'
	@echo ''
	@echo '  Syntax: make [install-location-options] [target]'
	@echo ''
	@echo '  install-location-options'
	@echo '    [KDIR=<path>]             - path to kernel source (default: running kernel)'
	@echo '    [INSTALL_MOD_PATH=<path>] - optional prefix to the default /lib/modules/...'
	@echo '    [INSTALL_FW_PATH=<path>]  - default: $$(INSTALL_MOD_PATH)/lib/firmware/'
	@echo ''
	@echo '  target'
	@echo '    [modules]                 - default target, build the module(s)'
	@echo '    {modules_install}         - install the module(s) and firmware'
	@echo '    {clean}                   - remove generated files'
	@echo ''

.PHONY: default modules modules_install clean help
else
subdir-ccflags-y += -include $(src)/compat/qat_compat.h
obj-m := drivers/crypto/qat/
endif
