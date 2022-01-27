#########################################################################
#
# @par
# This file is provided under a dual BSD/GPLv2 license.  When using or
#   redistributing this file, you may do so under either license.
# 
#   GPL LICENSE SUMMARY
# 
#   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
# 
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of version 2 of the GNU General Public License as
#   published by the Free Software Foundation.
# 
#   This program is distributed in the hope that it will be useful, but
#   WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#   General Public License for more details.
# 
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#   The full GNU General Public License is included in this distribution
#   in the file called LICENSE.GPL.
# 
#   Contact Information:
#   Intel Corporation
# 
#   BSD LICENSE
# 
#   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
#   All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# 
#  version: QAT1.7.L.4.6.0-00025
############################################################################


####################Common variables and definitions########################
export BASE_NAME = usdm_drv
export OSTYPE=$(shell uname -s)

ifeq ($(OSTYPE),FreeBSD)
ICP_OS := freebsd
OS := freebsd
export MAKE := make
DMESG := dmesg -c
FBSD_VERSION := $(shell uname -r | cut -d'-' -f1,2)
ifeq ($(FBSD_VERSION),8.4-RELEASE)
DMESG := dmesg && sudo sysctl kern.msgbuf_clear=1
endif
else
ICP_OS?=linux_2.6
OS?=linux
ICP_OS_LEVEL?=user_space
DMESG := dmesg -C
endif


all: cm_user cm_kernel
	@echo ; echo "Build Completed";
cm_user:
ifneq ($(OSTYPE),FreeBSD)
	@cd $(OS) && \
	$(MAKE) clean ICP_OS_LEVEL=user_space;
	@cd $(OS) && \
	$(MAKE) ICP_OS_LEVEL=user_space lib_shared UT=$(UT) BE=$(BE);
	@cd $(OS) && \
	$(MAKE) ICP_OS_LEVEL=user_space lib_static UT=$(UT) BE=$(BE);
	@cp $(OS)/build/$(ICP_OS)/user_space/lib$(BASE_NAME)_s.so lib$(BASE_NAME)_s.so ;
	@cp $(OS)/build/$(ICP_OS)/user_space/lib$(BASE_NAME).a lib$(BASE_NAME).a ;
else
	@cd $(ICP_OS) && \
	make clean ICP_OS_LEVEL=user_space;
	@cd $(ICP_OS) && \
	make ICP_OS_LEVEL=user_space;
	@cp $(ICP_OS)/user_space/libusdm_drv.so.0 libusdm_drv_s.so ;
	@cp $(ICP_OS)/user_space/libusdm_drv.a libusdm_drv.a ;
endif

cm_kernel:
ifneq ($(OSTYPE),FreeBSD)
	@echo $(ICP_BUILDSYSTEM_PATH)
	@cd $(OS) && \
	$(MAKE) clean ICP_OS_LEVEL=kernel_space && \
	$(MAKE) ICP_OS_LEVEL=kernel_space UT=$(UT) BE=$(BE);
	@mv $(OS)/build/$(ICP_OS)/kernel_space/$(BASE_NAME).a  $(OS)/build/$(ICP_OS)/kernel_space/lib$(BASE_NAME).a ;
	@cp linux/Module.symvers Module.symvers;
	@cp $(OS)/build/$(ICP_OS)/kernel_space/$(BASE_NAME).ko $(BASE_NAME).ko;
else
	@cd $(ICP_OS) && \
	make clean ICP_OS_LEVEL=kernel_space && \
	make ICP_OS_LEVEL=kernel_space UT=$(UT);
	@cp $(ICP_OS)/kernel_space/usdm_drv.ko usdm_drv.ko;
endif

bsd_kernel:
	@cd $(ICP_OS) && \
	make clean ICP_OS_LEVEL=kernel_space && \
	make ICP_OS_LEVEL=kernel_space UT=$(UT);
	@cp $(ICP_OS)/kernel_space/$(BASE_NAME).ko $(BASE_NAME).ko;


clean:
	rm -f *.a *.so *.ko
	@cd $(OS) && \
	rm -f kernel_space/.*.cmd && \
	rm -f kernel_space/.depend* && \
	rm -f user_space/.depend* && \
	$(MAKE) ICP_OS_LEVEL=user_space clean && \
	$(MAKE) ICP_OS_LEVEL=kernel_space clean

doxygen:
	@doxygen qae_mem.h > doxygen_output.txt 2>&1;
	@echo "The doxygen file is available at $(PWD)/html/index.html"
	@echo "The doxygen command output is available at $(PWD)/doxygen_output.txt"
