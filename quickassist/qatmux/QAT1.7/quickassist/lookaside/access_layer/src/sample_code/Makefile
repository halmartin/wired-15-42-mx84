################################################################
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
################################################################

#Set Upstream code based flags
ICP_BUILD_OUTPUT?=$(ICP_ROOT)/build
WITH_UPSTREAM?=1
DRV_OBJ?=libqat_s.so
WITH_CMDRV?=1
export WITH_UPSTREAM
export WITH_CMDRV

# The variables will be updated only if the driver object
# file exists.
PARAM_CHECK = -1
STATS = -1
CHAINING = -1
# The default value for SYM_UPDATE will ensure that
# the capability is disabled if driver object file does
# not exists.
SYM_UPDATE = -1
SYM_SESSION_INUSE = -1

#Check if Driver object file exists and then determine specific
# capabilities through presence of strings.
ifneq ("$(wildcard $(ICP_BUILD_OUTPUT)/$(DRV_OBJ))", "")
#Determine if Param check is enabled
PARAM_CHECK = $(shell strings $(ICP_BUILD_OUTPUT)/$(DRV_OBJ) | grep -q dcParamCheck 1>&2 2> /dev/null; echo $$?)

#Determine if Stats enabled
STATS = $(shell strings $(ICP_BUILD_OUTPUT)/$(DRV_OBJ) | grep -q LacDh_StatsInc 1>&2 2> /dev/null; echo $$?)

#Determine if Symmetric Session Update is available
SYM_UPDATE = $(shell strings $(ICP_BUILD_OUTPUT)/$(DRV_OBJ) | grep -q cpaCySymUpdateSession 1>&2 2> /dev/null; echo $$?)

#Determine if Symmetric Session InUse is available
SYM_SESSION_INUSE = $(shell strings $(ICP_BUILD_OUTPUT)/$(DRV_OBJ) | grep -q cpaCySymSessionInUse 1>&2 2> /dev/null; echo $$?)

#Determine if chaining is available
CHAINING = $(shell strings $(ICP_BUILD_OUTPUT)/$(DRV_OBJ) | grep -q cpaDcChainPerformOp 1>&2 2> /dev/null; echo $$?)

endif


ifeq ($(CHAINING),0)
SC_CHAINING_ENABLED?=1
export SC_CHAINING_ENABLED
endif

ifeq ($(PARAM_CHECK),0)
SC_PARAM_CHECK_ENABLED=1
export SC_PARAM_CHECK_ENABLED
endif

ifeq ($(STATS),0)
SC_STATS_ENABLED=1
export SC_STATS_ENABLED
endif

ifneq ($(SYM_UPDATE),0)
SC_SYM_UPDATE_DISABLED=1
export SC_SYM_UPDATE_DISABLED
endif

ifneq ($(SYM_SESSION_INUSE),0)
SC_SYM_SESSION_INUSE_DISABLED=1
export SC_SYM_SESSION_INUSE_DISABLED
endif


# Ensure The ICP_ENV_DIR environmental var is defined.
ICP_ENV_DIR?=$(ICP_ROOT)/quickassist/build_system/build_files/env_files
ICP_BUILDSYSTEM_PATH?=$(ICP_ROOT)/quickassist/build_system

#Add your project environment Makefile
include $(ICP_ENV_DIR)/environment.mk
#Lac include paths
include $(LAC_DIR)/common.mk
ICP_OS?=linux
ICP_OS_LEVEL?=kernel_space
EXTRA_LDFLAGS += -L$(ICP_BUILD_OUTPUT)
INCLUDE_COMPRESSION=1

# Define CMN_ROOT for Release  
ifeq ($(WITH_CMDRV),1)
    ifeq ($(WITH_ICP_TARGET),1)
        CMN_ROOT?=$(ICP_ROOT)/quickassist/utilities/libqae_mem/
        CMN_MODULE_NAME?=libqae_mem
    else
        CMN_ROOT?=$(ICP_ROOT)/quickassist/utilities/libusdm_drv/
        CMN_MODULE_NAME?=libusdm_drv
        CMN_MODULE_KAPI_NAME?=usdm_drv
    endif
endif

ifeq ($(ICP_OS),linux_2.6)
OS=linux
else
OS=freebsd
endif

SAMPLE_PATH?=$(ICP_ROOT)/quickassist/lookaside/access_layer/src/sample_code
PERF_PATH?=$(ICP_ROOT)/quickassist/lookaside/access_layer/src/sample_code/performance
FIPS_PATH?=$(ICP_ROOT)/quickassist/lookaside/access_layer/src/sample_code/fips
FUNC_PATH=$(SAMPLE_PATH)/functional

#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk

KERNEL_SOURCE_ROOT?=/lib/modules/`uname -r`/build
PERF_OUTPUT_NAME=cpa_sample_code

SAMPLE_BUILD_OUTPUT?=$(SAMPLE_PATH)/build


output_dir:
	test -d $(SAMPLE_BUILD_OUTPUT) || mkdir $(SAMPLE_BUILD_OUTPUT);
fips_code:
	@if test -d $(FIPS_PATH); then echo ; echo 'Building FIPS Code'; fi;
	@if test -d $(FIPS_PATH); \
        then cd $(FIPS_PATH) && \
        $(MAKE) OS_LEVEL=linux\
         TEAM_NAME=$(TEAM_NAME) \
         ICP_ROOT=$(ICP_ROOT) \
         ICP_OS=$(ICP_OS) \
         ICP_OS_LEVEL=$(ICP_OS_LEVEL) \
         KERNEL_SOURCE_ROOT=$(KERNEL_SOURCE_ROOT) \
         ICP_ENV_DIR=$(ICP_ENV_DIR); \
     fi;

fips_user_code:
	@if test -d $(FIPS_PATH); then echo ; echo 'Building FIPS Code'; fi;
	@if test -d $(FIPS_PATH); \
        then cd $(FIPS_PATH) && \
        $(MAKE) OS_LEVEL=linux\
         TEAM_NAME=$(TEAM_NAME) \
         ICP_ROOT=$(ICP_ROOT) \
         ICP_OS=$(ICP_OS) \
         ICP_OS_LEVEL=user_space \
         KERNEL_SOURCE_ROOT=$(KERNEL_SOURCE_ROOT) \
         ICP_ENV_DIR=$(ICP_ENV_DIR); \
     fi;

ifeq ($(LATENCY_CODE),1)
	EXTRA_CFLAGS += -DLATENCY_CODE
endif
ifeq ($(USE_ZLIB),1)
	EXTRA_CFLAGS += -DUSE_ZLIB
endif
ifeq ($(SC_WITH_QAT17),1)
    EXTRA_CFLAGS += -DSC_WITH_QAT17
endif
#ifdef WITH_CMDRV
ifeq ($(WITH_CMDRV),1)
MEM_DRIVER=common_mem_drv_kernel common_mem_drv_user
CODING_STD_MEM_DRIVER=" "
CLEAN_MEM_DRIVER=clean_common_mem_drv
endif
#endif


ifneq ($(ICP_DC_DYN_NOT_SUPPORTED),1)
        EXTRA_CFLAGS += -DSC_ENABLE_DYNAMIC_COMPRESSION
endif

MEM_DRIVER?=perf_qae
CODING_STD_MEM_DRIVER?=perf_qae_coding_standards
CLEAN_MEM_DRIVER?=clean_perf_qae


performance_code: $(MEM_DRIVER) perf_kernel

PERF_ALL=$(MEM_DRIVER) perf_user perf_kernel 


perf_all: $(PERF_ALL)
	@echo ; echo "Build output location $(SAMPLE_BUILD_OUTPUT)"; 


perf_user: output_dir
	@echo ; echo "Building perf_user";
	@echo ; echo "LD_LIBRARY_PATH: $(LD_LIBRARY_PATH)";
	@echo ; echo "ICP_BUILD_OUTPUT: $(ICP_BUILD_OUTPUT)";
	@cd $(PERF_PATH) && $(MAKE) clean ICP_OS_LEVEL=user_space && $(MAKE) ARCH=$(ICP_ARCH_USER) ICP_OS_LEVEL=user_space lib_shared exe
	@cp $(PERF_PATH)/build/$(ICP_OS)/user_space/$(PERF_OUTPUT_NAME)_s.so $(SAMPLE_BUILD_OUTPUT)/;
	@cp $(PERF_PATH)/build/$(ICP_OS)/user_space/$(PERF_OUTPUT_NAME) $(SAMPLE_BUILD_OUTPUT)/;


perf_kernel: output_dir
	@echo ; echo "Building perf_kernel";
	@cd $(PERF_PATH) && $(MAKE)  ICP_OS_LEVEL=kernel_space
	@cp $(PERF_PATH)/build/$(ICP_OS)/kernel_space/$(PERF_OUTPUT_NAME).ko $(SAMPLE_BUILD_OUTPUT)/;

common_mem_drv_kernel: output_dir
	@echo ; echo "Building common_mem_drv kernel space";
	@cd $(CMN_ROOT) && $(MAKE)  ICP_OS_LEVEL=user_space cm_kernel 
	@cp $(CMN_ROOT)/$(CMN_MODULE_KAPI_NAME).ko $(ICP_BUILD_OUTPUT)/;

common_mem_drv_user: output_dir
	@echo ; echo "Building common_mem_drv user space";
	@cd $(CMN_ROOT) && $(MAKE)  ICP_OS_LEVEL=user_space cm_user
	@cp $(CMN_ROOT)/$(CMN_MODULE_NAME).a $(ICP_BUILD_OUTPUT)/;

perf_qae: output_dir
	@echo ; echo "Building perf_qae";
	@cd $(CMN_ROOT)/$(OS)/kernel_space && $(MAKE)  ICP_OS_LEVEL=kernel_space
	@cp $(CMN_ROOT)/$(OS)/kernel_space/build/$(ICP_OS)/$(ICP_OS_LEVEL)/$(CMN_MODULE_KAPI_NAME).ko $(SAMPLE_BUILD_OUTPUT)/;


func: output_dir
	@cd $(FUNC_PATH) && $(MAKE) all;

all: $(MEM_DRIVER) fips_code perf_user perf_kernel func

perf: $(MEM_DRIVER) perf_kernel
RM=rm -f
CODE_CLEAN = curr_dir_clean sub_dir_clean
CURR_DIR_CLEAN = clean_performance_code
SUB_DIR_CLEAN = clean_fips_path clean_func_path clean_perf_path 


.PHONY: clean
clean: $(CODE_CLEAN)
	@echo; echo "Code cleaning ";
curr_dir_clean: $(CURR_DIR_CLEAN)
	@echo; echo "Current directly level cleaning done.";
sub_dir_clean:$(SUB_DIR_CLEAN)
	@echo; echo "Sub-directory level cleaning done.";

ifeq ($(WITH_CMDRV),1)
clean_performance_code: clean_perf_user clean_perf_kernel
	@echo; echo "Cleaning at clean_performance_code done."
else
clean_performance_code: $(CLEAN_MEM_DRIVER) clean_perf_user clean_perf_kernel
	@echo; echo "Cleaning at clean_performance_code done."

endif

clean_fips_path:
	@echo; echo "$(FIPS_PATH) Cleaning.";

clean_perf_path:
	@echo; echo "$(PERF_PATH) Cleaning.";
	@cd $(PERF_PATH) && $(MAKE) clean;

clean_func_path:
	@echo; echo "$(FUNC_PATH) Cleaning. "
	@cd $(FUNC_PATH) && $(MAKE) clean;
#ifdef WITH_CMDRV
clean_common_mem_drv:
	@echo; echo "Common Memory Drive Cleaning $(WITH_CMDRV).";
	$(RM) $(CMN_ROOT)/$(CMN_MODULE_NAME).a ;
	$(RM) $(ICP_BUILD_OUTPUT)/$(CMN_MODULE_NAME).a ;
	$(RM) $(CMN_ROOT)/$(CMN_MODULE_KAPI_NAME).ko ;
	$(RM) $(ICP_BUILD_OUTPUT)/$(CMN_MODULE_KAPI_NAME).ko;
	@cd $(CMN_ROOT) && $(MAKE) clean;
#endif
clean_perf_qae:
	@echo; echo "Qae Mem Driver Cleaning.";
	$(RM) $(CMN_ROOT)/$(OS)/kernel_space/build/$(ICP_OS)/$(ICP_OS_LEVEL)/$(CMN_MODULE_KAPI_NAME).ko ;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/$(CMN_MODULE_KAPI_NAME).ko;
	@cd $(CMN_ROOT)/$(OS)/kernel_space && $(MAKE) clean;

clean_perf_kernel:
	$(RM) $(PERF_PATH)/build/$(ICP_OS)/$(ICP_OS_LEVEL)/$(PERF_OUTPUT_NAME).ko ;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/$(PERF_OUTPUT_NAME).ko ;

clean_perf_user:
	$(RM) $(PERF_PATH)/build/$(ICP_OS)/user_space/$(PERF_OUTPUT_NAME)_s.so ;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/$(PERF_OUTPUT_NAME)_s.so;
	$(RM) $(PERF_PATH)/build/$(ICP_OS)/user_space/$(PERF_OUTPUT_NAME) ;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/$(PERF_OUTPUT_NAME) ;
