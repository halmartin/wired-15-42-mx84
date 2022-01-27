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

# Ensure The ICP_ENV_DIR environmental var is defined.
ICP_ENV_DIR?=$(ICP_ROOT)/quickassist/build_system/build_files/env_files
ICP_BUILDSYSTEM_PATH?=$(ICP_ROOT)/quickassist/build_system

#Add your project environment Makefile
include $(ICP_ENV_DIR)/environment.mk
#Lac include paths
include $(LAC_DIR)/common.mk
ICP_OS?=linux
ICP_OS_LEVEL?=kernel_space
ICP_BUILD_OUTPUT?=$(ICP_ROOT)/build
INCLUDE_COMPRESSION=1

ifeq ($(WITH_CMDRV),1)
    ifeq ($(WITH_ICP_TARGET),1)
        CMN_ROOT?=$(ICP_ROOT)/quickassist/utilities/libqae_mem/
        CMN_MODULE_NAME?=libqae_mem
    else
        ifeq ($(ICP_OS_LEVEL),user_space)
            CMN_ROOT?=$(ICP_ROOT)/quickassist/utilities/libusdm_drv/
            CMN_MODULE_NAME?=libusdm_drv
        else
            CMN_ROOT?=$(ICP_ROOT)/quickassist/lookaside/access_layer/src/sample_code/performance/qae/
            CMN_MODULE_NAME?=qaeMemDrv
        endif
    endif
endif

ifeq ($(ICP_OS),linux_2.6)
OS=linux
else
OS=freebsd
endif

PERF_SAMPLE_SRC_ROOT ?= $(ICP_ROOT)/quickassist/lookaside/access_layer/src/sample_code/performance
SAMPLE_BUILD_OUTPUT?=$(PERF_SAMPLE_SRC_ROOT)/../build
#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk

#Add the name for the executable, Library or Module output definitions
OUTPUT_NAME=cpa_sample_code


# List of Source Files to be compiled (to be in a single line or on different lines separated by a "\" and tab.

DO_CRYPTO?=1

ifeq ($(DO_CRYPTO),1)
EXTRA_CFLAGS += -DDO_CRYPTO
endif

ifeq ($(SC_CHAINING_ENABLED),1)
    EXTRA_CFLAGS += -DSC_CHAINING_ENABLED
endif

ifeq ($(SC_SYM_UPDATE_DISABLED),1)
    EXTRA_CFLAGS += -DSC_SYM_UPDATE_DISABLED
endif

ifeq ($(SC_PARAM_CHECK_ENABLED),1)
        EXTRA_CFLAGS += -DSC_PARAM_CHECK_ENABLED
endif

ifeq ($(SC_STATS_ENABLED),1)
        EXTRA_CFLAGS += -DSC_STATS_ENABLED
endif

ifeq ($(USE_HARD_CODED_PRIMES),1)
    EXTRA_CFLAGS += -DUSE_HARD_CODED_PRIMES
endif

ifeq ($(ICP_DELAYED_RESPONSE),1)
    EXTRA_CFLAGS += -DICP_DELAYED_RESPONSE
endif

SOURCES:= framework/$(OS)/$(ICP_OS_LEVEL)/cpa_sample_code_utils.c \
	framework/cpa_sample_code_framework.c \
	common/qat_perf_utils.c \
	cpa_sample_code_main.c

ifeq ($(DO_CRYPTO),1)
SOURCES+= crypto/cpa_sample_code_crypto_utils.c \
	crypto/cpa_sample_code_sym_perf.c \
	crypto/cpa_sample_code_sym_perf_dp.c \
	crypto/cpa_sample_code_rsa_perf.c \
	crypto/cpa_sample_code_dsa_perf.c \
	crypto/cpa_sample_code_ecdsa_perf.c \
	crypto/cpa_sample_code_dh_perf.c \
	crypto/qat_sym_main.c \
	crypto/qat_sym_utils.c \
	crypto/cpa_sample_code_sym_update_common.c \
	crypto/cpa_sample_code_sym_update.c \
	crypto/cpa_sample_code_sym_update_dp.c

ifneq ($(WITH_UPSTREAM),1)
SOURCES+= crypto/cpa_sample_code_nrbg_perf.c
endif

endif


ifeq ($(INCLUDE_COMPRESSION),1)
EXTRA_CFLAGS += -DINCLUDE_COMPRESSION
INCLUDES += -I$(PERF_SAMPLE_SRC_ROOT)/../busy_loop/
SOURCES+=  common/qat_perf_buffer_utils.c \
	compression/cpa_sample_code_dc_utils.c \
	compression/cpa_sample_code_dc_dp.c \
	compression/cpa_sample_code_zlib.c \
	compression/cpa_sample_code_dc_stateful2.c \
	common/qat_perf_latency.c \
	common/qat_perf_sleeptime.c \
	compression/qat_compression_main.c \
	compression/qat_compression_utils.c \
	compression/qat_compression_e2e.c \
	../busy_loop/busy_loop.c
ifeq ($(ICP_OS_LEVEL),user_space)
ifeq ($(SC_CHAINING_ENABLED),1)
SOURCES+=  compression/qat_chaining_main.c
endif #chaining enabled
endif

endif #include compression

SC_ENABLE_DYNAMIC_COMPRESSION?=1
ifeq ($(SC_ENABLE_DYNAMIC_COMPRESSION),1)
	EXTRA_CFLAGS += -DSC_ENABLE_DYNAMIC_COMPRESSION
endif

# FreeBSD does not support e-poll
ifeq ($(ICP_OS),freebsd)
EXTRA_CFLAGS += -DSC_EPOLL_DISABLED
else
# For everything else check the environment variable
ifeq ($(SC_EPOLL_DISABLED),1)
EXTRA_CFLAGS += -DSC_EPOLL_DISABLED
endif
endif

ifeq ($(LATENCY_CODE),1)
	EXTRA_CFLAGS += -DLATENCY_CODE=1
endif

ifeq ($(STV_BUFFER_SIZE_DEBUG),1)
	EXTRA_CFLAGS += -DSTV_BUFFER_SIZE_DEBUG=1
endif

ifeq ($(STV_ADD_TEST_IDS),1)
	EXTRA_CFLAGS += -DSTV_ADD_TEST_IDS=1
endif



EXTRA_CFLAGS += -DUSE_ZLIB

ifeq ($(ICP_OS_LEVEL),kernel_space)
SOURCES += framework/$(OS)/$(ICP_OS_LEVEL)/cpa_sample_code_module.c
endif

ifeq ($(DO_CRYPTO),1)
SOURCES += crypto/cpa_sample_code_ike_rsa_perf.c
endif


#common includes between all supported OSes
INCLUDES += -I$(PERF_SAMPLE_SRC_ROOT) \
	-I$(PERF_SAMPLE_SRC_ROOT)/framework/ \
	-I$(PERF_SAMPLE_SRC_ROOT)/framework/$(OS)/$(ICP_OS_LEVEL) \
	-I$(PERF_SAMPLE_SRC_ROOT)/crypto/ \
	-I$(PERF_SAMPLE_SRC_ROOT)/compression/ \
	-I$(PERF_SAMPLE_SRC_ROOT)/common/ \
	-I$(PERF_SAMPLE_SRC_ROOT)/compression/batch_and_pack/ \
	-I$(CMN_ROOT)/

ifeq ($(WITH_CMDRV),1)
	EXTRA_CFLAGS+=-DWITH_CMDRV
endif
ifeq ($(ICP_OS_LEVEL),kernel_space)
    SOURCES += qae/$(OS)/$(ICP_OS_LEVEL)/qae_mem_utils.c
endif


ifeq ($(WITH_UPSTREAM),1)
    EXTRA_CFLAGS+=-DWITH_UPSTREAM
endif

ifeq ($(ICP_OS_LEVEL),user_space)
EXTRA_CFLAGS += -DUSER_SPACE -D_GNU_SOURCE
EXTRA_LDFLAGS += -L$(ICP_BUILD_OUTPUT)

#ifdef WITH_CMDRV
ifeq ($(WITH_CMDRV),1)
	ifeq ($(WITH_ICP_TARGET),1)
	ADDITIONAL_OBJECTS += $(ICP_BUILD_OUTPUT)/libqae_mem.a
	else
	ADDITIONAL_OBJECTS += $(ICP_BUILD_OUTPUT)/libusdm_drv_s.so
	endif
endif
#endif
ifeq ($(WITH_UPSTREAM),1)
    ifeq ($(WITH_ICP_TARGET),1)
        ADDITIONAL_OBJECTS += $(ICP_BUILD_OUTPUT)/libicp_qa_al_s.so
    else
        ADDITIONAL_OBJECTS += $(ICP_BUILD_OUTPUT)/libqat_s.so
    endif
else
	ADDITIONAL_OBJECTS += $(ICP_BUILD_OUTPUT)/libicp_qa_al_s.so
endif





ADDITIONAL_OBJECTS += -L/Lib -lpthread
ADDITIONAL_OBJECTS += -lcrypto -lrt
ifeq ($(WITH_UPSTREAM),1)
ifeq ($(OS),linux)
	ADDITIONAL_OBJECTS += -ludev
endif
endif
ADDITIONAL_OBJECTS += -lcrypto -lz
MACHINE?=$(shell uname -m)
ifeq ($(MACHINE), x86_64)
    ifeq ($(ARCH), i386)
            EXTRA_CFLAGS+=-DSAMPLE_KERNEL64_USER32
    endif
    ifeq ($(ARCH), i686)
             EXTRA_CFLAGS+=-DSAMPLE_KERNEL64_USER32
             EXTRA_CFLAGS+=-march=i686
    endif
endif

else
EXTRA_CFLAGS += -DKERNEL_SPACE

KBUILD_EXTRA_SYMBOLS += $(PERF_SAMPLE_SRC_ROOT)/../../Module.symvers
export $KBUILD_EXTRA_SYMBOLS
endif

#include your $(ICP_OS)_$(ICP_OS_LEVEL).mk file
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk

RM=rm -f

# On the line directly below list the outputs you wish to build for
ifeq ($(ICP_OS_LEVEL),user_space)
install: exe
lib: lib_shared
all: lib_shared exe
else
install: module
endif
.PHONY : clean
clean: perf_clean
perf_clean:
	@echo ; echo "Cleaning performance generated files. $(PERF_SAMPLE_SRC_ROOT) "; 
	$(RM) $(PERF_SAMPLE_SRC_ROOT)/build/$(ICP_OS)/user_space/*;
	$(RM) $(PERF_SAMPLE_SRC_ROOT)/crypto/.*.o.cmd ;
	$(RM) $(PERF_SAMPLE_SRC_ROOT)/compression/.*.o.cmd ;
	$(RM) $(PERF_SAMPLE_SRC_ROOT)/framework/.*.o.cmd ;
	$(RM) $(PERF_SAMPLE_SRC_ROOT)/framework/$(OS)/kernel_space/.*.o.cmd ;

###################Include rules makefiles########################
include $(ICP_BUILDSYSTEM_PATH)/build_files/rules.mk
###################End of Rules inclusion#########################

