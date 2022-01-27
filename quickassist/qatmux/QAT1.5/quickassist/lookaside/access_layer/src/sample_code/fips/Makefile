#########################################################################
#  
# @par
# This file is provided under a dual BSD/GPLv2 license.  When using or 
#   redistributing this file, you may do so under either license.
# 
#   GPL LICENSE SUMMARY
# 
#   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
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
#   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
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
#  version: QAT1.5.L.1.11.0-36
############################################################################

####################Common variables and definitions########################


# Ensure The ICP_ENV_DIR environmental var is defined.
ifndef ICP_ENV_DIR
$(error ICP_ENV_DIR is undefined. Please set the path to your environment makefile \
        "-> setenv ICP_ENV_DIR <path>")
endif

#Add your project environment Makefile
include $(ICP_ENV_DIR)/environment.mk
include $(ICP_LAC_DIR)/common.mk

BACKDOOR_API_INCLUDE_DIR=$(LAC_DIR)/include


ICP_OS?=linux
ICP_OS_LEVEL?=kernel_space
ICP_BUILD_OUTPUT?=$(ICP_ROOT)/build/

CMN_ROOT?=$(ICP_ROOT)/quickassist/lookaside/access_layer/src/sample_code/performance/qae/
CMN_MODULE_NAME?=qaeMemDrv

ifeq ($(ICP_OS_LEVEL),kernel_space)
EXTRA_CFLAGS+= -std=gnu99 -Wall -DDEBUG
EXTRA_CFLAGS+= -DKERNEL_SPACE
else
CFLAGS+= -std=gnu99 -Wall -DDEBUG
LD_LIBRARY_PATH+= $(ICP_BUILD_OUTPUT)

ifeq ($(WITH_CPA_MUX),1)
        EXTRA_CFLAGS+=-DWITH_CPA_MUX
        ADDITIONAL_OBJECTS += $(ICP_BUILD_OUTPUT)/libqat_mux_s.so
else
        ADDITIONAL_OBJECTS += $(ICP_BUILD_OUTPUT)/libicp_qa_al_s.so
endif

endif

#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk

#Add the name for the executable, Library or Module output definitions
OUTPUT_NAME=fips_sample

# List of Source Files to be compiled (to be in a single line or on different lines separated by a "\" and tab.

# List of Source Files to be compiled as a module (to be in a single line or on different lines separated by a "\" and tab.
SOURCES=cpa_fips_sample_utils.c \
        cpa_fips_sample_rsa.c \
        cpa_fips_sample_dsa.c \
        cpa_fips_sample_ecdsa.c \
        cpa_fips_sample_aes_gcm.c

ifeq ($(ICP_OS_LEVEL),kernel_space)
SOURCES+=cpa_fips_sample_linux_kernel_module.c
endif

ifeq ($(CPM_LACKS_DRBG),1)
        EXTRA_CFLAGS+=-DCPM_LACKS_DRBG
        $(info Setting define CPM_LACKS_DRBG)
endif

SOURCES+=../performance/qae/linux/$(ICP_OS_LEVEL)/qae_mem_utils.c

#common includes between all supported OSes
INCLUDES+=-I. -I$(CMN_ROOT) -I$(BACKDOOR_API_INCLUDE_DIR)

#include your $(ICP_OS)_$(ICP_OS_LEVEL).mk file
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk

# On the line directly below list the outputs you wish to build for
ifeq ($(ICP_OS_LEVEL),kernel_space)
install: module
else
install: lib_shared lib_static
endif
	

###################Include rules makefiles########################
include $(ICP_BUILDSYSTEM_PATH)/build_files/rules.mk
###################End of Rules inclusion#########################
