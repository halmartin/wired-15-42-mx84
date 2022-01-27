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
#  version: QAT1.6.L.2.6.0-65
############################################################################


####################Common variables and definitions########################
# Ensure The ICP_ENV_DIR environmental var is defined.
ifndef ICP_ENV_DIR
$(error ICP_ENV_DIR is undefined. Please set the path to your environment makefile \
        "-> setenv ICP_ENV_DIR <path>")
endif

#Add your project environment Makefile
include $(ICP_ENV_DIR)/environment.mk


#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk


#Add the name for the executable, Library or Module output definitions
ifeq ($(ICP_OS_LEVEL),user_space)

ifeq ($(ICP_ARCH_USER), i386)
EXTRA_CFLAGS += -m32
EXE_FLAGS += -m32
EXTRA_LDFLAGS += -m32
LIB_SHARED_FLAGS += -m elf_i386
endif

ifeq ($(ICP_ARCH_USER), i686)
EXTRA_CFLAGS += -m32
EXTRA_CFLAGS += -march=i686
EXE_FLAGS += -m32
EXE_FLAGS += -march=i686
EXTRA_LDFLAGS += -m32
EXTRA_LDFLAGS += -march=i686
LIB_SHARED_FLAGS += -m elf_i386
endif

OUTPUT_NAME=libqat_mux
else
OUTPUT_NAME=qat_mux
endif

LIB_STATIC=$(OUTPUT_NAME).a
LIB_SHARED=$(OUTPUT_NAME).so

# List of Source Files to be compiled (to be in a single line or on different lines separated by a "\" and tab.
SOURCES=cpa_impl_mux.c cpa_impl_mux_api.c

#common includes between all supported OSes
INCLUDES += -I$(CPA_MUX_DIR)/include \
	-I$(API_DIR)/lac \
	-I$(API_DIR)/dc \
	-I$(LAC_DIR)/include

ifneq ($(MERAKI_CROSS_COMPILE_STAGING_DIR),)
INCLUDES +=-I$(MERAKI_CROSS_COMPILE_STAGING_DIR)/include \
           -L$(MERAKI_CROSS_COMPILE_STAGING_DIR)/lib
LIB_SHARED_FLAGS += -L$(MERAKI_CROSS_COMPILE_STAGING_DIR)/lib
endif

#include your $(ICP_OS)_$(ICP_OS_LEVEL).mk file
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk

ifdef ONE_KO_RELEASE_PACKAGE
EXTRA_CFLAGS += -DONE_KO_RELEASE_PACKAGE
endif

ifeq ($(ICP_OS_LEVEL),user_space)
EXTRA_CFLAGS += -DUSER_SPACE -g
ADDITIONAL_OBJECTS += $(OSAL_DIR)/src/build/$(ICP_OS)/$(ICP_OS_LEVEL)/libosal.a
LIB_SHARED_FLAGS += -lpthread
LIB_SHARED_FLAGS += -ldl
else
EXTRA_CFLAGS += -DKERNEL_SPACE
ADDITIONAL_KERNEL_LIBS += ../../../QAT1.6/quickassist/lookaside/access_layer/build/libs/libosal.a
MODULE_SOURCES=cpa_impl_linux_module.c
MODULE_SOURCES+= cpa_impl_linux_symbols.c
KRELCHK?=$(shell uname -r | cut -c1-5)
ifneq ($(KRELCHK),)
EXTRA_CFLAGS += -D"KBUILD_MODNAME=KBUILD_STR($(OUTPUT_NAME))"
endif
endif

ifeq ($(ICP_OS_LEVEL),user_space)
install: lib_static lib_shared
else
install: module
endif

###################Include rules makefiles########################
include $(ICP_BUILDSYSTEM_PATH)/build_files/rules.mk
###################End of Rules inclusion#########################
