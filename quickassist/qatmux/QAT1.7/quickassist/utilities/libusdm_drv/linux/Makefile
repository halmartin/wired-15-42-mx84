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
ICP_OS?=linux_2.6
OS?=linux
ICP_OS_LEVEL?=user_space
CMN_ROOT?=$(ICP_ROOT)/quickassist/utilities/libusdm_drv
QAT_DRV?=$(ICP_ROOT)/quickassist/qat

#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk

# List of Source Files to be compiled (to be in a single line or on different lines separated by a "\" and tab.

ifeq ($(ICP_OS_LEVEL),user_space)
SOURCES:= $(ICP_OS_LEVEL)/qae_mem_utils.c $(ICP_OS_LEVEL)/qae_mem_hugepage_utils.c
else
MODULE_SOURCES:= $(ICP_OS_LEVEL)/qae_mem_drv.c
SOURCES:=$(ICP_OS_LEVEL)/qae_mem_utils.c
endif

ifeq ($(UT),1)
MODULE_SOURCES+= ../test/$(OS)/$(ICP_OS_LEVEL)/qae_mem_drv_utils.c
else
MODULE_SOURCES+= $(ICP_OS_LEVEL)/qae_mem_drv_utils.c
endif

INCLUDES += -I$(CMN_ROOT)

INCLUDES += -I$(CMN_ROOT)/$(OS)/include 

ifeq ($(ICP_ADF_IOMMU), 1)
EXTRA_CFLAGS += -DICP_ADF_IOMMU
KBUILD_EXTRA_SYMBOLS+=$(QAT_DRV)/src/Module.symvers
export KBUILD_EXTRA_SYMBOLS
endif 

ifeq ($(ICP_OSAL_IOMMU), 1)
EXTRA_CFLAGS += -DICP_OSAL_IOMMU
KBUILD_EXTRA_SYMBOLS+=$(QAT_DRV)/src/Module.symvers
export KBUILD_EXTRA_SYMBOLS
endif 

ifeq ($(ICP_QDM_IOMMU), 1)
EXTRA_CFLAGS += -DICP_QDM_IOMMU
KBUILD_EXTRA_SYMBOLS+=$(QAT_DRV)/Module.symvers
export KBUILD_EXTRA_SYMBOLS
endif

ifdef QAE_USE_128K_SLABS
EXTRA_CFLAGS+=-DQAE_NUM_PAGES_PER_ALLOC=32
endif

ifeq ($(ICP_OS_LEVEL),user_space)
EXTRA_CFLAGS += -DUSER_SPACE
EXTRA_CFLAGS += -Wextra -Werror -Wno-missing-field-initializers
ifdef ICP_X86
EXTRA_CFLAGS += -m32 -D_FILE_OFFSET_BITS=64
LIB_SHARED_FLAGS += -m elf_i386
endif
ifdef ICP_DISABLE_SECURE_MEM_FREE
EXTRA_CFLAGS += -DICP_DISABLE_SECURE_MEM_FREE
endif
ifdef ICP_WITHOUT_THREAD
EXTRA_CFLAGS += -DICP_WITHOUT_THREAD
endif
else

EXTRA_CFLAGS += -DKERNEL_SPACE
ifeq ($(BE) ,1)
KBUILD_EXTRA_SYMBOLS+=$(BE_DIR)/run/linuxKernel/Module.symvers
export KBUILD_EXTRA_SYMBOLS
endif
ifdef ICP_NO_PROC_SUPPORT
EXTRA_CFLAGS += -DICP_NO_PROC_SUPPORT
endif
endif

ifeq ($(ICP_OS_LEVEL),user_space)
#include os dependent rules
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk
lib: lib_shared
all: lib_shared
OUTPUT_NAME=lib$(BASE_NAME)
export OUTPUT_NAME
else

OUTPUT_NAME=$(BASE_NAME)
export OUTPUT_NAME

#kernel space rules here 
#produce two artefacts: module and static library and copy them
ifeq ($(OS),linux)
EXTRA_CFLAGS+=-I$(INCLUDES) -Werror
obj-m+=$(OUTPUT_NAME).o
$(OUTPUT_NAME)-objs :=$(patsubst %.c,%.o, $(MODULE_SOURCES)) $(ADDITIONAL_KERNEL_LIBS)
lib-m := $(patsubst %.c,%.o, $(SOURCES)) $(patsubst %.S,%.o, $(ASM_SOURCES))
$(LIB_STATIC): dirs
	@echo 'Creating static library ${LIB_STATIC}'; \
	$(MAKE) -C $(KERNEL_SOURCE_ROOT)/ M=$(PWD) obj-m=""; \
	echo 'Copying outputs';\
	mv -f $(OBJ) $($(PROG_ACY)_FINAL_OUTPUT_DIR);\
	test -f built-in.o  &&  mv -f built-in.o $($(PROG_ACY)_FINAL_OUTPUT_DIR);\
	test -f lib.a  &&  mv lib.a $($(PROG_ACY)_FINAL_OUTPUT_DIR)/$(LIB_STATIC);\
	test -f $(OUTPUT_NAME).ko  &&  mv -f $(OUTPUT_NAME).ko $($(PROG_ACY)_FINAL_OUTPUT_DIR);\
	test -f $(OUTPUT_NAME).o  &&  mv -f $(OUTPUT_NAME).o $($(PROG_ACY)_FINAL_OUTPUT_DIR);\
	$(RM) -rf *.mod.* .*.cmd;

$(MODULENAME): dirs
	@echo 'Creating kernel module'; \
	$(MAKE) -C $(KERNEL_SOURCE_ROOT)/ M=$(PWD); \
	echo 'Copying outputs';\
	mv -f $(OBJ) $($(PROG_ACY)_FINAL_OUTPUT_DIR);\
	test -f built-in.o  &&  mv -f built-in.o $($(PROG_ACY)_FINAL_OUTPUT_DIR);\
	test -f lib.a  &&  mv lib.a $($(PROG_ACY)_FINAL_OUTPUT_DIR)/$(LIB_STATIC);\
	test -f $(OUTPUT_NAME).ko  &&  mv -f $(OUTPUT_NAME).ko $($(PROG_ACY)_FINAL_OUTPUT_DIR);\
	test -f $(OUTPUT_NAME).o  &&  mv -f $(OUTPUT_NAME).o $($(PROG_ACY)_FINAL_OUTPUT_DIR);\
	$(RM) -rf *.mod.* .*.cmd;
else
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk
endif

all: module
install: module
endif
###################Include rules makefiles########################
include $(ICP_BUILDSYSTEM_PATH)/build_files/rules.mk

###################End of Rules inclusion#########################
