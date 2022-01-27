################################################################
#   BSD LICENSE
# 
#   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
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
#  version: QAT1.7.L.4.6.0-00025
################################################################
# Ensure The ICP_ENV_DIR environmental var is defined.
ifndef ICP_ENV_DIR
$(error ICP_ENV_DIR is undefined. Please set the path to your environment makefile \
        "-> setenv ICP_ENV_DIR <path>")
endif

ICP_OS_LEVEL=user_space

#Add your project environment Makefile
include $(ICP_ENV_DIR)/environment.mk
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk

#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk

USE_LEGACY_TRANSPORTCTL=y
EXTRA_CFLAGS += -fno-strict-aliasing
EXTRA_CFLAGS += -DADF_PLATFORM_ACCELDEV
EXTRA_CFLAGS += -DQAT_UIO
EXTRA_CFLAGS += -DUSER_SPACE

ifeq ($(ARCH), i386)
EXTRA_CFLAGS += -m32
endif

ifeq ($(ARCH), i686)
EXTRA_CFLAGS += -m32
EXTRA_CFLAGS += -march=i686
endif


ifeq ($(ICP_HB_FAIL_SIM),y)
$(info Compiling with Heartbeat Failure Simulation feature)
EXTRA_CFLAGS += -DICP_HB_FAIL_SIM -DQAT_HB_FAIL_SIM
endif

ifdef ICP_DC_ONLY
EXTRA_CFLAGS+=-DICP_DC_ONLY
endif

EXTRA_CFLAGS += $(cmd_line_cflags)

INT_INCLUDES=$(addprefix -I, $(shell ls -d $(DIRECT_PATH)/src/include/*/ $(DIRECT_PATH)/src))
EXT_INCLUDES=$(addprefix -I, $(DIRECT_PATH)/include)
REF_INCLUDES=$(addprefix -I, $(ICP_ROOT)/quickassist/utilities/libusdm_drv \
                $(OSAL_DIR)/include \
                $(OSAL_DIR)/src/linux/user_space/include \
                $(API_DIR)/ \
                $(API_DIR)/lac \
                $(API_DIR)/dc \
                $(ICP_ROOT)/quickassist/qat/drivers/crypto/qat/qat_common \
                $(LAC_DIR)/src/common/include)

CFLAGS+=$(REF_INCLUDES) $(INT_INCLUDES) $(EXT_INCLUDES)

SOURCES=$(wildcard *.c)



OUTPUT_NAME=libadf_user

# On the line directly below list the outputs you wish to build for,
# # e.g "lib_static lib_shared exe module" as show below
install: lib_static

###################Include rules makefiles########################
include $(ICP_BUILDSYSTEM_PATH)/build_files/rules.mk
###################End of Rules inclusion#########################
