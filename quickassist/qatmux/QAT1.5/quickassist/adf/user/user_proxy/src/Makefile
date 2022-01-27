############################################################################
#
#  Targets supported
#  all     - builds everything and installs
#  install - identical to all
#  clean   - clears derived objects except the .depend files
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

#Set the OS_LEVEL to be user space for this build.
ICP_OS_LEVEL=user_space

#Add your project environment Makefile
include $(ICP_ENV_DIR)/environment.mk

#include adf platform
include $(ADF_DIR)/adf_platform.mk

#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk

#Add the name for the executable, Library or Module output definitions
OUTPUT_NAME= libadf_proxy


ifdef ICP_WITHOUT_THREAD
EXTRA_CFLAGS+=-DICP_WITHOUT_THREAD
endif

ifdef ADF_USER_PROXY_DEBUG
EXTRA_CFLAGS+=-D_DEBUG_
endif

ifeq ($(ARCH), i386)
EXTRA_CFLAGS += -m32
endif

ifeq ($(ARCH), i686)
EXTRA_CFLAGS += -m32
EXTRA_CFLAGS += -march=i686
endif

# List of Source Files to be compiled (to be in a single line or on different lines separated by a "\" and tab.
SOURCES = adf_user_init.c \
          adf_user_cfg.c \
		  adf_user_transport_ctrl.c \
		  adf_user_transport_dyn_pool.c \
		  adf_process_proxy.c \
		  adf_user_ETring_mgr.c \
		  adf_user_ETring_mgr_dp.c

INCLUDES+= -I$(ADF_DIR)/include           \
           -I$(ADF_PROCESS_PROXY_DIR)/include \
           -I$(ADF_ACCEL_MGR_DIR)/include \
           -I$(ADF_TRANSPORT_DIR)/include \
           -I$(ADF_DRIVERS_DIR)/include   \
           -I$(ADF_PLATFORM_DIR)/include  \
           -I$(ADF_CTL_DIR)/include

CFLAGS = -D_GNU_SOURCE -O2 $(INCLUDES)
#needed by SAL
EXTRA_CFLAGS += -fPIC
#include your $(ICP_OS)_$(ICP_OS_LEVEL).mk file
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk

# On the line directly below list the outputs you wish to build for,
# e.g "lib_static lib_shared exe module" as show below
install: lib_static

copy:
	echo "copying $(ICP_FINAL_OUTPUT_DIR)/$(OUTPUT_NAME).a to $(ADF_DIR)/build/$(ICP_OS)"; \
	cp $(ICP_FINAL_OUTPUT_DIR)/$(OUTPUT_NAME).a $(ADF_DIR)/build/$(ICP_OS)

user_proxy: all
###################Include rules makefiles########################
include $(ICP_BUILDSYSTEM_PATH)/build_files/rules.mk
###################End of Rules inclusion#########################
