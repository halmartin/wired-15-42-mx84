#########################################################################
#
# Targets supported
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

#Add your project environment Makefile
include $(ICP_ENV_DIR)/environment.mk

#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk

#include adf platform
include $(ADF_DIR)/adf_platform.mk

#Add the name for the executable, Library or Module output definitions
OUTPUT_NAME=adf_accel_mgr

# List of Source Files to be compiled (to be in a single line or on different lines separated by a "\" and tab.
SOURCES= adf_init.c
SOURCES+=adf_cfg.c
SOURCES+=adf_cfg_dp.c
SOURCES+=adf_cfg_debug.c
SOURCES+=adf_devmgr.c
SOURCES+=adf_user_proxy.c
SOURCES+=adf_esram.c

ifdef ICP_SRIOV
EXTRA_CFLAGS+=-DICP_SRIOV
endif

ifneq ($(ADF_PLATFORM),ACCELDEVVF)
SOURCES+=adf_pf2vf_ctrl.c
else
SOURCES+=adf_vf2pf_ctrl.c
endif

ifdef ADF_ACCEL_MGR_DEBUG
EXTRA_CFLAGS+=-D_DEBUG_
endif

#common includes between all supported OSes
ADF_INCLUDE = -I$(ADF_DIR)/include
ADF_INCLUDE+= -I$(ADF_DIR)/wireless/include
ADF_INCLUDE+= -I$(ADF_DIR)/dyn/include
ADF_INCLUDE+= -I$(ADF_DRIVERS_DIR)/include
ADF_INCLUDE+= -I$(ADF_PLATFORM_DIR)/include
ADF_INCLUDE+= -I$(ADF_TRANSPORT_DIR)/include
ADF_INCLUDE+= -I$(ADF_ACCEL_MGR_DIR)/include
ADF_INCLUDE+= -I$(ADF_ACCELENGINE_DIR)/include
ADF_INCLUDE+= -I$(HAL_DIR)/include
ADF_INCLUDE+= -I$(HAL_DIR)/include/platform/$(ICP_TOOLS_TARGET)
ADF_INCLUDE+= -I$(HAL_OS_DIR)

INCLUDES += $(ADF_INCLUDE) -I$(API_DIR)

#include your $(ICP_OS)_$(ICP_OS_LEVEL).mk file
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk

# On the line directly below list the outputs you wish to build for,
# e.g "lib_static lib_shared exe module" as show below
install: lib_static

copy:
	echo "copying $(ICP_FINAL_OUTPUT_DIR)/$(LIB_STATIC) to $(ADF_DIR)/build/$(ICP_OS)"; \
	cp $(ICP_FINAL_OUTPUT_DIR)/$(LIB_STATIC) $(ADF_DIR)/build/$(ICP_OS)
ifdef ONE_KO_RELEASE_PACKAGE
	cp $(ICP_FINAL_OUTPUT_DIR)/*.o $(ADF_DIR)/build/$(ICP_OS)
endif

accel_mgr: all
###################Include rules makefiles########################
include $(ICP_BUILDSYSTEM_PATH)/build_files/rules.mk
###################End of Rules inclusion#########################
