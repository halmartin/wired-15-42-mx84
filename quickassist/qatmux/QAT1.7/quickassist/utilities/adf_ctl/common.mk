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
PROG_ACY ?= ICP
GCC_VER_49 := 4.9

ifeq ($(CC), icc)
IS_CC_VER_LET49 := 1
else
IS_CC_VER_LET49 := $(shell expr `gcc -dumpversion | cut -f1-2 -d.` \< $(GCC_VER_49))
endif

# Defenses flags
ifneq ($($(PROG_ACY)_DEBUG),y)
# Compile Code with Defenses enabled
$(PROG_ACY)_DEFENSES_ENABLED ?= y
ifeq ($($(PROG_ACY)_DEFENSES_ENABLED),y)
# Stack execution protection
EXTRA_LDFLAGS += -z noexecstack
# Data relocation and protection (RELRO)
EXTRA_LDFLAGS += -z relro -z now
# Stack-based buffer overrun detection
ifeq "$(IS_CC_VER_LET49)" "1"
EXTRA_CXXFLAGS += -fstack-protector
else
EXTRA_CXXFLAGS += -fstack-protector-strong
endif
# Position Independent Execution
EXE_FLAGS += -fPIE -pie
EXTRA_CXXFLAGS += -fPIC
# Fortify source
EXTRA_CXXFLAGS += -O2 -D_FORTIFY_SOURCE=2
# Format string vulnerabilities
EXTRA_CXXFLAGS += -Wformat -Wformat-security
endif
endif

CXX ?= g++
CXXFLAGS += -std=c++0x
CXXFLAGS += -I$(KERNEL_SOURCE_DIR)/drivers/crypto/qat/qat_common

CXXFLAGS += -DQAT_UIO

ifeq ($(ICP_ARCH_USER), i386)
EXTRA_CXXFLAGS += -m32
EXE_FLAGS += -m32
endif

ifeq ($(ICP_ARCH_USER), i686)
EXTRA_CXXFLAGS += -m32
EXE_FLAGS += -m32
EXTRA_CXXFLAGS += -march=i686
endif

ADF_CTL_SRC_DIR = $(ADF_CTL_ROOT)/src
SRC_DIR = $(subst $(ADF_CTL_ROOT)/,,$(CURDIR))
BUILD_DIR = $(ADF_CTL_ROOT)/build
OUTPUT_DIR = $(BUILD_DIR)/$(SRC_DIR)
OUTPUT_BIN = $(OUTPUT_DIR)/$(OUTPUT_NAME)
DEPLOY_BIN = $(ADF_CTL_ROOT)/$(OUTPUT_NAME)

SRCS = $(shell find . -iname "*.cpp")
OBJS = $(foreach file,$(subst $(SRC_DIR)/,,$(SRCS:.cpp=.o)),$(OUTPUT_DIR)/$(file))
DEPS = $(OBJS:.o=.d)
DEPS_CXXFLAGS = -MMD -MP

$(OUTPUT_BIN): $(OBJS)
	@mkdir -p $(OUTPUT_DIR)
	$(CXX) $(EXE_FLAGS) -o $(OUTPUT_BIN) $(OBJS) $(LDFLAGS) $(EXTRA_LDFLAGS)

$(DEPLOY_BIN): $(OUTPUT_BIN)
	@cp -f $(OUTPUT_BIN) $(DEPLOY_BIN)

$(OUTPUT_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $(DEPS_CXXFLAGS) $(CXXFLAGS) $(EXTRA_CXXFLAGS) -o $@ $<

common_clean:
	rm -f $(OBJS)
	rm -f $(DEPS)
	rm -f $(OUTPUT_BIN)

-include $(DEPS)

.PHONY: common_clean $(DEPLOY_BIN)
