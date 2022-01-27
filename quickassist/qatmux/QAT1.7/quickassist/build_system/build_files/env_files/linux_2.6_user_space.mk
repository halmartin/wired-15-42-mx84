###############################################################################
# @par
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
###############################################################################

################################################################
#
# Included by all components when building in user space
#
################################################################

#--------------------------------------------------------------
#
# INCLUDES section
#
#-------------------------------------------------------------

INCLUDES+=-I/usr/include \
          -I$(API_DIR)   \
          -I$(ADF_CMN_DIR) \
          -I$(OSAL_DIR)/include \
          -I$(OSAL_DIR)/src/linux/user_space/include

#----------------------------------------------------------
#
# EXTRA_CFLAGS section
#
#----------------------------------------------------------

EXTRA_CFLAGS += $(cmd_line_cflags)
EXTRA_CFLAGS += -fno-strict-aliasing
EXTRA_LDFLAGS +=-whole-archive

ifeq ($(ARCH), i386)
EXTRA_CFLAGS += -m32
EXE_FLAGS += -m32
EXE_FLAGS += -shared-libgcc
EXTRA_LDFLAGS += -m32
LIB_SHARED_FLAGS += -m elf_i386
endif

ifeq ($(ARCH), i686)
EXTRA_CFLAGS += -m32
EXTRA_CFLAGS += -march=i686
EXE_FLAGS += -m32
EXE_FLAGS += -march=i686
EXE_FLAGS += -shared-libgcc
EXTRA_LDFLAGS += -m32
EXTRA_LDFLAGS += -march=i686
LIB_SHARED_FLAGS += -m elf_i386
endif
