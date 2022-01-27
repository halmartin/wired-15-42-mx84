###################
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
###################

MKDEFINC_DIR = $(ICP_TOOLS_XSC_CORE_ROOT)/makDefs
include $(MKDEFINC_DIR)/commonMak.def

LIB	= $(LIB_DIR)/$(ICP_TOOLS_TARGET)/icp_ae_loader_user.a
LIB_OBJS =  ../halAe.o ../halAeDbg.o ../halAeHw.o ../halAeUsr.o halAeChip.o \
            ../halOSUsr.o halMmapUsr.o ../halAeLinuxUsr.o ../halMemScrub.o \
            ../../uclo/uclo_exp.o ../../uclo/uclo.o ../../uclo/uclo_hw.o \
            ../../uclo/uclo_ivd.o ../../uclo/uclo_mof.o ../../uclo/uclo_overlay.o \
            ../../uclo/uclo_usr.o ../../uclo/uclo_helper.o  ../../uclo/uclo_suof.o
BLD_OBJS = $(addprefix $(OBJ_DIR)/, $(LIB_OBJS))

INCLUDE = \
	-I$(ICP_TOOLS_CMN_ROOT)/include \
	-I$(ICP_TOOLS_CMN_ROOT)/include/os/linux_usr \
	-I$(ICP_TOOLS_CMN_ROOT)/include/platform/$(ICP_TOOLS_TARGET) \
	-I$(ICP_TOOLS_XSC_CORE_ROOT)/halAe \
	-I$(ICP_TOOLS_XSC_CORE_ROOT)/halAe/$(ICP_TOOLS_TARGET) \
	-I$(ICP_TOOLS_XSC_CORE_ROOT)/halAe/include/linux_usr \
	-I$(ICP_TOOLS_XSC_CORE_ROOT)/uclo \
	-I$(ICP_TOOLS_XSC_CORE_ROOT)/uclo/include/linux_usr

all : lib

lib : $(LIB)
	@echo
	@echo  $(LIB) SUCCESSFULLY built
	@echo

exe : lib

$(LIB) : $(BLD_OBJS)
	-@ $(MKDIR) "$(LIB_DIR)"
	-@ $(MKDIR) "$(LIB_DIR)/$(ICP_TOOLS_TARGET)"
	$(AR) $(ARFLAGS) $(LIB) $(subst ../uclo/,,$(BLD_OBJS))

.PHONY: clean
clean	: 
	$(RM) $(BLD_OBJS) $(LIB)
