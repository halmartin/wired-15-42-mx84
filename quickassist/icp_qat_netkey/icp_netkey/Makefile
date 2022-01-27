#########################################################################
# @par
# GPL LICENSE SUMMARY
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
#  version: icp_qat_netkey.L.0.4.3-1
############################################################################

#QA API  
ICP_API_DIR=$(ICP_ROOT)/quickassist/include/
KBUILD_EXTRA_SYMBOLS += $(ICP_ROOT)/quickassist/lookaside/access_layer/src/Module.symvers
export KBUILD_EXTRA_SYMBOLS
ICP_LAC_API_DIR=$(ICP_API_DIR)/lac/

#include files
EXTRA_CFLAGS += -I$(ICP_API_DIR) -I$(ICP_LAC_API_DIR)

#digest null support for internal testing
EXTRA_CFLAGS += -DINTERNAL_TESTING

obj-m := icp_qat_netkey.o
icp_qat_netkey-objs := icp_netkey_shim_linux.o icp_netkey_shim_aes_sha1.o \
			icp_netkey_shim.o icp_netkey_shim_aes_md5.o \
			icp_netkey_shim_aes_sha256.o icp_netkey_shim_aes_sha512.o \
			icp_netkey_shim_des3_sha1.o icp_netkey_shim_des3_md5.o \
			icp_netkey_shim_des3_sha256.o icp_netkey_shim_des3_sha512.o \
			icp_netkey_hash.o \
			icp_netkey_shim_hmac_sha1.o \
			icp_netkey_shim_rfc3686_ctr_aes.o \
			icp_netkey_shim_hmac_sha256.o \
			icp_netkey_shim_rfc3686_aes_ctr_sha1_hmac.o

default: clean 
	make -C $(KERNEL_SOURCE_ROOT) M=$(PWD) modules

clean:
	/bin/rm -rf *.mod.c *.ko *.o *.a .*o.cmd modules.order .tmp_versions





