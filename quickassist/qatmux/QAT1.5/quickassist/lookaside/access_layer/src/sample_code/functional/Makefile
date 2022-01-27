################################################################
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
################################################################
SAMPLE_PATH?=$(ICP_ROOT)/quickassist/lookaside/access_layer/src/sample_code/functional
DC_PATH?=$(SAMPLE_PATH)/dc
SYM_PATH?=$(SAMPLE_PATH)/sym
ASYM_PATH?=$(SAMPLE_PATH)/asym
ICP_API_DIR?=$(ICP_ROOT)/quickassist/include/
ICP_LAC_DIR?=$(ICP_ROOT)/quickassist/lookaside/access_layer/

CMN_ROOT?=$(ICP_ROOT)/quickassist/lookaside/access_layer/src/sample_code/performance/qae/
CMN_MODULE_NAME?=qaeMemDrv

export CMN_ROOT
export CMN_MODULE_NAME

ifndef SAMPLE_PATH
$(error SAMPLE_PATH is undefined. Please set the path to your environment makefile \
        "-> setenv SAMPLE_PATH <functional sample code path>")
endif

ifndef DC_PATH
$(error DC_PATH is undefined. Please set the path to your environment makefile \
        "-> setenv DC_PATH <SAMPLE_PATH/dc>")
endif

ifndef SYM_PATH
$(error SYM_PATH is undefined. Please set the path to your environment makefile \
        "-> setenv SYM_PATH <SAMPLE_PATH/sym>")
endif

ifndef ASYM_PATH
$(error ASYM_PATH is undefined. Please set the path to your environment makefile \
        "-> setenv ASYM_PATH <SAMPLE_PATH/asym>")
endif

ifndef ICP_API_DIR
$(error ICP_API_DIR is undefined. Please set the path to the ICP_API_DIR \
        "-> setenv ICP_API_DIR <ICP_ROOT>/quickassist/include/")
endif

ifndef  ICP_LAC_DIR
$(error ICP_LAC_DIR is undefined. Please set the path to the ICP_LAC_DIR \
        "-> setenv ICP_LAC_DIR <ICP_ROOT>/quickassist/lookaside/access_layer/")
endif

SAMPLE_BUILD_OUTPUT?=$(SAMPLE_PATH)/build

output_dir:
	test -d $(SAMPLE_BUILD_OUTPUT) || mkdir $(SAMPLE_BUILD_OUTPUT);

BUILD_DC=dc_dp_sample stateless_sample stateful_sample \
         stateless_multi_op_checksum_sample
	
dc: $(BUILD_DC)
	@echo ; echo "Build dc component";

dc_dp_sample: output_dir
	@cd $(DC_PATH)/dc_dp_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(DC_PATH)/dc_dp_sample/dc_dp_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(DC_PATH)/dc_dp_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(DC_PATH)/dc_dp_sample/dc_dp_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif

stateless_sample: output_dir
	@cd $(DC_PATH)/stateless_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(DC_PATH)/stateless_sample/dc_stateless_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(DC_PATH)/stateless_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(DC_PATH)/stateless_sample/dc_stateless_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
stateful_sample: output_dir
	@cd $(DC_PATH)/stateful_sample && $(MAKE) clean ICP_OS_LEVEL=user_space && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(DC_PATH)/stateful_sample/dc_stateful_sample $(SAMPLE_BUILD_OUTPUT)/;
	@cp $(DC_PATH)/stateful_sample/paper4 $(SAMPLE_BUILD_OUTPUT)/;

stateless_multi_op_checksum_sample: output_dir
	@cd $(DC_PATH)/stateless_multi_op_checksum_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(DC_PATH)/stateless_multi_op_checksum_sample/dc_stateless_multi_op_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(DC_PATH)/stateless_multi_op_checksum_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(DC_PATH)/stateless_multi_op_checksum_sample/dc_stateless_multi_op_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif

BUILD_SYN=algchaining_sample ccm_sample cipher_sample  \
          gcm_sample hash_file_sample \
          hash_sample ipsec_sample \
          ssl_sample sym_dp_sample

ifneq ($(WITH_UPSTREAM),1)
BUILD_SYN += drbg_sample nrbg_sample
endif

sym:$(BUILD_SYN) 
	@echo ; echo "Build sym component";
algchaining_sample: output_dir
	@cd $(SYM_PATH)/alg_chaining_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/alg_chaining_sample/algchaining_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/alg_chaining_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/alg_chaining_sample/algchaining_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
ccm_sample: output_dir
	@cd $(SYM_PATH)/ccm_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/ccm_sample/ccm_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/ccm_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/ccm_sample/ccm_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
cipher_sample: output_dir
	@cd $(SYM_PATH)/cipher_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/cipher_sample/cipher_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/cipher_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/cipher_sample/cipher_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
drbg_sample: output_dir
	@cd $(SYM_PATH)/drbg_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/drbg_sample/drbg_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/drbg_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/drbg_sample/drbg_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
gcm_sample: output_dir
	@cd $(SYM_PATH)/gcm_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/gcm_sample/gcm_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/gcm_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/gcm_sample/gcm_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
hash_file_sample: output_dir
	@cd $(SYM_PATH)/hash_file_sample && $(MAKE) clean  ICP_OS_LEVEL=user_space&& $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/hash_file_sample/hash_file_sample $(SAMPLE_BUILD_OUTPUT)/;
	
hash_sample: output_dir
	@cd $(SYM_PATH)/hash_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/hash_sample/hash_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/hash_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/hash_sample/hash_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
ipsec_sample: output_dir
	@cd $(SYM_PATH)/ipsec_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/ipsec_sample/ipsec_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/ipsec_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/ipsec_sample/ipsec_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
nrbg_sample: output_dir
	@cd $(SYM_PATH)/nrbg_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/nrbg_sample/nrbg_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/nrbg_sample && $(MAKE) && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/nrbg_sample/nrbg_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
ssl_sample: output_dir
	@cd $(SYM_PATH)/ssl_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/ssl_sample/ssl_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/ssl_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/ssl_sample/ssl_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif
	
sym_dp_sample: output_dir
	@cd $(SYM_PATH)/symdp_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(SYM_PATH)/symdp_sample/sym_dp_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(SYM_PATH)/symdp_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(SYM_PATH)/symdp_sample/sym_dp_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif

BUILD_ASYM=diffie_hellman_sample prime_sample

asym: $(BUILD_ASYM)
	@echo ; echo "Build asym component";

diffie_hellman_sample: output_dir
	@cd $(ASYM_PATH)/diffie_hellman_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(ASYM_PATH)/diffie_hellman_sample/dh_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(ASYM_PATH)/diffie_hellman_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(ASYM_PATH)/diffie_hellman_sample/dh_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif

prime_sample: output_dir
	@cd $(ASYM_PATH)/prime_sample && $(MAKE) clean && $(MAKE) ICP_OS_LEVEL=user_space
	@cp $(ASYM_PATH)/prime_sample/prime_sample $(SAMPLE_BUILD_OUTPUT)/;
ifneq ($(WITH_UPSTREAM),1)
	@cd $(ASYM_PATH)/prime_sample && $(MAKE) ICP_OS_LEVEL=kernel_space
	@cp $(ASYM_PATH)/prime_sample/prime_sample.ko $(SAMPLE_BUILD_OUTPUT)/;
endif

CLEAN_DC=clean_dc_dp_sample clean_stateless_sample     \
	 clean_stateful_sample                         \
	 clean_stateless_multi_op_checksum_sample
	   
CLEAN_SYM=clean_algchaining_sample clean_ccm_sample    \
	  clean_cipher_sample                          \
	  clean_gcm_sample clean_hash_file_sample      \
	  clean_hash_sample clean_ipsec_sample         \
	  clean_ssl_sample                             \
	  clean_sym_dp_sample
	    
ifneq ($(WITH_UPSTREAM),1)
CLEAN_SYM += clean_drbg_sample clean_nrbg_sample
endif

CLEAN_ASYM=clean_diffie_hellman_sample                 \
	   clean_prime_sample

clean_dc: $(CLEAN_DC)
	@echo ; echo "Build dc component.";
clean_sym: $(CLEAN_SYM)
	@echo ; echo "Build sym component.";
clean_asym: $(CLEAN_ASYM)
	@echo ; echo "Build asym component.";
      
clean_dc_dp_sample:
	@cd $(DC_PATH)/dc_dp_sample && $(MAKE) clean ICP_OS_LEVEL=user_space && \
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(DC_PATH)/dc_dp_sample/dc_dp_sample 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dc_dp_sample;
	$(RM) $(DC_PATH)/dc_dp_sample/dc_dp_sample.ko; 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dc_dp_sample.ko;

clean_stateless_sample: 
	@cd $(DC_PATH)/stateless_sample && $(MAKE) clean ICP_OS_LEVEL=user_space && \
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(DC_PATH)/stateless_sample/dc_stateless_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dc_stateless_sample.ko;
	$(RM) $(DC_PATH)/stateless_sample/dc_stateless_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dc_stateless_sample;

clean_stateful_sample: 
	@cd $(DC_PATH)/stateful_sample && $(MAKE) clean  ICP_OS_LEVEL=user_space  
	$(RM) $(DC_PATH)/stateful_sample/dc_stateful_sample; 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dc_stateful_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/paper4;

clean_stateless_multi_op_checksum_sample:
	@cd $(DC_PATH)/stateless_multi_op_checksum_sample && \
	    $(MAKE) clean ICP_OS_LEVEL=user_space &&         \
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean; 
	$(RM) $(DC_PATH)/stateless_multi_op_checksum_sample/dc_stateless_multi_op_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dc_stateless_multi_op_sample.ko;
	$(RM) $(DC_PATH)/stateless_multi_op_checksum_sample/dc_stateless_multi_op_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dc_stateless_multi_op_sample;


clean_algchaining_sample: 
	@cd $(SYM_PATH)/alg_chaining_sample && \
	    $(MAKE) clean ICP_OS_LEVEL=user_space && \
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/alg_chaining_sample/algchaining_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/algchaining_sample.ko;
	$(RM) $(SYM_PATH)/alg_chaining_sample/algchaining_sample ;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/algchaining_sample;

clean_ccm_sample: 
	@cd $(SYM_PATH)/ccm_sample && $(MAKE) clean ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/ccm_sample/ccm_sample.ko ;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/ccm_sample.ko;
	$(RM) $(SYM_PATH)/ccm_sample/ccm_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/ccm_sample;

clean_cipher_sample: 
	@cd $(SYM_PATH)/cipher_sample && $(MAKE) clean ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/cipher_sample/cipher_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/cipher_sample.ko;
	$(RM) $(SYM_PATH)/cipher_sample/cipher_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/cipher_sample;

clean_drbg_sample: 
	@cd $(SYM_PATH)/drbg_sample && $(MAKE) clean ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/drbg_sample/drbg_sample.ko; 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/drbg_sample.ko;
	$(RM) $(SYM_PATH)/drbg_sample/drbg_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/drbg_sample;

clean_gcm_sample: 
	@cd $(SYM_PATH)/gcm_sample && $(MAKE) clean ICP_OS_LEVEL=user_space && \
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/gcm_sample/gcm_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/gcm_sample.ko;
	$(RM) $(SYM_PATH)/gcm_sample/gcm_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/gcm_sample;

clean_hash_file_sample: 
	@cd $(SYM_PATH)/hash_file_sample && $(MAKE) clean ICP_OS_LEVEL=user_space; 
	$(RM) $(SYM_PATH)/hash_file_sample/hash_file_sample 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/hash_file_sample;

clean_hash_sample: 
	@cd $(SYM_PATH)/hash_sample && $(MAKE) clean ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean 
	$(RM) $(SYM_PATH)/hash_sample/hash_sample.ko; 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/hash_sample.ko;
	$(RM) $(SYM_PATH)/hash_sample/hash_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/hash_sample;

clean_ipsec_sample: 
	@cd $(SYM_PATH)/ipsec_sample && $(MAKE) clean  ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/ipsec_sample/ipsec_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/ipsec_sample.ko;
	$(RM) $(SYM_PATH)/ipsec_sample/ipsec_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/ipsec_sample;

clean_nrbg_sample: 
	@cd $(SYM_PATH)/nrbg_sample && $(MAKE) clean  ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/nrbg_sample/nrbg_sample; 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/nrbg_sample;
	$(RM) $(SYM_PATH)/nrbg_sample/nrbg_sample.ko; 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/nrbg_sample.ko;

clean_ssl_sample: 
	@cd $(SYM_PATH)/ssl_sample && $(MAKE) clean ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/ssl_sample/ssl_sample.ko; 
	$(RM) $(SAMPLE_BUILD_OUTPUT)/ssl_sample.ko;
	$(RM) $(SYM_PATH)/ssl_sample/ssl_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/ssl_sample;

clean_sym_dp_sample: 
	@cd $(SYM_PATH)/symdp_sample && $(MAKE) clean ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(SYM_PATH)/symdp_sample/sym_dp_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/sym_dp_sample.ko;
	$(RM) $(SYM_PATH)/symdp_sample/sym_dp_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/sym_dp_sample;


clean_diffie_hellman_sample: 
	@cd $(ASYM_PATH)/diffie_hellman_sample && $(MAKE) clean ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(ASYM_PATH)/diffie_hellman_sample/dh_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dh_sample.ko;
	$(RM) $(ASYM_PATH)/diffie_hellman_sample/dh_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/dh_sample;

clean_prime_sample: 
	@cd $(ASYM_PATH)/prime_sample && $(MAKE) clean ICP_OS_LEVEL=user_space &&\
	    $(MAKE) ICP_OS_LEVEL=kernel_space clean;
	$(RM) $(ASYM_PATH)/prime_sample/prime_sample.ko;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/prime_sample.ko;
	$(RM) $(ASYM_PATH)/prime_sample/prime_sample;
	$(RM) $(SAMPLE_BUILD_OUTPUT)/prime_sample;

RM=rm -f
DO_CRYPTO?=1
ifeq ($(DO_CRYPTO),1)
CLEAN_ALL = clean_dc clean_sym clean_asym
BUILD_FUNCIONAL_SAMPLE_CODE = dc sym asym
else
CLEAN_ALL = clean_dc
BUILD_FUNCIONAL_SAMPLE_CODE = dc 
endif

all: $(BUILD_FUNCIONAL_SAMPLE_CODE)

.PHONY: clean
clean: $(CLEAN_ALL)
	@echo; echo "Cleaning funcitonal sample code.";


