/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2014 Intel Corporation.
  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  qat-linux@intel.com

  BSD LICENSE
  Copyright(c) 2014 Intel Corporation.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _ICP_QAT_FW_INIT_ADMIN_H_
#define _ICP_QAT_FW_INIT_ADMIN_H_

#include "icp_qat_fw.h"

enum icp_qat_fw_init_admin_cmd_id {
	ICP_QAT_FW_INIT_ME = 0,
	ICP_QAT_FW_TRNG_ENABLE = 1,
	ICP_QAT_FW_TRNG_DISABLE = 2,
	ICP_QAT_FW_CONSTANTS_CFG = 3,
	ICP_QAT_FW_STATUS_GET = 4,
	ICP_QAT_FW_COUNTERS_GET = 5,
	ICP_QAT_FW_LOOPBACK = 6,
	ICP_QAT_FW_HEARTBEAT_SYNC = 7,
	ICP_QAT_FW_HEARTBEAT_GET = 8,
	ICP_QAT_FW_COMP_CAPABILITY_GET = 9,
	ICP_QAT_FW_CRYPTO_CAPABILITY_GET = 10,
	ICP_QAT_FW_HEARTBEAT_TIMER_SET = 13,
	ICP_QAT_FW_TIMER_GET = 19,
	ICP_QAT_FW_CNV_STATS_GET = 20
};

enum icp_qat_fw_init_admin_resp_status {
	ICP_QAT_FW_INIT_RESP_STATUS_SUCCESS = 0,
	ICP_QAT_FW_INIT_RESP_STATUS_FAIL = 1,
	ICP_QAT_FW_INIT_RESP_STATUS_UNSUPPORTED = 4
};

enum icp_qat_fw_cnv_error_type {
	CNV_ERR_TYPE_NO_ERROR = 0,
	CNV_ERR_TYPE_CHECKSUM_ERROR,
	CNV_ERR_TYPE_DECOMP_PRODUCED_LENGTH_ERROR,
	CNV_ERR_TYPE_DECOMPRESSION_ERROR,
	CNV_ERR_TYPE_TRANSLATION_ERROR,
	CNV_ERR_TYPE_DECOMP_CONSUMED_LENGTH_ERROR,
	CNV_ERR_TYPE_UNKNOWN_ERROR
};

#define CNV_ERROR_TYPE_GET(latest_error)	\
	({__typeof__(latest_error) _lerror = latest_error;	\
	(_lerror >> 12)	> CNV_ERR_TYPE_UNKNOWN_ERROR	\
	? CNV_ERR_TYPE_UNKNOWN_ERROR	\
	: (enum icp_qat_fw_cnv_error_type)(_lerror >> 12); })
#define CNV_ERROR_LENGTH_DELTA_GET(latest_error)	\
	({__typeof__(latest_error) _lerror = latest_error;	\
	((s16)((_lerror & 0x0FFF)	\
	| (_lerror & 0x0800 ? 0xF000 : 0))); })
#define CNV_ERROR_DECOMP_STATUS_GET(latest_error) ((s8)(latest_error & 0xFF))

struct icp_qat_fw_init_admin_req {
	u16 init_cfg_sz;
	u8 resrvd1;
	u8 cmd_id;
	u32 resrvd2;
	u64 opaque_data;
	u64 init_cfg_ptr;

	union {
		/* ICP_QAT_FW_INIT_ME */
		struct {
			u16 ibuf_size_in_kb;
			u16 resrvd3;
		};
		/* ICP_QAT_FW_HEARTBEAT_TIMER_SET */
		u32 heartbeat_ticks;
	};

	u32 resrvd4;
} __packed;

struct icp_qat_fw_init_admin_resp {
	u8 flags;
	u8 resrvd1;
	u8 status;
	u8 cmd_id;
	union {
		u32 resrvd2;
		/* ICP_QAT_FW_STATUS_GET */
		struct {
			u16 version_minor_num;
			u16 version_major_num;
		};
		/* ICP_QAT_FW_COMP_CAPABILITY_GET */
		u32 extended_features;
		/* ICP_QAT_FW_CNV_STATS_GET */
		struct {
			u16 error_count;
			u16 latest_error;
		};
	};
	u64 opaque_data;
	union {
		u32 resrvd3[4];
		/* ICP_QAT_FW_STATUS_GET */
		struct {
			u32 version_patch_num;
			u8 context_id;
			u8 ae_id;
			u16 resrvd4;
			u64 resrvd5;
		};
		/* ICP_QAT_FW_COMP_CAPABILITY_GET */
		struct {
			u16    compression_algos;
			u16    checksum_algos;
			u32    deflate_capabilities;
			u32    resrvd6;
			u32    lzs_capabilities;
		};
		/* ICP_QAT_FW_CRYPTO_CAPABILITY_GET */
		struct {
			u32    cipher_algos;
			u32    hash_algos;
			u16    keygen_algos;
			u16    other;
			u16    public_key_algos;
			u16    prime_algos;
		};
		/* ICP_QAT_FW_TIMER_GET  */
		struct {
			u64 timestamp;
			u64 resrvd7;
		};
		struct {
			u64 req_rec_count;
			u64 resp_sent_count;
		};
	};
} __packed;

enum icp_qat_fw_init_admin_init_flag {
	ICP_QAT_FW_INIT_FLAG_PKE_DISABLED = 0
};

struct icp_qat_fw_init_admin_hb_cnt {
	u16 resp_heartbeat_cnt;
	u16 req_heartbeat_cnt;
};

struct icp_qat_fw_init_admin_hb_stats {
	struct icp_qat_fw_init_admin_hb_cnt stats[ADF_NUM_HB_CNT_PER_AE];
};

#define ICP_QAT_FW_COMN_HEARTBEAT_OK 0
#define ICP_QAT_FW_COMN_HEARTBEAT_BLOCKED 1
#define ICP_QAT_FW_COMN_HEARTBEAT_FLAG_BITPOS 0
#define ICP_QAT_FW_COMN_HEARTBEAT_FLAG_MASK 0x1
#define ICP_QAT_FW_COMN_STATUS_RESRVD_FLD_MASK 0xFE
#define ICP_QAT_FW_COMN_HEARTBEAT_HDR_FLAG_GET(hdr_t) \
	ICP_QAT_FW_COMN_HEARTBEAT_FLAG_GET(hdr_t.flags)

#define ICP_QAT_FW_COMN_HEARTBEAT_HDR_FLAG_SET(hdr_t, val) \
	ICP_QAT_FW_COMN_HEARTBEAT_FLAG_SET(hdr_t, val)

#define ICP_QAT_FW_COMN_HEARTBEAT_FLAG_GET(flags) \
	QAT_FIELD_GET(flags, \
		 ICP_QAT_FW_COMN_HEARTBEAT_FLAG_BITPOS, \
		 ICP_QAT_FW_COMN_HEARTBEAT_FLAG_MASK)
#endif
