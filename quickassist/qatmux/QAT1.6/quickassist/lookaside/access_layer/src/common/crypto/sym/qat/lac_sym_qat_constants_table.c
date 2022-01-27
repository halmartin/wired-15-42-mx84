/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify 
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but 
 *   WITHOUT ANY WARRANTY; without even the implied warranty of 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License 
 *   along with this program; if not, write to the Free Software 
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution 
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE 
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without 
 *   modification, are permitted provided that the following conditions 
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided with the 
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *  version: QAT1.6.L.2.6.0-65
 *
 ***************************************************************************/

/**
 ***************************************************************************
 * @file lac_sym_qat_constants_table.c
 *
 * @ingroup LacSymQat
 ***************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/

#include "cpa.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/

#include "lac_common.h"
#include "icp_qat_fw_la.h"
#include "lac_log.h"
#include "lac_mem.h"
#include "icp_platform.h"
#include "sal_string_parse.h"
#include "lac_sal_types_crypto.h"
#include "icp_adf_cfg.h"

#define WIRELESS_ENABLED "WirelessEnabled"

/*
*******************************************************************************
* Static Tables
*
* These tables have been manually generated from dh895xcc_fw_interface.xlsx 
*  v0.15
*
* Please don't modify content below unilaterally. To keep in sync, always
* modify the spreadsheet first, regenerate the tables and copy in here.
*
*******************************************************************************
*/

STATIC uint32_t icp_qat_fw_crypto_constants[128][2] = {  /* IA version */
/* Offset: 0, length: 8B, RESERVED_ZERO_LW_OFFSET */ {0x00000000, 0x00000000},
/* Offset: 8, length: 64B, CFG_ZERO_ARRAY */ {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000},

/* Offset: 72, length: 8B, CFG_CIPH_DES_ECB_ENCRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_ECB_MODE, ICP_QAT_HW_CIPHER_ALGO_DES, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 80, length: 8B, CFG_CIPH_DES_ECB_DECRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_ECB_MODE, ICP_QAT_HW_CIPHER_ALGO_DES, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_DECRYPT),0},
/* Offset: 88, length: 8B, CFG_CIPH_DES_CBC_ENCRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_CBC_MODE, ICP_QAT_HW_CIPHER_ALGO_DES, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 96, length: 8B, CFG_CIPH_DES_CBC_DECRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_CBC_MODE, ICP_QAT_HW_CIPHER_ALGO_DES, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_DECRYPT),0},
/* Offset: 104, length: 8B, CFG_CIPH_DES_CTR_ENCRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_CTR_MODE, ICP_QAT_HW_CIPHER_ALGO_DES, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 112, length: 8B, CFG_CIPH_AES128_ECB_ENCRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_ECB_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 120, length: 8B, CFG_CIPH_AES128_ECB_ENCRYPT_KEY_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_ECB_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_KEY_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 128, length: 8B, CFG_CIPH_AES128_ECB_DECRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_ECB_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_DECRYPT),0},
/* Offset: 136, length: 8B, CFG_CIPH_AES128_ECB_DECRYPT_KEY_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_ECB_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_KEY_CONVERT, ICP_QAT_HW_CIPHER_DECRYPT),0},
/* Offset: 144, length: 8B, CFG_CIPH_AES128_CBC_ENCRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_CBC_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 152, length: 8B, CFG_CIPH_AES128_CBC_ENCRYPT_KEY_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_CBC_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_KEY_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 160, length: 8B, CFG_CIPH_AES128_CBC_DECRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_CBC_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_DECRYPT),0},
/* Offset: 168, length: 8B, CFG_CIPH_AES128_CBC_DECRYPT_KEY_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_CBC_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_KEY_CONVERT, ICP_QAT_HW_CIPHER_DECRYPT),0},
/* Offset: 176, length: 8B, CFG_CIPH_AES128_CTR_ENCRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_CTR_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 184, length: 8B, CFG_CIPH_AES128_F8_ENCRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_F8_MODE, ICP_QAT_HW_CIPHER_ALGO_AES128, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 192, length: 8B, CFG_CIPH_ARC4_ECB_ENCRYPT_NO_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_ECB_MODE, ICP_QAT_HW_CIPHER_ALGO_ARC4, ICP_QAT_HW_CIPHER_NO_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 200, length: 8B, CFG_CIPH_ARC4_ECB_ENCRYPT_KEY_CONVERT */ {ICP_QAT_HW_CIPHER_CONFIG_BUILD(ICP_QAT_HW_CIPHER_ECB_MODE, ICP_QAT_HW_CIPHER_ALGO_ARC4, ICP_QAT_HW_CIPHER_KEY_CONVERT, ICP_QAT_HW_CIPHER_ENCRYPT),0},
/* Offset: 208, length: 88B, CFG_ZERO_ARRAY */ {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000},

/* Offset: 296, length: 32B, CFG_AUTH_MD5_MODE0_NO_NESTED */ {ICP_QAT_HW_AUTH_CONFIG_BUILD(ICP_QAT_HW_AUTH_MODE0, ICP_QAT_HW_AUTH_ALGO_MD5, 0),0}, {0x00000000, 0x00000000}, {0x67452301, 0xefcdab89}, {0x98badcfe, 0x10325476},
/* Offset: 328, length: 40B, CFG_AUTH_SHA1_MODE0_NO_NESTED */ {ICP_QAT_HW_AUTH_CONFIG_BUILD(ICP_QAT_HW_AUTH_MODE0, ICP_QAT_HW_AUTH_ALGO_SHA1, 0),0}, {0x00000000, 0x00000000}, {0x01234567, 0x89abcdef}, {0xfedcba98, 0x76543210}, {0xf0e1d2c3, 0x00000000},
/* Offset: 368, length: 16B, CFG_AUTH_SHA1_MODE1_NO_NESTED */ {ICP_QAT_HW_AUTH_CONFIG_BUILD(ICP_QAT_HW_AUTH_MODE1, ICP_QAT_HW_AUTH_ALGO_SHA1, 0),0}, {0x40000000, 0x00000000},
/* Offset: 384, length: 48B, CFG_AUTH_SHA224_MODE0_NO_NESTED */ {ICP_QAT_HW_AUTH_CONFIG_BUILD(ICP_QAT_HW_AUTH_MODE0, ICP_QAT_HW_AUTH_ALGO_SHA224, 0),0}, {0x00000000, 0x00000000}, {0xd89e05c1, 0x07d57c36}, {0x17dd7030, 0x39590ef7}, {0x310bc0ff, 0x11155868}, {0xa78ff964, 0xa44ffabe},
/* Offset: 432, length: 48B, CFG_AUTH_SHA256_MODE0_NO_NESTED */ {ICP_QAT_HW_AUTH_CONFIG_BUILD(ICP_QAT_HW_AUTH_MODE0, ICP_QAT_HW_AUTH_ALGO_SHA256, 0),0}, {0x00000000, 0x00000000}, {0x67e6096a, 0x85ae67bb}, {0x72f36e3c, 0x3af54fa5}, {0x7f520e51, 0x8c68059b}, {0xabd9831f, 0x19cde05b},
/* Offset: 480, length: 80B, CFG_AUTH_SHA384_MODE0_NO_NESTED */ {ICP_QAT_HW_AUTH_CONFIG_BUILD(ICP_QAT_HW_AUTH_MODE0, ICP_QAT_HW_AUTH_ALGO_SHA384, 0),0}, {0x00000000, 0x00000000}, {0x5d9dbbcb, 0xd89e05c1}, {0x2a299a62, 0x07d57c36}, {0x5a015991, 0x17dd7030}, {0xd8ec2f15, 0x39590ef7}, {0x67263367, 0x310bc0ff}, {0x874ab48e, 0x11155868}, {0x0d2e0cdb, 0xa78ff964}, {0x1d48b547, 0xa44ffabe},
/* Offset: 560, length: 80B, CFG_AUTH_SHA512_MODE0_NO_NESTED */ {ICP_QAT_HW_AUTH_CONFIG_BUILD(ICP_QAT_HW_AUTH_MODE0, ICP_QAT_HW_AUTH_ALGO_SHA512, 0),0}, {0x00000000, 0x00000000}, {0x67e6096a, 0x08c9bcf3}, {0x85ae67bb, 0x3ba7ca84}, {0x72f36e3c, 0x2bf894fe}, {0x3af54fa5, 0xf1361d5f}, {0x7f520e51, 0xd182e6ad}, {0x8c68059b, 0x1f6c3e2b}, {0xabd9831f, 0x6bbd41fb}, {0x19cde05b, 0x79217e13},
/* Offset: 640, length: 384B, CFG_ZERO_ARRAY */ {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}, {0x00000000, 0x00000000}};

STATIC uint8_t icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_DELIMITER][ICP_QAT_HW_CIPHER_MODE_DELIMITER][2][2] ; /* IA version */
STATIC uint8_t icp_qat_hw_auth_lookup_tbl[ICP_QAT_HW_AUTH_ALGO_DELIMITER][ICP_QAT_HW_AUTH_MODE_DELIMITER][2]; /* IA version */



#define ICP_QAT_HW_FILL_LOOKUP_TBLS { \
\
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_DES][ICP_QAT_HW_CIPHER_ECB_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 9; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_DES][ICP_QAT_HW_CIPHER_ECB_MODE][ICP_QAT_HW_CIPHER_DECRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 10; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_DES][ICP_QAT_HW_CIPHER_CBC_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 11; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_DES][ICP_QAT_HW_CIPHER_CBC_MODE][ICP_QAT_HW_CIPHER_DECRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 12; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_DES][ICP_QAT_HW_CIPHER_CTR_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 13; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_ECB_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 14; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_ECB_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_KEY_CONVERT] = 15; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_ECB_MODE][ICP_QAT_HW_CIPHER_DECRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 16; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_ECB_MODE][ICP_QAT_HW_CIPHER_DECRYPT][ICP_QAT_HW_CIPHER_KEY_CONVERT] = 17; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_CBC_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 18; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_CBC_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_KEY_CONVERT] = 19; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_CBC_MODE][ICP_QAT_HW_CIPHER_DECRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 20; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_CBC_MODE][ICP_QAT_HW_CIPHER_DECRYPT][ICP_QAT_HW_CIPHER_KEY_CONVERT] = 21; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_CTR_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 22; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_AES128][ICP_QAT_HW_CIPHER_F8_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 23; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_ARC4][ICP_QAT_HW_CIPHER_ECB_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_NO_CONVERT] = 24; \
icp_qat_hw_cipher_lookup_tbl[ICP_QAT_HW_CIPHER_ALGO_ARC4][ICP_QAT_HW_CIPHER_ECB_MODE][ICP_QAT_HW_CIPHER_ENCRYPT][ICP_QAT_HW_CIPHER_KEY_CONVERT] = 25; \
\
\
icp_qat_hw_auth_lookup_tbl[ICP_QAT_HW_AUTH_ALGO_MD5][ICP_QAT_HW_AUTH_MODE0][ICP_QAT_FW_AUTH_HDR_FLAG_NO_NESTED] = 37; \
icp_qat_hw_auth_lookup_tbl[ICP_QAT_HW_AUTH_ALGO_SHA1][ICP_QAT_HW_AUTH_MODE0][ICP_QAT_FW_AUTH_HDR_FLAG_NO_NESTED] = 41; \
icp_qat_hw_auth_lookup_tbl[ICP_QAT_HW_AUTH_ALGO_SHA1][ICP_QAT_HW_AUTH_MODE1][ICP_QAT_FW_AUTH_HDR_FLAG_NO_NESTED] = 46; \
icp_qat_hw_auth_lookup_tbl[ICP_QAT_HW_AUTH_ALGO_SHA224][ICP_QAT_HW_AUTH_MODE0][ICP_QAT_FW_AUTH_HDR_FLAG_NO_NESTED] = 48; \
icp_qat_hw_auth_lookup_tbl[ICP_QAT_HW_AUTH_ALGO_SHA256][ICP_QAT_HW_AUTH_MODE0][ICP_QAT_FW_AUTH_HDR_FLAG_NO_NESTED] = 54; \
icp_qat_hw_auth_lookup_tbl[ICP_QAT_HW_AUTH_ALGO_SHA384][ICP_QAT_HW_AUTH_MODE0][ICP_QAT_FW_AUTH_HDR_FLAG_NO_NESTED] = 60; \
icp_qat_hw_auth_lookup_tbl[ICP_QAT_HW_AUTH_ALGO_SHA512][ICP_QAT_HW_AUTH_MODE0][ICP_QAT_FW_AUTH_HDR_FLAG_NO_NESTED] = 70; \
}


/*
*******************************************************************************
*
* End of section copied from excel file
*
*******************************************************************************
*/



/**
 *****************************************************************************
 * @ingroup LacSymQat
 *      LacSymQat_ConstantsGetConfig()
 *
 * @description
 *
 *
 * @param[in]    device
 *
 * @retval CpaBoolean
 *
 *****************************************************************************/
STATIC CpaBoolean
LacSymQat_ConstantsGetConfig(icp_accel_dev_t* device)
{
    CpaStatus status;
    CpaBoolean use_table = CPA_TRUE;
    char    paramValue[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    LAC_ENSURE_NOT_NULL(device);

    /* By default the constants table is in use, but under
     * certain configuration it can be disabled */

    status = icp_adf_cfgGetParamValue(device,
            LAC_CFG_SECTION_GENERAL,
            "Disable_shram_constants_table",
            paramValue);

    if (status == CPA_STATUS_SUCCESS &&
            (Sal_Strtoul(paramValue, NULL, SAL_CFG_BASE_DEC)> 0 ))
    {
        LAC_LOG("Configured to not use SHRAM Sym Constants Table.");
        use_table = CPA_FALSE;
    }

    if (use_table == CPA_TRUE)
    {
        status = icp_adf_cfgGetParamValue(device,
                LAC_CFG_SECTION_GENERAL,
                WIRELESS_ENABLED,
                paramValue);

        if (CPA_STATUS_SUCCESS == status &&
                (Sal_Strtoul(paramValue, NULL, SAL_CFG_BASE_DEC)> 0 ))
        {
            LAC_LOG_DEBUG("Configured for wireless so not using "
                    "SHRAM Sym Constants Table.");
            use_table = CPA_FALSE;
        }
    }

    if (use_table)
    {
        LAC_LOG_DEBUG("SHRAM SymConstantsTable is in use.");
    }

    return use_table;
}

/**
 *****************************************************************************
 * @ingroup LacSymQat
 *      LacSymQat_ConstantsEnabled()
 *
 *
 * @retval CpaBoolean
 *
 *****************************************************************************/
CpaBoolean
LacSymQat_ConstantsEnabled(CpaInstanceHandle instanceHandle)
{
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;
    LAC_ENSURE_NOT_NULL(pService);

    return pService->constantsLookupTables.useConstantsTable;
}


/**
 *****************************************************************************
 * @ingroup LacSymQat
 *      LacSymQat_ConstantsInitLookupTables
 *
 *
 *****************************************************************************/
void
LacSymQat_ConstantsInitLookupTables(CpaInstanceHandle instanceHandle,
                                    icp_accel_dev_t* device)
{
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;
    LAC_ENSURE_NOT_NULL(pService);
    LAC_ENSURE_NOT_NULL(device);

    if (LacSymQat_ConstantsGetConfig(device) == CPA_TRUE)
    {
        pService->constantsLookupTables.useConstantsTable = CPA_TRUE;

        /* Note the global tables are initialised first, then copied
         * to the service which probably seems like a waste of memory
         * and processing cycles as the global tables are never needed again
         * but this allows use of the ICP_QAT_HW_FILL_LOOKUP_TBLS macro
         * supplied by FW without modification */

        /* First fill the global lookup tables with zeroes. */
        ICP_MEMSET(icp_qat_hw_cipher_lookup_tbl,
                        0,sizeof(icp_qat_hw_cipher_lookup_tbl));
        ICP_MEMSET(icp_qat_hw_auth_lookup_tbl,
                        0,sizeof(icp_qat_hw_auth_lookup_tbl));


        /* Override lookup tables with the offsets into the SHRAM table
         * for supported algorithms/modes */
       ICP_QAT_HW_FILL_LOOKUP_TBLS;

       /* Copy the global tables to the service instance */
       ICP_MEMCPY(pService->constantsLookupTables.cipher_offset,
               icp_qat_hw_cipher_lookup_tbl,
               sizeof(pService->constantsLookupTables.cipher_offset));
       ICP_MEMCPY(pService->constantsLookupTables.auth_offset,
               icp_qat_hw_auth_lookup_tbl,
               sizeof(pService->constantsLookupTables.auth_offset));

    }
    else
    {
        pService->constantsLookupTables.useConstantsTable = CPA_FALSE;
        /* Zero the service instance tables */
        ICP_MEMSET(pService->constantsLookupTables.cipher_offset,
                     0,sizeof(pService->constantsLookupTables.cipher_offset));
        ICP_MEMSET(pService->constantsLookupTables.auth_offset,
                     0,sizeof(pService->constantsLookupTables.auth_offset));
    }

}


/**
 *****************************************************************************
 * @ingroup LacSymQat
 *      LacSymQat_ConstantsGetCipherOffset
 *
 *
 *****************************************************************************/
void
LacSymQat_ConstantsGetCipherOffset(CpaInstanceHandle instanceHandle,
                                    uint8_t algo,
                                    uint8_t mode,
                                    uint8_t direction,
                                    uint8_t convert,
                                    uint8_t *poffset )
{
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;

    LAC_ENSURE_RANGE(algo,0,ICP_QAT_HW_CIPHER_DELIMITER);
    LAC_ENSURE_RANGE(mode,0,ICP_QAT_HW_CIPHER_MODE_DELIMITER);
    LAC_ENSURE_RANGE(direction,0,2);
    LAC_ENSURE_RANGE(convert,0,2);
    LAC_ENSURE_NOT_NULL(poffset);
    LAC_ENSURE_NOT_NULL(pService);

    *poffset = pService->constantsLookupTables.cipher_offset[algo][mode][direction][convert];

    LAC_LOG_DEBUG2("cipher constants offset (quad_words) = %d (0x%x)",
                                            *poffset, *poffset );

}

/**
 *****************************************************************************
 * @ingroup LacSymQat
 *      LacSymQat_ConstantsGetAuthOffset
 *
 *
 *****************************************************************************/
void
LacSymQat_ConstantsGetAuthOffset(CpaInstanceHandle instanceHandle,
                                    uint8_t algo,
                                    uint8_t mode,
                                    uint8_t nested,
                                    uint8_t *poffset )
{
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;

    LAC_ENSURE_RANGE(algo,0,ICP_QAT_HW_AUTH_ALGO_DELIMITER);
    LAC_ENSURE_RANGE(mode,0,ICP_QAT_HW_AUTH_MODE_DELIMITER);
    LAC_ENSURE_RANGE(nested,0,2);
    LAC_ENSURE_NOT_NULL(poffset);
    LAC_ENSURE_NOT_NULL(pService);


   *poffset = pService->constantsLookupTables.auth_offset[algo][mode][nested];

   LAC_LOG_DEBUG2("auth constants offset (quad_words) = %d (0x%x)",
                                           *poffset, *poffset );

}

/**
 *****************************************************************************
 * Local Data
 *
 *****************************************************************************/
Cpa32U *pSymConstantsTable;
Cpa64U  pSymConstantsTablePhys;

/**
 *****************************************************************************
 * @ingroup LacSymQat
 *      LacSymQat_ConstantsInitTableForFW
 *
 *
 *****************************************************************************/
CpaStatus
LacSymQat_ConstantsInitTableForFW(icp_accel_dev_t* device,
                                  CpaBoolean *pUseTable)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    *pUseTable = CPA_FALSE;

    if (LacSymQat_ConstantsGetConfig(device) == CPA_TRUE)
    {
        pSymConstantsTable = device->pShramConstants;
        pSymConstantsTablePhys = device->pShramConstantsDma;

        if ((NULL != pSymConstantsTable) && (0 != pSymConstantsTablePhys))
        {
            ICP_MEMCPY(pSymConstantsTable,
                    icp_qat_fw_crypto_constants,
                    sizeof(icp_qat_fw_crypto_constants));
                    *pUseTable = CPA_TRUE;
        }
        else
        {
            pSymConstantsTable = NULL;
            LAC_LOG_ERROR("Error allocating memory for SymConstantsTable");
            status = CPA_STATUS_FAIL;
        }
    }
    else
    {
        pSymConstantsTable = NULL;
        LAC_LOG_DEBUG("Use of SymConstantsTable disabled");
    }

    return status;
}

/**
 *****************************************************************************
 * @ingroup LacSymQat
 *      LacSymQat_ConstantsReleaseTableForFW
 *
 *
 *****************************************************************************/
void
LacSymQat_ConstantsReleaseTableForFW(void)
{

    /* The Shram Constants table is freed by ADF*/
    pSymConstantsTable = NULL;
    pSymConstantsTablePhys = 0;
}

/**
 *****************************************************************************
 * @ingroup LacSymQat
 *      LacSymQat_ConstantsGetTableInfo
 *
 *
 *****************************************************************************/
void
LacSymQat_ConstantsGetTableInfo(Cpa64U *pTable, Cpa16U *pSizeInBytes)
{

    LAC_ENSURE_NOT_NULL(pTable);
    LAC_ENSURE_NOT_NULL(pSizeInBytes);

    *pTable = pSymConstantsTablePhys;

    *pSizeInBytes = sizeof(icp_qat_fw_crypto_constants);


}

