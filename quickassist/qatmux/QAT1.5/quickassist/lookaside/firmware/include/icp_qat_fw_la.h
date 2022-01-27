/*
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
 *  version: QAT1.5.L.1.11.0-36
 */

/**
 *****************************************************************************
 * @file icp_qat_fw_la.h
 * @defgroup icp_qat_fw_la ICP QAT FW Lookaside Service Interface Definitions
 * @ingroup icp_qat_fw
 * @description
 *      This file documents structs used to provided the interface to the
 *      LookAside (LA) QAT FW service
 *
 *****************************************************************************/

#ifndef __ICP_QAT_FW_LA_H__
#define __ICP_QAT_FW_LA_H__

/*
******************************************************************************
* Include local header files
******************************************************************************
*/
#include "icp_qat_fw.h"


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the LookAside (LA) command types
 * @description
 *        Enumeration which is used to indicate the ids of functions
 *              that are exposed by the LA QAT FW service
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_FW_LA_CMD_CIPHER=0,
   /*!< Cipher Request */

   ICP_QAT_FW_LA_CMD_AUTH=1,
   /*!< Auth Request */

   ICP_QAT_FW_LA_CMD_CIPHER_HASH=2,
   /*!< Cipher-Hash Request */

   ICP_QAT_FW_LA_CMD_HASH_CIPHER=3,
   /*!< Hash-Cipher Request */

   ICP_QAT_FW_LA_CMD_TRNG_GET_RANDOM=4,
   /*!< TRNG Get Random Request */

   ICP_QAT_FW_LA_CMD_TRNG_TEST=5,
   /*!< TRNG Test Request */

   ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE=6,
   /*!< SSL3 Key Derivation Request */

   ICP_QAT_FW_LA_CMD_TLS_V1_1_KEY_DERIVE=7,
   /*!< TLS Key Derivation Request */

   ICP_QAT_FW_LA_CMD_TLS_V1_2_KEY_DERIVE=8,
   /*!< TLS Key Derivation Request */

   ICP_QAT_FW_LA_CMD_MGF1=9,
   /*!< MGF1 Request */

   ICP_QAT_FW_LA_CMD_AUTH_PRE_COMP=10,
   /*!< Auth Pre-Compute Request */

   ICP_QAT_FW_LA_CMD_CIPHER_CIPHER=11,
   /*!< Cipher-Cipher Request */

   ICP_QAT_FW_LA_CMD_HASH_HASH=12,
   /*!< Hash-Hash Request */
 
   ICP_QAT_FW_LA_CMD_CIPHER_PRE_COMP=13,

   /*!< Auth Pre-Compute Request */
   ICP_QAT_FW_LA_CMD_DELIMITER=14
   /**< Delimiter type */

} icp_qat_fw_la_cmd_id_t;



/*  For the definitions of the bits in the status field of the common 
 *  response, refer to icp_qat_fw.h.
 *  The return values specific to Lookaside service are given below.
 */
#define ICP_QAT_FW_LA_ICV_VER_STATUS_PASS      ICP_QAT_FW_COMN_STATUS_FLAG_OK
/**< @ingroup icp_qat_fw_la
 * Status flag indicating that the ICV verification passed */

#define ICP_QAT_FW_LA_ICV_VER_STATUS_FAIL      ICP_QAT_FW_COMN_STATUS_FLAG_ERROR
/**< @ingroup icp_qat_fw_la
 * Status flag indicating that the ICV verification failed */

#define ICP_QAT_FW_LA_TRNG_STATUS_PASS         ICP_QAT_FW_COMN_STATUS_FLAG_OK
/**< @ingroup icp_qat_fw_la
 * Status flag indicating that the TRNG returned valid entropy data */

#define ICP_QAT_FW_LA_TRNG_STATUS_FAIL         ICP_QAT_FW_COMN_STATUS_FLAG_ERROR
/**< @ingroup icp_qat_fw_la
 * Status flag indicating that the TRNG Command Failed. */


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the common fields for TLS/SSL    
 * @description
 *        Used for TLS/SSL Keygen commands to set the label and output length 
 *    fields 
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_ssl_tls_common_s
{
   uint8_t out_len;
   /**< Number of bytes of key material to output. */

   uint8_t label_len;
   /**< Number of bytes of label for SSL and bytes for TLS key generation  */

}icp_qat_fw_la_ssl_tls_common_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the common length fields for MGF Key Generation     
 * @description
 *        Used for MGF Keygen commands to set the Seed and Hash length fields. 
 *    fields 
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_mgf_common_s
{    
   uint8_t hash_len;
   /**< Number of bytes of hash output by the QAT per iteration */

   uint8_t seed_len;
   /**< Number of bytes of seed provided in src buffer for MGF1 */

}icp_qat_fw_la_mgf_common_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the common LA QAT FW request descriptor parameters
 * @description
 *        This part of the request is common across all of the LA commands
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_comn_req_s
{
   union 
   {
       uint16_t la_flags;
       /**< Definition of the common LA processing flags used for the bulk
        * processing */

       union 
       {
            icp_qat_fw_la_ssl_tls_common_t ssl_tls_common; 
            /**< For TLS or SSL Key Generation, this field is overloaded with 
            ssl_tls common information */
            
            icp_qat_fw_la_mgf_common_t mgf_common; 
            /**< For MGF Key Generation, this field is overloaded with mgf 
            information */

       } u;

   } u;

   union 
   {
       uint8_t resrvd;
       /**< If not useRd by a request this field must be set to 0 */

	   uint8_t tls_seed_len;
       /**<  Byte Len of tls seed */

       uint8_t req_params_blk_sz;
       /**<  For bulk processing this field represents the request parameters
        * block size */

       uint8_t trng_cfg_sz;
       /**<  This field is used for TRNG_ENABLE requests to indicate the size
       of the TRNG Slice configuration word. Size is in QW's */

   } u1;

   uint8_t la_cmd_id;
   /**< Definition of the LA command defined by this request */

} icp_qat_fw_la_comn_req_t;


/*
 *  REQUEST FLAGS IN COMMON LA FLAGS
 *
 *  + ===== + ---------- + ----- + ---- + ------ + ---- + ---- + ----- + ---- +
 *  |  Bit  |   [15:10]  |   9   | 8-6  |   5    |  4   |  3   |   2   |  1-0 |
 *  + ===== + ---------- + ----- + ---- + ------ + ---- + ---- + ------+ ---- +
 *  | Flags | Resvd Bits | GcmIv | Proto| Digest | Cmp  | Ret  |  Upd  | Part-|
 *  |       |     =0     |  Len  | Type | in Buff| Auth | Auth | State |  ial |
 *  + ===== + ---------- + ----- + ---- + ------ + ---- + ---- + ------+ ---- +
 */
/* Private defines */
#define ICP_QAT_FW_LA_GCM_IV_LEN_12_OCTETS                             1
/**< @ingroup icp_qat_fw_la
 * Indicates the IV Length for GCM protocol is 96 Bits (12 Octets) 
 * If set FW does the padding to compute CTR0 */

#define ICP_QAT_FW_LA_GCM_IV_LEN_NOT_12_OCTETS                         0
/**< @ingroup icp_qat_fw_la
 * Indicates the IV Length for GCM protocol is not 96 Bits (12 Octets) 
 * If IA computes CTR0 */

#define ICP_QAT_FW_LA_SNOW_3G_PROTO                                    4
/**< @ingroup icp_cpm_fw_la
 * Indicates SNOW_3G processing for a encrypt command */

#define ICP_QAT_FW_LA_GCM_PROTO                                        2
/**< @ingroup icp_qat_fw_la
 * Indicates GCM processing for a auth_encrypt command */

#define ICP_QAT_FW_LA_CCM_PROTO                                        1
/**< @ingroup icp_qat_fw_la
 * Indicates CCM processing for a auth_encrypt command */

#define ICP_QAT_FW_LA_NO_PROTO                                         0
/**< @ingroup icp_qat_fw_la
 * Indicates no specific protocol processing for the command */

#define ICP_QAT_FW_LA_CMP_AUTH_RES                                     1
/**< @ingroup icp_qat_fw_la
 * Flag representing the need to compare the auth result data to the expected
 * value in DRAM at the auth_address. */

#define ICP_QAT_FW_LA_NO_CMP_AUTH_RES                                  0
/**< @ingroup icp_qat_fw_la
 * Flag representing that there is no need to do a compare of the auth data
 * to the expected value */

#define ICP_QAT_FW_LA_RET_AUTH_RES                                     1
/**< @ingroup icp_qat_fw_la
 * Flag representing the need to return the auth result data to dram after the
 * request processing is complete */

#define ICP_QAT_FW_LA_NO_RET_AUTH_RES                                  0
/**< @ingroup icp_qat_fw_la
 * Flag representing that there is no need to return the auth result data */

#define ICP_QAT_FW_LA_DIGEST_IN_BUFFER                                 1
/**< @ingroup icp_qat_fw_la
 * Flag representing that authentication digest is stored or is extracted
 * from the source buffer. Auth Result Pointer will be ignored in this case. */
 
#define ICP_QAT_FW_LA_NO_DIGEST_IN_BUFFER                              0
/**< @ingroup icp_qat_fw_la
 * Flag representing that authentication digest is NOT stored or is NOT 
 * extracted from the source buffer. Auth result will get stored or extracted 
 * from the Auth Result Pointer. Please not that in this case digest CANNOT be
 * encrypted. */

#define ICP_QAT_FW_LA_UPDATE_STATE                                     1
/**< @ingroup icp_qat_fw_la
 * Flag representing the need to update the state data in dram after the
 * request processing is complete */

#define ICP_QAT_FW_LA_NO_UPDATE_STATE                                  0
/**< @ingroup icp_qat_fw_la
 * Flag representing that there is no need to update the state data */

#define ICP_QAT_FW_LA_PARTIAL_NONE                                     0
/**< @ingroup icp_qat_fw_la
 * Flag representing no need for partial processing condition */

#define ICP_QAT_FW_LA_PARTIAL_START                                    1
/**< @ingroup icp_qat_fw_la
 * Flag representing the first chunk of the partial packet */

#define ICP_QAT_FW_LA_PARTIAL_MID                                      3
/**< @ingroup icp_qat_fw_la
 * Flag representing a middle chunk of the partial packet */

#define ICP_QAT_FW_LA_PARTIAL_END                                      2
/**< @ingroup icp_qat_fw_la
 * Flag representing the final/end chunk of the partial packet */

/* The table below defines the meaning of the prefix_addr & hash_state_sz in
 * the case of partial processing. See the HLD for further details
 *
 *  + ====== + ------------------------- + ----------------------- +
 *  | Parial |       Prefix Addr         |       Hash State Sz     |
 *  | State  |                           |                         |
 *  + ====== + ------------------------- + ----------------------- +
 *  |  FULL  | Points to the prefix data | Prefix size as below.   |
 *  |        |                           | No update of state      |
 *  + ====== + ------------------------- + ----------------------- +
 *  |  SOP   | Points to the prefix      | = inner prefix rounded  |
 *  |        | data. State is updated    | to qwrds + outer prefix |
 *  |        | at prefix_addr - state_sz | rounded to qwrds. The   |
 *  |        | - 8 (counter size)        | writeback state sz      |
 *  |        |                           | comes from the CD       |
 *  + ====== + ------------------------- + ----------------------- +
 *  |  MOP   | Points to the state data  | State size rounded to   |
 *  |        | Updated state written to  | num qwrds + 8 (for the  |
 *  |        | same location             | counter) + inner prefix |
 *  |        |                           | rounded to qwrds +      |
 *  |        |                           | outer prefix rounded to |
 *  |        |                           | qwrds.                  |
 *  + ====== + ------------------------- + ----------------------- +
 *  |  EOP   | Points to the state data  | State size rounded to   |
 *  |        |                           | num qwrds + 8 (for the  |
 *  |        |                           | counter) + inner prefix |
 *  |        |                           | rounded to qwrds +      |
 *  |        |                           | outer prefix rounded to |
 *  |        |                           | qwrds.                  |
 *  + ====== + ------------------------- + ----------------------- +
 *
 *  Notes:
 *
 *  - If the EOP is set it is assumed that no state update is to be performed.
 *    However it is the clients responsibility to set the update_state flag
 *    correctly i.e. not set for EOP or Full packet cases. Only set for SOP and
 *    MOP with no EOP flag
 *  - The SOP take precedence over the MOP and EOP i.e. in the calculation of
 *    the address to writeback the state.
 *  - The prefix address must be on at least the 8 byte boundary
 */

/* Private defines */
#define QAT_LA_GCM_IV_LEN_FLAG_BITPOS                          9
/**< @ingroup icp_qat_fw_la
 * Starting bit position for GCM IV Length indication. If set 
 * the IV Length is 96 Bits, clear for other IV lengths  */

#define QAT_LA_GCM_IV_LEN_FLAG_MASK                            0x1
/**< @ingroup icp_qat_fw_la
 * One bit mask used to determine the GCM IV Length indication bit. 
 * If set the IV Length is 96 Bits, clear for other IV lengths  */

#define QAT_LA_PROTO_BITPOS                                     6
/**< @ingroup icp_qat_fw_la
 * Starting bit position for the Lookaside Protocols */

#define QAT_LA_PROTO_MASK                                      0x7
/**< @ingroup icp_qat_fw_la
 * Three bit mask used to determine the Lookaside Protocol  */

/* Private defines */
#define QAT_LA_DIGEST_IN_BUFFER_BITPOS                          5
/**< @ingroup icp_qat_fw_la
 * Starting bit position for Digest in Buffer flag */

#define QAT_LA_DIGEST_IN_BUFFER_MASK                            0x1
/**< @ingroup icp_qat_fw_la
 * One bit mask used to determine the Digest in Buffer flag */

#define QAT_LA_CMP_AUTH_RES_BITPOS                               4
/**< @ingroup icp_qat_fw_la
 * Starting bit position for Auth compare digest result */

#define QAT_LA_CMP_AUTH_RES_MASK                               0x1
/**< @ingroup icp_qat_fw_la
 * One bit mask used to determine the Auth compare digest result */

#define QAT_LA_RET_AUTH_RES_BITPOS                               3
/**< @ingroup icp_qat_fw_la
 * Starting bit position for Auth return digest result */

#define QAT_LA_RET_AUTH_RES_MASK                               0x1
/**< @ingroup icp_qat_fw_la
 * One bit mask used to determine the Auth return digest result */

#define QAT_LA_UPDATE_STATE_BITPOS                               2
/**< @ingroup icp_qat_fw_la
 * Starting bit position for Update State. */

#define QAT_LA_UPDATE_STATE_MASK                               0x1
/**< @ingroup icp_qat_fw_la
 * One bit mask used to determine the Update State */

#define QAT_LA_PARTIAL_BITPOS                                    0
/**< @ingroup icp_qat_fw_la
 * Starting bit position indicating partial state */

#define QAT_LA_PARTIAL_MASK                                    0x3
/**< @ingroup icp_qat_fw_la
 * Two bit mask used to determine the partial state */

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 * Macro used for the generation of the Lookaside flags for a request. This
 * should always be used for the generation of the flags field. No direct sets
 * or masks should be performed on the flags data
 *
 * @param proto            Protocol handled by a command
 * @param cmp_auth         Compare auth result with the expected value
 * @param ret_auth         Return auth result to the client via DRAM
 * @param digest_in_buff   Digest is stored/extracted in/from the source buffer
 *                         straight after the authenticated region
 * @param wr_state         Indicate Writeback of the crypto state information
 *                         is required
 * @param partial          Inidicate if the packet is a partial part
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_FLAGS_BUILD(proto, cmp_auth, ret_auth, digest_in_buff,\
                                  wr_state, partial, gcm_iv_12_octets)      \
        ( ((proto & QAT_LA_PROTO_MASK) <<                                   \
            QAT_LA_PROTO_BITPOS)                |                           \
          ((cmp_auth & QAT_LA_CMP_AUTH_RES_MASK) <<                         \
            QAT_LA_CMP_AUTH_RES_BITPOS)         |                           \
          ((ret_auth & QAT_LA_RET_AUTH_RES_MASK) <<                         \
           QAT_LA_RET_AUTH_RES_BITPOS)          |                           \
          ((digest_in_buff & QAT_LA_DIGEST_IN_BUFFER_MASK) <<              \
           QAT_LA_DIGEST_IN_BUFFER_BITPOS)        |                         \
          ((wr_state & QAT_LA_UPDATE_STATE_MASK) <<                         \
           QAT_LA_UPDATE_STATE_BITPOS)          |                           \
          ((partial & QAT_LA_PARTIAL_MASK)       <<                         \
           QAT_LA_PARTIAL_BITPOS)               |                           \
          ((gcm_iv_12_octets & QAT_LA_GCM_IV_LEN_FLAG_MASK ) <<             \
           QAT_LA_GCM_IV_LEN_FLAG_BITPOS)   )


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for extraction of the GCM IV Len is 12 Octets / 96 Bits 
 *        information
 *
 * @param flags        Flags to extract the protocol state from
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_GCM_IV_LEN_FLAG_GET(flags)                        \
    QAT_FIELD_GET(flags, QAT_LA_GCM_IV_LEN_FLAG_BITPOS,                 \
        QAT_LA_GCM_IV_LEN_FLAG_MASK)
/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for extraction of the LA protocol state
 *
 * @param flags        Flags to extract the protocol state from
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_PROTO_GET(flags)                        \
    QAT_FIELD_GET(flags,QAT_LA_PROTO_BITPOS,QAT_LA_PROTO_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for extraction of the "compare auth" state
 *
 * @param flags        Flags to extract the compare auth result state from
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_CMP_AUTH_GET(flags)                   \
    QAT_FIELD_GET(flags,QAT_LA_CMP_AUTH_RES_BITPOS,QAT_LA_CMP_AUTH_RES_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for extraction of the "return auth" state
 *
 * @param flags        Flags to extract the return auth result state from
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_RET_AUTH_GET(flags)                   \
    QAT_FIELD_GET(flags,QAT_LA_RET_AUTH_RES_BITPOS,QAT_LA_RET_AUTH_RES_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *      Macro for extraction of the "digest in buffer" state
 *
 * @param flags     Flags to extract the digest in buffer state from
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_DIGEST_IN_BUFFER_GET(flags)                   \
    QAT_FIELD_GET(flags,QAT_LA_DIGEST_IN_BUFFER_BITPOS,             \
    QAT_LA_DIGEST_IN_BUFFER_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for extraction of the "update state" value.
 *
 * @param flags        Flags to extract the update state bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_WR_STATE_GET(flags)                   \
    QAT_FIELD_GET(flags,QAT_LA_UPDATE_STATE_BITPOS,QAT_LA_UPDATE_STATE_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for extraction of the "partial" packet state
 *
 * @param flags        Flags to extract the partial state from
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_PARTIAL_GET(flags)                    \
    QAT_FIELD_GET(flags,QAT_LA_PARTIAL_BITPOS,QAT_LA_PARTIAL_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for setting the LA protocol state of the flags
 *
 * @param flags        Flags to set with the protocol state
 * @param val        Protocol value
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_GCM_IV_LEN_FLAG_SET(flags,val)                  \
    QAT_FIELD_SET(flags, val, QAT_LA_GCM_IV_LEN_FLAG_BITPOS ,         \
        QAT_LA_GCM_IV_LEN_FLAG_MASK)
/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for setting the LA protocol state of the flags
 *
 * @param flags        Flags to set with the protocol state
 * @param val          Protocol value
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_PROTO_SET(flags,val)                  \
    QAT_FIELD_SET(flags,val,QAT_LA_PROTO_BITPOS,QAT_LA_PROTO_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for setting the "compare auth" state in the flags
 *
 * @param flags        Flags to set with the compare auth result state
 * @param val        Compare Auth value
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_CMP_AUTH_SET(flags,val)               \
    QAT_FIELD_SET(flags,val,QAT_LA_CMP_AUTH_RES_BITPOS,QAT_LA_CMP_AUTH_RES_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for setting the "return auth" state of the flags
 *
 * @param flags        Flags to set with the return auth result state
 * @param val        Return Auth value
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_RET_AUTH_SET(flags,val)               \
    QAT_FIELD_SET(flags,val,QAT_LA_RET_AUTH_RES_BITPOS,QAT_LA_RET_AUTH_RES_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *      Macro for setting the "digest in buffer" state of the flags
 *
 * @param flags     Flags to set with the digest in buffer state
 * @param val       Digest in buffer value
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_DIGEST_IN_BUFFER_SET(flags,val)               \
    QAT_FIELD_SET(flags,val,QAT_LA_DIGEST_IN_BUFFER_BITPOS,         \
                  QAT_LA_DIGEST_IN_BUFFER_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for setting the "update state" value of the flags
 *
 * @param flags      Flags to set with the update state bit
 * @param val        Writeback State value
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_WR_STATE_SET(flags,val)               \
    QAT_FIELD_SET(flags,val,QAT_LA_UPDATE_STATE_BITPOS,QAT_LA_UPDATE_STATE_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *        Macro for setting the "partial" packet state of the flags
 *
 * @param flags      Flags to set with the partial state
 * @param val        Partial state value
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_PARTIAL_SET(flags,val)                \
    QAT_FIELD_SET(flags,val,QAT_LA_PARTIAL_BITPOS,QAT_LA_PARTIAL_MASK)


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Cipher header of the content descriptor header block
 * @description
 *      Definition of the structure used to describe the cipher processing to
 *      perform on data. The cipher parameters are defined per algorithm
 *      and are located in the icp_qat_hw.h file.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_cipher_hdr_s
{
   uint8_t state_sz;
   /**< State size in quad words of the cipher algorithm used in this session.
    * Set to zero if the algorithm doesnt provide any state */

   uint8_t offset;
   /**< Quad word offset from the content descriptor parameters address i.e.
    * (content_address + (cd_hdr_sz << 3)) to the parameters for the cipher
    * processing */

   uint8_t curr_id;
   /**< Initialised with the cipher slice type */

   uint8_t next_id;
   /**< Set to the next slice to pass the ciphered data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * anymore slices after cipher */

   uint16_t resrvd;
   /**< Reserved padding byte to bring the struct to the word boundary. MUST be
    * set to 0 */

   uint8_t state_padding_sz;
   /**< State padding size in quad words. Set to 0 if no padding is required. */

   uint8_t key_sz;
   /**< Key size in quad words of the cipher algorithm used in this session */

} icp_qat_fw_cipher_hdr_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Authentication header of the content descriptor header block
 * @description
 *      Definition of the structure used to describe the auth processing to
 *      perform on data. The auth parameters are defined per algorithm
 *      and are located in the icp_qat_hw.h file.
 *
 *      Unused fields must be set to 0.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_auth_hdr_s
{
   uint8_t hash_flags;
   /**< General flags defining the processing to perform. 0 is normal processing
    * and 1 means there is a nested hash processing loop to go through */
   
   uint8_t offset;
   /**< Quad word offset from the content descriptor parameters address to the
    * parameters for the auth processing */
   
   uint8_t curr_id;
   /**< Initialised with the auth slice type */
   
   uint8_t next_id;
   /**< Set to the next slice to pass data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * anymore slices after auth */
   
   union 
   {
      uint8_t inner_prefix_sz;
      /**< Size in bytes of the inner prefix data */
     
      uint8_t aad_sz;
      /**< Size in bytes of padded AAD data to prefix to the packet for CCM
       *  or GCM processing */

   } u;
   
   uint8_t outer_prefix_sz;
   /**< Size in bytes of outer prefix data */
   
   uint8_t final_sz;
   /**< Size in bytes of digest to be returned to the client if requested */
   
   uint8_t inner_res_sz;
   /**< Size in bytes of the digest from the inner hash algorithm */

   uint8_t resrvd;
   /**< This field is unused, assumed value is zero. */
   
   uint8_t inner_state1_sz;
   /**< Size in bytes of inner hash state1 data. Must be a qword multiple */
   
   uint8_t inner_state2_off;
   /**< Quad word offset from the content descriptor parameters pointer to the
    * inner state2 value */
   
   uint8_t inner_state2_sz;
   /**< Size in bytes of inner hash state2 data. Must be a qword multiple */

   uint8_t outer_config_off;
   /**< Quad word offset from the content descriptor parameters pointer to the
    * outer configuration information */
   
   uint8_t outer_state1_sz;
   /**< Size in bytes of the outer state1 value */
   
   uint8_t outer_res_sz;
   /**< Size in bytes of digest from the outer auth algorithm */
   
   uint8_t outer_prefix_off;
   /**< Quad word offset from the start of the inner prefix data to the outer
    * prefix information. Should equal the rounded inner prefix size, converted
    * to qwords  */

} icp_qat_fw_auth_hdr_t;


#define ICP_QAT_FW_AUTH_HDR_FLAG_DO_NESTED   1
/**< @ingroup icp_qat_fw_comn
 * Definition of the hash_flags bit of the auth_hdr to inidacte the request
 * requires nested hashing */

#define ICP_QAT_FW_AUTH_HDR_FLAG_NO_NESTED   0
/**< @ingroup icp_qat_fw_comn
 * Definition of the hash_flags bit of the auth_hdr for no nested hashing
 * required */

#define ICP_QAT_FW_CCM_GCM_AAD_SZ_MAX        240
/**< @ingroup icp_qat_fw_comn
 * Maximum size of AAD data allowed for CCM or GCM processing. AAD data size
 * is stored in 8-bit field and must be multiple of hash block size. 240 is
 * largest value which satisfy both requirements.AAD_SZ_MAX is in byte units*/


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the common LA QAT FW bulk request
 * @description
 *        Definition of the bulk processing request type. Used for hash only,
 *        cipher only, hash-cipher and authentication-encryption requests
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_bulk_req_s
{
   icp_qat_fw_comn_req_hdr_t comn_hdr;
   /**< Common request header */

   uint32_t flow_id;
   /** < Field used by Firmware to limit the number of stateful requests 
   * for a session being processed at a given point of time            */

   icp_qat_fw_la_comn_req_t comn_la_req;
   /**< Common LA request parameters */

   icp_qat_fw_comn_req_mid_t comn_mid;
   /**< Common request middle section */

   uint64_t req_params_addr;
   /**< Memory address of the request parameters */

   icp_qat_fw_comn_req_ftr_t comn_ftr;
   /**< Common request footer */

} icp_qat_fw_la_bulk_req_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the Lookaside Eagle Tail Response
 * @description
 *        This is the response delivered to the ET rings by the Lookaside
 *              QAT FW service for all commands
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_resp_s
{
   icp_qat_fw_comn_resp_hdr_t comn_resp;
   /**< Common interface response format see icp_qat_fw.h */

   uint32_t resrvd[(ICP_QAT_FW_RESP_DEFAULT_SZ -           \
                    sizeof(icp_qat_fw_comn_resp_hdr_t)) /  \
                    sizeof(uint32_t)];
   /**< Fields reserved for future use and assumed set to 0 */

} icp_qat_fw_la_resp_t;

/*
 * request parameter #defines
 */
#define ICP_QAT_FW_HASH_REQUEST_PARAMETERS_OFFSET \
                                (sizeof(icp_qat_fw_la_cipher_req_params_t))
/**< @ingroup icp_qat_fw_comn
 * Offset in bytes from the start of the request parameters block to the hash
 * (auth) request parameters */

#define ICP_QAT_FW_CIPHER_REQUEST_PARAMETERS_OFFSET (0)
/**< @ingroup icp_qat_fw_comn
 * Offset in bytes from the start of the request parameters block to the cipher
 * request parameters */

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the cipher request parameters block
 * @description
 *        Definition of the cipher processing request parameters block
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_cipher_req_params_s
{
   uint8_t resrvd;
   /**< Reserved field and assumed set to 0 */

   uint8_t cipher_state_sz;
   /**< Number of quad words of state data for the cipher algorithm */

   uint8_t curr_id;
   /**< Initialised with the cipher slice type */

   uint8_t next_id;
   /**< Set to the next slice to pass the ciphered data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * anymore slices after cipher */

   uint16_t resrvd1;
   /**< Reserved field, should be set to zero*/

   uint8_t resrvd2;
   /**< Reserved field, should be set to zero*/

   uint8_t next_offset;
   /**< Offset in bytes to the next request parameter block */

   uint32_t cipher_off;
   /**< Byte offset from the start of packet to the cipher data region */

   uint32_t cipher_len;
   /**< Byte length of the cipher data region */

   uint64_t state_address;
   /**< Flat buffer address in memory of the cipher state information. Unused
    * if the state size is 0 */

} icp_qat_fw_la_cipher_req_params_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the auth request parameters block
 * @description
 *        Definition of the auth processing request parameters block
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_auth_req_params_s
{
   uint8_t auth_res_sz;
   /**< Size in quad words of digest information to validate */

   uint8_t hash_state_sz;
   /**< Number of quad words of inner and outer hash prefix data to process */

   uint8_t curr_id;
   /**< Initialised with the auth slice type */

   uint8_t next_id;
   /**< Set to the next slice to pass the auth data through.
    * Set to ICP_QAT_FW_SLICE_NULL for in-place auth-only requests
    * Set to ICP_QAT_FW_SLICE_DRAM_WR for all other request types
    * if the data is not to go through anymore slices after auth */

   union 
   {
       uint16_t resrvd;
       /**< Reserved field should be set to zero for bulk services */
       
       uint16_t tls_secret_len;
       /**< Length of Secret information for TLS.   */

   } u;

   uint8_t resrvd;
   /**< Reserved field, should be set to zero*/

   uint8_t next_offset;
   /**< offset in bytes to the next request parameter block */

   uint32_t auth_off;
   /**< Byte offset from the start of packet to the auth data region */

   uint32_t auth_len;
   /**< Byte length of the auth data region */

   union 
   {
       uint64_t prefix_addr;
       /**< Address of the prefix information */

       uint64_t aad_addr;
       /**< Address of the AAD info in DRAM. Used for the CCM and GCM
        * protocols */

   } u1;

   uint64_t auth_res_address;
   /**< Address of the auth result information to validate or the location to
    * writeback the digest information to */

} icp_qat_fw_la_auth_req_params_t;



/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the common LA QAT FW TRNG request
 * @description
 *        Definition of the TRNG processing request type
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_trng_req_s
{
   icp_qat_fw_comn_req_hdr_t comn_hdr;
   /**< Common request header */
 
   uint32_t flow_id;
   /** < Field used by Firmware to limit the number of stateful requests 
    * for a session being processed at a given point of time            */

   icp_qat_fw_la_comn_req_t comn_la_req;
   /**< Common LA request parameters */

   icp_qat_fw_comn_req_mid_t comn_mid;
   /**< Common request middle section */

   union 
   {
       uint32_t resrvd;
       /**< Reserved field, should be set to zero*/

       uint32_t entropy_length;
       /**< Size of the data in bytes to process. Used by the get_random 
        * command. Set to 0 for commands that dont need a length parameter */

   } u;

   uint32_t resrvd;
   /**< Reserved field, should be set to zero*/

   icp_qat_fw_comn_req_ftr_t comn_ftr;
   /**< Common request footer */

} icp_qat_fw_la_trng_req_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the Lookaside TRNG Test Status Structure
 * @description
 *        As an addition to ICP_QAT_FW_LA_TRNG_STATUS Pass or Fail information 
 *        in common response fields, as a response to TRNG_TEST request, Test 
 *        status, Counter for failed tests and 4 entropy counter values are 
 *        sent
 *        Status of test status and the fail counts. 
 *
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_trng_test_result_s
{
   uint32_t test_status_info;
   /**< TRNG comparator health test status& Validity information
   see Test Status Bit Fields below. */ 

   uint32_t test_status_fail_count;
   /**< TRNG comparator health test status, 32bit fail counter*/     

   uint64_t r_ent_ones_cnt; 
   /**< Raw Entropy ones counter */ 

   uint64_t r_ent_zeros_cnt;
   /**< Raw Entropy zeros counter */ 

   uint64_t c_ent_ones_cnt; 
   /**< Conditioned Entropy ones counter */ 

   uint64_t c_ent_zeros_cnt; 
   /**< Conditioned Entropy zeros counter */ 

   uint64_t resrvd; 
   /**< Reserved field must be set to zero */ 

}icp_qat_fw_la_trng_test_result_t; 


/*  Definitions of the bits in the test_status_info of the TRNG_TEST response.
 *  The values returned by the Lookaside service are given below
 *  The Test result and Test Fail Count values are only valid if the Test 
 *  Results Valid (Tv) is set. 
 *
 *  TRNG Test Status Info 
 *  + ===== + ------------------------------------------------ + --- + --- +
 *  |  Bit  |                   31 - 2                         |  1  |  0  |
 *  + ===== + ------------------------------------------------ + --- + --- +
 *  | Flags |                 RESERVED = 0                     | Tv  | Ts  |
 *  + ===== + ------------------------------------------------------------ +
 */
/******************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the Lookaside TRNG Test Status Information received as 
 *        a part of icp_qat_fw_la_trng_test_result_t
 *
 *****************************************************************************/
#define QAT_FW_LA_TRNG_TEST_STATUS_TS_BITPOS                 0
/**< @ingroup icp_qat_fw_la
 * TRNG Test Result t_status field bit pos definition.*/

#define QAT_FW_LA_TRNG_TEST_STATUS_TS_MASK                   0x1 
/**< @ingroup icp_qat_fw_la
 * TRNG Test Result t_status field mask definition.*/

#define QAT_FW_LA_TRNG_TEST_STATUS_TEST_VALID_BITPOS         1
/**< @ingroup icp_qat_fw_la
 * TRNG Test Result test results valid field bit pos definition.*/

#define QAT_FW_LA_TRNG_TEST_STATUS_TEST_VALID_MASK           0x1   
/**< @ingroup icp_qat_fw_la
 * TRNG Test Result test results valid field mask definition.*/

/******************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the Lookaside TRNG test_status values. 
 *        
 *
 *****************************************************************************/
#define QAT_FW_LA_TRNG_TEST_STATUS_TEST_VALID                 1
/**< @ingroup icp_qat_fw_la
 * TRNG TEST Response Test Results Valid Value.*/

#define QAT_FW_LA_TRNG_TEST_STATUS_TEST_NOT_VALID             0
/**< @ingroup icp_qat_fw_la
 * TRNG TEST Response Test Results are NOT Valid Value.*/

#define QAT_FW_LA_TRNG_TEST_STATUS_TEST_NO_FAILS              1
/**< @ingroup icp_qat_fw_la
 * Value for TRNG Test status tests have NO FAILs Value.*/

#define QAT_FW_LA_TRNG_TEST_STATUS_TEST_HAS_FAILS             0
/**< @ingroup icp_qat_fw_la
 * Value for TRNG Test status tests have one or more FAILS Value.*/

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *       Macro for extraction of the Test Status Field returned in the response
 *       to TRNG TEST command.
 *
 * @param test_status        8 bit test_status value to extract the status bit 
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_TRNG_TEST_STATUS_TS_FLD_GET(test_status)        \
   QAT_FIELD_GET(test_status,                                         \
                 QAT_FW_LA_TRNG_TEST_STATUS_TS_BITPOS,                \
                 QAT_FW_LA_TRNG_TEST_STATUS_TS_MASK)
/**
 ******************************************************************************
 * @ingroup icp_qat_fw_la
 *
 * @description
 *       Macro for extraction of the Test Results Valid Field returned in the 
 *       response to TRNG TEST command.
 *
 * @param test_status        8 bit test_status value to extract the Tests 
 *                           Results valid bit 
 *
 *****************************************************************************/
#define ICP_QAT_FW_LA_TRNG_TEST_STATUS_TV_FLD_GET(test_status)        \
   QAT_FIELD_GET(test_status,                                         \
                 QAT_FW_LA_TRNG_TEST_STATUS_TEST_VALID_BITPOS,        \
                 QAT_FW_LA_TRNG_TEST_STATUS_TEST_VALID_MASK)


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the Lookaside TRNG Response
 * @description
 *        This is the response delivered to the ET rings by the Lookaside
 *              QAT FW TRNG service commands
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_trng_resp_s
{
   icp_qat_fw_comn_resp_hdr_t comn_resp;
   /**< Common interface response format see icp_qat_fw.h */

   uint32_t resrvd[(ICP_QAT_FW_RESP_DEFAULT_SZ -                     \
                   sizeof(icp_qat_fw_comn_resp_hdr_t)) /             \
                   sizeof(uint32_t)];

} icp_qat_fw_la_trng_resp_t;


/*
 ******************************************************************************
 * MGF Max supported input parameters
 ******************************************************************************
 */
#define ICP_QAT_FW_LA_MGF_SEED_LEN_MAX            255
/**< @ingroup icp_qat_fw_la
 * Maximum seed length for MGF1 request in bytes
 * Typical values may be 48, 64, 128 bytes (or any).*/

#define ICP_QAT_FW_LA_MGF_MASK_LEN_MAX            65528
/**< @ingroup icp_qat_fw_la
 * Maximum mask length for MGF1 request in bytes
 * Typical values may be 8 (64-bit), 16 (128-bit). MUST be quad word multiple */

/*
 ******************************************************************************
 * SSL Max supported input parameters
 ******************************************************************************
 */
#define ICP_QAT_FW_LA_SSL_SECRET_LEN_MAX          512
/**< @ingroup icp_qat_fw_la
 * Maximum secret length for SSL3 Key Gen request (bytes) */

#define ICP_QAT_FW_LA_SSL_ITERATES_LEN_MAX        16
/**< @ingroup icp_qat_fw_la
 * Maximum iterations for SSL3 Key Gen request (integer) */

#define ICP_QAT_FW_LA_SSL_LABEL_LEN_MAX           136
/**< @ingroup icp_qat_fw_la
 * Maximum label length for SSL3 Key Gen request (bytes) */

#define ICP_QAT_FW_LA_SSL_SEED_LEN_MAX            64
/**< @ingroup icp_qat_fw_la
 * Maximum seed length for SSL3 Key Gen request (bytes) */

#define ICP_QAT_FW_LA_SSL_OUTPUT_LEN_MAX          248
/**< @ingroup icp_qat_fw_la
 * Maximum output length for SSL3 Key Gen request (bytes) */


/*
 ******************************************************************************
 * TLS Max supported input parameters
 ******************************************************************************
 */
#define ICP_QAT_FW_LA_TLS_SECRET_LEN_MAX          128
/**< @ingroup icp_qat_fw_la
 * Maximum secret length for TLS Key Gen request (bytes) */

#define ICP_QAT_FW_LA_TLS_V1_1_SECRET_LEN_MAX          128
/**< @ingroup icp_qat_fw_la
 * Maximum secret length for TLS Key Gen request (bytes) */

#define ICP_QAT_FW_LA_TLS_V1_2_SECRET_LEN_MAX          64
/**< @ingroup icp_qat_fw_la
 * Maximum secret length for TLS Key Gen request (bytes) */

#define ICP_QAT_FW_LA_TLS_LABEL_LEN_MAX            255
/**< @ingroup icp_qat_fw_la
 * Maximum label length for TLS Key Gen request (bytes) */

#define ICP_QAT_FW_LA_TLS_SEED_LEN_MAX             64
/**< @ingroup icp_qat_fw_la
 * Maximum seed length for TLS Key Gen request (bytes) */

#define ICP_QAT_FW_LA_TLS_OUTPUT_LEN_MAX           248
/**< @ingroup icp_qat_fw_la
 * Maximum output length for TLS Key Gen request (bytes) */


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the shared fields between SSL or MGF information.
 *        This 64 bit structure replaces with a request parameter pointer for
 *        TLS key generation command. 
 * @description
 *         
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_key_gen_common_s
{
   uint32_t resrvd; 
   /**< Reserved data assumed set to 0 */

   union
   {
       uint16_t secret_len; 
       /**< Length of Secret information for SSL. In the case of TLS the
       * secret is supplied in the content descriptor */

       uint16_t mask_len; 
       /**< Size in bytes of the desired output mask for MGF1*/

   } u;

   union
   {
       uint8_t iter_count;
       /**< Iteration count used by the SSL key gen request */

       uint8_t resrvd;
       /**< Reserved filed set to 0 for MGF1 */

   } u1;

   uint8_t resrvd1;
   /**< Reserved data assumed set to 0 */

}icp_qat_fw_la_key_gen_common_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *        Definition of the common LA QAT FW Key Generate Functions
 * @description
 *        Definition of the Key Generate Function processing request types.
 *        This struct defines the layout for the SSL3 key generation, TLS Key
 *        generation and MGF1
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_key_gen_req_s
{
   icp_qat_fw_comn_req_hdr_t comn_hdr;
   /**< Common request header */

   uint32_t flow_id;
   /** < Field used by Firmware to limit the number of stateful requests 
    * for a session being processed at a given point of time            */

   icp_qat_fw_la_comn_req_t comn_la_req;
   /**< Common LA request parameters. Note the la command flags have no
    * meaning for this request type */

   icp_qat_fw_comn_req_mid_t comn_mid;
   /**< Common request middle section */

   union
   {
       uint64_t req_params_addr; 
       /**< For the TLS processing this is the pointer to the request
        *   parameters */

       icp_qat_fw_la_key_gen_common_t keygen_comn;
       /**< For other key gen processing these field holds ssl or mgf
        *   parameters */

   } u;

   icp_qat_fw_comn_req_ftr_t comn_ftr;
   /**< Common request footer */

} icp_qat_fw_la_key_gen_req_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *      Definition of the Lookaside SSL Key Material Input
 * @description
 *      This struct defines the layout of input parameters for the
 *      SSL3 key generation (source flat buffer format)
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_ssl_key_material_input_s
{
   uint64_t seed_addr;
   /**< Pointer to seed */

   uint64_t label_addr;
   /**< Pointer to label(s) */

   uint64_t secret_addr;
   /**< Pointer to secret */

} icp_qat_fw_la_ssl_key_material_input_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_la
 *      Definition of the Lookaside TLS Key Material Input
 * @description
 *      This struct defines the layout of input parameters for the
 *      TLS key generation (source flat buffer format)
 * @note
 *      Secret state value (S split into S1 and S2 parts) is supplied via
 *      Content Descriptor. S1 is placed in an outer prefix buffer, and S2
 *      inside the inner prefix buffer.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_la_tls_key_material_input_s
{
   uint64_t seed_addr;
   /**< Pointer to seed */

   uint64_t label_addr;
   /**< Pointer to label(s) */

} icp_qat_fw_la_tls_key_material_input_t;

#endif /* __ICP_QAT_FW_LA_H__ */
