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
 * @file icp_qat_fw.h
 * @defgroup icp_qat_fw_comn ICP QAT FW Common Processing Definitions
 * @ingroup icp_qat_fw
 *
 * @description
 *      This file documents the common interfaces that the QAT FW running on
 *      the QAT AE exports. This common layer is used by a number of services
 *      to export content processing services.
 *
 *****************************************************************************/

#ifndef __ICP_QAT_FW_H__
#define __ICP_QAT_FW_H__

/*
* ==============================
* General Notes on the Interface
*/

/*
*
* ==============================
*
* Introduction
*
* Data movement and slice chaining
*
* Endianness
*      - Unless otherwise stated all structures are defined in BIG ENDIAN MODE
*
* Alignment
*      - In general all data structures provided to a request should be aligned
*      on the 64 byte boundary so as to allow optimal memory transfers. At the
*      minimum they must be aligned to the 8 byte boundary
*
* Ordering
*      -
*
* Extensibility
*
* Debugging
*
* Further information
*
* Sizes
*   Quad words = 8 bytes
*
* Terminology
*
* ==============================
*/

/*
******************************************************************************
* Include public/global header files
******************************************************************************
*/

#include "icp_arch_interfaces.h"
#include "icp_qat_hw.h"

/* Big assumptions that both bitpos and mask are constants */
#define QAT_FIELD_SET(flags,val,bitpos,mask)      \
   (flags) = (((flags) & (~((mask) << (bitpos)))) | (((val) & (mask)) << \
   (bitpos)))

#define QAT_FIELD_GET(flags,bitpos,mask)         \
   (((flags) >> (bitpos)) & (mask))


/**< @ingroup icp_qat_fw_comn
 * Default request and response ring size in bytes */
#define ICP_QAT_FW_REQ_DEFAULT_SZ                64
#define ICP_QAT_FW_RESP_DEFAULT_SZ               64

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Common request flags type
 *
 * @description
 *      Definition of the common request and response flags. These describe
 *      the contents of the common request and processing attributes for the
 *      request.
 *
 *****************************************************************************/
typedef uint16_t icp_qat_fw_comn_flags;

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Definition of the common QAT FW request header.
 * @description
 *      Common section of the request used across all of the services exposed
 *      by the QAT FW. Each of the services inherit these common fields
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comn_req_hdr_s
{
   icp_arch_if_req_hdr_t arch_if;
   /**< Common arch fields used by all ICP interface requests. Remaining
    * fields are specific to the common QAT FW service. */

   icp_qat_fw_comn_flags comn_req_flags;
   /**< Flags used to describe common processing required by the request and
    * the meaning of parameters in it i.e. differentiating between a buffer
    * descriptor and a flat buffer pointer in the source (src) and destination
    * (dest) data address fields. Full definition of the fields is given
    * below */

   uint8_t content_desc_params_sz;
   /**< Size of the content descriptor parameters in quad words. These
    * parameters describe the session setup configuration info for the
    * slices that this request relies upon i.e. the configuration word and
    * cipher key needed by the cipher slice if there is a request for cipher
    * processing. The format of the parameters are contained in icp_qat_hw.h
    * and vary depending on the algorithm and mode being used. It is the
    * clients responsibility to ensure this structure is correctly packed */

   uint8_t   content_desc_hdr_sz;
   /**< Size of the content descriptor header in quad words. This information
    * is read into the QAT AE xfr registers */

   uint64_t content_desc_addr;
   /**< Address of the content descriptor containing both the content header
    * the size of which is defined by content_desc_hdr_sz followed by the
    * content parameters whose size is described by content_desc_params_sz */

} icp_qat_fw_comn_req_hdr_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Definition of the common QAT FW request middle block.
 * @description
 *      Common section of the request used across all of the services exposed
 *      by the QAT FW. Each of the services inherit these common fields
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comn_req_mid_s
{
   uint64_t opaque_data;
   /**< Opaque data passed unmodified from the request to response messages by
    * firmware (fw) */

   uint64_t src_data_addr;
   /**< Generic definition of the source data supplied to the QAT AE. The
    * common flags are used to further describe the attributes of this
    * field */

   uint64_t dest_data_addr;
   /**< Generic definition of the destination data supplied to the QAT AE. The
    * common flags are used to further describe the attributes of this
    * field */

} icp_qat_fw_comn_req_mid_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Definition of the common QAT FW request footer.
 * @description
 *      Common section of the request used across all of the services exposed
 *      by the QAT FW. Each of the services inherit these common fields
 *
 *****************************************************************************/
typedef union icp_qat_fw_comn_req_ftr_s
{
   uint64_t next_request_addr;
   /** < Overloaded field, for stateful requests, this field is the pointer to 
         next request descriptor */
   struct 
   { 
       uint32_t src_length;
       /** < Length of source flat buffer incase src buffer type is flat */

       uint32_t dst_length;
       /** < Length of source flat buffer incase dst buffer type is flat */

   } s;

} icp_qat_fw_comn_req_ftr_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Error code field
 *
 * @description
 *      Overloaded field with 8 bit common error field or two 
 *      8 bit compression error fields for compression and translator slices
 *
 *****************************************************************************/
typedef union icp_qat_fw_comn_error_s
{
   struct 
   {
       uint8_t resrvd;
       /**< 8 bit reserved field */

       uint8_t comn_err_code;
       /**< 8 bit common error code */

   } s;
   /**< Structure which is used for non-compression responses */

   struct 
   {
       uint8_t xlat_err_code;
       /**< 8 bit translator error field */

       uint8_t cmp_err_code;
       /**< 8 bit compression error field */

   } s1;
   /** Structure which is used for compression responses */

} icp_qat_fw_comn_error_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Definition of the common QAT FW response header.
 * @description
 *      This section of the response is common across all of the services
 *      that generate a S interface response
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comn_resp_hdr_s
{
   icp_arch_if_resp_hdr_t arch_if;
   /**< Common arch fields used by all ICP interface response messages. The
    * remaining fields are specific to the QAT FW */

   icp_qat_fw_comn_error_t  comn_error;
   /**< This field is overloaded to allow for one 8 bit common error field
    *   or two 8 bit error fields from compression and translator  */

   uint8_t   comn_status;
   /**< Status field which specifies which slice(s) report an error */

   uint8_t   serv_cmd_id;
   /**< For services that define multiple commands this field represents the
    * command. If only 1 command is supported then this field will be 0 */

   uint64_t opaque_data;
   /**< Opaque data passed from the request to the response message */

} icp_qat_fw_comn_resp_hdr_t;


/* ========================================================================= */
/*                        ARCH IF PRIVILEGE OVERRIDING INTERFACE DEFINITIONS */
/*           Overrides PVL bit field of icp_arch_if_req_hdr_t.req_type       */
/*            See icp_arch_interfaces.h for icp_arch_if_req_hdr_t            */
/* ========================================================================= */

/*
 *  icp_arch_if_req_hdr_t req_type is overriden with pvl as follows:
 * 
 *  + ===== + ---- + ---------- + ----- + --------- + --------- + ----------- +
 *  |  Bit  |  31  |   30 - 24  | 23 22 |  21 - 16  |  15 - 8   |  7 - 0      |
 *  + ===== + ---- + ---------- + ----- + --------- + --------- + ----------- +
 *  | Flags |  V   |   Flags    |  Pvl  | Req Type  | Res Pipe  | Res Dest ID |
 *  + ===== + ---- + ---------- + ----- + --------- + --------- + ----------- +
*/

#define QAT_COMN_PRIVILEGE_BITPOS   6
/**< @ingroup icp_qat_fw_comn
 * Starting bit position overriden Privilege bitpos  */

#define QAT_COMN_PRIVILEGE_MASK    0x3
/**< @ingroup icp_qat_fw_comn
 * Bit mask used to determine Privilege Level Field*/

#define QAT_COMN_REQ_TYPE_BITPOS   0
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Request Type Field */

#define QAT_COMN_REQ_TYPE_MASK     0x3F
/**< @ingroup icp_qat_fw_comn
 * Bit mask used to determine Req Type Field*/


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of Privilege level field in request type field
 *
 * @param arch_if_resp_hdr      Given Arch Header IF to override
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_OV_PRV_GET(arch_if_resp_hdr)   \
   QAT_FIELD_GET(arch_if_resp_hdr.req_type,QAT_COMN_PRIVILEGE_BITPOS, \
                 QAT_COMN_PRIVILEGE_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for overriding of Privilege level, in request type field
 *
 * @param  arch_if_resp_hdr      Given Arch Header IF to override
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_OV_PRV_SET(arch_if_resp_hdr, val)   \
   QAT_FIELD_SET(arch_if_resp_hdr.req_type,val,QAT_COMN_PRIVILEGE_BITPOS, \
                 QAT_COMN_PRIVILEGE_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of Request Type Field
 *
 * @param arch_if_resp_hdr      Given Arch Header IF to override
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_OV_REQ_TYPE_GET(arch_if_resp_hdr)   \
   QAT_FIELD_GET(arch_if_resp_hdr.req_type,QAT_COMN_REQ_TYPE_BITPOS, \
                 QAT_COMN_REQ_TYPE_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting of Request Type Field
 *
 * @param arch_if_resp_hdr      Given Arch Header IF to override
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_OV_REQ_TYPE_SET(arch_if_resp_hdr, val)   \
   QAT_FIELD_SET(arch_if_resp_hdr.req_type,val,QAT_COMN_REQ_TYPE_BITPOS, \
                 QAT_COMN_REQ_TYPE_MASK)

/*
 *  < @ingroup icp_qat_fw_comn
 *  Common Request Flags Definition  
 *  The bit offsets below are within the flags field. These are NOT relative to
 *  the memory word. Unused field must be zeroed.
 * 
 *  + ===== + ------ + --- + --- + --- + --- + --- + --- + --- + --- +
 *  | Bits [15:8]    |  15 |  14 |  13 |  12 |  11 |  10 |  9  |  8  |
 *  + ===== + ------ + --- + --- + --- + --- + --- + --- + --- + --- +
 *  | Flags[15:8]    | Ord | Ptr | EOrd| Shi | Rex | Xlt | Cpr | Blk |
 *  + ===== + ------ + --- + --- + --- + --- + --- + --- + --- + --- +
 *  | Bits  [7:0]    |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 *  + ===== + ------ + --- + --- + --- + --- + --- + --- + --- + --- +
 *  | Flags [7:0]    | Stg | Rnd | Pk1 | Pk0 | Au1 | Au0 | Ci1 | Ci0 |
 *  + ===== + ------ + --- + --- + --- + --- + --- + --- + --- + --- +
 */

#define QAT_COMN_ORD_BITPOS                 15
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating default ordered requests/responses  */

#define QAT_COMN_ORD_MASK                   0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine the default ordering. */

#define QAT_COMN_PTR_TYPE_BITPOS            14
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Src&Dst Buffer Pointer type  */

#define QAT_COMN_PTR_TYPE_MASK              0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Src&Dst Buffer Pointer type */

#define QAT_COMN_EXT_ORD_BITPOS             13
/**< @ingroup icp_qat_fw_comn
 * Starting bit position for extend ordered requests/response  */

#define QAT_COMN_EXT_ORD_MASK              0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine the extend ordering  */

#define QAT_COMN_SHRAM_INIT_BITPOS          12
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Shram Initialization Flag */

#define QAT_COMN_SHRAM_INIT_MASK            0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Shram Initialization Flag */

#define QAT_COMN_REGEX_SLICE_BITPOS         11
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Regex Slice Required Flag */

#define QAT_COMN_REGEX_SLICE_MASK           0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Regex Slice Required Flag  */

#define QAT_COMN_XLAT_SLICE_BITPOS          10
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Translator Slice Required Flag */

#define QAT_COMN_XLAT_SLICE_MASK            0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Translator Slice Required Flag */

#define QAT_COMN_CPR_SLICE_BITPOS           9
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Compression Slice Required Flag */

#define QAT_COMN_CPR_SLICE_MASK             0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Compression Slice Required Flag */

#define QAT_COMN_BULK_SLICE_BITPOS          8
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Bulk Slice Required Flag */

#define QAT_COMN_BULK_SLICE_MASK            0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Bulk Slice Required Flag */

#define QAT_COMN_STORAGE_SLICE_BITPOS       7
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Storage Slice Required Flag */

#define QAT_COMN_STORAGE_SLICE_MASK         0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Storage Slice Required Flag */

#define QAT_COMN_RND_SLICE_BITPOS           6
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating TRNG Slice Required Flag */

#define QAT_COMN_RND_SLICE_MASK             0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine TRNG Slice Required Flag */

#define QAT_COMN_PKE1_SLICE_BITPOS          5
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating PKE1 Slice Required Flag */

#define QAT_COMN_PKE1_SLICE_MASK            0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine PKE1 Slice Required Flag */

#define QAT_COMN_PKE0_SLICE_BITPOS          4
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating PKE0 Slice Required Flag */

#define QAT_COMN_PKE0_SLICE_MASK            0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine PKE0 Slice Required Flag */

#define QAT_COMN_AUTH1_SLICE_BITPOS         3
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Auth Slice 1 Required Flag */

#define QAT_COMN_AUTH1_SLICE_MASK           0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Auth Slice 1 Required Flag */

#define QAT_COMN_AUTH0_SLICE_BITPOS         2
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Auth Slice 0 Required Flag */

#define QAT_COMN_AUTH0_SLICE_MASK           0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Auth Slice 0 Required Flag */

#define QAT_COMN_CIPHER1_SLICE_BITPOS       1
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Cipher Slice 1 Required Flag */

#define QAT_COMN_CIPHER1_SLICE_MASK         0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Cipher Slice 1 Required Flag */

#define QAT_COMN_CIPHER0_SLICE_BITPOS       0
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Cipher Slice 0 Required Flag */

#define QAT_COMN_CIPHER0_SLICE_MASK         0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Cipher Slice 0 Required Flag */
 
/* ========================================================================= */
/*                                       Default Ordering Flag definitions   */
/* ========================================================================= */

#define ICP_QAT_FW_COMN_ORD_FLAG_NONE          0
/**< @ingroup icp_qat_fw_comn
 * Definition of a request that will not guarantee ordered processing in the
 * QAT FW*/

#define ICP_QAT_FW_COMN_ORD_FLAG_STRICT      1
/**< @ingroup icp_qat_fw_comn
 * Definition of a request that will guarantee ordered processing in the
 * QAT FW. NOTE: the ordering is based on ring ordering not session ordering.
 * If a session uses a single ring to submit demand to the QAT FW then
 * ordering of requests will be maintained assuming the bit is set. However
 * if requests for a session are submitted across a number of rings then the
 * QAT FW will not guarantee the ordering */

/* ========================================================================= */
/*                                       Pointer Type Flag definitions       */
/* ========================================================================= */
#define QAT_COMN_PTR_TYPE_FLAT                0x0
/**< @ingroup icp_qat_fw_comn
 * Constant value indicating Src&Dst Buffer Pointer type is flat */

#define QAT_COMN_PTR_TYPE_SGL                 0x1
/**< @ingroup icp_qat_fw_comn
 * Constant value indicating Src&Dst Buffer Pointer type is SGL type */

/* ========================================================================= */
/*                                       Extend Ordering Flag definitions    */
/* ========================================================================= */

#define ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE          0
/**< @ingroup icp_qat_fw_comn
 * Definition of a request that will not guarantee ordered processing in the
 * QAT FW*/

#define ICP_QAT_FW_COMN_EXT_ORD_FLAG_STRICT        1
/**< @ingroup icp_qat_fw_comn
 * Extend definition of a request that will guarantee ordered processing in
 * the QAT FW. NOTE: it only guarantees the ordering when bit15 is set in
 * common flag, thus to say, bit15 and bit13 determine the ordering together.*/

/* ========================================================================= */
/*                                      Shram Init Required Flag definitions */
/* ========================================================================= */
#define QAT_COMN_SHRAM_INIT_REQUIRED          0x1
/**< @ingroup icp_qat_fw_comn
 * Constant value to indicate ME is responsible for initialization of 
   shram (For ECC), this bit is ignored for non Init/Admin requests. */

#define QAT_COMN_SHRAM_INIT_NOT_REQUIRED      0x0
/**< @ingroup icp_qat_fw_comn
 * Constant value to indicate Shram init is not required.  */

/* ========================================================================= */
/*                                    Regex Slice Required Flag definitions  */
/* ========================================================================= */
#define QAT_COMN_REGEX_SLICE_REQUIRED         0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Regex Slice Required Flag */

#define QAT_COMN_REGEX_SLICE_NOT_REQUIRED     0x0
/**< @ingroup icp_qat_fw_comn
 * Constant value to indicate Regex Slice is not required.  */

/* ========================================================================= */
/*                                Translator Slice Required Flag definitions */
/* ========================================================================= */
#define QAT_COMN_XLAT_SLICE_REQUIRED          0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Translator Slice Required Flag */

#define QAT_COMN_XLAT_SLICE_NOT_REQUIRED      0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Translator Slice not Required Flag */

/* ========================================================================= */
/*                               Compression Slice Required Flag definitions */
/* ========================================================================= */
#define QAT_COMN_CPR_SLICE_REQUIRED           0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Compression Slice Required Flag */

#define QAT_COMN_CPR_SLICE_NOT_REQUIRED       0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Compression Slice not Required Flag */

/* ========================================================================= */
/*                                Bulk Slice Required Flag definitions       */
/* ========================================================================= */
#define QAT_COMN_BULK_SLICE_REQUIRED          0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Bulk Slice Required Flag */

#define QAT_COMN_BULK_SLICE_NOT_REQUIRED      0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Bulk Slice not Required Flag */

/* ===============================================================+========== */
/*                                Storage Slice Required Flag definitions     */
/* ========================================================================== */
#define QAT_COMN_STORAGE_SLICE_REQUIRED       0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Storage Slice Required Flag */

#define QAT_COMN_STORAGE_SLICE_NOT_REQUIRED   0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Storage Slice not Required Flag */

/* ========================================================================== */
/*                                TRNG Slice Required Flag definitions        */
/* ========================================================================== */
#define QAT_COMN_RND_SLICE_REQUIRED           0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for TRNG Slice Required Flag */

#define QAT_COMN_RND_SLICE_NOT_REQUIRED       0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for TRNG Slice not Required Flag */

/* ========================================================================== */
/*                                PKE1 Slice Required Flag definitions        */
/* ========================================================================== */
#define QAT_COMN_PKE1_SLICE_REQUIRED          0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for PKE1 Slice Required Flag */

#define QAT_COMN_PKE1_SLICE_NOT_REQUIRED      0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for PKE1 Slice not Required Flag */

/* ========================================================================== */
/*                                PKE0 Slice Required Flag definitions        */
/* ========================================================================== */
#define QAT_COMN_PKE0_SLICE_REQUIRED          0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for PKE0 Slice Required Flag */

#define QAT_COMN_PKE0_SLICE_NOT_REQUIRED      0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for PKE0 Slice not Required Flag */

/* ========================================================================== */
/*                               Auth1 Slice Required Flag definitions        */
/* ========================================================================== */
#define QAT_COMN_AUTH1_SLICE_REQUIRED         0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Auth1 Slice   Required Flag */

#define QAT_COMN_AUTH1_SLICE_NOT_REQUIRED     0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Auth1 Slice  not Required Flag */

/* ========================================================================== */
/*                               Auth0 Slice Required Flag definitions        */
/* ========================================================================== */
#define QAT_COMN_AUTH0_SLICE_REQUIRED         0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Auth0 Slice  Required Flag */

#define QAT_COMN_AUTH0_SLICE_NOT_REQUIRED     0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Auth0 Slice  not Required Flag */

/* ========================================================================== */
/*                             Cipher1 Slice Required Flag definitions        */
/* ========================================================================== */
#define QAT_COMN_CIPHER1_SLICE_REQUIRED       0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Cipher1 Slice  Required Flag */

#define QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED   0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Cipher1 Slice  not Required Flag  */

/* ========================================================================== */
/*                             Cipher0 Slice Required Flag definitions        */
/* ========================================================================== */
#define QAT_COMN_CIPHER0_SLICE_REQUIRED       0x1
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Cipher0 Slice Required Flag  */

#define QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED   0x0
/**< @ingroup icp_qat_fw_comn
 * Constant Value for Cipher0 Slice not Required Flag */


 /**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro that must be used when building the flags for the common request
 *
 * @param ord   Value of the default ordering flag
 * @param ptr   Value of the pointer type flag
 * @param eord  Value of the extend ordering flag
 * @param shi   Value of the shram init flag    
 * @param rex   Value of the regex slice required flag  
 * @param xlt   Value of the translator slice required flag  
 * @param cpr   Value of the compression slice required flag  
 * @param blk   Value of the bulk slice required flag  
 * @param stg   Value of the storage slice required flag  
 * @param rnd   Value of the TRNG slice required flag   
 * @param pk1   Value of the PKE1 slice required flag 
 * @param pk0   Value of the PKE0 slice required flag  
 * @param au1   Value of the Auth1 slice required flag  
 * @param au0   Value of the Auth0 slice required flag  
 * @param ci1   Value of the Cipher1 slice required flag  
 * @param ci0   Value of the Cipher0 slice required flag  
 *****************************************************************************/

#define ICP_QAT_FW_COMN_FLAGS_BUILD(ord,ptr,eord,shi,rex,xlt,cpr,blk,               \
                                    stg,rnd,pk1,pk0,au1,au0,ci1,ci0)           \
   ((((ord) & QAT_COMN_ORD_MASK) << QAT_COMN_ORD_BITPOS)                      |\
    (((ptr) & QAT_COMN_PTR_TYPE_MASK) << QAT_COMN_PTR_TYPE_BITPOS)            |\
    (((eord) & QAT_COMN_EXT_ORD_MASK) << QAT_COMN_EXT_ORD_BITPOS)             |\
    (((shi) & QAT_COMN_SHRAM_INIT_MASK) << QAT_COMN_SHRAM_INIT_BITPOS)        |\
    (((rex) & QAT_COMN_REGEX_SLICE_MASK) << QAT_COMN_REGEX_SLICE_BITPOS)      |\
    (((xlt) & QAT_COMN_XLAT_SLICE_MASK ) << QAT_COMN_XLAT_SLICE_BITPOS)       |\
    (((cpr) & QAT_COMN_CPR_SLICE_MASK) << QAT_COMN_CPR_SLICE_BITPOS)          |\
    (((blk) & QAT_COMN_BULK_SLICE_MASK) << QAT_COMN_BULK_SLICE_BITPOS)        |\
    (((stg) & QAT_COMN_STORAGE_SLICE_MASK) << QAT_COMN_STORAGE_SLICE_BITPOS)  |\
    (((rnd) & QAT_COMN_RND_SLICE_MASK) << QAT_COMN_RND_SLICE_BITPOS)          |\
    (((pk1) & QAT_COMN_PKE1_SLICE_MASK) << QAT_COMN_PKE1_SLICE_BITPOS)        |\
    (((pk0) & QAT_COMN_PKE0_SLICE_MASK) << QAT_COMN_PKE0_SLICE_BITPOS)        |\
    (((au1) & QAT_COMN_AUTH1_SLICE_MASK) << QAT_COMN_AUTH1_SLICE_BITPOS)      |\
    (((au0) & QAT_COMN_AUTH0_SLICE_MASK) << QAT_COMN_AUTH0_SLICE_BITPOS)      |\
    (((ci1) & QAT_COMN_CIPHER1_SLICE_MASK) << QAT_COMN_CIPHER1_SLICE_BITPOS)  |\
    (((ci0) & QAT_COMN_CIPHER0_SLICE_MASK) << QAT_COMN_CIPHER0_SLICE_BITPOS))

/* ========================================================================= */
/*                                                                   GETTERS */
/* ========================================================================= */
/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of the ordering bit from the common flags
 *
 * @param flags      Flags to extract the ordering bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_ORD_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_ORD_BITPOS,QAT_COMN_ORD_MASK)

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Definition possible combinations of ordering bits in common flag.
 *
 * @description
 *      Enumeration used to indicate the possible types for ordering bits
 *		for request and response.
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_FW_ORDER_NONE=0,         /**< No ordering request: bit15=0,bit13=0*/
   ICP_QAT_FW_DEFAULT_ORDER=1,      /**< Default ordering request: bit15=1,bit13=0*/
   ICP_QAT_FW_EXTEND_ORDER=2,       /**< Extend ordering request: bit15=1,bit13=1*/
} icp_qat_fw_resp_order_t;

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of the pointer type bit from the common flags
 *
 * @param flags      Flags to extract the pointer type bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_PTR_TYPE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_PTR_TYPE_BITPOS,QAT_COMN_PTR_TYPE_MASK)
 
 
/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of shram init bit from the common flags
 *
 * @param flags      Flags to extract the shram init bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_SHRAM_INIT_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_SHRAM_INIT_BITPOS,QAT_COMN_SHRAM_INIT_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of Regex Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the Regex Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_REGEX_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_REGEX_SLICE_BITPOS, QAT_COMN_REGEX_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of Translator Slice Required bit from the common 
 *      flags
 *
 * @param flags      Flags to extract the Translator Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_XLAT_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_XLAT_SLICE_BITPOS, QAT_COMN_XLAT_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of Compression Slice Required bit from the common
 *       flags
 * 
 * @param flags      Flags to extract the Compression Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_CPR_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_CPR_SLICE_BITPOS, QAT_COMN_CPR_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of Bulk Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the Bulk Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_BULK_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_BULK_SLICE_BITPOS, QAT_COMN_BULK_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *     Macro for extraction of Storage Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the Storage Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_STORAGE_SLICE_GET(flags)      \
   QAT_FIELD_GET(flags,QAT_COMN_STORAGE_SLICE_BITPOS, \
                 QAT_COMN_STORAGE_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of TRNG Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the TRNG Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_RND_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_RND_SLICE_BITPOS, QAT_COMN_RND_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of PKE1 Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the PKE1 Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_PKE1_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_PKE1_SLICE_BITPOS, QAT_COMN_PKE1_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of PKE0 Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the PKE0 Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_PKE0_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_PKE0_SLICE_BITPOS, QAT_COMN_PKE0_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of AUTH1 Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the AUTH1 Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_AUTH1_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_AUTH1_SLICE_BITPOS, QAT_COMN_AUTH1_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of AUTH0 Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the AUTH0 Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_AUTH0_SLICE_GET(flags)   \
   QAT_FIELD_GET(flags,QAT_COMN_AUTH0_SLICE_BITPOS, QAT_COMN_AUTH0_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of Cipher1 Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the Cipher1 Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_CIPHER1_SLICE_GET(flags)      \
   QAT_FIELD_GET(flags,QAT_COMN_CIPHER1_SLICE_BITPOS, \
                 QAT_COMN_CIPHER1_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of Cipher0 Slice Required bit from the common flags
 *
 * @param flags      Flags to extract the Cipher0 Slice Required bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_CIPHER0_SLICE_GET(flags)      \
   QAT_FIELD_GET(flags,QAT_COMN_CIPHER0_SLICE_BITPOS, \
                 QAT_COMN_CIPHER0_SLICE_MASK)


/* ========================================================================= */
/*                                                                  SETTERS  */
/* ========================================================================= */
/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting the ordering bit in the common flags field
 *
 * @param flags      Flags in which Ordering Bit bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_ORD_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_ORD_BITPOS,QAT_COMN_ORD_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting the pointer type bit in the common flags
 *
 * @param flags      Flags in which Pointer Type bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_PTR_TYPE_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_PTR_TYPE_BITPOS,QAT_COMN_PTR_TYPE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting the extend ordering bit of the common flags
 *
 * @param flags    Flags to set with the ordering bit
 * @param val      Ordering value
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_EXT_ORD_SET(flags,val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_EXT_ORD_BITPOS,QAT_COMN_EXT_ORD_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting shram init bit in the common flags
 *
 * @param flags      Flags in which Shram Init Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_SHRAM_INIT_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_SHRAM_INIT_BITPOS,QAT_COMN_SHRAM_INIT_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting Regex Slice Required bit in the common flags
 *
 * @param flags      Flags in which Regex Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_REGEX_SLICE_SET(flags, val)     \
   QAT_FIELD_SET(flags,val,QAT_COMN_REGEX_SLICE_BITPOS, \
                 QAT_COMN_REGEX_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting Translator Slice Required bit in the common flags
 *      
 *
 * @param flags      Flags in which Translator Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_XLAT_SLICE_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_XLAT_SLICE_BITPOS, QAT_COMN_XLAT_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting Compression Slice Required bit in the common flags
 *      
 *
 * @param flags      Flags in which Compression Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_CPR_SLICE_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_CPR_SLICE_BITPOS, QAT_COMN_CPR_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting Bulk Slice Required bit in the common flags
 *
 * @param flags      Flags in which Bulk Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_BULK_SLICE_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_BULK_SLICE_BITPOS, QAT_COMN_BULK_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting Storage Slice Required bit in the common flags
 *
 * @param flags      Flags in which Storage Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_STORAGE_SLICE_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_STORAGE_SLICE_BITPOS, \
                 QAT_COMN_STORAGE_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting TRNG Slice Required bit in the common flags
 *
 * @param flags      Flags in which TRNG Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_RND_SLICE_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_RND_SLICE_BITPOS, QAT_COMN_RND_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting PKE1 Slice Required bit in the common flags
 *
 * @param flags      Flags in which PKE1 Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_PKE1_SLICE_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_PKE1_SLICE_BITPOS, QAT_COMN_PKE1_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting PKE0 Slice Required bit in the common flags
 *
 * @param flags      Flags in which PKE0 Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_PKE0_SLICE_SET(flags, val)   \
   QAT_FIELD_SET(flags,val,QAT_COMN_PKE0_SLICE_BITPOS, QAT_COMN_PKE0_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting AUTH1 Slice Required bit in the common flags
 *
 * @param flags      Flags in which Auth1 Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_AUTH1_SLICE_SET(flags, val)     \
   QAT_FIELD_SET(flags,val,QAT_COMN_AUTH1_SLICE_BITPOS, \
                 QAT_COMN_AUTH1_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting AUTH0 Slice Required bit in the common flags
 *
 * @param flags      Flags in which Auth0 Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_AUTH0_SLICE_SET(flags, val)    \
   QAT_FIELD_SET(flags,val,QAT_COMN_AUTH0_SLICE_BITPOS, \
                 QAT_COMN_AUTH0_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting Cipher1 Slice Required bit in the common flags
 *
 * @param flags      Flags in which Cipher1 Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_CIPHER1_SLICE_SET(flags, val)     \
   QAT_FIELD_SET(flags,val,QAT_COMN_CIPHER1_SLICE_BITPOS, \
                 QAT_COMN_CIPHER1_SLICE_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for setting Cipher0 Slice Required bit in the common flags
 *
 * @param flags      Flags in which Cipher0 Slice Required bit will be set
 * @param val        Value of the bit to be set in flags
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_CIPHER0_SLICE_SET(flags, val)     \
   QAT_FIELD_SET(flags,val,QAT_COMN_CIPHER0_SLICE_BITPOS, \
   QAT_COMN_CIPHER0_SLICE_MASK)


/*  
 *  < @ingroup icp_qat_fw_comn
 *  Common Status Field Definition  The bit offsets below are within the flags
 *  field. These are NOT relative to the memory word. Unused field must be 
 *  zeroed.
 *  + ===== + ------ + ---- + ---- + ---- + ---- + --- + ---------- +
 *  |  Bit  |   7    |  6   |  5   |  4   |  3   |  2  | [ 1 - 0 ]  |
 *  + ===== + ------ + ---- + ---- + ---- + ---- + ----+ ---------- +
 *  | Flags | Crypto | Pke  | Cmp  | Xlat |  PM  | I/A |  Reserved  |
 *  + ===== + ------ + ---- + ---- + ---- + ---- + --- + ---------- +
 * Note: 
 * For the service specific status bit definitions refer to service header files
 * Eg. Crypto Status bit refers to Symmetric Crypto, Key Generation, and NRBG
 * Requests' Status. 
 */


#define        QAT_COMN_RESP_CRYPTO_STATUS_BITPOS             7 
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Response for Crypto service Flag */

#define        QAT_COMN_RESP_CRYPTO_STATUS_MASK               0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Crypto status mask */

#define        QAT_COMN_RESP_PKE_STATUS_BITPOS                6
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Response for PKE service Flag */

#define        QAT_COMN_RESP_PKE_STATUS_MASK                  0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine PKE status mask */

#define        QAT_COMN_RESP_CMP_STATUS_BITPOS                5 
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Response for Compression service Flag */

#define        QAT_COMN_RESP_CMP_STATUS_MASK                  0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Compression status mask */

#define        QAT_COMN_RESP_XLAT_STATUS_BITPOS               4 
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Response for Xlat service Flag */

#define        QAT_COMN_RESP_XLAT_STATUS_MASK                 0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Translator status mask */

#define        QAT_COMN_RESP_PM_STATUS_BITPOS                 3 
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Response for Pattern Match service Flag */

#define        QAT_COMN_RESP_PM_STATUS_MASK                   0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Pattern Match Service status mask */

#define        QAT_COMN_RESP_INIT_ADMIN_STATUS_BITPOS         2 
/**< @ingroup icp_qat_fw_comn
 * Starting bit position indicating Response for Init and Admin service Flag */

#define        QAT_COMN_RESP_INIT_ADMIN_STATUS_MASK           0x1
/**< @ingroup icp_qat_fw_comn
 * One bit mask used to determine Init and Admin Service status mask */


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro that must be used when building the status 
 *      for the common response
 *
 * @param crypto   Value of the Crypto Service status flag
 * @param pke      Value of the Pke Service Status flag   
 * @param comp     Value of the Compression Service Status flag   
 * @param xlat     Value of the Xlator Status flag   
 * @param pm       Value of the Pattern Match Service Status flag   
 * @param ia       Value of the Init/Admin Service Status flag   

 *****************************************************************************/
#define ICP_QAT_FW_COMN_RESP_STATUS_BUILD(crypto,pke,comp,xlat,pm,ia)          \
   ((((crypto) & QAT_COMN_RESP_CRYPTO_STATUS_MASK) <<                          \
                                QAT_COMN_RESP_CRYPTO_STATUS_BITPOS)         |  \
    (((pke) & QAT_COMN_RESP_PKE_STATUS_MASK) <<                                \
                                QAT_COMN_RESP_PKE_STATUS_BITPOS)            |  \
    (((comp) & QAT_COMN_RESP_CMP_STATUS_MASK) <<                               \
                                QAT_COMN_RESP_CMP_STATUS_BITPOS)            |  \
    (((xlat) & QAT_COMN_RESP_XLAT_STATUS_MASK) <<                              \
                                QAT_COMN_RESP_XLAT_STATUS_BITPOS)           |  \
    (((pm) & QAT_COMN_RESP_PM_STATUS_MASK ) <<                                 \
                                QAT_COMN_RESP_PM_STATUS_BITPOS)             |  \
    (((ia) & QAT_COMN_RESP_INIT_ADMIN_STATUS_MASK ) <<                         \
                                QAT_COMN_RESP_INIT_ADMIN_STATUS_BITPOS))


/* ========================================================================= */
/*                                                                   GETTERS */
/* ========================================================================= */
/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of the Crypto bit from the status
 *
 * @param status 
 *      Status to extract the status bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_RESP_CRYPTO_STAT_GET(status)         \
   QAT_FIELD_GET(status, QAT_COMN_RESP_CRYPTO_STATUS_BITPOS, \
                 QAT_COMN_RESP_CRYPTO_STATUS_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of the PKE bit from the status
 *
 * @param status
 *      Status to extract the status bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_RESP_PKE_STAT_GET(status)         \
   QAT_FIELD_GET(status, QAT_COMN_RESP_PKE_STATUS_BITPOS, \
                 QAT_COMN_RESP_PKE_STATUS_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of the Compression bit from the status
 *
 * @param status 
 *      Status to extract the status bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_RESP_CMP_STAT_GET(status)         \
   QAT_FIELD_GET(status, QAT_COMN_RESP_CMP_STATUS_BITPOS, \
                 QAT_COMN_RESP_CMP_STATUS_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of the Translator bit from the status
 *
 * @param status 
 *      Status to extract the status bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_RESP_XLAT_STAT_GET(status)         \
   QAT_FIELD_GET(status, QAT_COMN_RESP_XLAT_STATUS_BITPOS, \
                 QAT_COMN_RESP_XLAT_STATUS_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of the Pattern Match Service Status bit from the 
 *        status
 *
 * @param status 
 *      Status to extract the status bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_RESP_PM_STAT_GET(status)         \
   QAT_FIELD_GET(status, QAT_COMN_RESP_PM_STATUS_BITPOS, \
                 QAT_COMN_RESP_PM_STATUS_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comn
 *
 * @description
 *      Macro for extraction of the Init/Admin Service Status bit from the 
 *      status
 *
 * @param status 
 *      Status to extract the status bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_RESP_INIT_ADMIN_STAT_GET(status)         \
   QAT_FIELD_GET(status, QAT_COMN_RESP_INIT_ADMIN_STATUS_BITPOS, \
                 QAT_COMN_RESP_INIT_ADMIN_STATUS_MASK)


/* ========================================================================= */
/*                                        Status Flag definitions */
/* ========================================================================= */

#define ICP_QAT_FW_COMN_STATUS_FLAG_OK       0
/**< @ingroup icp_qat_fw_comn
 * Definition of successful processing of a request */

#define ICP_QAT_FW_COMN_STATUS_FLAG_ERROR    1
/**< @ingroup icp_qat_fw_comn
 * Definition of erroneous processing of a request */

#define   ERR_CODE_NO_ERROR                  0
/**< Error Code constant value for no error  */

#define   ERR_CODE_OVERFLOW_ERROR            -11
/**< Error Code constant value for overflow error  */

#define   ERR_CODE_SOFT_ERROR                -12
/**< Error Code constant value for soft error  */

#define   ERR_CODE_FATAL_ERROR               -13
/**< Error Code constant value for hard/fatal error  */


/* LZ77 Error codes, comp + xltr slice only */

#define   ERR_CODE_INVALID_BLOCK_TYPE        -1
/**< Error Code constant value for lz77 error  */
/* Invalid block type (type == 3)*/

#define   ERR_CODE_NO_MATCH_ONES_COMP        -2
/**< Error Code constant value for lz77 error  */
/* Stored block length does not match one's complement*/

#define   ERR_CODE_TOO_MANY_LEN_OR_DIS       -3
/**< Error Code constant value for lz77 error  */
/* Too many length or distance codes */

#define   ERR_CODE_INCOMPLETE_LEN            -4
/**< Error Code constant value for lz77 error  */
/* Code lengths codes incomplete */

#define   ERR_CODE_RPT_LEN_NO_FIRST_LEN      -5
/**< Error Code constant value for lz77 error  */
/* Repeat lengths with no first length */

#define   ERR_CODE_RPT_GT_SPEC_LEN           -6
/**< Error Code constant value for lz77 error  */
/* Repeat more than specified lengths */

#define   ERR_CODE_INV_LIT_LEN_CODE_LEN      -7
/**< Error Code constant value for lz77 error  */
/* Invalid lit/len code lengths */

#define   ERR_CODE_INV_DIS_CODE_LEN          -8
/**< Error Code constant value for lz77 error  */
/* Invalid distance code lengths */

#define   ERR_CODE_INV_LIT_LEN_DIS_IN_BLK    -9
/**< Error Code constant value for lz77 error  */
/* Invalid lit/len or distance code in fixed/dynamic block */

#define   ERR_CODE_DIS_TOO_FAR_BACK          -10
/**< Error Code constant value for lz77 error  */
/* Distance too far back in fixed or dynamic block */


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Slice types for building of the processing chain within the content
 *      descriptor
 *
 * @description
 *      Enumeration used to indicate the ids of the slice types through which
 *      data will pass.
 *
 *      A logical slice is not a hardware slice but is a software FSM
 *      performing the actions of a slice
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_FW_SLICE_NULL=0,         /**< NULL slice type */
   ICP_QAT_FW_SLICE_CIPHER=1,       /**< CIPHER slice type */
   ICP_QAT_FW_SLICE_AUTH=2,         /**< AUTH slice type */
   ICP_QAT_FW_SLICE_DRAM_RD=3,      /**< DRAM_RD Logical slice type */
   ICP_QAT_FW_SLICE_DRAM_WR=4,      /**< DRAM_WR Logical slice type */
   ICP_QAT_FW_SLICE_COMP=5,         /**< Compression slice type */
   ICP_QAT_FW_SLICE_XLAT=6,         /**< Translator slice type */
   ICP_QAT_FW_SLICE_REGEX=7,        /**< Regular Expression slice type */
   ICP_QAT_FW_SLICE_IPSEC=8,        /**< IPsec Logical slice type */
   ICP_QAT_FW_SLICE_STORAGE=9,      /**< STORAGE Logical slice type */
   ICP_QAT_FW_SLICE_BULK=10,        /**< BULK Logical slice type */
   ICP_QAT_FW_SLICE_DELIMITER       /**< End delimiter */

} icp_qat_fw_slice_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Definition of the common request structure with service specific
 *      fields
 * @description
 *      This is a definition of the full qat request structure used by all
 *      services. Each service is free to use the service fields in its own
 *      way. This struct is useful as a message passing argument before the
 *      service contained within the request is determined.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comn_req_s
{
   icp_qat_fw_comn_req_hdr_t comn_hdr;
   /**< Common request header */

   uint32_t flow_id;
   /** < Field used by Firmware to limit the number of stateful requests 
    * for a session being processed at a given point of time            */

   uint32_t serv_session_field;
   /**< Service specific session field of the standard request */

   icp_qat_fw_comn_req_mid_t comn_mid;
   /**< Common request middle section */

   uint32_t serv_fields[(ICP_QAT_FW_REQ_DEFAULT_SZ -            \
                         sizeof(icp_qat_fw_comn_req_hdr_t) -    \
                         sizeof(uint32_t) -                     \
                         sizeof(uint32_t) -                     \
                         sizeof(icp_qat_fw_comn_req_mid_t) -    \
                         sizeof(icp_qat_fw_comn_req_ftr_t)) /   \
                         sizeof(uint32_t)];
   /**< Service Specific fields of the standard request */

   icp_qat_fw_comn_req_ftr_t comn_ftr;
   /**< Common request footer */

} icp_qat_fw_comn_req_t;


#define ICP_QAT_FW_REQ_DESC_COPY_HDR_SIZE (sizeof(icp_qat_fw_comn_req_hdr_t) + \
                                           sizeof(uint32_t) +                  \
                                           sizeof(uint32_t))
/**< @ingroup icp_qat_fw_comn
 * Number of bytes at the start of request descriptor up to the end of the
 * session service field used when copying multiple header sections
 * together which are constant for each service session */


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comn
 *      Definition of the common response structure with service specific
 *      fields
 * @description
 *      This is a definition of the full qat response structure used by all
 *      services. Each service is free to use the service fields in its own
 *      way. This struct is useful as a message passing argument before the
 *      service contained within the request is determined.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comn_resp_s
{
   icp_qat_fw_comn_resp_hdr_t comn_hdr;
   /**< Common header fields */

   uint32_t serv_fields[(ICP_QAT_FW_RESP_DEFAULT_SZ -               \
                         sizeof(icp_qat_fw_comn_resp_hdr_t)) /      \
                         sizeof(uint32_t)];
   /**< Service Specific fields of the standard request */

} icp_qat_fw_comn_resp_t;

#endif /* __ICP_QAT_FW_H__ */
