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
 * @file icp_qat_fw_init.h
 * @defgroup icp_qat_fw_init ICP QAT FW Initialisation Interface Definitions
 * @ingroup icp_qat_fw
 *
 * @description
 *      This file documents structs used at init time in the configuration of
 *      the QAT FW.
 *
 *****************************************************************************/

#ifndef __ICP_QAT_FW_INIT_H__
#define __ICP_QAT_FW_INIT_H__

/*
******************************************************************************
* Include local header files
******************************************************************************
*/


#include "icp_qat_fw.h"

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      Definition of the init time command types
 * @description
 *      Enumeration which is used to indicate the ids of commands that are
 *      processed at init time
 *    
 *****************************************************************************/


typedef enum
{
   ICP_QAT_FW_INIT_CMD_SET_AE_INFO=0,
   /**< Setup AE Info command type */

   ICP_QAT_FW_INIT_CMD_SET_RING_INFO=1,
   /**< Setup Ring Info command type */

   ICP_QAT_FW_INIT_CMD_TRNG_ENABLE=2,
   /**< TRNG Enable command type */

   ICP_QAT_FW_INIT_CMD_TRNG_DISABLE=3,
   /**< TRNG Disable command type */

   ICP_QAT_FW_INIT_CMD_DELIMITER=4
   /**< Delimiter type */

} icp_qat_fw_init_cmd_id_t;


/* ========================================================================= */
/*                                   Ring Polling setup definitions */
/* ========================================================================= */

/**< @ingroup icp_qat_fw_init
 * Number of entries in the polling ring table */
#define INIT_RING_TABLE_SZ                      (128)

/**< @ingroup icp_qat_fw_init
 * Definition of a NULL ring entry in the ring polling table. Note ensure that
 * this field is the correct endianness in the ring table */
#define ICP_QAT_FW_INIT_NULL_RING_ENTRY         (0xFF)


/**< @ingroup icp_qat_fw_init
 * Definition of the size of the bitvector array to hold 128bit ring mask table 
 * (Size in Longwords) */
#define INIT_RING_MASK_TABLE_LW_SZ              (4)

/**< @ingroup icp_qat_fw_init
 * Definition of the Size of mask table entry in bits used to calculate the bit 
 * index of the ring mask */
#define INIT_RING_MASK_TABLE_ENTRY_BIT_SZ       (32)

/**< @ingroup icp_qat_fw_init
 * Log2 of ring mask table entry bit size used to calculate the table index in 
 * ring table mask array*/ 
#define INIT_RING_MASK_TABLE_ENTRY_LOG_SZ       (5)

/**< @ingroup icp_qat_fw_init
 * Definition of the mask to calculate the mod 32 of the ring id. */ 
#define INIT_RING_MASK_TABLE_ENTRY_MOD_MASK     (0x1F)

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init ring parameters entry
 * @description
 *      This struct contains fields neccesary to initialize the ring table
 *
 *****************************************************************************/
typedef struct icp_qat_fw_init_ring_params_s
{
   uint8_t curr_weight;
   /**< Current ring weight (working copy), has to be equal to init_weight */

   uint8_t init_weight;
   /**< Initial ring weight: -1 ... 0 */
   /**< -1 is equal to FF, -2 is equal to FE, the weighting uses negative logic
    *  where FF means poll the ring once, -2 is poll the ring twice, 0 is poll
    *  the ring 255 times                                                   */

   uint8_t ring_pvl;
   /**< Ring Privilege Level. */

   uint8_t resrvd;
   /**< Reserved field which must be set to 0 by the client */

} icp_qat_fw_init_ring_params_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init ring polling table
 * @description
 *      This struct contains list of rings that the QAT FW will continuously
 *      poll checking for request demand
 *
 *****************************************************************************/
typedef struct icp_qat_fw_init_ring_table_s
{
   icp_qat_fw_init_ring_params_t bulk_rings[INIT_RING_TABLE_SZ];
   /**< An array of ring parameters */

   uint32_t ring_mask[INIT_RING_MASK_TABLE_LW_SZ];
   /**< Structure to hold the bit masks for 128 rings. */

} icp_qat_fw_init_ring_table_t;


/**
 *****************************************************************************
 * Ring Mask Table Accessing Macros.
 * These macros are used to access the ring_mask member of ring_table
 * structure.
 *
 *****************************************************************************/

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_init
 *
 * @description
 *        Macro used to set one bit Ring Mask entry in 
 *        icp_qat_fw_init_ring_table_t structure.
 *
 * @param ring_table_t   Instance of icp_qat_fw_init_ring_table_t structure
 * @param ring_id        Id of the ring to set the mask. [0-127]
 *****************************************************************************/
#define ICP_QAT_FW_INIT_RING_MASK_SET(ring_table_t, ring_id)              \
   ring_table_t.ring_mask[ring_id >> INIT_RING_MASK_TABLE_ENTRY_LOG_SZ] = \
   ring_table_t.ring_mask[ring_id >> INIT_RING_MASK_TABLE_ENTRY_LOG_SZ] | \
   (1 << (ring_id & 0x1f))


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_init
 *
 * @description
 *        Macro used to get one bit Ring Mask entry in 
 *        icp_qat_fw_init_ring_table_t structure.
 *
 * @param ring_table_t   Instance of icp_qat_fw_init_ring_table_t structure
 * @param ring_id        Id of the ring to set the mask. [0-127]
 *****************************************************************************/
#define ICP_QAT_FW_INIT_RING_MASK_GET(ring_table_t, ring_id)               \
   (ring_table_t.ring_mask[ring_id >> INIT_RING_MASK_TABLE_ENTRY_LOG_SZ] & \
   (1 << (ring_id & 0x1f))) >> (ring_id &0x1f)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_init
 *
 * @description
 *        Macro used to clear one bit Ring Mask entry in 
 *        icp_qat_fw_init_ring_table_t structure.
 *
 * @param ring_table_t   Instance of icp_qat_fw_init_ring_table_t structure
 * @param ring_id        Id of the ring to set the mask. [0-127]
 *****************************************************************************/
#define ICP_QAT_FW_INIT_RING_MASK_CLEAR(ring_table_t, ring_id)            \
   ring_table_t.ring_mask[ring_id >> INIT_RING_MASK_TABLE_ENTRY_LOG_SZ] = \
   ring_table_t.ring_mask[ring_id >> INIT_RING_MASK_TABLE_ENTRY_LOG_SZ] & \
   ~(1 << (ring_id & 0x1f))

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init Set AE info request entries - header portion  
 * @description
 *      This struct contains structure to access init set AE info type 
 *      request entries 
 *
 *****************************************************************************/ 
typedef struct icp_qat_fw_init_set_ae_info_hdr_s
{
   uint16_t   init_slice_mask; 
   /**< Init time flags to set the ownership of the slices */

   uint16_t   resrvd;
   /**< Reserved field and must be set to 0 by the client */

   uint8_t    init_qat_id;
   /**< Init time qat id described in the request */

   uint8_t    init_ring_cluster_id; 
   /**< Init time ring cluster Id */

   uint8_t    init_trgt_id; 
   /**< Init time target AE id described in the request */

   uint8_t    init_cmd_id; 
   /**< Init time command that is described in the request */

} icp_qat_fw_init_set_ae_info_hdr_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init Set AE info request entries  
 * @description
 *      This struct contains structure to access init set AE info type 
 *      request entries 
 *
 *****************************************************************************/ 
typedef struct icp_qat_fw_init_set_ae_info_s
{
   uint64_t   init_shram_mask; 
   /**< Init time shram mask to set the page ownership in page pool of AE*/

   uint64_t   resrvd;
   /**< Reserved field and must be set to 0 by the client */

} icp_qat_fw_init_set_ae_info_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init ring info request entries - header portion
 * @description
 *      This struct contains structure to access init ring info type of 
 *      request entries. 
 *
 *****************************************************************************/ 
typedef struct icp_qat_fw_init_set_ring_info_hdr_s
{
   uint32_t    resrvd; 
   /**< Reserved field and must be set to 0 by the client */

   uint16_t    init_ring_tbl_sz; 
   /**< Init time information to state size of the ring table */

   uint8_t     init_trgt_id; 
   /**< Init time target AE id described in the request */

   uint8_t     init_cmd_id; 
   /**< Init time command that is described in the request */

} icp_qat_fw_init_set_ring_info_hdr_t; 

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init ring info request enteries  
 * @description
 *      This struct contains structure to access init ring info type of 
 *      request entries. 
 *
 *****************************************************************************/ 
typedef struct icp_qat_fw_init_set_ring_info_s
{
   uint64_t    init_ring_table_ptr; 
   /**< Pointer to weighting information for 128 rings  */

   uint64_t    resrvd; 
   /**< Reserved field and must be set to 0 by the client */

} icp_qat_fw_init_set_ring_info_t; 


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init TRNG Enable/Disable request entries  
 * @description
 *      This struct contains structure to access init ring info type of 
 *      request entries. 
 *
 *****************************************************************************/ 
typedef struct icp_qat_fw_init_trng_hdr_s
{
   uint32_t    resrvd; 
   /**< Reserved field and must be set to 0 by the client */

   union 
   {
     uint8_t        resrvd; 
     /**< Reserved field set to if cmd type is trng disable */
     
     uint8_t        init_trng_cfg_sz; 
     /**< Size of the trng config word in QW*/

   } u;

   uint8_t    resrvd1; 
   /**< Reserved field and must be set to 0 by the client */

   uint8_t     init_trgt_id; 
   /**< Init time target AE id described in the request */

   uint8_t     init_cmd_id; 
   /**< Init time command that is described in the request */

} icp_qat_fw_init_trng_hdr_t; 


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init TRNG Enable/Disable request entries  
 * @description
 *      This struct contains structure to access init ring info type of 
 *      request entries. 
 *
 *****************************************************************************/ 
typedef struct icp_qat_fw_init_trng_s
{
   union 
   {
     uint64_t        resrvd; 
     /**< Reserved field set to 0 if cmd type is trng disable */

     uint64_t        init_trng_cfg_ptr; 
     /**< Pointer to TRNG Slice config word*/

   } u; 

   uint64_t    resrvd; 
   /**< Reserved field and must be set to 0 by the client */

} icp_qat_fw_init_trng_t; 


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init time request
 * @description
 *      This struct contains data needed to generate a init time request
 *
 *****************************************************************************/
typedef struct icp_qat_fw_init_req_s
{
   icp_qat_fw_comn_req_hdr_t comn_hdr;
   /**< Common request header */

   union 
   {
      icp_qat_fw_init_set_ae_info_hdr_t     set_ae_info;
      /**< INIT SET_AE_INFO request header structure */
      
      icp_qat_fw_init_set_ring_info_hdr_t   set_ring_info;
      /**< INIT SET_RING_INFO request header structure */
      
      icp_qat_fw_init_trng_hdr_t            init_trng;
      /**< INIT TRNG ENABLE/DISABLE request header structure */

   } u;

   icp_qat_fw_comn_req_mid_t comn_mid;
   /**< Common request middle section */

   union 
   {
      icp_qat_fw_init_set_ae_info_t         set_ae_info;
      /**< INIT SET_AE_INFO request data structure */
      
      icp_qat_fw_init_set_ring_info_t       set_ring_info;
      /**< INIT SET_RING_INFO request data structure */
      
      icp_qat_fw_init_trng_t                init_trng;
      /**< INIT TRNG ENABLE/DISABLE request data structure */

   } u1;
   
} icp_qat_fw_init_req_t;

 
/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init
 *      QAT FW Init response
 * @description
 *      This struct contains data needed to generate a init response.
 *      These always go to the ET rings with size ICP_QAT_FW_RESP_DEFAULT_SZ
 *      bytes
 *
 *****************************************************************************/
typedef struct icp_qat_fw_init_resp_s
{
   icp_qat_fw_comn_resp_hdr_t comn_resp;
   /**< Common interface response */

   uint8_t resrvd[ICP_QAT_FW_RESP_DEFAULT_SZ - \
                  sizeof(icp_qat_fw_comn_resp_hdr_t)];
   /**< Reserved padding out to the default response size */

} icp_qat_fw_init_resp_t;


#endif  /* __ICP_QAT_FW_INIT_H__ */
