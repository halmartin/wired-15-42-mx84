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
 *  version: QAT1.6.L.2.6.0-65
 */

/**
 *****************************************************************************
 * @file icp_qat_fw_init_admin.h
 * @defgroup icp_qat_fw_init_admin ICP QAT FW Initialisation/Admin Interface Definitions
 * @ingroup icp_qat_fw
 *
 * @description
 *      This file documents structs used at init time in the configuration of
 *      the QAT FW, as well as for Admin requests and responses.
 *
 *****************************************************************************/

#ifndef _ICP_QAT_FW_INIT_ADMIN_H_
#define _ICP_QAT_FW_INIT_ADMIN_H_

/*
******************************************************************************
* Include local header files
******************************************************************************
*/

#include "icp_qat_fw.h"


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *      Definition of the init time and admin command types
 * @description
 *      Enumeration which is used to indicate the ids of Init/Admin commands
 *    
 *****************************************************************************/
typedef enum
{
   ICP_QAT_FW_INIT_ME=0,
   /**< ME Initialisation command type */

   ICP_QAT_FW_TRNG_ENABLE=1,
   /**< TRNG Enable command type */

   ICP_QAT_FW_TRNG_DISABLE=2,
   /**< TRNG Disable command type */

   ICP_QAT_FW_CONSTANTS_CFG=3,
   /**< Constants configuration command type */
   
   ICP_QAT_FW_STATUS_GET=4,
  /**< Admin: Status Get command type */

   ICP_QAT_FW_COUNTERS_GET=5,
   /**< Admin: Counters Get command type */

   ICP_QAT_FW_LOOPBACK=6,
   /**< Admin: Loopback command type */

   ICP_QAT_FW_HEARTBEAT_SYNC=7, 
   /**< Admin: Heartbeat Sync command type */

   ICP_QAT_FW_HEARTBEAT_GET=8
   /**< Admin: Heartbeat Get command type */

} icp_qat_fw_init_admin_cmd_id_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *      Definition of Init/Admin Response status
 * @description
 *      Enumeration which is used to indicate the possible values of the status 
 *      field within an Init/Admin Response message.
 *    
 *****************************************************************************/
typedef enum
{
   ICP_QAT_FW_INIT_RESP_STATUS_SUCCESS=0,
   /**< ME Initialisation/Admin response indication successful status */

   ICP_QAT_FW_INIT_RESP_STATUS_FAIL
   /**< ME Initialisation/Admin response indication failure status */
   
} icp_qat_fw_init_admin_resp_status_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *      QAT FW Init/Admin request message
 * @description
 *      This struct contains data needed to generate a init/admin request
 *
 *****************************************************************************/
typedef struct icp_qat_fw_init_admin_req_s
{
    /**< LW0 */
    uint16_t        init_cfg_sz; 
    /**< Initialisation config size */

    uint8_t        resrvd1; 
    /**< Reserved field */

    uint8_t     init_admin_cmd_id; 
    /**< Init/Admin time command that is described in the request */

    /**< LW1 */
    uint32_t    resrvd2; 
    /**< Reserved field -to keep 64-bit ptr alignment */

	/**< LWs 2-3 */
	uint64_t opaque_data;

    /**< LWs 4-5 */
       uint64_t        init_cfg_ptr; 
       /**< Pointer to configuration data */
     
    /**< LWs 6-7 */
    uint64_t    resrvd3; 
    /**< Reserved field and must be set to 0 by the client */
   
} icp_qat_fw_init_admin_req_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *      QAT FW Init/Admin response header
 * @description
 *      Structure containing the data for the Initialisation/Admin Response 
 *      message header.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_init_admin_resp_hdr_s
{
	/**< LW0 */
    uint8_t     flags;
    /**< Flags field */

    uint8_t     resrvd1;
    /**< Reserved field */

    uint8_t     status; 
    /**< Status field */

    uint8_t     init_admin_cmd_id; 
    /**< Init/Admin time command that is described in the request */
    
} icp_qat_fw_init_admin_resp_hdr_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *      QAT FW Init/Admin response Parameters
 * @description
 *      Structure containing the data for the Initialisation/Admin Response 
 *      message Parameters field.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_init_admin_resp_pars_s
{
	/**< LWs 4-7 */
    union
    {
        uint32_t     resrvd1[ICP_QAT_FW_NUM_LONGWORDS_4]; 
        /**< Reserved fields - unused by all init admin common responses */
        
        /**< Specific to an Admin Status Get Response only */
        struct
        {
            uint32_t version_patch_num;
            /**< QAT FW build patch number */
            
            uint8_t context_id;
            /**< Context id of the context that serviced the status request */

            uint8_t ae_id;
            /**< id of the acceleration engine that serviced the status request */
            
            uint16_t     resrvd1; 
            /**< Reserved field */
            
            uint64_t     resrvd2; 
            /**< Now a reserved field */
            
        } s1;  

        /**< Specific to an Admin Counters Get Response only */
        struct
        {
            uint64_t     req_rec_count; 
            /**< Request received count */
        
            uint64_t     resp_sent_count; 
            /**< Response sent count */
            
        } s2;  
    
    } u;
           
} icp_qat_fw_init_admin_resp_pars_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *      QAT FW Init/Admin response
 * @description
 *      This struct contains data needed to generate a init/admin response
 *
 *****************************************************************************/
typedef struct icp_qat_fw_init_admin_resp_s
{
    /**< LW0 */
    icp_qat_fw_init_admin_resp_hdr_t init_resp_hdr;
    /**< Initialisation/Admin response header */

    /**< LW1 */
    union
    {
     	uint32_t resrvd2;
        /**< Reserved field - to keep 64-bit ptr alignment 
         * - specific to all init common responses */
    	
        /**< Specific to an Admin Status Get response only */
    	struct
    	{
		    uint16_t version_minor_num;
		    /**< QAT FW minor build number */
		
		    uint16_t version_major_num;
		    /**< QAT FW major build number */
		    
    	} s;
    	
    } u;

	/**< LWs 2-3 */
	uint64_t opaque_data;
    
    /**< LWs 4-7 */
    icp_qat_fw_init_admin_resp_pars_t init_resp_pars;
    /**< Initialisation/Admin response parameters */
       
} icp_qat_fw_init_admin_resp_t;


/* ========================================================================= */
/*                              HEARTBEAT MACROS                             */
/* ========================================================================= */
/**< @ingroup icp_qat_fw_init_admin
 *  Definition of the setting of the Init-Admin response 
    heartbeat flag to OK */
#define ICP_QAT_FW_COMN_HEARTBEAT_OK                   0
/**< @ingroup icp_qat_fw_init_admin
 *  Definition of the setting of the Init-Admin response 
    heartbeat flag to BLOCKED */
#define ICP_QAT_FW_COMN_HEARTBEAT_BLOCKED              1

/**< @ingroup icp_qat_fw_init_admin
 * Macros defining the bit position and mask of the Init-Admin response 
   heartbeat flag within the flags field */
#define ICP_QAT_FW_COMN_HEARTBEAT_FLAG_BITPOS          0
#define ICP_QAT_FW_COMN_HEARTBEAT_FLAG_MASK            0x1
#define ICP_QAT_FW_COMN_STATUS_RESRVD_FLD_MASK         0xFE

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *
 * @description
 *      Extract the heartbeat flag from the Init-Admin response header structure. 
 *
 * @param hdr_t  Response header structure 'icp_qat_fw_init_admin_resp_hdr_t'.
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_HEARTBEAT_HDR_FLAG_GET(hdr_t)                     \
        ICP_QAT_FW_COMN_HEARTBEAT_FLAG_GET(hdr_t.flags)
        

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *
 * @description
 *      Extract the heartbeat flag from the Init-Admin response header structure. 
 *
 * @param hdr_t  Response header structure 'icp_qat_fw_init_admin_resp_hdr_t'.
 *               Value of the heartbeat flag.
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_HEARTBEAT_HDR_FLAG_SET(hdr_t, val)                \
        ICP_QAT_FW_COMN_HEARTBEAT_FLAG_SET(hdr_t, val)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_init_admin
 *
 * @description
 *      Extract the heartbeat flag from the Init-Admin response header structure
 *      status field.
 *
 * @param hdr_t  Status flags field.
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMN_HEARTBEAT_FLAG_GET(flags)                        \
        QAT_FIELD_GET(flags,                                             \
                 ICP_QAT_FW_COMN_HEARTBEAT_FLAG_BITPOS,                   \
                 ICP_QAT_FW_COMN_HEARTBEAT_FLAG_MASK)


#endif  /* _ICP_QAT_FW_INIT_ADMIN_H_ */
