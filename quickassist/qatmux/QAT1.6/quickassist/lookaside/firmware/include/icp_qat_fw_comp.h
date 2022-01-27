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
 * @file icp_qat_fw_comp.h
 * @defgroup icp_qat_fw_comp ICP QAT FW Compression Service
 *           Interface Definitions
 * @ingroup icp_qat_fw
 * @description
 *      This file documents structs used to provide the interface to the
 *      Compression QAT FW service
 *
 *****************************************************************************/

#ifndef _ICP_QAT_FW_COMP_H_
#define _ICP_QAT_FW_COMP_H_

/*
******************************************************************************
* Include local header files
******************************************************************************
*/
#include "icp_qat_fw.h"


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the Compression command types
 * @description
 *        Enumeration which is used to indicate the ids of functions
 *              that are exposed by the Compression QAT FW service
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_FW_COMP_CMD_STATIC=0,
   /*!< Static Compress Request */

   ICP_QAT_FW_COMP_CMD_DYNAMIC=1,
   /*!< Dynamic Compress Request */

   ICP_QAT_FW_COMP_CMD_DECOMPRESS=2,
   /*!< Decompress Request */

   ICP_QAT_FW_COMP_CMD_DELIMITER
   /**< Delimiter type */

} icp_qat_fw_comp_cmd_id_t;


/*
 *  REQUEST FLAGS IN COMMON COMPRESSION
 *
 *  + ===== + ----------- + -----  + ----- + ----- + -- + --- + --- +
 *  |  Bit  |   15 - 5    |  5     |  4    |   3   |  2 |  1  |  0  |
 *  + ===== + ----------- + -----  + ----- + ----- + -- + --- + --- +
 *  | Flags | Resvd Bits=0| Dis.   | Enh.  |Auto   |Sess|Resvd|Resvd|
 *  |       |             | Type0  | ASB   |Select |Type|  =0 |  =0 |
 *  |       |             | Header |       |Best   |    |     |     |
 *  + ===== + ----------- + ------ + ----- + ----- + -- + --- + --- +
 */
 
#define ICP_QAT_FW_COMP_STATELESS_SESSION                         0
/**< @ingroup icp_qat_fw_comp
 * Flag representing that session is stateless */

#define ICP_QAT_FW_COMP_STATEFUL_SESSION                          1
/**< @ingroup icp_qat_fw_comp
 * Flag representing that session is stateful  */
 
#define ICP_QAT_FW_COMP_NOT_AUTO_SELECT_BEST                      0
/**< @ingroup icp_qat_fw_comp
 * Flag representing that autoselectbest is NOT used */
 
#define ICP_QAT_FW_COMP_AUTO_SELECT_BEST                          1
/**< @ingroup icp_qat_fw_comp
 * Flag representing that autoselectbest is used */

#define ICP_QAT_FW_COMP_NOT_ENH_AUTO_SELECT_BEST                      0
/**< @ingroup icp_qat_fw_comp
 * Flag representing that enhanced autoselectbest is NOT used */
 
#define ICP_QAT_FW_COMP_ENH_AUTO_SELECT_BEST                          1
/**< @ingroup icp_qat_fw_comp
 * Flag representing that enhanced autoselectbest is used */

#define ICP_QAT_FW_COMP_NOT_DISABLE_TYPE0_ENH_AUTO_SELECT_BEST       0
/**< @ingroup icp_qat_fw_comp
 * Flag representing that enhanced autoselectbest is NOT used */
 
#define ICP_QAT_FW_COMP_DISABLE_TYPE0_ENH_AUTO_SELECT_BEST           1
/**< @ingroup icp_qat_fw_comp
 * Flag representing that enhanced autoselectbest is used */


 #define ICP_QAT_FW_COMP_SESSION_TYPE_BITPOS                      2
/**< @ingroup icp_qat_fw_comp
 * Starting bit position for the session type */

#define ICP_QAT_FW_COMP_SESSION_TYPE_MASK                         0x1
/**< @ingroup icp_qat_fw_comp
 * One bit mask used to determine the session type */
 
#define ICP_QAT_FW_COMP_AUTO_SELECT_BEST_BITPOS                   3
/**< @ingroup icp_qat_fw_comp
 * Starting bit position for auto select best */

#define ICP_QAT_FW_COMP_AUTO_SELECT_BEST_MASK                     0x1
/**< @ingroup icp_qat_fw_comp
 * One bit mask for auto select best */


#define ICP_QAT_FW_COMP_ENHANCED_AUTO_SELECT_BEST_BITPOS           4
/**< @ingroup icp_qat_fw_comp
 * Starting bit position for enhanced auto select best */

#define ICP_QAT_FW_COMP_ENHANCED_AUTO_SELECT_BEST_MASK            0x1
/**< @ingroup icp_qat_fw_comp
 * One bit mask for enhanced auto select best */
 
#define ICP_QAT_FW_COMP_RET_DISABLE_TYPE0_HEADER_DATA_BITPOS       5
/**< @ingroup icp_qat_fw_comp
 * Starting bit position for disabling type zero header write back 
   when Enhanced autoselect best is enabled. If set firmware does 
   not return type0 store block header, only copies src to dest. 
   (if best output is Type0) */

#define ICP_QAT_FW_COMP_RET_DISABLE_TYPE0_HEADER_DATA_MASK        0x1
/**< @ingroup icp_qat_fw_comp
 * One bit mask for auto select best */


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 * Macro used for the generation of the command flags for Compression Request.
 * This should always be used for the generation of the flags. No direct sets or
 * masks should be performed on the flags data
 *
 * @param sesstype         Session Type
 * @param autoselect       AutoSelectBest 
 * @enhanced_asb           Enhanced AutoSelectBest
 * @ret_uncomp             RetUnCompressed
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_FLAGS_BUILD(sesstype,                                   \
                                    autoselect, enhanced_asb,                   \
                                    ret_uncomp)                                 \
        ( ((sesstype & ICP_QAT_FW_COMP_SESSION_TYPE_MASK) <<                    \
            ICP_QAT_FW_COMP_SESSION_TYPE_BITPOS)       |                        \
          ((autoselect & ICP_QAT_FW_COMP_AUTO_SELECT_BEST_MASK) <<              \
            ICP_QAT_FW_COMP_AUTO_SELECT_BEST_BITPOS)   |                        \
          ((enhanced_asb & ICP_QAT_FW_COMP_ENHANCED_AUTO_SELECT_BEST_MASK) <<   \
            ICP_QAT_FW_COMP_ENHANCED_AUTO_SELECT_BEST_BITPOS) |                 \
          ((ret_uncomp & ICP_QAT_FW_COMP_RET_DISABLE_TYPE0_HEADER_DATA_MASK) << \
            ICP_QAT_FW_COMP_RET_DISABLE_TYPE0_HEADER_DATA_BITPOS))


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *        Macro for extraction of the session type bit
 *
 * @param flags        Flags to extract the session type bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_SESSION_TYPE_GET(flags)                            \
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_SESSION_TYPE_BITPOS,                \
                 ICP_QAT_FW_COMP_SESSION_TYPE_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *        Macro for extraction of the autoSelectBest bit
 *
 * @param flags        Flags to extract the autoSelectBest bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_AUTO_SELECT_BEST_GET(flags)                        \
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_AUTO_SELECT_BEST_BITPOS,            \
                 ICP_QAT_FW_COMP_AUTO_SELECT_BEST_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *        Macro for extraction of the enhanced asb bit
 *
 * @param flags        Flags to extract the enhanced asb bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_EN_ASB_GET(flags)                        \
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_ENHANCED_AUTO_SELECT_BEST_BITPOS, \
                            ICP_QAT_FW_COMP_ENHANCED_AUTO_SELECT_BEST_MASK)


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *        Macro for extraction of the RetUncomp bit
 *
 * @param flags        Flags to extract the Ret Uncomp bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_RET_UNCOMP_GET(flags)                        \
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_RET_DISABLE_TYPE0_HEADER_DATA_BITPOS, \
                            ICP_QAT_FW_COMP_RET_DISABLE_TYPE0_HEADER_DATA_MASK)

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the compression header cd pars block
 * @description
 *      Definition of the compression processing cd pars block.
 *      The structure is a service-specific implementation of the common 
 *      structure.
 *****************************************************************************/
typedef union icp_qat_fw_comp_req_hdr_cd_pars_s
{
	/**< LWs 2-5 */
	struct 
	{ 
		uint64_t content_desc_addr;
		/**< Address of the content descriptor */

		uint16_t content_desc_resrvd1;
      	/**< Content descriptor reserved field */
      
		uint8_t content_desc_params_sz;
	   	/**< Size of the content descriptor parameters in quad words. These
	   	 * parameters describe the session setup configuration info for the
	   	 * slices that this request relies upon i.e. the configuration word and
	   	 * cipher key needed by the cipher slice if there is a request for cipher
	     * processing. */
	
		uint8_t   content_desc_hdr_resrvd2;
	    /**< Content descriptor reserved field */
	    
		uint32_t	content_desc_resrvd3;
	  	/**< Content descriptor reserved field */
	} s;
	
	struct 
	{ 
		uint32_t comp_slice_cfg_word[ICP_QAT_FW_NUM_LONGWORDS_2];
		/* Compression Slice Config Word */
		
		uint32_t content_desc_resrvd4;
      	/**< Content descriptor reserved field */
       
	} sl;
   	
} icp_qat_fw_comp_req_hdr_cd_pars_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the compression request parameters block
 * @description
 *      Definition of the compression processing request parameters block.
 *      The structure below forms part of the Compression + Translation 
 *      Parameters block spanning LWs 14-21, thus differing from the common
 *      base Parameters block structure. Unused fields must be set to 0.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_req_params_s
{
   /**< LW 14 */
   uint32_t comp_len;
   /**< Size of input to process in bytes Note:  Only EOP requests can be odd for 
     *  decompression. IA must set LSB to zero for odd sized intermediate inputs */

   /**< LW 15 */
   uint32_t out_buffer_sz;
   /**< Size of output buffer in bytes */

   /**< LW 16 */
   uint32_t initial_crc32;
   /**< CRC for processed bytes (input byte count) */

   /**< LW 17 */
   uint32_t initial_adler;
   /**< Adler for processed bytes (input byte count) */

   /**< LW 18 */
   uint32_t req_par_flags;

   /**< LW 19 */	
   uint32_t rsrvd;			

} icp_qat_fw_comp_req_params_t;



/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 * Macro used for the generation of the request parameter flags.
 * This should always be used for the generation of the flags. No direct sets or
 * masks should be performed on the flags data
 *
 * @param sop		   SOP Flag, 0 restore, 1 don't restore	
 * @param eop              EOP Flag, 0 restore, 1 don't restore
 * @bfinal                 Set bfinal in this block or not
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_REQ_PARAM_FLAGS_BUILD(sop,                \
                                    eop, bfinal)                  \
        ( ((sop & ICP_QAT_FW_COMP_SOP_MASK) <<                    \
            ICP_QAT_FW_COMP_SOP_BITPOS) |                         \
          ((eop & ICP_QAT_FW_COMP_EOP_MASK) <<                    \
            ICP_QAT_FW_COMP_EOP_BITPOS) |                         \
          ((bfinal & ICP_QAT_FW_COMP_BFINAL_MASK) <<              \
            ICP_QAT_FW_COMP_BFINAL_BITPOS))

/*
 *  REQUEST FLAGS IN REQUEST PARAMETERS COMPRESSION
 *
 *  + ===== + ----------- + --- + --- + --- + --- + --- + -- + -- +
 *  |  Bit  |   15 - 7    |  6  |  5  |  4  |  3  |  2  | 1  | 0  |
 *  + ===== + ----------- + --- | --- + --- + --- + --- + -- + -- +
 *  | Flags | Resvd Bits=0|BFin |Resvd|Resvd|Resvd|Resvd|EOP |SOP |
 *  |       |             |     | =0  | =0  |  =0 |  =0 |    |    |
 *  |       |             |     |     |     |     |     |    |    |
 *  + ===== + ----------- + --- | --- + --- + --- + --- + -- + -- +
 */

#define ICP_QAT_FW_COMP_NOT_SOP                                   0
/**< @ingroup icp_qat_fw_comp
 * Flag representing that a request is NOT Start of Packet */

#define ICP_QAT_FW_COMP_SOP                                       1
/**< @ingroup icp_qat_fw_comp
 * * Flag representing that a request IS Start of Packet */

#define ICP_QAT_FW_COMP_NOT_EOP                                   0
/**< @ingroup icp_qat_fw_comp
 * Flag representing that a request is NOT Start of Packet  */

 #define ICP_QAT_FW_COMP_EOP                                      1
/**< @ingroup icp_qat_fw_comp
 * Flag representing that a request IS End of Packet  */

#define ICP_QAT_FW_COMP_NOT_BFINAL                                0
/**< @ingroup icp_qat_fw_comp
 * Flag representing to indicate firmware this is not the last block */

#define ICP_QAT_FW_COMP_BFINAL                                    1
/**< @ingroup icp_qat_fw_comp
 * Flag representing to indicate firmware this is the last block */

#define ICP_QAT_FW_COMP_SOP_BITPOS                                0
/**< @ingroup icp_qat_fw_comp
 * Starting bit position for SOP */

#define ICP_QAT_FW_COMP_SOP_MASK                                  0x1
/**< @ingroup icp_qat_fw_comp
 * One bit mask used to determine SOP */

#define ICP_QAT_FW_COMP_EOP_BITPOS                                1
/**< @ingroup icp_qat_fw_comp
 * Starting bit position for EOP */

#define ICP_QAT_FW_COMP_EOP_MASK                                  0x1
/**< @ingroup icp_qat_fw_comp
 * One bit mask used to determine EOP */

#define ICP_QAT_FW_COMP_BFINAL_MASK								  0x1
/**< @ingroup icp_qat_fw_comp
 * One bit mask for the bfinal bit */

#define ICP_QAT_FW_COMP_BFINAL_BITPOS							  6
/**< @ingroup icp_qat_fw_comp
 * Starting bit position for the bfinal bit*/

/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *        Macro for extraction of the SOP bit
 *
 * @param flags        Flags to extract the SOP bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_SOP_GET(flags)                        \
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_SOP_BITPOS,ICP_QAT_FW_COMP_SOP_MASK)
    
 /**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *        Macro for extraction of the EOP bit
 *
 * @param flags        Flags to extract the EOP bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_EOP_GET(flags)                        \
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_EOP_BITPOS,ICP_QAT_FW_COMP_EOP_MASK)
/**


 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *        Macro for extraction of the bfinal bit
 *
 * @param flags        Flags to extract the bfinal bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_BFINAL_GET(flags)                        \
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_BFINAL_BITPOS,           \
                 ICP_QAT_FW_COMP_BFINAL_MASK)

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the translator request parameters block
 * @description
 *        Definition of the translator processing request parameters block
 *        The structure below forms part of the Compression + Translation 
 *        Parameters block spanning LWs 14-21, thus differing from the common
 *        base Parameters block structure. Unused fields must be set to 0.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_xlt_req_params_s
{
   /**< LWs 20-21 */
   uint64_t inter_buff_ptr;
   /**< This field specifies the physical address of an intermediate
     *  buffer SGL array. The array contains a pair of 64-bit
     *  intermediate buffer pointers to SGL buffer descriptors, one pair
     ^  per CPM. Please refer to the CPM1.6 Firmware Interface HLD 
     *  specification for more details. */

} icp_qat_fw_xlt_req_params_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *      Compression header of the content descriptor block
 * @description
 *      Definition of the service-specific compression control block header 
 *      structure. The compression parameters are defined per algorithm
 *      and are located in the icp_qat_hw.h file. This compression
 *      cd block spans LWs 24-29, forming part of the compression + translation 
 *      cd block, thus differing from the common base content descriptor 
 *      structure.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_cd_hdr_s
{
   /**< LW 24 */	
   uint16_t ram_bank_flags;
   /**< Flags to show which ram banks to access */

   uint8_t comp_cfg_offset;
   /**< Quad word offset from the content descriptor parameters address to the 
    * parameters for the compression processing */

   uint8_t next_curr_id;
  /**< This field combines the next and current id (each four bits) - 
    * the next id is the most significant nibble.
    * Next Id:  Set to the next slice to pass the compressed data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * anymore slices after compression 
    * Current Id: Initialised with the compression slice type */

   /**< LW 25 */
   uint32_t resrvd;

   /**< LWs 26-27 */
   uint64_t comp_state_addr;
   /**< Pointer to compression state */
   
   /**< LWs 28-29 */
   uint64_t ram_banks_addr;
   /**< Pointer to banks */
   
} icp_qat_fw_comp_cd_hdr_t;

#define COMP_CPR_INITIAL_CRC    0
#define COMP_CPR_INITIAL_ADLER  1

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *      Translator content descriptor header block
 * @description
 *      Definition of the structure used to describe the translation processing
 *      to perform on data. The translator parameters are defined per algorithm
 *      and are located in the icp_qat_hw.h file. This translation cd block spans 
 *      LWs 30-31, forming part of the compression + translation cd block, thus 
 *      differing from the common base content descriptor structure.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_xlt_cd_hdr_s
{
   /**< LW 30 */
   uint16_t resrvd1;
   /**< Reserved field and assumed set to 0 */

   uint8_t resrvd2;
   /**< Reserved field and assumed set to 0 */


   uint8_t next_curr_id;
  /**< This field combines the next and current id (each four bits) - 
    * the next id is the most significant nibble.
    * Next Id:  Set to the next slice to pass the translated data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * any more slices after compression 
    * Current Id: Initialised with the translation slice type */

   /**< LW 31 */
   uint32_t resrvd3;
   /**< Reserved and should be set to zero, needed for quadword alignment */

} icp_qat_fw_xlt_cd_hdr_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the common Compression QAT FW request
 * @description
 *        This is a definition of the full request structure for
 *        compression and translation.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_req_s
{
   	/**< LWs 0-1 */
   icp_qat_fw_comn_req_hdr_t comn_hdr;
    /**< Common request header - for Service Command Id, 
     * use service-specific Compression Command Id.
     * Service Specific Flags - use Compression Command Flags */
 
  	/**< LWs 2-5 */		 
    icp_qat_fw_comp_req_hdr_cd_pars_t 	cd_pars;      
    /**< Compression service-specific content descriptor field which points 
     * either to a content descriptor parameter block or contains the 
     * compression slice config word. */
 
	/**< LWs 6-13 */
   icp_qat_fw_comn_req_mid_t comn_mid;
   /**< Common request middle section */

	/**< LWs 14-19 */
	icp_qat_fw_comp_req_params_t comp_pars;
	/**< Compression request Parameters block */

	/**< LWs 20-21 */
    union
    {
    	icp_qat_fw_xlt_req_params_t xlt_pars;
		/**< Translation request Parameters block */
     	
	    uint32_t resrvd1[ICP_QAT_FW_NUM_LONGWORDS_2];
	    /**< Reserved if not used for translation */
	     	
    } u1;

	/**< LWs 22-23 */
    uint32_t resrvd2[ICP_QAT_FW_NUM_LONGWORDS_2];
    /**< Reserved - not used */

	/**< LWs 24-29 */
	icp_qat_fw_comp_cd_hdr_t comp_cd_ctrl;
	/**< Compression request content descriptor control 
	 * block header */

	/**< LWs 30-31 */
    union
    {
    	icp_qat_fw_xlt_cd_hdr_t xlt_cd_ctrl;
		/**< Translation request content descriptor 
		 * control block header */
     	
	    uint32_t resrvd3[ICP_QAT_FW_NUM_LONGWORDS_2];
	    /**< Reserved if not used for translation */
	     	
    } u2;

} icp_qat_fw_comp_req_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the compression QAT FW response descriptor
 *        parameters
 * @description
 *        This part of the response is specific to the compression response.
  *
 *****************************************************************************/
typedef struct icp_qat_fw_resp_comp_pars_s
{
   /**< LW 4 */
   uint32_t input_byte_counter;
   /**< Input byte counter */

   /**< LW 5 */
   uint32_t output_byte_counter;
   /**< Output byte counter */

   /**< LW 6 */
   uint32_t curr_crc32;
   /**< Current CRC32 */
   
   /**< LW 7 */
   uint32_t curr_adler_32;
  /**< Current Adler32 */
	
} icp_qat_fw_resp_comp_pars_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the Compression Eagle Tail Response
 * @description
 *        This is the response delivered to the ET rings by the Compression
 *              QAT FW service for all commands
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_resp_s
{
   /**< LWs 0-1 */
   icp_qat_fw_comn_resp_hdr_t comn_resp;
   /**< Common interface response format see icp_qat_fw.h */

   /**< LWs 2-3 */
   uint64_t opaque_data;
   /**< Opaque data passed from the request to the response message */
  
   /**< LWs 4-7 */
   icp_qat_fw_resp_comp_pars_t comp_resp_pars;
   /**< Common response params (checksums and byte counts) */

} icp_qat_fw_comp_resp_t;


/* RAM Bank defines */
#define    QAT_FW_COMP_BANK_FLAG_MASK    0x1

#define QAT_FW_COMP_COUNTERS_BITPOS   7
#define QAT_FW_COMP_BANK_G_BITPOS     6
#define QAT_FW_COMP_BANK_F_BITPOS     5
#define QAT_FW_COMP_BANK_E_BITPOS     4
#define QAT_FW_COMP_BANK_D_BITPOS     3
#define QAT_FW_COMP_BANK_C_BITPOS     2
#define QAT_FW_COMP_BANK_B_BITPOS     1
#define QAT_FW_COMP_BANK_A_BITPOS     0


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *      Definition of the ram bank enabled values
 * @description
 *      Enumeration used to define whether a ram bank is enabled or not
 *
 *****************************************************************************/
typedef enum
{
   ICP_QAT_FW_COMP_BANK_DISABLED=0,                /*!< BANK DISABLED */
   ICP_QAT_FW_COMP_BANK_ENABLED=1,                 /*!< BANK ENABLED */
   ICP_QAT_FW_COMP_BANK_DELIMITER=2                /**< Delimiter type */

} icp_qat_fw_comp_bank_enabled_t;


/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *      Build the ram bank flags in the compression content descriptor
 *      which specify which banks are used to save history
 *
 * @param counters_enable
 * @param bank_g_enable
 * @param bank_f_enable
 * @param bank_e_enable
 * @param bank_d_enable
 * @param bank_c_enable
 * @param bank_b_enable 
 * @param bank_a_enable
 *****************************************************************************/
#define ICP_QAT_FW_COMP_RAM_FLAGS_BUILD(counters_enable,bank_g_enable,       \
                                        bank_f_enable,bank_e_enable,         \
                                        bank_d_enable, bank_c_enable,        \
                                        bank_b_enable,bank_a_enable)         \
        ((((counters_enable) & QAT_FW_COMP_BANK_FLAG_MASK) <<                \
                                QAT_FW_COMP_COUNTERS_BITPOS )|               \
        (((bank_g_enable) & QAT_FW_COMP_BANK_FLAG_MASK) <<                   \
                                QAT_FW_COMP_BANK_G_BITPOS )  |               \
        (((bank_f_enable) & QAT_FW_COMP_BANK_FLAG_MASK) <<                   \
                                QAT_FW_COMP_BANK_F_BITPOS )  |               \
        (((bank_e_enable) & QAT_FW_COMP_BANK_FLAG_MASK) <<                   \
                                QAT_FW_COMP_BANK_E_BITPOS )  |               \
        (((bank_d_enable) & QAT_FW_COMP_BANK_FLAG_MASK) <<                   \
                                QAT_FW_COMP_BANK_D_BITPOS )  |               \
        (((bank_c_enable) & QAT_FW_COMP_BANK_FLAG_MASK) <<                   \
                                QAT_FW_COMP_BANK_C_BITPOS )  |               \
        (((bank_b_enable) & QAT_FW_COMP_BANK_FLAG_MASK) <<                   \
                                QAT_FW_COMP_BANK_B_BITPOS )  |               \
        (((bank_a_enable) & QAT_FW_COMP_BANK_FLAG_MASK) <<                   \
                                QAT_FW_COMP_BANK_A_BITPOS ))
        
#endif /* _ICP_QAT_FW_COMP_H_ */
