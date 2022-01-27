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
 * @file icp_qat_fw_comp.h
 * @defgroup icp_qat_fw_comp ICP QAT FW Compression Service
 *           Interface Definitions
 * @ingroup icp_qat_fw
 * @description
 *      This file documents structs used to provide the interface to the
 *      Compression QAT FW service
 *
 *****************************************************************************/

#ifndef __ICP_QAT_FW_COMP_H__
#define __ICP_QAT_FW_COMP_H__

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

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the common Compression QAT FW request descriptor 
 *        parameters
 * @description
 *        This part of the request is common across all of the Comp commands
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_comn_req_s
{
   uint16_t cmd_flags;
   /**< Definition of the common processing flags used for Compression */

   uint8_t req_params_blk_sz;
   /**<  For bulk processing this field represents the request parameters
    * block size */

   uint8_t cmd_id;
   /**< Definition of the Compression command defined by this request */

} icp_qat_fw_comp_comn_req_t;


/*
 *  REQUEST FLAGS IN COMMON COMPRESSION
 *
 *  + ===== + ----------- + -----  + ----- + ----- + -- + -- + -- +
 *  |  Bit  |   15 - 5    |  5     |  4    |   3   |  2 |  1 | 0  |
 *  + ===== + ----------- + -----  + ----- + ----- + -- + -- + -- +
 *  | Flags | Resvd Bits=0| Dis.   | Enh.  |Auto   |Sess| EOP|SOP |
 *  |       |             | Type0  | ASB   |Select |Type|    |    |
 *  |       |             | Header |       |Best   |    |    |    |
 *  + ===== + ----------- + ------ + ----- + ----- + -- + -- + -- +
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

#define ICP_QAT_FW_COMP_BFINAL_MASK								  0x1
/**< @ingroup icp_qat_fw_comp
 * One bit mask for the bfinal bit */

#define ICP_QAT_FW_COMP_BFINAL_BITPOS							  6
/**< @ingroup icp_qat_fw_comp
 * Starting bit position for the bfinal bit*/

#define ICP_QAT_FW_COMP_NOT_BFINAL                                0
/**< @ingroup icp_qat_fw_comp
 * Flag representing to indicate firmware this is not the last block */

#define ICP_QAT_FW_COMP_BFINAL                                    1
/**< @ingroup icp_qat_fw_comp
 * Flag representing to indicate firmware this is the last block */
/**
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 * Macro used for the generation of the command flags for Compression Request.
 * This should always be used for the generation of the flags. No direct sets or
 * masks should be performed on the flags data
 *
 * @param sop              SOP part of data
 * @param eop              EOP part of data
 * @param sesstype         Session Type
 * @param autoselect       AutoSelectBest 
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_FLAGS_BUILD(sop, eop, sesstype,                   \
                                    autoselect, enhanced_asb,             \
                                    ret_uncomp, bfinal)                   \
        ( ((sop & ICP_QAT_FW_COMP_SOP_MASK) <<                            \
            ICP_QAT_FW_COMP_SOP_BITPOS)                |                  \
          ((eop & ICP_QAT_FW_COMP_EOP_MASK) <<                            \
            ICP_QAT_FW_COMP_EOP_BITPOS)                |                  \
          ((sesstype & ICP_QAT_FW_COMP_SESSION_TYPE_MASK) <<              \
            ICP_QAT_FW_COMP_SESSION_TYPE_BITPOS)       |                  \
          ((autoselect & ICP_QAT_FW_COMP_AUTO_SELECT_BEST_MASK) <<        \
            ICP_QAT_FW_COMP_AUTO_SELECT_BEST_BITPOS)   |                  \
          ((enhanced_asb & ICP_QAT_FW_COMP_ENHANCED_AUTO_SELECT_BEST_MASK) << \
            ICP_QAT_FW_COMP_ENHANCED_AUTO_SELECT_BEST_BITPOS) |           \
          ((ret_uncomp & ICP_QAT_FW_COMP_RET_DISABLE_TYPE0_HEADER_DATA_MASK) << \
            ICP_QAT_FW_COMP_RET_DISABLE_TYPE0_HEADER_DATA_BITPOS) |       \
		  ((bfinal & ICP_QAT_FW_COMP_BFINAL_MASK) <<                      \
		    ICP_QAT_FW_COMP_BFINAL_BITPOS)) 


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
 ******************************************************************************
 * @ingroup icp_qat_fw_comp
 *
 * @description
 *        Macro for extraction of the session type bit
 *
 * @param flags        Flags to extract the session type bit from
 *
 *****************************************************************************/
#define ICP_QAT_FW_COMP_SESSION_TYPE_GET(flags)                        \
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_SESSION_TYPE_BITPOS,           \
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
   QAT_FIELD_GET(flags,ICP_QAT_FW_COMP_AUTO_SELECT_BEST_BITPOS,           \
                 ICP_QAT_FW_COMP_AUTO_SELECT_BEST_MASK)

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the common Compression QAT FW response descriptor 
 *        parameters
 * @description
 *        This part of the response is common across all of the Comp commands
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_comn_resp_s
{
   uint32_t input_byte_counter;
   /**< Input byte count */

   uint32_t output_byte_counter;

   /**< Output byte count */

   uint32_t current_crc32;
   /**< CRC for processed bytes (input byte count) */

   uint32_t current_adler;
   /**< Adler for processed bytes (input byte count) */

} icp_qat_fw_comp_comn_resp_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the common Compression QAT FW request
 * @description
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_req_s
{
   icp_qat_fw_comn_req_hdr_t comn_hdr;
   /**< Common request header */

   uint32_t flow_id;
   /** < Field used by Firmware to limit the number of stateful requests 
    * for a session being processed at a given point of time            */

   icp_qat_fw_comp_comn_req_t comn_req_params;
   /**< Common Comp request parameters */

   icp_qat_fw_comn_req_mid_t comn_mid;
   /**< Common request middle section */

   uint64_t req_params_addr;
   /**< Memory address of the request parameters */

   icp_qat_fw_comn_req_ftr_t comn_ftr;
   /**< Common request footer */

} icp_qat_fw_comp_req_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the Compression Response
 * @description
 *        This is the response delivered to the ET rings by the Compression
 *              QAT FW service for all commands
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_resp_s
{
   icp_qat_fw_comn_resp_hdr_t comn_resp;
   /**< Common interface response format see icp_qat_fw.h */

   icp_qat_fw_comp_comn_resp_t comn_resp_params;
   /**< Common response params, (checksums and byte counts) */

   uint32_t resrvd1[(ICP_QAT_FW_RESP_DEFAULT_SZ -                           \
                                   sizeof(icp_qat_fw_comp_comn_resp_t) -    \
                                   sizeof(icp_qat_fw_comn_resp_hdr_t))   /  \
                                   sizeof(uint32_t)];
   /**< Fields reserved for future use and assumed set to 0 */

} icp_qat_fw_comp_resp_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *      Compression header of the content descriptor header block
 * @description
 *      Definition of the structure used to describe the compression processing
 *      to perform on data. The compression parameters are defined per algorithm
 *      and are located in the icp_qat_hw.h file.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_hdr_s
{
   uint16_t offset;
   /**< Quad word offset from the content descriptor parameters address i.e.
    * (content_address + (cd_hdr_sz << 3)) to the parameters for the compression
    * processing */

   uint8_t curr_id;
   /**< Initialised with the compression slice type */

   uint8_t next_id;
   /**< Set to the next slice to pass the compressed data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * anymore slices after compression */

   uint16_t ram_flags;
   /**< Flags to show which ram banks to access */

   uint16_t resrvd;
   /**< Reserved field, should be set to zero*/

   uint64_t comp_state_addr;
   /**< Pointer to compression state */
   
   uint64_t ram_banks_addr;
   /**< Pointer to banks */

   uint32_t initial_crc32;
   /**< CRC for processed bytes (input byte count) */

   uint32_t initial_adler;
   /**< Adler for processed bytes (input byte count) */

} icp_qat_fw_comp_hdr_t;

/*Private Defines*/
#define    QAT_FW_COMP_BANK_FLAG_MASK    0x1

#define QAT_FW_COMP_COUNTERS_BITPOS   7
#define QAT_FW_COMP_BANK_G_BITPOS     6
#define QAT_FW_COMP_BANK_F_BITPOS     5
#define QAT_FW_COMP_BANK_E_BITPOS     4
#define QAT_FW_COMP_BANK_D_BITPOS     3
#define QAT_FW_COMP_BANK_C_BITPOS     2
#define QAT_FW_COMP_BANK_B_BITPOS     1
#define QAT_FW_COMP_BANK_A_BITPOS     0

#define COMP_CPR_INITIAL_CRC    0
#define COMP_CPR_INITIAL_ADLER  1
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
        
                                

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *      Translator header of the content descriptor header block
 * @description
 *      Definition of the structure used to describe the translation processing
 *      to perform on data. The translator parameters are defined per algorithm
 *      and are located in the icp_qat_hw.h file.
 *
 *****************************************************************************/
typedef struct icp_qat_fw_trans_hdr_s
{
   uint16_t offset;
   /**< Quad word offset from the content descriptor parameters address i.e.
    * (content_address + (cd_hdr_sz << 3)) to the parameters for the compression
    * processing */

   uint8_t curr_id;
   /**< Initialised with the translation slice type */

    uint8_t next_id;
   /**< Set to the next slice to pass the translated data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * anymore slices after translation */

   uint32_t resrvd;
   /**< Reserved and should be set to zero, needed for quagword alignment */

} icp_qat_fw_trans_hdr_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the compression request parameters block
 * @description
 *        Definition of the compression processing request parameters block
 *
 *****************************************************************************/
typedef struct icp_qat_fw_comp_req_params_s
{
   uint16_t resrvd;
   /**< Reserved field and assumed set to 0 */

   uint8_t curr_id;
   /**< Initialised with the compression slice type */

   uint8_t next_id;
   /**< Set to the next slice to pass the compressed data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * anymore slices after  */

   uint32_t resrvd1;
   /**< Reserved field and assumed set to 0 */

   uint32_t comp_len;
   /**< Size of input to process in bytes */

   uint32_t out_buffer_sz;
   /**< Size of output buffer in bytes */

} icp_qat_fw_comp_req_params_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *      Definition of flat_buffer_flag values
 * @description
 *      Enumeration used to define whether flat buffers are used for
 *      intermediate buffers
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_FW_INTER_USE_SGL=0,                     /*!< Use SGL Buffer */
   ICP_QAT_FW_INTER_USE_FLAT=1,                    /*!< Use Flat Buffer */
   ICP_QAT_FW_INTER_USE_DELIMITER=2                /**< Delimiter type */

} icp_qat_fw_comp_inter_buffer_type_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_fw_comp
 *        Definition of the translator request parameters block
 * @description
 *        Definition of the translator processing request parameters block
 *
 *****************************************************************************/
typedef struct icp_qat_fw_trans_req_params_s
{
   uint16_t inter_buffer_type;
   /**< Use flat buffer for intermediate buffers */

   uint8_t curr_id;
   /**< Initialised with the comnpression slice type */

   uint8_t next_id;
   /**< Set to the next slice to pass the compressed data through.
    * Set to ICP_QAT_FW_SLICE_DRAM_WR if the data is not to go through
    * anymore slices after  */

   uint32_t resrvd;
   /**< Reserved field and assumed set to 0 */

   uint64_t inter_buffer_1;
   /**< Address of intermediate buffer 1 */

   uint64_t inter_buffer_2;
   /**< Address of intermediate buffer 2 */

} icp_qat_fw_trans_req_params_t;

#endif /* __ICP_QAT_FW_COMP_H__ */
