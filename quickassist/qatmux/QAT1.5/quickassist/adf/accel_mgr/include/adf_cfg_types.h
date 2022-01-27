/******************************************************************************
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
 *  version: QAT1.5.L.1.11.0-36
 *
 *****************************************************************************/

/******************************************************************************
 * @file adf_cfg_types.h
 *
 * @description
 *      This is the header file for the ADF Resource Configuration Table types.
 *
 *****************************************************************************/
#ifndef ADF_CFG_TYPES_H
#define ADF_CFG_TYPES_H


#define ADF_CFG_SECTION u.section
#define ADF_CFG_VALUE   u.entry
#define ADF_CFG_HEADER_VERIFIER 0xADFCFADF
/******************************************************************************
* Section for struct definitions
******************************************************************************/
typedef enum
{
        ADF_DEC,
        ADF_HEX,
        ADF_STR
} adf_cfg_val_type_t;

typedef struct adf_cfg_key_val_s
{
        char key[ADF_CFG_MAX_KEY_LEN_IN_BYTES];
        char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
        union {
            adf_cfg_val_type_t type;
            uint64_t padding_type;
        };
        /* User space pointer where the data will be

         * copied to */
        union {
            int8_t *user_data_ptr;
            uint64_t padding_user_data_ptr;
        };
        union {
            struct adf_cfg_key_val_s *pPrev;
            uint64_t padding_pPrev;
        };
        union {
            struct adf_cfg_key_val_s *pNext;
            uint64_t padding_pNext;
        };
} adf_cfg_key_val_t;

typedef struct adf_cfg_section_s
{
        char name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES];
        union {
            adf_cfg_key_val_t *params;
            uint64_t padding_param;
        };
        union{
            struct adf_cfg_section_s *pPrev;
            uint64_t padding_pPrev;
        };
        union{
            struct adf_cfg_section_s *pNext;
            uint64_t padding_pNext;
        };
} adf_cfg_section_t;

/*
 * Data plane config types
 */
/* Data plane entry type */
typedef enum adf_dp_config_entry_type_e
{
    ADF_DP_ENTRY_TYPE_SECTION = 1,
    ADF_DP_ENTRY_TYPE_VALUE,
    ADF_DP_ENTRY_TYPE_END
} adf_dp_config_entry_type_t;

/* Data plane config header */
typedef struct adf_dp_config_header_s
{
    Cpa8U   dev_id;
    Cpa32U  valid;
    Cpa32U  number_of_sections;
} adf_dp_config_header_t;

/* Data plane config entry */
typedef struct adf_dp_config_entry_s
{
    union {
        adf_dp_config_entry_type_t type;
        uint64_t padding_type;
    };
    union{
        struct {
            int8_t name [ADF_CFG_MAX_SECTION_LEN_IN_BYTES];
            Cpa32U number_of_values;
        } section;
        struct {
            int8_t name [ADF_CFG_MAX_KEY_LEN_IN_BYTES];
            int8_t value [ADF_CFG_MAX_VAL_LEN_IN_BYTES];
        } entry;
    } u;
} adf_dp_config_entry_t;

#endif /* ADF_CFG_TYPES_H */
