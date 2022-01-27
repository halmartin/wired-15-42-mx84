/**
 **************************************************************************
 * @file uclo_suof.h
 *
 * @description
 *      This is a header file for SUOF Object File Loader 
 *
 * @par 
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
 **************************************************************************/ 

#ifndef __UCLO_SUOF_H__
#define __UCLO_SUOF_H__

#ifdef __cplusplus
extern "C"{
#endif

#define SIMG_OFFSET_2_DESC 0x80
#define RESV_DESC_2_SIMG (SIMG_OFFSET_2_DESC - sizeof(fw_auth_desc_t) - sizeof(uint64))

/**
 * @description suof object header
*/
typedef struct suof_img_hdr_s {
    long        simgBuf;             /**< signed image pointer */
    uint64      simgLen;
    
    long        cssHeader;           /**< CSS header pointer */
    long        cssKey;              /**< key pointer */
    long        cssSignature;        /**< signature pointer */

    long        cssImg;              /**< image pointer */
    uint64      imgSize;

    unsigned int aeNum;
    unsigned int aeMask;

    unsigned int fwType;

    long        pImgName;
    long        pAppMetaData;    
} suof_img_hdr_t;

/**
 * @description suof table
*/ 
typedef struct suof_imgtab_s {
    unsigned int numImgs;           /**< suof objects amount */
    suof_img_hdr_t *ImgHdr;          /**< suof objects table pointer */
} suof_imgtab_t;

/**
 * @description suof handle
*/
typedef struct uclo_suof_handle_s {
    unsigned int fileId;            /**< file ID, 0x53554f46 for "SUOF" */
    unsigned int checkSum;          /**< checksum from &minVer to the end */

    char         minVer;            /**< minor version */
    char         majVer;            /**< major version */

    char         fwType;            /**< firwware type */

    long         suofBuf;            /**< suof object pointer */
    uint64       suofSize;           /**< suof object size */

    long         symStr;            /**< SYM_OBJS pointer */
    unsigned int symSize;           /**< SYM_OBJS size */

    suof_imgtab_t imgTable;          /**< suof img table */
} uclo_suof_handle_t;

/**
 * @description firmware authentication descriptor
*/
typedef struct fw_auth_desc_s {
    unsigned int imgLen;
    unsigned int reservedForAlignment;

    unsigned int   cssHeaderH;
    unsigned int   cssHeaderL;
    unsigned int   imgH;
    unsigned int   imgL;
    unsigned int   signatureH;
    unsigned int   signatureL;
    unsigned int   fwskPubH;
    unsigned int   fwskPubL;

    unsigned int   imgAeModeDataH;
    unsigned int   imgAeModeDataL;
    unsigned int   imgAeInitDataH;
    unsigned int   imgAeInitDataL;
    unsigned int   imgAeInstructionsH;
    unsigned int   imgAeInstructionsL;
} fw_auth_desc_t;


/* memory chunk includes descriptor & simg    
 *     +---------------+
 *     | descriptor    |
 *     +---------------+
 *     | chunk size    |
 *     +---------------+
 *     | reserved      |
 *     +---------------+ SIMG_OFFSET_2_DESC
 *     | simg          | 
 *     |               |
 *     +---------------+
 */
typedef struct fw_auth_chunk_s {
    fw_auth_desc_t fwAuthDesc;      /* Descriptor for AuthFW */
    uint64   chunkSize;             /* whole memory chunk size */
    char     reserved[RESV_DESC_2_SIMG];      
    char*    simg;                  /* start address of simg */
} fw_auth_chunk_t;

#ifdef __cplusplus
}
#endif

#endif        /* __UCLO_SUOF_H__ */
