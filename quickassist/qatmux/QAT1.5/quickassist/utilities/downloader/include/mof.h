/**
 **************************************************************************
 * @file mof.h
 *
 * @description
 *      This is the header file for uof librarian file (MOF)
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

/*
 ****************************************************************************
 * Doxygen group definitions
 ****************************************************************************/

/**
 *****************************************************************************
 * @file mof.h
 * 
 * @defgroup MOF Microcode File Format Definition
 *
 * @description
 *      This header file that contains the definitions of micorcode file 
 *      format used by linker and loader.
 *
 *****************************************************************************/

#ifndef __MOF_H__
#define __MOF_H__

#ifdef __cplusplus
extern "C"{
#endif

#define MOF_OBJ_ID_LEN        8     /**< length of MOF object ID, can be SYM_OBJS/UOF_OBJS */
#define MOF_OBJ_CHUNKID_LEN   8     /**< length of object chunk Id, can be UOF_IMAG */
#define MAX_UOF_OBJ_CHUNKS    0xffff /**< max UOF object chunks allowed */
#define MOF_FID 0x00666f6d          /**< MOF file ID, ASIC "MOF" */
#define MOF_MAJVER 0x0              /**< SOF major version */
#define MOF_MINVER 0x1              /**< SOF minor version */
#define SYM_OBJS "SYM_OBJS"         /**< symbol object string */
#define UOF_OBJS "UOF_OBJS"         /**< uof object string */
#define SUOF_OBJS "SUF_OBJS"         /**< suof object string */
#define UOF_IMAG  "UOF_IMAG"         /**< uof chunk ID string */
#define SUOF_IMAG "SUF_IMAG"         /**< suof chunk ID string */

typedef struct mof_fileHdr_S{
   unsigned int   fileId;             /**< file id, should be 0x00666f6d for MOF */
   unsigned int   checkSum;           /**< CRC checksum of the file */
   char           minVer;             /**< minor version */
   char           majVer;             /**< major version */
   unsigned short reserved;           /**< reserved */
   unsigned short maxChunks;          /**< maximum chunk */
   unsigned short numChunks;          /**< actual chunk amount */
} mof_fileHdr_T;


typedef struct mof_fileChunkHdr_S{
   char         chunkId [MOF_OBJ_ID_LEN];      /**< chunk identifier, can be SYM_OBJS/UOF_OBJS */
   uint64       offset;                        /**< offset of the chunk in the file */
   uint64       size;                          /**< size of the chunk */
} mof_fileChunkHdr_T;


typedef struct mof_strTableObj_S{
   unsigned int tabLength;               /**< size of the string table */
   unsigned int strings;                 /**< string table address, each string is NULL terminated */
} mof_strTableObj_T;


typedef struct mof_uofObjHdr_S{
   unsigned short maxChunks;             /**< max supported Chunk Id */
   unsigned short numChunks;             /**< actual chunk amount */
   unsigned int   reserved;              /**< not used */
} mof_uofObjHdr_T;


typedef struct mof_uofObjChunkHdr_S{
   char         chunkId [MOF_OBJ_CHUNKID_LEN];   /**< UOF chunk Id, should be UOF_IMAGE */
   uint64       offset;                          /**< offset from the beginning of the object to individual UOF image */
   uint64       size;                            /**< size of the string table */
   unsigned int name;                            /**< uof name string-table offset */
   unsigned int reserved;                        /**< not used */
} mof_uofObjChunkHdr_T;


#ifdef __cplusplus
}
#endif

#endif  /* __MOF_H__ */
