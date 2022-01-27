/**
 **************************************************************************
 * @file suof.h
 *
 * @description
 *      This is the header file for signed uof librarian file (suof)
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
 * @file suof.h
 * 
 * @defgroup SUOF Microcode File Format Definition
 *
 * @description
 *      This header file that contains the definitions of micorcode file 
 *      format used by linker and loader.
 *
 *****************************************************************************/

#ifndef __SUOF_H__
#define __SUOF_H__

#ifdef __cplusplus
extern "C"{
#endif

#define SUOF_OBJ_ID_LEN      8      /**< length of MOF object ID, can be SYM_OBJS/IMG_OBJS */
#define MAX_SUOF_OBJ_CHUNKS  0xffff /**< max UOF object chunks allowed */
#define IMG_FID  0x00494d47         /**< IMG file ID, ASIC "IMG" */
#define SUOF_FID 0x53554f46         /**< SUOF file ID, ASIC "SUOF" */
#define IMG_MAJVER 0x0              /**< IMG major version */
#define IMG_MINVER 0x1              /**< IMG minor version */

#define SUOF_MAJVER 0x0             /**< SOF major version */
#define SUOF_MINVER 0x1             /**< SOF minor version */

#define CSS_MODULE_TYPE  0x6        /**< To be defined by CSS team */

enum {
    SUOF_QA_FIRMWARE = 0,
    SUOF_SPI_FIRMWARE = 2,
    SUOF_PCIE_FIRMWARE = 3
};
   
enum {
    CSS_AE_FIRMWARE = 0,
    CSS_MMP_FIRMWARE = 1
};

#define IMG_RESERVED 0xa
#define IMG_NAME_LEN 0x100
#define IMG_APP_META_DATA 0x100

#define MAX_MMP_IMG_SIZE 0x40000
#define MAX_INIT_SEQ 50            /**< initailzation sequence size 50 * 8bytes = 400Bytes*/

#define SIMG_OBJS "SIG_OBJS"       /**< signed image object string */

#define SIMG_AE_MODE_DATA_LEN   (sizeof (img_aeMode_T))
#define SIMG_AE_INIT_SEQ_LEN    (MAX_INIT_SEQ * sizeof (unsigned long long))
#define SIMG_AE_INSTRUCTS_LEN   (MAX_USTORE * sizeof (unsigned long long))

#define CSS_FWSK_MODULUS_LEN    256
#define CSS_FWSK_EXPONENT_LEN   4
#define CSS_FWSK_PADDING_LEN    252
#define CSS_FWSK_PUB_LEN        (CSS_FWSK_MODULUS_LEN + CSS_FWSK_EXPONENT_LEN + CSS_FWSK_PADDING_LEN)
#define CSS_SIGNATURE_LEN       256
#define CSS_AE_IMG_LEN          (SIMG_AE_MODE_DATA_LEN + SIMG_AE_INIT_SEQ_LEN + SIMG_AE_INSTRUCTS_LEN)
#define CSS_AE_SIMG_LEN         (sizeof (css_header_T) + CSS_FWSK_PUB_LEN + CSS_SIGNATURE_LEN + CSS_AE_IMG_LEN)
#define CSS_MAX_IMAGE_LEN       0x40000    /* max space for MMP/AE is 256KB */

typedef struct css_header_S {
	unsigned int moduleType;            /**< Required CSS field  (need confirm with CSS team) */
	unsigned int headerLen;             /**< Required CSS field  */
	unsigned int headerVersion;         /**< Required CSS field (need confirm with CSS team) */
	unsigned int moduleID;              /**< Required CSS field (need confirm with CSS team - 0x80000000 for debug) */
	unsigned int moduleVendor;          /**< Required CSS field (8086) */
	unsigned int date;                  /**< Required CSS field  */
	unsigned int size;                  /**< Required CSS field (total size in longword) */
	unsigned int keySize;               /**< Required CSS field (0x40) */
	unsigned int modulusSize;           /**< Required CSS field (0x40) */
	unsigned int exponentSize;          /**< Required CSS field (1) */
	unsigned int fwType;                /**< 0: ME, 1: MMP */
	unsigned int reserved[21];          /**< Reserved */
} css_header_T;

typedef struct img_aeMode_S{

   unsigned int     fileId;             /**< file id 0x494d47 */
   unsigned short   majVer;             /**< file format major version */
   unsigned short   minVer;             /**< file format minor version */
   unsigned int     devType;            /**< product type, like ACCELCOMP_B_CPU_TYPE */
   unsigned short   devMaxVer;          /**< max supported version */
   unsigned short   devMinVer;          /**< minimum supported version */
   unsigned int     aeMask;             /**< AE mask this image will be applied to */
   unsigned int     ctxEnables;         /**< value be written to CtxEnablesStartData */
   char             fwType;             /**< firmware type, 0 QA FW, 2 SPI FW, 3 PCIE FW */
   char             ctxMode;            /**< number of in-use contexts, 0:8ctxs mode, 1:4ctxs mode */
   char             NNMode;             /**< Next Neighbor usage mode. 0:for previous neighbor ME,1:from this ME */
   char             LM0Mode;            /**< LM_ADDR_0 usage 0:context relative, 1:global */
   char             LM1Mode;            /**< LM_ADDR_1 usage 0:context relative, 1:global */
   char             sharedCSMode;       /**< shared ustore mode 0:not shared, 1:shared */
   unsigned char    reserved[IMG_RESERVED];
   char             imgName[IMG_NAME_LEN];           /**< image name */
   char             appMetaData[IMG_APP_META_DATA];  /**< appMetaData string */
}img_aeMode_T;

typedef struct img_aeInitIns_S{
   unsigned int     csrData;            /**< data will be written into that address  */
   unsigned int     csrAddr;            /**< CPP-centric local CSR address  */
}img_aeInitIns_T;

typedef struct img_aeInitSeq_S{
   img_aeInitIns_T  instructions[MAX_INIT_SEQ];  /**< initialization sequence paris */
}img_aeInitSeq_T;

typedef struct suof_fileHdr_S{
   unsigned int     fileId;             /**< file id, 0x53554F46 for SUOF */
   unsigned int     checkSum;           /**< CRC checksum of the file  */
   char             minVer;             /**< file format minor version */
   char             majVer;             /**< file format major version */
   char             fwType;             /**< firmware type */
   char             reserved1;          /**< reserved for future use */
   unsigned short   maxChunks;          /**< max chunks in file */
   unsigned short   numChunks;          /**< num of actual chunks */
}suof_fileHdr_T;

typedef struct suof_fileChunkHdr_S{
   char         chunkId [SUOF_OBJ_ID_LEN];     /**< chunk identifier, can be SYM_OBJS/SIMG_OBJS */
   uint64       offset;                        /**< offset of the chunk in the file */
   uint64       size;                          /**< size of the chunk */
} suof_fileChunkHdr_T;  

typedef struct suof_strTableObj_S{
   unsigned int tabLength;               /**< size of the string table */
   unsigned int strings;                 /**< string table address, each string is NULL terminated */
} suof_strTableObj_T;

typedef struct suof_objHdr_S{
   unsigned int imgLength;               /**< length of the signed image */
   unsigned int reserved;                /**< not used */
} suof_objHdr_T;

#ifdef __cplusplus
}
#endif

#endif  /* __SUOF_H__ */
