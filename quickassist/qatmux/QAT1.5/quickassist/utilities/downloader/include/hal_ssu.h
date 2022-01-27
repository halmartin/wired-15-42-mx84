/**
 **************************************************************************
 * @file hal_ssu.h
 *
 * @description
 *      This is the header file for Hardware Abstraction Layer -- SSU Unit
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

/**
 *****************************************************************************
 * @file hal_ssu.h
 * 
 * @ingroup icp_hal
 *
 * @description
 *      This header file contains the macros of SSU access
 *
 *****************************************************************************/
 
#ifndef __HAL_SSU_H
#define __HAL_SSU_H

#include "core_io.h"
#include "ae_platform.h"

typedef enum{
    /* Use these in the GET/PUT_SSU_QAT_CSR macros - pass QAT number [0-1] in the chan parameter */
    SSU_INTMASKSSU               = 0x0000,
    SSU_INTSTATSSU               = 0x0004,
    SSU_PPERR                    = 0x0008,
    SSU_PPERRID                  = 0x000C,
    SSU_CERRSSUSH                = 0x0010,
    SSU_CERSSUSHAD               = 0x0014,
    SSU_UERRSSUSH                = 0x0018,
    SSU_UERRSSUSHAD              = 0x001C,
    SSU_CBOVRDSSUSH              = 0x0020,
    SSU_CLKCFGPSSSUSH            = 0x0024,
    SSU_CLKCFGSPSSUSH            = 0x0028,

    /* new registers for Accel Complex */
    SSU_SLICEPWRDOWN             = 0x002C,
    SSU_QATINSTID                = 0x0030,
    SSU_NB_IA_EVENTMSK_CMP       = 0x0038,
    SSU_NB_IA_EVENTMSK_RE        = 0x0040,
    SSU_NB_IA_EVENTMSK_XLT       = 0x0048,
    
    SSU_BM_CNTRL                 = 0x0060,
    SSU_MASTER_PUSH_FIFO_THR     = 0x0064,
    SSU_INLET_CMD_FIFO_THR       = 0x0068,
    SSU_SLICE_STATUS             = 0x006C,

    /* Use these in the GET/PUT_SSU_QAT_MMP_CSR macros - pass QAT number [0-1] & MMP number [0-1] in the qatChan and mmpChan parameter */
    SSU_CERRSSUMMP               = 0x0380,
    SSU_CERRSSUMMPAD             = 0x0384,
    SSU_UERRSSUMMP               = 0x0388,
    SSU_UERRSSUMMPAD             = 0x038C,
    SSU_CBOVRDSSUMMP             = 0x0390,
    SSU_DBPCSRSSUMMP             = 0x03C0,
    SSU_DBPPCSSUMMP              = 0x03C4,
    SSU_DIOVRDSSUMMP             = 0x03C8,
    SSU_DMWISSUMMP               = 0x03CC,
    SSU_DOAWDSSUMMP              = 0x03D0,
    SSU_DOBWDSSUMMP              = 0x03D4,
    SSU_DPMWDSSUMMP              = 0x03D8,
    SSU_DMRISSUMMP               = 0x03DC,
    SSU_DOARDSSUMMP              = 0x03E0,
    SSU_DOBRDSSUMMP              = 0x03E4,
    SSU_DPMRDSSUMMP              = 0x03E8,
    SSU_DSMRDSSUMMP              = 0x03EC,
    SSU_DISRDSSUMMP              = 0x03F0,
    
    /* new registers for Accel Complex */
    SSU_EXPRPSSUCMP              = 0x0400,
    SSU_EXPRPSSUXLT              = 0x0500,

    /* Use these in the GET/PUT_SSU_QAT_CSR macros - pass QAT number [0-1] in the chan parameter */
    SSU_CERRSSURGX               = 0x0600,
    SSU_CERRSSURGXAD             = 0x0604,
    SSU_UERRSSURGX               = 0x0608,
    SSU_UERRSSURGXAD             = 0x060C,
    SSU_CBOVRDSSURGX             = 0x0610,
    SSU_NERRSSURGXJDT            = 0x0614,
    SSU_NERRSSURGXIDT            = 0x0618,
    SSU_NERRSSURGXBMT            = 0x061C,
    SSU_NERRSSURGXSRC            = 0x0620,
    SSU_NERRSSURGXMSK            = 0x0624,
    SSU_DBPCRSSURE               = 0x0640,
    SSU_DBPSRSSURE               = 0x0644,
    SSU_DBPCTSSURE               = 0x0648,
    SSU_DBPIBCSSURE              = 0x064C,
    SSU_DBPOBCSSURE              = 0x0650,
    SSU_DMRISSURE                = 0x0654,
    SSU_DDATACSRSSURE            = 0x0658,
    SSU_DBLASTOUTREC0            = 0x065C,
    SSU_DBLASTOUTREC1            = 0x0660,
    SSU_DBLASTOUTREC2            = 0x0664,
    SSU_DBLASTOUTREC3            = 0x0668,
    SSU_DBLASTOUTREC4            = 0x066C,
    SSU_DBLASTOUTREC5            = 0x0670,
}Hal_Ssu_CSR_T;

#define ACCELCOMP_C_SSUA_MASK 0x3
#define ACCELCOMP_C_SSUB_MASK 0x3c
#define ACCELCOMP_B_SSUA_MASK 0x1

#define ACCELCOMP_C_IS_SSUA(qat) ((ACCELCOMP_C_SSUA_MASK & (1<<qat)) >> qat)
#define ACCELCOMP_C_IS_SSUB(qat) ((ACCELCOMP_C_SSUB_MASK & (1<<qat)) >> qat)
#define ACCELCOMP_B_IS_SSUA(qat) ((ACCELCOMP_B_SSUA_MASK & (1<<qat)) >> qat)

typedef enum {
    QAT_SLICE_BEGIN=0x0,
    QAT0_NRBG0=0x0,
    QAT0_STR0,
    QAT0_BULK0,
    QAT0_AUTH0,
    QAT0_AUTH1,
    QAT0_CPH0,
    QAT0_CPH1,
    QAT0_MMP0,
    QAT0_MMP1,
    QAT0_MMP2,
    QAT0_MMP3,
    QAT0_MMP4,
    QAT0_RGX0,
    QAT0_CPR0,
    QAT0_DECPR0,
    QAT0_XLT0,
    QAT0_SLICE_END=0x10,
    QAT1_NRBG0=0x10,
    QAT1_STR0,
    QAT1_BULK0,
    QAT1_AUTH0,
    QAT1_AUTH1,
    QAT1_CPH0,
    QAT1_CPH1,
    QAT1_MMP0,
    QAT1_MMP1,
    QAT1_MMP2,
    QAT1_MMP3,
    QAT1_MMP4,
    QAT1_RGX0,
    QAT1_CPR0,
    QAT1_DECPR0,
    QAT1_XLT0,
    QAT1_SLICE_END=0x20,    
    QAT2_NRBG0=0x20,
    QAT2_STR0,
    QAT2_BULK0,
    QAT2_AUTH0,
    QAT2_AUTH1,
    QAT2_CPH0,
    QAT2_CPH1,
    QAT2_MMP0,
    QAT2_MMP1,
    QAT2_MMP2,
    QAT2_MMP3,
    QAT2_MMP4,
    QAT2_RGX0,
    QAT2_CPR0,
    QAT2_DECPR0,
    QAT2_XLT0,
    QAT2_SLICE_END=0x30,    
    QAT3_NRBG0=0x30,
    QAT3_STR0,
    QAT3_BULK0,
    QAT3_AUTH0,
    QAT3_AUTH1,
    QAT3_CPH0,
    QAT3_CPH1,
    QAT3_MMP0,
    QAT3_MMP1,
    QAT3_MMP2,
    QAT3_MMP3,
    QAT3_MMP4,
    QAT3_RGX0,
    QAT3_CPR0,
    QAT3_DECPR0,
    QAT3_XLT0,
    QAT3_SLICE_END=0x40,        
    QAT4_NRBG0=0x40,
    QAT4_STR0,
    QAT4_BULK0,
    QAT4_AUTH0,
    QAT4_AUTH1,
    QAT4_CPH0,
    QAT4_CPH1,
    QAT4_MMP0,
    QAT4_MMP1,
    QAT4_MMP2,
    QAT4_MMP3,
    QAT4_MMP4,
    QAT4_RGX0,
    QAT4_CPR0,
    QAT4_DECPR0,
    QAT4_XLT0,
    QAT4_SLICE_END=0x50,        
    QAT5_NRBG0=0x50,
    QAT5_STR0,
    QAT5_BULK0,
    QAT5_AUTH0,
    QAT5_AUTH1,
    QAT5_CPH0,
    QAT5_CPH1,
    QAT5_MMP0,
    QAT5_MMP1,
    QAT5_MMP2,
    QAT5_MMP3,
    QAT5_MMP4,
    QAT5_RGX0,
    QAT5_CPR0,
    QAT5_DECPR0,
    QAT5_XLT0,
    QAT5_SLICE_END=0x60,
    QAT_SLICE_END=0x60,
} qat_dev_id_t;

#define QAT_CPH(qat, cph) ((qat * QAT0_SLICE_END) + QAT0_CPH0 + cph)
#define QAT_AUTH(qat, auth) ((qat * QAT0_SLICE_END) + QAT0_AUTH0 + auth)
#define QAT_RGX(qat, rgx) ((qat * QAT0_SLICE_END) + QAT0_RGX0 + rgx)
#define QAT_CPR(qat, cpr) ((qat * QAT0_SLICE_END) + QAT0_CPR0 + cpr)
#define QAT_DECPR(qat, decpr) ((qat * QAT0_SLICE_END) + QAT0_DECPR0 + decpr)
#define QAT_XLT(qat, xlt) ((qat * QAT0_SLICE_END) + QAT0_XLT0 + xlt)
#define QAT_MMP(qat, pke) ((qat * QAT0_SLICE_END) + QAT0_MMP0 + pke)
#define QAT_BULK(qat, bulk) ((qat * QAT0_SLICE_END) + QAT0_BULK0 + bulk)
#define QAT_STR(qat, str) ((qat * QAT0_SLICE_END) + QAT0_STR0 + str)
#define QAT_NRBG(qat, nrbg) ((qat * QAT0_SLICE_END) + QAT0_NRBG0 + nrbg)

typedef enum {
    SLICETYPE_BEGIN=0x0,
    SLICETYPE_NRBG=0x0,
    SLICETYPE_STR,
    SLICETYPE_BULK,
    SLICETYPE_AUTH,
    SLICETYPE_CPH,
    SLICETYPE_MMP,
    SLICETYPE_RGX,
    SLICETYPE_CPR,
    SLICETYPE_DECPR,
    SLICETYPE_XLT,
/* add more slicetype here for the future silicon support */
    SLICETYPE_END,
} slicetype_t;

typedef enum {
    QAT_BEGIN=0x0,
    QAT0=0x0,
    QAT1=0x1,
    QAT2=0x2,
    QAT3=0x3,
    QAT4=0x4,
    QAT5=0x5,
/* add more qat here for the future silicon support */
    QAT_END,
} qat_id_t;

#define GET_QATDEV_ID(qatDevId, qatId, sliceType, sliceInstanceId)  \
    do { \
           switch(sliceType) \
           { \
           case SLICETYPE_NRBG: \
               qatDevId = QAT_NRBG(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_STR: \
               qatDevId = QAT_STR(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_BULK: \
               qatDevId = QAT_BULK(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_AUTH: \
               qatDevId = QAT_AUTH(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_CPH: \
               qatDevId = QAT_CPH(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_MMP: \
               qatDevId = QAT_MMP(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_RGX: \
               qatDevId = QAT_RGX(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_CPR: \
               qatDevId = QAT_CPR(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_DECPR: \
               qatDevId = QAT_DECPR(qatId, sliceInstanceId); \
               break; \
           case SLICETYPE_XLT: \
               qatDevId = QAT_XLT(qatId, sliceInstanceId); \
               break; \
           default: \
               qatDevId = QAT_SLICE_END; \
               break; \
           } \
    } while(0)

typedef enum {
    QAT_SHRAM_CSR = 0x0,
    QAT_BULK0_CSR = 0x100,
    QAT_MMP0_CSR = 0x380,
    QAT_MMP1_CSR = 0x1380,
    QAT_MMP2_CSR = 0x2380,
    QAT_MMP3_CSR = 0x3380,
    QAT_MMP4_CSR = 0xB80,
    QAT_CPR0_CSR = 0x400,
    QAT_XLT0_CSR = 0x500,
    QAT_RGX0_CSR = 0x600
} qat_slice_csr_t;

typedef enum {
    MISC_PERF_MOR_BEGIN=0x0,
    PCIE=0x0,
    MISC_PERF_MOR_END,
    /* Reserved for furture performance monitor use */
    Reserved  
} misc_perf_mor_id_t;

#define QAT_SLICE_NO_BITPOS 12
#define QAT_SLICE_TYPE_BITPOS 7
#define QAT_SLICE_NO (0x3 << QAT_SLICE_NO_BITPOS)
#define QAT_SLICE_TYPE (0x1f << QAT_SLICE_TYPE_BITPOS)

#define SSU_QAT_CSR(handle, chan) (handle->Hal_ssu_csr_virtAddr + QAT_TO_QAT_OFFSET*(chan))

#define MMP_TO_MMP_OFFSET 0x1000
#define SSU_QAT_MMP_CSR(handle, qatChan, mmpChan) \
    (SSU_QAT_CSR((handle), (qatChan)) + MMP_TO_MMP_OFFSET*(mmpChan%0x4) + 0x800*(mmpChan/0x4))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword data to SSU QAT control and status register.
 *      
 * @param chan - IN Specifies QAT number
 * @param csr - IN Specifies register offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
#define PUT_SSU_QAT_CSR(handle, chan, csr, val)  \
               WRITE_LWORD((SSU_QAT_CSR(handle, chan)+((csr) & 0x3FFF)), (val));

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword data from SSU QAT control and status register.
 *      
 * @param chan - IN Specifies QAT number
 * @param csr - IN Specifies register offset
 *
 * @retval - data read
 * 
 * 
 *****************************************************************************/

#define GET_SSU_QAT_CSR(handle, chan, csr) \
              READ_LWORD((SSU_QAT_CSR(handle, chan)+((csr) & 0x3FFF)));

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword data to SSU QAT MMP control and status register.
 *      
 * @param qatChan - IN Specifies QAT number
 * @param mmpChan - IN Specifies MMP number
 * @param csr - IN Specifies register offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
#define PUT_SSU_QAT_MMP_CSR(handle, qatChan, mmpChan, csr, val)  \
              WRITE_LWORD((SSU_QAT_MMP_CSR(handle, qatChan, mmpChan)+((csr) & 0x3FF)), (val))
              

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword data from SSU QAT MMP control and status register.
 *      
 * @param qatChan - IN Specifies QAT number
 * @param mmpChan - IN Specifies MMP number
 * @param csr - IN Specifies register offset
 *
 * @retval - data read
 * 
 * 
 *****************************************************************************/
#define GET_SSU_QAT_MMP_CSR(handle, qatChan, mmpChan, csr) \
              READ_LWORD(SSU_QAT_MMP_CSR(handle, qatChan, mmpChan)+((csr) & 0x3FF));

#endif      /* __HAL_SSU_H */
