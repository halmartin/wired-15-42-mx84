/**
 **************************************************************************
 * @file uclo_suof.c
 *
 * @description
 *      This file provides SUOF objects interfaces
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

#include "uclo_platform.h"
#include "uclo_dev.h"
#include "uclo.h"
#include "uclo_helper.h"
#include "icp_firml_handle.h"
#include "suof.h"
#include "uclo_suof.h"

#define UCLO_SUOF_RETURN_IN_CONDITION(condition, errInfo, retValue)\
if (condition == TRUE) {\
    PRINTF (errInfo);\
    return retValue;\
}\

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parse SUOF file header and map it to suof handle
 *
 * @param handle - IN
 * @param suofPtr  - IN  Start address of SUOF buffer
 * @param suofSize - IN SUOF length
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
static int 
UcLo_mapSuofFileHdr (icp_firml_handle_t *handle, 
                     void *suofPtr, 
                     uint64 suofSize)
{
    uclo_suof_handle_t* suofHandle = (uclo_suof_handle_t *)handle->sobjHandle;
    unsigned int checkSum = 0;

    /* verify file type */
    UCLO_SUOF_RETURN_IN_CONDITION (
    (((suof_fileHdr_T *)suofPtr)->fileId != SUOF_FID),
        "incorrect SUOF head type \n",
        UCLO_BADOBJ);
    
    suofHandle->fileId = SUOF_FID;
    suofHandle->suofBuf= (long)suofPtr;
    suofHandle->suofSize= suofSize;

    /* verify checksum */
    checkSum = UcLo_strChecksum ((char *)
        ((long)suofPtr + (long)&((suof_fileHdr_T *)0)->minVer),
        ((long)suofSize - (long)&((suof_fileHdr_T *)0)->minVer));
    UCLO_SUOF_RETURN_IN_CONDITION (
        (checkSum != ((suof_fileHdr_T *)suofPtr)->checkSum),
        "incorrect SUOF checksum \n",
        UCLO_BADOBJ);
        
    suofHandle->checkSum = ((suof_fileHdr_T *)suofPtr)->checkSum;

    /* major & minor version */
    UCLO_SUOF_RETURN_IN_CONDITION (
        ((((suof_fileHdr_T *)suofPtr)->majVer != SUOF_MAJVER) ||
        (((suof_fileHdr_T *)suofPtr)->minVer != SUOF_MINVER)),
        "incompatible SUOF version \n",
        UCLO_BADOBJ);

    suofHandle->minVer = ((suof_fileHdr_T *)suofPtr)->minVer;
    suofHandle->majVer = ((suof_fileHdr_T *)suofPtr)->majVer;

    /* firmware type */
    UCLO_SUOF_RETURN_IN_CONDITION (
        ((((suof_fileHdr_T *)suofPtr)->fwType != SUOF_QA_FIRMWARE) &&
         (((suof_fileHdr_T *)suofPtr)->fwType != SUOF_SPI_FIRMWARE) &&
         (((suof_fileHdr_T *)suofPtr)->fwType != SUOF_PCIE_FIRMWARE)),
        "unsupported firmware type \n",
        UCLO_BADOBJ);

    suofHandle->fwType = ((suof_fileHdr_T *)suofPtr)->fwType;
    
    /* actual chunk amount should less than maxchunks */
    UCLO_SUOF_RETURN_IN_CONDITION (
        (((suof_fileHdr_T *)suofPtr)->maxChunks <
        ((suof_fileHdr_T *)suofPtr)->numChunks),
        "incorrect SUOF file chunk amount\n",
        UCLO_BADOBJ);

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Map an uof object into SUOF handle
 *
 * @param handle - IN
 * @param mofObjHdr - OUT  uof object 
 * @param uofObjChunkHdr - IN uof objct chunk header
 *
 * @retval  UCLO_SUCCESS, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
static int 
UcLo_mapSUofImgObj (uclo_suof_handle_t* suofHandle,
                    suof_img_hdr_t* suofImgHdr,
                   suof_fileChunkHdr_T* suofChunkHdr)
{
    //suof_img_hdr_t* suofImgHdr = suofHandle->imgTable.ImgHdr;
    unsigned long suofEnd = (long)suofHandle->suofBuf + 
                            (long)suofHandle->suofSize;
    img_aeMode_T* aeMode = NULL;

    UCLO_SUOF_RETURN_IN_CONDITION (
        (suofChunkHdr->size != 
        CSS_AE_IMG_LEN + CSS_FWSK_MODULUS_LEN + CSS_SIGNATURE_LEN +
        CSS_FWSK_EXPONENT_LEN + sizeof (css_header_T) + 
        sizeof (suof_objHdr_T)),
        "Incorrect SIMG size is detected\n",
        UCLO_BADOBJ);

    /* simg offset */
    suofImgHdr->simgBuf  = (suofHandle->suofBuf + 
            (long)suofChunkHdr->offset + sizeof (suof_objHdr_T));
    suofImgHdr->simgLen = ((suof_objHdr_T *)
            (suofHandle->suofBuf + (long)suofChunkHdr->offset))->imgLength;
    
    suofImgHdr->cssHeader = suofImgHdr->simgBuf;
    suofImgHdr->cssKey = (suofImgHdr->cssHeader + sizeof (css_header_T));
    suofImgHdr->cssSignature = suofImgHdr->cssKey + 
        CSS_FWSK_MODULUS_LEN + CSS_FWSK_EXPONENT_LEN;
    suofImgHdr->cssImg = suofImgHdr->cssSignature + CSS_SIGNATURE_LEN;

    aeMode = (img_aeMode_T *)(suofImgHdr->cssImg);
    suofImgHdr->aeMask = aeMode->aeMask;
    suofImgHdr->pImgName = (long) &aeMode->imgName;
    suofImgHdr->pAppMetaData = (long) &aeMode->appMetaData;
    suofImgHdr->fwType = aeMode->fwType;

    UCLO_SUOF_RETURN_IN_CONDITION (
        ((unsigned int)suofImgHdr->cssImg > suofEnd),
        "Incorrect UOF chunk offset is detected\n",
        UCLO_BADOBJ);

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parse SUOF file string table object and map it to mof handle
 *
 * @param handle - IN
 * @param suofFilechunkHdr - IN  symbol chunk header
 *
 * @retval  UCLO_SUCCESS, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
static int 
UcLo_mapSuofSymObjs (uclo_suof_handle_t *suofHandle, 
                     suof_fileChunkHdr_T* suofFilechunkHdr)
{
    char** symStr = (char**)&suofHandle->symStr;
    unsigned int* symSize = &suofHandle->symSize;
    unsigned long suofEnd = (long)suofHandle->suofBuf + 
                            (long)suofHandle->suofSize;
    suof_strTableObj_T* strTableObj = NULL;
    
    /* string table */
    *symSize = *(unsigned int*)
        ((long)suofFilechunkHdr->offset + suofHandle->suofBuf);
    *symStr = (char *)(suofHandle->suofBuf + 
        (long)suofFilechunkHdr->offset + sizeof(strTableObj->tabLength));
    
    /* 4 bytes aligned */
    UCLO_SUOF_RETURN_IN_CONDITION (
        ((*symSize % sizeof (unsigned int)) != 0),
        "unaligned string table size inside SUOF \n",
        UCLO_BADOBJ);

    /* string table bounday check */
    UCLO_SUOF_RETURN_IN_CONDITION (
        ((suofFilechunkHdr->size !=
        (*symSize + sizeof(strTableObj->tabLength)))||
        (suofFilechunkHdr->size == 0)),
        "incorrect stringTable size inside SUOF\n",
        UCLO_BADOBJ);

    UCLO_SUOF_RETURN_IN_CONDITION (
        (((long)*symStr + *symSize) > suofEnd),
        "incorrect String table inside SUOF \n",
        UCLO_BADOBJ);
    
    /* end of stringTable should be 0 */
    UCLO_SUOF_RETURN_IN_CONDITION (
        ((*(char *)(*symStr + *symSize - sizeof (char))) != 0),
        "incorrect tableString end of SUOF \n",
        UCLO_BADOBJ);

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Verify an uof object into SUOF handle
 *
 * @param handle - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
static int 
UcLo_VerifySUofImgObj (icp_firml_handle_t* handle)
{
    uclo_suof_handle_t *suofHandle = handle->sobjHandle;
    suof_img_hdr_t* imgHdr = NULL;
    img_aeMode_T*   imgAeMode = NULL;
    unsigned int prodType, prodRev, i = 0;

    for (i = 0; i < suofHandle->imgTable.numImgs; i++)
    {
        imgHdr = &suofHandle->imgTable.ImgHdr[i];
        imgAeMode = (img_aeMode_T *)imgHdr->cssImg;

        /* validate fileId */
        UCLO_SUOF_RETURN_IN_CONDITION (
            (imgAeMode->fileId != IMG_FID),
            "Incorrect IMG id is detected \n",
            UCLO_BADOBJ);
        
        /* validate product device type */
        halAe_GetProdInfo (handle, &prodType, &prodRev);
        UCLO_SUOF_RETURN_IN_CONDITION (
            (imgAeMode->devType!= prodType),
            "Incompatible product type \n",
            UCLO_UOFINCOMPAT);
    }

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Release SUOF handle created during UcLo_MapSuofAddr
 *
 * @param handle - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG
 * 
 * 
 *****************************************************************************/
int 
UcLo_DelSuof (icp_firml_handle_t *handle)
{
    if (!handle)
    {
        return UCLO_BADARG;
    }

    if ((uclo_suof_handle_t *)(handle->sobjHandle) != NULL)
    {
        if (((uclo_suof_handle_t *)
            (handle->sobjHandle))->imgTable.ImgHdr != NULL)
        {
        
            osalMemFree(((uclo_suof_handle_t *)handle->sobjHandle)
                        ->imgTable.ImgHdr);
            ((uclo_suof_handle_t *)
                (handle->sobjHandle))->imgTable.ImgHdr = NULL;
        }

        osalMemFree((uclo_suof_handle_t *)(handle->sobjHandle));
        handle->sobjHandle = NULL;
    }
    else
    {
        return UCLO_NOOBJ;
    }

    return UCLO_SUCCESS;
}


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parses SUOF file format
 *
 * @param handle - IN
 * @param suofPtr - IN  Start address of SUOF buffer
 * @param suofSize - IN SUOF length
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
int 
UcLo_MapSuofAddr (icp_firml_handle_t *handle, 
                     void *suofPtr, 
                     uint64 suofSize)
{
    uclo_suof_handle_t* suofHandle = handle->sobjHandle;
    suof_fileChunkHdr_T* suofFilechunkHdr = NULL;
    suof_img_hdr_t* suof_img_hdr = NULL;
    unsigned short NumSuofChunks = 0;
    int retVal = UCLO_SUCCESS, ae0_img = MAX_AE;
    unsigned int i = 0;
    suof_img_hdr_t img_header;

    UCLO_SUOF_RETURN_IN_CONDITION (
        (handle == NULL),
        "input paramter handle is NULL\n",
        UCLO_BADARG);

    UCLO_SUOF_RETURN_IN_CONDITION (
        ((suofPtr == NULL) || (suofSize == 0)),
        "input paramter SUOF pointer/size is NULL\n",
        UCLO_BADARG);    

    UCLO_SUOF_RETURN_IN_CONDITION (
        (((suof_fileHdr_T *) suofPtr)->fileId != SUOF_FID),
        "input paramter SUOF pointer/size is NULL\n",
        UCLO_BADOBJ);

    /* map SUOF header */
    retVal = UcLo_mapSuofFileHdr (handle, suofPtr, suofSize);
    if (retVal != UCLO_SUCCESS)
    {
        UcLo_DelSuof (handle);
        return retVal;
    }

    suofFilechunkHdr = (suof_fileChunkHdr_T *)
        ((long)suofPtr + sizeof (suof_fileHdr_T));
    NumSuofChunks = ((suof_fileHdr_T *)suofPtr)->numChunks;

    UCLO_SUOF_RETURN_IN_CONDITION (
        (NumSuofChunks <= 0x1),
        "input paramter SUOF chunk amount is incorrect\n",
        UCLO_BADOBJ);
        
    /* 1st chunck header must be SYM_OBJS*/
    retVal = UcLo_mapSuofSymObjs (suofHandle, &suofFilechunkHdr[0]);
    if (retVal != UCLO_SUCCESS)
    {
        UcLo_DelSuof (handle);
        return retVal;
    }

    /* all others are simg */
    suofHandle->imgTable.numImgs = NumSuofChunks - 1;

    if (suofHandle->imgTable.numImgs != 0)
    {
        suof_img_hdr = (suof_img_hdr_t *) 
            osalMemAlloc (suofHandle->imgTable.numImgs * 
                          sizeof (suof_img_hdr_t));
        UCLO_SUOF_RETURN_IN_CONDITION (
            (suof_img_hdr == NULL),
            "memory allocation fail\n",
            UCLO_FAILURE);
        osalMemSet(suof_img_hdr, 0, 
            sizeof (suofHandle->imgTable.numImgs *  sizeof (suof_img_hdr_t)));
        
        suofHandle->imgTable.ImgHdr = suof_img_hdr;
    }

   /* Parse SUOF file chunks */
   for (i = 0; i < suofHandle->imgTable.numImgs; i++)
   {
       retVal = UcLo_mapSUofImgObj 
           (handle->sobjHandle, &suof_img_hdr[i], &suofFilechunkHdr[1 + i]);
       if (retVal != UCLO_SUCCESS)
       {
           UcLo_DelSuof (handle);
           return retVal;
       }
       if ((suof_img_hdr[i].aeMask & 0x1) != 0)
       {
           ae0_img = i;
       }
   }

   /* move AE0 image to the tail of the array, 
      AE0 ustore will be loaded after all other AEs */
   if ((ae0_img != suofHandle->imgTable.numImgs) && (ae0_img != MAX_AE))
   {
       /* swap ME0 img_header to the tail */
       osalMemCopy (&img_header, 
                    &suof_img_hdr[suofHandle->imgTable.numImgs - 1], 
                    sizeof (suof_img_hdr_t));

       osalMemCopy (&suof_img_hdr[suofHandle->imgTable.numImgs - 1], 
                    &suof_img_hdr[ae0_img],
                    sizeof (suof_img_hdr_t));
       
       osalMemCopy (&suof_img_hdr[ae0_img], 
                    &img_header,
                    sizeof (suof_img_hdr_t));
   }
   
   /* verify it */
   retVal = UcLo_VerifySUofImgObj (handle);
   if (retVal != UCLO_SUCCESS)
   {
       UcLo_DelSuof (handle);
       return retVal;
   }

    return retVal;
}


