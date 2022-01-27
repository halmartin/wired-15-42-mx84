/**
 **************************************************************************
 * @file uclo_mof.c
 *
 * @description
 *      This file provides MOF objects interfaces
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
#include "mof.h"
#include "suof.h"
#include "uclo_mof.h"

#define UCLO_MOF_RETURN_IN_CONDITION(condition, errInfo, retValue)\
if (condition == TRUE) {\
    PRINTF (errInfo);\
    return retValue;\
}\

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parse MOF file header and map it to mof handle
 *
 * @param handle - IN
 * @param mofPtr - IN  Start address of MOF/UOF buffer
 * @param mofSize - IN MOF/UOF length
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
static int 
UcLo_mapMofFileHdr (icp_firml_handle_t *handle, 
                     void *mofPtr, 
                     uint64 mofSize)
{
    uclo_mofhandle_t* mofHandle = NULL;
    unsigned int checkSum = 0;
    
    mofHandle = (uclo_mofhandle_t *) osalMemAlloc (sizeof (uclo_mofhandle_t));
    UCLO_MOF_RETURN_IN_CONDITION (
        (mofHandle == NULL),
        "memory allocation failure\n",
        UCLO_FAILURE);
    osalMemSet(mofHandle, 0, sizeof (uclo_mofhandle_t));
    handle->mofHandle = mofHandle;

    mofHandle->fileId = MOF_FID;
    mofHandle->mofBuf = (long)mofPtr;
    mofHandle->mofSize = mofSize;

    /* verify checksum */
    checkSum = UcLo_strChecksum ((char *)
        ((long)mofPtr + (long)&((mof_fileHdr_T *)0)->minVer),
        ((long)mofSize - (long)&((mof_fileHdr_T *)0)->minVer));
    UCLO_MOF_RETURN_IN_CONDITION (
        (checkSum != ((mof_fileHdr_T *)mofPtr)->checkSum),
        "incorrect MOF checksum \n",
        UCLO_BADOBJ);
        
    mofHandle->checkSum = ((mof_fileHdr_T *)mofPtr)->checkSum;

    /* major & minor version */
    UCLO_MOF_RETURN_IN_CONDITION (
        ((((mof_fileHdr_T *)mofPtr)->majVer != MOF_MAJVER) ||
        (((mof_fileHdr_T *)mofPtr)->minVer != MOF_MINVER)),
        "incompatible MOF version \n",
        UCLO_BADOBJ);

    mofHandle->minVer = ((mof_fileHdr_T *)mofPtr)->minVer;
    mofHandle->majVer = ((mof_fileHdr_T *)mofPtr)->majVer;

    /* actual chunk amount should less than maxchunks */
    UCLO_MOF_RETURN_IN_CONDITION (
        (((mof_fileHdr_T *)mofPtr)->maxChunks <
        ((mof_fileHdr_T *)mofPtr)->numChunks),
        "incorrect MOF file chunk amount\n",
        UCLO_BADOBJ);

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Seek specified uof object inside a MOF object
 *
 * @param handle - IN
 * @param uofName - IN  uof file name
 * @param uofPtr  - OUT uof address 
 * @param uofSize - OUT uof file size 
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
static int 
UcLo_mapMofSeekUof (uclo_mofhandle_t* mofHandle, 
                    char* uofName, 
                    char** uofPtr, 
                    unsigned int* uofSize)
{
    unsigned int i = 0;
    mof_objhdr_t* uofObjHdr = mofHandle->objTable.objHdr;

    for (i = 0;
         i < mofHandle->objTable.numObjs;
         i++)
    {
        if (!strcmp (uofObjHdr[i].uofName, uofName))
        {
            /* return the uof and berak */
            if ((uofPtr != NULL) && (uofSize != NULL))
            {
                if (uofPtr != NULL)
                {
                    *uofPtr  = uofObjHdr[i].uofBuf;
                }
                if (uofSize != NULL)
                {
                    *uofSize = uofObjHdr[i].uofSize;
                }
            }
            break;
        }
    }

    UCLO_MOF_RETURN_IN_CONDITION (
        (i >= mofHandle->objTable.numObjs),
        "UOF is not found inside MOF\n",
        UCLO_FAILURE);

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Map an uof/suof object into MOF handle
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
UcLo_mapMofUofObj (uclo_mofhandle_t* mofHandle,
                   mof_objhdr_t* mofObjHdr,
                   mof_uofObjChunkHdr_T* uofObjChunkHdr)
{
    unsigned long mofEnd = (long)mofHandle->mofBuf + (long)mofHandle->mofSize;

    /* uof offset */
    if ((strncmp ((char *)(uofObjChunkHdr->chunkId), 
        UOF_IMAG, MOF_OBJ_CHUNKID_LEN)) == 0)
    {
        mofObjHdr->uofBuf = (char *)
        ((long)uofObjChunkHdr->offset + mofHandle->uofObjsHdr);
        UCLO_MOF_RETURN_IN_CONDITION (
            ((unsigned long)mofObjHdr->uofBuf > mofEnd),
            "Incorrect UOF chunk offset is detected\n",
            UCLO_BADOBJ);
    }
    else if ((strncmp ((char *)(uofObjChunkHdr->chunkId), 
            SUOF_IMAG, MOF_OBJ_CHUNKID_LEN)) == 0)
    {
        mofObjHdr->uofBuf = (char *)
            ((long)uofObjChunkHdr->offset + mofHandle->suofObjsHdr);
        UCLO_MOF_RETURN_IN_CONDITION (
            ((unsigned long)mofObjHdr->uofBuf > mofEnd),
            "Incorrect SUOF chunk offset is detected\n",
            UCLO_BADOBJ);
    }
    else
    {
        PRINTF("unsupported chunck id\n");
        return UCLO_BADOBJ;
    }
    
    /* uof size */
    mofObjHdr->uofSize = (unsigned int)uofObjChunkHdr->size;
    UCLO_MOF_RETURN_IN_CONDITION (
        ((mofObjHdr->uofSize + (unsigned long)mofObjHdr->uofBuf) > mofEnd),
        "Incorrect UOF/SUOF chunk size is detected\n",
        UCLO_BADOBJ);
    
    /* uof name */
    mofObjHdr->uofName = (char*)(uofObjChunkHdr->name + mofHandle->symStr);
    UCLO_MOF_RETURN_IN_CONDITION (
        (uofObjChunkHdr->name >= mofHandle->symSize),
        "Incorrect UOF/SUOF chunk name is detected\n",
        UCLO_BADOBJ);

    return UCLO_SUCCESS;
}


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     map all uof and suof object inside MOF file
 *
 * @param handle - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
static int 
UcLo_mapMofUofObjs (uclo_mofhandle_t* mofHandle)
{
    mof_objhdr_t* mof_objHdr = NULL;
    mof_uofObjHdr_T* uofObjHdr = NULL;
    mof_uofObjHdr_T* suofObjHdr = NULL;
    mof_uofObjChunkHdr_T* uofObjChunkHdr = NULL;   
    mof_uofObjChunkHdr_T* suofObjChunkHdr = NULL;  
    
    unsigned int NumUofChunks = 0;
    unsigned int NumSuofChunks = 0;    
    unsigned int* validChunks = 0;
    int retVal = UCLO_SUCCESS;
    unsigned int i = 0; 

    uofObjHdr = (mof_uofObjHdr_T*)mofHandle->uofObjsHdr;
    suofObjHdr = (mof_uofObjHdr_T*)mofHandle->suofObjsHdr;

    if (uofObjHdr != NULL)
    {
        /* uof chunk */
        UCLO_MOF_RETURN_IN_CONDITION (
            (uofObjHdr->numChunks > uofObjHdr->maxChunks),
            "Incorrect UOF chunk ammount inside MOF \n",
            UCLO_BADOBJ);
        NumUofChunks = uofObjHdr->numChunks;
    }

    if (suofObjHdr != NULL)
    {
        /* uof chunk */
        UCLO_MOF_RETURN_IN_CONDITION (
            (suofObjHdr->numChunks > suofObjHdr->maxChunks),
            "Incorrect SUOF chunk ammount inside MOF \n",
            UCLO_BADOBJ);
        NumSuofChunks = suofObjHdr->numChunks;
    }

    /* allocate uof/suof objects header list*/
    if ((NumUofChunks + NumSuofChunks) != 0)
    {
        mof_objHdr = (mof_objhdr_t *) 
            osalMemAlloc ((NumUofChunks + NumSuofChunks)
            *  sizeof (mof_objhdr_t));
        UCLO_MOF_RETURN_IN_CONDITION (
            (mof_objHdr == NULL),
            "memory allocation fail\n",
            UCLO_FAILURE);
        osalMemSet(mof_objHdr, 0, 
            sizeof ((NumUofChunks + NumSuofChunks) *
            sizeof (mof_objhdr_t)));
    }
    
    mofHandle->objTable.objHdr = mof_objHdr;
    validChunks = &mofHandle->objTable.numObjs;
    *validChunks = 0;
    
    uofObjChunkHdr = (mof_uofObjChunkHdr_T*)
        ((long)uofObjHdr + sizeof (mof_uofObjHdr_T));
    
    suofObjChunkHdr = (mof_uofObjChunkHdr_T*)
        ((long)suofObjHdr + sizeof (mof_uofObjHdr_T));

    /* map uof objects */
    for (i = 0; i < NumUofChunks; i++)
    {
        /* Init handle base on uof image info */
        if((strncmp ((char *)(uofObjChunkHdr[i].chunkId), 
            UOF_IMAG, MOF_OBJ_CHUNKID_LEN)) != 0)
        {
            PRINTF("unsupported UOF object chunk is detected\n");
            continue;
        }

        retVal = UcLo_mapMofUofObj 
            (mofHandle, &mof_objHdr[*validChunks], &uofObjChunkHdr[i]);
        
        if (retVal != UCLO_SUCCESS)
        {
            return retVal;
        }
        
        (*validChunks)++;    
    }

    /* map suof objects */
    for (i = 0 ; i < NumSuofChunks; i++)
    {
        /* Init handle base on uof image info */
        if((strncmp ((char *)(suofObjChunkHdr[i].chunkId), 
            SUOF_IMAG, MOF_OBJ_CHUNKID_LEN)) != 0)
        {
            PRINTF("unsupported SUOF object chunk is detected\n");
            continue;
        }

        retVal = UcLo_mapMofUofObj 
            (mofHandle, &mof_objHdr[*validChunks], &suofObjChunkHdr[i]);
        
        if (retVal != UCLO_SUCCESS)
        {
            return retVal;
        }
        
        (*validChunks)++;    
    }
    
    /* uof chunk */
    UCLO_MOF_RETURN_IN_CONDITION (
        ((NumUofChunks + NumSuofChunks) != *validChunks),
        "Inconsistent UOF/SUOF chunk ammount\n",
        UCLO_BADOBJ);

    return retVal;
}


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parse MOF file string table object and map it to mof handle
 *
 * @param handle - IN
 * @param mofFilechunkHdr - IN  symbol chunk header
 *
 * @retval  UCLO_SUCCESS, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
static int 
UcLo_mapMofSymObjs (uclo_mofhandle_t *mofHandle, 
                         mof_fileChunkHdr_T* mofFilechunkHdr)
{
    char** symStr = (char**)&mofHandle->symStr;
    unsigned int* symSize = &mofHandle->symSize;
    unsigned long mofEnd = (long)mofHandle->mofBuf + (long)mofHandle->mofSize;
    mof_strTableObj_T* strTableObj = NULL;
    
    /* string table */
    *symSize = *(unsigned int*)
        ((long)mofFilechunkHdr->offset + mofHandle->mofBuf);
    *symStr = (char *)(mofHandle->mofBuf + 
        (long)mofFilechunkHdr->offset + sizeof(strTableObj->tabLength));
    
    /* 4 bytes aligned */
    UCLO_MOF_RETURN_IN_CONDITION (
        ((*symSize % sizeof (unsigned int)) != 0),
        "unaligned string table size inside MOF \n",
        UCLO_BADOBJ);

    /* string table bounday check */
    UCLO_MOF_RETURN_IN_CONDITION (
        ((mofFilechunkHdr->size !=
        (*symSize + sizeof(strTableObj->tabLength)))||
        (mofFilechunkHdr->size == 0)),
        "incorrect stringTable size inside MOF\n",
        UCLO_BADOBJ);

    UCLO_MOF_RETURN_IN_CONDITION (
        (((long)*symStr + *symSize) > mofEnd),
        "incorrect String table inside MOF \n",
        UCLO_BADOBJ);
    
    /* end of stringTable should be 0 */
    UCLO_MOF_RETURN_IN_CONDITION (
        ((*(char *)(*symStr + *symSize - sizeof (char))) != 0),
        "incorrect tableString end of MOF \n",
        UCLO_BADOBJ);

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parse MOF file UOF object and map it to mof handle
 *
 * @param handle - IN
 * @param mofFilechunkHdr - IN  MOF file chunk header
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADOBJ
 * 
 *****************************************************************************/
static int
UcLo_mapMofChunk (uclo_mofhandle_t* mofHandle,
                    mof_fileChunkHdr_T* mofFilechunkHdr)
{
    long mofEnd = (long)mofHandle->mofBuf + (long)mofHandle->mofSize;  
    int retVal = UCLO_SUCCESS;

    if (!strncmp (mofFilechunkHdr->chunkId, SYM_OBJS, MOF_OBJ_ID_LEN))
    {
        UCLO_MOF_RETURN_IN_CONDITION (
            (mofHandle->symStr != 0),
            ("More than 1 SYM_OBJS are detected inside MOF \n"),
            UCLO_BADOBJ);
        
        retVal = UcLo_mapMofSymObjs (mofHandle, mofFilechunkHdr);
        if (retVal != UCLO_SUCCESS)
        {
            return retVal;
        }
    }
    else if (!strncmp 
        (mofFilechunkHdr->chunkId, UOF_OBJS, MOF_OBJ_ID_LEN))
    {
        UCLO_MOF_RETURN_IN_CONDITION (
            (mofHandle->uofObjsHdr != 0),
            ("More than 1 UOF_OBJS are detected inside MOF \n"),
            UCLO_BADOBJ);
    
        mofHandle->uofObjsHdr = mofHandle->mofBuf + 
            (long)mofFilechunkHdr->offset;
    }
    else if (!strncmp 
        (mofFilechunkHdr->chunkId, SUOF_OBJS, MOF_OBJ_ID_LEN))
    {
        UCLO_MOF_RETURN_IN_CONDITION (
            (mofHandle->suofObjsHdr != 0),
            ("More than 1 SUOF_OBJS are detected inside MOF \n"),
            UCLO_BADOBJ);
    
        mofHandle->suofObjsHdr = mofHandle->mofBuf + 
            (long)mofFilechunkHdr->offset;
    }
    else
    {
        PRINTF("unsupported MOF file chunk is detected\n");
    }

    /* offset should be between fileChunkHdrs & file end */
    UCLO_MOF_RETURN_IN_CONDITION (
            ((mofFilechunkHdr->offset < (sizeof(mof_fileHdr_T))) ||
            (mofFilechunkHdr->offset > mofHandle->mofSize)),
            ("incorrect chunk offset inside MOF \n"),
            UCLO_BADOBJ);

        /* chunk end should less than file end */
    UCLO_MOF_RETURN_IN_CONDITION (
             (((long)mofHandle->mofBuf + mofFilechunkHdr->offset + 
             mofFilechunkHdr->size) > (long)mofEnd),
             ("chunk end is overflow inside MOF\n"),
             UCLO_BADOBJ);
        
    return retVal;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parses MOF/UOF file format, returns required UOF objects
 *     While the expected uofName is NULL, this function would parse the MOF 
 *     object and initialize mof_handle, but no UOF object would be returned.
 *
 * @param handle - IN
 * @param mofPtr - IN  Start address of MOF/UOF buffer
 * @param mofSize - IN MOF/UOF length
 * @param uofName - IN The expected UOF file name
 * @param uofPtr - OUT Start address of expected UOF
 * @param uofSize - OUT UOF length
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
int 
UcLo_MapMofAddr (icp_firml_handle_t *handle, 
                     void *mofPtr, 
                     uint64 mofSize, 
                     char* uofName, 
                     char** uofPtr, 
                     unsigned int* uofSize)
{
    uclo_mofhandle_t* mofHandle = NULL;
    mof_fileChunkHdr_T* mofFilechunkHdr = NULL;
    unsigned short NumMofChunks = 0;
    int retVal = UCLO_SUCCESS;
    unsigned int i = 0;

    UCLO_MOF_RETURN_IN_CONDITION (
        (handle == NULL),
        "input paramter handle is NULL\n",
        UCLO_BADARG);
    handle->mofHandle = NULL;

    UCLO_MOF_RETURN_IN_CONDITION (
        ((mofPtr == NULL) || (mofSize == 0)),
        "input paramter mofPtr/mofSize is NULL\n",
        UCLO_BADARG);    

    /* UOF_FID (0xc6c2) for uof object 
     * SUOF_FID for suof object
     */
    if ((((uof_fileHdr_T *) mofPtr)->fileId == UOF_FID) ||
        (((suof_fileHdr_T *) mofPtr)->fileId == SUOF_FID))
    {
        if (uofPtr != NULL)
        {
            *uofPtr = mofPtr;
        }
        if (uofSize != NULL)
        {
            *uofSize = (unsigned int)mofSize;
        }
        
        return UCLO_SUCCESS;
    }
    /* return BADOBJ if neither UOF/SUOF nor MOF */
    else if (((mof_fileHdr_T *)mofPtr)->fileId != MOF_FID)
    {
        PRINTF("unsupported file format\n");
        return (UCLO_BADOBJ);                 
    }

    /* map MOF header */
    retVal = UcLo_mapMofFileHdr (handle, mofPtr, mofSize);
    if (retVal != UCLO_SUCCESS)
    {
        UcLo_DelMof (handle);
        return retVal;
    }
    
    mofHandle = (uclo_mofhandle_t*)handle->mofHandle;
    mofFilechunkHdr = (mof_fileChunkHdr_T *)
        ((long)mofPtr + sizeof (mof_fileHdr_T));
    NumMofChunks = ((mof_fileHdr_T *)mofPtr)->numChunks;
        
    /* Parse MOF file chunks */
    for (i = 0; i < NumMofChunks; i++)
    {
        /* map sym_objs */
        retVal = UcLo_mapMofChunk (mofHandle, &mofFilechunkHdr[i]);
        if (retVal != UCLO_SUCCESS)
        {
            UcLo_DelMof (handle);
            return retVal;
        }
    }

    /* both sym_objs and uof_objs should be available */
    if ((mofHandle->symStr == 0) || 
        ((mofHandle->uofObjsHdr == 0) && 
        (mofHandle->suofObjsHdr == 0)))
    {
        UcLo_DelMof (handle);
        return UCLO_BADOBJ;
    }
    
    /* map uof_objs & suof_objs after sym_objs */
    retVal = UcLo_mapMofUofObjs (mofHandle);
    if (retVal != UCLO_SUCCESS)
    {
        UcLo_DelMof (handle);
        return retVal;
    }
    
    /* Seek specified uof object in mofObjHdr */
    if (uofName != NULL)
    {
        retVal = UcLo_mapMofSeekUof (mofHandle, uofName, uofPtr, uofSize);
        if (retVal != UCLO_SUCCESS)
        {
            UcLo_DelMof (handle);
            return retVal;
        }
    }

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Release MOF handle created during UcLo_MapMofAddr
 *
 * @param handle - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG
 * 
 * 
 *****************************************************************************/
int 
UcLo_DelMof (icp_firml_handle_t *handle)
{
    if (!handle)
    {
        return UCLO_BADARG;
    }

    if ((uclo_mofhandle_t *)(handle->mofHandle) != NULL)
    {
        if (((uclo_mofhandle_t *)(handle->mofHandle))->objTable.objHdr != NULL)
        {
        
            osalMemFree(((uclo_mofhandle_t *)(handle->mofHandle))
                        ->objTable.objHdr);
            ((uclo_mofhandle_t *)(handle->mofHandle))->objTable.objHdr = NULL;
        }

        osalMemFree((uclo_mofhandle_t *)(handle->mofHandle));
        handle->mofHandle = NULL;
    }
    else
    {
        return UCLO_NOOBJ;
    }

    return UCLO_SUCCESS;
}


