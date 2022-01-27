/**
 **************************************************************************
 * @file uclo_ivd.c
 *
 * @description
 *      This file provides Ucode Object File Loader facilities
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

#define MAX_NAME_LEN    128         /* maximum string length */
#define MAX_BINDINFO    1024        /* maximum bind information */
#define COMMENT_TOKEN_LENGTH   2    /* comment token length */

/**
 * @description bind symbol information in ivd file or buffer 
*/
typedef struct bindinfo_s {
    char chipName[MAX_NAME_LEN];    /* simulation chip name */
    char image[MAX_NAME_LEN];       /* uof image name */
    char symbol[MAX_NAME_LEN];      /* symbol name */
    uint64 value;               /* value to be assigned to the symbol */
} bindinfo_t;

/* The following is a kludge. This should probably be part of the objHandle */
static bindinfo_t BindInfo[MAX_BINDINFO];
unsigned int NumBindInfo = 0;


int UcLo_parseBindInfo(char *lineBuf);
static int storeBindInfo(char *chipName, 
                         char *image, 
                         char *symbol, 
                         uint64 value);
int UcLo_bindIvd(void *objHandle);

int UcLo_isChipNameMatch(char *name);

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Initialize the import-variables
 *
 * @param objHandle - IN
 * @param 
 *
 * @retval  UCLO_SUCCESS, UCLO_NOOBJ, UCLO_BADARG, UCLO_IMGNOTFND,
                UCLO_SYMNOTFND
 * 
 * 
 *****************************************************************************/
int 
UcLo_bindIvd(void *objHandle)
{
    int status=UCLO_SUCCESS, stat=UCLO_SUCCESS, ii=0, jj=0, giveWarning=1;

    if (NumBindInfo == 0) 
    {
        return (UCLO_SUCCESS);
    }
    for (ii=0; ii < MAX_BINDINFO; ii++)
    {
        if ((BindInfo[ii].image[0] != '\0') &&
            (BindInfo[ii].symbol[0] != '\0'))
        {
            if (!UcLo_isChipNameMatch(BindInfo[ii].chipName))
            {
                continue;
            }

            if ((stat = UcLo_BindSymbol(objHandle, BindInfo[ii].image, \
                        BindInfo[ii].symbol, BindInfo[ii].value))!=UCLO_SUCCESS)
            {
                if (stat == UCLO_IMGNOTFND)
                {
                    /* look thru the symbol info that we have already processed
                     * to avoid reporting the same image not found multiple
                     * times. if the image name that we are currently 
                     * processing appears in the list ahead of it then that 
                     * implies that we have already reported the image as not 
                     * found. */
                    giveWarning = 1;
                    for (jj = 0; jj < ii; jj++)
                    {
                        if (strcmp(BindInfo[ii].image, BindInfo[jj].image) == 0)
                        {
                            giveWarning = 0;
                            break;
                        }
                    }
                    if (giveWarning) 
                    {
                        DBGINFO(("WARNING: UOF image '%s' not found.\n", \
                                      BindInfo[ii].image));
                    }    
                }
                else if (stat == UCLO_SYMNOTFND) 
                {
                    /* don't report unfound symbol if image name is "*" because it would have 
                       already been reported in UcLo_bindAllImagesSym() */
                    if (strcmp(BindInfo[ii].image, "*") != 0) 
                    {
                        DBGINFO(("WARNING: Symbol '%s' \
                                  not found in UOF image '%s'.\n", \
                                BindInfo[ii].symbol, BindInfo[ii].image));
                    }    
                }
                else
                {
                    /* had some other fatal error */
                    return (stat);
                }

                /* remember the first missing file or symbol status, 
                 * but keep processing symbols */
                if (status == UCLO_SUCCESS) 
                {
                    status = stat;
                }    
            }
        }
    }
    return (status);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     store information to initialize the import-variable
 *
 * @param chipName - IN
 * @param image - IN
 * @param symbol - IN
 * @param value - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE
 * 
 * 
 *****************************************************************************/
static int 
storeBindInfo(char *chipName, 
              char *image, 
              char *symbol, 
              uint64 value)
{
    int tt=MAX_BINDINFO, ii;

    for(ii=0; ii < MAX_BINDINFO; ii++)
    {
        if((!strcmp(BindInfo[ii].chipName, chipName)) &&
            (!strcmp(BindInfo[ii].image, image)) &&
            (!strcmp(BindInfo[ii].symbol, symbol)))
        {
            tt = ii;
            break;
        }else
         {
            if(BindInfo[ii].image[0] == '\0')
            {
               tt = ii;
            }   
         }
    }
    if(tt == MAX_BINDINFO)
    {
        ERRINFO(("Too many bind-info; exceed %d bindSymbols for image %s.\n", \
                    MAX_BINDINFO, image));
        return (UCLO_FAILURE);
    }

    strncpy(BindInfo[tt].chipName, chipName, sizeof(BindInfo[tt].chipName)-1);
    BindInfo[tt].chipName[sizeof(BindInfo[tt].chipName)-1] = '\0';
    strncpy(BindInfo[tt].image, image, sizeof(BindInfo[tt].image)-1);
    BindInfo[tt].image[sizeof(BindInfo[tt].chipName)-1] = '\0';
    strncpy(BindInfo[tt].symbol, symbol, sizeof(BindInfo[tt].symbol)-1);
    BindInfo[tt].symbol[sizeof(BindInfo[tt].chipName)-1] = '\0';
    BindInfo[tt].value = value;
    NumBindInfo++;

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parse a string for import-variable initialization data
 *
 * @param lineBuf - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE
 * 
 * 
 *****************************************************************************/
int 
UcLo_parseBindInfo(char *lineBuf)
{
    char *chipName=NULL, *pSavPtr=NULL, *image=NULL, *symb=NULL;
    uint64  value=0;

    if (lineBuf[0] == '\n') 
    {
        return (0);
    } 
    pSavPtr = UcLo_stripLeadBlanks(lineBuf);

    if(pSavPtr == NULL) 
    {
        return (0);    
    }
    /* Ignore lines that only have spaces */
    if ((*pSavPtr == '\n') || (*pSavPtr == '\0')) 
    {
        return (0);
    }    

    /* Lines that begin with // are ignored as comment lines */
    if (!strncmp(pSavPtr, "//", COMMENT_TOKEN_LENGTH)) 
    {
        return (0);    /* skip comment */
    }
    /* Parse out the chip name. It may be enclosed in quotes. */
    if (pSavPtr[0] == '"')
    {
        if (pSavPtr[1] == '"')
        {
            pSavPtr += COMMENT_TOKEN_LENGTH;
            chipName = "";
        }
        else 
        {
            chipName = UcLo_strtoken(pSavPtr+1, "\"\n", &pSavPtr);
        }    
    }
    else 
    {
        chipName = UcLo_strtoken(pSavPtr, " \t\n", &pSavPtr);
    }    

    if (pSavPtr == NULL)
    {
        ERRINFO(("Incorrect IVD format -- missing image name\n"));
        return (UCLO_FAILURE);
    }
    
    /* Parse out the image name. It may be enclosed in quotes. */
    pSavPtr = UcLo_stripLeadBlanks(pSavPtr);
    if (pSavPtr[0] == '"')
    {
        if (pSavPtr[1] == '"')
        {
            ERRINFO(("Incorrect IVD format -- missing image name\n"));
            return (UCLO_FAILURE);
        }
        else 
        {
            image = UcLo_strtoken(pSavPtr+1, "\"\n", &pSavPtr);
        }    
    }
    else 
    {
        image = UcLo_strtoken(pSavPtr, " \t", &pSavPtr);
    }    
    if ((image == NULL) || (image[0] == '\0'))
    {
        ERRINFO(("Incorrect IVD format -- missing or invalid image name\n"));
        return (UCLO_FAILURE);
    }

    /* Parse out symbol name */
    if (!pSavPtr)
    {
        ERRINFO(("Incorrect IVD format -- missing symbol name\n"));
        return (UCLO_FAILURE);
    }
    pSavPtr = UcLo_stripLeadBlanks(pSavPtr);
    symb = UcLo_strtoken(pSavPtr, " \t\n", &pSavPtr);
    if ((symb == NULL) || (symb[0] == '\0'))
    {
        ERRINFO(("Incorrect IVD format -- missing or invalid symbol name\n"));
        return (UCLO_FAILURE);
    }

    /* Parse out symbol value. */
    if (UcLo_parseNumEx(pSavPtr, &value))
    {
        ERRINFO(("Incorrect IVD format -- missing or invalid value\n"));
        return (UCLO_FAILURE);
    }

    if(chipName == NULL) 
    {
        chipName = "";
    }
    if (storeBindInfo(chipName, image, symb, value))
    {
        ERRINFO(("storeBindInfo error\n"));  
        return (UCLO_FAILURE);
    }
    return (UCLO_SUCCESS);
}


/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Removes all the information and resources that were created as a 
 *      result of calls to UcLo_LoadIvdFile, or UcLo_LoadIvdBuf
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param - none
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
void 
UcLo_DelIvd(void)
{
    int ii = 0;
    for(ii=0; ii < MAX_BINDINFO; ii++)
    {
        BindInfo[ii].chipName[0] = '\0';
        BindInfo[ii].image[0] = '\0';
        BindInfo[ii].symbol[0] = '\0';
    }
    NumBindInfo = 0;
    return;
}


/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Extracts the information from string buffer to be used to initialize 
 *      import-variables.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param ivdBuf - IN Character string pointer to the buffer
 * @param ivdBufLen - IN Buffer string length
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/
int 
UcLo_LoadIvdBuf(char *ivdBuf, 
                unsigned int ivdBufLen)
{
    char *pLinePtr=NULL, *pSavPtr=NULL, *pBufEnd=NULL;
    int    status=UCLO_SUCCESS;

    if((ivdBuf == NULL) || (ivdBufLen == 0)) 
    {
        ERRINFO(("bad argument, ivdBuf is NULL or ivdBufLen is 0\n "));  
        return (UCLO_BADARG);
    }
    pSavPtr = ivdBuf;
    pBufEnd = pSavPtr + ivdBufLen;

    /* delete the existing bindInfo if necessary */
    if(NumBindInfo) 
    {
       UcLo_DelIvd();
    }   

    while(pSavPtr && *pSavPtr && (pSavPtr < pBufEnd))
    {
        pLinePtr = UcLo_strtoken(NULL, "\n", &pSavPtr);
        if(pLinePtr && *pLinePtr)
        {
            if((status = UcLo_parseBindInfo(pLinePtr))) 
            {
                break;
            }
        }
    }
    return (status);
}
