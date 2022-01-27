/**
**************************************************************************
* @file uclo_helper.c
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

#include "uclo_helper.h"
#define HEX 16
#define DECIMAL 10
#define OCTAL 8
#define MAX_DIGIT 100

static const unsigned int Int64Bits = 64;

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parses either an octal, decimal, or hex integer number from
 *
 * @param pStr - IN
 * @param pNum - OUT
 *
 * @retval  0: SUCCESS, 1:FAILURE
 * 
 * 
 *****************************************************************************/
int 
UcLo_parseNum(char *pStr, 
              int *pNum)
{
    char *token = NULL, *endPtr = NULL, *digitCheck = NULL;
    int    base = DECIMAL;
    char digit[MAX_DIGIT+0x3], *pDigit = digit;

    if(!pStr || (*pStr == '\0')) 
    {
        return (1);
    }    
    /* strip leading spaces */
    token = UcLo_stripLeadBlanks(pStr);

    if((*token == '-') || (*token == '+')) 
    {
        *pDigit++ = token[0];
        digitCheck = token+1;
    }
    else 
    {
        digitCheck = token;
    }    

    if(!isdigit(*digitCheck))
    {
        return (1);
    }    

    if(token[0] == '0') 
    {
        *pDigit++ = token[0];
        if((token[1] == 'x') || (token[1] == 'X'))
        {
            *pDigit++ = token[1];
            base = HEX;
            digitCheck = token+0x2;
        }      
        else 
        {
            base = OCTAL;
            digitCheck = token+1;
        }      
    }
    UcLo_stripFollowBlanks(digitCheck, base, pDigit);

    STR_TO_32(digit, base, pNum, endPtr);

    return (0);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parses either an octal, decimal, or hex integer number from
 *
 * @param pStr - IN
 * @param pNum - OUT
 *
 * @retval  0: SUCCESS, 1:FAILURE
 * 
 * 
 *****************************************************************************/
int 
UcLo_parseNumEx(char *pStr, 
              uint64 *pNum)
{
    char *token = NULL, *endPtr = NULL, *digitCheck = NULL;
    int    base = DECIMAL;
    int  negative = 0;
    uint64 positiveNum;
    char digit[MAX_DIGIT+0x3], *pDigit = digit;

    if(!pStr || (*pStr == '\0')) 
    {
        return (1);
    }    
    /* strip leading spaces */
    token = UcLo_stripLeadBlanks(pStr);

    if((*token == '-') || (*token == '+')) 
    {
        if (*token == '-')
        {
            negative = 1;
        }
        digitCheck = ++token;
    }
    else 
    {
        digitCheck = token;
    }    

    if(!isdigit(*digitCheck))
    {
        return (1);
    }    

    if(token[0] == '0') 
    {
        *pDigit++ = token[0];
        if((token[1] == 'x') || (token[1] == 'X'))
        {
            *pDigit++ = token[1];
            base = HEX;
            digitCheck = token+0x2;
        }      
        else 
        {
            base = OCTAL;
            digitCheck = token+1;
        }      
    }
    UcLo_stripFollowBlanks(digitCheck, base, pDigit);
    
    STR_TO_64(digit, base, &positiveNum, endPtr);

    if (negative) 
    { 
       *pNum = -positiveNum;
    } else {
       *pNum = positiveNum;
    }

    return (0);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Remove leading blanks or tabs from string
 *
 * @param pStr - IN
 *
 * @retval   pointer to the string
 * 
 * 
 *****************************************************************************/
char *
UcLo_stripLeadBlanks(char *pStr)
{
    char *pC = NULL;

    for (pC = pStr; pC && ((*pC == '\t') ||( *pC == ' ')); pC++) ;
    return (pC);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Remove following blanks or tabs from string
 *
 * @param pStr - IN
 *        base - IN
 *        pDigit - OUT
 *
 * @retval   None
 * 
 * 
 *****************************************************************************/
void
UcLo_stripFollowBlanks(char *pStr, int base, char *pDigit)
{
    char *digitCheck = NULL;
    int i;

    for (i=0, digitCheck = pStr; 
         (i < MAX_DIGIT) && (*digitCheck != '\0');
         i++, digitCheck++, pDigit++)
    {
        *pDigit = *digitCheck;
        switch(base)
        {
        case HEX:
            if(!UcLo_isHex(*digitCheck))
            {
                *pDigit='\0';
                return;
            }
            break;
        case DECIMAL:
            if(!UcLo_isDecimal(*digitCheck))
            {
                *pDigit='\0';
                return;
            }
            break;
        case OCTAL:
            if(!UcLo_isOctal(*digitCheck))
            {
                *pDigit='\0';
                return;
            }
            break;
        default:
            return;
        }
    }
    *pDigit='\0';    
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Check if it is hex integer number
 *
 * @param number - IN
 *
 * @retval   1 hex integer number
 *           0 not hex integer number
 * 
 * 
 *****************************************************************************/
int
UcLo_isHex(char number)
{
    if(((number >= '0') && (number <= '9'))
       || ((number >= 'a') && (number <= 'f'))
       || ((number >= 'A') && (number <= 'F')))
    {
        return 1;
    }
    return 0;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Check if it is decimal integer number
 *
 * @param number - IN
 *
 * @retval   1 decimal integer number
 *           0 not decimal integer number
 * 
 * 
 *****************************************************************************/
int
UcLo_isDecimal(char number)
{
    if((number >= '0') && (number <= '9'))
    {
        return 1;
    }
    return 0;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Check if it is octal integer number
 *
 * @param number - IN
 *
 * @retval   1 octal integer number
 *           0 not octal integer number
 * 
 * 
 *****************************************************************************/
int
UcLo_isOctal(char number)
{
    if((number >= '0') && (number <= '7'))
    {
        return 1;
    }
    return 0;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Compare two strings lexicographically. This function is similar
 *     to ANSI strcmp, except it allows case-insensitivity comparision.
 *
 * @param pStr1 - IN
 * @param pStr2 - IN
 * @param sensitive - IN
 *
 * @retval   -1 if lex(pStr1) < lex(pStr2)
 *			0 if lex(pStr1 == lex(pStr2)
 *			1 if lex(pStr1) > lex(pStr2)
 * 
 * 
 *****************************************************************************/
int 
UcLo_strcmp(const char *pStr1, 
            const char *pStr2, 
            int sensitive)
{
    char c1 = 0, c2 = 0;
    int result = 0;

    while(1)
    {
        c1 = *pStr1;  c2 = *pStr2;

        if(!sensitive)
        {
            c1 = TOLOWER(c1);
            c2 = TOLOWER(c2);
        }

        if(c1 < c2)
        {
            result = -1;
        }    
        else if(c1 > c2) 
        {
            result = 1;
        }    
        if(result || !c1 || !c2) 
        {
            break;
        }    
        pStr1++;  pStr2++;
    }
    return (result);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Extracts the token between the specified delimiters.
 *
 * @param pStr1 - IN
 * @param pDlim - IN
 * @param savPtr - IN
 *
 * @retval  The pointer to the token, or NULL.
 * 
 * 
 *****************************************************************************/
char *
UcLo_strtoken(char *pStr, 
              char *pDlim, 
              char **savPtr)
{
    char *pBegStr = NULL, *pS = NULL;

    /* if pStr is NULL, then use the last reference pointer */
    if (!pStr) 
    {
        if (savPtr && *savPtr)
        {
            pStr = *savPtr;
        }    
        else
        {
            return (NULL);
        }    
    }

    if (savPtr)
    {
        *savPtr = NULL;
    } 
    /* locate the first byte of pStr that does not occur in pDlim */
    for (pBegStr = pS = pStr; *pS; pS++) 
    {
        if (!(strchr(pDlim, *pS))) 
        {
            pBegStr = pS;
            break;
        }
    }

    /* locate the first byte of pStr that does occur in pDlim */
    for (; *pS; pS++) {
        if ((strchr(pDlim, *pS))) 
        {
            *pS = '\0';
            if (savPtr)
            {
                *savPtr = pS + 1;
            }   
            break;
        }
    }

    return (pBegStr);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Calculate the CRC checksum
 *
 * @param reg - IN
 * @param ch - IN
 *
 * @retval  The calculated checksum value for the string
 * 
 * 
 *****************************************************************************/
static unsigned int 
Crc_Calc(unsigned int reg, 
         int ch)
{
    int  ii = 0;
    unsigned int topbit = CRC_BITMASK(CRC_WIDTH - 1);
    unsigned int inbyte = (unsigned int) ((reg >> 0x18) ^ ch);

    reg ^= inbyte << (CRC_WIDTH - 0x8);
    for (ii = 0; ii < 0x8; ii++)
    {
        if (reg & topbit) 
        {
            reg = (reg << 1) ^ CRC_POLY;
        }    
        else 
        {
            reg <<= 1;
        }    
    } 
    return (reg & CRC_WIDTHMASK(CRC_WIDTH));
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Calculate the CRC checksum of a string
 *
 * @param pChar - IN
 * @param numChar - IN
 *
 * @retval  The calculated checksum value for the string
 * 
 * 
 *****************************************************************************/
unsigned int 
UcLo_strChecksum(char *pChar, 
                 int numChar)
{
    unsigned int chksum = 0;

    if(pChar) 
    {
        while (numChar--) 
        {
            chksum = Crc_Calc(chksum, *pChar++);
        }     
    }   
    return (chksum);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Sets a specified range of bits of 'word' equal to the bits of
 * 	 'value'--starting from the zero bit-position of 'value'.
 *
 * @param word - IN
 * @param startBit - IN
  * @param fieldLen - IN
 * @param value - IN
 *
 * @retval  The result of the operation
 * 
 * 
 *****************************************************************************/
uint64 
UcLo_setField64(uint64 word, 
                int startBit, 
                int fieldLen, 
                uint64 value)
{
    unsigned int stopBit = 0;
    uint64 mask = 0, bit = 0;

    /* check that the field is within bound of the word */
    if((stopBit = (startBit + fieldLen)) > Int64Bits)
    {
        DBGINFO(("UcLo_setField64: field exceeds the size of the word\n"));

        return (word);
    }

    /* determine the mask */
    for(bit = startBit; bit < stopBit; bit++)
    {
        mask |= ((uint64)1 << bit);
    }   

    /* Assign the first 'numBit' bits of 'value' to the field of 'fieldLen'
    starting at the 'startBit' of 'word'. */
    word &= ~mask;
    return ((word |= mask & (value << startBit)));
}
