/****************************************************************************
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
 ***************************************************************************/

/*
  Copyright (C) 2002-2010 Mark Adler, all rights reserved
  version 2.1, 4 Apr 2010

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the author be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Mark Adler    madler@alumni.caltech.edu
*/


/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include "cpa.h"
#include "cpa_dc.h"

#include "lac_common.h"
#include "dc_flattree.h"

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Return need bits from the input stream.
 *      In this function, we've added the option to switch to another stream
 *      if the stream pointed by "state->in" runs out.
 *
 *      Intel wrote this function based on the bits() function from the
 *      file "puff.c" provided by Mark Adler. This current function is
 *      a modified version from the bits() function in the file puff.c
 *
 * @description
 *      Return need bits from the input stream.  This always leaves less than
 *      eight bits in the buffer. bits() works properly for need == 0.
 *
 * Format notes:
 * - Bits are stored in bytes from the least significant bit to the most
 *   significant bit.  Therefore bits are dropped from the bottom of the bit
 *   buffer, using shift right, and new bytes are appended to the top of the
 *   bit buffer, using shift left.
 *
 * @param[in/out]   state            Input and output state
 * @param[in]       need             Number of bits to read
 * @param[in]       pSource          Address of the new stream to switch to
 *                                   if state->in runs out.
 * @param[in]       sourceSize       Size of data source buffer
 *
 * @retval          bitValue         Value represented by the number of
 *                                   needed bits.
 * @retval   DC_ERROR_OUT_OF_INPUT   Could not read all the bits requested
 *                                   in the input stream
 *****************************************************************************/
STATIC Cpa32S
bits(state_t *state, Cpa32S need, Cpa8U *pSource, Cpa32U sourceSize)
{
    Cpa64S val = 0;           /* bit accumulator (can use up to 20 bits) */
    /* load at least "need" bits into "val" */
    val = state->bitbuf;

    while (state->bitcnt < need)
    {
        /* If we reach the end of the residues in the registers
         * then switch to the stream.
         */
        if (state->incnt == state->inlen && ((state->bitcnt - need) <= 0))
        {
            if (NULL != pSource)
            {
                state->in = pSource;
                state->incnt = 0;
                state->inlen = sourceSize;
            }
            else
            {
                return DC_ERROR_OUT_OF_INPUT; /* out of input */
            }
        }
        /* load 8 bits */
        val |= (Cpa64S)(state->in[state->incnt++]) << state->bitcnt;
        state->bitcnt += 8;
    }

    /* drop need bits and update buffer, always zero to seven bits left */
    state->bitbuf = (Cpa32S)(val >> need);
    state->bitcnt -= need;
    state->bits_total += need;
    /* return need bits, zeroing the bits above that */
    return (Cpa32S)(val & ((1L << need) - 1));
}


/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Decode a code from the stream s using huffman table.
 *      Intel wrote this function based on the decode() function from the
 *      file "puff.c" provided by Mark Adler. This current function is
 *      a modified version from the decode() function in the file puff.c
 *
 * @description
 *      Decode a code from the stream s using huffman table h. Return the
 *      symbol or a negative value if there is an error.  If all of the
 *      lengths are zero, i.e. an empty code, or if the code is incomplete
 *      and an invalid code is received, then -9 is returned after reading
 *      MAXBITS bits.
 *
 * Format notes:
 * - The codes as stored in the compressed data are bit-reversed relative to
 *   a simple integer ordering of codes of the same lengths.  Hence below the
 *   bits are pulled from the compressed data one at a time and used to
 *   build the code value reversed from what is in the stream in order to
 *   permit simple integer comparisons for decoding.  A table-based decoding
 *   scheme (as used in zlib) does not need to do this reversal.
 *
 * - The first code for the shortest length is all zeros.  Subsequent codes of
 *   the same length are simply integer increments of the previous code.  When
 *   moving up a length, a zero bit is appended to the code.  For a complete
 *   code, the last code of the longest length will be all ones.
 *
 * - Incomplete codes are handled by this decoder, since they are permitted
 *   in the deflate format.  See the format notes for fixed() and dynamic().
 *
 * @param[in]       state            Input and output state
 * @param[in/out]   hTable           Huffman table used to perform the
 *                                   decoding
 * @param[in]       pSource          Address of data source buffer.
 * @param[in]       sourceSize       Size of data source buffer
 *
 * @retval          symbol           Symbol value
 * @retval   DC_ERROR_INVALID_BIT    Invalid bit error returned if the stream
 *                                   pointer goes beyond the stream length.
 * @retval   DC_ERROR_OUT_OF_CODES   Invalid code value
 *****************************************************************************/
STATIC Cpa32S
decode(state_t *state, huffman_t *hTable, Cpa8U *pSource, Cpa32U sourceSize)
{
    Cpa32S len = 0;    /* current number of bits in code */
    Cpa32S code = 0;   /* len bits being decoded */
    Cpa32S first = 0;  /* first code of length len */
    Cpa32S count = 0;  /* number of codes of length len */
    Cpa32S index = 0;  /* index of first code of length len in symbol table */
    Cpa8S status = 0;

    code = first = index = 0;
    for (len=1; len<=MAXBITS; len++)
    {
        code |= bits(state, 1, pSource, sourceSize);
        status = CHKBITS(state->incnt, state->inlen);
        if (status < 0)
        {
            return DC_ERROR_INVALID_BIT;
        }
        count = hTable->count[len];
        if (code - count < first)       /* if length len, return symbol */
        {
            return hTable->symbol[index + (code - first)];
        }
        index += count;                 /* else update for next length */
        first += count;
        first <<= 1;
        code <<= 1;
    }
    return DC_ERROR_OUT_OF_CODES;
}

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Construct tables required to decode Huffman codes.
 *      Intel wrote this function based on the construct() function from the
 *      file "puff.c" provided by Mark Adler. This current function is
 *      a modified version from the construct() function in the file puff.c
 *
 * @description
 * Given the list of code lengths length[0..n-1] representing a canonical
 * Huffman code for n symbols, construct the tables required to decode those
 * codes.  Those tables are the number of codes of each length, and the symbols
 * sorted by length, retaining their original order within each length.  The
 * return value is zero for a complete code set, negative for an over-
 * subscribed code set, and positive for an incomplete code set.  The tables
 * can be used if the return value is zero or positive, but they cannot be used
 * if the return value is negative.  If the return value is zero, it is not
 * possible for decode() using that table to return an error--any stream of
 * enough bits will resolve to a symbol.  If the return value is positive, then
 * it is possible for decode() using that table to return an error for received
 * codes past the end of the incomplete lengths.
 *
 * Not used by decode(), but used for error checking, h->count[0] is the number
 * of the n symbols not in the code.  So n - h->count[0] is the number of
 * codes.  This is useful for checking for incomplete codes that have more than
 * one symbol, which is an error in a dynamic block.
 *
 * Assumption: for all i in 0..n-1, 0 <= length[i] <= MAXBITS
 * This is assured by the construction of the length arrays in dynamic() and
 * fixed() and is not verified by construct().
 *
 * Format notes:
 *
 * - Permitted and expected examples of incomplete codes are one of the fixed
 *   codes and any code with a single symbol which in deflate is coded as one
 *   bit instead of zero bits.  See the format notes for fixed() and dynamic().
 *
 * - Within a given code length, the symbols are kept in ascending order for
 *   the code bits definition.
 *
 * @param[in/out]   hTable           Huffman table used to perform the
 *                                   decoding
 * @param[in]       length           Address of length buffer
 * @param[in]       size             Length buffer size
 *
 * @retval          0                0 for valid symbol
 *                                   >0 for incomplete symbol
 ****************************************************************************/
STATIC Cpa32S
construct(huffman_t *hTable, Cpa16U *length, Cpa32S size)
{
    Cpa32S symbol = 0;    /* current symbol when stepping through length[]  */
    Cpa32S len = 0;       /* current length when stepping through h->count[]*/
    Cpa32S left = 0;      /* number of possible codes left of current length*/
    Cpa16U offs[MAXBITS+1];/* offsets in symbol table for each length */
    Cpa16U currentLength = 0;
    Cpa16U bit_width_index = 0;

    osalMemSet (hTable->symbol, 0, size * sizeof(Cpa16S));
    osalMemSet (hTable->bit_width, 0, size * sizeof(Cpa8U));
    osalMemSet (hTable->count, 0, (MAXBITS+1) * sizeof(Cpa16S));

    /* Count number of codes of each length */
    for (symbol=0; symbol<size; symbol++)
    {
        if (length[symbol] < BANK_E_MAXLCODES)
        {
           /* assumes lengths are within bounds */
           (hTable->count[length[symbol]])++;
        }
    }
    if (hTable->count[0] == size)          /* no codes! */
    {
        return 0;                       /* complete, but decode() will fail */
    }

    /* Check for an over-subscribed or incomplete set of lengths */
    left = 1;                           /* one possible code of zero length */
    for (len=1; len<=MAXBITS; len++)
    {
        left <<= 1;                     /* one more bit, double codes left */
        left -= hTable->count[len];     /* deduct count from possible codes */
        if (left < 0)                   /* over-subscribed--return negative */
        {
            return left;                /* left > 0 means incomplete */
        }
    }

    /* Generate offsets into symbol table for each length for sorting */
    offs[1] = 0;
    for (len=1; len<MAXBITS; len++)
    {
        offs[len + 1] = offs[len] + hTable->count[len];
    }

    /* Put symbols in table sorted by offset, by symbol
     * order within each length
     */
    for (symbol=0; symbol<size && symbol<BANK_E_ORDER_SIZE; symbol++)
    {
        currentLength = length[symbol];
        if (currentLength > 0 && currentLength< (MAXBITS+1))
        {
            bit_width_index = offs[currentLength];
            if (bit_width_index < BANK_E_MAXLCODES)
            {
                hTable->bit_width[bit_width_index] = currentLength;
                hTable->symbol[bit_width_index] = symbol;
                if (currentLength < MAXBITS+1)
                {
                    offs[currentLength]++;
                }
            }
        }
    }
    /* Return zero for complete set */
    return 0;
}

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Construct tables required to decode Huffman codes.
 *      Intel wrote this function based on the construct() function from the
 *      file "puff.c" provided by Mark Adler. This current function is
 *      a modified version from the construct() function in the file puff.c
 *
 * @description
 * Given the list of code lengths length[0..n-1] representing a canonical
 * Huffman code for n symbols, construct the tables required to decode those
 * codes.  Those tables are the number of codes of each length, and the symbols
 * sorted by length, retaining their original order within each length.  The
 * return value is zero for a complete code set, negative for an over-
 * subscribed code set, and positive for an incomplete code set.  The tables
 * can be used if the return value is zero or positive, but they cannot be used
 * if the return value is negative.  If the return value is zero, it is not
 * possible for decode() using that table to return an error--any stream of
 * enough bits will resolve to a symbol.  If the return value is positive, then
 * it is possible for decode() using that table to return an error for received
 * codes past the end of the incomplete lengths.
 *
 * Not used by decode(), but used for error checking, h->count[0] is the number
 * of the n symbols not in the code.  So n - h->count[0] is the number of
 * codes.  This is useful for checking for incomplete codes that have more than
 * one symbol, which is an error in a dynamic block.
 *
 * Assumption: for all i in 0..n-1, 0 <= length[i] <= MAXBITS
 * This is assured by the construction of the length arrays in dynamic() and
 * fixed() and is not verified by construct().
 *
 * Format notes:
 *
 * - Permitted and expected examples of incomplete codes are one of the fixed
 *   codes and any code with a single symbol which in deflate is coded as one
 *   bit instead of zero bits.  See the format notes for fixed() and dynamic().
 *
 * - Within a given code length, the symbols are kept in ascending order for
 *   the code bits definition.
 *
 * @param[in/out]   hTable           Huffman table used to perform the
 *                                   decoding
 * @param[in]       length           Address of length buffer
 * @param[in]       size             Length buffer size
 *
 * @retval          0                0 for valid symbol
 *                                   >0 for incomplete symbol
 ****************************************************************************/
STATIC Cpa32S
construct2(huffman_t *hTable, Cpa16U *length, Cpa32S size)
{
    Cpa32S symbol= 0;  /* current symbol when stepping through length[] */
    Cpa32S len = 0;    /* current length when stepping through h->count[] */
    Cpa32S left= 0;    /* number of possible codes left of current length */
    Cpa16U offs[MAXBITS+1];   /* offsets in symbol table for each length */
    Cpa16S nextcode = 0;
    Cpa16S codelen = 0;
    Cpa16S start[MAXBITS+1];      /* offsets in symbol table for each length */
    Cpa16U currentLength = 0;

    /* Count number of codes of each length */
    osalMemSet (hTable->symbol, 0, size * sizeof(Cpa16S));
    osalMemSet (hTable->bit_width, 0, size * sizeof(Cpa8U));
    osalMemSet (hTable->count, 0, (BANK_E_MAXLCODES) * sizeof(Cpa16S));

    for (symbol=0; symbol<size; symbol++)
    {
        if (length[symbol] < BANK_E_MAXLCODES)
        {
            /* assumes lengths are within bounds */
            hTable->count[length[symbol]]++;
        }
    }
    if (hTable->count[0] == size)       /* no codes! */
    {
        return 0;                       /* complete, but decode() will fail */
    }

    /* Check for an over-subscribed or incomplete set of lengths */
    left = 1;                           /* one possible code of zero length */
    for (len=1; len<=MAXBITS; len++)
    {
        left <<= 1;                     /* one more bit, double codes left */
        left -= hTable->count[len];     /* deduct count from possible codes */
        if (left < 0)
        {
            return left;                /* over-subscribed--return negative */
        }
    }                                   /* left > 0 means incomplete */

    /* Generate offsets into symbol table for each length for sorting */
    offs[1] = 0;
    start[0] = -1;
    for (len=1; len<MAXBITS; len++)
    {
        offs[len + 1] = offs[len] + (2*hTable->count[len]);
        start[len] = -1;
    }

    nextcode = 0;
    codelen = 1;
    while (codelen < 16)
    {
        start[codelen] = nextcode;
        nextcode += hTable->count[codelen];
        nextcode = nextcode << 1;
        codelen++;
    }

    /* Put symbols in table sorted by length, by symbol order
     * within each length
     */
    for (symbol=0; symbol<size; symbol++)
    {
        currentLength = length[symbol];
        if (currentLength > 0 && currentLength < (MAXBITS+1))
        {
            hTable->bit_width[symbol]= currentLength;
            hTable->symbol[symbol] = start[currentLength]++;
            hTable->lit[symbol] = symbol;
            offs[currentLength]++;
        }
    }
    /* Return zero for complete set */
    return 0;
}

CpaStatus
decodeDynamicBlock (state_t *state, Cpa8U residueBit,
        Cpa8U *pSource, Cpa32U sourceSize,
        CpaBufferList *pCtxBuffer,
        Cpa32U bufferIndex, Cpa32U offsetInBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa16U i = 0;
    huffman_t *lencode = NULL;
    huffman_t *distcode = NULL;
    Cpa16U cl4cl[BANK_E_ORDER_SIZE];
    Cpa16U order[BANK_E_ORDER_SIZE] = {16,17,18, 0,
                                        8, 7, 9, 6,
                                       10, 5,11, 4,
                                       12, 3,13, 2,
                                       14, 1,15};
    Cpa32S rc = 0;
    Cpa32S ncode = 0;
    Cpa16U lengths[BANK_E_MAXCODES];
    Cpa32S symbol = 0;     /* decoded value */
    Cpa32S len = 0;        /* last length to repeat */
    Cpa32S nLit = 0;
    Cpa32S nDist = 0;

    /* Reset cl4cl[] and lengths[] arrays */
    osalMemSet(cl4cl, 0, BANK_E_ORDER_SIZE);
    osalMemSet(lengths, 0, BANK_E_MAXCODES);

    if (residueBit)
    {
        bits(state, residueBit, pSource, sourceSize);
    }

    bits(state, 3, pSource, sourceSize);
    nLit = bits(state, 5, pSource, sourceSize);
    if (nLit < 0)
    {
        status = CPA_STATUS_FAIL;
    }
    nLit += MIN_LENGTH_SYMBOL;

    if (CPA_STATUS_SUCCESS == status)
    {
        nDist = bits(state, 5, pSource, sourceSize);
        if (nDist < 0)
        {
            status = CPA_STATUS_FAIL;
        }
        nDist++;
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        ncode = bits(state, 4, pSource, sourceSize);
        if (ncode < 0)
        {
            status = CPA_STATUS_FAIL;
        }
        ncode += MIN_NUMBER_OF_CODES;

        if ((CPA_STATUS_FAIL == status) || (ncode > BANK_E_ORDER_SIZE))
        {
            if (ncode > BANK_E_ORDER_SIZE)
            {
                LAC_LOG_ERROR1("Error in flat tree construction: "
                               "Invalid number of codes %d", ncode);
            }
            status = CPA_STATUS_FAIL;
        }
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        for (i=0; (Cpa32S)i<ncode; i++)
        {
            cl4cl[order[i]] = bits(state, 3, pSource, sourceSize);
        }
        for ( ; i<BANK_E_ORDER_SIZE; i++)
        {
            cl4cl[order[i]] = 0;
        }
        /* Allocate memory for length code Huffman tables */
        status = osalCreateHuffmanTable((void *)&lencode);
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* Construct length code */
        rc = construct(lencode, cl4cl, BANK_E_ORDER_SIZE);
        if (0 != rc)
        {
            LAC_LOG_ERROR("Error in flat tree construction: "
                          "Could not construct Huffman tables.");
            status = CPA_STATUS_FAIL;
        }
    }

    i= 0;
    while ((i < nLit + nDist) && (CPA_STATUS_SUCCESS == status))
    {
        symbol = decode(state, lencode, pSource, sourceSize);
        if (symbol >= 0 && symbol < BANK_E_SYMBOL_16)
        {
            /* length in 0..15 */
            if ((i+1) > BANK_E_MAXCODES)
            {
                LAC_LOG_ERROR("Error in flat tree construction: "
                        "going beyond maximal code.");
                status = CPA_STATUS_FAIL;
            }
            else
            {
                lengths[i++] = symbol;
            }
        }
        else if (symbol >= BANK_E_SYMBOL_16)
        {                          /* repeat instruction */
            len = 0;               /* assume repeating zeros */
            switch (symbol)
            {
            case BANK_E_SYMBOL_16:
                /* repeat last length 3..6 times */
                if (i== 0)
                {
                    LAC_LOG_ERROR("Error in flat tree construction: "
                            "no last length");
                    status = CPA_STATUS_FAIL;
                }
                else
                {
                    len = lengths[i- 1];    /* last length */
                    symbol = 3 + bits(state, 2, pSource, sourceSize);
                }
                break;
            case BANK_E_SYMBOL_17:
                /* repeat zero 3..10 times */
                symbol = 3 + bits(state, 3, pSource, sourceSize);
                break;
            case BANK_E_SYMBOL_18:
                /* symbol == 18, repeat zero 11..138 times */
                symbol = 11 + bits(state, 7, pSource, sourceSize);
                break;
            }

            if ((i + symbol > nLit + nDist) &&
                (CPA_STATUS_SUCCESS == status))
            {
                LAC_LOG_ERROR("Error in flat tree construction: "
                        "too many lengths");
                status = CPA_STATUS_FAIL;
            }

            /* repeat last or zero symbol times */
            while ((symbol--) && (CPA_STATUS_SUCCESS == status))
            {
                /* Check if we don't over range the array size */
                if((i+1) < BANK_E_MAXCODES)
                {
                    lengths[i++] = len;
                }
                else
                {
                    LAC_LOG_ERROR("Error in flat tree construction: "
                            "invalid symbol value");
                    status = CPA_STATUS_FAIL;
                    break;
                }
            }
        }
        else if (DC_ERROR_INVALID_BIT == symbol)
        {
            status = CPA_STATUS_RETRY;
            break;
        }
        else
        {
            LAC_LOG_ERROR("Error in flat tree construction: "
                    "invalid input stream");
            status = CPA_STATUS_FAIL;
            break;
        }
    }

    /* check for end-of-block code. There should be one. */
    if (0 == lengths[LENGTH_INDEX_256] || CPA_STATUS_FAIL == status)
    {
        /* Destroy lencode huffman table */
        osalDestroyHuffmanTable((void *)&lencode);
        return CPA_STATUS_FAIL;
    }

    /* Construct length code */
    if (CPA_STATUS_SUCCESS == status)
    {
        rc = construct2(lencode, lengths, nLit);
        if (0 != rc)
        {
            status = CPA_STATUS_FAIL;
        }
    }

    /* Allocate memory for distance code Huffman tables */
    if (CPA_STATUS_SUCCESS == status)
    {
        status = osalCreateHuffmanTable((void *)&distcode);
    }

    /* Construct distance code */
    if (CPA_STATUS_SUCCESS == status)
    {
        rc = construct2(distcode, lengths + nLit, nDist);
        if (0 != rc)
        {
            status = CPA_STATUS_FAIL;
        }
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* Compute and overwrite bank E in the context buffer */
        status = osalMakeBankE((void *)pCtxBuffer,
                               (void *)lencode, (void *)distcode,
                               &nLit, &nDist,
                               bufferIndex, offsetInBuffer);
    }
    osalDestroyHuffmanTable((void *)&lencode);
    osalDestroyHuffmanTable((void *)&distcode);
    return status;
}
