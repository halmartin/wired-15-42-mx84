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

/**
 *****************************************************************************
 * @file dc_flattree.h
 *
 * @ingroup Dc_DataCompression
 *
 * @description
 *      Definition of the Data Compression flat tree parameters.
 *
 *****************************************************************************/
#ifndef DC_FLATTREE_H
#define DC_FLATTREE_H

#include "cpa_dc_dp.h"


/* Bank E error codes */
#define DC_ERROR_OUT_OF_INPUT       (-1)
#define DC_ERROR_INVALID_BIT        (-9)
#define DC_ERROR_OUT_OF_CODES       (-10)
/* Bank-E */
#define BANK_E_SYMBOL_16            (16)
#define BANK_E_SYMBOL_17            (17)
#define BANK_E_SYMBOL_18            (18)

/* maximum number of literal/length codes */
#define BANK_E_MAXLCODES            (286)
/* maximum number of distance codes */
#define BANK_E_MAXDCODES            (30)
#define BANK_E_ORDER_SIZE           (19)
 /* maximum codes lengths to read */
#define BANK_E_MAXCODES             (BANK_E_MAXLCODES+BANK_E_MAXDCODES)

#define LENGTH_INDEX_256            (256)

#define CHKBITS(incnt, inlen)       \
        (incnt == inlen) ? (CPA_STATUS_FAIL) : (CPA_STATUS_SUCCESS)

/*
 * Maximums for allocations and loops.  It is not useful to change these --
 * they are fixed by the deflate format.
 */
/* maximum bits in a code */
#define MAXBITS                     (15)
/* number of fixed literal/length codes */
#define FIXLCODES                   (288)

/* Literals, lengths, and the end-of-block code are combined into a single
 * code of up to 286 symbols.  They are 256 literals (0..255), 29 length
 * symbols (257..285), and the end-of-block symbol (256).
 */
#define MIN_LENGTH_SYMBOL           (257)
/* Minimum number of codes */
#define MIN_NUMBER_OF_CODES         (4)

/**
 * Huffman code decoding tables.  count[1..MAXBITS] is the number of symbols
 * of each length, which for a canonical code are stepped through in order.
 * symbol[] are the symbol values in canonical order, where the number of
 * entries is the sum of the counts in count[]. The decoding process can be
 * seen in the decode() function called by decodeDynamicBlock().
 */
typedef struct huffman_s
{
    Cpa16S count[BANK_E_MAXLCODES];   /* number of symbols of each length */
    Cpa16S symbol[BANK_E_MAXLCODES];  /* canonically ordered symbols */
    Cpa16S lit[BANK_E_MAXLCODES];     /* canonically ordered literals */
    Cpa8U bit_width[BANK_E_MAXLCODES];/* bit width for symbol */
}huffman_t;

/* Input and output state */
typedef struct state_s
{
    /* output state */
    Cpa8U *out;         /* output buffer */
    Cpa64U outlen;      /* available space at out */
    Cpa64U outcnt;      /* bytes written to out so far */

    /* input state */
    Cpa8U *in;          /* input buffer */
    Cpa64U inlen;       /* available input at in */
    Cpa64U incnt;       /* bytes read so far */
    Cpa32S bitbuf;      /* bit buffer */
    Cpa32S bitcnt;      /* number of bits in bit buffer */
    Cpa64U bits_total;  /* number of bits consummed */
}state_t;

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Process a dynamic codes block.
 *      Intel wrote this function based on the dynamic() function from the
 *      file "puff.c" provided by Mark Adler. This current function is
 *      a modified version from the dynamic() function in the file puff.c
 *
 * @description
 *
 * Format notes:
 *
 * - A dynamic block starts with a description of the literal/length and
 *   distance codes for that block.  New dynamic blocks allow the compressor to
 *   rapidly adapt to changing data with new codes optimized for that data.
 *
 * - The codes used by the deflate format are "canonical", which means that
 *   the actual bits of the codes are generated in an unambiguous way simply
 *   from the number of bits in each code.  Therefore the code descriptions
 *   are simply a list of code lengths for each symbol.
 *
 * - The code lengths are stored in order for the symbols, so lengths are
 *   provided for each of the literal/length symbols, and for each of the
 *   distance symbols.
 *
 * - If a symbol is not used in the block, this is represented by a zero as
 *   as the code length.  This does not mean a zero-length code, but rather
 *   that no code should be created for this symbol.  There is no way in the
 *   deflate format to represent a zero-length code.
 *
 * - The maximum number of bits in a code is 15, so the possible lengths for
 *   any code are 1..15.
 *
 * - The fact that a length of zero is not permitted for a code has an
 *   interesting consequence.  Normally if only one symbol is used for a given
 *   code, then in fact that code could be represented with zero bits.  However
 *   in deflate, that code has to be at least one bit.  So for example, if
 *   only a single distance base symbol appears in a block, then it will be
 *   represented by a single code of length one, in particular one 0 bit.  This
 *   is an incomplete code, since if a 1 bit is received, it has no meaning,
 *   and should result in an error.  So incomplete distance codes of one symbol
 *   should be permitted, and the receipt of invalid codes should be handled.
 *
 * - It is also possible to have a single literal/length code, but that code
 *   must be the end-of-block code, since every dynamic block has one.  This
 *   is not the most efficient way to create an empty block (an empty fixed
 *   block is fewer bits), but it is allowed by the format.  So incomplete
 *   literal/length codes of one symbol should also be permitted.
 *
 * - If there are only literal codes and no lengths, then there are no distance
 *   codes.  This is represented by one distance code with zero bits.
 *
 * - The list of up to 286 length/literal lengths and up to 30 distance lengths
 *   are themselves compressed using Huffman codes and run-length encoding.  In
 *   the list of code lengths, a 0 symbol means no code, a 1..15 symbol means
 *   that length, and the symbols 16, 17, and 18 are run-length instructions.
 *   Each of 16, 17, and 18 are follwed by extra bits to define the length of
 *   the run.  16 copies the last length 3 to 6 times.  17 represents 3 to 10
 *   zero lengths, and 18 represents 11 to 138 zero lengths.  Unused symbols
 *   are common, hence the special coding for zero lengths.
 *
 * - The symbols for 0..18 are Huffman coded, and so that code must be
 *   described first.  This is simply a sequence of up to 19 three-bit values
 *   representing no code (0) or the code length for that symbol (1..7).
 *
 * - A dynamic block starts with three fixed-size counts from which is computed
 *   the number of literal/length code lengths, the number of distance code
 *   lengths, and the number of code length code lengths (ok, you come up with
 *   a better name!) in the code descriptions.  For the literal/length and
 *   distance codes, lengths after those provided are considered zero, i.e. no
 *   code.  The code length code lengths are received in a permuted order (see
 *   the order[] array below) to make a short code length code length list more
 *   likely.  As it turns out, very short and very long codes are less likely
 *   to be seen in a dynamic code description, hence what may appear initially
 *   to be a peculiar ordering.
 *
 * - Given the number of literal/length code lengths (nlen) and distance code
 *   lengths (ndist), then they are treated as one long list of nlen + ndist
 *   code lengths.  Therefore run-length coding can and often does cross the
 *   boundary between the two sets of lengths.
 *
 * - So to summarize, the code description at the start of a dynamic block is
 *   three counts for the number of code lengths for the literal/length codes,
 *   the distance codes, and the code length codes.  This is followed by the
 *   code length code lengths, three bits each.  This is used to construct the
 *   code length code which is used to read the remainder of the lengths.  Then
 *   the literal/length code lengths and distance lengths are read as a single
 *   set of lengths using the code length codes.  Codes are constructed from
 *   the resulting two sets of lengths, and then finally you can start
 *   decoding actual compressed data in the block.
 *
 * - For reference, a "typical" size for the code description in a dynamic
 *   block is around 80 bytes.
 *
 *
 * @param[in]       state            Input and output state
 * @param[in]       residueBits      Number of residue bits to skip in the
 *                                   stream
 * @param[in]       pSource          Address of data source buffer
 * @param[in]       sourceSize       Size of data source buffer
 * @param[in/out]   pCtxBuffer       Address of context source buffer
 * @param[in]       bufferIndex      Buffer index in SGL where Bank E starts
 * @param[in]       offsetInBuffer   Offset in buffer where Bank E starts
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully
 * @retval CPA_STATUS_FAIL           Function failed
 *****************************************************************************/
CpaStatus
decodeDynamicBlock (state_t *state, Cpa8U residueBit,
		Cpa8U *pSource, Cpa32U sourceSize,
		CpaBufferList *pCtxBuffer,
		Cpa32U bufferIndex, Cpa32U offsetInBuffer);

#endif /* DC_FLATTREE_H */
