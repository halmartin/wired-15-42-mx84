/**
 * @file OsalBankE.c (user space)
 *
 * @brief Implementation for constructing Bank E in context buffer.
 *
 *
 * @par
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
 *  version: QAT1.5.L.1.11.0-36
 */

#include "Osal.h"
#include "OsalOsTypes.h"
#include "cpa.h"

OSAL_PRIVATE UINT16
bit_rev(UINT16 in, INT32 bits)
{
    UINT16 out = 0;
    UINT16 temp = 0;
    UINT16 mask[BANK_E_REGISTER_SIZE];
    INT32 i = 0;

    for (i=0; i<BANK_E_REGISTER_SIZE; i++)
    {
        mask[i] = 1<<i;
    }
    for (i=0; i<bits; i++)
    {
        temp =  in & mask[i];
        temp = temp>>i;
        temp = temp <<(bits-i-1);
        out = out | temp;
    }
    return out;
}

OSAL_PUBLIC OSAL_STATUS
osalMakeBankE(void *ctxBufferList,
              void *literal, void *distances,
              INT32 *nlit, INT32 *ndist,
              UINT32 bankEStartBuffIdx,
              UINT32 bankEStartBuffOffs)
{
    UINT8 *bankE = NULL;
    INT32 *lit_map = NULL;
    INT32 *lit_cl_map = NULL;
    INT32 *len_map = NULL;
    INT32 *len_cl_map = NULL;
    INT32 *dst_map = NULL;
    INT32 *dst_cl_map = NULL;
    UINT32 buffIdx = 0;
    UINT32 startBuffIdx = 0;
    UINT32 dataIdx = 0;
    UINT32 i = 0;
    UINT32 j = 0;
    UINT16 sym, valid[BANK_E_REGISTER_SIZE];
    UINT16 *us_ptr = NULL;

    CpaBufferList* const pContextBuffer = (CpaBufferList *)ctxBufferList;
    huffman_table_t* const ll = (huffman_table_t *)literal;
    huffman_table_t* const dist = (huffman_table_t *)distances;

    OSAL_STATUS error = OSAL_SUCCESS;

    /* Allocate memory for temporary buffer storing bankE. */
    bankE = (UINT8*) malloc(BANK_E_SIZE * sizeof(UINT8));
    if(!bankE)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    lit_map = (INT32*) malloc(BANK_E_LIT_MAP_SIZE * sizeof(INT32));
    if(!lit_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    lit_cl_map = (INT32*) malloc(BANK_E_LIT_MAP_SIZE * sizeof(INT32));
    if(!lit_cl_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    len_map = (INT32*) malloc(BANK_E_MAXDCODES * sizeof(INT32));
    if(!len_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    len_cl_map = (INT32*) malloc(BANK_E_MAXDCODES * sizeof(INT32));
    if(!len_cl_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    dst_map = (INT32*) malloc(BANK_E_MAXDCODES * sizeof(INT32));
    if(!dst_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    dst_cl_map = (INT32*) malloc(BANK_E_MAXDCODES * sizeof(INT32));
    if(!dst_cl_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    for (i=0; i<CODE_128; i++)
    {
        lit_map[i] = i<<3;
        lit_map[i+CODE_128] = (i<<3) + 2;
        lit_cl_map[i] = (i>>1) + BANK_E_2K_SIZE;
        lit_cl_map[i+CODE_128] = ((i+CODE_128)>>1) + BANK_E_2K_SIZE;
    }

    for (i = 0; i < BANK_E_MAXDCODES; i++)
    {
        len_map[i] = BANK_E_1K_SIZE + (i<<3);
        len_cl_map[i] = (i>>1) + 2176;
        dst_map[i] = 1026 + (i<<3);
        dst_cl_map[i] = (i>>1) + BANK_E_3K_SIZE;
    }

    valid[0]=0xffff;
    for (i = 1; i <= 15; i++)
    {
        valid[i] = valid[i-1]<<1;
    }

    osalMemSet (bankE, 0, BANK_E_SIZE);

    for (i = 0; i < *nlit; i++)
    {
        if (ll->bit_width[i] > 0 )
        {
            if (ll->lit[i] < BANK_E_LIT_MAP_SIZE)
            {
                us_ptr = (UINT16 *)&(bankE[lit_map[ll->lit[i]]]);
                *us_ptr = bit_rev(ll->symbol[i], ll->bit_width[i]);
                us_ptr++;
                us_ptr++;
                *us_ptr = valid[ll->bit_width[i]];
                if (( ll->lit[i] ) % 2 == 0 )
                {
                    bankE[lit_cl_map[ll->lit[i]]] |= ll->bit_width[i];
                }
                else
                {
                    bankE[lit_cl_map[ll->lit[i]]] |= (ll->bit_width[i]<<4);
                }
            }
            else if ( ll->lit[i] == BANK_E_LIT_MAP_SIZE)
            {
                us_ptr = (UINT16 *)&(bankE[len_map[0]]);
                *us_ptr = bit_rev(ll->symbol[i], ll->bit_width[i] );
                us_ptr++;
                us_ptr++;
                *us_ptr = valid[ll->bit_width[i]];

                if (( ll->lit[i] ) % 2 == 0 )
                {
                    bankE[len_cl_map[0]] |= ll->bit_width[i];
                }
                else
                {
                    bankE[len_cl_map[0]] |= (ll->bit_width[i]<<4);
                }
            }
            else
            {
                sym = ll->lit[i]-BANK_E_LIT_MAP_SIZE;
                us_ptr = (UINT16 *)&(bankE[len_map[sym]]);
                *us_ptr = bit_rev(ll->symbol[i], ll->bit_width[i]);
                us_ptr++;
                us_ptr++;
                *us_ptr = valid[ll->bit_width[i]];

                if (sym < BANK_E_MAXDCODES)
                {
                    if ((sym  % 2) == 0)
                    {
                        bankE[len_cl_map[sym]] |= ll->bit_width[i];
                    }
                    else
                    {
                        bankE[len_cl_map[sym]] |= (ll->bit_width[i]<<4);
                    }
                }
            }
        }

    }

    for ( i = 0; i < *ndist; i++ )
    {
        if (dist->bit_width[i] > 0 )
        {
            us_ptr = (UINT16 *)&(bankE[dst_map[dist->lit[i]]]);
            *us_ptr = bit_rev(dist->symbol[i], dist->bit_width[i]);
            us_ptr++;
            us_ptr++;
            *us_ptr = valid[dist->bit_width[i]];

            if ((dist->lit[i]) % 2 == 0 )
            {
                bankE[dst_cl_map[dist->lit[i]]] |= dist->bit_width[i];
            }
            else
            {
                bankE[dst_cl_map[dist->lit[i]]] |= (dist->bit_width[i]<<4);
            }
        }
    }

    /* Update bank E inside context buffer */
    startBuffIdx = bankEStartBuffIdx;
    dataIdx = bankEStartBuffOffs;
    buffIdx = startBuffIdx;
    for (i=0, j=0; i<BANK_E_SIZE; i++, j++)
    {
        /* Check if we are at the last byte of the current buffer */
        if ((dataIdx + j) >= pContextBuffer->pBuffers[buffIdx].dataLenInBytes)
        {
            buffIdx++;
            dataIdx = 0;
            j = 0;
        }
        pContextBuffer->pBuffers[buffIdx].pData[dataIdx+j] = bankE[i];
    }

cleanup:
    /* Exit and cleanup */
    if (NULL != dst_cl_map)
    {
        free(dst_cl_map);
        dst_cl_map = NULL;
    }

    if (NULL != dst_map)
    {
        free(dst_map);
        dst_map = NULL;
    }

    if (NULL != len_cl_map)
    {
        free(len_cl_map);
        len_cl_map = NULL;
    }

    if (NULL != len_map)
    {
        free(len_map);
        len_map = NULL;
    }

    if (NULL != lit_cl_map)
    {
        free(lit_cl_map);
        lit_cl_map = NULL;
    }

    if (NULL != lit_map)
    {
        free(lit_map);
        lit_map = NULL;
    }

    if (NULL != bankE)
    {
        free(bankE);
        bankE = NULL;
    }
    return error;
}

OSAL_PUBLIC OSAL_STATUS
osalCreateHuffmanTable (void **huffman_table)
{
    huffman_table_t *huff=(huffman_table_t*)malloc(sizeof (huffman_table_t));

    if (NULL == huff)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                "osalmakeBankE(): Error while allocating bankE "
                "Huffman tables\n",
                0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
    *huffman_table = (void*)huff;
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalDestroyHuffmanTable (void **huffman_table)
{
    /* Cast the pointer parameter huffman_table to a huffman_t pointer */
    huffman_table_t* huff = (huffman_table_t *) (*huffman_table);

    if (NULL != huff)
    {
        free(huff);
        huff = NULL;
    }
    return OSAL_SUCCESS;
}
