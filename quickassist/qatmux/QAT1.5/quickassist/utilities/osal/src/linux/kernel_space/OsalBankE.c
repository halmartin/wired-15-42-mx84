/**
 * @file OsalBankE.c (linux kernel space)
 *
 * @brief Implementation for constructing Bank E in context buffer.
 *
 *
 * @par
 * GPL LICENSE SUMMARY
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
    bankE = kmalloc(BANK_E_SIZE * sizeof(UINT8), GFP_ATOMIC);
    if(!bankE)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    lit_map = kmalloc(BANK_E_LIT_MAP_SIZE * sizeof(INT32), GFP_ATOMIC);
    if(!lit_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    lit_cl_map = kmalloc(BANK_E_LIT_MAP_SIZE * sizeof(INT32), GFP_ATOMIC);
    if(!lit_cl_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    len_map = kmalloc(BANK_E_MAXDCODES * sizeof(INT32), GFP_ATOMIC);
    if(!len_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    len_cl_map = kmalloc(BANK_E_MAXDCODES * sizeof(INT32), GFP_ATOMIC);
    if(!len_cl_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    dst_map = kmalloc(BANK_E_MAXDCODES * sizeof(INT32), GFP_ATOMIC);
    if(!dst_map)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalmakeBankE(): Error while allocating bankE\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        error = OSAL_FAIL;
        goto cleanup;
    }

    dst_cl_map = kmalloc(BANK_E_MAXDCODES * sizeof(INT32), GFP_ATOMIC);
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
        kfree(dst_cl_map);
        dst_cl_map = NULL;
    }

    if (NULL != dst_map)
    {
        kfree(dst_map);
        dst_map = NULL;
    }

    if (NULL != len_cl_map)
    {
        kfree(len_cl_map);
        len_cl_map = NULL;
    }

    if (NULL != len_map)
    {
        kfree(len_map);
        len_map = NULL;
    }

    if (NULL != lit_cl_map)
    {
        kfree(lit_cl_map);
        lit_cl_map = NULL;
    }

    if (NULL != lit_map)
    {
        kfree(lit_map);
        lit_map = NULL;
    }

    if (NULL != bankE)
    {
        kfree(bankE);
        bankE = NULL;
    }
    return error;
}

OSAL_PUBLIC OSAL_STATUS
osalCreateHuffmanTable (void **huffman_table)
{
    huffman_table_t *huff = kmalloc (sizeof (huffman_table_t), GFP_ATOMIC);

    if (NULL == huff)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                "osalmakeBankE(): Error while allocating bank E "
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
        kfree(huff);
        huff = NULL;
    }
    return OSAL_SUCCESS;
}
