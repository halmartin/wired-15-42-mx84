/***************************************************************************
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

/**
 ***************************************************************************
 * @file cpa_nrbg_utils.h
 *
 * @defgroup sampleNrbgFunctional
 *
 * @ingroup sampleCode
 *
 * @description
 * Defines macros for printing and debugging and inline functions for memory
 * allocating and freeing
 *
 ***************************************************************************/
#ifndef CPA_NRBG_UTILS_H
#define CPA_NRBG_UTILS_H


#include "cpa.h"

#ifdef USER_SPACE
/* User space Utils */
#include <stdio.h>
#include <stdlib.h>

#define UTIL_PRINT            printf
#define UTIL_FLUSH            fflush(stdout)
#define UTIL_ALLOC(size)      malloc(size)
#define UTIL_FREE             free
#else
/* Kernel space Utils */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#define UTIL_PRINT            printk
#define UTIL_FLUSH
#define UTIL_ALLOC(size)      kmalloc(size, GFP_ATOMIC)
#define UTIL_FREE             kfree
#endif

/**< Prints the name of the function and the arguments only if debugParam is
 * TRUE.
 */
#define PRINT_DBG(args...)              \
do {                                    \
    if (TRUE == debugParam) {           \
        UTIL_PRINT("%s(): ", __func__); \
        UTIL_PRINT(args);               \
        UTIL_FLUSH;                     \
    }                                   \
} while (0)

/**< Prints the name of the function and the arguments */
#define PRINT_ERR(args...)          \
do {                                \
    UTIL_PRINT("%s(): ", __func__); \
    UTIL_PRINT(args);               \
} while (0)

/**
 *******************************************************************************
 * @ingroup sampleNrbgFunctional
 *
 *      This function and associated macro allocates the memory for the given
 *      size and stores the address of the memory allocated in the pointer.
 *
 * @param[out] ppMemAddr    address of pointer where address will be stored
 * @param[in] sizeBytes     the size of the memory to be allocated
 *
 * @retval CPA_STATUS_RESOURCE  Macro failed to allocate Memory
 * @retval CPA_STATUS_SUCCESS   Macro executed successfully
 *
 ******************************************************************************/
static __inline CpaStatus
Mem_OsMemAlloc(void **ppMemAddr,
                  Cpa32U sizeBytes)
{
    *ppMemAddr = UTIL_ALLOC(sizeBytes);
    if (NULL == *ppMemAddr)
    {
        return CPA_STATUS_RESOURCE;
    }

    return CPA_STATUS_SUCCESS;
}

/**
 *******************************************************************************
 * @ingroup sampleNrbgFunctional
 *      Macro for the Mem_OsMemAlloc function
 *
 ******************************************************************************/
#define OS_MALLOC(ppMemAddr, sizeBytes) \
    Mem_OsMemAlloc((void *)(ppMemAddr), (sizeBytes))

/**
 *******************************************************************************
 * @ingroup sampleNrbgFunctional
 *      This function and associated macro frees the memory at the given address
 *      and resets the pointer to NULL
 *
 * @param[out] ppMemAddr    address of pointer where mem address is stored.
 *                          If pointer is NULL, the function will exit silently
 *
 * @retval void
 *
 ******************************************************************************/
static __inline void
Mem_OsMemFree(void **ppMemAddr)
{
    if (NULL != *ppMemAddr)
    {
        UTIL_FREE(*ppMemAddr);
        *ppMemAddr = NULL;
    }
}

/**
 *******************************************************************************
 * @ingroup sampleNrbgFunctional
 *      Macro for the Mem_OsMemFree function
 *
 ******************************************************************************/
#define OS_FREE(pMemAddr) \
    Mem_OsMemFree((void *)&pMemAddr)


#endif /* CPA_NRBG_UTILS_H */
