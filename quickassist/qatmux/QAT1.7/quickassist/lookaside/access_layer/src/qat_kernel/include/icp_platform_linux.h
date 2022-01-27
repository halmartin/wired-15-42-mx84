/*****************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
 *
 *****************************************************************************/

/*****************************************************************************
 * @file icp_platform_linux.h
 *
 * @description
 *      This file contains kernel space specific macros
 *
 *****************************************************************************/
#ifndef ICP_PLATFORM_KERNEL_H
#define ICP_PLATFORM_KERNEL_H
/* ***********************************************************
 * USER SPACE MACROS
 ************************************************************/
#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/page.h>
#include "Osal.h"

#define VOLATILE volatile

#define ICP_MDELAY medlay
#define ICP_SSLEEP ssleep

/* string conversion*/
#define ICP_STRTOL kstrtol
#define ICP_STRTOUL kstrtoul
#define ICP_STRTOULL kstrtoull

#define ICP_ISDIGIT isdigit
#define ICP_ISSPACE isspace

/* locking  */
#define ICP_SPINLOCK OsalLock
#define ICP_SPINLOCK_INIT(ptr) osalLockInit(ptr, TYPE_IGNORE)
#define ICP_SPINLOCK_LOCK osalLock
#define ICP_SPINLOCK_UNLOCK osalUnlock
#define ICP_SPINLOCK_LOCK_BH osalLockBh
#define ICP_SPINLOCK_UNLOCK_BH osalUnlockBh
#define ICP_SPINLOCK_LOCK_IRQ osalLockIrqSave
#define ICP_SPINLOCK_UNLOCK_IRQ osalUnlockIrqRestore
#define ICP_SPINLOCK_UNINIT osalLockDestroy

#define ICP_MUTEX OsalMutex
#define ICP_MUTEX_INIT osalMutexInit
#define ICP_MUTEX_LOCK(ptr) osalMutexLock(ptr, OSAL_WAIT_FOREVER)
#define ICP_MUTEX_LOCK_INTERRUPTIBLE(ptr) osalMutexLock(ptr, OSAL_WAIT_FOREVER)
#define ICP_MUTEX_TRYLOCK osalMutexTryLock
#define ICP_MUTEX_UNLOCK osalMutexUnlock
#define ICP_MUTEX_UNINIT osalMutexDestroy

#ifndef STATIC
#define STATIC static
#endif

/* Macro for adding an element to the tail of a doubly linked list */
/* The currentptr tracks the tail, and the headptr tracks the head */
#define ICP_ADD_ELEMENT_TO_END_OF_LIST(elementtoadd, currentptr, headptr)      \
    do                                                                         \
    {                                                                          \
        if (NULL == currentptr)                                                \
        {                                                                      \
            currentptr = elementtoadd;                                         \
            elementtoadd->pNext = NULL;                                        \
            elementtoadd->pPrev = NULL;                                        \
            headptr = currentptr;                                              \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            elementtoadd->pPrev = currentptr;                                  \
            currentptr->pNext = elementtoadd;                                  \
            elementtoadd->pNext = NULL;                                        \
            currentptr = elementtoadd;                                         \
        }                                                                      \
    } while (0)

#define ICP_REMOVE_ELEMENT_FROM_LIST(elementtoremove, currentptr, headptr)     \
    do                                                                         \
    {                                                                          \
        /* If the previous pointer is not NULL */                              \
        if (NULL != elementtoremove->pPrev)                                    \
        {                                                                      \
            elementtoremove->pPrev->pNext = elementtoremove->pNext;            \
            if (elementtoremove->pNext)                                        \
            {                                                                  \
                elementtoremove->pNext->pPrev = elementtoremove->pPrev;        \
            }                                                                  \
            else                                                               \
            {                                                                  \
                /* Move the tail pointer backwards */                          \
                currentptr = elementtoremove->pPrev;                           \
            }                                                                  \
        }                                                                      \
        else if (NULL != elementtoremove->pNext)                               \
        {                                                                      \
            /* Remove the head pointer */                                      \
            elementtoremove->pNext->pPrev = NULL;                              \
            /* Hence move the head forward */                                  \
            headptr = elementtoremove->pNext;                                  \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            /* Remove the final entry in the list */                           \
            currentptr = NULL;                                                 \
            headptr = NULL;                                                    \
        }                                                                      \
    } while (0)

#ifdef ICP_PARAM_CHECK
#define ICP_CHECK_FOR_NULL_PARAM(param)                                        \
    do                                                                         \
    {                                                                          \
        if (NULL == param)                                                     \
        {                                                                      \
            ADF_ERROR("%s(): invalid param: %s\n", __FUNCTION__, #param);      \
            return CPA_STATUS_INVALID_PARAM;                                   \
        }                                                                      \
    } while (0)

#define ICP_CHECK_FOR_NULL_PARAM_VOID(param)                                   \
    do                                                                         \
    {                                                                          \
        if (NULL == param)                                                     \
        {                                                                      \
            ADF_ERROR("%s(): invalid param: %s\n", __FUNCTION__, #param);      \
            return;                                                            \
        }                                                                      \
    } while (0)
#else
#define ICP_CHECK_FOR_NULL_PARAM(param)
#define ICP_CHECK_FOR_NULL_PARAM_VOID(param)
#endif

extern char *icp_module_name;
#define xprintk(level, level_str, fmt, args...)                                \
    osalStdLog(level "%s %s: %s: " fmt,                                        \
               icp_module_name,                                                \
               level_str,                                                      \
               (__func__),                                                     \
               ##args)

#define ADF_ERROR(format, args...) xprintk(KERN_ERR, "err", format, ##args)
extern struct module *qat_api_module;
#endif /* ICP_PLATFORM_KERNEL_H */
