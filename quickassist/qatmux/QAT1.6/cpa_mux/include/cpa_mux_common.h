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
 *  version: QAT1.6.L.2.6.0-65
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file cpa_mux_common.h 
 *
 * @defgroup cpaMuxCommon CPA Mux Common
 *
 * Definition of the mux/demux layer for managing CPA API implementations
 *
 *****************************************************************************/


/*****************************************************************************/

#ifndef CPA_MUX_COMMON_H
#define CPA_MUX_COMMON_H

#include "cpa_impl_mux.h"
#include "Osal.h"

#define STATIC static
typedef enum
{
    CPA_MUX_INST_TYPE_CY = 0,
    CPA_MUX_INST_TYPE_DC
} CpaMuxInstanceType;

/**
 * Generic double-linked list node
 */
typedef struct list_s
{
    struct list_s *next, *prev;
} ListEntry;

/**
 * Descriptor for a single implementation registered with the Cpa Mux
 */
typedef struct
{
    ListEntry drbgSessions;
    void *dlHandle;
    CpaFuncPtrs implFunctPtrs;
    int registered;
} CpaMuxImpl;

/**
 * Used for type-casting a CpaInstance to retrieve a reserved field
 * which is used to store a reference to a registered Cpa Mux implementation
 */
typedef struct
{
    CpaMuxImpl *pMuxImpl;
} CpaMuxInstance;

#define MAX_NUM_IMPLS CPA_MUX_DRIVER_TYPE_INVALID

extern CpaMuxImpl cpaMuxImpls[];
extern volatile Cpa16U cpaMuxNumImpls;

static inline CpaMuxImpl *
firstImplHandle (void)
{
    Cpa16U i=0;
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        { 
            return &cpaMuxImpls[i];
        }
    }
    return NULL;
}

static inline CpaMuxImpl *cpaMuxGetImpl(CpaInstanceHandle instanceHandle)
{
    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
        return firstImplHandle();
    }
    else
    {
        return (((CpaMuxInstance *)instanceHandle)->pMuxImpl);
    }
}

/* Logging macros */
#define MUX_ERROR(log)                                                \
    (void)osalLogString (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDERR,     \
        "%s() - : " log "\n", (char*)__func__, 0, 0, 0, 0, 0)

#define MUX_WARN(log)                                                 \
    (void)osalLogString (OSAL_LOG_LVL_WARNING, OSAL_LOG_DEV_STDERR,   \
        "%s() - : " log "\n", (char*)__func__, 0, 0, 0, 0, 0)

#define MUX_WARN1(log,param)                                              \
    (void)osalLogString (OSAL_LOG_LVL_WARNING, OSAL_LOG_DEV_STDERR,       \
        "%s() - : " log "\n", (char*)__func__, (char*)param, 0, 0, 0, 0)

#define MUX_PRINT osalStdLog

#define CHECK_NULL_PARAM(x)                                             \
    do {                                                                \
        if (NULL == (x)) {                                              \
            MUX_ERROR("called with null handle");\
            return CPA_STATUS_INVALID_PARAM;                            \
        }                                                               \
    } while (0)

#define IMPL_HANDLE_CHECK(pImplHandle, fn, nullCheck) \
    if (0 && !strstr(#fn, "PollInstance"))\
    { \
        MUX_PRINT("CpaMux: %s(%p, ...) called\n", #fn, pImplHandle); \
    } \
    if (nullCheck) \
    { \
        CHECK_NULL_PARAM(pImplHandle); \
    } \
    if (0 == cpaMuxNumImpls) \
    { \
        MUX_ERROR("No implementations have been registered"); \
        return CPA_STATUS_INVALID_PARAM;     \
    }

#define CPA_MUX_CALL_IMPL_FN(fn, instanceHandle, ...)          \
    {                                                          \
        CpaMuxImpl *impl = cpaMuxGetImpl(instanceHandle);      \
        IMPL_HANDLE_CHECK (impl, fn, 1);                       \
        return impl->implFunctPtrs.fn (__VA_ARGS__);           \
    }
#endif
