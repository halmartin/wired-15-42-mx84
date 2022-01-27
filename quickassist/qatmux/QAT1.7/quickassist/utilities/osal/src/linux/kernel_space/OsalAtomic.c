/**
 * @file kernel_space/OsalAtomic.c (linux)
 *
 * @brief OS-specific Atomic API's implementation.
 *
 *
 * @par
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
 */

#include <linux/version.h>
#include "Osal.h"

#define OSAL_SHIFT_32 32

#if defined(CONFIG_64BIT)

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicGet(OsalAtomic *pAtomicVar)
{
    return (( INT64 )atomic64_read((OsalAtomic*) pAtomicVar ));
}

OSAL_PUBLIC OSAL_INLINE void
osalAtomicSet(INT64 inValue, OsalAtomic *pAtomicVar)
{
    atomic64_set((OsalAtomic*)pAtomicVar,inValue);
}

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicAdd(INT64 inValue, OsalAtomic *pAtomicVar)
{
    return atomic64_add_return((long)inValue, (OsalAtomic*)pAtomicVar);
}

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicSub(INT64 inValue, OsalAtomic *pAtomicVar)
{
    return atomic64_sub_return((long)inValue, (OsalAtomic*)pAtomicVar);
}

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicInc(OsalAtomic *pAtomicVar)
{
    return atomic64_inc_return((OsalAtomic*)pAtomicVar);
}

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicDec(OsalAtomic *pAtomicVar)
{
    return atomic64_dec_return((OsalAtomic*)pAtomicVar);
}

OSAL_PUBLIC OSAL_INLINE OSAL_STATUS
osalAtomicDecAndTest(OsalAtomic *pAtomicVar)
{
    return (OSAL_STATUS)(atomic64_dec_and_test((OsalAtomic*)pAtomicVar));
}

#else

asmlinkage INT64 osal_atomic_add(INT64 v, OsalAtomic *a);
asmlinkage INT64 osal_atomic_sub(INT64 v, OsalAtomic *a);
asmlinkage INT64 osal_atomic_inc(OsalAtomic *a);
asmlinkage INT64 osal_atomic_dec(OsalAtomic *a);

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicGet(OsalAtomic *pAtomicVar)
{
    int64_t val;
    asm volatile("xor %%eax, %%eax\n"  \
                 "xor %%edx, %%edx\n"  \
                 "xor %%ecx, %%ecx\n"  \
                 "xor %%ebx, %%ebx\n"  \
                 "lock cmpxchg8b (%%edi)\n"
                 : "=A" (val)
                 : "D" (pAtomicVar)
                 : "ecx", "ebx", "cc"
                 );
    return val;
}

OSAL_PUBLIC OSAL_INLINE void
osalAtomicSet(INT64 value, OsalAtomic *pAtomicVar)
{
    int32_t var_hi = (int32_t)(value >> OSAL_SHIFT_32);
    int32_t var_lo = (int32_t)value;
    asm volatile("movl (%%edi), %%eax\n"    \
                 "movl 4(%%edi), %%edx\n"   \
                 "lab1: \n"                 \
                 "lock cmpxchg8b (%%edi)\n" \
                 "jnz lab1\n"
                 :
                 : "D"(pAtomicVar), "b"(var_lo), "c"(var_hi)
                 : "eax", "edx", "cc"
                 );
}

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicAdd(INT64 value, OsalAtomic *pAtomicVar)
{
    return osal_atomic_add(value, pAtomicVar);
}

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicSub(INT64 value, OsalAtomic *pAtomicVar)
{
    return osal_atomic_sub(value, pAtomicVar);
}

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicInc(OsalAtomic *pAtomicVar)
{
    return osal_atomic_inc(pAtomicVar);
}

OSAL_PUBLIC OSAL_INLINE INT64
osalAtomicDec(OsalAtomic *pAtomicVar)
{
    return osal_atomic_dec(pAtomicVar);
}

OSAL_PUBLIC OSAL_INLINE OSAL_STATUS
osalAtomicDecAndTest(OsalAtomic *pAtomicVar)
{
   return (OSAL_STATUS)(osal_atomic_dec(pAtomicVar) == 0 ) ? 1 : 0 ;
}

#endif

