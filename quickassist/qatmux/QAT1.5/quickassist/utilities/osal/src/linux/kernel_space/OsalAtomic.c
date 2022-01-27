/**
 * @file kernel_space/OsalAtomic.c (linux)
 *
 * @brief OS-specific Atomic API's implementation.
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

