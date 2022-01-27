/**
 **************************************************************************
 * @file uclo_platform.h
 *
 * @description
 *      This is common header file for Ucode Object File Loader
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

#ifndef __UCLO_PLATFORM_H__
#define __UCLO_PLATFORM_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/poll.h>
#include <linux/mman.h>
#include <linux/types.h>
#include <linux/version.h>

#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/div64.h>

#include <linux/sched.h>
#include <linux/unistd.h>

#define XFMT64 "llx"
#define PRINTF printk

#if (defined(_DBG_PRINT))

#define ERRINFO(arg)  do{ printk("\n***Error: Function %s in %s at line %d\n", __FUNCTION__, __FILE__, __LINE__); printk arg;}while(0)
#define DBGINFO(arg) 	do{ printk ("\n***Info: Function %s in %s at line %d\n", __FUNCTION__, __FILE__, __LINE__); printk arg;} while(0)

#else	/*#if (defined(_DBG_PRINT))*/

#define ERRINFO(arg)
#define DBGINFO(arg)

#endif 	/*#if (defined(_DBG_PRINT))*/
#define MUTEX_LOCK(mutex)       { if((mutex) != NULL) osalMutexLock(&(mutex), OSAL_WAIT_FOREVER); }
#define MUTEX_UNLOCK(mutex)     { if((mutex) != NULL) osalMutexUnlock(&(mutex)); }
#define MUTEX_DESTROY(mutex)    { if((mutex) != NULL) osalMutexDestroy(&(mutex)); (mutex) = NULL; }

/* simple_strtoul/simple_strtoull/strict_strtoull will convert "-1" to 0 */
/* simple_strtol will convert "-1" to 0xffffffff */
/* strict_strtoll will convert "-1" to 0xffffffffffffffff */
/* 32-bit/64-bit 2.6.29 kernel does not export simple_strtoll */
#define STR_TO_32(str, base, num, endPtr) {*(num) = (int)simple_strtol((str), &(endPtr), (base));}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
#define STR_TO_64(str, base, num, endPtr) {endPtr=NULL; if (kstrtoull((str), (base), (num))) printk("Error strtoull convert %s\n", str); }
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
#define STR_TO_64(str, base, num, endPtr) {endPtr=NULL; if (strict_strtoull((str), (base), (num))) printk("Error strtoull convert %s\n", str); }
#else 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
#define STR_TO_64(str, base, num, endPtr) {endPtr=NULL; strict_strtoll((str), (base), (num));}
#else
#define STR_TO_64(str, base, num, endPtr)                                 \
       do {                                                               \
             if (str[0] == '-')                                           \
             {                                                            \
                  *(num) = -(simple_strtoull((str+1), &(endPtr), (base))); \
             }else {                                                      \
                  *(num) = simple_strtoull((str), &(endPtr), (base));      \
             }                                                            \
       } while(0)
#endif
#endif
#endif

int aToi(const char *pStr);

#ifdef __x86_64__

#define div_u64(a, b, c) { (*c) = (a) / (b); } 
#define div_u64_rem(a, b, c) { (*c) = (a) % (b); }

#else

#define div_u64(a, b, c)                      \
do{                                           \
    do_div((a), (unsigned int)(b & 0xffffffff));                         \
    (*c) = (a);                               \
} while(0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
static inline u64 div_u64_rem2(u64 d1, u32 d2, u32 *r)
{
     u32 rh;
     union {
        u64    ll;
        struct {
        u32 low, high;
        } l;
     } res;

     res.ll = d1;
     rh = res.l.high;

      res.l.high = 0;
      if (d2 < rh ) {
            res.l.high = rh / d2;
            rh %= d2;
      }
      asm volatile
          ("divl %2"
            : "=a" (res.l.low), "=d" (*r)
            : "rm" (d2), "0" (res.l.low), "1" (rh));
    return res.ll;
}
#define div_u64_rem     div_u64_rem2
#endif

#endif

#define TOLOWER(c) tolower(c)

#endif /* __UCLO_PLATFORM_H__ */
