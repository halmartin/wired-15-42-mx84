/**
 **************************************************************************
 * @file halAe_platform.h
 *
 * @description
 *      This is the common header file for Ucode AE Library (OS-dependent code)
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

#ifndef __HALAE_PLATFORM_H__
#define __HALAE_PLATFORM_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/unistd.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/mman.h>
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <asm/processor.h>  
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/ioctl.h>
#include <asm/irq.h>
#include <linux/dma-mapping.h>
#include "Osal.h"
#include "icp_firml_handle.h"

#define DECL(var, val) var = val 
#define EXTERN
#define DECLSPEC
#define XFMT64 "llx"

typedef unsigned char   uchar;

#define PRINTF printk

#if (defined(_DBG_PRINT))
#ifndef ERRINFO
#define ERRINFO(arg)  do{ PRINTF("\n***Error: Function %s in %s at line %d\n", __FUNCTION__, __FILE__, __LINE__); PRINTF arg;}while(0)
#define DBGINFO(arg) 	do{ PRINTF ("\n***Info: Function %s in %s at line %d\n", __FUNCTION__, __FILE__, __LINE__); PRINTF arg;} while(0)
#endif
#else	/*#if (defined(_DBG_PRINT))*/
#ifndef ERRINFO
#define ERRINFO(arg)
#define DBGINFO(arg)
#endif
#endif 	/*#if (defined(_DBG_PRINT))*/

#define SPINLOCK_T             OsalLock
#define SPIN_LOCK_FINI(self)   osalLockDestroy(&(self)) 
#define SPIN_LOCK(self)        osalLock(&(self))
#define SPIN_UNLOCK(self)      osalUnlock(&(self))

#define SPIN_LOCK_FINI_STRUCT(self)   osalLockDestroy(&((self).lock)) 
#define SPIN_LOCK_STRUCT(self)        osalLock(&((self).lock))
#define SPIN_UNLOCK_STRUCT(self)      osalUnlock(&((self).lock))   

#define SPIN_LOCK_INIT(self)   osalLockInit(&(self), 0)

/** @{ */
/* below macros can lock a sturcutre containing lock field */
#define SPIN_LOCK_INIT_STRUCT(self)   osalLockInit(&((self).lock), 0)
#define SPIN_LOCK_IRQSAVE(self, level)           osalLockIrqSave(&(self), &(level))
#define SPIN_UNLOCK_IRQRESTORE(self, level)      osalUnlockIrqRestore(&(self), &(level))  
/** @} */

#define SEM_POST                osalSemaphorePostWakeup
#define SEM_WAIT                osalSemaphoreWaitInterruptible

int halAe_getPciCsr(icp_firml_handle_t *handle, unsigned int offset, unsigned int numBytes, unsigned int *data);
int halAe_putPciCsr(icp_firml_handle_t *handle, unsigned int offset, unsigned int numBytes, unsigned int data);
#define GET_PCI_CSR halAe_getPciCsr
#define PUT_PCI_CSR halAe_putPciCsr

#endif /* __HALAE_PLATFORM_H__ */
