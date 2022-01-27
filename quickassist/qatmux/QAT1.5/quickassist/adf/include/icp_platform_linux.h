/*****************************************************************************
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
 *****************************************************************************/

/*****************************************************************************
 * @file icp_platform_linux.h
 *
 * @description
 *      This file contains linux specific macros
 *
 *****************************************************************************/
#ifndef ICP_PLATFORM_LINUX_H
#define ICP_PLATFORM_LINUX_H

/************************************************************
 * LINUX KERNEL SPACE MACROS
 ************************************************************/
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#include "Osal.h"

#define STATIC static
#define VOLATILE volatile

#define ICP_MDELAY(msecs)          mdelay(msecs)
#define ICP_MSLEEP(msecs)          msleep(msecs)
#define ICP_SSLEEP(secs)           ssleep(secs)

/* memory */
#define ICP_ZALLOC_GEN(size)       kzalloc(size, GFP_KERNEL)
#define ICP_MALLOC_ATOMIC(size)    kmalloc(size, GFP_ATOMIC)
#define ICP_MALLOC_GEN(size)       kmalloc(size, GFP_KERNEL)
#define ICP_MALLOC_COHERENT(dev, size, dma_addr) \
         dma_alloc_coherent(dev, size, dma_addr, GFP_KERNEL)
#define ICP_VMALLOC(size)          vmalloc(size)
#define ICP_MALLOC_GEN_NUMA(size, node_id) \
                     kmalloc_node(size, GFP_KERNEL, node_id)
#define ICP_FREE(ptr)        \
do {                         \
    if (ptr) {               \
        kfree(ptr);          \
        ptr = NULL;          \
    }                        \
} while(0)
#define ICP_VFREE(ptr)       \
do {                         \
    if (ptr) {               \
        vfree(ptr);          \
        ptr = NULL;          \
    }                        \
} while(0)
#define ICP_FREE_COHERENT(dev, size, cpu_addr, dma_addr) \
do {                        \
    if (cpu_addr) {         \
         dma_free_coherent(dev, size, cpu_addr, dma_addr); \
        cpu_addr = NULL;    \
    }                       \
} while(0)
#define ICP_FREE_NUMA(ptr)         ICP_FREE(ptr)

/* pci config access macros */
#define ICP_READ_CONFIG_DWORD      pci_read_config_dword
#define ICP_READ_CONFIG_WORD       pci_read_config_word
#define ICP_READ_CONFIG_BYTE       pci_read_config_byte

/* iomap functions */
#define ICP_IOMAP(addr, size)      ioremap(addr, size)
#define ICP_IOUNMAP(addr, size)    iounmap(addr)

/* string conversion*/
#define ICP_STRTOL                 simple_strtol
#define ICP_STRTOUL                simple_strtoul
#define ICP_STRTOULL               simple_strtoull

/* driver entry/exit points */
#define ICP_MODULE_INIT(func)      module_init(func)
#define ICP_MODULE_EXIT(func)      module_exit(func)

/* character device file structures */
#define ICP_CHRDEV_FILE_OPS        struct file_operations
#define ICP_CHRDEV_CDEV            struct cdev
#define ICP_CHRDEV_CLASS           struct class
#define ICP_DMA_ADDR               dma_addr_t

#define ICP_ISDIGIT                isdigit
#define ICP_ISSPACE                isspace

static inline CpaStatus adf_checkRingAlignment(Cpa64U baseAddr,
                                               Cpa32U sizeInBytes)
{
    if (((sizeInBytes - 1) & baseAddr) != 0) {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

#endif /* ICP_PLATFORM_LINUX_H */
