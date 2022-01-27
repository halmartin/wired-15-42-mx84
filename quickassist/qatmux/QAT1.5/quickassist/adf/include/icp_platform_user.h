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
 * @file icp_platform_user.h
 *
 * @description
 *      This file contains user space specific macros
 *
 *****************************************************************************/
#ifndef ICP_PLATFORM_USER_H
#define ICP_PLATFORM_USER_H
/* ***********************************************************
 * USER SPACE MACROS
 ************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "Osal.h"
#include <ctype.h>

#define VOLATILE volatile
#define KERN_ERR ""
#define KERN_INFO ""

#define ICP_MDELAY         sleep
#define ICP_SSLEEP         sleep

/* string conversion*/
#define ICP_STRTOL         strtol
#define ICP_STRTOUL        strtoul
#define ICP_STRTOULL       strtoull

#define printk             printf

/* memory */
#define ICP_MALLOC_GEN(size)     malloc(size)
#define ICP_FREE(ptr)     \
    do {                  \
        if(ptr) {         \
            free(ptr);    \
            ptr = NULL;   \
        }                 \
    } while(0)
#define ICP_ZALLOC_GEN(size)       calloc(1, size)
#define ICP_MALLOC_ATOMIC(size)    malloc(size)
#define STATIC static
#define ICP_MMAP(addr,len,prot,flags,fd,offset) \
                                   mmap(addr, len, prot,flags,fd,offset)
#define ICP_MUNMAP(addr_v,len) \
                                 munmap(addr_v, len)
#define ICP_READ(fd, buf, count)   read(fd, buf, count)

#define ICP_MSLEEP(msecs)  \
do {                                                        \
    if (msecs < 1000)                                       \
    {                                                       \
        usleep(1000*msecs);                                 \
    }                                                       \
    else                                                    \
    {                                                       \
        sleep(msecs/1000);                                  \
    }                                                       \
} while(0)

#define ICP_USLEEP(x) usleep(x)

#define ICP_ISDIGIT                isdigit
#define ICP_ISSPACE                isspace
#endif /* ICP_PLATFORM_USER_H */
