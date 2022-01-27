/***************************************************************************
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
 ***************************************************************************/
/**
 ****************************************************************************
 * @file qae_mem_utils.c
 *
 * This file provides for Linux user space memory allocation. It uses
 * a driver that allocates the memory in kernel memory space (to ensure
 * physically contiguous memory) and maps it to
 * user space for use by the  quick assist sample code
 *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#ifndef ICP_WITHOUT_THREAD
#include <pthread.h>
#endif
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>
#include "qae_mem.h"
#include "qae_mem_utils.h"
#include "qae_mem_user_utils.h"
#include "qae_page_table.h"
#include "qae_mem_hugepage_utils.h"

STATIC int fd = -1;

/**************************************************************************
                                   macro
**************************************************************************/

#define QAE_MEM "/dev/usdm_drv"

/**************************************************************************
    static variable
**************************************************************************/

/* Current cached memory size. */
static size_t g_cache_size = 0;
/* Maximum cached memory size, 8 Mb by default */
static size_t g_max_cache = 0x800000;
/* The maximum number we allow to search for available size */
static size_t g_max_lookup_num = 10;
/* User space page table for fast virtual to physical address translation */
static page_table_t g_page_table = {{{0}}};

typedef struct
{
    dev_mem_info_t *head;
    dev_mem_info_t *tail;
} slab_list_t;
/* User space hash for fast slab searching */
static slab_list_t g_slab_list[PAGE_SIZE] = {{0}};

static int g_strict_node = 1;

#ifndef ICP_WITHOUT_THREAD
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

static dev_mem_info_t *pUserCacheHead = NULL;
static dev_mem_info_t *pUserCacheTail = NULL;
static dev_mem_info_t *pUserMemListHead = NULL;
static dev_mem_info_t *pUserMemListTail = NULL;
static dev_mem_info_t *pUserLargeMemListHead = NULL;
static dev_mem_info_t *pUserLargeMemListTail = NULL;


static free_page_table_fptr_t free_page_table_fptr = free_page_table;
static load_addr_fptr_t load_addr_fptr = load_addr;
static load_key_fptr_t load_key_fptr = load_key;

/**************************************************************************
                                  function
**************************************************************************/

API_LOCAL
void set_free_page_table_fptr(free_page_table_fptr_t fp)
{
    free_page_table_fptr = fp;
}

API_LOCAL
void set_loadaddr_fptr(load_addr_fptr_t fp)
{
    load_addr_fptr = fp;
}

API_LOCAL
void set_loadkey_fptr(load_key_fptr_t fp)
{
    load_key_fptr = fp;
}

static inline size_t div_round_up(const size_t n, const size_t d)
{
    return (n + d - 1) / d;
}

static inline void add_slab_to_hash(dev_mem_info_t *slab)
{
    const size_t key = get_key(slab->phy_addr);

    ADD_ELEMENT_TO_HEAD_LIST(
        slab, g_slab_list[key].head, g_slab_list[key].tail, _user_hash);
}
static inline void del_slab_from_hash(dev_mem_info_t *slab)
{
    const size_t key = get_key(slab->phy_addr);

    REMOVE_ELEMENT_FROM_LIST(
        slab, g_slab_list[key].head, g_slab_list[key].tail, _user_hash);
}

static inline dev_mem_info_t *find_slab_in_hash(void *virt_addr)
{
    const size_t key = load_key_fptr(&g_page_table, virt_addr);
    dev_mem_info_t *slab = g_slab_list[key].head;

    while (slab)
    {
        uintptr_t offs = (uintptr_t)virt_addr - (uintptr_t)slab->virt_addr;
        if (offs < slab->size)
            return slab;
        slab = slab->pNext_user_hash;
    }

    return NULL;
}

/* mem_ctzll function
 * input: a 64-bit bitmap window
 * output: number of contiguous 0s from least significant bit position
 * __GNUC__ predefined macro and __builtin_ctz() are supported by Intel C
 */
static inline int32_t mem_ctzll(uint64_t bitmap_window)
{
    if (bitmap_window)
    {
#ifdef __GNUC__
        return __builtin_ctzll(bitmap_window);
#else
#error "Undefined built-in function"
#endif
    }
    return QWORD_WIDTH;
}

/* bitmap_read function
 * reads a 64-bit window from a BITMAP_LENx64-bit bitmap
 * starting from window_pos (0 <-> BITMAP_LENx64 -1)
 * map points to the BITMAP_LENx64 bit map area
 * returns the 64-bit window from the BITMAP_LENx64 bitmap.
 * Each bit represents a 1k block in the 2 Meg buffer
 */

static uint64_t bitmap_read(uint64_t *map, size_t window_pos)
{
    uint64_t quad_word_window = 0ULL;
    uint64_t next_quad_word = 0ULL;
    size_t quad_word_pos = 0;
    size_t bit_pos = 0;

    quad_word_pos = window_pos / QWORD_WIDTH;

    if (quad_word_pos >= BITMAP_LEN)
    {
        return QWORD_ALL_ONE;
    }
    bit_pos = window_pos % QWORD_WIDTH;

    quad_word_window = map[quad_word_pos];

    if (0 == bit_pos)
    {
        return quad_word_window;
    }

    /* it is safe to read the next quad word because
     * there is always a barrier at the end */
    next_quad_word = map[quad_word_pos + 1];

    quad_word_window >>= bit_pos;
    next_quad_word <<= QWORD_WIDTH - bit_pos;
    quad_word_window |= next_quad_word;

    return quad_word_window;
}

static const uint64_t __bitmask[65] = {
    0x0000000000000000ULL, 0x0000000000000001ULL, 0x0000000000000003ULL,
    0x0000000000000007ULL, 0x000000000000000fULL, 0x000000000000001fULL,
    0x000000000000003fULL, 0x000000000000007fULL, 0x00000000000000ffULL,
    0x00000000000001ffULL, 0x00000000000003ffULL, 0x00000000000007ffULL,
    0x0000000000000fffULL, 0x0000000000001fffULL, 0x0000000000003fffULL,
    0x0000000000007fffULL, 0x000000000000ffffULL, 0x000000000001ffffULL,
    0x000000000003ffffULL, 0x000000000007ffffULL, 0x00000000000fffffULL,
    0x00000000001fffffULL, 0x00000000003fffffULL, 0x00000000007fffffULL,
    0x0000000000ffffffULL, 0x0000000001ffffffULL, 0x0000000003ffffffULL,
    0x0000000007ffffffULL, 0x000000000fffffffULL, 0x000000001fffffffULL,
    0x000000003fffffffULL, 0x000000007fffffffULL, 0x00000000ffffffffULL,
    0x00000001ffffffffULL, 0x00000003ffffffffULL, 0x00000007ffffffffULL,
    0x0000000fffffffffULL, 0x0000001fffffffffULL, 0x0000003fffffffffULL,
    0x0000007fffffffffULL, 0x000000ffffffffffULL, 0x000001ffffffffffULL,
    0x000003ffffffffffULL, 0x000007ffffffffffULL, 0x00000fffffffffffULL,
    0x00001fffffffffffULL, 0x00003fffffffffffULL, 0x00007fffffffffffULL,
    0x0000ffffffffffffULL, 0x0001ffffffffffffULL, 0x0003ffffffffffffULL,
    0x0007ffffffffffffULL, 0x000fffffffffffffULL, 0x001fffffffffffffULL,
    0x003fffffffffffffULL, 0x007fffffffffffffULL, 0x00ffffffffffffffULL,
    0x01ffffffffffffffULL, 0x03ffffffffffffffULL, 0x07ffffffffffffffULL,
    0x0fffffffffffffffULL, 0x1fffffffffffffffULL, 0x3fffffffffffffffULL,
    0x7fffffffffffffffULL, 0xffffffffffffffffULL,
};

/* clear_bitmap function
 * clear the BITMAP_LENx64-bit bitmap from pos
 * for len length
 * input : map - pointer to the bitmap
 *         pos - bit position
 *         len - number of contiguous bits
 */
static inline void clear_bitmap(uint64_t *bitmap,
                                const size_t index,
                                size_t len)
{
    size_t qword = index / QWORD_WIDTH;
    const size_t offset = index % QWORD_WIDTH;
    size_t num;

    if (offset > 0)
    {
        const size_t width = MIN(len, QWORD_WIDTH - offset);
        const uint64_t mask = __bitmask[width] << offset;

        /* Clear required bits */
        bitmap[qword] &= ~mask;

        len -= width;
        qword += 1;
    }

    num = len / QWORD_WIDTH;
    len %= QWORD_WIDTH;

    while (num--)
    {
        bitmap[qword++] = 0;
    }

    /* Clear remaining bits */
    bitmap[qword] &= ~__bitmask[len];
}

/* set_bitmap function
 * set the BITMAP_LENx64-bit bitmap from pos
 * for len length
 * input : map - pointer to the bitmap
 *         pos - bit position
 *         len - number of contiguous bits
 */
static inline void set_bitmap(uint64_t *bitmap, const size_t index, size_t len)
{
    size_t qword = index / QWORD_WIDTH;
    const size_t offset = index % QWORD_WIDTH;
    size_t num;

    if (offset > 0)
    {
        const size_t width = MIN(len, QWORD_WIDTH - offset);
        const uint64_t mask = __bitmask[width] << offset;

        /* Set required bits */
        bitmap[qword] |= mask;

        len -= width;
        qword += 1;
    }

    num = len / QWORD_WIDTH;
    len %= QWORD_WIDTH;

    while (num--)
    {
        bitmap[qword++] = ~0ULL;
    }

    /* Set remaining bits */
    bitmap[qword] |= __bitmask[len];
}

/* mem_alloc function
 * mem_alloc allocates memory with min. size = UNIT_SIZE
 * block_ctrl points to a block_ctrl_t structure with virtual address
 * size is the requested number of bytes
 * minimum allocation size is UNIT_SIZE
 * returns a pointer to the newly allocated block
 * input: block_ctrl - pointer to the memory control block
 *        size - size requested in bytes
 * output: pointer to the allocated area
 */
static void *mem_alloc(block_ctrl_t *block_ctrl, size_t size, size_t align)
{
    uint64_t *bitmap = NULL;
    size_t window_pos = 0;
    void *retval = NULL;
    size_t blocks_found = 0;
    uint64_t bitmap_window = 0ULL;
    size_t blocks_required = 0ULL;
    size_t first_block = 0;
    size_t width = 0;
    size_t width_ones = 0;

    if (NULL == block_ctrl || 0 == size)
    {
        CMD_ERROR(" %s:%d invalid control block or size provided "
                  "block_ctrl = %p and size = %d \n",
                  __func__,
                  __LINE__,
                  block_ctrl,
                  size);
        return retval;
    }

    bitmap = block_ctrl->bitmap;

    blocks_required = div_round_up(size, UNIT_SIZE);

    window_pos = 0;
    first_block = window_pos;

    do
    {
        /* read 64-bit bitmap window from window_pos (0-BITMAP_LEN*64) */
        bitmap_window = bitmap_read(bitmap, window_pos);
        /* find number of contiguous 0s from right */
        width = mem_ctzll(bitmap_window);

        /* increment number of blocks found with number of contig. 0s
           in bitmap window */
        blocks_found += width;
        /* check if a fit is found */
        if (blocks_found >= blocks_required)
        {
            /* calculate return address from virtual address and
               first block number */
            retval = (uint8_t *)(block_ctrl) + first_block * UNIT_SIZE;
            if (first_block + blocks_required > BITMAP_LEN * QWORD_WIDTH)
            {
                CMD_ERROR("%s:%d Allocation error - Required blocks exceeds "
                          "bitmap window. Block index = %d, Blocks required"
                          " = %zu and Bitmap window = %d \n",
                          __func__,
                          __LINE__,
                          first_block,
                          blocks_required,
                          (BITMAP_LEN * QWORD_WIDTH));
                return NULL;
            }
            /* save length in the reserved area right after the bitmap  */
            block_ctrl->sizes[first_block] = (uint16_t)blocks_required;
            /* set bit maps from bit position (0<->BITMAP_LEN*64 -1) =
             * first_block(0<->BITMAP_LEN*64-1)
             * with blocks_required length in bitmap
             */
            set_bitmap(bitmap, first_block, blocks_required);
            break;
        }
        else
        {
            /* did not find fit check if bitmap_window has at least a 1*/
            if (bitmap_window)
            {
                /* bit field of 0s not contiguous, clear blocks_found adjust
                 * first_block and window_pos find width of contiguous 1 bits
                 * and move window position will read next 64-bit wide window
                 * from bitmap
                 */
                bitmap_window >>= (width + 1);
                width_ones = mem_ctzll(~bitmap_window);
                blocks_found = 0;
                window_pos += width + 1 + width_ones;
                if (align && window_pos % align)
                {
                    window_pos += align - window_pos % align;
                }
                first_block = window_pos;
            }
            else
            {
                /* bit field of 0s is contiguous, but fit not found yet
                 * move window_pos an search more 0s */
                window_pos += width;
            }
        }
    } while (window_pos < BITMAP_LEN * QWORD_WIDTH);
    return retval;
}
/*
 * deallocates previously allocated blocks
 * block_ctrl is a pointer to block_ctrl_t structure
 * block is a result from a previous mem_alloc call
 */
static void mem_free(block_ctrl_t *block_ctrl, void *block)
{
    size_t first_block = 0;
    uint32_t length = 0;
    uint8_t *start_of_block = block;
    uint64_t *bitmap = NULL;

    if (NULL == block_ctrl || NULL == block)
    {
        CMD_ERROR("%s:%d One of the parameters is NULL. block_ctrl = %p "
                  "block = %p\n",
                  __func__,
                  __LINE__,
                  block_ctrl,
                  block);
        return;
    }

    if ((uintptr_t)block % UNIT_SIZE)
    {
        CMD_ERROR("%s:%d Block address(%p) must be multiple of Unit size(%d)\n",
                  __func__,
                  __LINE__,
                  block,
                  UNIT_SIZE);
        return;
    }

    bitmap = block_ctrl->bitmap;

    /* find start of block in block numbers using the address of start of
     * buffer and block retrieve first_block and length of block from integer
     * at the start of block
     */
    first_block =
        (uintptr_t)(start_of_block - (uint8_t *)(block_ctrl)) / UNIT_SIZE;

    length = block_ctrl->sizes[first_block];

    if (length + first_block > BITMAP_LEN * QWORD_WIDTH)
    {
        CMD_ERROR("%s:%d Invalid block address provided - "
                  "block length exceeds bitmap window. block index = %d "
                  "and block length: %d\n",
                  __func__,
                  __LINE__,
                  first_block,
                  length);
        return;
    }
    /* clear bitmap from bitmap position (0<->BITMAP_LEN*64 - 1) for length*/
    clear_bitmap(bitmap, first_block, length);

#ifndef ICP_DISABLE_SECURE_MEM_FREE
    memset(block, 0, length * UNIT_SIZE);
#endif
}

static dev_mem_info_t *userMemLookupBySize(size_t size,
                                           int node,
                                           void **block,
                                           const size_t align)
{
    dev_mem_info_t *pCurr = NULL;
    size_t link_num = 0;

    for (pCurr = pUserMemListHead; pCurr != NULL; pCurr = pCurr->pNext_user)
    {
        if (g_strict_node && (pCurr->nodeId != node))
        {
            continue;
        }
        *block = mem_alloc((block_ctrl_t *)pCurr, size, align);
        if (NULL != *block)
        {
            return pCurr;
        }
        /* Prevent from visiting whole chain, because after the first
         * several node, the chance to get one is very small.
         * Another consideration is to prevent new allocation from old
         * link, so that the old link could be released
         */
        link_num++;
        if (link_num >= g_max_lookup_num)
        {
            break;
        }
    }
    return NULL;
}

static inline void *init_slab_and_alloc(block_ctrl_t *slab,
                                        const size_t size,
                                        const size_t phys_align_unit)
{
    const size_t last = slab->mem_info.size / CHUNK_SIZE;
    dev_mem_info_t *p_ctrl_blk = &slab->mem_info;
    const size_t reserved = div_round_up(sizeof(block_ctrl_t), UNIT_SIZE);
    void *virt_addr = NULL;

    /* initialise the bitmap to 1 for reserved blocks */
    slab->bitmap[0] = (1ULL << reserved) - 1;
    /* make a barrier to stop search at the end of the bitmap */
    slab->bitmap[last] = QWORD_ALL_ONE;

    virt_addr = mem_alloc(slab, size, phys_align_unit);
    if (NULL != virt_addr)
    {
        ADD_ELEMENT_TO_HEAD_LIST(
            p_ctrl_blk, pUserMemListHead, pUserMemListTail, _user);
    }
    return virt_addr;
}

static inline int push_slab(dev_mem_info_t *slab)
{
    if (g_cache_size + slab->size <= g_max_cache)
    {
        g_cache_size += slab->size;
        ADD_ELEMENT_TO_HEAD_LIST(slab, pUserCacheHead, pUserCacheTail, _user);
        return 0;
    }
    return -ENOMEM;
}

static inline dev_mem_info_t *pop_slab(const int node)
{
    dev_mem_info_t *slab = NULL;

    for (slab = pUserCacheHead; slab != NULL; slab = slab->pNext_user)
    {
        if (node != NUMA_ANY_NODE)
            if (g_strict_node && (node != slab->nodeId))
                continue;

        g_cache_size -= slab->size;
        REMOVE_ELEMENT_FROM_LIST(slab, pUserCacheHead, pUserCacheTail, _user);
        return slab;
    }
    return NULL;
}

static inline void free_slab(const int fd, dev_mem_info_t *slab)
{
    dev_mem_info_t memInfo;
    int ret = 0;

    del_slab_from_hash(slab);

    memcpy(&memInfo, slab, sizeof(dev_mem_info_t));
    /* Need to disconnect from orignal chain */
    ret = qae_munmap(memInfo.virt_addr, memInfo.size);
    if (ret)
    {
        CMD_ERROR("%s:%d munmap failed, ret = %d\n", __func__, __LINE__, ret);
    }
    if (LARGE == memInfo.type)
    {
        ret = qae_munmap(slab, getpagesize());
        if (ret)
        {
            CMD_ERROR(
                "%s:%d munmap failed, ret = %d\n", __func__, __LINE__, ret);
        }
    }

#ifndef __FreeBSD__
    if (HUGE_PAGE == memInfo.type)
    {
        hugepage_free_slab(&memInfo);
    }
    else
#endif
    {
        ret = mem_ioctl(fd, DEV_MEM_IOC_MEMFREE, &memInfo);
        if (unlikely(ret))
        {
            CMD_ERROR("%s:%d ioctl call for mem free failed, ret = %d\n",
                      __func__,
                      __LINE__,
                      ret);
        }
    }
}

static inline dev_mem_info_t *find_slab(const int fd,
                                        const size_t size,
                                        const int node,
                                        void **addr,
                                        const size_t align)
{
    dev_mem_info_t *slab = userMemLookupBySize(size, node, addr, align);

    if (NULL == slab)
    {
        slab = pop_slab(node);
        if (NULL != slab)
        {
            *addr = init_slab_and_alloc((block_ctrl_t *)slab, size, align);
            if (NULL == *addr)
            {
                CMD_ERROR("%s:%d Memory allocation failed Virtual address: %p "
                          " Size: %x \n",
                          __func__,
                          __LINE__,
                          slab,
                          size);
                free_slab(fd, slab);
                return NULL;
            }
        }
    }
    return slab;
}

/**************************************
 * Memory functions
 *************************************/
void *qaeMemAlloc(size_t memsize)
{
    void *memPtr = NULL;
    memPtr = calloc(memsize, sizeof(uint8_t));
    return memPtr;
}

void qaeMemFree(void **ptr)
{
    if ((!ptr) || !(*ptr))
    {
        CMD_ERROR("%s:%d Trying to Free NULL Pointer\n", __func__, __LINE__);
        return;
    }
    free(*ptr);
    *ptr = NULL;
}

static inline int check_pid(void)
{
    static pid_t pid = 0;

    if (pid != getpid())
    {
        pid = getpid();
        return 1;
    }
    return 0;
}

static inline int qaeOpenFd(void)
{
    /* Check if it is a new process or child. */
    const int is_new_pid = check_pid();

    if (fd < 0 || is_new_pid)
    {
        /* Reset all control structures. */
        free_page_table_fptr(&g_page_table);
        memset(&g_page_table, 0, sizeof(g_page_table));
        memset(&g_slab_list, 0, sizeof(g_slab_list));
        g_cache_size = 0;

        pUserCacheHead = NULL;
        pUserCacheTail = NULL;
        pUserMemListHead = NULL;
        pUserMemListTail = NULL;
        pUserLargeMemListHead = NULL;
        pUserLargeMemListTail = NULL;

        CMD_DEBUG("%s:%d Memory file handle is not initialized. "
                  "Initializing it now \n",
                  __func__,
                  __LINE__);

        if (fd > 0)
            close(fd);
        fd = qae_open(QAE_MEM, O_RDWR);
        if (fd < 0)
        {
            CMD_ERROR("%s:%d Unable to initialize memory file handle %s \n",
                      __func__,
                      __LINE__,
                      QAE_MEM);
            return -ENOENT;
        }

        if (init_hugepages(fd))
            return -EIO;
    }
    return 0;
}

int32_t qaeMemInit()
{
    int32_t fd_status = 0;
    int32_t status = 0;

    status = mem_mutex_lock(&mutex);
    if (status)
    {
        CMD_ERROR("%s:%d Error on thread mutex lock %s\n",
                  __func__,
                  __LINE__,
                  strerror(status));
        return -EIO;
    }

    fd_status = qaeOpenFd();

    status = mem_mutex_unlock(&mutex);
    if (status)
    {
        CMD_ERROR("%s:%d Error on thread mutex unlock %s\n",
                  __func__,
                  __LINE__,
                  strerror(status));
        return -EIO;
    }
    return fd_status;
}

#ifdef __FreeBSD__
int qaeMemInitAndReturnFd(int *mem_fd)
{
    int status = -1;
    if (NULL != mem_fd)
    {
        status = qaeMemInit();
    }
    if (status == 0)
    {
        *mem_fd = fd;
    }
    return status;
}
#endif /* __FreeBSD__ */

static void destroyList(const int fd, dev_mem_info_t *pList)
{
    dev_mem_info_t *pCurr = pList;

    while (pCurr)
    {
        dev_mem_info_t *next = pCurr->pNext_user;
        free_slab(fd, pCurr);
        pCurr = next;
    }
}

static inline void reset_cache(const int fd)
{
    dev_mem_info_t *slab = NULL;
    do
    {
        slab = pop_slab(NUMA_ANY_NODE);
        if (NULL != slab)
            free_slab(fd, slab);
    } while (slab != NULL);
}

void qaeMemDestroy(void)
{
    int ret = 0;

    /* Free all of the chains */
    ret = mem_mutex_lock(&mutex);
    if (unlikely(ret))
    {
        CMD_ERROR(
            "%s:%d Error(%d) on thread mutex lock \n", __func__, __LINE__, ret);
        return;
    }

    /* release all control buffers */
    free_page_table_fptr(&g_page_table);
    reset_cache(fd);
    destroyList(fd, pUserMemListHead);
    destroyList(fd, pUserLargeMemListHead);

    pUserCacheHead = NULL;
    pUserCacheTail = NULL;
    pUserMemListHead = NULL;
    pUserMemListTail = NULL;
    pUserLargeMemListHead = NULL;
    pUserLargeMemListTail = NULL;

    /* Send ioctl to kernel space to remove block for this pid */
    if (fd > 0)
    {
        ret = mem_ioctl(fd, DEV_MEM_IOC_RELEASE, NULL);
        if (ret)
        {
            CMD_ERROR("%s:%d ioctl call for mem release failed, ret = %d\n",
                      __func__,
                      __LINE__,
                      ret);
        }
        close(fd);
        fd = -1;
    }

    ret = mem_mutex_unlock(&mutex);
    if (unlikely(ret))
    {
        CMD_ERROR("%s:%d Error(%d) on thread mutex unlock\n",
                  __func__,
                  __LINE__,
                  ret);
    }
}

#ifndef __FreeBSD__
static inline void *mem_protect(void *const addr, const size_t len)
{
    int ret = 0;

    ret = qae_madvise(addr, len, MADV_DONTFORK);
    if (0 != ret)
    {
        munmap(addr, len);
        return NULL;
    }
    return addr;
}
#endif

static inline void *mmap_phy_addr(const int fd,
                                  const uint64_t phy_addr,
                                  const size_t len)
{
    void *addr = NULL;

#ifdef __FreeBSD__
    addr =
        qae_mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phy_addr);
    if (0 != mlock(addr, len))
    {
        munmap(addr, len);
        return NULL;
    }
#endif
#ifndef __FreeBSD__
    addr = qae_mmap(NULL,
                    len,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED | MAP_LOCKED,
                    fd,
                    phy_addr);
#endif

    if (MAP_FAILED == addr)
        return NULL;

#ifndef __FreeBSD__
    addr = mem_protect(addr, len);
#endif

    return addr;
}

static inline dev_mem_info_t *ioctl_alloc_slab(const int fd,
                                               const size_t size,
                                               const int node,
                                               enum slabType type)
{
    dev_mem_info_t params = {0};
    int ret = 0;
    dev_mem_info_t *slab = NULL;

    params.size = size;
    params.nodeId = node;
    params.type = type;

    ret = mem_ioctl(fd, DEV_MEM_IOC_MEMALLOC, &params);
    if (ret)
    {
        CMD_ERROR("%s:%d ioctl call for mem allocation failed, ret = %d\n",
                  __func__,
                  __LINE__,
                  ret);
        return NULL;
    }

    if (node != params.nodeId)
    {
        g_strict_node = 0;
    }

    if (SMALL == type)
        slab = mmap_phy_addr(fd, params.phy_addr, params.size);
    else
        slab = mmap_phy_addr(fd, params.phy_addr, getpagesize());

    if (NULL == slab)
    {
        CMD_ERROR("%s:%d mmap on memory allocated through ioctl failed\n",
                  __func__,
                  __LINE__);
        ret = mem_ioctl(fd, DEV_MEM_IOC_MEMFREE, &params);
        if (unlikely(ret))
        {
            CMD_ERROR("%s:%d ioctl call for mem free failed, ret = %d\n",
                      __func__,
                      __LINE__,
                      ret);
        }
        return NULL;
    }

    if (SMALL == type)
        slab->virt_addr = slab;
    else
    {
        slab->virt_addr = mmap_phy_addr(fd, params.phy_addr, params.size);

        if (NULL == slab->virt_addr)
        {
            CMD_ERROR("%s:%d mmap failed for large memory allocation\n",
                      __func__,
                      __LINE__);
            munmap(slab, getpagesize());
            ret = mem_ioctl(fd, DEV_MEM_IOC_MEMFREE, &params);
            if (unlikely(ret))
            {
                CMD_ERROR("%s:%d ioctl call for mem free failed, ret = %d\n",
                          __func__,
                          __LINE__,
                          ret);
            }
            return NULL;
        }
    }

    return slab;
}

static inline dev_mem_info_t *alloc_slab(const int fd,
                                         const size_t size,
                                         const int node,
                                         enum slabType type)
{
    dev_mem_info_t *slab = NULL;

    if (HUGE_PAGE == type)
    {
#ifndef __FreeBSD__
        slab = hugepage_alloc_slab(fd, size, node, type);
#endif
    }
    else
    {
        slab = ioctl_alloc_slab(fd, size, node, type);
    }

    /* Store a slab into the hash table for a fast lookup. */
    if (slab)
        add_slab_to_hash(slab);

    return slab;
}

static inline void *alloc_addr(size_t size,
                               const int node,
                               const size_t phys_alignment_byte)
{
    dev_mem_info_t *p_ctrl_blk = NULL;
    void *pVirtAddress = NULL;
    size_t allocate_pages = 0;
    enum slabType mem_type = SMALL;

    const size_t phys_align_unit = phys_alignment_byte / UNIT_SIZE;
    const size_t reserved = div_round_up(sizeof(block_ctrl_t), UNIT_SIZE);
    /* calculate units needed */
    const size_t requested_pages = div_round_up(size, UNIT_SIZE) + reserved;

    if (0 != qaeOpenFd())
        return NULL;

    if (requested_pages > QAE_NUM_PAGES_PER_ALLOC * QAE_PAGE_SIZE / UNIT_SIZE ||
        phys_alignment_byte >= QAE_NUM_PAGES_PER_ALLOC * QAE_PAGE_SIZE)
    {
        mem_type = LARGE;
        /* Huge page and Large memory are mutually exclusive
         * Since Large slabs are NOT 2 MB aligned, but huge
         * pages are always 2 MB aligned.
         */
        if (hugepage_enabled())
            return NULL;

        size = MAX(size, phys_alignment_byte);
        allocate_pages = div_round_up(size, UNIT_SIZE);
    }
    else
    {
        allocate_pages = QAE_NUM_PAGES_PER_ALLOC * QAE_PAGE_SIZE / UNIT_SIZE;
        if (hugepage_enabled())
            mem_type = HUGE_PAGE;

        p_ctrl_blk = find_slab(fd, size, node, &pVirtAddress, phys_align_unit);

        if (p_ctrl_blk)
        {
            p_ctrl_blk->allocations += 1;
            return pVirtAddress;
        }
    }

    /* Try to allocate memory as much as possible */
    p_ctrl_blk = alloc_slab(fd, allocate_pages * UNIT_SIZE, node, mem_type);
    if (NULL == p_ctrl_blk)
        return NULL;

    store_mmap_range(&g_page_table,
                     p_ctrl_blk->virt_addr,
                     p_ctrl_blk->phy_addr,
                     p_ctrl_blk->size,
                     hugepage_enabled());

    if (LARGE == mem_type)
    {
        p_ctrl_blk->allocations = 1;

        ADD_ELEMENT_TO_HEAD_LIST(
            p_ctrl_blk, pUserLargeMemListHead, pUserLargeMemListTail, _user);

        pVirtAddress = p_ctrl_blk->virt_addr;
    }
    else
    {
        p_ctrl_blk->allocations = 1;

        if ((uintptr_t)p_ctrl_blk->virt_addr % QAE_PAGE_SIZE)
        {
            CMD_ERROR("%s:%d Bad virtual address alignment %lux %x %lux\n",
                      __func__,
                      __LINE__,
                      (uintptr_t)p_ctrl_blk->virt_addr,
                      QAE_NUM_PAGES_PER_ALLOC,
                      QAE_PAGE_SIZE);
            free_slab(fd, p_ctrl_blk);

            return NULL;
        }
        pVirtAddress = init_slab_and_alloc(
            (block_ctrl_t *)p_ctrl_blk, size, phys_align_unit);
        if (NULL == pVirtAddress)
        {
            CMD_ERROR("%s:%d Memory allocation failed Virtual address: %p "
                      " Size: %x \n",
                      __func__,
                      __LINE__,
                      p_ctrl_blk,
                      size);
            free_slab(fd, p_ctrl_blk);

            return NULL;
        }
    }
    return pVirtAddress;
}

void *qaeMemAllocNUMA(size_t size, int node, size_t phys_alignment_byte)
{
    void *pVirtAddress = NULL;
    int ret = 0;
    /* Maximum supported alignment is 4M. */
    const size_t MAX_PHYS_ALIGN = 0x400000;

    if (!size)
    {
        CMD_ERROR("%s:%d Size cannot be zero \n", __func__, __LINE__);
        return NULL;
    }

    if (!phys_alignment_byte || phys_alignment_byte > MAX_PHYS_ALIGN ||
        (phys_alignment_byte & (phys_alignment_byte - 1)))
    {
        CMD_ERROR("%s:%d Invalid alignment parameter %d. It must be non zero, "
                  "not more than %d and multiple of 2 \n",
                  __func__,
                  __LINE__,
                  phys_alignment_byte,
                  MAX_PHYS_ALIGN);
        return NULL;
    }

    ret = mem_mutex_lock(&mutex);
    if (unlikely(ret))
    {
        CMD_ERROR("%s:%d Error on thread mutex lock %s\n",
                  __func__,
                  __LINE__,
                  strerror(ret));
        return NULL;
    }

    pVirtAddress = alloc_addr(size, node, phys_alignment_byte);

    ret = mem_mutex_unlock(&mutex);
    if (unlikely(ret))
    {
        CMD_ERROR("%s:%d Error on thread mutex unlock %s\n",
                  __func__,
                  __LINE__,
                  strerror(ret));
        return NULL;
    }
    return pVirtAddress;
}

static inline void free_addr(void **p_va)
{
    dev_mem_info_t *p_ctrl_blk = NULL;

    if (0 != qaeOpenFd())
        return;

    if ((p_ctrl_blk = find_slab_in_hash(*p_va)) == NULL)
    {
        CMD_ERROR("%s:%d Unable to free as lookup failed on address (%p) "
                  "provided \n",
                  __func__,
                  __LINE__,
                  *p_va);
        return;
    }
    if (SMALL == p_ctrl_blk->type || HUGE_PAGE == p_ctrl_blk->type)
    {
        mem_free((block_ctrl_t *)p_ctrl_blk, *p_va);

        p_ctrl_blk->allocations -= 1;

        if (p_ctrl_blk->allocations)
        {
            *p_va = NULL;
            return;
        }

        REMOVE_ELEMENT_FROM_LIST(
            p_ctrl_blk, pUserMemListHead, pUserMemListTail, _user);
        if (0 != push_slab(p_ctrl_blk))
            free_slab(fd, p_ctrl_blk);
    }
    else
    {
        REMOVE_ELEMENT_FROM_LIST(
            p_ctrl_blk, pUserLargeMemListHead, pUserLargeMemListTail, _user);
        free_slab(fd, p_ctrl_blk);
    }
    *p_va = NULL;
}

void qaeMemFreeNUMA(void **ptr)
{
    int ret = 0;

    if (NULL == ptr)
    {
        CMD_ERROR(
            "%s:%d Input parameter cannot be NULL \n", __func__, __LINE__);
        return;
    }
    if (NULL == *ptr)
    {
        CMD_ERROR(
            "%s:%d Address to be freed cannot be NULL \n", __func__, __LINE__);
        return;
    }
    ret = mem_mutex_lock(&mutex);
    if (ret)
    {
        CMD_ERROR("%s:%d Error on thread mutex lock %s\n",
                  __func__,
                  __LINE__,
                  strerror(ret));
        *ptr = NULL;
        return;
    }

    free_addr(ptr);

    ret = mem_mutex_unlock(&mutex);
    if (ret)
    {
        CMD_ERROR("%s:%d Error on thread mutex unlock %s\n",
                  __func__,
                  __LINE__,
                  strerror(ret));
    }
    return;
}

/*translate a virtual address to a physical address */
uint64_t qaeVirtToPhysNUMA(void *pVirtAddress)
{
    return load_addr_fptr(&g_page_table, pVirtAddress);
}

static int32_t memoryRemap(dev_mem_info_t *head)
{
    // NOT SUPPORTED
    if (NULL != head)
    {
        CMD_ERROR("%s:%d not supported \n", __func__, __LINE__);
        return -EIO;
    }

    return 0;
}

void qaeAtFork()
{
    int ret = 0;
    int32_t status0 = 0;
    int32_t status1 = 0;
    int32_t status2 = 0;

    ret = mem_mutex_lock(&mutex);
    if (unlikely(ret))
    {
        CMD_ERROR(
            "%s:%d Error(%d) on thread mutex lock \n", __func__, __LINE__, ret);
        return;
    }

    status0 = memoryRemap(pUserCacheHead);
    status1 = memoryRemap(pUserMemListHead);
    status2 = memoryRemap(pUserLargeMemListHead);

    ret = mem_mutex_unlock(&mutex);
    if (unlikely(ret))
    {
        CMD_ERROR("%s:%d Error on thread mutex unlock %s\n",
                  __func__,
                  __LINE__,
                  strerror(ret));
        goto fork_exit;
    }

fork_exit:
    if (unlikely(status0))
    {
        CMD_ERROR(
            "%s:%d Failed to remap memory allocations \n", __func__, __LINE__);
    }
    if (unlikely(status1))
    {
        CMD_ERROR(
            "%s:%d Failed to remap memory allocations \n", __func__, __LINE__);
    }
    if (unlikely(status2))
    {
        CMD_ERROR("%s:%d Failed to remap large memory allocations \n",
                  __func__,
                  __LINE__);
    }
    return;
}
