/**
 * @file OsalUsrKrnProxy.c (linux user space)
 *
 * @brief Implementation for NUMA.
 *
 *
 * @par
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
 *  version: QAT1.5.L.1.11.0-36
 */

#include "Osal.h"
#include "OsalOsTypes.h"
#include "OsalDevDrv.h"
#include "OsalDevDrvCommon.h"

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
#include <signal.h>
#include <assert.h>

/* The maximum number we allow to search for available size */
#define MAX_LOOKUP_NUM  10

/* the maximum retry to wait for memcpy ready */
#define MAX_MEMCPY_WAITNUM 100

#define QWORD_WIDTH  (8 * sizeof(UINT64))
#define DWORD_WIDTH  (8 * sizeof (UINT32))
#define WORD_WIDTH  (8 * sizeof(UINT16))
/* size of allocation unit */
#define UNIT_SIZE 1024
#define BITMAP_LEN  ((NUM_PAGES_PER_ALLOC * (PAGE_SIZE/UNIT_SIZE)) / QWORD_WIDTH)
#define BLOCK_SIZES (BITMAP_LEN * QWORD_WIDTH)
#define RSVD_BLOCKS (BLOCK_SIZES / UNIT_SIZE + 1)
#define QWORD_ALL_ONE 0xFFFFFFFFFFFFFFFFULL
#define QWORD_MSB_SET 0x8000000000000000ULL
#define LOWER_WORD_ALL_ONE 0xFFFFUL

#ifdef __FreeBSD__
#define MMAP_FLAGS 0
#else
#define MMAP_FLAGS MAP_PRIVATE
#endif

/*block control structure */
typedef struct block_ctrl_s 
{
    dev_mem_info_t mem_info;   /* memory device info type */
    UINT64 bitmap[BITMAP_LEN]; /* space to keep the bitmap */
    UINT16 sizes[BLOCK_SIZES]; /* space to keep the size of each allocated block */
}block_ctrl_t;

static int fd = -1;
static int fdp = -1;
static int strict_node = 1;

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_page = PTHREAD_MUTEX_INITIALIZER;

static dev_mem_info_t *pUserMemList = NULL;
static dev_mem_info_t *pUserMemListHead = NULL;

static dev_mem_info_t *pUserLargeMemList = NULL;
static dev_mem_info_t *pUserLargeMemListHead = NULL;

static dev_mem_info_t *pUserMemListPage = NULL;
static dev_mem_info_t *pUserMemListHeadPage = NULL;

/* mem_ctzll function
 * input: a 64-bit bitmap window 
 * output: number of contiguous 0s from least significant bit position 
 * __GNUC__ predefined macro and __builtin_ffs() are supported by Intel C
 */

static inline INT32 mem_ctzll(UINT64 bitmap_window)
{
    if( bitmap_window ) 
    {
        UINT32 dword = (UINT32)((bitmap_window << DWORD_WIDTH) >> DWORD_WIDTH);
        if ( dword )
        {
#ifdef __GNUC__
            return __builtin_ffs(dword) - 1;
#else
#error "Undefined built-in function"
#endif
        }
        dword = (UINT32)(bitmap_window >> DWORD_WIDTH);
#ifdef __GNUC__
        return __builtin_ffs(dword) + DWORD_WIDTH - 1;
#else
#error "Undefined built-in function"
#endif
    }
    return QWORD_WIDTH;
}

static inline int mem_mutex_lock(pthread_mutex_t *pmutex)
{ 
      int ret = pthread_mutex_lock(&mutex); 
      if( ret )
      {
          osalLogString (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "mutex lock error: %s\n",
                  strerror(ret), 0, 0, 0, 0, 0 );
      }
      return ret;
}

static inline int mem_mutex_unlock(pthread_mutex_t *pmutex)
{
      int ret = pthread_mutex_unlock(&mutex);
      if ( ret ) 
      {
            osalLogString (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "mutex unlock error %s\n",
                  strerror(ret), 0, 0, 0, 0, 0 );
      }
      return ret;
}

/* bitmap_read function 
 * reads a 64-bit window from a BITMAP_LENx64-bit bitmap
 * starting from window_pos (0 <-> BITMAP_LENx64 -1)
 * map points to the BITMAP_LENx64 bit map area
 * returns the 64-bit window from the BITMAP_LENx64 bitmap. 
 * Each bit represents a 1k block in the 2 Meg buffer
 */

static UINT64 bitmap_read(UINT64 *map, size_t window_pos)
{
    UINT64 quad_word_window = 0ULL;
    UINT64 next_quad_word = 0ULL;
    size_t quad_word_pos = 0;
    size_t bit_pos = 0;

    if( BITMAP_LEN <= window_pos/QWORD_WIDTH ) 
    {
        return QWORD_ALL_ONE;
    } 
    
    quad_word_pos =  BITMAP_LEN - window_pos/QWORD_WIDTH - 1;
    
    bit_pos = window_pos % QWORD_WIDTH;

    quad_word_window = map[quad_word_pos];
    
    if( 0 == bit_pos ) 
    {
        return quad_word_window;
    }

    if( 0 == quad_word_pos ) 
    {
        next_quad_word = QWORD_ALL_ONE;
    } 
    else 
    {
        next_quad_word = map[quad_word_pos - 1];
    }

    quad_word_window>>=bit_pos;
    next_quad_word &= ( (1ULL << bit_pos) -1 );
    next_quad_word <<= QWORD_WIDTH - bit_pos;
    quad_word_window |= next_quad_word; 

    return  quad_word_window;
}
/* modify_bitmap function
 * modify the BITMAP_LENx64-bit bitmap from pos
 * for len length
 * if set nonzero bits from pos for len length are set to 1
 * otherwise they are cleared to zero
 * input : map - pointer to the bitmap
 *         pos - bit position
 *         len - number of contiguous bits
 *         set - 1 to set bits 0 - clear bits
 */
static void modify_bitmap(UINT64* map, size_t pos, size_t len, UINT32 set)
{
    size_t window = 0;
    size_t bitfield_pos = 0;
    size_t bitfield_len = 0; 
    UINT64 mask = 0ULL;
    while (len > 0 ) {
        window = BITMAP_LEN - pos / QWORD_WIDTH - 1;
        bitfield_pos = pos % QWORD_WIDTH;
        if( len + bitfield_pos >= QWORD_WIDTH )
        {
            bitfield_len = QWORD_WIDTH - bitfield_pos;
            mask = ((1ULL << bitfield_pos ) - 1);
        } 
        else 
        {
            bitfield_len = len;
            mask = ((1ULL << bitfield_pos ) - 1) | ~( (1ULL << (bitfield_pos + bitfield_len)) - 1);
        }
        if( set ) 
        {
            map[window] |= ~mask;
        } 
        else 
        {
            map[window] &= mask;
        }
        len-=bitfield_len;
        pos+=bitfield_len;
    }
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
static void* mem_alloc(block_ctrl_t* block_ctrl, size_t size)
{
    UINT64* bitmap = NULL;
    INT32 window_pos = 0;
    void* retval = NULL;
    INT32  blocks_found = 0;
    UINT64 bitmap_window = 0ULL;
    UINT32 blocks_required = 0ULL;
    UINT32 first_block = 0;
    INT32 width = 0;
    INT32 width_ones = 0;
    const UINT32 bit_set = 1; 

    if( NULL == block_ctrl || 0 == size) 
    {
        return retval;
    }

    bitmap = block_ctrl->bitmap;
    
    blocks_required = size / UNIT_SIZE;

    if ( size % UNIT_SIZE ) 
    {
        blocks_required += 1;
    }
    
    do 
    {
        width = 0;

        /* read 64-bit bitmap window from window_pos (0-BITMAP_LEN*64) */
        bitmap_window = bitmap_read(bitmap, window_pos);

        /* check if there is at least a 1 in bitmap window */
        if ( bitmap_window ) 
        {
            /* find number of contiguous 0s from right */
              width = mem_ctzll(bitmap_window);
        } 
        else 
        {
            /* bitmap window is 0 there are 64 contiguous 0s*/ 
            width = QWORD_WIDTH;
        }
        
        /* increment number of blocks found with number of contig. 0s in bitmap window */
        blocks_found += width;
        /* check if a fit is found */
        
        if ( blocks_found >= blocks_required )
        { 
            /* calculate return address from virtual address and first block number */
            retval = (UINT8*)(block_ctrl->mem_info.virt_addr)+first_block * UNIT_SIZE;

            if( first_block + blocks_required > BITMAP_LEN * QWORD_WIDTH)
            {    
                osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                    "Allocation error \n",
                    0, 0, 0, 0, 0, 0, 0, 0);
                return retval;
            }
            /* save length in the reserved area right after the bitmap  */
            block_ctrl->sizes[first_block] =  blocks_required;

            /* set bit maps from bit position (0<->BITMAP_LEN*64 -1) = 
             * first_block(0<->BITMAP_LEN*64-1)
             * with blocks_required length in bitmap 
             */ 
            modify_bitmap(bitmap, first_block, blocks_required, bit_set);
            break;
        } 
        else 
        {
            /* did not find fit check if bitmap_window has at least a 1*/
            if ( bitmap_window ) 
            {
                /* bit field of 0s not contiguous, clear blocks_found adjust first_block
                 * and window_pos find width of contiguous 1 bits and move window position
                 * will read next 64-bit wide window from bitmap
                 */
                width_ones = 0;
                bitmap_window >>= (width+1); 
                width_ones = mem_ctzll(~bitmap_window);
                blocks_found = 0;
                window_pos += width + 1 + width_ones; 
                first_block = window_pos;
            } 
            else 
            {
                /* bit field of 0s is contiguous, but fit not found yet
                 * move window_pos an search more 0s */
                window_pos += width;
            }
        }
    }while(window_pos < BITMAP_LEN * QWORD_WIDTH ); 

    return retval;
}
/* 
 * deallocates previously allocated blocks
 * block_ctrl is a pointer to block_ctrl_t structure
 * block is a result from a previous mem_alloc call
 */ 
static void mem_free(block_ctrl_t *block_ctrl,void *block)
{
    UINT32 first_block = 0;
    UINT32 length = 0;
    UINT8* start_of_block = block;
    UINT64* bitmap = NULL;
    const UINT32 bit_clear = 0;
    
    if( NULL == block_ctrl ) 
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "Block control address is zero \n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return;
    }

    if ( NULL == block || (UARCH_INT)block % UNIT_SIZE ) 
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "Invalid block address \n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return;
    }
    
    bitmap = block_ctrl->bitmap;
    
    /* find start of block in block numbers 
     * using the address of start of buffer and block */  
    /* retrieve first_block and length of block from integer at the start of block */
    
    first_block = (UARCH_INT)( start_of_block - 
                    (UINT8*)(block_ctrl->mem_info.virt_addr )) / UNIT_SIZE;
    
    length = block_ctrl->sizes[first_block];
    
    if( length + first_block > BITMAP_LEN * QWORD_WIDTH ) 
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "Invalid block first block: %d length: %d\n",
                 first_block, length, 0, 0, 0, 0, 0, 0);
        return;
    }
    /* clear bitmap from bitmap position (0<->BITMAP_LEN*64 - 1) for length */    
    modify_bitmap(bitmap, first_block, length, bit_clear);
    memset(block, 0, length*UNIT_SIZE);
}
OSAL_STATUS userMemListAddPage(dev_mem_info_t *pMemInfo)
{
      mem_mutex_lock(&mutex_page);
      ADD_ELEMENT_TO_END_OF_LIST(pMemInfo,
                                 pUserMemListPage,
                                 pUserMemListHeadPage);
      mem_mutex_unlock(&mutex_page);

      return OSAL_SUCCESS;
}

void userMemListFree(dev_mem_info_t *pMemInfo)
{
      dev_mem_info_t *pCurr = NULL;
      int done = 0;

      for (pCurr = pUserLargeMemListHead; pCurr != NULL; pCurr = pCurr->pNext)
      {
          if (pCurr == pMemInfo)
          {
               REMOVE_ELEMENT_FROM_LIST(pCurr, pUserLargeMemList, pUserLargeMemListHead);
               done = 1;
               break;
          }
      }

      for (pCurr = pUserMemListHead; (pCurr != NULL) && (done == 0);
                           pCurr = pCurr->pNext)
      {
          if (pCurr == pMemInfo)
          {
               REMOVE_ELEMENT_FROM_LIST(pCurr, pUserMemList, pUserMemListHead);
               break;
          }
      }
}

void userMemListFreePage(dev_mem_info_t *pMemInfo)
{
      dev_mem_info_t *pCurr = NULL;

      for (pCurr = pUserMemListHeadPage; pCurr != NULL; pCurr = pCurr->pNext)
      {
          if (pCurr == pMemInfo)
          {
               REMOVE_ELEMENT_FROM_LIST(pCurr,
                                        pUserMemListPage,
                                        pUserMemListHeadPage);
               break;
          }
      }
}

dev_mem_info_t *userMemLookupBySize(UINT32 size, UINT32 node, void** block )
{
      dev_mem_info_t *pCurr = NULL;
      int link_num = 0;
      /* Search from new to old */
      for (pCurr = pUserMemListHead; pCurr != NULL; pCurr = pCurr->pNext)
      {
          if (strict_node && (pCurr->nodeId != node))
          {
              continue;
          }
          *block = mem_alloc((block_ctrl_t*)pCurr, size ); 
          if (*block != NULL )
          {
              return pCurr;
          }
          /* Prevent from visiting whole chain, because after the first
           * several node, the chance to get one is very small.
           * Another consideration is to prevent new allocation from old
           * link, so that the old link could be released
           */
          link_num++;
          if (link_num >= MAX_LOOKUP_NUM )
          {
              break;
          }
      }
      return NULL;
}

dev_mem_info_t *userMemLookupByVirtAddr(void* virt_addr)
{
      dev_mem_info_t *pCurr = NULL;

      for (pCurr = pUserMemListHead; pCurr != NULL; pCurr = pCurr->pNext)
      {
          if ((UARCH_INT)pCurr->virt_addr <= (UARCH_INT)virt_addr &&
          ((UARCH_INT)pCurr->virt_addr + pCurr->size) > (UARCH_INT)virt_addr)
          {
              return pCurr;
          }
      }
      return NULL;
}

static dev_mem_info_t *userLargeMemLookupByVirtAddr(void* virt_addr)
{
      dev_mem_info_t *pCurr = NULL;

      for (pCurr = pUserLargeMemListHead; pCurr != NULL; pCurr = pCurr->pNext)
      {
          if ((UARCH_INT)pCurr->virt_addr <= (UARCH_INT)virt_addr &&
          ((UARCH_INT)pCurr->virt_addr + pCurr->size) > (UARCH_INT)virt_addr)
          {
              return pCurr;
          }
      }
      return NULL;
}

dev_mem_info_t *userMemLookupByVirtAddrPage(void* virt_addr)
{
      dev_mem_info_t *pCurr = NULL;

      for (pCurr = pUserMemListHeadPage; pCurr != NULL; pCurr = pCurr->pNext)
      {
           if((UARCH_INT)pCurr->virt_addr == (UARCH_INT)virt_addr)
           {
              return pCurr;
           }
      }
      return NULL;
}

OSAL_PUBLIC OSAL_STATUS
osalMemInitialize(char* path)
{
      char mem_path[DEV_PATH_SIZE] = "";
      char mempage_path[DEV_PATH_SIZE] = "";
      size_t path_len = 0;
      size_t os_dev_dir_len = 0;
      size_t dev_mem_name_page_len = 0;
      size_t dev_mem_name_len = 0;

      if ( fd > 0 && fdp > 0 ) {
          return OSAL_SUCCESS;
      }

      if(path != NULL)
      {
          path_len = OSAL_OS_GET_STRING_LENGTH(path);
          os_dev_dir_len = OSAL_OS_GET_STRING_LENGTH(OS_DEV_DIRECTORY);
          dev_mem_name_page_len = OSAL_OS_GET_STRING_LENGTH(DEV_MEM_NAME_PAGE);
          dev_mem_name_len = OSAL_OS_GET_STRING_LENGTH(DEV_MEM_NAME);

          if (path_len > (DEV_PATH_SIZE - os_dev_dir_len -
                          dev_mem_name_page_len))
          {
              osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "path to device is greater that max length %d\n",
                  fd, 0, 0, 0, 0, 0, 0, 0);
              return OSAL_FAIL;
          }

          strncpy(mem_path, OS_DEV_DIRECTORY, os_dev_dir_len);
          strncpy(mempage_path, OS_DEV_DIRECTORY, os_dev_dir_len);

          strncat(mem_path, path, path_len);
          strncat(mempage_path, path, path_len);
          strncat(mem_path, DEV_MEM_NAME, dev_mem_name_len);
          strncat(mempage_path, DEV_MEM_NAME_PAGE, dev_mem_name_page_len);
      }

      fd = open((path) ? mem_path : DEV_MEM_PATH, O_RDWR);
      if (fd < 0) {
      char fileno[8];
      snprintf(fileno,sizeof fileno,"%d",fd);
          osalLogString (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                         "unable to open %s %d\n",
                         (path) ? mem_path : DEV_MEM_PATH, fileno, 
                         NULL,NULL,NULL,NULL);
          return OSAL_FAIL;
      }

      fdp = open((path) ? mempage_path : DEV_MEM_PAGE_PATH , O_RDWR);
      if (fdp < 0) {
          char fileno[8];
          snprintf(fileno,sizeof fileno,"%d",fdp);
          osalLogString (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                        "unable to open %s %d\n",
                        (path) ? mempage_path : DEV_MEM_PAGE_PATH, fileno, 
                        NULL,NULL,NULL,NULL); 
         close(fd);
         fd = -1;
         return OSAL_FAIL;
      }
      return OSAL_SUCCESS;
}

OSAL_PUBLIC void
osalMemDestroy()
{
      if ( fd > 0 ) {
          close(fd);
          fd = -1;
      }
      if ( fdp > 0 ) {
          close(fdp);
          fdp  = -1;
      }
}

OSAL_PUBLIC void*
osalMemAllocContiguousNUMA(UINT32 size, UINT32 node, UINT32 alignment)
{
      int ret = 0;
      dev_mem_info_t* pMemInfo = NULL;
      void *pVirtAddress = NULL;
      int requested_pages = 0;
      int allocate_pages = 0;
      int mmap_size = 0;
      int alloc_size = 0;
      int align = 0;
      int large_memory = 0;
	

      if (size == 0 || alignment == 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "Invalid size or alignment parameter \n",
                  0, 0, 0, 0, 0, 0, 0, 0);
          return NULL;
      }
      if(fd < 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "Memory file handle %d is not ready\n",
                  fd, 0, 0, 0, 0, 0, 0, 0);
          return NULL;
      }

      if( mem_mutex_lock(&mutex) )
          return NULL;
      
      /* calculate pages needed */
      if( alignment > UNIT_SIZE )
      {
            size += (alignment/UNIT_SIZE - 1)*UNIT_SIZE;
      }
      requested_pages = size/UNIT_SIZE;
      if ( size % UNIT_SIZE ) 
      {
          requested_pages+=1;
      }
      
      if (requested_pages > NUM_PAGES_PER_ALLOC * PAGE_SIZE/UNIT_SIZE )
      {
         large_memory = 1;
         allocate_pages = requested_pages;
      }
      else
      {
         allocate_pages = NUM_PAGES_PER_ALLOC*PAGE_SIZE/UNIT_SIZE;
      }

      if( !large_memory && 
        (pMemInfo = userMemLookupBySize(size , node, &pVirtAddress)) != NULL)
      {
          pMemInfo->allocations += 1;
          if ( mem_mutex_unlock(&mutex) ) 
                return NULL;
              
          return  pVirtAddress;
      }

      pMemInfo = calloc(1, sizeof (block_ctrl_t) );
      
      if ( NULL == pMemInfo )
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "unable to allocate Memory buffer control area\n",
                  0, 0, 0, 0, 0, 0, 0, 0);
          return NULL;
      }
      

      pMemInfo->allocations = 0;
      pMemInfo->nodeId = node;
      pMemInfo->size = allocate_pages * UNIT_SIZE;

      /* Try to allocate memory as much as possible */
      ret = ioctl(fd, DEV_MEM_IOC_MEMALLOC, pMemInfo);
      if (ret != 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "ioctl memory alloc failed, ret = %d\n",
                  ret, 0, 0, 0, 0, 0, 0, 0);
          free(pMemInfo);
          return NULL;
      }

      if (node != pMemInfo->nodeId ) {
          strict_node = 0;
      }

      mmap_size = pMemInfo->size;

      pMemInfo->mmap_size = mmap_size;
      pMemInfo->fvirt_addr = mmap((caddr_t) 0, mmap_size,
                                  PROT_READ|PROT_WRITE, MAP_SHARED, fd,
                                  (pMemInfo->id * getpagesize()));

      if (pMemInfo->fvirt_addr == (caddr_t) MAP_FAILED)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "mmap failed\n",
                  0, 0, 0, 0, 0, 0, 0, 0);
          ret = ioctl(fd, DEV_MEM_IOC_MEMFREE, pMemInfo);
          if (ret != 0)
          {
              osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "ioctl DEV_MEM_IOC_MEMFREE call failed, ret = %d\n",
                  ret, 0, 0, 0, 0, 0, 0, 0);
          }
          free(pMemInfo);
          return NULL;
      }

      /* available size needs to substract current size and header */
      /* As DMA remapping may add extra page, so we need to use alloc_size
         instead of the size in pMemInfo structure */
#ifdef ICP_SRIOV
    /* Save one page for IORemapping */
    alloc_size -= PAGE_SIZE;
#endif
      pMemInfo->allocations = 1;
      pMemInfo->virt_addr = pMemInfo->fvirt_addr;
      if (large_memory)
      {
          pMemInfo->available_size = alloc_size - size;
          pMemInfo->virt_addr = pMemInfo->fvirt_addr;
          pMemInfo->fvirt_addr = NULL;
          ADD_ELEMENT_TO_HEAD_OF_LIST(pMemInfo, pUserLargeMemList, 
                                      pUserLargeMemListHead);
          if( mem_mutex_unlock(&mutex) )
              return NULL;
          pVirtAddress = pMemInfo->virt_addr;
      }
      else
      {
          if ((UARCH_INT)pMemInfo->virt_addr % PAGE_SIZE)
          {
               osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "Bad virtual address alignment %x %x %x\n",
                  (UARCH_INT)pMemInfo->virt_addr, NUM_PAGES_PER_ALLOC, PAGE_SIZE, 0, 0, 0, 0, 0);
               ioctl(fd, DEV_MEM_IOC_MEMFREE, pMemInfo);
               free(pMemInfo); 
               return NULL;
          }

          pVirtAddress = mem_alloc( (block_ctrl_t*)pMemInfo, size );
          if( NULL == pVirtAddress ) 
          {
               osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "Memory allocation failed Virtual address: %x Size: %x \n",
                  (UARCH_INT)pMemInfo->virt_addr, size, 0, 0, 0, 0, 0, 0);
               ioctl(fd, DEV_MEM_IOC_MEMFREE, pMemInfo);
               free(pMemInfo); 
               return NULL;
          } 
          ADD_ELEMENT_TO_HEAD_OF_LIST(pMemInfo, 
                                       pUserMemList, pUserMemListHead);
          if( mem_mutex_unlock(&mutex) )
              return NULL;
      }

      if ( alignment > UNIT_SIZE )
      {
          align = alignment - ((UARCH_INT)pVirtAddress % alignment);
          pVirtAddress = (void*)((UARCH_INT)pVirtAddress + align);
      }   
      return pVirtAddress;
}

OSAL_PUBLIC void*
osalMemAllocPage(UINT32 node, UINT64 *physAddr)
{
      int ret = 0;
      dev_mem_info_t *pMemInfo = NULL;

      if (fd < 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "Memory file handle is not ready\n",
                  fd, 0, 0, 0, 0, 0, 0, 0);
          return NULL;
      }

      pMemInfo = malloc(sizeof(dev_mem_info_t));
      if (NULL == pMemInfo)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "unable to allocate pMemInfo buffer\n",
                  fd, 0, 0, 0, 0, 0, 0, 0);
          return NULL;
      }

      pMemInfo->nodeId = node;
      pMemInfo->size = getpagesize();

      ret = ioctl(fdp, DEV_MEM_IOC_MEMALLOCPAGE, pMemInfo);
      if (ret != 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "ioctl call failed, ret = %d\n",
                  ret, 0, 0, 0, 0, 0, 0, 0);
          free(pMemInfo);
          return NULL;
      }

      pMemInfo->virt_addr = mmap(NULL, pMemInfo->size, PROT_READ|PROT_WRITE,
                                 MAP_PRIVATE, fdp, pMemInfo->id * getpagesize());
      if (pMemInfo->virt_addr == MAP_FAILED)
      {
          osalStdLog("Errno: %d\n", errno);
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "mmap failed\n",
                  0, 0, 0, 0, 0, 0, 0, 0);
          ret = ioctl(fd, DEV_MEM_IOC_MEMFREEPAGE, pMemInfo);
          if (ret != 0)
          {
              osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "ioctl call failed, ret = %d\n",
                  ret, 0, 0, 0, 0, 0, 0, 0);
           }
           free(pMemInfo);
           return NULL;
      }

      userMemListAddPage(pMemInfo);
      *physAddr = pMemInfo->phy_addr;
      return pMemInfo->virt_addr;
}

OSAL_PUBLIC void
osalMemFreeNUMA(void* pVirtAddress)
{
      int ret = 0;
      dev_mem_info_t *pMemInfo = NULL;

      if (NULL == pVirtAddress)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "Invalid virtual address\n",
                  0, 0, 0, 0, 0, 0, 0, 0);
          return;
      }

      if( mem_mutex_lock(&mutex) )
          return;

      if ((pMemInfo = userMemLookupByVirtAddr(pVirtAddress)) != NULL)
      {
          pMemInfo->allocations -= 1;
          if (pMemInfo->allocations != 0)
          {
              mem_free((block_ctrl_t*)pMemInfo, pVirtAddress);    
              mem_mutex_unlock(&mutex); 
              return;
          }
      }
      else
      {
          if ((pMemInfo = userLargeMemLookupByVirtAddr(pVirtAddress)) == NULL)
          {
              osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "userMemLookupByVirtAddr failed\n",
                  0, 0, 0, 0, 0, 0, 0, 0);
              mem_mutex_unlock(&mutex);
              return;
          }
      }

      if (pMemInfo->fvirt_addr)
      {
          ret = munmap(pMemInfo->fvirt_addr, pMemInfo->mmap_size);
      }
      else
      {
          ret = munmap(pMemInfo->virt_addr, pMemInfo->mmap_size);
      }

      if (ret != 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "munmap failed, ret = %d\n",
                  ret, 0, 0, 0, 0, 0, 0, 0);
      }

      ret = ioctl(fd, DEV_MEM_IOC_MEMFREE, pMemInfo);

      if (ret != 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "ioctl call failed, ret = %d\n",
                  ret, 0, 0, 0, 0, 0, 0, 0);
      }

      userMemListFree(pMemInfo);
      free(pMemInfo);
      mem_mutex_unlock(&mutex);

      return;
}

OSAL_PUBLIC void
osalMemFreePage(void* pVirtAddress)
{
      int ret = 0;
      dev_mem_info_t *pMemInfo = NULL;

      if (NULL == pVirtAddress)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "Invalid virtual address\n",
                  0, 0, 0, 0, 0, 0, 0, 0);
          return;
      }

      mem_mutex_lock(&mutex_page);
      pMemInfo = userMemLookupByVirtAddrPage(pVirtAddress);

      if(pMemInfo == NULL)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "userMemLookupByVirtAddrPage failed \n", 0, 0, 0, 0,
                  0, 0, 0, 0);
           mem_mutex_unlock(&mutex_page);
           return;
      }

      ret = munmap(pMemInfo->virt_addr, getpagesize());
      if (ret != 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "munmap failed, ret = %d\n",
                  ret, 0, 0, 0, 0, 0, 0, 0);
      }

      ret = ioctl(fdp, DEV_MEM_IOC_MEMFREEPAGE, pMemInfo);
      if (ret != 0)
      {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "ioctl call failed, ret = %d\n",
                  ret, 0, 0, 0, 0, 0, 0, 0);
      }
      userMemListFreePage(pMemInfo);
      free(pMemInfo);
      mem_mutex_unlock(&mutex_page);
      return;
}

UINT64
osalVirtToPhysNUMA(void* pVirtAddress)
{
    dev_mem_info_t *pMemInfo = NULL;
    UARCH_INT offset = 0;
    UINT64 phy_address = 0;
      
    OSAL_LOCAL_ENSURE(pVirtAddress != NULL,
                  "osalVirtToPhysNUMA():   Null virtual address pointer", 0);
    /* Firstly search the large memory tree */
    if ( mem_mutex_lock(&mutex) ) 
        return 0;
 
    if ((pMemInfo = userLargeMemLookupByVirtAddr(pVirtAddress)) != NULL)
    {
        if ( mem_mutex_unlock(&mutex) )
            return 0;
        return (UINT64)(pMemInfo->phy_addr + (UARCH_INT)pVirtAddress -
                 (UARCH_INT)pMemInfo->virt_addr);
    }
    pMemInfo = userMemLookupByVirtAddr(pVirtAddress);

    mem_mutex_unlock(&mutex); 

    if ( NULL == pMemInfo ) 
    {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                "Invalid block address %x !\n",
                (UARCH_INT)pVirtAddress, 0, 0, 0, 0, 0, 0, 0);
        return (UINT64)0;
    }

    offset = (UARCH_INT)pVirtAddress - (UARCH_INT)(pMemInfo->virt_addr);
    phy_address = pMemInfo->phy_addr;

    return (UINT64)(phy_address + offset);
}

#ifndef ICP_WITHOUT_IOMMU
int osalIOMMUMap(UINT64 phaddr, UINT64 iova, size_t size)
{
      dev_iommu_info_t info = {phaddr, iova, size};
      return ioctl(fd, DEV_MEM_IOC_IOMMUMAP, &info);
}

int osalIOMMUUnmap(UINT64 iova, size_t size)
{
      dev_iommu_info_t info = {0, iova, size};
      return ioctl(fd, DEV_MEM_IOC_IOMMUUNMAP, &info);
}

UINT64 osalIOMMUVirtToPhys(UINT64 iova)
{
      dev_iommu_info_t info = {0, iova, 0};
      return (ioctl(fd, DEV_MEM_IOC_IOMMUVTOP, &info) == 0) ? info.phaddr : 0;
}

size_t osalIOMMUgetRemappingSize(size_t size)
{
      int pages = (size >> PAGE_SHIFT) + 1;
      size_t new_size = (pages * PAGE_SIZE);
      return new_size;
}
#else
int osalIOMMUMap(UINT64 phaddr, UINT64 iova, size_t size)
{
      return 0;
}

int osalIOMMUUnmap(UINT64 iova, size_t size)
{
      return 0;
}

UINT64 osalIOMMUVirtToPhys(UINT64 iova)
{
      return iova;
}

size_t osalIOMMUgetRemappingSize(size_t size)
{
      return size;
}
#endif

