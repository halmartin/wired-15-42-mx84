/**
 * @file kernel_space/OsalMemMgmt.c (linux)
 *
 * @brief OS-specific Proxy Memory operations
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

#include "Osal.h"
#include "OsalOsTypes.h"
#include "OsalDevDrv.h"
#include "OsalDevDrvCommon.h"

#include <linux/version.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/nodemask.h>
#include <asm/io.h>
/*
 * Find memory information
 */
dev_mem_info_t *userMemGetInfo(user_mem_dev_t* dev, UINT32 id, UINT32 pid)
{
    user_proc_mem_list_t *list = NULL;
    dev_mem_info_t *mem = NULL;

    list = dev->head;
    while(list)
    {
        if(pid == list->pid)
        {
            mem = list->head;
            while(mem)
            {
                if(mem->id == id)
                {
                    return mem;
                }
                mem = mem->pNext;
            }
        }
        list = list->pNext;
    }
    return NULL;
}

dev_mem_info_t *userMemGetInfoPage(user_mem_dev_t* dev,
        UINT32 id, UINT32 pid)
{
    return userMemGetInfo(dev, id, pid);
}

/*
 * Allocate numa memory
 */
dev_mem_info_t *userMemAlloc(user_mem_dev_t* dev, UINT32 sizeInBytes,
                             UINT32 node, UINT32 pid)
{
    dev_mem_info_t *mem_info = NULL;
    user_proc_mem_list_t *list = NULL;

    if (0 == sizeInBytes)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                    "Invalid parameter value [%u]\n", sizeInBytes,
                    0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }

    /*check if given node is online, fall back to node=0 if not*/
    if(!node_online(node))
    {
        node = 0;
    }

    /*
     * Find the process allocation list
     */
    list = dev->head;
    while(list)
    {
        if(pid == list->pid)
            break;
        list = list->pNext;
    }
    if (NULL == list)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "userMemAlloc(): find pid in the list failed\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }

    /*
     * Check if we have some free slots
     */
    if(list->allocs_nr == list->max_id)
    {
        /* Allocate one if we dont have a free slot */
        mem_info = kzalloc_node(sizeof(dev_mem_info_t), GFP_KERNEL, node);
        if (NULL == mem_info)
        {
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                     "userMemAlloc(): allocation failed\n",
                     0, 0, 0, 0, 0, 0, 0, 0);
            return NULL;
        }

        mem_info->id = list->max_id++;
    }
    else
    {
        /* Reuse a free slot */
        mem_info = list->freed_tail;
        REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail, list->freed_head);
    }
    mem_info->nodeId = node;
    mem_info->size = osalIOMMUgetRemappingSize(sizeInBytes);
    mem_info->kmalloc_ptr = kzalloc_node(mem_info->size, GFP_KERNEL, node);
    if (NULL == mem_info->kmalloc_ptr)
    {
           osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                    "Unable to allocate memory mem_info->kmalloc_ptr\n",
                    0, 0, 0, 0, 0, 0, 0, 0);
        ADD_ELEMENT_TO_END_OF_LIST(mem_info, list->freed_tail, list->freed_head);
        return NULL;
    }

    mem_info->kmalloc_area = ((int *)
        ((((unsigned long)mem_info->kmalloc_ptr) + PAGE_SIZE -1) & PAGE_MASK));
    mem_info->phy_addr = virt_to_phys(mem_info->kmalloc_ptr);
    osalIOMMUMap(mem_info->phy_addr, mem_info->phy_addr, mem_info->size);
    list->allocs_nr++;
    ADD_ELEMENT_TO_END_OF_LIST(mem_info, list->tail, list->head);
    return mem_info;
}

/*
 * Alloc page
 */
dev_mem_info_t *userMemAllocPage(user_mem_dev_t* dev,
                                 UINT32 node, UINT32 pid)
{
    dev_mem_info_t *mem_info = NULL;
    user_proc_mem_list_t *list = NULL;

    list = dev->head;
    while(list)
    {
        if(pid == list->pid)
            break;
        list = list->pNext;
    }
    if (NULL == list)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "userMemAlloc(): find pid in the list failed\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }

    /*
     * Check if we have some free slots
     */
    if(list->allocs_nr == list->max_id)
    {
        /* Allocate one if we dont have a free slot */
        mem_info = kzalloc_node(sizeof(dev_mem_info_t), GFP_KERNEL, node);
        if (NULL == mem_info)
        {
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                     "userMemAlloc(): allocation failed \n",
                     0, 0, 0, 0, 0, 0, 0, 0);
            return NULL;
        }

        mem_info->id = list->max_id++;
    }
    else
    {
        /* Reuse a free slot */
        mem_info = list->freed_tail;
        REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail, list->freed_head);
    }

    mem_info->size = OSAL_PAGE_SIZE;
    mem_info->kmalloc_ptr = (void *)get_zeroed_page(GFP_KERNEL);
    if (NULL == mem_info->kmalloc_ptr)
    {
           osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                    "Unable to allocate memory mem_info->kmalloc_ptr\n",
                    0, 0, 0, 0, 0, 0, 0, 0);

        ADD_ELEMENT_TO_END_OF_LIST(mem_info, list->freed_tail, list->freed_head);
        return NULL;
    }

    mem_info->kmalloc_area = ((int *)
        ((((unsigned long)mem_info->kmalloc_ptr) + PAGE_SIZE -1) & PAGE_MASK));

    mem_info->phy_addr = virt_to_phys(mem_info->kmalloc_ptr);
    list->allocs_nr++;
    ADD_ELEMENT_TO_END_OF_LIST(mem_info, list->tail, list->head);
    return mem_info;
}

/*
 * Free memory
 */
void userMemFree(user_mem_dev_t* dev, UINT32 id, UINT32 pid)
{
    dev_mem_info_t *mem_info = NULL;
    dev_mem_info_t *freed_info = NULL;
    user_proc_mem_list_t *list = NULL;

    list = dev->head;
    while(list)
    {
        if(pid == list->pid)
        {
            mem_info = list->head;
            while(mem_info)
            {
                if(mem_info->id == id)
                {
                    osalIOMMUUnmap(mem_info->phy_addr, mem_info->size);
                    FREE(mem_info->kmalloc_ptr);
                    mem_info->kmalloc_ptr = NULL;
                    mem_info->size = 0;
                    REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
                    list->allocs_nr--;
                    if(mem_info->id != list->max_id)
                    {
                        ADD_ELEMENT_TO_END_OF_LIST(mem_info,
                                list->freed_tail, list->freed_head);
                    }
                    else
                    {
                        FREE(mem_info);
                        list->max_id--;
                        freed_info = list->freed_tail;
                        while(freed_info->id == list->max_id)
                        {
                            dev_mem_info_t *ptr = freed_info->pPrev;
                            list->max_id--;
                            FREE(freed_info);
                            freed_info = ptr;
                        }
                    }
                    return;
                }
                mem_info = mem_info->pNext;
            }
        }
        list = list->pNext;
    }
    osalStdLog("Could not find id: %d pid: %d\n", id, pid);
    return;
}

/*
 * Free page
 */
void userMemFreePage(user_mem_dev_t* dev,
                     UINT32 id, UINT32 pid)
{
    dev_mem_info_t *mem_info = NULL;
    dev_mem_info_t *freed_info = NULL;
    user_proc_mem_list_t *list = NULL;

    list = dev->head;
    while(list)
    {
        if(pid == list->pid)
        {
            mem_info = list->head;
            while(mem_info)
            {
                if(mem_info->id == id)
                {
                    free_page((unsigned long)mem_info->kmalloc_ptr);
                    mem_info->kmalloc_ptr = NULL;
                    REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
                    list->allocs_nr--;
                    if(mem_info->id != list->max_id)
                    {
                        ADD_ELEMENT_TO_END_OF_LIST(mem_info,
                                list->freed_tail, list->freed_head);
                    }
                    else
                    {
                        FREE(mem_info);
                        list->max_id--;
                        freed_info = list->freed_tail;
                        while(freed_info->id == list->max_id)
                        {
                            dev_mem_info_t *ptr = freed_info->pPrev;
                            list->max_id--;
                            FREE(freed_info);
                            freed_info = ptr;
                        }
                    }
                    return;
                }
                mem_info = mem_info->pNext;
            }
        }
        list = list->pNext;
    }
    osalStdLog("Could not find id: %d pid: %d\n", id, pid);
    return;
}

/*
 * Clean all memory for a process
 */
void userMemFreeAllPid(user_mem_dev_t* dev, UINT32 pid)
{
    dev_mem_info_t *mem_info = NULL, *next = NULL;
    user_proc_mem_list_t *list = NULL, *l_next = NULL;

    list = dev->head;
    while(list)
    {
        l_next = list->pNext;
        if(list->pid == pid)
        {
            mem_info = list->head;
            while(mem_info)
            {
                next = mem_info->pNext;
                osalIOMMUUnmap(mem_info->phy_addr, mem_info->size);
                FREE(mem_info->kmalloc_ptr);
                REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
                FREE(mem_info);
                mem_info = next;
            }
            mem_info = list->freed_head;
            while(mem_info)
            {
                next = mem_info->pNext;
                REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail, list->freed_head);
                FREE(mem_info);
                mem_info = next;
            }
            REMOVE_ELEMENT_FROM_LIST(list, dev->tail, dev->head);
            FREE(list);
        }
        list = l_next;
    }
}

/*
 * Free page
 */
void userMemFreeAllPagePid(user_mem_dev_t* dev, UINT32 pid)
{
    dev_mem_info_t *mem_info = NULL, *next = NULL;
    user_proc_mem_list_t *list = NULL, *l_next = NULL;

    list = dev->head;
    while(list)
    {
        l_next = list->pNext;
        if(list->pid == pid)
        {
            mem_info = list->head;
            while(mem_info)
            {
                next = mem_info->pNext;
                free_page((unsigned long)mem_info->kmalloc_ptr);
                REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
                FREE(mem_info);
                mem_info = next;
            }
            mem_info = list->freed_head;
            while(mem_info)
            {
                next = mem_info->pNext;
                REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail, list->freed_head);
                FREE(mem_info);
                mem_info = next;
            }
            REMOVE_ELEMENT_FROM_LIST(list, dev->tail, dev->head);
            FREE(list);
        }
        list = l_next;
    }
}

/*
 * Free all memory
 */
void userMemFreeAll(user_mem_dev_t* dev)
{
    dev_mem_info_t *mem_info = NULL, *next = NULL;
    user_proc_mem_list_t *list = NULL, *l_next = NULL;

    list = dev->head;
    while(list)
    {
        l_next = list->pNext;
        mem_info = list->head;
        while(mem_info)
        {
            next = mem_info->pNext;
            osalIOMMUUnmap(mem_info->phy_addr, mem_info->size);
            FREE(mem_info->kmalloc_ptr);
            REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
            FREE(mem_info);
            mem_info = next;
        }
        mem_info = list->freed_head;
        while(mem_info)
        {
            next = mem_info->pNext;
            REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail, list->freed_head);
            FREE(mem_info);
            mem_info = next;
        }

        REMOVE_ELEMENT_FROM_LIST(list, dev->tail, dev->head);
        FREE(list);
        list = l_next;
    }
}

/*
 * Free all pages
 */
void userMemFreeAllPage(user_mem_dev_t* dev)
{
    dev_mem_info_t *mem_info = NULL, *next = NULL;
    user_proc_mem_list_t *list = NULL, *l_next = NULL;

    list = dev->head;
    while(list)
    {
        l_next = list->pNext;
        mem_info = list->head;
        while(mem_info)
        {
            next = mem_info->pNext;
            __free_page(mem_info->kmalloc_ptr);
            REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
            FREE(mem_info);
            mem_info = next;
        }
        mem_info = list->freed_head;
        while(mem_info)
        {
            next = mem_info->pNext;
            REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail, list->freed_head);
            FREE(mem_info);
            mem_info = next;
        }
        REMOVE_ELEMENT_FROM_LIST(list, dev->tail, dev->head);
        FREE(list);
        list = l_next;
    }
}
