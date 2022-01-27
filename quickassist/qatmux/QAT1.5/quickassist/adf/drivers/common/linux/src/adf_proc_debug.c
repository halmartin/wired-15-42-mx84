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
 * @file adf_proc_debug.c
 *
 * @description
 *    This file contains ADF debug feature code for setting up the
 *    user specific directory/file structure in the proc file system.
 *    Contains the procfs calls which are OS specific.
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/gfp.h>
#include <asm/page.h>
#include <linux/mutex.h>

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_debug.h"
#include "adf_proc_debug.h"

#define ADF_DEBUG_BASE_DIRECTORY     "icp_%s_dev%d"
#define ADF_DEBUG_NAME_LEN            20

STATIC DEFINE_MUTEX(proc_debug_lock);

/* Mutex lock "proc_debug_seq_lock" is used to enclose the
   start() and stop() of the seq_file sequences 
   to ensure the pointer to allocated page is not modified 
   by another instance of same file "cat /proc/files.." till the complete
   sequence is finished.
*/ 
STATIC DEFINE_MUTEX(proc_debug_seq_lock);

/*
 * adf_debug_start
 * Start of the sequence
 */
static void *adf_debug_start(struct seq_file *sfile, loff_t *pos)
{
        debug_file_info_t* file_info = sfile->private;
        mutex_lock(&proc_debug_seq_lock);
        if (*pos == 0) {
                file_info->page = (char*) get_zeroed_page(GFP_KERNEL);
                if (!file_info->page) {
                        ADF_ERROR("Can't alloc memory\n");
                        return NULL;
                }
                file_info->offset = 0;
                return file_info->page;
        }
        else if (file_info->offset != 0) {
                return file_info->page;
        }
        else {
                file_info->page = NULL;
                *pos = 0;
                return NULL;
        }
}

/*
 * adf_debug_show
 * Read step of the sequence
 */
static int adf_debug_show(struct seq_file *sfile, void *v)
{
        debug_file_info_t* file_info = sfile->private;
        if (file_info && file_info->seq_read && file_info->page) {
                int ret = 0, old_offset = file_info->offset;
                file_info->offset =
                                   file_info->seq_read(file_info->private_data,
                                   file_info->page, PAGE_SIZE - 1,
                                   file_info->offset);
                ret = seq_puts(sfile, (char*)file_info->page);
                if (ret) {
                        /* run out of space - need to reprint */
                        file_info->offset = old_offset;
                }
        }
        return 0;
}

/*
 * adf_debug_next
 * Next step of the sequence
 */
static void *adf_debug_next(struct seq_file *sfile, void *v, loff_t *pos)
{
        debug_file_info_t* file_info = sfile->private;
        if (file_info->offset == 0) {
                return NULL;
        }
        if(file_info->page)
        {
            ICP_MEMSET(file_info->page, '\0', PAGE_SIZE);
        }
        return file_info->page;
}

/*
 * adf_debug_stop
 * Stop of the sequence
 */
static void adf_debug_stop(struct seq_file *sfile, void *v)
{
        debug_file_info_t* file_info = sfile->private;
        if ((file_info->offset == 0) && file_info->page) {
                free_page((unsigned long)file_info->page);
                file_info->page = NULL;
        }
        mutex_unlock(&proc_debug_seq_lock);
}

/*
 * adf_debug_ops
 * Sequence operations for the seq proc files
 */
static struct seq_operations adf_debug_ops = {
        .start = adf_debug_start,
        .next  = adf_debug_next,
        .stop  = adf_debug_stop,
        .show  = adf_debug_show
};

/*
 * adf_debug_open
 * Open function for the seq proc files
 */
static int adf_debug_open(struct inode *inode, struct file *file)
{
        int ret = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))
        struct proc_dir_entry *entry = NULL;
#endif
        mutex_lock(&proc_debug_lock);
        ret = seq_open(file, &adf_debug_ops);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))
        if (!ret) {
                entry = PDE(inode);
                ((struct seq_file *)file->private_data)->private = entry->data;
        }
#else
        if (!ret) {
                 ((struct seq_file *)file->private_data)->private = PDE_DATA(inode);
        }
#endif
        mutex_unlock(&proc_debug_lock);
        return ret;
}

/*
 * adf_debug_file_ops
 * File operations for the seq proc files
 */
static struct file_operations adf_debug_file_ops = {
        .owner   = THIS_MODULE,
        .open    = adf_debug_open,
        .read    = seq_read,
        .llseek  = seq_lseek,
        .release = seq_release
};

/*
 * adf_debugInit
 * Function initialises the debug feature for the specified device
 *
 */
CpaStatus adf_debugInit(icp_accel_dev_t *accel_dev)
{
        debug_dir_info_t *base_dir = NULL;
        adf_hw_device_data_t *hw_data = NULL;

        ICP_CHECK_FOR_NULL_PARAM(accel_dev);
        ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);

        mutex_lock(&proc_debug_lock);
        base_dir = ICP_ZALLOC_GEN(sizeof(debug_dir_info_t));
        if (NULL == base_dir) {
                ADF_ERROR("Memory allocation failed for base_dir\n");
                mutex_unlock(&proc_debug_lock);
                return CPA_STATUS_FAIL;
        }
        base_dir->name = ICP_ZALLOC_GEN(ADF_DEBUG_NAME_LEN);
        if (NULL == base_dir->name) {
                ADF_ERROR("Memory allocation failed for base_dir->name\n");
                ICP_FREE(base_dir);
                mutex_unlock(&proc_debug_lock);
                return CPA_STATUS_FAIL;
        }

        hw_data = accel_dev->pHwDeviceData;
        sprintf(base_dir->name, ADF_DEBUG_BASE_DIRECTORY,
                        hw_data->dev_class->name, hw_data->instanceId);

        base_dir->proc_entry = proc_mkdir(base_dir->name, NULL);
        if (NULL == base_dir->proc_entry) {
                ADF_ERROR("Could not create directory %s in proc filesystem",
                                                        base_dir->name);
                ICP_FREE(base_dir->name);
                ICP_FREE(base_dir);
                mutex_unlock(&proc_debug_lock);
                return CPA_STATUS_FAIL;
        }
        accel_dev->base_dir = base_dir;
        mutex_unlock(&proc_debug_lock);
        return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_debugAddDir
 * Function creates a debug directory in the proc filesystem
 */
CpaStatus icp_adf_debugAddDir(icp_accel_dev_t *accel_dev,
                                               debug_dir_info_t* dir_info)
{
        CpaStatus status = CPA_STATUS_SUCCESS;

        if (NULL == accel_dev || NULL == dir_info || NULL == dir_info->name) {
                ADF_ERROR("Invalid param\n");
                return CPA_STATUS_INVALID_PARAM;
        }
        mutex_lock(&proc_debug_lock);
        if (NULL == dir_info->parent) {
                dir_info->parent = accel_dev->base_dir;
        }

        dir_info->dirChildListHead = NULL;
        dir_info->dirChildListTail = NULL;
        dir_info->pNext = NULL;
        dir_info->pPrev = NULL;
        dir_info->fileListHead = NULL;
        dir_info->fileListTail = NULL;

        dir_info->proc_entry = proc_mkdir(dir_info->name,
                                         dir_info->parent->proc_entry);

        if (NULL == dir_info->proc_entry) {
                ADF_ERROR("Could not create directory %s in proc filesystem\n",
                                                               dir_info->name);
                mutex_unlock(&proc_debug_lock);
                return CPA_STATUS_FAIL;
        }

        /*Maintain the parent data & file list internally */
        ICP_ADD_ELEMENT_TO_END_OF_LIST(dir_info,
                                   dir_info->parent->dirChildListTail,
                                   dir_info->parent->dirChildListHead);
        mutex_unlock(&proc_debug_lock);
        return status;
}

/*
 * icp_adf_debugAddFile
 * Function creates a debug file under the proc filesystem.
 */
CpaStatus icp_adf_debugAddFile(icp_accel_dev_t *accel_dev,
                               debug_file_info_t* file_info)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        struct proc_dir_entry *entry = NULL;

        if (NULL == accel_dev || NULL == file_info || NULL == file_info->name) {
                ADF_ERROR("Invalid param\n");
                return CPA_STATUS_INVALID_PARAM;
        }
        mutex_lock(&proc_debug_lock);
        if(NULL == file_info->parent) {
                file_info->parent = accel_dev->base_dir;
        }

        file_info->pNext = NULL;
        file_info->pPrev = NULL;
        if (NULL == file_info->seq_read) {
                ADF_ERROR("Invalid param seq_read handler\n");
                mutex_unlock(&proc_debug_lock);
                return CPA_STATUS_INVALID_PARAM;
        }

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))
        entry = create_proc_entry(file_info->name,
                                  ADF_PROC_FILE_PERM,
                                  file_info->parent->proc_entry);
        if (entry) {
                entry->proc_fops = &adf_debug_file_ops;
                entry->data = (void*)file_info;
        }
#else
        entry = proc_create_data(file_info->name,
                                 ADF_PROC_FILE_PERM,
                                 file_info->parent->proc_entry,
                                 &adf_debug_file_ops, (void*) file_info);
#endif
        file_info->proc_entry = entry;
        if (NULL == file_info->proc_entry) {
                ADF_ERROR("Failed to create proc file: %s\n", file_info->name);
                mutex_unlock(&proc_debug_lock);
                return CPA_STATUS_FAIL;
        }
        /*Maintain the parent data & file list internally */
        ICP_ADD_ELEMENT_TO_END_OF_LIST(file_info,
                                             file_info->parent->fileListTail,
                                             file_info->parent->fileListHead);
        mutex_unlock(&proc_debug_lock);
        return status;
}

/*
 * icp_adf_debugRemoveFile
 * Function removes a debug file from the proc filesystem
 *
 */
void icp_adf_debugRemoveFile(debug_file_info_t* file_info)
{

        ICP_CHECK_FOR_NULL_PARAM_VOID(file_info);
        ICP_CHECK_FOR_NULL_PARAM_VOID(file_info->parent);
        ICP_CHECK_FOR_NULL_PARAM_VOID(file_info->name);

        mutex_lock(&proc_debug_lock);
        ICP_REMOVE_ELEMENT_FROM_LIST(file_info,
                                            file_info->parent->fileListTail,
                                            file_info->parent->fileListHead);
        mutex_unlock(&proc_debug_lock);
        /* fn always returns void */
        remove_proc_entry(file_info->name, file_info->parent->proc_entry);
}

/*
 * icp_adf_debugRemoveDir
 * Function removes a debug dir from the proc filesystem, and also
 * removes any sub directories/files.
 *
 */
void icp_adf_debugRemoveDir(debug_dir_info_t* dir_info)
{
        debug_file_info_t *file = NULL;
        debug_dir_info_t *dir = NULL;

        ICP_CHECK_FOR_NULL_PARAM_VOID(dir_info);
        ICP_CHECK_FOR_NULL_PARAM_VOID(dir_info->parent);
        ICP_CHECK_FOR_NULL_PARAM_VOID(dir_info->name);

        /* First remove all files from the dir */
        file = dir_info->fileListHead;
        while(file)
        {
                icp_adf_debugRemoveFile(file);
                mutex_lock(&proc_debug_lock);
                ICP_REMOVE_ELEMENT_FROM_LIST(file,
                                           file->parent->fileListTail,
                                           file->parent->fileListHead);
                mutex_unlock(&proc_debug_lock);
                file = file->pNext;
        }
        /* then remove all dirs under the dir */
        dir = dir_info->dirChildListHead;
        while(dir)
        {
                mutex_lock(&proc_debug_lock);
                ICP_REMOVE_ELEMENT_FROM_LIST(dir,
                                          dir->parent->dirChildListTail,
                                          dir->parent->dirChildListHead);
                mutex_unlock(&proc_debug_lock);
                dir = dir->pNext;
        }

        /* fn always returns void */
        remove_proc_entry(dir_info->name, dir_info->parent->proc_entry);
}

/*
 * adf_debugShutdown
 * Function shuts down the debug feature for the specified device
 */
void adf_debugShutdown(icp_accel_dev_t *accel_dev)
{
        debug_dir_info_t *base_dir = NULL;
        if (accel_dev && accel_dev->base_dir) {
                base_dir = accel_dev->base_dir;
                accel_dev->base_dir = NULL;
                remove_proc_entry(base_dir->name, NULL);
                ICP_FREE(base_dir->name);
                ICP_FREE(base_dir);
        }
}
