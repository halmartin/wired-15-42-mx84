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
 * @file adf_gige_wd_drv.c
 *
 * @description
 *      This file is an interface for the gigE watchdog running in user space.
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "adf_drv.h"
#include "icp_platform.h"
#include "adf_chr_drv.h"
#include "adf_platform.h"
#include <linux/mutex.h>
#include <linux/completion.h>
/* Character Device Driver Name */
#define DEVICE_NAME                     "icp_adf_gige_wd"
#define COMPLETION_TIME                 5000

static DEFINE_MUTEX(adf_gige_lock);
static wait_queue_head_t read_wait_queue_head;
static DECLARE_COMPLETION(write_wait_completion);
static icp_accel_dev_t *accel_dev = NULL;
static int opened = 0;

/* Increase module usage counter function */
extern int adf_get_module(void);

/* Decrease module usage counter function */
extern void adf_put_module(void);

static ssize_t adf_gige_wd_read(struct file *f, char __user *buf, size_t s,
                         loff_t *p)
{
        size_t ret = 0;
        Cpa32U bus_nr = 0;
        DECLARE_WAITQUEUE(wait_queue, current);

        add_wait_queue(&read_wait_queue_head, &wait_queue);
        do {
                set_current_state(TASK_INTERRUPTIBLE);
                if (NULL != accel_dev) {
                        bus_nr = accel_dev->pciAccelDev.pciDomain.bus;
                        ret = copy_to_user(buf, (char*)&bus_nr, sizeof(Cpa32U));
                        if (ret) {
                                ADF_ERROR("Copy to user failed.\n");
                                ret = -EFAULT;
                                break;
                        }
                        ret = sizeof(Cpa32U);
                        break;
                }
                if (signal_pending(current)) {
                       ret = -EAGAIN;
                       break;
                }
                schedule();
        } while(1);
        set_current_state(TASK_RUNNING);
        remove_wait_queue(&read_wait_queue_head, &wait_queue);
        return ret;
}

static ssize_t adf_gige_wd_write(struct file *f, const char __user *buf,
                          size_t s, loff_t *p)
{
        Cpa32U bus = 0;
        int ret = 0;
        ret = copy_from_user((char*)&bus, buf, sizeof(Cpa32U));
        if (ret) {
                ADF_ERROR("Copy from user failed.\n");
                accel_dev = NULL;
                return -EFAULT;
        }
        if (NULL != accel_dev && accel_dev->pciAccelDev.pciDomain.bus == bus) {
                complete(&write_wait_completion);
        }
        accel_dev = NULL;
        return 0;
}

static int adf_gige_wd_open(struct inode *i, struct file *f)
{
        int ret;
        mutex_lock(&adf_gige_lock);
        if (opened) {
                mutex_unlock(&adf_gige_lock);
                ADF_ERROR("Only one watchdog app can be started\n");
                return -EBUSY;
        }
        ret = adf_get_module();
        if (!ret) {
                ADF_ERROR("Get module failed\n");
                ret = -EFAULT;
        } else {
                opened = 1;
                ret = 0;
        }
        mutex_unlock(&adf_gige_lock);
        return ret;
}

static int adf_gige_wd_release(struct inode *i, struct file *f)
{
        mutex_lock(&adf_gige_lock);
        opened = 0;
        mutex_unlock(&adf_gige_lock);
        adf_put_module();
        return 0;
}

static struct file_operations adf_gige_ops = {
        owner:THIS_MODULE,
        read:adf_gige_wd_read,
        write:adf_gige_wd_write,
        open:adf_gige_wd_open,
        release:adf_gige_wd_release,
};

static adf_chr_drv_info_t adf_gige_drv_info = {
        owner:THIS_MODULE,
        major:0,
        min_minor:0,
        max_minor:1,
        name:DEVICE_NAME,
        file_ops:&adf_gige_ops,
};

int adf_gige_notify_restarting_dev(icp_accel_dev_t *dev)
{
        int ret = 0;
        if(!opened)
                return GIGE_WATCHDOG_NOT_RUNNING;
        mutex_lock(&adf_gige_lock);
        accel_dev = dev;
        wake_up_interruptible(&read_wait_queue_head);
        ret = wait_for_completion_timeout(&write_wait_completion,
                                    get_jiffies_64()
                                    + msecs_to_jiffies(COMPLETION_TIME));
        accel_dev = NULL;
        mutex_unlock(&adf_gige_lock);
        return (0 != ret) ? 0 : GIGE_WATCHDOG_NOT_RUNNING;
}

/*
 * register_adf_gige_wd_device_driver
 *
 * Function which dynamically allocates the major number, creates the gige wd
 * char device driver and adds it to the system.
 */
int register_adf_gige_wd_device_driver(void)
{
        int ret = 0;

        /*
         * Create a mutex to be used to guard the device driver
         * ioctl function.
         */
        mutex_init(&adf_gige_lock);
        ret = adf_chr_drv_create(&adf_gige_drv_info);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create gige wd device driver\n");
                return FAIL;
        }
        ret = adf_chr_drv_create_device(&adf_gige_drv_info,
                                        0, DEVICE_NAME);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create gige wd device\n");
                return FAIL;
        }
        init_waitqueue_head(&read_wait_queue_head);
        return SUCCESS;
}

/*
 * unregister_adf_gige_wd_device_driver
 *
 * Function which removes the gige wd char device from the system, deallocates
 * the major number.
 */
void unregister_adf_gige_wd_device_driver(void)
{
        adf_chr_drv_destroy(&adf_gige_drv_info);
}
