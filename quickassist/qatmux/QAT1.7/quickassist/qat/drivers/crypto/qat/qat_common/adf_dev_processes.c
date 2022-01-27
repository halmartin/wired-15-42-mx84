/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2014 Intel Corporation.
  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  qat-linux@intel.com

  BSD LICENSE
  Copyright(c) 2014 Intel Corporation.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include "adf_accel_devices.h"
#include "adf_common_drv.h"
#include "adf_cfg.h"

#define ADF_DEV_PROCESSES_MAX_MINOR	(255)
#define ADF_DEV_PROCESSES_BASE_MINOR       (0)

#define ADF_DEV_PROCESSES_NAME "qat_dev_processes"

static int adf_processes_open(struct inode *inp, struct file *fp);
static int adf_processes_release(struct inode *inp, struct file *fp);

static ssize_t adf_processes_read(struct file *fp,
				  char __user *buf,
				  size_t count,
				  loff_t *pos);

static ssize_t adf_processes_write(struct file *fp,
				   const char __user *buf,
				   size_t count, loff_t *pos);

struct adf_processes_priv_data {
	char   name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES];
	int    read_flag;
	struct list_head list;
};

struct adf_chr_drv_info {
	struct module       *owner;
	unsigned	    major;
	unsigned	    min_minor;
	unsigned	    max_minor;
	char		*name;
	const struct file_operations *file_ops;
	struct cdev     drv_cdev;
	struct class    *drv_class;
	unsigned	num_devices;
};

static const struct file_operations adf_processes_ops = {
	.owner = THIS_MODULE,
	.open = adf_processes_open,
	.release = adf_processes_release,
	.read  = adf_processes_read,
	.write = adf_processes_write,
};

static struct adf_chr_drv_info adf_processes_drv_info = {
	.owner = THIS_MODULE,
	.major = 0,
	.min_minor = ADF_DEV_PROCESSES_BASE_MINOR,
	.max_minor = ADF_DEV_PROCESSES_MAX_MINOR,
	.name = ADF_DEV_PROCESSES_NAME,
	.file_ops = &adf_processes_ops,
};

static LIST_HEAD(processes_list);
static DEFINE_SEMAPHORE(processes_list_sema);

static void adf_chr_drv_destroy(void)
{
	device_destroy(adf_processes_drv_info.drv_class,
		       MKDEV(adf_processes_drv_info.major, 0));
	cdev_del(&adf_processes_drv_info.drv_cdev);
	class_destroy(adf_processes_drv_info.drv_class);
	unregister_chrdev_region(MKDEV(
				 adf_processes_drv_info.major, 0), 1);
}

static int adf_chr_drv_create(void)
{
	dev_t dev_id;
	struct device *drv_device;

	if (alloc_chrdev_region(&dev_id, 0, 1, ADF_DEV_PROCESSES_NAME)) {
		pr_err("QAT: unable to allocate chrdev region\n");
		return -EFAULT;
	}

	adf_processes_drv_info.drv_class =
		class_create(THIS_MODULE, ADF_DEV_PROCESSES_NAME);
	if (IS_ERR(adf_processes_drv_info.drv_class)) {
		pr_err("QAT: class_create failed for adf_ctl\n");
		goto err_chrdev_unreg;
	}
	adf_processes_drv_info.major = MAJOR(dev_id);
	cdev_init(&adf_processes_drv_info.drv_cdev, &adf_processes_ops);
	if (cdev_add(&adf_processes_drv_info.drv_cdev, dev_id, 1)) {
		pr_err("QAT: cdev add failed\n");
		goto err_class_destr;
	}

	drv_device = device_create(adf_processes_drv_info.drv_class, NULL,
				   MKDEV(adf_processes_drv_info.major, 0),
				   NULL, ADF_DEV_PROCESSES_NAME);
	if (IS_ERR(drv_device)) {
		pr_err("QAT: failed to create device\n");
		goto err_cdev_del;
	}
	return 0;
err_cdev_del:
	cdev_del(&adf_processes_drv_info.drv_cdev);
err_class_destr:
	class_destroy(adf_processes_drv_info.drv_class);
err_chrdev_unreg:
	unregister_chrdev_region(dev_id, 1);
	return -EFAULT;
}

static int adf_processes_open(struct inode *inp, struct file *fp)
{
	int i = 0, devices = 0;
	struct adf_accel_dev *accel_dev = NULL;
	struct adf_processes_priv_data *prv_data = NULL;

	for (i = 0; i < ADF_MAX_DEVICES; i++) {
		accel_dev = adf_devmgr_get_dev_by_id(i);
		if (!accel_dev)
			continue;
		if (!adf_dev_started(accel_dev))
			continue;
		devices++;
	}
	if (!devices) {
		pr_err("QAT: Device not yet ready.\n");
		return -EACCES;
	}
	prv_data = kzalloc(sizeof(*prv_data), GFP_KERNEL);
	if (!prv_data)
		return -ENOMEM;
	INIT_LIST_HEAD(&prv_data->list);
	fp->private_data = prv_data;

	return 0;
}

static struct adf_accel_dev *adf_get_first_started_dev(void)
{
	int i = 0;
	struct adf_accel_dev *accel_dev = NULL;

	for (i = 0; i < ADF_MAX_DEVICES; i++) {
		accel_dev = adf_devmgr_get_dev_by_id(i);
		if (!accel_dev)
			continue;
		if (adf_dev_started(accel_dev) && (accel_dev->is_vf ||
		   (!accel_dev->is_vf && !accel_dev->pf.vf_info)))
			return accel_dev;
	}

	return NULL;
}

static int adf_sec_name_present_in_config_files(const char *sec)
{
	struct adf_accel_dev *accel_dev = NULL;
	u32 num_accel_devs = 0;
	int i;

	adf_devmgr_get_num_dev(&num_accel_devs);

	for (i = 0; i < num_accel_devs; i++) {
		accel_dev = adf_devmgr_get_dev_by_id(i);
		if (accel_dev && adf_cfg_sec_find(accel_dev, sec))
			return 1;
	}
	return 0;
}

static ssize_t adf_processes_write(struct file *fp, const char __user *buf,
				   size_t count, loff_t *pos)
{
	struct adf_processes_priv_data *prv_data = NULL;
	struct adf_processes_priv_data *pdata = NULL;
	int dev_num = 0, pr_num = 0;
	struct list_head *lpos = NULL;
	char usr_name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
	char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
	struct adf_accel_dev *accel_dev = NULL;
	struct adf_cfg_section *section_ptr = NULL;
	bool pr_name_available = 1;
	uint32_t num_accel_devs = 0;
	unsigned int dev_access_limit = 0;

	if (!fp || !fp->private_data) {
		pr_err("QAT: invalid file descriptor\n");
		return -EBADF;
	}

	prv_data = (struct adf_processes_priv_data *)fp->private_data;
	if (prv_data->read_flag == 1) {
		pr_err("QAT: can only write once\n");
		return -EBADF;
	}
	if (!count || count >= ADF_CFG_MAX_SECTION_LEN_IN_BYTES) {
		pr_err("QAT: wrong size %d\n", (int)count);
		return -EIO;
	}

	if (copy_from_user(usr_name, buf, count)) {
		pr_err("QAT: can't copy data\n");
		return -EIO;
	}

	/* Lock other processes and try to find out the process name */
	if (down_interruptible(&processes_list_sema)) {
		pr_err("QAT: can't aquire process info lock\n");
		return -EBADF;
	}

	accel_dev = adf_get_first_started_dev();
	if (!accel_dev) {
		pr_err("QAT: could not find started device\n");
		up(&processes_list_sema);
		return -EIO;
	}

	if (!adf_sec_name_present_in_config_files(usr_name)) {
		pr_err("QAT: could not find %s section in any config files\n",
		       usr_name);
		up(&processes_list_sema);
		return -EINVAL;
	}

	if (adf_cfg_get_param_value(accel_dev, usr_name,
				    ADF_LIMIT_DEV_ACCESS, val))
		dev_access_limit = 0;
	else
		if (kstrtouint(val, 10, &dev_access_limit))
			dev_access_limit = 0;

	/* If there is nothing there then take the first name and return */
	if (list_empty(&processes_list)) {
		if (dev_access_limit) {
			snprintf(prv_data->name,
				 ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
				 "%s_DEV%d"ADF_INTERNAL_USERSPACE_SEC_SUFF"%d",
				 usr_name, accel_dev->accel_id, 0);
		} else {
			snprintf(prv_data->name,
				 ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
				 "%s"ADF_INTERNAL_USERSPACE_SEC_SUFF"%d",
				 usr_name, 0);
		}
		list_add(&prv_data->list, &processes_list);
		up(&processes_list_sema);
		prv_data->read_flag = 1;
		return 0;
	}

	/* If there are processes running then search for a first free name */
	adf_devmgr_get_num_dev(&num_accel_devs);
	for (dev_num = 0; dev_num < num_accel_devs; dev_num++) {
		accel_dev = adf_devmgr_get_dev_by_id(dev_num);
		if (!accel_dev)
			continue;

		if (!adf_dev_started(accel_dev))
			continue; /* to next device */

		if (adf_cfg_get_param_value(accel_dev, usr_name,
					    ADF_LIMIT_DEV_ACCESS, val))
			dev_access_limit = 0;
		else
			if (kstrtouint(val, 10, &dev_access_limit))
				dev_access_limit = 0;

		/* one device can support up to GET_MAX_PROCESSES processes */
		for (pr_num = 0;
		     pr_num < GET_MAX_PROCESSES(accel_dev);
		     pr_num++) {
			/* figure out name */
			if (dev_access_limit) {
				snprintf(prv_data->name,
					 ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
					 "%s_DEV%d"ADF_INTERNAL_USERSPACE_SEC_SUFF"%d",
					 usr_name, dev_num, pr_num);
			} else {
				snprintf(prv_data->name,
					 ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
					 "%s"ADF_INTERNAL_USERSPACE_SEC_SUFF"%d",
					 usr_name, pr_num);
			}
			pr_name_available = 1;
			/* Figure out if section exists in the config table */
			section_ptr = adf_cfg_sec_find(accel_dev,
						       prv_data->name);
			if (NULL == section_ptr) {
				/* This section name doesn't exist */
				pr_name_available = 0;
				/* As process_num enumerates from 0, once we get
				 * to one which doesn't exist no further ones
				 * will exist. On to next device
				 */
				break;
			}
			/* Figure out if it's been taken already */
			list_for_each(lpos, &processes_list) {
				pdata = list_entry(lpos,
						   struct adf_processes_priv_data,
						   list);
				if (!strncmp(pdata->name, prv_data->name,
					     ADF_CFG_MAX_SECTION_LEN_IN_BYTES)) {
					pr_name_available = 0;
					break;
				}
			}
			if (pr_name_available)
				break;
		}
		if (pr_name_available)
			break;
	}
	/*
	 * If we have a valid name that is not on
	 * the list take it and add to the list
	 */
	if (pr_name_available) {
		list_add(&prv_data->list, &processes_list);
		up(&processes_list_sema);
		prv_data->read_flag = 1;
		return 0;
	}
	/* If not then the process needs to wait */
	up(&processes_list_sema);
	memset(prv_data->name, '\0', sizeof(prv_data->name));
	prv_data->read_flag = 0;
	return 1;
}

static ssize_t adf_processes_read(struct file *fp, char __user *buf,
				  size_t count, loff_t *pos)
{
	struct adf_processes_priv_data *prv_data = NULL;

	if (!fp || !fp->private_data) {
		pr_err("QAT: invalid file descriptor\n");
		return -EBADF;
	}
	prv_data = (struct adf_processes_priv_data *)fp->private_data;

	/*
	 * If there is a name that the process can use then give it
	 * to the proocess.
	 */
	if (prv_data->read_flag) {
		if (copy_to_user(buf, prv_data->name,
				 strnlen(prv_data->name,
					 sizeof(prv_data->name)))) {
			pr_err("QAT: failed to copy data to user\n");
			return -EIO;
		}
		return 0;
	}

	return -EIO;
}

static int adf_processes_release(struct inode *inp, struct file *fp)
{
	struct adf_processes_priv_data *prv_data = NULL;

	if (!fp || !fp->private_data) {
		pr_err("QAT: invalid file descriptor\n");
		return -EBADF;
	}
	prv_data = (struct adf_processes_priv_data *)fp->private_data;
	down(&processes_list_sema);
	list_del(&prv_data->list);
	up(&processes_list_sema);
	kfree(fp->private_data);
	return 0;
}

int adf_processes_dev_register(void)
{
	return adf_chr_drv_create();
}

void adf_processes_dev_unregister(void)
{
	adf_chr_drv_destroy();
}
