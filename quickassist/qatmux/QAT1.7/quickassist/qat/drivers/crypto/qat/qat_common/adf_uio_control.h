/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2015 Intel Corporation.
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
  Copyright(c) 2015 Intel Corporation.
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
#ifndef QAT_UIO_CONTROL_H
#define QAT_UIO_CONTROL_H

#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/uio_driver.h>

#include "adf_uio.h"
#define UIO_MAX_NAME_LENGTH 32

struct pci_dev;

struct adf_uio_instance_rings {
	unsigned int user_pid;
	uint16_t ring_mask;
	uintptr_t vma;
	struct list_head list;
};

struct adf_uio_control_bundle {
	struct kobject kobj;
	char name[UIO_MAX_NAME_LENGTH];
	uint8_t hardware_bundle_number;
	unsigned int device_minor;
	struct list_head list;
	struct mutex list_lock; /* protects list struct */
	struct mutex lock; /* protects rings_used and csr_addr */
	uint16_t rings_used;
	void *csr_addr;
	struct uio_info uio_info;
	struct qat_uio_bundle_dev uio_priv;
};

struct adf_uio_control_accel {
	struct adf_accel_dev *accel_dev;
	struct kobject kobj;
	unsigned int nb_bundles;
	unsigned int first_minor;
	unsigned int last_minor;
	unsigned int num_ker_bundles;
	/* bundle[] must be last to allow dynamic size allocation. */
	struct adf_uio_control_bundle *bundle[0];
};

int adf_uio_sysfs_create(struct adf_accel_dev *accel_dev);
void adf_uio_sysfs_bundle_delete(struct adf_accel_dev *accel_dev,
				 unsigned bundle_num);
void adf_uio_sysfs_delete(struct adf_accel_dev *accel_dev);
int adf_uio_sysfs_bundle_create(struct pci_dev *pdev,
				unsigned bundle_num,
				struct adf_uio_control_accel *accel);

void adf_uio_accel_ref(struct adf_uio_control_accel *accel);
void adf_uio_accel_unref(struct adf_uio_control_accel *accel);
void adf_uio_bundle_ref(struct adf_uio_control_bundle *bundle);
void adf_uio_bundle_unref(struct adf_uio_control_bundle *bundle);
#endif /* end of include guard: QAT_UIO_CONTROL_H */
