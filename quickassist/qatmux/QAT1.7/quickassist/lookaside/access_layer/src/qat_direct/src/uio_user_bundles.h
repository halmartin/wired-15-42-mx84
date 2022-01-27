/***************************************************************************
 *
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
 *
 ***************************************************************************/
#ifndef UIO_USER_BUNDLES_H

#define UIO_USER_BUNDLES_H

#include "icp_accel_devices.h"
int uio_control_mutex_init(void);

struct udev_device;

struct adf_uio_user_bundle
{
    unsigned int device_minor;
    struct udev_device *udev_dev;
    int fd;
    void *ptr;
};

struct adf_uio_user_bundle *uio_get_bundle_from_accelid(int accelid,
                                                        int bundle_nr);
int uio_acces_dev_exist(int dev_id, struct udev_device **udev_dev);
int uio_create_accel_dev(icp_accel_dev_t **accel_dev, int dev_id);

void uio_destroy_accel_dev(icp_accel_dev_t *accel_dev);
void uio_free_bundle(struct adf_uio_user_bundle *bundle);

#endif /* end of include guard: UIO_USER_BUNDLES_H */
