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
#ifndef UIO_USER_UTILS_H

#define UIO_USER_UTILS_H

#define UIO_IS_ACCEL_STARTED(bit, status) (status & (0x1UL << bit))

#define UIO_SYS_NAME "name"
#define UIO_SYS_VERSION "version"
#define UIO_SYS_MAP_NAME "maps/map0/name"
#define UIO_SYS_MAP_SIZE "maps/map0/size"
#define UIO_SYS_MAP_OFFSET "maps/map0/offset"
#define UIO_SYS_MAP_ADDR "maps/map0/addr"
#define UIO_MAX_DIR_NAME_LENGTH 64

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include "uio_user.h"

#define UDEV_DEVICE_UIOCTL_BASE "uio_ctrl/"
#define UDEV_DEVICE_UIO_BASE "uio/"
#define UDEV_ATTRIBUTE_ACCELID UDEV_DEVICE_UIOCTL_BASE "accelid"
#define UDEV_ATTRIBUTE_FIRST_MINOR UDEV_DEVICE_UIOCTL_BASE "first_minor"
#define UDEV_ATTRIBUTE_LAST_MINOR UDEV_DEVICE_UIOCTL_BASE "last_minor"
#define UDEV_ATTRIBUTE_ACCEL_TYPE UDEV_DEVICE_UIOCTL_BASE "type"
#define UDEV_ATTRIBUTE_ACCEL_NAME UDEV_DEVICE_UIOCTL_BASE "name"
#define UDEV_ATTRIBUTE_ACCEL_REVID UDEV_DEVICE_UIOCTL_BASE "revid"
#define UDEV_ATTRIBUTE_MMP_ADDR UDEV_DEVICE_UIOCTL_BASE "mmp_addr"
#define UDEV_ATTRIBUTE_NUM_BUNDLES UDEV_DEVICE_UIOCTL_BASE "num_bundles"
#define UDEV_ATTRIBUTE_DEV_MINOR UDEV_DEVICE_UIOCTL_BASE "bundle_%d/dev_minor"
#define UDEV_ATTRIBUTE_DEV_NAME UDEV_DEVICE_UIOCTL_BASE "bundle_%d/dev_name"
#define UDEV_ATTRIBUTE_HARDWARE_BUNDLE                                         \
    UDEV_DEVICE_UIOCTL_BASE "bundle_%d/hardware_bundle"
#define UDEV_ATTRIBUTE_MAP0_ADDR UDEV_DEVICE_UIO_BASE "uio%d/maps/map0/addr"
#define UDEV_ATTRIBUTE_MAP0_NAME UDEV_DEVICE_UIO_BASE "uio%d/maps/map0/name"
#define UDEV_ATTRIBUTE_MAP0_OFFSET UDEV_DEVICE_UIO_BASE "uio%d/maps/map0/offset"
#define UDEV_ATTRIBUTE_MAP0_SIZE UDEV_DEVICE_UIO_BASE "uio%d/maps/map0/size"
#define UDEV_ATTRIBUTE_NODEID "numa_node"

struct udev_device;

CpaStatus uio_udev_get_device_from_devid(int devid, struct udev_device **dev);
CpaStatus uio_udev_free_device(struct udev_device *dev);
CpaStatus uio_udev_read_str(struct udev_device *dev,
                            char *value,
                            unsigned size,
                            const char *attribute,
                            ...);
CpaStatus uio_udev_read_long(struct udev_device *dev,
                             unsigned long *value,
                             const char *attribute,
                             ...);
CpaStatus uio_udev_read_uint(struct udev_device *dev,
                             unsigned int *value,
                             const char *attribute,
                             ...);

#endif /* UIO_USER_UTILS_H */
