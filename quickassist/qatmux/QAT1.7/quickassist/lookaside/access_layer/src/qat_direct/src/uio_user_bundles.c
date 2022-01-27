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
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <pthread.h>

#include "cpa.h"
#include "icp_platform.h"

#include "uio_user.h"
#include "adf_kernel_types.h"
#include "adf_cfg_user.h"
#include "uio_user_cfg.h"

#define MAP_INDEX 0

static inline void *uio_map_bundle_ptr(struct adf_uio_user_bundle *bundle)
{
    unsigned long size;
    unsigned long offset;
    void *bundle_ptr;

    if (CPA_STATUS_SUCCESS != uio_udev_read_long(bundle->udev_dev,
                                                 &size,
                                                 UDEV_ATTRIBUTE_MAP0_SIZE,
                                                 bundle->device_minor))
        return NULL;
    if (CPA_STATUS_SUCCESS != uio_udev_read_long(bundle->udev_dev,
                                                 &offset,
                                                 UDEV_ATTRIBUTE_MAP0_OFFSET,
                                                 bundle->device_minor))
        return NULL;

    bundle_ptr = ICP_MMAP(NULL,
                          size,
                          PROT_READ | PROT_WRITE,
                          MAP_SHARED,
                          bundle->fd,
                          MAP_INDEX * getpagesize());

    madvise(bundle_ptr, size, MADV_DONTFORK);

    if (bundle_ptr == MAP_FAILED)
    {
        perror("Mapping error");
        return NULL;
    }
    return (void *)((unsigned char *)bundle_ptr + offset);
}
/**
 * This function is using 'udev_device_get_sysattr_value()' function.
 *
 * From man pages: The retrieved value is cached in the device.
 * Repeated calls will return the same value and not open the attribute again.
 *
 * More calls of 'get_free_bundle_from_dev_cached()' without enumerating
 * of the device via 'uio_udev_get_device_from_devid()' gives incorrect values
 * and is major issue for 'used' flag! (e.g. 'uio_ctrl/bundle_0/used').
 */
static inline struct adf_uio_user_bundle *get_bundle_from_dev_cached(
    struct udev_device *dev,
    int bundle_nr)
{
    unsigned long attribute_value;
    struct adf_uio_user_bundle *bundle = NULL;
    char uio_dev_name[32];

    if (uio_udev_read_str(dev,
                          uio_dev_name,
                          sizeof(uio_dev_name),
                          UDEV_ATTRIBUTE_DEV_NAME,
                          bundle_nr) != CPA_STATUS_SUCCESS)
        return NULL;

    bundle = ICP_MALLOC_GEN(sizeof(*bundle));
    if (!bundle)
    {
        perror("failed to allocate bundle structure\n");
        return NULL;
    }
    bundle->fd = open(uio_dev_name, O_RDWR);
    if (bundle->fd < 0)
    {
        ADF_ERROR("failed to open uio dev %s\n", uio_dev_name);
        ICP_FREE(bundle);
        return NULL;
    }
    bundle->udev_dev = dev;
    uio_udev_read_long(
        dev, &attribute_value, UDEV_ATTRIBUTE_DEV_MINOR, bundle_nr);
    bundle->device_minor = attribute_value;
    ADF_DEBUG("Opened uio device %s\n", uio_dev_name);

    bundle->ptr = uio_map_bundle_ptr(bundle);
    if (NULL == bundle->ptr)
    {
        close(bundle->fd);
        ICP_FREE(bundle);
        return NULL;
    }
    return bundle;
}

struct adf_uio_user_bundle *uio_get_bundle_from_accelid(int accelid,
                                                        int bundle_nr)
{
    struct udev_device *dev;
    struct adf_uio_user_bundle *bundle;

    if (CPA_STATUS_SUCCESS != uio_udev_get_device_from_devid(accelid, &dev))
        return NULL;

    bundle = get_bundle_from_dev_cached(dev, bundle_nr);

    return bundle;
}

static void uio_free_bundle_ptr(void *bundle_ptr, unsigned long size)
{
    ICP_MUNMAP(bundle_ptr, size);
}

void uio_free_bundle(struct adf_uio_user_bundle *bundle)
{
    unsigned long size = -1;

    if (CPA_STATUS_SUCCESS != uio_udev_read_long(bundle->udev_dev,
                                                 &size,
                                                 UDEV_ATTRIBUTE_MAP0_SIZE,
                                                 bundle->device_minor))
    {
        printf("Failed to read size from sysfs attribute %s\n",
               UDEV_ATTRIBUTE_MAP0_SIZE);
    }
    else
    {
        uio_free_bundle_ptr(bundle->ptr, size);
    }

    close(bundle->fd);

    uio_udev_free_device(bundle->udev_dev);
    ICP_FREE(bundle);
}

static int uio_populate_accel_dev(struct udev_device *udev_dev,
                                  icp_accel_dev_t *accel_dev)
{
    char config_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
    unsigned long attribute_value;
    char attr_str[ADF_DEVICE_TYPE_LENGTH];

    memset(accel_dev, '\0', sizeof(*accel_dev));
    uio_udev_read_long(udev_dev, &attribute_value, UDEV_ATTRIBUTE_ACCELID);
    accel_dev->accelId = attribute_value;
    accel_dev->maxNumRingsPerBank = 16;

    /* read Maximal Number of Banks */
    if (CPA_STATUS_SUCCESS !=
        icp_adf_cfgGetParamValue(
            accel_dev, ADF_GENERAL_SEC, ADF_DEV_MAX_BANKS, config_value))
    {
        return -EINVAL;
    }
    accel_dev->maxNumBanks =
        (Cpa32U)strtoul(config_value, NULL, ADF_CFG_BASE_DEC);

    /* Get capabilities mask */
    if (CPA_STATUS_SUCCESS !=
        icp_adf_cfgGetParamValue(accel_dev,
                                 ADF_GENERAL_SEC,
                                 ADF_DEV_CAPABILITIES_MASK,
                                 config_value))
    {
        return -EINVAL;
    }
    accel_dev->accelCapabilitiesMask =
        (Cpa32U)strtoul(config_value, NULL, ADF_CFG_BASE_HEX);

    /* Get dc extended capabilities */
    if (CPA_STATUS_SUCCESS !=
        icp_adf_cfgGetParamValue(
            accel_dev, ADF_GENERAL_SEC, ADF_DC_EXTENDED_FEATURES, config_value))
    {
        return -EINVAL;
    }
    accel_dev->dcExtendedFeatures =
        (Cpa32U)strtoul(config_value, NULL, ADF_CFG_BASE_HEX);

    /* Get device type */
    uio_udev_read_long(udev_dev, &attribute_value, UDEV_ATTRIBUTE_ACCEL_TYPE);
    accel_dev->deviceType = attribute_value;

    /* Get device name */
    uio_udev_read_str(
        udev_dev, attr_str, sizeof(attr_str), UDEV_ATTRIBUTE_ACCEL_NAME);
    ICP_STRNCPY(accel_dev->deviceName, attr_str, sizeof(accel_dev->deviceName));

    /* Get device revision id */
    uio_udev_read_long(udev_dev, &attribute_value, UDEV_ATTRIBUTE_ACCEL_REVID);
    accel_dev->revisionId = attribute_value;

    uio_udev_read_long(udev_dev, &attribute_value, UDEV_ATTRIBUTE_MMP_ADDR);

    uio_udev_read_long(udev_dev, &attribute_value, UDEV_ATTRIBUTE_NODEID);
    accel_dev->numa_node = attribute_value;

    return 0;
}

int uio_acces_dev_exist(int dev_id, struct udev_device **udev_dev)
{
    struct udev_device *dev;

    if (CPA_STATUS_SUCCESS != uio_udev_get_device_from_devid(dev_id, &dev))
        return 0;
    if (udev_dev)
        *udev_dev = dev;
    else
        uio_udev_free_device(dev);

    return 1;
}

int uio_create_accel_dev(icp_accel_dev_t **accel_dev, int dev_id)
{
    CpaStatus status;
    struct udev_device *dev;

    *accel_dev = ICP_MALLOC_GEN(sizeof(**accel_dev));
    if (!*accel_dev)
        return -ENOMEM;

    if (!uio_acces_dev_exist(dev_id, &dev))
    {
        status = -EINVAL;
        goto accel_fail;
    }

    if (uio_populate_accel_dev(dev, *accel_dev))
    {
        status = -EINVAL;
        uio_udev_free_device(dev);
        goto accel_fail;
    }
    uio_udev_free_device(dev);

    return 0;

accel_fail:
    ICP_FREE(*accel_dev);
    *accel_dev = NULL;
    return status;
}

void uio_destroy_accel_dev(icp_accel_dev_t *accel_dev)
{
    ICP_FREE(accel_dev);
}
