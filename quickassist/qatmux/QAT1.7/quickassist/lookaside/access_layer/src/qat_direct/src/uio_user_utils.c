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
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <libudev.h>

#include "cpa.h"
#include "uio_user_utils.h"
#include "icp_platform.h"
#include "uio_user_cfg.h"

static const char *BDF_STRING = "%04x:%02x:%02x.%1x";

CpaStatus uio_udev_get_device_from_devid(int devid, struct udev_device **dev)
{
    struct udev *udev;
    Cpa16U bdf;
    Cpa8U bus, device, function;
    Cpa32S domain;
    char sysname[32];

    ADF_DEBUG("uio_udev_get_device_from_devid(%d, %p)\n", devid, dev);

    bdf = icp_adf_cfgGetBusAddress(devid);
    if (bdf == 0xFFFF)
    {
        ADF_DEBUG("uio_udev_get_device_from_devid: error reading the BDF "
                  "of devid %d\n",
                  devid);
        return CPA_STATUS_FAIL;
    }

    udev = udev_new();
    if (!udev)
    {
        ADF_ERROR("uio_udev_get_device_from_devid: udev_new failed\n");
        return CPA_STATUS_FAIL;
    }

    bus = bdf >> 8;
    device = (bdf >> 3) & 0x1F;
    function = bdf & 0x07;
    domain = icp_adf_cfgGetDomainAddress(devid);
    snprintf(
        sysname, sizeof(sysname), BDF_STRING, domain, bus, device, function);
    ADF_DEBUG("devid %d has bdf %u = %s\n", devid, bdf, sysname);

    *dev = udev_device_new_from_subsystem_sysname(udev, "pci", sysname);
    if (*dev)
    {
        const char *accelId =
            udev_device_get_sysattr_value(*dev, UDEV_ATTRIBUTE_ACCELID);
        ADF_DEBUG("uio_udev_get_device_from_devid: syspath for devid %d = %s\n",
                  devid,
                  udev_device_get_syspath(*dev));

        if (accelId)
        {
            return CPA_STATUS_SUCCESS;
        }
        else
        {
            ADF_DEBUG(
                "uio_udev_get_device_from_devid: no %s sysattr for devid %d\n",
                UDEV_ATTRIBUTE_ACCELID,
                devid);
            udev_device_unref(*dev);
            *dev = NULL;
        }
    }
    else
    {
        ADF_ERROR(
            "udev_device_new_from_subsystem_sysname failed for sysname %s\n",
            sysname);
    }

    udev_unref(udev);
    return CPA_STATUS_FAIL;
}

CpaStatus uio_udev_free_device(struct udev_device *dev)
{
    struct udev *udev = udev_device_get_udev(dev);
    udev_device_unref(dev);
    udev_unref(udev);
    return CPA_STATUS_SUCCESS;
}

static CpaStatus uio_udev_read_str_ap(struct udev_device *dev,
                                      char *value,
                                      unsigned size,
                                      const char *attribute,
                                      va_list ap)
{
    char attribute_name[128];
    const char *attribute_value;

    vsnprintf(attribute_name, sizeof(attribute_name), attribute, ap);

    attribute_value = udev_device_get_sysattr_value(dev, attribute_name);

    if (!attribute_value)
        return CPA_STATUS_FAIL;

    snprintf(value, size, "%s", attribute_value);
    ADF_DEBUG("uio_udev_read_str_ap. Attribute %s value %s\n",
              attribute_name,
              attribute_value);
    return CPA_STATUS_SUCCESS;
}

CpaStatus uio_udev_read_str(struct udev_device *dev,
                            char *value,
                            unsigned size,
                            const char *attribute,
                            ...)
{
    CpaStatus status;
    va_list ap;

    va_start(ap, attribute);
    status = uio_udev_read_str_ap(dev, value, size, attribute, ap);
    va_end(ap);

    return status;
}

CpaStatus uio_udev_read_long(struct udev_device *dev,
                             unsigned long *value,
                             const char *attribute,
                             ...)
{
    CpaStatus status;
    char attribute_value[32];
    va_list ap;

    va_start(ap, attribute);
    status = uio_udev_read_str_ap(
        dev, attribute_value, sizeof(attribute_value), attribute, ap);
    va_end(ap);

    if (status == CPA_STATUS_SUCCESS)
        *value = strtol(attribute_value, NULL, 0);

    return status;
}

CpaStatus uio_udev_read_uint(struct udev_device *dev,
                             unsigned int *value,
                             const char *attribute,
                             ...)
{
    CpaStatus status;
    long long_value;
    char attribute_string[32];
    va_list ap;

    va_start(ap, attribute);
    status = uio_udev_read_str_ap(
        dev, attribute_string, sizeof(attribute_string), attribute, ap);
    va_end(ap);

    if (status == CPA_STATUS_SUCCESS)
    {
        long_value = strtol(attribute_string, NULL, 0);
        *value = (unsigned int)long_value;
    }

    return status;
}
