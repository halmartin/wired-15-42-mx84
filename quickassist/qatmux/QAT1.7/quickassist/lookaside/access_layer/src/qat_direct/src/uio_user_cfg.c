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
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#include "cpa.h"
#include "adf_kernel_types.h"
#include "adf_cfg_user.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "uio_user_cfg.h"

static const char *qat_ctl_file = ADF_CTL_DEVICE_NAME;

/*
 * Open kernel driver interface
 */
static int open_dev()
{
    int file_desc = -1;

    file_desc = open(qat_ctl_file, O_RDWR);
    if (file_desc < 0)
    {
        ADF_ERROR("Error: Failed to open device %s\n", qat_ctl_file);
    }
    return file_desc;
}

/*
 * Close kernel driver interface
 */
static void close_dev(int fd)
{
    close(fd);
}


/*
 * icp_adf_cfgGetParamValue
 * This function is used to determine the value configured for the
 * given parameter name.
 */
CpaStatus icp_adf_cfgGetParamValue(icp_accel_dev_t *accel_dev,
                                   const char *pSection,
                                   const char *pParamName,
                                   char *pParamValue)
{
    CpaStatus status = CPA_STATUS_FAIL;
    struct adf_user_cfg_ctl_data config = {{0}};
    struct adf_user_cfg_key_val kval = {{0}};
    struct adf_user_cfg_section section = {{0}};

    int fd = -1;
    int res = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pSection);
    ICP_CHECK_FOR_NULL_PARAM(pParamName);
    ICP_CHECK_FOR_NULL_PARAM(pParamValue);

    /* do ioctl to get the data */
    fd = open_dev();
    if (fd < 0)
    {
        return CPA_STATUS_FAIL;
    }

    config.device_id = accel_dev->accelId;
    config.config_section = &section;
    snprintf(section.name, ADF_CFG_MAX_SECTION_LEN_IN_BYTES, "%s", pSection);
    section.params = &kval;
    snprintf(kval.key, ADF_CFG_MAX_KEY_LEN_IN_BYTES, "%s", pParamName);

    /* send the request down to get the configuration
     * information from kernel space. */
    res = ioctl(fd, IOCTL_GET_CFG_VAL, &config);
    if (!res)
    {
        snprintf(pParamValue, ADF_CFG_MAX_VAL_LEN_IN_BYTES, "%s", kval.val);
        status = CPA_STATUS_SUCCESS;
    }
    close_dev(fd);

    return status;
}

Cpa32S icp_adf_cfgGetDomainAddress(Cpa16U packageId)
{
    struct adf_dev_status_info dev_info = {0};
    int fd = open_dev();
    int domain = -1;

    if (fd < 0)
        return domain;

    dev_info.accel_id = packageId;
    if (!ioctl(fd, IOCTL_STATUS_ACCEL_DEV, &dev_info))
    {
        domain = dev_info.domain;
    }

    close_dev(fd);

    return domain;
}

Cpa16U icp_adf_cfgGetBusAddress(Cpa16U packageId)
{
    struct adf_dev_status_info dev_info = {0};
    Cpa16U bdf = 0xFFFF;
    int fd = open_dev();

    if (fd < 0)
        return bdf;

    dev_info.accel_id = packageId;
    if (!ioctl(fd, IOCTL_STATUS_ACCEL_DEV, &dev_info))
    {
        /* Device bus address (B.D.F)
         * Bit 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
         *    |-BUS-------------------|-DEVICE-------|-FUNCT--|
         */
        bdf = dev_info.fun & 0x07;
        bdf |= (dev_info.dev & 0x1F) << 3;
        bdf |= (dev_info.bus & 0xFF) << 8;
    }

    close_dev(fd);

    return bdf;
}

Cpa32U icp_adf_cfgGetKptAcHandle(Cpa16U packageId)
{
    struct adf_dev_status_info dev_info = {0};
    Cpa32U kpt_achandle = 0;
    int fd = open_dev();

    if (fd < 0)
    {
        ADF_ERROR("Failed to open device fd = %d\n", fd);
        return kpt_achandle;
    }

    dev_info.accel_id = packageId;
    if (!ioctl(fd, IOCTL_STATUS_ACCEL_DEV, &dev_info))
    {
        kpt_achandle = dev_info.kpt_achandle;
    }

    close_dev(fd);

    return kpt_achandle;
}

enum ring_ioctl_ops
{
    RING_OP_RESERVE,
    RING_OP_RELEASE,
    RING_OP_ENABLE,
    RING_OP_DISABLE
};

STATIC CpaStatus ring_ioctl(Cpa16U accel_id,
                            Cpa16U bank_nr,
                            Cpa16U ring_nr,
                            enum ring_ioctl_ops op)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    struct adf_user_reserve_ring reserve;
    int fd = open_dev();

    if (fd < 0)
        return CPA_STATUS_FAIL;

    reserve.accel_id = accel_id;
    reserve.bank_nr = bank_nr;
    reserve.ring_mask = 1 << ring_nr;

    switch (op)
    {
        case RING_OP_RESERVE:
            if (ioctl(fd, IOCTL_RESERVE_RING, &reserve) < 0)
                status = CPA_STATUS_FAIL;
            break;
        case RING_OP_RELEASE:
            if (ioctl(fd, IOCTL_RELEASE_RING, &reserve) < 0)
                status = CPA_STATUS_FAIL;
            break;
        case RING_OP_ENABLE:
            if (ioctl(fd, IOCTL_ENABLE_RING, &reserve) < 0)
                status = CPA_STATUS_FAIL;
            break;
        case RING_OP_DISABLE:
            if (ioctl(fd, IOCTL_DISABLE_RING, &reserve) < 0)
                status = CPA_STATUS_FAIL;
            break;
        default:
            ADF_ERROR("Error: invalid ring operation %d\n", op);
            status = CPA_STATUS_FAIL;
    }

    close_dev(fd);
    return status;
}

CpaStatus icp_adf_reserve_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr)
{
    return ring_ioctl(accel_id, bank_nr, ring_nr, RING_OP_RESERVE);
}

CpaStatus icp_adf_release_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr)
{
    return ring_ioctl(accel_id, bank_nr, ring_nr, RING_OP_RELEASE);
}

CpaStatus icp_adf_enable_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr)
{
    return ring_ioctl(accel_id, bank_nr, ring_nr, RING_OP_ENABLE);
}

CpaStatus icp_adf_disable_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr)
{
    return ring_ioctl(accel_id, bank_nr, ring_nr, RING_OP_DISABLE);
}


/*
 * icp_adf_reset_device
 *
 * reset device - calls the IOCTL in
 * the driver which resets the device based on accelId
 */
CpaStatus icp_adf_reset_device(Cpa32U accelId)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    struct adf_user_cfg_ctl_data ctl_data = {{0}};
    int fd;

    fd = open_dev();
    if (fd <= 0)
    {
        return CPA_STATUS_FAIL;
    }

    ctl_data.device_id = accelId;
    if (ioctl(fd, IOCTL_RESET_ACCEL_DEV, &ctl_data))
    {
        if (EBUSY == errno)
            ADF_ERROR("Device busy \n");
        else
            ADF_ERROR("Failed to reset device \n");
        status = CPA_STATUS_FAIL;
    }

    close_dev(fd);

    return status;
}
