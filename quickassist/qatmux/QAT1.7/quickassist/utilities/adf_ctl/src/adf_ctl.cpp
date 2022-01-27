/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
 *
 ****************************************************************************/
#include "adf_ctl.h"

#include <exception>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <sys/ioctl.h>

#include "config_section.h"
#include "dev_config.h"
#include "global.h"
#include "sections.h"
#include "utils.h"

class dev_config;
std::unique_ptr<dev_config> cfg;
int qat_file = -1;

class config_section;
class section_general;

namespace adf_ctl
{

int configure_dev(adf_dev_status_info* dev_info)
{
    int ret = 0;
    /* Create device configuration instance and
     * configure the device. All errors during configuration
     * are handled here using std exceptions. */
    try
    {
        cfg = std::unique_ptr<dev_config>(new dev_config(dev_info));
    }
    catch (std::exception& e)
    {
        std::cerr << "QAT Error: " << e.what() << std::endl;
        return -1;
    }
    try
    {
        cfg->configure_dev();
    }
    catch (std::exception& e)
    {
        std::cerr << "QAT Error: " << e.what() << std::endl;
        ret = -1;
    }
    return ret;
}

int perform_start_dev(int dev_id)
{
    adf_dev_status_info dev_info;
    int ret = 0;
    dev_info.type = DEV_UNKNOWN;
    /* Start all devices that are not yet started */
    if (ADF_CFG_ALL_DEVICES == dev_id)
    {
        int num_devices, devs_found = 0;
        if (ioctl(qat_file, IOCTL_GET_NUM_DEVICES, &num_devices))
        {
            std::cerr << "Ioctl failed" << std::endl;
            return -1;
        }
        for (int i = 0; i < ADF_MAX_DEVICES; i++)
        {
            dev_info.accel_id = i;
            if (ioctl(qat_file, IOCTL_STATUS_ACCEL_DEV, &dev_info))
            {
                if (errno != ENODEV)
                {
                    std::cerr << "Ioctl failed" << std::endl;
                    return -1;
                }
                continue;
            }


            if (dev_info.state)
            {
                    continue;
            }

            if (configure_dev(&dev_info))
            {
                ret = -1;
                std::cerr << "Failed to configure qat_dev" << i << std::endl;
            }

            if (++devs_found == num_devices)
                break;
        }
    }
    else
    {

        dev_info.accel_id = dev_id;
        if (ioctl(qat_file, IOCTL_STATUS_ACCEL_DEV, &dev_info))
        {
            std::cerr << "Ioctl failed" << std::endl;
            return -1;
        }
        if (dev_info.state)
        {
                std::cerr << "Device qat_dev" << dev_id << " already configured"
                          << std::endl;
                return 0;
        }


        if (configure_dev(&dev_info))
        {
            std::cerr << "Failed to configure qat_dev" << dev_id << std::endl;
            return -1;
        }
    }
    return ret;
}

int perform_stop_dev(int dev_id)
{
    adf_user_cfg_ctl_data ctl_data = { { 0 }, 0 };
    ctl_data.device_id = dev_id;
    int ret = 0;

    ret = ioctl(qat_file, IOCTL_STOP_ACCEL_DEV, &ctl_data);

    if (ret)
    {
        if (EBUSY == errno)
            std::cerr << "device busy" << std::endl;
        else
            std::cerr << "Failed to stop device " << std::endl;
        return -1;
    }
    return 0;
}

void print_dev_info(adf_dev_status_info* dev_info)
{
    std::cout << " qat_dev" << (int)dev_info->accel_id
              << " - type: " << dev_info->name << ", "
              << " inst_id: " << (int)dev_info->instance_id << ", "
              << " node_id: " << (int)dev_info->node_id << ", "
              << " bsf: " << std::setbase(16) << std::setfill('0')
              << std::setw(4) << (int)dev_info->domain << std::setw(1) << ":"
              << std::setw(2) << (int)dev_info->bus << std::setw(1) << ":"
              << std::setw(2) << (int)dev_info->dev << std::setw(1) << "."
              << (int)dev_info->fun << ", " << std::setfill(' ')
              << std::setbase(10) << " #accel: " << (int)dev_info->num_accel
              << " #engines: " << (int)dev_info->num_ae
              << " state: " << (dev_info->state ? "up" : "down") << std::endl;
}

int perform_query_dev(int dev_id)
{
    adf_dev_status_info dev_info;

    dev_info.type = DEV_UNKNOWN;

    /* Start all devices that are not yet started */
    if (ADF_CFG_ALL_DEVICES == dev_id)
    {
        int num_devices, devs_found = 0;

        if (ioctl(qat_file, IOCTL_GET_NUM_DEVICES, &num_devices))
        {
            std::cerr << "Ioctl failed" << std::endl;
            return -1;
        }
        std::cout << "There is " << num_devices
                  << " QAT acceleration device(s) in the system:" << std::endl;
        for (int i = 0; i < ADF_MAX_DEVICES; i++)
        {
            dev_info.accel_id = i;
            if (ioctl(qat_file, IOCTL_STATUS_ACCEL_DEV, &dev_info))
            {
                if (errno != ENODEV)
                {
                    std::cerr << "Ioctl failed" << std::endl;
                    return -1;
                }
                continue;
            }
            print_dev_info(&dev_info);
            if (++devs_found == num_devices)
                break;
        }
    }
    else
    {

        dev_info.accel_id = dev_id;
        if (ioctl(qat_file, IOCTL_STATUS_ACCEL_DEV, &dev_info))
        {
            std::cerr << "Ioctl failed" << std::endl;
            return -1;
        }
        print_dev_info(&dev_info);
    }
    return 0;
}

int perform_reset_dev(int dev_id)
{
    adf_user_cfg_ctl_data ctl_data = { { 0 }, 0 };
    ctl_data.device_id = dev_id;
    int ret = ioctl(qat_file, IOCTL_RESET_ACCEL_DEV, &ctl_data);

    if (ret)
    {
        if (EBUSY == errno)
            std::cerr << "device busy" << std::endl;
        else
            std::cerr << "Failed to reset device " << std::endl;
        return -1;
    }
    return 0;
}

} // namespace adf_ctl
