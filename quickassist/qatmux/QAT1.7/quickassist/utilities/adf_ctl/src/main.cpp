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
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <getopt.h>
#include <iostream>
#include <unistd.h>

#include "adf_ctl.h"
#include "global.h"

extern int qat_file;

void adf_ctl_help()
{
    std::cout
        << "Use of adf_ctl: "
           "adf_ctl [qat_dev<N>] [up | down | restart | reset] - "
           "to bring up or down device(s)\n"
           "or:\n adf_ctl [qat_dev<N>] status - to print device(s) status\n"
           "or no parameters to configure all devices\n\n"
        << std::endl;
}

void print_action(adf_ctl::action a, int dev)
{
    switch (a)
    {
        case adf_ctl::action::START:
            std::cout << "Starting ";
            break;
        case adf_ctl::action::STOP:
            std::cout << "Stopping ";
            break;
        case adf_ctl::action::QUERY:
            std::cout << "Checking status of ";
            break;
        case adf_ctl::action::RESTART:
            std::cout << "Restarting ";
            break;
        case adf_ctl::action::RESET:
            std::cout << "Resetting ";
            break;
        case adf_ctl::action::NONE:
            return;
    }
    if (dev == ADF_CFG_ALL_DEVICES)
        std::cout << "all devices.";
    else
        std::cout << "device qat_dev" << dev;
    std::cout << std::endl;
}

adf_ctl::action parse_action(const std::string& act_str)
{
    if (constants::up == act_str)
    {
        return adf_ctl::action::START;
    }
    else if (constants::dw == act_str)
    {
        return adf_ctl::action::STOP;
    }
    else if (constants::status == act_str)
    {
        return adf_ctl::action::QUERY;
    }
    else if (constants::restart == act_str)
    {
        return adf_ctl::action::RESTART;
    }
    else if (constants::reset == act_str)
    {
        return adf_ctl::action::RESET;
    }
    else
    {
        return adf_ctl::action::NONE;
    }
}

int parse_device(const std::string& dev_str)
{
    int dev_id = ADF_CFG_NO_DEVICE;
    if (dev_str.find(constants::dev_param_name) == std::string::npos)
    {
        return ADF_CFG_NO_DEVICE;
    }
    dev_id = stoi(dev_str.substr(strlen(constants::dev_param_name)));

    return dev_id;
}

int main(int argc, char** argv)
{
    int dev_id = ADF_CFG_ALL_DEVICES;
    adf_ctl::action act = adf_ctl::action::NONE;

    /* Handle command line params */
    if (argc == 1)
    {
        act = adf_ctl::action::START;
    }

    if (argc >= 2)
    {
        /* For backwards compatibility, if the first argument does not contain
         * the "-" character, it will work exactly as it did before */
        if (argv[1][0] != '-')
        {
            if (std::string(argv[1]) == "help")
            {
                adf_ctl_help();
                return 0;
            }

            if (argc == 2 || argc == 3)
            {
                act = parse_action(argv[argc - 1]);
            }

            if (argc == 3)
            {
                try
                {
                    dev_id = parse_device(argv[1]);
                    if (dev_id == -1)
                    {
                        adf_ctl_help();
                        return -1;
                    }
                }
                catch (std::exception& e)
                {
                    std::cerr
                        << "QAT Error: invalid param dev_id<n>: " << e.what()
                        << std::endl;
                    return -1;
                }
            }
        }
    }


    /* Verify params */
    if (act == adf_ctl::action::NONE || dev_id == ADF_CFG_NO_DEVICE)
    {
        adf_ctl_help();
        return -1;
    }
    print_action(act, dev_id);

    qat_file = open(constants::qat_ctl_file, O_RDWR);
    if (qat_file < 0)
    {
        std::cerr << "Can not open " << constants::qat_ctl_file << std::endl;
        return -1;
    }

    /* Do the job */
    int ret = 0;
    switch (act)
    {
        case adf_ctl::action::START:
            ret = adf_ctl::perform_start_dev(dev_id);
            break;
        case adf_ctl::action::STOP:
            ret = adf_ctl::perform_stop_dev(dev_id);
            break;
        case adf_ctl::action::QUERY:
            ret = adf_ctl::perform_query_dev(dev_id);
            break;
        case adf_ctl::action::RESTART:
            ret = adf_ctl::perform_stop_dev(dev_id);
            if (ret != 0)
            {
                break;
            }
            ret = adf_ctl::perform_start_dev(dev_id);
            break;
        case adf_ctl::action::RESET:
            ret = adf_ctl::perform_reset_dev(dev_id);
            break;
        case adf_ctl::action::NONE:
            ret = -1;
    }

    close(qat_file);


    return ret;
}
