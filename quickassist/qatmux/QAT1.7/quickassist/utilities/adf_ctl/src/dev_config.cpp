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
#include "dev_config.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/ioctl.h>

#include "adf_ctl.h"
#include "config_section.h"
#include "sections.h"
#include "utils.h"

extern int qat_file;
extern dev_config* cfg;

dev_config::dev_config(adf_dev_status_info* dev_info)
    : dev_info(dev_info)
    , config_file(constants::path_to_config)
    , gen(NULL)
{
    int inst = dev_info->instance_id;
    config_file += dev_info->name;
    config_file += constants::config_file_dev_name;
    config_file += utils::to_string(inst);
    config_file += constants::config_extension;

    read();
}

dev_config::~dev_config()
{
    /* Free all configuration */
    for (auto it = sections.begin(); it != sections.end(); ++it)
    {
        delete *it;
    }
    for (auto it = user_cfg_key_val.begin(); it != user_cfg_key_val.end(); ++it)
    {
        delete *it;
    }
    for (auto it = user_cfg_sections.begin(); it != user_cfg_sections.end();
         ++it)
    {
        delete *it;
    }
}

void dev_config::read()
{
    /* Read the whole config file */
    std::cout << "Processing " << config_file << std::endl;
    if (!ini_config::config::read_ini(config_file, config))
    {
        throw std::runtime_error(std::string("Filename ") + config_file
                                 + " cannot be read");
    }
    validate_config();
}

void dev_config::validate_field(const std::string& s, size_t max_len)
{
    if (s.length() >= max_len)
    {
        throw std::runtime_error(
            s
            + ": entry too long. Maximum allowed: " + utils::to_string(max_len)
            + ", current value: " + utils::to_string(s.length()));
    }
}

void dev_config::validate_key(const std::string& s)
{
    validate_field(s, ADF_CFG_MAX_KEY_LEN_IN_BYTES);
}

void dev_config::validate_value(const std::string& s)
{
    validate_field(s, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
}

void dev_config::validate_section(const std::string& s)
{
    validate_field(s, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
}

void dev_config::validate_config()
{
    auto sections = config.getSections();

    /* General section */
    auto generalSec = sections.find(constants::general_sec);
    if (generalSec != sections.end())
    {
        auto sectionValues = generalSec->second->getValues();

        /* ConfigVersion must be 2 */
        auto configVersion = sectionValues.find(constants::config_ver);
        if (configVersion != sectionValues.end())
        {
            int version = utils::to_number<int>(configVersion->second->val);
            if (version != 2)
            {
                throw std::runtime_error(
                    "Only version 2 config file supported");
            }
        }
        else
        {
            throw std::runtime_error(std::string("Key ") + constants::config_ver
                                     + " missing in " + constants::general_sec
                                     + " section");
        }
    }
}

void dev_config::configure_dev()
{
    config_section* s;
    std::string sec_name;

    /* Loop through all sections from the given configuration file
     * and create section instances that will be used later to
     * porcess the configuration */
    auto iniSections = config.getSections();
    for (auto sectionIt = iniSections.begin(); sectionIt != iniSections.end();
         ++sectionIt)
    {
        sec_name = sectionIt->first;
        validate_section(sec_name);
        if (sec_name == constants::general_sec)
        {
            /* Check if we already have generel section */
            if (gen)
            {
                /* Note: uniqueness of sections and keys
                 * within a section is guaranteed by ini_config::config.
                 * This check will prevent multiple calls to
                 * configure_dev() on one dev_config instance. */
                throw std::runtime_error(
                    "You can call configure_dev() only once");
            }
            gen = new section_general(sec_name.c_str());
            s = dynamic_cast<config_section*>(gen);
        }
        else if (sec_name == constants::kernel_sec)
        {
            s = new section_kernel(sec_name);
        }
        else
        {
            s = new config_section(sec_name);
        }
        sections.push_back(s);

        /* Loop through all entries in a section and store
         * it in out new created config_section or derived type */
        auto iniValues = sectionIt->second->getValues();
        for (auto valueIt = iniValues.begin(); valueIt != iniValues.end();
             ++valueIt)
        {
            validate_key(valueIt->first);
            validate_value(valueIt->second->val);
            *s += std::make_pair(valueIt->first.data(),
                                 valueIt->second->val.data());
        }
    }

    if (gen == NULL)
    {
        throw std::runtime_error(std::string("No ") + constants::general_sec
                                 + " section found");
    }

    /* Finally load the configuration to the driver
     * and start the device */
    adf_user_cfg_ctl_data dev_data = { { 0 }, 0 };
    dev_data.device_id = dev_info->accel_id;
    dev_data.config_section = NULL;
    adf_user_cfg_section* section;
    adf_user_cfg_section* section_prev = NULL;
    adf_user_cfg_key_val* key_val;

    for (auto sec_itr = sections.begin(); sec_itr != sections.end(); ++sec_itr)
    {
        auto this_section = *sec_itr;
        adf_user_cfg_key_val* key_val_prev = NULL;
        section = new adf_user_cfg_section;
        /* This is stored for convenience to be deleted later
         * in destructor, same as key_values below */
        user_cfg_sections.push_back(section);
        utils::init_adf_user_cfg_section(*section);
        if (dev_data.config_section == NULL)
        {
            dev_data.config_section = section;
        }
        if (section_prev)
            section_prev->next = section;
        section_prev = section;
        strncpy(section->name,
                this_section->get_name().c_str(),
                ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
        key_val_prev = NULL;
        for (auto val_itr = this_section->get_entries().begin();
             val_itr != this_section->get_entries().end();
             ++val_itr)
        {
            auto value = *val_itr;
            key_val = new adf_user_cfg_key_val;
            user_cfg_key_val.push_back(key_val);
            utils::init_adf_user_cfg_key_val(*key_val);
            if (section->params == NULL)
            {
                section->params = key_val;
            }
            if (key_val_prev)
                key_val_prev->next = key_val;
            key_val_prev = key_val;
            strncpy(key_val->key,
                    value.first.c_str(),
                    ADF_CFG_MAX_KEY_LEN_IN_BYTES);
            key_val->type = utils::get_value_type(value.second);
            long val = 0;
            std::stringstream ss;
            switch (key_val->type)
            {
                case ADF_STR:
                    strncpy(key_val->val,
                            value.second.c_str(),
                            ADF_CFG_MAX_VAL_LEN_IN_BYTES);
                    break;
                case ADF_DEC:
                    val = utils::to_number<long>(value.second.c_str());
                    memcpy(key_val->val, &val, sizeof(long));
                    break;
                case ADF_HEX:
                    ss << std::hex << value.second;
                    ss >> val;
                    memcpy(key_val->val, &val, sizeof(long));
                    break;
            }
        }
    }

    if (ioctl(qat_file, IOCTL_CONFIG_SYS_RESOURCE_PARAMETERS, &dev_data))
    {
        std::cerr << "Ioctl failed" << std::endl;
        throw std::runtime_error("Failed to load config data to device");
    }
    if (ioctl(qat_file, IOCTL_START_ACCEL_DEV, &dev_data))
    {
        std::cerr << "Ioctl failed" << std::endl;
        throw std::runtime_error("Failed to start device");
    }
}
