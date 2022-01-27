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
#include "utils.h"

#include "adf_cfg_user.h"
#include "config_section.h"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace utils
{

static size_t long_max_str_len = get_max_str_len<long>();

adf_cfg_val_type get_value_type(const std::string& val)
{
    /* Check for hex number */
    if (val.size() >= 3 && val.find("0x") == 0)
    {
        bool validHexNumber = true;
        for (unsigned i = 2; i < val.size(); ++i)
        {
            if (!isxdigit(val[i]))
            {
                validHexNumber = false;
                break;
            }
        }

        if (validHexNumber)
        {
            /* Only long values are supported for HEX values, so anything longer
             * than that has to be considered as a STR */
            if (val.size() - 2 > 2 * sizeof(long))
            {
                return ADF_STR;
            }

            return ADF_HEX;
        }
    }

    /* Check for decimal number */
    bool validDecNumber = true;
    for (unsigned i = 0; i < val.size(); ++i)
    {
        if (!isdigit(val[i]))
        {
            validDecNumber = false;
            break;
        }
    }
    if (validDecNumber)
    {
        /* Only long values are supported for DEC values, so anything longer
         * than that has to be considered as a STR */
        if (val.size() > long_max_str_len)
        {
            return ADF_STR;
        }

        return ADF_DEC;
    }

    /* If the format doesn't match an hex nor a decimal number,
     * it has to be a string */
    return ADF_STR;
}

std::string sanitize_string(const std::string& str)
{
    std::string san_str = str.substr(0, str.find_first_of("#"));
    remove_whitespaces(san_str);
    return san_str;
}

void remove_whitespaces(std::string& str)
{
    str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
    str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
}

void init_adf_user_cfg_section(adf_user_cfg_section& section)
{
    memset(&section, 0, sizeof(section));
}

void init_adf_user_cfg_key_val(adf_user_cfg_key_val& key_val)
{
    memset(&key_val, 0, sizeof(key_val));
}

} // namespace utils
