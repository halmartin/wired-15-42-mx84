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
#include "ini_config.h"
#include "global.h"
#include "utils.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace ini_config
{

config::config() {}

config::~config()
{
    for (auto it = sections.begin(); it != sections.end(); ++it)
    {
        delete it->second;
    }
}

bool config::read_ini(const std::string& filename, config& conf)
{
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    if (!ifs.good())
    {
        return false;
    }

    std::string line, sectionName, key, val;
    section* mySection = NULL;
    int lineNum = 0;
    while (getline(ifs, line))
    {
        lineNum++;
        if (line.empty() || line.at(0) == '#' || line.at(0) == '\r')
        {
            continue;
        }

        /* Check whether this is a section line */
        if (line.at(0) == '[')
        {
            get_section_name(utils::sanitize_string(line), sectionName);
            if (!sectionName.empty())
            {
                auto sectionIt = conf.sections.find(sectionName);
                if (sectionIt != conf.sections.end())
                {
                    mySection = sectionIt->second;
                }
                else
                {
                    mySection = new section();
                    conf.sections.insert(
                        std::make_pair(sectionName, mySection));
                }
            }
            else
            {
                std::cout << "Warning: Wrong section format in line "
                          << utils::to_string(lineNum) << ": \"" << line
                          << "\"\n";
            }
        }
        else
        {
            size_t pos = line.find("=");
            key = utils::sanitize_string(line.substr(0, pos));
            val = utils::sanitize_string(line.substr(pos + 1));
            if (val[val.length() - 1] == '\"' && val[0] == '\"')
                val = val.substr(1, val.length() - 2);

            if (!key.empty() && !val.empty())
            {
                value* myValue = new value(val, lineNum);
                mySection->values.insert(std::make_pair(key, myValue));
            }
            else
            {
                std::cout << "Warning: Wrong key-value format in line "
                          << utils::to_string(lineNum) << ": \"" << line
                          << "\"\n";
            }
        }
    }

    return true;
}

void config::get_section_name(const std::string& str, std::string& section)
{
    section.clear();
    size_t end = str.find("]");
    if (str.at(0) == '[' && end != std::string::npos)
    {
        section = str.substr(1, end - 1);
        section = utils::sanitize_string(section);
    }
}

const std::map<std::string, section*>& config::getSections() const
{
    return sections;
}

value::value(const std::string& val, unsigned line)
    : val(val)
    , line(line)
{
}

section::section() {}

section::~section()
{
    for (auto it = values.begin(); it != values.end(); ++it)
    {
        delete it->second;
    }
}

const std::map<std::string, value*>& section::getValues() const
{
    return values;
}

} // namespace ini_config
