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
#ifndef GLOBAL_H
#define GLOBAL_H

#include <bitset>
#include <cstdint>

typedef std::uint8_t u8;
typedef std::uint16_t u16;
typedef std::uint32_t u32;
typedef std::uint64_t u64;

extern "C"
{
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif
#include <adf_cfg_user.h>
}

#define UNUSED(x) (void)(x)

struct constants
{
    static const char* general_sec;
    static const char* kernel_sec;
    static const char* accelerator_sec;
    static const char* num_cy_inst;
    static const char* num_dc_inst;
    static const char* num_process;
    static const char* limit_dev_access;
    static const char* derived_sec_name;
    static const char* derived_sec_name_dev;
    static const char* inst_cy;
    static const char* inst_dc;
    static const char* inst_name;
    static const char* inst_is_polled;
    static const char* inst_affinity;
    static const char* inst_bank;
    static const char* inst_cy_asym_tx;
    static const char* inst_cy_sym_tx;
    static const char* inst_cy_asym_rx;
    static const char* inst_cy_sym_rx;
    static const char* inst_cy_asym_req;
    static const char* inst_cy_sym_req;
    static const int nr_cy_ring_pairs;
    static const char* inst_dc_rx;
    static const char* inst_dc_tx;
    static const char* inst_dc_req;
    static const int nr_dc_ring_pairs;
    static const char* config_ver;
    static const char* config_accel_bank;
    static const char* config_accel_coalesc;
    static const char* config_accel_coalesc_time;
    static const char* config_accel_affinity;
    static const char* config_accel_affinity_num_resp;
    static const int config_accel_def_coales;
    static const int config_accel_def_coales_timer;
    static const int config_accel_def_coales_num_msg;
    static const int config_resp_poll;
    static const int config_resp_epoll;
    static const char* range_separator;
    static const char* path_to_config;
    static const char* config_extension;
    static const char* up;
    static const char* dw;
    static const char* status;
    static const char* restart;
    static const char* reset;
    static const char* dev_param_name;
    static const char* config_file_dev_name;
    static const char* qat_ctl_file;
    static const int config_default_big_ring_size;
    static const int config_default_small_ring_size;
    static const char* first_user_bundle;
    static const int nr_proc_per_bundle;
    static const char* adf_uio_name;
};

// max_nr_cpus needs to be defined here so that the typedef already has the
// value for every translation unit
static const int max_nr_cpus = 256;
typedef std::bitset<max_nr_cpus> affinity_mask_t;

#endif
