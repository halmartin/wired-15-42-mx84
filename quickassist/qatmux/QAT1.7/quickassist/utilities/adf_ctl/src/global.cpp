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
#include "global.h"

const char* constants::general_sec = ADF_GENERAL_SEC;
const char* constants::kernel_sec = ADF_KERNEL_SEC;
const char* constants::accelerator_sec = ADF_ACCEL_SEC;
const char* constants::num_cy_inst = ADF_NUM_CY;
const char* constants::num_dc_inst = ADF_NUM_DC;
const char* constants::num_process = "NumProcesses";
const char* constants::limit_dev_access = "LimitDevAccess";
const char* constants::derived_sec_name = "_INT_";
const char* constants::derived_sec_name_dev = "_DEV";
const char* constants::inst_cy = ADF_CY;
const char* constants::inst_dc = ADF_DC;
const char* constants::inst_name = "Name";
const char* constants::inst_is_polled = "IsPolled";
const char* constants::inst_affinity = ADF_ETRMGR_CORE_AFFINITY;
const char* constants::inst_bank = ADF_RING_BANK_NUM;
const char* constants::inst_cy_asym_tx = ADF_RING_ASYM_TX;
const char* constants::inst_cy_sym_tx = ADF_RING_SYM_TX;
const char* constants::inst_cy_asym_rx = ADF_RING_ASYM_RX;
const char* constants::inst_cy_sym_rx = ADF_RING_SYM_RX;
const char* constants::inst_cy_asym_req = ADF_RING_ASYM_SIZE;
const char* constants::inst_cy_sym_req = ADF_RING_SYM_SIZE;
const int constants::nr_cy_ring_pairs = 2;
const char* constants::inst_dc_rx = ADF_RING_DC_RX;
const char* constants::inst_dc_tx = ADF_RING_DC_TX;
const char* constants::inst_dc_req = ADF_RING_DC_SIZE;
const int constants::nr_dc_ring_pairs = 1;
const char* constants::config_ver = "ConfigVersion";
const char* constants::config_accel_bank = "Bank";
const char* constants::config_accel_coalesc = ADF_ETRMGR_COALESCING_ENABLED;
const char* constants::config_accel_coalesc_time = ADF_ETRMGR_COALESCE_TIMER;
const char* constants::config_accel_affinity = ADF_ETRMGR_CORE_AFFINITY;
const char* constants::config_accel_affinity_num_resp
    = ADF_ETRMGR_COALESCING_MSG_ENABLED;
const int constants::config_accel_def_coales = 1;
const int constants::config_accel_def_coales_timer = 10000;
const int constants::config_accel_def_coales_num_msg = 0;
const int constants::config_resp_poll = 1;
const int constants::config_resp_epoll = 2;
const char* constants::range_separator = "-";
const char* constants::path_to_config = "/etc/";
const char* constants::config_extension = ".conf";
const char* constants::up = "up";
const char* constants::dw = "down";
const char* constants::status = "status";
const char* constants::restart = "restart";
const char* constants::reset = "reset";
const char* constants::dev_param_name = "qat_dev";
const char* constants::config_file_dev_name = "_dev";
const char* constants::qat_ctl_file = "/dev/qat_adf_ctl";
const int constants::config_default_big_ring_size = 512;
const int constants::config_default_small_ring_size = 64;
const char* constants::first_user_bundle = "FirstUserBundle";
const int constants::nr_proc_per_bundle = 4;
const char* constants::adf_uio_name = "UIO_%s_%02d_BUNDLE_%02d";
