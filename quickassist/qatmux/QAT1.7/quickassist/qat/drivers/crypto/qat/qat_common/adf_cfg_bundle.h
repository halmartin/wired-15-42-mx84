/*
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 *  redistributing this file, you may do so under either license.
 *
 *  GPL LICENSE SUMMARY
 *  Copyright(c) 2017 Intel Corporation.
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Contact Information:
 *
 *  qat-linux@intel.com
 *
 *  BSD LICENSE
 *  Copyright(c) 2017 Intel Corporation.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ADF_CFG_BUNDLE_H_
#define ADF_CFG_BUNDLE_H_

#include "adf_accel_devices.h"
#include "adf_cfg_common.h"
#include "adf_cfg_instance.h"
#include "adf_cfg_device.h"

#define MAX_SECTIONS_PER_BUNDLE 8
#define MAX_SECTION_NAME_LEN 64
#define ADF_CFG_NUM_SERVICES 4

#define TX	0x0
#define RX	0x1

#define ASSIGN_SERV_TO_RINGS(bund, index, base, stype, rng_per_srv) \
	do { \
		int j = 0; \
		typeof(bund) b = (bund); \
		typeof(index) i = (index); \
		typeof(base) s = (base); \
		typeof(stype) t = (stype); \
		typeof(rng_per_srv) rps = (rng_per_srv); \
		for (j = 0; j < rps; j++) { \
			b->rings[i + j]->serv_type = t; \
			b->rings[i + j + s]->serv_type = t; \
		} \
	} while (0)

struct adf_cfg_device;

enum adf_accel_serv_type {
	ADF_ACCEL_SERV_NA = 0x0,
	ADF_ACCEL_SERV_ASYM,
	ADF_ACCEL_SERV_SYM,
	ADF_ACCEL_SERV_RND,
	ADF_ACCEL_SERV_DC
};

struct adf_cfg_ring {
	u8 mode:1;
	enum adf_accel_serv_type serv_type;
	u8 number:4;
};

struct adf_cfg_bundle {
	/* Section(s) name this bundle is shared by */
	char **sections;
	int max_section;
	int section_index;
	int number;
	enum adf_cfg_bundle_type type;
	cpumask_t affinity_mask;
	int polling_mode;
	int instance_num;
	int num_of_rings;
	/* contains all the info about rings */
	struct adf_cfg_ring **rings;
	u16 in_use;
};

bool adf_cfg_is_free(struct adf_cfg_bundle *bundle);

int adf_cfg_get_ring_pairs_from_bundle(struct adf_cfg_bundle *bundle,
				       struct adf_cfg_instance *inst,
				       const char *process_name,
				       struct adf_cfg_instance *bundle_inst);

struct adf_cfg_instance *adf_cfg_get_free_instance(
					struct adf_cfg_device *device,
					struct adf_cfg_bundle *bundle,
					struct adf_cfg_instance *inst,
					const char *process_name);

int adf_cfg_bundle_init(struct adf_cfg_bundle *bundle,
			struct adf_cfg_device *device,
			int bank_num,
			struct adf_accel_dev *accel_dev);

void adf_cfg_bundle_clear(struct adf_cfg_bundle *bundle,
			  struct adf_accel_dev *accel_dev);

int adf_cfg_init_ring2serv_mapping(struct adf_accel_dev *accel_dev,
				   struct adf_cfg_bundle *bundle,
				   struct adf_cfg_device *device);

int adf_cfg_rel_ring2serv_mapping(struct adf_cfg_bundle *bundle);

int adf_cfg_rel_ring2serv_mapping(struct adf_cfg_bundle *bundle);
#endif
