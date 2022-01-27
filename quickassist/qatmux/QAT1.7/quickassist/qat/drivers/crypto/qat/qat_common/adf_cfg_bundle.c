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
#include <linux/pci.h>
#include "adf_cfg_bundle.h"
#include "adf_cfg_strings.h"

static bool adf_cfg_is_interrupt_mode(struct adf_cfg_bundle *bundle)
{
	return (bundle->polling_mode == ADF_CFG_RESP_EPOLL) ||
	       (bundle->type == KERNEL &&
	       (bundle->polling_mode != ADF_CFG_RESP_POLL));
}

static bool adf_cfg_can_be_shared(struct adf_cfg_bundle *bundle,
				  const char *process_name,
				  int polling_mode)
{
	if (adf_cfg_is_free(bundle))
		return true;

	if (bundle->polling_mode != polling_mode)
		return false;

	return !adf_cfg_is_interrupt_mode(bundle) ||
	       !strcmp(process_name, bundle->sections[0]);
}

bool adf_cfg_is_free(struct adf_cfg_bundle *bundle)
{
	return bundle->type == FREE;
}

struct adf_cfg_instance *adf_cfg_get_free_instance(
					struct adf_cfg_device *device,
					struct adf_cfg_bundle *bundle,
					struct adf_cfg_instance *inst,
					const char *process_name)
{
	int i = 0;
	struct adf_cfg_instance *ret_instance = NULL;

	if (adf_cfg_can_be_shared(bundle, process_name, inst->polling_mode)) {
		for (i = 0; i < device->instance_index; i++) {
			/*
			 * the selected instance must match two criteria
			 * 1) instance is from the bundle
			 * 2) instance type is same
			 */
			if ((bundle->number == device->instances[i]->bundle) &&
			    (inst->stype == device->instances[i]->stype)) {
				ret_instance = device->instances[i];
				break;
			}
			/*
			 * no opportunity to match,
			 * quit the loop as early as possible
			 */
			if ((bundle->number + 1) ==
					device->instances[i]->bundle)
				break;
		}
	}

	return  ret_instance;
}

int adf_cfg_get_ring_pairs_from_bundle(struct adf_cfg_bundle *bundle,
				       struct adf_cfg_instance *inst,
				       const char *process_name,
				       struct adf_cfg_instance *bundle_inst)
{
	if (inst->polling_mode == ADF_CFG_RESP_POLL &&
	    adf_cfg_is_interrupt_mode(bundle)) {
		pr_err("Trying to get ring pairs for a non-interrupt bundle from an interrupt bundle\n");
		return -EFAULT;
	}

	if (inst->stype != bundle_inst->stype) {
		pr_err("Got an instance of different type (cy/dc) than the one request\n");
		return -EFAULT;
	}

	if (strcmp(ADF_KERNEL_SEC, process_name) &&
	    strcmp(ADF_KERNEL_SAL_SEC, process_name) &&
	    inst->polling_mode != ADF_CFG_RESP_EPOLL &&
	    inst->polling_mode != ADF_CFG_RESP_POLL) {
		pr_err("User instance %s needs to be configured with IsPolled 1 or 2 for poll and epoll mode, respectively\n",
		       inst->name);
		return -EFAULT;
	}

	strlcpy(bundle->sections[bundle->section_index],
		process_name, ADF_CFG_MAX_STR_LEN);
	bundle->section_index++;

	if (adf_cfg_is_free(bundle)) {
		bundle->polling_mode = inst->polling_mode;
		bundle->type = (!strcmp(ADF_KERNEL_SEC, process_name) ||
				!strcmp(ADF_KERNEL_SAL_SEC, process_name))
				? KERNEL
				: USER;
		if (adf_cfg_is_interrupt_mode(bundle)) {
			cpumask_clear(&bundle->affinity_mask);
			cpumask_copy(&bundle->affinity_mask,
				     &inst->affinity_mask);
		}
	}

	switch (inst->stype) {
	case CRYPTO:
		inst->asym_tx = bundle_inst->asym_tx;
		inst->asym_rx = bundle_inst->asym_rx;
		inst->sym_tx = bundle_inst->sym_tx;
		inst->sym_rx = bundle_inst->sym_rx;
		break;
	case COMP:
		inst->dc_tx = bundle_inst->dc_tx;
		inst->dc_rx = bundle_inst->dc_rx;
		break;
	case ASYM:
		inst->asym_tx = bundle_inst->asym_tx;
		inst->asym_rx = bundle_inst->asym_rx;
		break;
	case SYM:
		inst->sym_tx = bundle_inst->sym_tx;
		inst->sym_rx = bundle_inst->sym_rx;
		break;
	default:
		/* unknown service type of instance */
		pr_err("Unknown service type %d of instance\n", inst->stype);
	}

	/* mark it as used */
	bundle_inst->stype = USED;

	inst->bundle = bundle->number;

	return 0;
}

int adf_cfg_init_and_insert_inst(struct adf_cfg_bundle *bundle,
				 struct adf_cfg_device *device,
				 int bank_num,
				 struct adf_accel_dev *accel_dev)
{
	struct adf_cfg_instance *cfg_instance = NULL;
	int ring_pair_index = 0;
	int i = 0;
	u8 serv_type;
	int num_req_rings = bundle->num_of_rings / 2;
	int num_rings_per_srv = num_req_rings / ADF_CFG_NUM_SERVICES;

	/* init the bundle with instance information */
	for (ring_pair_index = 0;
	     ring_pair_index < ADF_CFG_NUM_SERVICES;
	     ring_pair_index++) {
		serv_type =
			GET_SRV_TYPE(device->serv_ena_mask, ring_pair_index);
		for (i = 0; i < num_rings_per_srv; i++) {
			cfg_instance =
				kzalloc(sizeof(*cfg_instance), GFP_KERNEL);
			if (!cfg_instance)
				goto failed;

			switch (serv_type) {
			case CRYPTO:
				crypto_instance_init(cfg_instance, bundle);
				break;
			case COMP:
				dc_instance_init(cfg_instance, bundle);
				break;
			case ASYM:
				asym_instance_init(cfg_instance, bundle);
				break;
			case SYM:
				sym_instance_init(cfg_instance, bundle);
				break;
			default:
				/* Unknown service type */
				dev_dbg(&GET_DEV(accel_dev),
					"Unknown service type %d of instance, mask is 0x%x\n",
					serv_type, device->serv_ena_mask);
			}
			cfg_instance->bundle = bank_num;
			device->instances[device->instance_index++] =
					cfg_instance;
			cfg_instance = NULL;
		}
		if (serv_type == CRYPTO)
			ring_pair_index++;
	}
	return 0;
failed:
	/* if failed to alloc memory, need to do clean up */
		kfree(cfg_instance);
		if (device->instance_index > 0) {
			for (i = 0; i < device->instance_index; i++)
				kfree(device->instances[i]);
		}
		return -ENOMEM;
}

int adf_cfg_bundle_init(struct adf_cfg_bundle *bundle,
			struct adf_cfg_device *device,
			int bank_num,
			struct adf_accel_dev *accel_dev)
{
	int i = 0;
	int ret = -ENOMEM;

	/* init ring to service mapping for this bundle */
	ret = adf_cfg_init_ring2serv_mapping(accel_dev, bundle, device);
	if (ret)
		goto failed;

	/* init the bundle with instance information */
	ret = adf_cfg_init_and_insert_inst(bundle, device, bank_num, accel_dev);
	if (ret)
		goto failed;

	cpumask_setall(&bundle->affinity_mask);
	bundle->type = FREE;
	bundle->polling_mode = -1;
	bundle->section_index = 0;
	bundle->number = bank_num;

	bundle->sections =
		kzalloc(sizeof(char *) * bundle->max_section, GFP_KERNEL);

	if (!bundle->sections)
		goto failed;

	for (i = 0; i < bundle->max_section; i++) {
		bundle->sections[i] = kzalloc(ADF_CFG_MAX_STR_LEN,
						GFP_KERNEL);
		if (!bundle->sections[i])
			goto failed;
	}
	return 0;
failed:
	for (i = 0; i < bundle->max_section; i++) {
		if (bundle->sections && bundle->sections[i])
			kfree(bundle->sections[i]);
	}
	kfree(bundle->sections);

	dev_err(&GET_DEV(accel_dev),
		"failed to alloc memory during bundle init\n");

	return ret;
}

void adf_cfg_bundle_clear(struct adf_cfg_bundle *bundle,
			  struct adf_accel_dev *accel_dev)
{
	int i = 0;

	for (i = 0; i < bundle->max_section; i++) {
		if (bundle->sections && bundle->sections[i]) {
			kfree(bundle->sections[i]);
			bundle->sections[i] = NULL;
		}
	}

	kfree(bundle->sections);
	bundle->sections = NULL;

	adf_cfg_rel_ring2serv_mapping(bundle);
}

int adf_cfg_assign_serv_to_rings(struct adf_cfg_bundle *bundle,
				 struct adf_cfg_device *device)
{
	int ring_pair_index = 0;
	int ring_index = 0;
	u8 serv_type = 0;
	int num_req_rings = bundle->num_of_rings / 2;
	int num_rings_per_srv = num_req_rings / ADF_CFG_NUM_SERVICES;

	for (ring_pair_index = 0;
		ring_pair_index < ADF_CFG_NUM_SERVICES;
		ring_pair_index++) {
		serv_type =
			GET_SRV_TYPE(device->serv_ena_mask, ring_pair_index);
		ring_index = num_rings_per_srv * ring_pair_index;
		switch (serv_type) {
		case CRYPTO:
			ASSIGN_SERV_TO_RINGS(bundle, ring_index, num_req_rings,
					     ADF_ACCEL_SERV_ASYM,
					     num_rings_per_srv);
			ring_pair_index++;
			ring_index = num_rings_per_srv * ring_pair_index;
			if (ring_pair_index ==
				ADF_CFG_NUM_SERVICES)
				break;
			ASSIGN_SERV_TO_RINGS(bundle, ring_index, num_req_rings,
					     ADF_ACCEL_SERV_SYM,
					     num_rings_per_srv);
			break;
		case COMP:
			ASSIGN_SERV_TO_RINGS(bundle, ring_index, num_req_rings,
					     ADF_ACCEL_SERV_DC,
					     num_rings_per_srv);
			break;
		case SYM:
			ASSIGN_SERV_TO_RINGS(bundle, ring_index, num_req_rings,
					     ADF_ACCEL_SERV_SYM,
					     num_rings_per_srv);
			break;
		case ASYM:
			ASSIGN_SERV_TO_RINGS(bundle, ring_index, num_req_rings,
					     ADF_ACCEL_SERV_ASYM,
					     num_rings_per_srv);
			break;
		case NA:
			ASSIGN_SERV_TO_RINGS(bundle, ring_index, num_req_rings,
					     ADF_ACCEL_SERV_NA,
					     num_rings_per_srv);
			break;

		default:
			/* unknown service type */
			pr_err("Unknown service type %d, mask 0x%x.\n",
			       serv_type, device->serv_ena_mask);
		}
	}

	return 0;
}

int adf_cfg_init_ring2serv_mapping(struct adf_accel_dev *accel_dev,
				   struct adf_cfg_bundle *bundle,
				   struct adf_cfg_device *device)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct adf_cfg_ring *ring_in_bundle;
	int ring_num = 0;

	bundle->num_of_rings = hw_data->num_rings_per_bank;

	bundle->rings =
		kzalloc(bundle->num_of_rings * sizeof(*ring_in_bundle),
			GFP_KERNEL);
	if (!bundle->rings)
		goto failed;

	for (ring_num = 0; ring_num < bundle->num_of_rings; ring_num++) {
		ring_in_bundle = kzalloc(sizeof(*ring_in_bundle), GFP_KERNEL);
		if (!ring_in_bundle)
			goto failed;
		ring_in_bundle->mode =
			(ring_num < bundle->num_of_rings / 2) ? TX : RX;
		ring_in_bundle->number = ring_num;
		bundle->rings[ring_num] = ring_in_bundle;
	}

	if (adf_cfg_assign_serv_to_rings(bundle, device))
		goto failed;

	return 0;

failed:
	if (bundle->rings) {
		for (ring_num = 0;
		     ring_num < bundle->num_of_rings;
		     ring_num++)
			kfree(bundle->rings[ring_num]);

		kfree(bundle->rings);
	}

	return -ENOMEM;
}

int adf_cfg_rel_ring2serv_mapping(struct adf_cfg_bundle *bundle)
{
	int i = 0;

	if (bundle->rings) {
		for (i = 0; i < bundle->num_of_rings; i++)
			kfree(bundle->rings[i]);

		kfree(bundle->rings);
	}

	return 0;
}
