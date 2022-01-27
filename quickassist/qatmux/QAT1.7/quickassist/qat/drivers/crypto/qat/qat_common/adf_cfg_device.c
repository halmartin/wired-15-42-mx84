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
#include "adf_cfg_device.h"

int adf_cfg_get_ring_pairs(struct adf_cfg_device *device,
			   struct adf_cfg_instance *inst,
			   const char *process_name,
			   struct adf_accel_dev *accel_dev)
{
	int i = 0;
	int ret = -EFAULT;
	struct adf_cfg_instance *free_inst = NULL;
	struct adf_cfg_bundle *first_free_bundle = NULL;
	enum adf_cfg_bundle_type free_bundle_type;
	int first_user_bundle = 0;

	dev_dbg(&GET_DEV(accel_dev),
		"get ring pair for section %s, bundle_num is %d.\n",
				process_name, device->bundle_num);

	/* Section of user process with poll mode */
	if (strcmp(ADF_KERNEL_SEC, process_name) &&
	    strcmp(ADF_KERNEL_SAL_SEC, process_name) &&
	    (inst->polling_mode == ADF_CFG_RESP_POLL)) {
		first_user_bundle = device->max_kernel_bundle_nr + 1;
		for (i = first_user_bundle; i < device->bundle_num; i++) {
			free_inst =
				adf_cfg_get_free_instance(device,
							  device->bundles[i],
							  inst,
							  process_name);

			if (!free_inst)
				continue;

			ret = adf_cfg_get_ring_pairs_from_bundle(
					device->bundles[i], inst,
					process_name, free_inst);
			return ret;
		}
	} else {
		/* Section of in-tree, or kernel API or user process
		 * with epoll mode
		 */
		if (!strcmp(ADF_KERNEL_SEC, process_name) ||
		    !strcmp(ADF_KERNEL_SAL_SEC, process_name))
			free_bundle_type = KERNEL;
		else
			free_bundle_type = USER;

		for (i = 0; i < device->bundle_num; i++) {
			/* Since both in-tree and kernel API's bundle type
			 * are kernel, use cpumask_subset to check if the
			 * ring's affinity mask is a subset of a bundle's
			 * one.
			 */
			if ((free_bundle_type == device->bundles[i]->type) &&
			    cpumask_subset(
					&inst->affinity_mask,
					&device->bundles[i]->affinity_mask)) {
				free_inst =
					adf_cfg_get_free_instance(
							device,
							device->bundles[i],
							inst,
							process_name);

				if (!free_inst)
					continue;

				ret = adf_cfg_get_ring_pairs_from_bundle(
							device->bundles[i],
							inst,
							process_name,
							free_inst);

				return ret;

			} else if (!first_free_bundle &&
				   adf_cfg_is_free(device->bundles[i])) {
				first_free_bundle = device->bundles[i];
			}
		}

		if (first_free_bundle) {
			free_inst = adf_cfg_get_free_instance(device,
							      first_free_bundle,
							      inst,
							      process_name);

			if (!free_inst)
				return ret;

			ret = adf_cfg_get_ring_pairs_from_bundle(
					first_free_bundle, inst,
					process_name, free_inst);

			if (free_bundle_type == KERNEL) {
				device->max_kernel_bundle_nr =
					first_free_bundle->number;
			}
			return ret;
		}
	}
	pr_err("Don't have enough rings for instance %s in process %s\n",
	       inst->name, process_name);

	return ret;
}

int adf_cfg_get_token_val_str(struct adf_accel_dev *accel_dev,
			      char *val, char *tokens[], int *token_num)
{
	char *ptr = val;
	char *token = NULL;
	char *str = NULL;
	int i = 0;

	for (i = 0; i < ADF_CFG_MAX_TOKENS_IN_CONFIG; i++) {
		token = kzalloc(ADF_CFG_MAX_KEY_LEN_IN_BYTES, GFP_KERNEL);
		if (!token)
			return -ENOMEM;
		str = token;
		while ('\0' != *ptr) {
			if (';' == *ptr) {
				*token = '\0';
				tokens[i] = str;
				ptr++;
				break;
			}
			*token = *ptr;
			token++;
			ptr++;
		}
		if ('\0' == *ptr) {
			*token = '\0';
			tokens[i] = str;
			break;
		}
	}
	*token_num = i + 1;
	dev_dbg(&GET_DEV(accel_dev),
		"ptr is %s, tokens[0] %s, tokens[1] %s, token_num %d.\n",
		val, tokens[0], tokens[1], *token_num);
	return 0;
}

int adf_cfg_get_user_section(struct adf_accel_dev *accel_dev,
			     char **user_sec_list,
			     int *user_sec_num)
{
	struct adf_cfg_device_data *cfg = accel_dev->cfg;
	struct list_head *list;
	struct adf_cfg_section *section = NULL;
	char *user_sec = NULL;

	list_for_each(list, &cfg->sec_list) {
		section = list_entry(list, struct adf_cfg_section, list);
		if (strcmp(section->name, ADF_GENERAL_SEC) &&
		    strcmp(section->name, ADF_KERNEL_SEC) &&
		    strcmp(section->name, ADF_KERNEL_SAL_SEC) &&
		    strcmp(section->name, ADF_ACCEL_SEC)) {
			user_sec = kzalloc(ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
					   GFP_KERNEL);
			if (!user_sec)
				return -ENOMEM;

			dev_dbg(&GET_DEV(accel_dev),
				"user section %s\n", section->name);
			strlcpy(user_sec,
				section->name,
				ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
			user_sec_list[*user_sec_num] = user_sec;
			(*user_sec_num) += 1;
		}
	}

	return 0;
}

int adf_cfg_get_def_serv_mask(struct adf_accel_dev *accel_dev,
			      u16 *def_serv_mask)
{
	(*def_serv_mask) = CRYPTO;
	(*def_serv_mask) |= CRYPTO << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
	(*def_serv_mask) |= NA << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
	(*def_serv_mask) |= COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT;

	return 0;
}

int adf_cfg_get_serv_ena_mask(struct adf_accel_dev *accel_dev,
			      char *tokens[],
			      int token_num,
			      u16 num_cy_inst,
			      u16 num_dc_inst,
			      u16 *serv_ena_mask)
{
	int t_ind = 0;
	u8 col_cy_inst = 0;
	u8 col_dc_inst = 0;
	u16 bundle_num = accel_dev->hw_device->num_banks;

	*serv_ena_mask = 0;

	col_cy_inst =
		num_cy_inst ? ((num_cy_inst - 1) / (2 * bundle_num) + 1) : 0;
	col_dc_inst =
		num_dc_inst ? ((num_dc_inst - 1) / (2 * bundle_num) + 1) : 0;

	for (t_ind = 0; t_ind < token_num; t_ind++) {
		if (strncmp(tokens[t_ind],
			    ADF_CFG_CY,
			    strlen(ADF_CFG_CY)) == 0) {
			if (col_cy_inst > 0 &&
			    (col_cy_inst + col_dc_inst > 3)) {
				dev_err(&GET_DEV(accel_dev),
					"number of inst overflow cy %d, dc %d.\n",
					num_cy_inst, num_dc_inst);
				goto failed;
			}
			switch (col_cy_inst) {
			case 0:
				break;
			case 1:
				*serv_ena_mask |= CRYPTO;
				*serv_ena_mask |=
				CRYPTO << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				break;
			case 2:
				*serv_ena_mask |= CRYPTO;
				*serv_ena_mask |=
				CRYPTO << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				*serv_ena_mask |=
				CRYPTO << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
				*serv_ena_mask |=
				CRYPTO << ADF_CFG_SERV_RING_PAIR_3_SHIFT;
				break;
			default:
				dev_err(&GET_DEV(accel_dev),
					"number of cy inst overflow %d.\n",
					num_cy_inst);
				goto failed;
			}
		} else if (strncmp(tokens[t_ind],
				   ADF_CFG_DC,
				   strlen(ADF_CFG_DC)) == 0) {
			switch (col_dc_inst) {
			case 0:
				break;
			case 1:
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT;
				break;
			case 2:
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT;
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
				break;
			case 3:
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT;
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				break;
			case 4:
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_3_SHIFT;
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
				*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				*serv_ena_mask |= COMP;
				break;
			default:
				dev_err(&GET_DEV(accel_dev),
					"number of dc inst overflow %d.\n",
					num_dc_inst);
				goto failed;
			}
		} else if (strncmp(tokens[t_ind],
				   ADF_CFG_ASYM,
				   strlen(ADF_CFG_ASYM)) == 0) {
			if (col_cy_inst > 0 &&
			    (col_cy_inst + col_dc_inst > 4)) {
				dev_err(&GET_DEV(accel_dev),
					"number of inst overflow asym %d, dc %d.\n",
					num_cy_inst, num_dc_inst);
				goto failed;
			}
			switch (col_cy_inst) {
			case 0:
				break;
			case 1:
				*serv_ena_mask |= ASYM;
				if (col_dc_inst == 3)
					*serv_ena_mask |=
					COMP << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				break;
			case 2:
				*serv_ena_mask |= ASYM;
				*serv_ena_mask |=
					ASYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				break;
			case 3:
				*serv_ena_mask |= ASYM;
				*serv_ena_mask |=
					ASYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				*serv_ena_mask |=
					ASYM << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
				break;
			case 4:
				*serv_ena_mask |= ASYM;
				*serv_ena_mask |=
					ASYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				*serv_ena_mask |=
					ASYM << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
				*serv_ena_mask |=
					ASYM << ADF_CFG_SERV_RING_PAIR_3_SHIFT;
				break;
			default:
				dev_err(&GET_DEV(accel_dev),
					"number of asym inst overflow %d.\n",
					num_cy_inst);
				goto failed;
			}
		} else if (strncmp(tokens[t_ind],
				   ADF_CFG_SYM,
				   strlen(ADF_CFG_SYM)) == 0) {
			if (col_cy_inst > 0 &&
			    (col_cy_inst + col_dc_inst) > 4) {
				dev_err(&GET_DEV(accel_dev),
					"number of inst overflow sym %d, dc %d.\n",
					num_cy_inst, num_dc_inst);
				goto failed;
			}
			switch (col_cy_inst) {
			case 0:
				break;
			case 1:
				*serv_ena_mask |=
					SYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				if (col_dc_inst == 3)
					*serv_ena_mask |= COMP;
				break;
			case 2:
				*serv_ena_mask |=
					SYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				*serv_ena_mask |= SYM;
				break;
			case 3:
				*serv_ena_mask |=
					SYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				*serv_ena_mask |= SYM;
				*serv_ena_mask |=
					SYM << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
				break;
			case 4:
				*serv_ena_mask |=
					SYM << ADF_CFG_SERV_RING_PAIR_1_SHIFT;
				*serv_ena_mask |= SYM;
				*serv_ena_mask |=
					SYM << ADF_CFG_SERV_RING_PAIR_2_SHIFT;
				*serv_ena_mask |=
					SYM << ADF_CFG_SERV_RING_PAIR_3_SHIFT;
				break;
			default:
				dev_err(&GET_DEV(accel_dev),
					"number of sym inst overflow %d.\n",
					num_cy_inst);
				goto failed;
			}
		} else {
			dev_err(&GET_DEV(accel_dev),
				"Unknown token %s to ServicesEnabled variables.\n",
				tokens[t_ind]);
			goto failed;
		}
	}
	return 0;

failed:
	return -EFAULT;
}

int adf_cfg_chk_serv_ena_tokens(struct adf_accel_dev *accel_dev,
				char *tokens[],
				int token_num)
{
	int i = 0;
	bool cy_enabled = false;
	bool dc_enabled = false;
	bool asym_enabled = false;
	bool sym_enabled = false;
	u32 capabilities = GET_HW_DATA(accel_dev)->accel_capabilities_mask;

	for (i = 0; i < token_num; i++) {
		if (strncmp(tokens[i],
			    ADF_CFG_CY,
			    strlen(ADF_CFG_CY)) == 0) {
			cy_enabled = true;
			continue;
		}
		if (strncmp(tokens[i],
			    ADF_CFG_DC,
			    strlen(ADF_CFG_DC)) == 0) {
			dc_enabled = true;
			continue;
		}
		if (strncmp(tokens[i],
			    ADF_CFG_ASYM,
			    strlen(ADF_CFG_ASYM)) == 0) {
			asym_enabled = true;
			continue;
		}
		if (strncmp(tokens[i],
			    ADF_CFG_SYM,
			    strlen(ADF_CFG_SYM)) == 0)
			sym_enabled = true;
	}

	dev_dbg(&GET_DEV(accel_dev),
		"ServicesEnabled %s%s%s%s.\n",
		cy_enabled ? "cy," : "",
		dc_enabled ? "dc," : "",
		asym_enabled ? "asym," : "",
		sym_enabled ? "sym," : "");

	if (!(capabilities & ADF_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC &&
	      capabilities & ADF_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC) &&
	    cy_enabled) {
		dev_err(&GET_DEV(accel_dev),
			"Device does not support cy service\n");
		goto failed;
	}
	if (!(capabilities & ADF_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC) &&
	    sym_enabled) {
		dev_err(&GET_DEV(accel_dev),
			"Device does not support sym service\n");
		goto failed;
	}
	if (!(capabilities & ADF_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC) &&
	    asym_enabled) {
		dev_err(&GET_DEV(accel_dev),
			"Device does not support asym service\n");
		goto failed;
	}
	if (!(capabilities & ADF_ACCEL_CAPABILITIES_COMPRESSION) &&
	    dc_enabled) {
		dev_err(&GET_DEV(accel_dev),
			"Device does not support dc service\n");
		goto failed;
	}
	if (cy_enabled && asym_enabled) {
		dev_err(&GET_DEV(accel_dev),
			"ServicesEnabled var cy and asym cannot co-exist.\n");
		goto failed;
	}
	if (cy_enabled && sym_enabled) {
		dev_err(&GET_DEV(accel_dev),
			"ServicesEnabled var cy and sym cannot co-exist.\n");
		goto failed;
	}
	if (asym_enabled && sym_enabled) {
		dev_err(&GET_DEV(accel_dev),
			"ServicesEnabled var asym and sym cannot co-exist.\n");
		goto failed;
	}

	return 0;

failed:
	return -EFAULT;
}

int adf_cfg_get_services_enabled(struct adf_accel_dev *accel_dev,
				 u16 *serv_ena_mask)
{
	char *key = NULL;
	char *val = NULL;
	int token_num = 0;
	char *tokens[ADF_CFG_MAX_NUM_OF_TOKENS];
	u16 num_intr_inst = 0;
	u16 num_cy_inst = 0;
	u16 num_dc_inst = 0;
	u16 bundle_num = accel_dev->hw_device->num_banks;
	int ret = -ENOMEM;
	int i = 0;

	*serv_ena_mask = 0;

	key = kzalloc(ADF_CFG_MAX_KEY_LEN_IN_BYTES, GFP_KERNEL);
	if (!key)
		goto failed;

	val = kzalloc(ADF_CFG_MAX_VAL_LEN_IN_BYTES, GFP_KERNEL);
	if (!val)
		goto failed;

	strlcpy(key, ADF_SERVICES_ENABLED, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
	if (adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC, key, val))
		goto failed;

	dev_dbg(&GET_DEV(accel_dev), "services enabled string is %s.\n", val);

	ret = adf_cfg_get_token_val_str(accel_dev, val, tokens, &token_num);
	if (ret)
		goto failed;

	ret = adf_cfg_chk_serv_ena_tokens(accel_dev, tokens, token_num);
	if (ret)
		goto failed;

	if (accel_dev->is_vf) {
		adf_cfg_get_def_serv_mask(accel_dev, serv_ena_mask);
		dev_dbg(&GET_DEV(accel_dev),
			"using default service mask 0x%x due to virtualization.\n",
			*serv_ena_mask);
		ret = 0;
		goto failed;
	}

	ret = adf_cfg_get_intr_inst(accel_dev, &num_intr_inst);
	if (ret)
		goto failed;

	dev_dbg(&GET_DEV(accel_dev),
		"number of interrupt instances %d.\n",
		num_intr_inst);

	if (num_intr_inst > 0) {
		adf_cfg_get_def_serv_mask(accel_dev, serv_ena_mask);
		dev_dbg(&GET_DEV(accel_dev),
			"using default service mask 0x%x due to interrupt/epoll mode instances.\n",
			*serv_ena_mask);
		ret = 0;
		goto failed;
	}

	ret = adf_cfg_get_num_of_inst(accel_dev, &num_cy_inst, &num_dc_inst);
	if (ret)
		goto failed;

	dev_dbg(&GET_DEV(accel_dev),
		"number of cy inst %d, number of dc inst %d.\n",
		num_cy_inst, num_dc_inst);

	if (num_intr_inst > 0 ||
	    (num_cy_inst <= 2 * bundle_num && num_dc_inst <= 2 * bundle_num)) {
		adf_cfg_get_def_serv_mask(accel_dev, serv_ena_mask);
		dev_dbg(&GET_DEV(accel_dev),
			"using default service mask 0x%x due to number of instances.\n",
			*serv_ena_mask);
		ret = 0;
		goto failed;
	}

	ret = adf_cfg_get_serv_ena_mask(accel_dev,
					tokens,
					token_num,
					num_cy_inst,
					num_dc_inst,
					serv_ena_mask);
	if (ret)
		goto failed;

	ret = 0;
failed:
	kfree(val);
	kfree(key);

	for (i = 0; i < token_num; i++)
		kfree(tokens[i]);

	dev_dbg(&GET_DEV(accel_dev),
		"Get enabled services 0x%x in config file.\n",
		*serv_ena_mask);

	return ret;
}

int adf_cfg_get_num_of_inst(struct adf_accel_dev *accel_dev,
			    u16 *num_cy_inst,
			    u16 *num_dc_inst)
{
	char *key = NULL;
	char *val = NULL;
	char **user_sec_l = NULL;
	int  user_sec_n = 0;

	unsigned long num_inst = 0;
	unsigned long num_proc = 0;
	int ret = -ENOMEM;
	int i = 0;

	*num_cy_inst = 0;
	*num_dc_inst = 0;

	key = kzalloc(ADF_CFG_MAX_KEY_LEN_IN_BYTES, GFP_KERNEL);
	if (!key)
		goto failed;

	val = kzalloc(ADF_CFG_MAX_VAL_LEN_IN_BYTES, GFP_KERNEL);
	if (!val)
		goto failed;

	user_sec_l = kcalloc(ADF_CFG_MAX_NUM_OF_SECTIONS,
			     sizeof(char *),
			     GFP_KERNEL);
	if (!user_sec_l)
		goto failed;

	ret = -EFAULT;
	strlcpy(key, ADF_NUM_CY, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
	if (adf_cfg_get_param_value(accel_dev, ADF_KERNEL_SEC, key, val))
		goto failed;

	if (kstrtoul(val, 0, &num_inst))
		goto failed;

	(*num_cy_inst) += num_inst;

	strlcpy(key, ADF_NUM_DC, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
	if (adf_cfg_get_param_value(accel_dev, ADF_KERNEL_SEC, key, val))
		goto failed;

	if (kstrtoul(val, 0, &num_inst))
		goto failed;

	(*num_dc_inst) += num_inst;

	if (adf_cfg_get_user_section(accel_dev, user_sec_l, &user_sec_n))
		goto failed;

	for (i = 0; i < user_sec_n; i++) {
		strlcpy(key, ADF_NUM_CY, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
		if (adf_cfg_get_param_value(accel_dev, user_sec_l[i], key, val))
			goto failed;

		if (kstrtoul(val, 0, &num_inst))
			goto failed;

		strlcpy(key, ADF_NUM_PROCESSES, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
		if (adf_cfg_get_param_value(accel_dev, user_sec_l[i], key, val))
			num_proc = 0;
		else if (kstrtoul(val, 0, &num_proc))
			goto failed;

		(*num_cy_inst) += num_inst * num_proc;

		strlcpy(key, ADF_NUM_DC, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
		if (adf_cfg_get_param_value(accel_dev, user_sec_l[i], key, val))
			goto failed;

		if (kstrtoul(val, 0, &num_inst))
			goto failed;

		(*num_dc_inst) += num_inst * num_proc;
	} /* for loop */

	ret = 0;

failed:
	kfree(key);
	kfree(val);
	if (user_sec_l) {
		for (i = 0; i < user_sec_n; i++)
			kfree(user_sec_l[i]);
		kfree(user_sec_l);
	}
	dev_dbg(&GET_DEV(accel_dev),
		"get number of instances in config file. ret %d\n",
		ret);

	return ret;
}

int adf_cfg_get_intr_inst_in_kernel_section(struct adf_accel_dev *accel_dev,
					    char *key,
					    char *val,
					    u16 *num_of_intr_inst)
{
	int ret = -EFAULT;
	int i = 0;
	unsigned long num_inst = 0;
	unsigned long polling_mode = 0;

	strlcpy(key, ADF_NUM_CY, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
	if (adf_cfg_get_param_value(accel_dev, ADF_KERNEL_SEC, key, val))
		goto failed;

	if (kstrtoul(val, 0, &num_inst))
		goto failed;

	for (i = 0; i < num_inst; i++) {
		snprintf(key, ADF_CFG_MAX_KEY_LEN_IN_BYTES,
			 ADF_CY_POLL_MODE_FORMAT, i);
		if (adf_cfg_get_param_value(accel_dev, ADF_KERNEL_SEC,
					    key, val))
			goto failed;
		dev_dbg(&GET_DEV(accel_dev), "%s is %s\n", key, val);
		if (kstrtoul(val, 0, &polling_mode))
			goto failed;
		if (polling_mode != 1) {
			(*num_of_intr_inst) += 1;
			ret = 0;
			goto failed;
		}
	}

	strlcpy(key, ADF_NUM_DC, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
	if (adf_cfg_get_param_value(accel_dev, ADF_KERNEL_SEC, key, val))
		goto failed;

	if (kstrtoul(val, 0, &num_inst))
		goto failed;

	for (i = 0; i < num_inst; i++) {
		snprintf(key, ADF_CFG_MAX_KEY_LEN_IN_BYTES,
			 ADF_DC_POLL_MODE_FORMAT, i);
		if (adf_cfg_get_param_value(accel_dev, ADF_KERNEL_SEC,
					    key, val))
			goto failed;
		dev_dbg(&GET_DEV(accel_dev), "%s is %s\n", key, val);
		if (kstrtoul(val, 0, &polling_mode))
			goto failed;
		if (polling_mode != 1) {
			(*num_of_intr_inst) += 1;
			ret = 0;
			goto failed;
		}
	}

	ret = 0;
failed:
	return ret;
}

int adf_cfg_get_intr_inst_in_user_sections(struct adf_accel_dev *accel_dev,
					   char **user_sec_list,
					   int user_sec_num,
					   char *key,
					   char *val,
					   u16 *num_of_intr_inst)
{
	int ret = -EFAULT;
	int i = 0;
	int j = 0;
	unsigned long num_inst = 0;
	unsigned long polling_mode = 0;

	for (j = 0; j < user_sec_num; j++) {
		strlcpy(key, ADF_NUM_CY, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
		if (adf_cfg_get_param_value(accel_dev, user_sec_list[j],
					    key, val))
			goto failed;

		if (kstrtoul(val, 0, &num_inst))
			goto failed;

		for (i = 0; i < num_inst; i++) {
			snprintf(key, ADF_CFG_MAX_KEY_LEN_IN_BYTES,
				 ADF_CY_POLL_MODE_FORMAT, i);
			if (adf_cfg_get_param_value(accel_dev, user_sec_list[j],
						    key, val))
				goto failed;
			dev_dbg(&GET_DEV(accel_dev), "%s is %s\n", key, val);
			if (kstrtoul(val, 0, &polling_mode))
				goto failed;
			if (polling_mode != 1) {
				(*num_of_intr_inst) += 1;
				ret = 0;
				goto failed;
			}
		}

		strlcpy(key, ADF_NUM_DC, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
		if (adf_cfg_get_param_value(accel_dev,
					    user_sec_list[j], key, val))
			goto failed;

		if (kstrtoul(val, 0, &num_inst))
			goto failed;

		for (i = 0; i < num_inst; i++) {
			snprintf(key, ADF_CFG_MAX_KEY_LEN_IN_BYTES,
				 ADF_DC_POLL_MODE_FORMAT, i);
			if (adf_cfg_get_param_value(accel_dev, user_sec_list[j],
						    key, val))
				goto failed;
			dev_dbg(&GET_DEV(accel_dev), "%s is %s\n", key, val);
			if (kstrtoul(val, 0, &polling_mode))
				goto failed;
			if (polling_mode != 1) {
				(*num_of_intr_inst) += 1;
				ret = 0;
				goto failed;
			}
		}
	} /* for loop j */

	ret = 0;
failed:
	return ret;
}

int adf_cfg_get_intr_inst(struct adf_accel_dev *accel_dev,
			  u16 *num_of_intr_inst)
{
	char *key = NULL;
	char *val = NULL;
	char **user_sec_l = NULL;
	int  user_sec_n = 0;
	int j = 0;
	int ret = -ENOMEM;
	*num_of_intr_inst = 0;

	key = kzalloc(ADF_CFG_MAX_KEY_LEN_IN_BYTES, GFP_KERNEL);
	if (!key)
		goto failed;

	val = kzalloc(ADF_CFG_MAX_VAL_LEN_IN_BYTES, GFP_KERNEL);
	if (!val)
		goto failed;

	user_sec_l = kcalloc(ADF_CFG_MAX_NUM_OF_SECTIONS,
			     sizeof(char *),
			     GFP_KERNEL);
	if (!user_sec_l)
		goto failed;

	ret = adf_cfg_get_intr_inst_in_kernel_section(accel_dev,
						      key,
						      val,
						      num_of_intr_inst);
	if (ret || (*num_of_intr_inst))
		goto failed;

	ret = adf_cfg_get_user_section(accel_dev, user_sec_l, &user_sec_n);
	if (ret)
		goto failed;

	ret = adf_cfg_get_intr_inst_in_user_sections(accel_dev,
						     user_sec_l,
						     user_sec_n,
						     key,
						     val,
						     num_of_intr_inst);
	if (ret)
		goto failed;

failed:
	kfree(key);
	kfree(val);
	if (user_sec_l) {
		for (j = 0; j < user_sec_n; j++)
			kfree(user_sec_l[j]);
		kfree(user_sec_l);
	}

	return ret;
}

void adf_cfg_set_asym_rings_mask(struct adf_accel_dev *accel_dev)
{
	int service;
	u16 ena_srv_mask;
	u16 service_type;
	u16 asym_mask = 0;
	struct adf_cfg_device *cfg_dev = accel_dev->cfg->dev;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;

	if (!cfg_dev) {
		hw_data->asym_rings_mask = ADF_CFG_DEF_ASYM_MASK;
		return;
	}

	ena_srv_mask = cfg_dev->serv_ena_mask;

	/* parse each service */
	for (service = 0;
	     service < ADF_CFG_MAX_SERVICES;
	     service++) {
		service_type =
			GET_SRV_TYPE(ena_srv_mask, service);
		switch (service_type) {
		case CRYPTO:
		case ASYM:
			SET_ASYM_MASK(asym_mask, service);
			if (service_type == CRYPTO)
				service++;
			break;
		}
	}

	hw_data->asym_rings_mask = asym_mask;
}
EXPORT_SYMBOL_GPL(adf_cfg_set_asym_rings_mask);

void adf_cfg_gen_dispatch_arbiter(struct adf_accel_dev *accel_dev,
				  const u32 *thrd_to_arb_map,
				  u32 *thrd_to_arb_map_gen,
				  u32 total_engines)
{
	int engine;
	int thread;
	int service;
	int bits;
	u32 thread_ability;
	u32 ability_map;
	u32 service_mask;
	u16 ena_srv_mask;
	u16 service_type;
	struct adf_cfg_device *device = accel_dev->cfg->dev;

	if (!device) {
		/* if not set, return the default dispatch arbiter */
		for (engine = 0;
		     engine < total_engines;
		     engine++) {
			thrd_to_arb_map_gen[engine] = thrd_to_arb_map[engine];
		}
		return;
	}

	ena_srv_mask = device->serv_ena_mask;

	for (engine = 0; engine < total_engines; engine++) {
		bits = 0;
		/* ability_map is used to indicate the threads ability */
		ability_map = thrd_to_arb_map[engine];
		thrd_to_arb_map_gen[engine] = 0;
		/* parse each thread on the engine */
		for (thread = 0;
		     thread < ADF_NUM_THREADS_PER_AE;
		     thread++) {
			/* get the ability of this thread */
			thread_ability = ability_map & ADF_THRD_ABILITY_MASK;
			ability_map >>= ADF_THRD_ABILITY_BIT_LEN;
			/* parse each service */
			for (service = 0;
			     service < ADF_CFG_MAX_SERVICES;
			     service++) {
				service_type =
					GET_SRV_TYPE(ena_srv_mask, service);
				switch (service_type) {
				case CRYPTO:
					service_mask = ADF_CFG_ASYM_SRV_MASK;
					if (thread_ability & service_mask)
						thrd_to_arb_map_gen[engine] |=
								(1 << bits);
					bits++;
					service++;
					service_mask = ADF_CFG_SYM_SRV_MASK;
					break;
				case COMP:
					service_mask = ADF_CFG_DC_SRV_MASK;
					break;
				case SYM:
					service_mask = ADF_CFG_SYM_SRV_MASK;
					break;
				case ASYM:
					service_mask = ADF_CFG_ASYM_SRV_MASK;
					break;
				default:
					service_mask = ADF_CFG_UNKNOWN_SRV_MASK;
				}
				if (thread_ability & service_mask)
					thrd_to_arb_map_gen[engine] |=
								(1 << bits);
				bits++;
			}
		}
	}
}
EXPORT_SYMBOL_GPL(adf_cfg_gen_dispatch_arbiter);

static int update_accel_cap_mask(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	unsigned long pke_disabled = 0;
	unsigned long storage_enabled = 0;
	char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES];

	if (!adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				     ADF_PKE_DISABLED, val)) {
		if (kstrtoul(val, 0, &pke_disabled))
			return -EFAULT;
	}

	if (!adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				     ADF_STORAGE_FIRMWARE_ENABLED, val)) {
		if (kstrtoul(val, 0, &storage_enabled))
			return -EFAULT;
	}

	if (hw_data->get_accel_cap) {
		hw_data->accel_capabilities_mask =
			hw_data->get_accel_cap(accel_dev);
	}

	if (pke_disabled || storage_enabled) {
		hw_data->accel_capabilities_mask &=
			~ADF_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
	}
	return 0;
}

int adf_cfg_device_init(struct adf_cfg_device *device,
			struct adf_accel_dev *accel_dev)
{
	int i = 0;
	/* max_inst indicates the max instance number one bank can hold */
	int max_inst = accel_dev->hw_device->tx_rx_gap;
	int ret = -ENOMEM;

	device->bundle_num = 0;
	device->bundles = (struct adf_cfg_bundle **)
		kzalloc(sizeof(struct adf_cfg_bundle *)
			* accel_dev->hw_device->num_banks,
			GFP_KERNEL);
	if (!device->bundles)
		goto failed;

	device->bundle_num = accel_dev->hw_device->num_banks;

	device->instances = (struct adf_cfg_instance **)
		kzalloc(sizeof(struct adf_cfg_instance *)
			* device->bundle_num * max_inst,
			GFP_KERNEL);
	if (!device->instances)
		goto failed;

	device->instance_index = 0;

	device->max_kernel_bundle_nr = -1;

	dev_dbg(&GET_DEV(accel_dev), "init device with bundle information\n");

	ret = -EFAULT;

	/* Update the acceleration capability mask based on User capability */
	if (update_accel_cap_mask(accel_dev))
		goto failed;

	/* parse and get value of ServicesEnabled */
	if (adf_cfg_get_services_enabled(accel_dev, &device->serv_ena_mask))
		goto failed;

	ret = -ENOMEM;
	/*
	 * 1) get the config information to generate the ring to service
	 *    mapping table
	 * 2) init each bundle of this device
	 */
	for (i = 0; i < device->bundle_num; i++) {
		device->bundles[i] =
			kzalloc(sizeof(struct adf_cfg_bundle), GFP_KERNEL);
		if (!device->bundles[i])
			goto failed;

		device->bundles[i]->max_section = max_inst;
		adf_cfg_bundle_init(device->bundles[i], device, i, accel_dev);
	}

	return 0;

failed:
	for (i = 0; i < device->bundle_num; i++) {
		if (device->bundles[i])
			adf_cfg_bundle_clear(device->bundles[i], accel_dev);
	}

	for (i = 0; i < (device->bundle_num * max_inst); i++) {
		if (device->instances && device->instances[i])
			kfree(device->instances[i]);
	}

	kfree(device->instances);
	device->instances = NULL;

	dev_err(&GET_DEV(accel_dev), "Failed to do device init\n");
	return ret;
}

void adf_cfg_device_clear(struct adf_cfg_device *device,
			  struct adf_accel_dev *accel_dev)
{
	int i = 0;

	dev_dbg(&GET_DEV(accel_dev), "clear device with bundle information\n");
	for (i = 0; i < device->bundle_num; i++) {
		if (device->bundles && device->bundles[i]) {
			adf_cfg_bundle_clear(device->bundles[i], accel_dev);
			kfree(device->bundles[i]);
			device->bundles[i] = NULL;
		}
	}

	kfree(device->bundles);
	device->bundles = NULL;

	for (i = 0; i < device->instance_index; i++) {
		if (device->instances && device->instances[i]) {
			kfree(device->instances[i]);
			device->instances[i] = NULL;
		}
	}

	kfree(device->instances);
	device->instances = NULL;
}

int adf_config_device(struct adf_accel_dev *accel_dev)
{
	struct adf_cfg_device_data *cfg = NULL;
	struct adf_cfg_device *cfg_device = NULL;
	struct adf_cfg_section *sec;
	struct list_head *list;
	int ret = -ENOMEM;

	if (!accel_dev)
		return ret;

	cfg = accel_dev->cfg;
	cfg->dev = NULL;
	cfg_device = (struct adf_cfg_device *)
			kzalloc(sizeof(*cfg_device), GFP_KERNEL);
	if (!cfg_device)
		goto failed;

	ret = -EFAULT;

	if (adf_cfg_device_init(cfg_device, accel_dev))
		goto failed;

	cfg->dev = cfg_device;

	/* GENERAL and KERNEL section must be processed before others */
	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		if (!strcmp(sec->name, ADF_GENERAL_SEC)) {
			dev_dbg(&GET_DEV(accel_dev), "Process section %s\n",
				sec->name);
			ret = adf_cfg_process_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
			sec->processed = true;
			break;
		}
	}

	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		if (!strcmp(sec->name, ADF_KERNEL_SEC)) {
			dev_dbg(&GET_DEV(accel_dev), "Process section %s\n",
				sec->name);
			ret = adf_cfg_process_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
			sec->processed = true;
			break;
		}
	}

	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		if (!strcmp(sec->name, ADF_KERNEL_SAL_SEC)) {
			dev_dbg(&GET_DEV(accel_dev), "Process section %s\n",
				sec->name);
			ret = adf_cfg_process_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
			sec->processed = true;
			break;
		}
	}

	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		/* avoid reprocessing one section */
		if (!sec->processed && !sec->is_derived) {
			dev_dbg(&GET_DEV(accel_dev), "Process section %s\n",
				sec->name);
			ret = adf_cfg_process_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
			sec->processed = true;
		}
	}

	/* newly added accel section */
	ret = adf_cfg_process_section(accel_dev,
				      ADF_ACCEL_SEC,
				      accel_dev->accel_id);
	if (ret)
		goto failed;

	/*
	 * put item-remove task after item-process
	 * because during process we may fetch values from those items
	 */
	list_for_each(list, &cfg->sec_list) {
		sec = list_entry(list, struct adf_cfg_section, list);
		if (!sec->is_derived) {
			dev_dbg(&GET_DEV(accel_dev), "Clean up section %s\n",
				sec->name);
			ret = adf_cfg_cleanup_section(accel_dev,
						      sec->name,
						      accel_dev->accel_id);
			if (ret)
				goto failed;
		}
	}

	ret = 0;
failed:
	if (ret) {
		if (cfg_device) {
			adf_cfg_device_clear(cfg_device, accel_dev);
			kfree(cfg_device);
			cfg->dev = NULL;
		}
		adf_cfg_del_all(accel_dev);
		dev_err(&GET_DEV(accel_dev), "Failed to config device\n");
	}

	return ret;
}
EXPORT_SYMBOL_GPL(adf_config_device);
