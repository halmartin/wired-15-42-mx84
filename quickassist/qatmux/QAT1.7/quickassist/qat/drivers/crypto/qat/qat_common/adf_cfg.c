/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2014 Intel Corporation.
  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  qat-linux@intel.com

  BSD LICENSE
  Copyright(c) 2014 Intel Corporation.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "adf_accel_devices.h"
#include "adf_cfg.h"
#include "adf_common_drv.h"
#include "adf_cfg_dev_dbg.h"
#include "adf_heartbeat_dbg.h"
#include "adf_ver_dbg.h"
#include "adf_fw_counters.h"
#include "adf_cnvnr_freq_counters.h"

/**
 * adf_cfg_dev_add() - Create an acceleration device configuration table.
 * @accel_dev:  Pointer to acceleration device.
 *
 * Function creates a configuration table for the given acceleration device.
 * The table stores device specific config values.
 * To be used by QAT device specific drivers.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_cfg_dev_add(struct adf_accel_dev *accel_dev)
{
	struct adf_cfg_device_data *dev_cfg_data;

	dev_cfg_data = kzalloc(sizeof(*dev_cfg_data), GFP_KERNEL);
	if (!dev_cfg_data)
		return -ENOMEM;
	INIT_LIST_HEAD(&dev_cfg_data->sec_list);
	init_rwsem(&dev_cfg_data->lock);
	accel_dev->cfg = dev_cfg_data;

	if (adf_cfg_dev_dbg_add(accel_dev))
		goto err;

	if (!accel_dev->is_vf) {
		if (adf_heartbeat_dbg_add(accel_dev))
			goto err;

		if (adf_ver_dbg_add(accel_dev))
			goto err;

		if (adf_fw_counters_add(accel_dev))
			goto err;

		if (adf_cnvnr_freq_counters_add(accel_dev))
			goto err;
	}

	return 0;

err:
	kfree(dev_cfg_data);
	accel_dev->cfg = NULL;
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(adf_cfg_dev_add);

static void adf_cfg_section_del_all(struct list_head *head);

void adf_cfg_del_all(struct adf_accel_dev *accel_dev)
{
	struct adf_cfg_device_data *dev_cfg_data = accel_dev->cfg;

	down_write(&dev_cfg_data->lock);
	adf_cfg_section_del_all(&dev_cfg_data->sec_list);
	up_write(&dev_cfg_data->lock);
	clear_bit(ADF_STATUS_CONFIGURED, &accel_dev->status);
}

/**
 * adf_cfg_dev_remove() - Clears acceleration device configuration table.
 * @accel_dev:  Pointer to acceleration device.
 *
 * Function removes configuration table from the given acceleration device
 * and frees all allocated memory.
 * To be used by QAT device specific drivers.
 *
 * Return: void
 */
void adf_cfg_dev_remove(struct adf_accel_dev *accel_dev)
{
	struct adf_cfg_device_data *dev_cfg_data = accel_dev->cfg;

	if (!dev_cfg_data)
		return;

	down_write(&dev_cfg_data->lock);
	adf_cfg_section_del_all(&dev_cfg_data->sec_list);
	up_write(&dev_cfg_data->lock);

	adf_cfg_dev_dbg_remove(accel_dev);
	if (!accel_dev->is_vf) {
		adf_ver_dbg_del(accel_dev);
		adf_heartbeat_dbg_del(accel_dev);
		adf_fw_counters_remove(accel_dev);
		adf_cnvnr_freq_counters_remove(accel_dev);
	}
	kfree(dev_cfg_data);
	accel_dev->cfg = NULL;
}
EXPORT_SYMBOL_GPL(adf_cfg_dev_remove);

static void adf_cfg_keyval_add(struct adf_cfg_key_val *new,
			       struct adf_cfg_section *sec)
{
	list_add_tail(&new->list, &sec->param_head);
}

static void adf_cfg_keyval_remove(const char *key,
				  struct adf_cfg_section *sec)
{
	struct list_head *list_ptr, *tmp;
	struct list_head *head = &sec->param_head;

	list_for_each_prev_safe(list_ptr, tmp, head) {
		struct adf_cfg_key_val *ptr =
			list_entry(list_ptr, struct adf_cfg_key_val, list);

		if (strcmp(ptr->key, key) != 0)
			continue;

		list_del(list_ptr);
		kfree(ptr);
		break;
	}
}

/**
 * adf_cfg_section_del() - Delete config section entry to config table.
 * @accel_dev:  Pointer to acceleration device.
 * @name: Name of the section
 *
 * Function deletes configuration section where key - value entries
 * will be stored.
 * To be used by QAT device specific drivers.
 */
static void adf_cfg_section_del(struct adf_accel_dev *accel_dev,
				const char *name)
{
	struct adf_cfg_section *sec = adf_cfg_sec_find(accel_dev, name);

	if (!sec)
		return;
	adf_cfg_keyval_del_all(&sec->param_head);
	list_del(&sec->list);
	kfree(sec);
}

void adf_cfg_keyval_del_all(struct list_head *head)
{
	struct list_head *list_ptr, *tmp;

	list_for_each_prev_safe(list_ptr, tmp, head) {
		struct adf_cfg_key_val *ptr =
			list_entry(list_ptr, struct adf_cfg_key_val, list);
		list_del(list_ptr);
		kfree(ptr);
	}
}

static void adf_cfg_section_del_all(struct list_head *head)
{
	struct adf_cfg_section *ptr;
	struct list_head *list, *tmp;

	list_for_each_prev_safe(list, tmp, head) {
		ptr = list_entry(list, struct adf_cfg_section, list);
		adf_cfg_keyval_del_all(&ptr->param_head);
		list_del(list);
		kfree(ptr);
	}
}

static struct adf_cfg_key_val *adf_cfg_key_value_find(struct adf_cfg_section *s,
						      const char *key)
{
	struct list_head *list;

	list_for_each(list, &s->param_head) {
		struct adf_cfg_key_val *ptr =
			list_entry(list, struct adf_cfg_key_val, list);
		if (!strcmp(ptr->key, key))
			return ptr;
	}
	return NULL;
}

struct adf_cfg_section *adf_cfg_sec_find(struct adf_accel_dev *accel_dev,
						const char *sec_name)
{
	struct adf_cfg_device_data *cfg = accel_dev->cfg;
	struct list_head *list;

	list_for_each(list, &cfg->sec_list) {
		struct adf_cfg_section *ptr =
			list_entry(list, struct adf_cfg_section, list);
		if (!strcmp(ptr->name, sec_name))
			return ptr;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(adf_cfg_sec_find);

static int adf_cfg_key_val_get(struct adf_accel_dev *accel_dev,
			       const char *sec_name,
			       const char *key_name,
			       char *val)
{
	struct adf_cfg_section *sec = adf_cfg_sec_find(accel_dev, sec_name);
	struct adf_cfg_key_val *keyval = NULL;

	if (sec)
		keyval = adf_cfg_key_value_find(sec, key_name);
	if (keyval) {
		memcpy(val, keyval->val, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
		return 0;
	}
	return -1;
}

/**
 * adf_cfg_add_key_value_param() - Add key-value config entry to config table.
 * @accel_dev:  Pointer to acceleration device.
 * @section_name: Name of the section where the param will be added
 * @key: The key string
 * @val: Value pain for the given @key
 * @type: Type - string, int or address
 *
 * Function adds configuration key - value entry in the appropriate section
 * in the given acceleration device
 * To be used by QAT device specific drivers.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_cfg_add_key_value_param(struct adf_accel_dev *accel_dev,
				const char *section_name,
				const char *key, const void *val,
				enum adf_cfg_val_type type)
{
	char temp_val[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
	struct adf_cfg_device_data *cfg = accel_dev->cfg;
	struct adf_cfg_key_val *key_val;
	struct adf_cfg_section *section = adf_cfg_sec_find(accel_dev,
							   section_name);
	if (!section)
		return -EFAULT;

	key_val = kzalloc(sizeof(*key_val), GFP_KERNEL);
	if (!key_val)
		return -ENOMEM;

	INIT_LIST_HEAD(&key_val->list);
	strlcpy(key_val->key, key, sizeof(key_val->key));

	if (type == ADF_DEC) {
		snprintf(key_val->val, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
			 "%ld", (*((long *)val)));
	} else if (type == ADF_STR) {
		strlcpy(key_val->val, (char *)val, sizeof(key_val->val));
	} else if (type == ADF_HEX) {
		snprintf(key_val->val, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
			 "0x%lx", (unsigned long)val);
	} else {
		dev_err(&GET_DEV(accel_dev), "Unknown type given.\n");
		kfree(key_val);
		return -1;
	}
	key_val->type = type;

	/* Add the key-value pair as below policy:
	 *     1. If the key doesn't exist, add it,
	 *     2. If the key already exists with a different value
	 *        then delete it,
	 *     3. If the key exists with the same value, then return
	 *        without doing anything.
	 */
	if (adf_cfg_key_val_get(accel_dev, section_name, key, temp_val) == 0) {
		if (strcmp(temp_val, key_val->val) != 0) {
			adf_cfg_keyval_remove(key, section);
		} else {
			kfree(key_val);
			return 0;
		}
	}

	down_write(&cfg->lock);
	adf_cfg_keyval_add(key_val, section);
	up_write(&cfg->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(adf_cfg_add_key_value_param);

/**
 * adf_cfg_remove_key_param() - remove config entry in config table.
 * @accel_dev:  Pointer to acceleration device.
 * @section_name: Name of the section where the param will be added
 * @key: The key string
 *
 * Function remove configuration key
 * To be used by QAT device specific drivers.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_cfg_remove_key_param(struct adf_accel_dev *accel_dev,
			     const char *section_name,
			     const char *key)
{
	struct adf_cfg_device_data *cfg = accel_dev->cfg;
	struct adf_cfg_section *section = adf_cfg_sec_find(accel_dev,
		section_name);
	if (!section)
		return -EFAULT;

	down_write(&cfg->lock);
	adf_cfg_keyval_remove(key, section);
	up_write(&cfg->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(adf_cfg_remove_key_param);

/**
 * adf_cfg_section_add() - Add config section entry to config table.
 * @accel_dev:  Pointer to acceleration device.
 * @name: Name of the section
 *
 * Function adds configuration section where key - value entries
 * will be stored.
 * To be used by QAT device specific drivers.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_cfg_section_add(struct adf_accel_dev *accel_dev, const char *name)
{
	struct adf_cfg_device_data *cfg = accel_dev->cfg;
	struct adf_cfg_section *sec = adf_cfg_sec_find(accel_dev, name);

	if (sec)
		return 0;

	sec = kzalloc(sizeof(*sec), GFP_KERNEL);
	if (!sec)
		return -ENOMEM;

	strlcpy(sec->name, name, sizeof(sec->name));
	INIT_LIST_HEAD(&sec->param_head);
	down_write(&cfg->lock);
	list_add_tail(&sec->list, &cfg->sec_list);
	up_write(&cfg->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(adf_cfg_section_add);

/* need to differentiate derived section with the original section */
int adf_cfg_derived_section_add(struct adf_accel_dev *accel_dev,
				const char *name)
{
	struct adf_cfg_device_data *cfg = accel_dev->cfg;
	struct adf_cfg_section *sec = NULL;

	if (adf_cfg_section_add(accel_dev, name))
		return -EFAULT;

	sec = adf_cfg_sec_find(accel_dev, name);
	if (!sec)
		return -EFAULT;

	down_write(&cfg->lock);
	sec->is_derived = true;
	up_write(&cfg->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(adf_cfg_derived_section_add);

static int adf_cfg_restore_key_value_param(struct adf_accel_dev *accel_dev,
					   const char *section_name,
					   const char *key, const char *val,
					   enum adf_cfg_val_type type)
{
	struct adf_cfg_device_data *cfg = accel_dev->cfg;
	struct adf_cfg_key_val *key_val;
	struct adf_cfg_section *section = adf_cfg_sec_find(accel_dev,
							   section_name);
	if (!section)
		return -EFAULT;

	key_val = kzalloc(sizeof(*key_val), GFP_KERNEL);
	if (!key_val)
		return -ENOMEM;

	INIT_LIST_HEAD(&key_val->list);

	strlcpy(key_val->key, key, sizeof(key_val->key));
	strlcpy(key_val->val, val, sizeof(key_val->val));
	key_val->type = type;
	down_write(&cfg->lock);
	adf_cfg_keyval_add(key_val, section);
	up_write(&cfg->lock);
	return 0;
}

int adf_cfg_save_section(struct adf_accel_dev *accel_dev,
			 const char *name,
			 struct adf_cfg_section *section)
{
	int ret = 0;
	struct adf_cfg_key_val *ptr;
	struct adf_cfg_section *sec = adf_cfg_sec_find(accel_dev, name);

	if (!sec) {
		dev_err(&GET_DEV(accel_dev), "Couldn't find section %s\n",
			name);
		return -EFAULT;
	}

	strlcpy(section->name, name, sizeof(section->name));
	INIT_LIST_HEAD(&section->param_head);

	/* now we save all the parameters */
	list_for_each_entry(ptr, &sec->param_head, list) {
		struct adf_cfg_key_val *key_val;

		key_val = kzalloc(sizeof(*key_val), GFP_KERNEL);
		if (!key_val) {
			ret = -ENOMEM;
			goto err_save;
		}

		memcpy(key_val, ptr, sizeof(*key_val));
		list_add_tail(&key_val->list, &section->param_head);
	}
	return 0;

err_save:
	adf_cfg_keyval_del_all(&section->param_head);
	return ret;
}
EXPORT_SYMBOL_GPL(adf_cfg_save_section);

int adf_cfg_restore_section(struct adf_accel_dev *accel_dev,
			    struct adf_cfg_section *section)
{
	struct adf_cfg_key_val *ptr;
	int ret = 0;

	ret = adf_cfg_section_add(accel_dev, section->name);
	if (ret)
		goto err;

	list_for_each_entry(ptr, &section->param_head, list) {
		ret = adf_cfg_restore_key_value_param(accel_dev,
						      section->name,
						      ptr->key,
						      ptr->val,
						      ptr->type);
		if (ret)
			goto err_remove_sec;
	}
	return 0;

err_remove_sec:
	adf_cfg_section_del(accel_dev, section->name);
err:
	dev_err(&GET_DEV(accel_dev), "Failed to restore section %s\n",
		section->name);
	return ret;
}
EXPORT_SYMBOL_GPL(adf_cfg_restore_section);


int adf_cfg_get_param_value(struct adf_accel_dev *accel_dev,
			    const char *section, const char *name,
			    char *value)
{
	struct adf_cfg_device_data *cfg = accel_dev->cfg;
	int ret;

	down_read(&cfg->lock);
	ret = adf_cfg_key_val_get(accel_dev, section, name, value);
	up_read(&cfg->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(adf_cfg_get_param_value);
