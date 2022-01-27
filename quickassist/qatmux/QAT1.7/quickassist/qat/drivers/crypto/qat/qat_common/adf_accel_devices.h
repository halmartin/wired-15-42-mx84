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
#ifndef ADF_ACCEL_DEVICES_H_
#define ADF_ACCEL_DEVICES_H_
#ifndef USER_SPACE
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/ratelimit.h>
#include "adf_cfg_common.h"
#endif /* USER_SPACE */

#define ADF_DH895XCC_DEVICE_NAME "dh895xcc"
#define ADF_DH895XCCVF_DEVICE_NAME "dh895xccvf"
#define ADF_C62X_DEVICE_NAME "c6xx"
#define ADF_C62XVF_DEVICE_NAME "c6xxvf"
#define ADF_C3XXX_DEVICE_NAME "c3xxx"
#define ADF_C3XXXVF_DEVICE_NAME "c3xxxvf"
#define ADF_D15XX_DEVICE_NAME "d15xx"
#define ADF_D15XXVF_DEVICE_NAME "d15xxvf"
#define ADF_DH895XCC_PCI_DEVICE_ID 0x435
#define ADF_DH895XCCIOV_PCI_DEVICE_ID 0x443
#define ADF_C62X_PCI_DEVICE_ID 0x37c8
#define ADF_C62XIOV_PCI_DEVICE_ID 0x37c9
#define ADF_C3XXX_PCI_DEVICE_ID 0x19e2
#define ADF_C3XXXIOV_PCI_DEVICE_ID 0x19e3
#define ADF_D15XX_PCI_DEVICE_ID 0x6f54
#define ADF_D15XXIOV_PCI_DEVICE_ID 0x6f55

#if defined(CONFIG_PCI_IOV)
#define ADF_VF2PF_SET_SIZE 32
#define ADF_MAX_VF2PF_SET 4
#define ADF_VF2PF_SET_OFFSET(set_nr) ((set_nr) * ADF_VF2PF_SET_SIZE)
#define ADF_VF2PF_VFNR_TO_SET(vf_nr) ((vf_nr) / ADF_VF2PF_SET_SIZE)
#define ADF_VF2PF_VFNR_TO_MASK(vf_nr) \
	({ \
	u32 vf_nr_ = (vf_nr); \
	BIT((vf_nr_) - ADF_VF2PF_SET_SIZE * ADF_VF2PF_VFNR_TO_SET(vf_nr_)); \
	})
#endif

#define ADF_ADMINMSGUR_OFFSET (0x3A000 + 0x574)
#define ADF_ADMINMSGLR_OFFSET (0x3A000 + 0x578)
#define ADF_MAILBOX_BASE_OFFSET 0x20970
#define ADF_MAILBOX_STRIDE 0x1000
#define ADF_ADMINMSG_LEN 32
#define ADF_DEVICE_FUSECTL_OFFSET 0x40
#define ADF_DEVICE_LEGFUSE_OFFSET 0x4C
#define ADF_DEVICE_FUSECTL_MASK 0x80000000
#define ADF_PCI_MAX_BARS 3
#define ADF_DEVICE_NAME_LENGTH 32
#define ADF_ETR_MAX_RINGS_PER_BANK 16
#define ADF_MAX_MSIX_VECTOR_NAME 32
#define ADF_DEVICE_NAME_PREFIX "qat_"
#define ADF_STOP_RETRY 100
#define ADF_SRV_TYPE_BIT_LEN 3
#define ADF_SRV_TYPE_MASK 0x7
#define ADF_RINGS_PER_SRV_TYPE 2
#define ADF_THRD_ABILITY_BIT_LEN 4
#define ADF_THRD_ABILITY_MASK 0xf

enum adf_accel_capabilities {
	ADF_ACCEL_CAPABILITIES_NULL = 0,
	ADF_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC = 1,
	ADF_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC = 2,
	ADF_ACCEL_CAPABILITIES_CIPHER = 4,
	ADF_ACCEL_CAPABILITIES_AUTHENTICATION = 8,
	ADF_ACCEL_CAPABILITIES_COMPRESSION = 32,
	ADF_ACCEL_CAPABILITIES_LZS_COMPRESSION = 64,
	ADF_ACCEL_CAPABILITIES_RANDOM_NUMBER = 128
};

#ifndef USER_SPACE
struct adf_bar {
	resource_size_t base_addr;
	void __iomem *virt_addr;
	resource_size_t size;
} __packed;

struct adf_irq {
	bool enabled;
	char name[ADF_MAX_MSIX_VECTOR_NAME];
} __packed;

struct adf_accel_msix {
	struct msix_entry *entries;
	struct adf_irq *irqs;
	u32 num_entries;
} __packed;

struct adf_accel_pci {
	struct pci_dev *pci_dev;
	struct adf_accel_msix msix_entries;
	struct adf_bar pci_bars[ADF_PCI_MAX_BARS];
	uint8_t revid;
	uint8_t sku;
} __packed;
#endif /* USER_SPACE */

enum dev_state {
	DEV_DOWN = 0,
	DEV_UP
};

enum dev_sku_info {
	DEV_SKU_1 = 0,
	DEV_SKU_2,
	DEV_SKU_3,
	DEV_SKU_4,
	DEV_SKU_VF,
	DEV_SKU_UNKNOWN,
};

static inline const char *get_sku_info(enum dev_sku_info info)
{
	switch (info) {
	case DEV_SKU_1:
		return "SKU1";
	case DEV_SKU_2:
		return "SKU2";
	case DEV_SKU_3:
		return "SKU3";
	case DEV_SKU_4:
		return "SKU4";
	case DEV_SKU_VF:
		return "SKUVF";
	case DEV_SKU_UNKNOWN:
	default:
		break;
	}
	return "Unknown SKU";
}

#ifndef USER_SPACE
struct adf_hw_device_class {
	const char *name;
	const enum adf_device_type type;
	uint32_t instances;
} __packed;

struct adf_cfg_device_data;
struct adf_accel_dev;
struct adf_etr_data;
struct adf_etr_ring_data;

struct adf_hw_device_data {
	struct adf_hw_device_class *dev_class;
	uint32_t (*get_accel_mask)(struct adf_accel_dev *accel_dev);
	uint32_t (*get_ae_mask)(struct adf_accel_dev *accel_dev);
	uint32_t (*get_sram_bar_id)(struct adf_hw_device_data *self);
	uint32_t (*get_misc_bar_id)(struct adf_hw_device_data *self);
	uint32_t (*get_etr_bar_id)(struct adf_hw_device_data *self);
	uint32_t (*get_num_aes)(struct adf_hw_device_data *self);
	uint32_t (*get_num_accels)(struct adf_hw_device_data *self);
	uint32_t (*get_pf2vf_offset)(uint32_t i);
	uint32_t (*get_vintmsk_offset)(uint32_t i);
	uint32_t (*get_clock_speed)(struct adf_hw_device_data *self);
	enum dev_sku_info (*get_sku)(struct adf_hw_device_data *self);
#if defined(CONFIG_PCI_IOV)
	void (*get_vf2pf_mask)(void __iomem *pmisc_bar_addr,
			       u32 vf_mask_sets[ADF_MAX_VF2PF_SET]);
	void (*enable_vf2pf_interrupts)(void __iomem *pmisc_bar_addr,
					u32 vf_mask_sets, u8 vf2pf_set);
	void (*disable_vf2pf_interrupts)(void __iomem *pmisc_bar_addr,
					 u32 vf_mask_sets, u8 vf2pf_set);
#endif
	int (*alloc_irq)(struct adf_accel_dev *accel_dev);
	void (*free_irq)(struct adf_accel_dev *accel_dev);
	void (*enable_error_correction)(struct adf_accel_dev *accel_dev);
	int (*check_uncorrectable_error)(struct adf_accel_dev *accel_dev);
	void (*print_err_registers)(struct adf_accel_dev *accel_dev);
	void (*disable_error_interrupts)(struct adf_accel_dev *accel_dev);
	int (*init_admin_comms)(struct adf_accel_dev *accel_dev);
	void (*exit_admin_comms)(struct adf_accel_dev *accel_dev);
	int (*send_admin_init)(struct adf_accel_dev *accel_dev);
	int (*get_heartbeat_status)(struct adf_accel_dev *accel_dev);
	uint32_t (*get_ae_clock)(struct adf_hw_device_data *self);
#ifdef QAT_KPT
	int (*enable_kpt)(struct adf_accel_dev *accel_dev);
#endif
	void (*set_asym_rings_mask)(struct adf_accel_dev *accel_dev);
	uint32_t (*get_accel_cap)(struct adf_accel_dev *accel_dev);
	int (*init_arb)(struct adf_accel_dev *accel_dev);
	void (*exit_arb)(struct adf_accel_dev *accel_dev);
	void (*disable_arb)(struct adf_accel_dev *accel_dev);
	void (*get_arb_mapping)(struct adf_accel_dev *accel_dev,
				const uint32_t **cfg);
	void (*disable_iov)(struct adf_accel_dev *accel_dev);
	void (*configure_iov_threads)(struct adf_accel_dev *accel_dev,
				      bool enable);
	void (*enable_ints)(struct adf_accel_dev *accel_dev);
	bool (*check_slice_hang)(struct adf_accel_dev *accel_dev);
	int (*set_ssm_wdtimer)(struct adf_accel_dev *accel_dev);
	int (*enable_vf2pf_comms)(struct adf_accel_dev *accel_dev);
	int (*disable_vf2pf_comms)(struct adf_accel_dev *accel_dev);
	void (*reset_device)(struct adf_accel_dev *accel_dev);
	int (*measure_clock)(struct adf_accel_dev *accel_dev);
	void (*quiesce_device)(struct adf_accel_dev *accel_dev);
	const char *fw_name;
	const char *fw_mmp_name;
	uint32_t fuses;
	uint32_t accel_capabilities_mask;
	uint32_t instance_id;
	uint16_t accel_mask;
	u32 ae_mask;
	uint16_t tx_rings_mask;
	uint8_t tx_rx_gap;
	uint8_t num_banks;
	u8 num_rings_per_bank;
	uint8_t num_accel;
	uint8_t num_logical_accel;
	uint8_t num_engines;
	uint8_t min_iov_compat_ver;
	u32 extended_dc_capabilities;
	u32 clock_frequency;
	int (*config_device)(struct adf_accel_dev *accel_dev);
	u16 asym_rings_mask;
#ifdef QAT_KPT
	u32 kpt_hw_capabilities;
	u32 kpt_achandle;
#endif
} __packed;

/* CSR write macro */
#define ADF_CSR_WR(csr_base, csr_offset, val) \
	__raw_writel(val, csr_base + csr_offset)

/* CSR read macro */
#define ADF_CSR_RD(csr_base, csr_offset) __raw_readl(csr_base + csr_offset)

/* Macro applying a percentage to a value */
#define ADF_APPLY_PERCENTAGE(value, percentage) ((value * percentage) / 100)

#define GET_DEV(accel_dev) ((accel_dev)->accel_pci_dev.pci_dev->dev)
#define GET_BARS(accel_dev) ((accel_dev)->accel_pci_dev.pci_bars)
#define GET_HW_DATA(accel_dev) (accel_dev->hw_device)
#define GET_MAX_BANKS(accel_dev) (GET_HW_DATA(accel_dev)->num_banks)
#define GET_NUM_RINGS_PER_BANK(accel_dev) \
	(GET_HW_DATA(accel_dev)->num_rings_per_bank)
#define GET_MAX_ACCELENGINES(accel_dev) (GET_HW_DATA(accel_dev)->num_engines)
#define accel_to_pci_dev(accel_ptr) accel_ptr->accel_pci_dev.pci_dev
#define ADF_NUM_THREADS_PER_AE (8)
#define ADF_AE_ADMIN_THREAD (7)
#define ADF_NUM_PKE_STRAND (2)
#define ADF_AE_STRAND0_THREAD (8)
#define ADF_AE_STRAND1_THREAD (9)
#define ADF_NUM_HB_CNT_PER_AE (ADF_NUM_THREADS_PER_AE + ADF_NUM_PKE_STRAND)
#define GET_MAX_PROCESSES(accel_dev) \
	({ \
	typeof(accel_dev) dev = (accel_dev); \
	(GET_MAX_BANKS(dev) * (GET_NUM_RINGS_PER_BANK(dev) / 2)); \
	})
#define GET_SRV_TYPE(ena_srv_mask, srv) \
	(((ena_srv_mask) >> (ADF_SRV_TYPE_BIT_LEN * (srv))) & ADF_SRV_TYPE_MASK)
#define SET_ASYM_MASK(asym_mask, srv) \
	({ \
	typeof(srv) srv_ = (srv); \
	(asym_mask) |= \
	((1 << (srv_) * ADF_RINGS_PER_SRV_TYPE) | \
	 (1 << ((srv_) * ADF_RINGS_PER_SRV_TYPE + 1))); \
	})

static inline void adf_csr_fetch_and_and(void __iomem *csr,
					 size_t offs, unsigned long mask)
{
	unsigned int val = ADF_CSR_RD(csr, offs);

	val &= mask;
	ADF_CSR_WR(csr, offs, val);
}

static inline void adf_csr_fetch_and_or(void __iomem *csr,
					size_t offs, unsigned long mask)
{
	unsigned int val = ADF_CSR_RD(csr, offs);

	val |= mask;
	ADF_CSR_WR(csr, offs, val);
}

struct pfvf_stats {
	struct dentry *stats_file;
	/* Messages put in CSR */
	unsigned int tx;
	/* Messages read from CSR */
	unsigned int rx;
	/* Interrupt fired but int bit was clear */
	unsigned int spurious;
	/* Block messages sent */
	unsigned int blk_tx;
	/* Block messages received */
	unsigned int blk_rx;
	/* Blocks received with CRC errors */
	unsigned int crc_err;
	/* CSR in use by other side */
	unsigned int busy;
	/* Receiver did not acknowledge */
	unsigned int no_ack;
	/* Collision detected */
	unsigned int collision;
	/* Couldn't send a response */
	unsigned int tx_timeout;
	/* Didn't receive a response */
	unsigned int rx_timeout;
	/* Responses received */
	unsigned int rx_rsp;
	/* Messages re-transmitted */
	unsigned int retry;
	/* Event put timeout */
	unsigned int event_timeout;
};

#define NUM_PFVF_COUNTERS 14

struct adf_admin_comms {
	dma_addr_t phy_addr;
	dma_addr_t const_tbl_addr;
	dma_addr_t phy_hb_addr;
	void *virt_addr;
	void *virt_tbl_addr;
	void *virt_hb_addr;
	void __iomem *mailbox_addr;
	struct mutex lock;	/* protects adf_admin_comms struct */
};

struct icp_qat_fw_loader_handle;
struct adf_fw_loader_data {
	struct icp_qat_fw_loader_handle *fw_loader;
	const struct firmware *uof_fw;
	const struct firmware *mmp_fw;
};

struct adf_accel_vf_info {
	struct adf_accel_dev *accel_dev;
	struct mutex pf2vf_lock; /* protect CSR access for PF2VF messages */
	struct ratelimit_state vf2pf_ratelimit;
	u32 vf_nr;
	bool init;
	u8 compat_ver;
	struct pfvf_stats pfvf_counters;
};

struct adf_fw_versions {
	u8 fw_version_major;
	u8 fw_version_minor;
	u8 fw_version_patch;
	u8 mmp_version_major;
	u8 mmp_version_minor;
	u8 mmp_version_patch;
};

#define ADF_COMPAT_CHECKER_MAX 8
typedef int (*adf_iov_compat_checker_t)(struct adf_accel_dev *accel_dev,
					u8 vf_compat_ver);
struct adf_accel_compat_manager {
	u8 num_chker;
	adf_iov_compat_checker_t iov_compat_checkers[ADF_COMPAT_CHECKER_MAX];
};

struct adf_heartbeat;
struct adf_ver;
struct adf_uio_control_accel;
struct adf_accel_dev {
	struct adf_etr_data *transport;
	struct adf_hw_device_data *hw_device;
	struct adf_cfg_device_data *cfg;
	struct adf_fw_loader_data *fw_loader;
	struct adf_admin_comms *admin;
	struct adf_uio_control_accel *accel;
	unsigned int num_ker_bundles;
	struct adf_heartbeat *heartbeat;
	struct adf_ver *pver;
	unsigned int autoreset_on_error;
	struct list_head crypto_list;
	unsigned long status;
	atomic_t ref_count;
	struct dentry *debugfs_dir;
	struct dentry *clock_dbgfile;
	struct dentry *fw_cntr_dbgfile;
	struct dentry *cnvnr_dbgfile;
	struct dentry *pfvf_dbgdir;
	struct list_head list;
	struct module *owner;
	struct adf_accel_pci accel_pci_dev;
	struct adf_accel_compat_manager *cm;
	u8 compat_ver;
	struct adf_fw_versions fw_versions;
	union {
		struct {
			/* vf_info is non-zero when SR-IOV is init'ed */
			struct adf_accel_vf_info *vf_info;
		} pf;
		struct {
			bool irq_enabled;
			char *irq_name;
			struct tasklet_struct pf2vf_bh_tasklet;
			struct mutex vf2pf_lock; /* protect CSR access */
			struct completion iov_msg_completion;
			uint8_t compatible;
			uint8_t pf_version;
			u8 pf2vf_block_byte;
			u8 pf2vf_block_resp_type;
			struct pfvf_stats pfvf_counters;
		} vf;
	};
	bool is_vf;
	u32 accel_id;
#ifdef QAT_KPT
	u32 detect_kpt;
#endif
	void *lac_dev;
} __packed;
#endif
#endif
