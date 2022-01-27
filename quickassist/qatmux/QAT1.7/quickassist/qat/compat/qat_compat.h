/*
 * Copyright (C) 2015 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _QAT_COMPAT_H_
#define _QAT_COMPAT_H_

/*
 * If necessary, define LINUX_VERSION_CODE on the command line when compiling
 * against a kernel which does not have this value set as expected.  For
 * example, this value does not get updated in Linus' kernel tree during the
 * merge window until -rc1 is released so this might need to be set manually:
 * make EXTRA_CFLAGS="-DLINUX_VERSION_CODE=nnnnnn" ...
 * where "nnnnnn" is the value generated by the KERNEL_VERSION macro with the
 * desired kernel version numbers.  For example, for kernel version 3.20.0:
 * KERNEL_VERSION(3,20,0) == ((3 << 16) + (20 << 8) + 0) == 201728
 */
#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#else
#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
#endif

#ifndef QAT_AEAD_OLD_SUPPORTED
#if (KERNEL_VERSION(4, 3, 0) > LINUX_VERSION_CODE)
#define QAT_AEAD_OLD_SUPPORTED
#endif
#endif

#ifndef QAT_PKE_OLD_SUPPORTED
#if KERNEL_VERSION(4, 8, 0) > LINUX_VERSION_CODE
#define QAT_PKE_OLD_SUPPORTED
#endif
#endif

#if KERNEL_VERSION(4, 12, 0) > LINUX_VERSION_CODE
#define QAT_PKE_CONST
#else
#define QAT_PKE_CONST const
#endif

#ifndef RHEL_RELEASE_CODE
#define RHEL_RELEASE_CODE 0
#endif
#ifndef RHEL_RELEASE_VERSION
#define RHEL_RELEASE_VERSION(a, b) (((a) << 8) + (b))
#endif

/* SuSE version macro is the same as Linux kernel version */
#ifndef SLE_VERSION
#define SLE_VERSION(a, b, c) KERNEL_VERSION(a, b, c)
#endif
#ifdef CONFIG_SUSE_KERNEL
#if (KERNEL_VERSION(3, 0, 13) == LINUX_VERSION_CODE)
/* SLES11 SP2 is 3.0.13 based */
#define SLE_VERSION_CODE SLE_VERSION(11, 2, 0)
#elif (KERNEL_VERSION(3, 0, 76) == LINUX_VERSION_CODE)
/* SLES11 SP3 is 3.0.76 based */
#define SLE_VERSION_CODE SLE_VERSION(11, 3, 0)
#elif (KERNEL_VERSION(3, 0, 101) == LINUX_VERSION_CODE)
/* SLES11 SP4 is 3.0.101 based */
#define SLE_VERSION_CODE SLE_VERSION(11, 4, 0)
#elif (KERNEL_VERSION(4, 12, 14) <= LINUX_VERSION_CODE)
/* SLES15 is 4.12.14 based */
#define SLE_VERSION_CODE SLE_VERSION(15, 0, 0)
#elif (KERNEL_VERSION(3, 12, 0) <= LINUX_VERSION_CODE)
/* SLES12 GA is 3.12.y based */
#define SLE_VERSION_CODE SLE_VERSION(12, 0, 0)
#endif /* LINUX_VERSION_CODE == KERNEL_VERSION(x,y,z) */
#endif /* CONFIG_SUSE_KERNEL */
#ifndef SLE_VERSION_CODE
#define SLE_VERSION_CODE 0
#endif /* SLE_VERSION_CODE */

#if !(SLE_VERSION_CODE && SLE_VERSION(15, 0, 0) == SLE_VERSION_CODE)
#if KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE
#define QAT_PKE_MAX_SZ_SIGN
#else
#define QAT_PKE_MAX_SZ_SIGN unsigned
#endif
#endif

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/crypto.h>

/*
 * The driver is only supported on longterm/stable (listed on
 * http://www.kernel.org), RHEL6.x, 2.6.38 and newer kernels.
 */
#if (KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE && \
	!(KERNEL_VERSION(2, 6, 32) == LINUX_VERSION_CODE) && \
	!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 0) <= RHEL_RELEASE_CODE))
#error Driver is not supported on this kernel
#endif

#include <linux/ratelimit.h>

/******************************************************************************/
#if (KERNEL_VERSION(2, 6, 33) > LINUX_VERSION_CODE)
#define alloc_workqueue(fmt, flags, max_active, args...) create_workqueue(fmt)

#if !(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 0) <= RHEL_RELEASE_CODE)
static inline int pci_pcie_cap(struct pci_dev *dev)
{
	/* NOTE: differs from 2.6.33 implementation */
	return pci_find_capability(dev, PCI_CAP_ID_EXP);
}
#endif /* !RHEL6.x */

#if !(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 1) <= RHEL_RELEASE_CODE)
static inline bool pci_is_pcie(struct pci_dev *dev)
{
	return !!pci_pcie_cap(dev);
}
#endif /* RHEL6.1 */
#endif /* 2.6.33 */

/******************************************************************************/
#if (KERNEL_VERSION(2, 6, 34) > LINUX_VERSION_CODE)
#define __round_mask(x, y) ((__typeof__(x))((y)-1))
#define round_up(x, y) ((((x)-1) | __round_mask(x, y))+1)
#define for_each_set_bit(bit, addr, size) \
	for ((bit) = find_first_bit((addr), (size)); \
		(bit) < (size); \
		(bit) = find_next_bit((addr), (size), (bit) + 1))

#if (!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 0) <= RHEL_RELEASE_CODE))
static inline int pci_num_vf(struct pci_dev __maybe_unused *dev)
{
	int num_vf = 0;
#ifdef CONFIG_PCI_IOV
	struct pci_dev *vfdev;

	/* loop through all Co-processor devices starting at PF dev */
	vfdev = pci_get_class(PCI_CLASS_PROCESSOR_CO << 8, NULL);
	while (vfdev) {
		if (vfdev->is_virtfn && vfdev->physfn == dev)
			num_vf++;

		vfdev = pci_get_class(PCI_CLASS_PROCESSOR_CO << 8, vfdev);
	}
#endif
	return num_vf;
}
#endif /* !(RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6, 0)) */
#endif /* 2.6.34 */

/******************************************************************************/
#if (KERNEL_VERSION(2, 6, 35) > LINUX_VERSION_CODE)
#if !(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 0) <= RHEL_RELEASE_CODE)
#define irq_set_affinity_hint(irq, mask)	do {} while (0)
#endif /* !RHEL6.x */

#ifndef pr_warn
#define pr_warn pr_warning
#endif

static inline void ratelimit_state_init(struct ratelimit_state *rs,
					int interval, int burst)
{
#if (KERNEL_VERSION(2, 6, 32) < LINUX_VERSION_CODE)
	spin_lock_init(&rs->lock);
#endif
	rs->interval = interval;
	rs->burst = burst;
	rs->printed = 0;
	rs->missed = 0;
	rs->begin = 0;
}
#endif /* 2.6.35 */

/******************************************************************************/
#if (KERNEL_VERSION(2, 6, 39) > LINUX_VERSION_CODE && \
	!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 4) <= RHEL_RELEASE_CODE))
#include <linux/ctype.h>
int __must_check _kstrtoul(const char *s, unsigned int base,
			   unsigned long *res);
int __must_check kstrtoull(const char *s, unsigned int base,
			   unsigned long long *res);
static inline int __must_check kstrtoul(const char *s, unsigned int base,
					unsigned long *res)
{
	/*
	 * We want to shortcut function call, but
	 * __builtin_types_compatible_p(unsigned long, unsigned long long) = 0.
	 */
	if (sizeof(unsigned long) == sizeof(unsigned long long) &&
	    __alignof__(unsigned long) == __alignof__(unsigned long long))
		return kstrtoull(s, base, (unsigned long long *)res);
	else
		return _kstrtoul(s, base, res);
}
int __must_check kstrtouint(const char *s, unsigned int base,
			    unsigned int *res);
#endif /* 2.6.39 && !(RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6,4)) */

/******************************************************************************/
#if (KERNEL_VERSION(3, 2, 0) > LINUX_VERSION_CODE)
#if (!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 4) <= RHEL_RELEASE_CODE) &&\
	!(SLE_VERSION_CODE && SLE_VERSION(11, 4, 0) <= SLE_VERSION_CODE))
#include <linux/dma-mapping.h>
static inline void *dma_zalloc_coherent(struct device *dev, size_t size,
					dma_addr_t *dma_handle, gfp_t flag)
{
	void *ret = dma_alloc_coherent(dev, size, dma_handle, flag);

	if (ret)
		memset(ret, 0, size);
	return ret;
}
#endif /* !(RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6,4)) */

#define iommu_present(b)	iommu_found()

#endif /* 3.2.0 */

/******************************************************************************/
#if (KERNEL_VERSION(5, 0, 0) > LINUX_VERSION_CODE)

#undef dma_alloc_coherent
#define dma_alloc_coherent(d, s, h, f) dma_zalloc_coherent(d, s, h, f)
#endif /* 5.0.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE)
#include <linux/crypto.h>
int crypto_register_algs(struct crypto_alg *algs, int count);
int crypto_unregister_algs(struct crypto_alg *algs, int count);

#if ((KERNEL_VERSION(3, 2, 50) >= LINUX_VERSION_CODE || \
	KERNEL_VERSION(3, 3, 0) <= LINUX_VERSION_CODE) && \
	(!(RHEL_RELEASE_CODE && \
		RHEL_RELEASE_VERSION(6, 4) <= RHEL_RELEASE_CODE) && \
	!(SLE_VERSION_CODE && \
		SLE_VERSION(11, 4, 0) <= SLE_VERSION_CODE)))

static inline void *kmalloc_array(size_t n, size_t size, gfp_t flags)
{
	if (size != 0 && n > ULONG_MAX / size)
		return NULL;
	return __kmalloc(n * size, flags);
}
#endif /* !((RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6,4)) || 3.2.50->3.3) */
#endif /* 3.4.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 6, 0) > LINUX_VERSION_CODE)
#define PCI_EXP_LNKCAP2		44	/* Link Capability 2 */
#endif /* 3.6.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 7, 0) > LINUX_VERSION_CODE)
#include <linux/scatterlist.h>

int sg_nents(struct scatterlist *sg);

#define pci_pcie_type(x)	((x)->pcie_type)
#define PCI_EXP_LNKSTA2		50	/* Link Status 2 */

int pcie_capability_read_word(struct pci_dev *dev, int pos, u16 *val);
#endif /* 3.7.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 8, 0) > LINUX_VERSION_CODE)
#define NO_SRIOV_CONFIGURE

#if (!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 5) <= RHEL_RELEASE_CODE) &&\
	!(SLE_VERSION_CODE && SLE_VERSION(11, 4, 0) <= SLE_VERSION_CODE))
static inline int pci_sriov_get_totalvfs(struct pci_dev *dev)
{
#ifdef CONFIG_PCI_IOV
	u16 totalvfs;
	int pos;

	if (!dev->is_physfn)
		return -EINVAL;

	pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_SRIOV);
	if (!pos)
		return -ENODEV;

	pci_read_config_word(dev, pos + PCI_SRIOV_TOTAL_VF, &totalvfs);
	return totalvfs;
#else
	return 0;
#endif
}
#endif /* !(RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6, 5)) */
#endif /* 3.8.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 12, 0) > LINUX_VERSION_CODE)
#if !(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(7, 0) <= RHEL_RELEASE_CODE)
int pci_wait_for_pending_transaction(struct pci_dev *dev);
#endif /* !RHEL7.x */

#ifdef CONFIG_HAVE_IOREMAP_PROT
#undef CONFIG_HAVE_IOREMAP_PROT
#endif

#endif /* 3.12.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 13, 0) > LINUX_VERSION_CODE)
#if !(RHEL_RELEASE_CODE && \
	(RHEL_RELEASE_VERSION(6, 8) <= RHEL_RELEASE_CODE) && \
	(RHEL_RELEASE_VERSION(7, 0) != RHEL_RELEASE_CODE))
#include <linux/rtnetlink.h>
#include <crypto/authenc.h>

struct crypto_authenc_keys {
	const u8 *authkey;
	const u8 *enckey;

	unsigned int authkeylen;
	unsigned int enckeylen;
};

int crypto_authenc_extractkeys(struct crypto_authenc_keys *keys, const u8 *key,
			       unsigned int keylen);
#endif /* !RHEL7.1+ && SUSE12.1 */
#ifndef GENMASK_ULL
#define GENMASK_ULL(h, l)	(((U64_C(1) << ((h) - (l) + 1)) - 1) << (l))
#endif /* GENMASK_ULL */

#if (SLE_VERSION_CODE && SLE_VERSION(12, 0, 0) == SLE_VERSION_CODE)
#define ratelimit_state_init(s, i, b)	ratelimit_state_init(s, i, b, 0)
#endif /* SLES12 */
#endif /* 3.13.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 14, 0) > LINUX_VERSION_CODE)
#if !(RHEL_RELEASE_CODE && \
	(RHEL_RELEASE_VERSION(6, 8) <= RHEL_RELEASE_CODE) && \
	(RHEL_RELEASE_VERSION(7, 0) != RHEL_RELEASE_CODE))
#ifdef CONFIG_PCI_MSI
static inline int pci_enable_msix_exact(struct pci_dev *dev,
					struct msix_entry *entries, int nvec)
{
	int rc;

	do {
		rc = pci_enable_msix(dev, entries, nvec);
		if (rc < 0) {
			return rc;
		} else if (rc > 0) {
			if (rc < nvec)
				return -ENOSPC;
			nvec = rc;
		}
	} while (rc);

	return rc;
}
#else
static inline int pci_enable_msix_exact(struct pci_dev *dev,
                                        struct msix_entry *entries, int nvec)
{ return -ENOSYS; }
#endif /* CONFIG_PCI_MSI */
#endif /* !RHEL7.1+ */

#ifndef OPTIMIZER_HIDE_VAR
/* Make the optimizer believe the variable can be manipulated arbitrarily. */
#ifdef __GNUC__
#define OPTIMIZER_HIDE_VAR(var) __asm__("" : "=r" (var) : "0" (var))
#else
#define OPTIMIZER_HIDE_VAR(var) barrier()
#endif
#endif
#endif /* 3.14.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 17, 3) > LINUX_VERSION_CODE && \
	!(KERNEL_VERSION(3, 14, 24) <= LINUX_VERSION_CODE && \
	KERNEL_VERSION(3, 15, 0) > LINUX_VERSION_CODE) && \
	!(KERNEL_VERSION(3, 12, 33) <= LINUX_VERSION_CODE && \
	KERNEL_VERSION(3, 13, 0) > LINUX_VERSION_CODE) && \
	!(KERNEL_VERSION(3, 10, 60) <= LINUX_VERSION_CODE && \
	KERNEL_VERSION(3, 11, 0) > LINUX_VERSION_CODE) && \
	!(KERNEL_VERSION(3, 2, 65) <= LINUX_VERSION_CODE && \
	KERNEL_VERSION(3, 3, 0) > LINUX_VERSION_CODE))
void memzero_explicit(void *s, size_t count);
#endif /* 3.17.3, etc. */

/******************************************************************************/
#if (KERNEL_VERSION(3, 18, 0) > LINUX_VERSION_CODE)
#define SHASH_DESC_ON_STACK(shash, ctx)	\
	char __##shash##_desc[sizeof(struct shash_desc) +		\
		crypto_shash_descsize(ctx)] CRYPTO_MINALIGN_ATTR;	\
	struct shash_desc *shash = (struct shash_desc *)__##shash##_desc

#if (!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(7, 3) <= RHEL_RELEASE_CODE))
struct uio_device {
	struct module *owner;
	struct device *dev;
	int minor;
	atomic_t event;
	struct fasync_struct *async_queue;
	wait_queue_head_t wait;
#if (KERNEL_VERSION(3, 12, 0) > LINUX_VERSION_CODE)
	int vma_count;
#endif
	struct uio_info *info;
	struct kobject *map_dir;
	struct kobject *portio_dir;
};
#endif /* RHEL_RELEASE_VERSION(7.3) */
#endif /* 3.18.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 19, 0) > LINUX_VERSION_CODE)
#ifndef MODULE_ALIAS_CRYPTO
#define MODULE_ALIAS_CRYPTO	MODULE_ALIAS
#endif
#endif /* 3.19.0 */

#if (KERNEL_VERSION(4, 2, 0) > LINUX_VERSION_CODE)
#if (!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(7, 3) <= RHEL_RELEASE_CODE) &&\
	!(RHEL_RELEASE_CODE && \
		RHEL_RELEASE_VERSION(6, 8) == RHEL_RELEASE_CODE) &&\
	!(RHEL_RELEASE_CODE && \
		RHEL_RELEASE_VERSION(6, 9) == RHEL_RELEASE_CODE))
static inline void crypto_aead_set_reqsize(struct crypto_aead *aead,
					   unsigned int reqsize)
{
	crypto_aead_crt(aead)->reqsize = reqsize;
}
#endif /* RHEL_RELEASE_VERSION(7.3) */
#endif /* 4.2.0 */

#ifndef list_next_entry
#define list_next_entry(pos, member) \
	list_entry((pos)->member.next, typeof(*(pos)), member)
#endif /* list_next_entry */

#if (KERNEL_VERSION(2, 6, 36) > LINUX_VERSION_CODE)
#define DEFINE_SEMAPHORE  DECLARE_MUTEX
#endif  /* 2.6.36 */
#if (KERNEL_VERSION(4, 3, 0) > LINUX_VERSION_CODE)
void seq_hex_dump(struct seq_file *m, const char *prefix_str, int prefix_type,
		  int rowsize, int groupsize, const void *buf, size_t len,
		  bool ascii);
#endif  /* 4.2.0 */

#if (KERNEL_VERSION(3, 2, 0) > LINUX_VERSION_CODE)
#define iommu_present(b) iommu_found()
#endif

#if ((KERNEL_VERSION(3, 16, 4) > LINUX_VERSION_CODE) && \
	(!RHEL_RELEASE_CODE || RHEL_RELEASE_VERSION(7, 1) == RHEL_RELEASE_CODE))
static inline void pci_ignore_hotplug(struct pci_dev *dev)
{
	/* Not supported in RHEL7.1 or Linux kernel before 3.16.4 */
}
#endif /* RHEL7.1 */

#if (RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(7, 3) <= RHEL_RELEASE_CODE)
#define QAT_KPT_CAP_DISCOVERY
#endif
#endif /* _QAT_COMPAT_H_ */
