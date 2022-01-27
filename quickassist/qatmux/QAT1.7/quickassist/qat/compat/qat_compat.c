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
#include "qat_compat.h"

/******************************************************************************/
#if (KERNEL_VERSION(2, 6, 39) > LINUX_VERSION_CODE && \
	!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 4) <= RHEL_RELEASE_CODE))
static inline char _tolower(const char c)
{
	return c | 0x20;
}

static int _kstrtoull(const char *s, unsigned int base, unsigned long long *res)
{
	unsigned long long acc;
	int ok;

	if (base == 0) {
		if (s[0] == '0') {
			if (_tolower(s[1]) == 'x' && isxdigit(s[2]))
				base = 16;
			else
				base = 8;
		} else
			base = 10;
	}
	if (base == 16 && s[0] == '0' && _tolower(s[1]) == 'x')
		s += 2;

	acc = 0;
	ok = 0;
	while (*s) {
		unsigned int val;

		if ('0' <= *s && *s <= '9')
			val = *s - '0';
		else if ('a' <= _tolower(*s) && _tolower(*s) <= 'f')
			val = _tolower(*s) - 'a' + 10;
		else if (*s == '\n' && *(s + 1) == '\0')
			break;
		else
			return -EINVAL;

		if (val >= base)
			return -EINVAL;
		if (acc > div_u64(ULLONG_MAX - val, base))
			return -ERANGE;
		acc = acc * base + val;
		ok = 1;

		s++;
	}
	if (!ok)
		return -EINVAL;
	*res = acc;
	return 0;
}

int kstrtoull(const char *s, unsigned int base, unsigned long long *res)
{
	if (s[0] == '+')
		s++;
	return _kstrtoull(s, base, res);
}

int _kstrtoul(const char *s, unsigned int base, unsigned long *res)
{
	unsigned long long tmp;
	int rv;

	rv = kstrtoull(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (tmp != (unsigned long long)(unsigned long)tmp)
		return -ERANGE;
	*res = tmp;
	return 0;
}

int kstrtouint(const char *s, unsigned int base, unsigned int *res)
{
	unsigned long long tmp;
	int rv;

	rv = kstrtoull(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (tmp != (unsigned long long)(unsigned int)tmp)
		return -ERANGE;
	*res = tmp;
	return 0;
}
#endif /* 2.6.39 && !(RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6,4)) */

/******************************************************************************/
#if (KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE)
int crypto_register_algs(struct crypto_alg *algs, int count)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = crypto_register_alg(&algs[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	for (--i; i >= 0; --i)
		crypto_unregister_alg(&algs[i]);

	return ret;
}

int crypto_unregister_algs(struct crypto_alg *algs, int count)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = crypto_unregister_alg(&algs[i]);
		if (ret)
			pr_err("Failed to unregister %s %s: %d\n",
		algs[i].cra_driver_name, algs[i].cra_name, ret);
	}

	return 0;
}
#endif /* 3.4.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 7, 0) > LINUX_VERSION_CODE)
static inline int pcie_cap_version(struct pci_dev *dev)
{
	int pos;
	u16 reg16;

	pos = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (!pos)
		return 0;
	pci_read_config_word(dev, pos + PCI_EXP_FLAGS, &reg16);
	return reg16 & PCI_EXP_FLAGS_VERS;
}

#define pcie_cap_has_devctl(type, flags)	1
#define pcie_cap_has_lnkctl(type, flags)	\
	((flags & PCI_EXP_FLAGS_VERS) > 1 ||	\
	(type == PCI_EXP_TYPE_ROOT_PORT ||	\
	type == PCI_EXP_TYPE_ENDPOINT ||	\
	type == PCI_EXP_TYPE_LEG_END))
#define pcie_cap_has_sltctl(type, flags)	\
	((flags & PCI_EXP_FLAGS_VERS) > 1 ||	\
	((type == PCI_EXP_TYPE_ROOT_PORT) ||	\
	(type == PCI_EXP_TYPE_DOWNSTREAM &&	\
	(flags & PCI_EXP_FLAGS_SLOT))))
#define pcie_cap_has_rtctl(type, flags)		\
	((flags & PCI_EXP_FLAGS_VERS) > 1 ||	\
	(type == PCI_EXP_TYPE_ROOT_PORT ||	\
	type == PCI_EXP_TYPE_RC_EC))

#if !(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(6, 5) <= RHEL_RELEASE_CODE)
static bool pcie_capability_reg_implemented(struct pci_dev *dev, int pos)
{
	u16 flags;
	int type;

	if (!pci_is_pcie(dev))
		return false;

	pci_read_config_word(dev, pos + PCI_EXP_FLAGS, &flags);
	type = pci_pcie_type(dev);

	switch (pos) {
	case PCI_EXP_FLAGS_TYPE:
		return true;
	case PCI_EXP_DEVCAP:
	case PCI_EXP_DEVCTL:
	case PCI_EXP_DEVSTA:
		return pcie_cap_has_devctl(type, flags);
	case PCI_EXP_LNKCAP:
	case PCI_EXP_LNKCTL:
	case PCI_EXP_LNKSTA:
		return pcie_cap_has_lnkctl(type, flags);
	case PCI_EXP_SLTCAP:
	case PCI_EXP_SLTCTL:
	case PCI_EXP_SLTSTA:
		return pcie_cap_has_sltctl(type, flags);
	case PCI_EXP_RTCTL:
	case PCI_EXP_RTCAP:
	case PCI_EXP_RTSTA:
		return pcie_cap_has_rtctl(type, flags);
	case PCI_EXP_DEVCAP2:
	case PCI_EXP_DEVCTL2:
	case PCI_EXP_LNKCAP2:
	case PCI_EXP_LNKCTL2:
	case PCI_EXP_LNKSTA2:
		return pcie_cap_version(dev) > 1;
	default:
		return false;
	}
}

int pcie_capability_read_word(struct pci_dev *dev, int pos, u16 *val)
{
	int ret;

	*val = 0;
	if (pos & 1)
		return -EINVAL;

	if (pcie_capability_reg_implemented(dev, pos)) {
		ret = pci_read_config_word(dev, pci_pcie_cap(dev) + pos, val);
		/*
		 * Reset *val to 0 if pci_read_config_word() fails, it may
		 * have been written as 0xFFFF if hardware error happens
		 * during pci_read_config_word().
		 */
		if (ret)
			*val = 0;
		return ret;
	}

	/*
	 * For Functions that do not implement the Slot Capabilities,
	 * Slot Status, and Slot Control registers, these spaces must
	 * be hardwired to 0b, with the exception of the Presence Detect
	 * State bit in the Slot Status register of Downstream Ports,
	 * which must be hardwired to 1b.  (PCIe Base Spec 3.0, sec 7.8)
	 */
	if (pci_is_pcie(dev) && pos == PCI_EXP_SLTSTA &&
	    pci_pcie_type(dev) == PCI_EXP_TYPE_DOWNSTREAM) {
		*val = PCI_EXP_SLTSTA_PDS;
	}

	return 0;
}
#endif /* !(RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6,5)) */

int sg_nents(struct scatterlist *sg)
{
	int nents;

	for (nents = 0; sg; sg = sg_next(sg))
		nents++;
	return nents;
}
#endif /* 3.7.0 */

/******************************************************************************/
#if (KERNEL_VERSION(3, 12, 0) > LINUX_VERSION_CODE && \
	!(RHEL_RELEASE_CODE && RHEL_RELEASE_VERSION(7, 0) <= RHEL_RELEASE_CODE))
int pci_wait_for_pending_transaction(struct pci_dev *dev)
{
	int i;
	u16 status;

	/* Wait for Transaction Pending bit clean */
	for (i = 0; i < 4; i++) {
		if (i)
			msleep((1 << (i - 1)) * 100);

		pcie_capability_read_word(dev, PCI_EXP_DEVSTA, &status);
		if (!(status & PCI_EXP_DEVSTA_TRPND))
			return 1;
	}

	return 0;
}
#endif /* 3.12.0 && !RHEL7.x */

/******************************************************************************/
#if (KERNEL_VERSION(3, 13, 0) > LINUX_VERSION_CODE && \
	!(RHEL_RELEASE_CODE && \
		(RHEL_RELEASE_VERSION(6, 8) <= RHEL_RELEASE_CODE) && \
		(RHEL_RELEASE_VERSION(7, 0) != RHEL_RELEASE_CODE)))
int crypto_authenc_extractkeys(struct crypto_authenc_keys *keys, const u8 *key,
			       unsigned int keylen)
{
	struct rtattr *rta = (struct rtattr *)key;
	struct crypto_authenc_key_param *param;

	if (!RTA_OK(rta, keylen))
		return -EINVAL;
	if (rta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM)
		return -EINVAL;
	if (RTA_PAYLOAD(rta) < sizeof(*param))
		return -EINVAL;

	param = RTA_DATA(rta);
	keys->enckeylen = be32_to_cpu(param->enckeylen);

	key += RTA_ALIGN(rta->rta_len);
	keylen -= RTA_ALIGN(rta->rta_len);

	if (keylen < keys->enckeylen)
		return -EINVAL;

	keys->authkeylen = keylen - keys->enckeylen;
	keys->authkey = key;
	keys->enckey = key + keys->authkeylen;

	return 0;
}
#endif /* 3.13.0 && !RHEL7.1+ && !SUSE12.1 */

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
void memzero_explicit(void *s, size_t count)
{
	memset(s, 0, count);
	OPTIMIZER_HIDE_VAR(s);
}
#endif /* 3.17.3, etc. */
#if (KERNEL_VERSION(4, 3, 0) > LINUX_VERSION_CODE)

#include <linux/seq_file.h>

void seq_hex_dump(struct seq_file *sfile, const char *prefix_s, int prefix_t,
		  int rowsize, int groupsize, const void *buf, size_t len,
		  bool ascii)
{
	const u32 *msg = buf;
	int i, x;

	seq_printf(sfile, "%p:", msg);
	x = 0;
	i = 0;
	for (; i < (len >> 2); i++) {
		seq_printf(sfile, " %08X", *(msg + i));
		if ((len >> 2) != i + 1 && (++x == 8)) {
			seq_printf(sfile, "\n%p:", msg + i + 1);
			x = 0;
		}
	}
	seq_puts(sfile, "\n");
}
#endif  /* 4.2.0 */
