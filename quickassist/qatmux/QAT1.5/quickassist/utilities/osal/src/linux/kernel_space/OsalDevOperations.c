/**
 * @file OsalDevOperations.c (linux kernel space)
 *
 * @brief Osal interface to linux pci store & restore API.
 *
 * @par
 * GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
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
 *  version: QAT1.5.L.1.11.0-36
 */

#include <Osal.h>
#include <OsalOsTypes.h>
#include <cpa.h>


/* PCI Save/Restore is directly supported by 
 * the linux kernel from ver 3.0
 * For previous version a sequence
 * of library function calls must be used.
 * CentOS (Red Hat) Linux behaves similarly:
 * PCI save/restore is directly supported
 * by CentOS > 6.4 and a combination of
 * kernel functions must be used for earlier
 * versions. To keep conditional compilation
 * expression simpler, the LINUX_VERSION_CODE
 * macro is adjusted in case of CentOS
 */ 

#ifdef RHEL_RELEASE_CODE

#ifdef LINUX_VERSION_CODE
#undef LINUX_VERSION_CODE
#endif /*LINUX_VERSION_CODE*/

#if(RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7,0))
#define LINUX_VERSION_CODE KERNEL_VERSION(3,10,0)
#if(RHEL_RELEASE_CODE == RHEL_RELEASE_VERSION(7,0))
#define RHEL_70
#else
#define RHEL_71_ABOVE
#endif /* RHEL_RELEASE_CODE == RHEL_RELEASE_VERSION(7,0) */
#elif (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6,0))
#define LINUX_VERSION_CODE KERNEL_VERSION(2,6,32)
#if(RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6,5))
#define RHEL_65_ABOVE
#endif
#else
#define LINUX_VERSION_CODE KERNEL_VERSION(2,6,18)
#endif /*RHEL_RELEASE_VERSION */

#endif /*RHEL_RELEASE_CODE */
#ifdef ICP_SRIOV
/*Sriov capabilities*/
struct osal_pci_sriov {
        int pos;
        int nres;
        uint32_t cap;
        uint16_t ctrl;
        uint16_t total;
        uint16_t initial;
        uint16_t nr_virtfn;
        uint16_t offset;
        uint16_t stride;
        uint32_t pgsz;
        uint8_t link;
        struct pci_dev *dev;
        struct pci_dev *self;
        struct mutex lock;
        struct work_struct mtask;
        uint8_t __iomem *mstate;
};
#endif /*built with SRIOV on*/

/* PCIe Capabilitiy Offset */
#define QAT_PCIE_CAP_OFFSET 0x50

/*Capability structure*/
struct osal_pci_capabilities {
        int8_t cap_nr;
        uint32_t data[0];
};

/*Pci backup structure*/
struct osal_pci_saved_state {
        uint32_t config_space[16];
        struct osal_pci_capabilities cap[0];
};
/*
* Restore PCI state after a reset
*/
OSAL_STATUS osalPCIStateRestore(void* dev, void* state)
{
    struct pci_dev *pdev = dev;
		
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
#if (((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0))) \
    || defined(RHEL_70))
    struct pci_saved_state *pstate = state;
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)*/
#else
    struct osal_pci_saved_state *pstate = state;
    struct pci_cap_saved_state *tmp = NULL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)) && defined(ICP_SRIOV)
    struct osal_pci_sriov *iov = NULL;
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30) and ICP_SRIOV defined*/
    uint16_t *ptr = NULL;
    int8_t i = 0;
    int32_t base = 0;
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)*/

    if (!dev) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalPCIStateRestore(): Invalid device\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    if (!state) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalPCIStateRestore(): No Backup found for this device\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
#if (((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0))) \
    || defined(RHEL_70))
    if ( pci_load_saved_state(pdev, pstate) ){
           return OSAL_FAIL;
    }
#endif
    pci_restore_state(pdev);
#if (((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0))) \
    || defined(RHEL_71_ABOVE))
    pci_save_state(pdev);
#endif
    return OSAL_SUCCESS;
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
    if (!pdev->state_saved) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalPCIStateRestore(): No Backup found for this device\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29) */

    /*restore dev config space*/
    memcpy(pdev->saved_config_space, pstate->config_space,
                        sizeof(pstate->config_space));
#ifdef RHEL_65_ABOVE
    {
        struct hlist_node *pos = NULL;
        hlist_for_each_entry(tmp, pos, &pdev->saved_cap_space, next) {
            if (PCI_CAP_ID_EXP == tmp->cap.cap_nr) {
                break;
            }
        }
    }
#else
    /*restore capabilities*/
    tmp = pci_find_saved_cap(pdev, PCI_CAP_ID_EXP);
#endif
    base = pci_find_capability(pdev, PCI_CAP_ID_EXP);
    if (!tmp || base <= 0) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalPCIStateRestore(): Capacities buffer not found\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
#ifdef RHEL_65_ABOVE
    ptr = (uint16_t *)&tmp->cap.data[0];
#else
    ptr = (uint16_t *)&tmp->data[0];
#endif
    i = 0;
    pci_write_config_word(pdev, base + PCI_EXP_DEVCTL, ptr[i++]);
    pci_write_config_word(pdev, base + PCI_EXP_LNKCTL, ptr[i++]);
    pci_write_config_word(pdev, base + PCI_EXP_SLTCTL, ptr[i++]);
    pci_write_config_word(pdev, base + PCI_EXP_RTCTL, ptr[i++]);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28))
    pci_write_config_word(pdev, base + PCI_EXP_DEVCTL2, ptr[i++]);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
    pci_write_config_word(pdev, base + PCI_EXP_LNKCTL2, ptr[i++]);
    pci_write_config_word(pdev, base + PCI_EXP_SLTCTL2, ptr[i++]);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)*/
    /*reconfigure config space*/
    for (i = 15; i >= 0; i--)
    {
        pci_write_config_dword(pdev,
                             i * sizeof(uint32_t), pdev->saved_config_space[i]);
    }
    /*restore MSI state*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25))
    pci_restore_msi_state(pdev);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25) */
    /*restore iov state*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)) && defined(ICP_SRIOV)
    iov = (struct osal_pci_sriov *)pdev->sriov;
    if (NULL != iov)
    {
        for (i = PCI_IOV_RESOURCES; i <= PCI_IOV_RESOURCE_END; i++) {
            struct pci_bus_region region;
            uint32_t new, check, mask;
            int reg;
            struct resource *res = pdev->resource + i;
            reg = iov->pos + PCI_SRIOV_BAR +
                    sizeof(uint32_t) * (i - PCI_IOV_RESOURCES);
            pcibios_resource_to_bus(pdev, &region, res);
            new = region.start | (res->flags & PCI_REGION_FLAG_MASK);
            mask = (uint32_t)PCI_BASE_ADDRESS_MEM_MASK;

            pci_write_config_dword(pdev, reg, new);
            pci_read_config_dword(pdev, reg, &check);
            if ((new ^ check) & mask) {
                osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                        "osalPCIStateRestore(): BAR %d: error updating"
                        "(%#08x != %#08x)\n",
                        i, new, check, 0, 0, 0, 0, 0);
                continue;
            }
            res->flags &= ~IORESOURCE_UNSET;
        }
   
        pci_write_config_dword(pdev, iov->pos + PCI_SRIOV_SYS_PGSIZE,
                                iov->pgsz);
        pci_write_config_word(pdev, iov->pos + PCI_SRIOV_NUM_VF,
                                iov->nr_virtfn);
        pci_write_config_word(pdev, iov->pos + PCI_SRIOV_CTRL,
                                iov->ctrl);
    } else {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                       "osalPCIStateRestore(): sriov capabilities not "
                       "visible, but driver built for sriov\n",
                       0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30) and ICP_SRIOV defined */
#endif /* else LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0) */
    return OSAL_SUCCESS;
}

/*
 * Store PCI state at probing time
 */
void* osalPCIStateStore(void *dev, UINT32 node)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    uint16_t *ptr = NULL, i = 0;
    int32_t base = 0;
    struct osal_pci_saved_state *pstate = NULL;
    struct pci_cap_saved_state *tmp = NULL;
    struct osal_pci_capabilities *cap = NULL;
    struct hlist_node *pos = NULL;
    size_t size = 0;
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0) */
    struct pci_dev *pdev = dev;
    if (!dev) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalPCIStateStore(): Invalid device\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
    if( pci_save_state(pdev) ){
	    return NULL;
    }
    return pci_store_saved_state(pdev);
#else 
 
    /*save config space*/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,27)
    if( pci_save_state(pdev) ){
        return NULL;
    }
#else
    for (i = 0; i < 16 ; i++)
    {
         pci_read_config_dword(pdev,
                            i * sizeof(uint32_t), &pdev->saved_config_space[i]);
    }
#endif /* End of check for KERNEL_VERSIONS lower than 2.6.28 */

    /*back up pci-e capabilities*/
#ifdef RHEL_65_ABOVE
    hlist_for_each_entry(tmp, pos, &pdev->saved_cap_space, next) {
        if (PCI_CAP_ID_EXP == tmp->cap.cap_nr) {
            break;
        }
    }
#else
    tmp = pci_find_saved_cap(pdev, PCI_CAP_ID_EXP);
#endif
    base = pci_find_capability(pdev, PCI_CAP_ID_EXP);
    if (NULL==tmp || base <= 0) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalPCIStateStore(): Capacities buffer not found \n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }
    /*compute backup size*/
    size = sizeof(struct osal_pci_saved_state) +
           sizeof(struct pci_cap_saved_state);
    hlist_for_each_entry(tmp, pos, &pdev->saved_cap_space, next) {
        size += sizeof(struct pci_cap_saved_state);
    }
    /*alloc the backup structure*/
    pstate = kmalloc_node(size, GFP_KERNEL, node);
    if (!pstate) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalPCIStateStore(): failed to alloc %d bytes on node %d"
                 "to backup the pci state\n",
                 size, node, 0, 0, 0, 0, 0, 0);
        return NULL;
    }
#ifdef RHEL_65_ABOVE
    ptr = (uint16_t *)&tmp->cap.data[0];
#else
    ptr = (uint16_t *)&tmp->data[0];
#endif
    i = 0;
    pci_read_config_word(pdev, base + PCI_EXP_DEVCTL, &ptr[i++]);
    pci_read_config_word(pdev, base + PCI_EXP_LNKCTL, &ptr[i++]);
    pci_read_config_word(pdev, base + PCI_EXP_SLTCTL, &ptr[i++]);
    pci_read_config_word(pdev, base + PCI_EXP_RTCTL, &ptr[i++]);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28))
    pci_read_config_word(pdev, base + PCI_EXP_DEVCTL2, &ptr[i++]);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28) */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
    pci_read_config_word(pdev, base + PCI_EXP_LNKCTL2, &ptr[i++]);
    pci_read_config_word(pdev, base + PCI_EXP_SLTCTL2, &ptr[i++]);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30) */
   /* copy config space into the backup struct */
    memcpy(pstate->config_space, pdev->saved_config_space,
                        sizeof(pstate->config_space));
    /*copy the capabilities into the back-up*/
    cap = pstate->cap;
    hlist_for_each_entry(tmp, pos, &pdev->saved_cap_space, next) {
         size_t len = sizeof(struct osal_pci_capabilities);
#ifdef RHEL_65_ABOVE
         memcpy(cap++, &tmp->cap.cap_nr, len);
#else
         memcpy(cap++, &tmp->cap_nr, len);
#endif
    }
    /*backup complete and return the allocated structure*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
    pdev->state_saved = true;
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29) */
    return pstate;
#endif /* else LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0) */
}


/*
* Set PCIe Capabilities Offset
*/
void osalSetPCICapabilitiesOffset(void* dev)
{
    struct pci_dev *pdev = NULL;

    if (!dev) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalSetPCICapabilitiesOffset(): Invalid device\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return;
    }
    pdev = dev;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32))
    pdev->pcie_cap = QAT_PCIE_CAP_OFFSET;
#else
#ifdef RHEL_RELEASE_CODE
#if (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6,0))
     pdev->pcie_cap = QAT_PCIE_CAP_OFFSET; 
#endif
#endif /* RHEL_RELEASE_CODE */
#endif

}

