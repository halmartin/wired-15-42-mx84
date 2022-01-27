/***************************************************************************
 *
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
 *  version: icp_qat_netkey.L.0.4.3-1
 *
 ***************************************************************************/

/*
 * icp_netkey_shim_linux.c
 *
 * This is an implementation of Linux Kernel Crypto API shim that uses
 * the Intel(R) Quick Assist API. The focus here is IPsec, (Netkey stack).
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "cpa.h"
#include "cpa_cy_im.h"
#include "icp_netkey_shim_linux.h"
#include "icp_netkey_hash.h"
#include "icp_netkey_shim_rfc3686_ctr_aes.h"

/* Global instance info - set at module init */
struct icp_qat_instances qat_instances = { 0 };
struct icp_qat_instances *instsByCore;
CpaInstanceHandle *coreInstances = NULL;

struct icp_qat_alg_driver *alg_drivers[] = { &aes_cbc_sha1_hmac_driver,
                                             &aes_cbc_md5_hmac_driver,
                                             &aes_cbc_sha256_hmac_driver,
                                             &aes_cbc_sha512_hmac_driver,
                                             &des3_cbc_sha1_hmac_driver,
                                             &des3_cbc_md5_hmac_driver,
                                             &des3_cbc_sha256_hmac_driver,
                                             &des3_cbc_sha512_hmac_driver, 
                                             &hmac_sha1_driver,
                                             &rfc3686_ctr_aes_driver,
                                             &authenc_rfc3686_aes_ctr_sha1_hmac_driver,
                                             &hmac_sha256_driver,
                                             NULL };

CpaInstanceHandle get_instance_rr(void)
{   
        return qat_instances.instances[(qat_instances.index++) %
                                       qat_instances.total];
}


/* This function gets the QA instances and registers our module with LKCF */
static int __init icp_netkey_init(void)
{
        Cpa16U num = 0;
        int err = 0;
        int i, j = 0;
        CpaStatus status = CPA_STATUS_SUCCESS;
        int retval = 0;
        int startCtr = 0;
        CpaInstanceInfo2 instanceInfo2;
        int numOfCores = 0;   
 
        /* QA API crypto instances are found at module init and are assumed to
           stay static while this module is active. */
        if (cpaCyGetNumInstances(&num) != CPA_STATUS_SUCCESS) {
                printk(KERN_ERR
                       "%s: Could not get number of Quick Assist"
                       " crypto instances\n", __func__);
                retval = -EINVAL;
                return retval;
        }

        /* Allocate memory to store global array of QA instances */
        qat_instances.instances = kmalloc(sizeof(CpaInstanceHandle)
                                          * num, GFP_KERNEL);
        if (NULL == qat_instances.instances) {
                printk(KERN_ERR "%s: Failed to alloc instance memory\n",
                       __func__);
                retval = -ENOMEM;
                return retval;
        }

        if (cpaCyGetInstances(num, qat_instances.instances) !=
            CPA_STATUS_SUCCESS) {
                printk(KERN_ERR
                       "%s: Could not get Quick Assist crypto instance\n",
                       __func__);
                retval = -EINVAL;
                /* free qat instances */
                kfree(qat_instances.instances);
                return retval;
        }

        qat_instances.total = num;
        numOfCores = num_online_cpus();

        
        /* Allocate memory to store global multidimensional array of QA instances and cores */
        instsByCore = kzalloc(sizeof(struct icp_qat_instances)
                              * numOfCores, GFP_KERNEL);
        if (NULL == instsByCore) {
                printk(KERN_ERR "%s: Failed to alloc instance by core memory\n",
                       __func__);
                retval = -ENOMEM;
                /* free qat instances */
                kfree(qat_instances.instances);
                return retval;
        }

        coreInstances = kzalloc(sizeof(CpaInstanceHandle) * numOfCores * num, GFP_KERNEL);

        if (NULL == coreInstances) {
                printk(KERN_ERR "%s: Failed to alloc instance by core array memory\n",
                       __func__);
                retval = -ENOMEM;
                /* free instances */
                kfree(instsByCore);
                kfree(qat_instances.instances);
                return retval;
        }


        for (i = 0; i < numOfCores; i++)
        {
                instsByCore[i].instances = coreInstances+(i * num);
        }
        
        /* order the instances to be indexed by core affinity */
        for (i = 0; i < num; i++)
        {
                if (cpaCyInstanceGetInfo2(qat_instances.instances[i], 
                                          &instanceInfo2) != CPA_STATUS_SUCCESS) 
                {
                        printk(KERN_ERR
                               "%s: Could not get Quick Assist crypto instance info\n",
                               __func__);
                        retval = -EINVAL;
                        /* free instances */
                        kfree(coreInstances);
                        kfree(instsByCore);
                        kfree(qat_instances.instances);
                        return retval;
                }

                for (j = 0; (j < CPA_MAX_CORES && (0 == CPA_BITMAP_BIT_TEST(instanceInfo2.coreAffinity, j))); 
                     j++)
                {
                        /* do nothing here - scanning through bitmask for find bit set */
                        /* assuming an instance is affinitised to only 1 core */
                }

                if (j >= numOfCores)
                {
                        printk(KERN_ERR
                               "%s: Exceeded number of CPU cores\n",
                               __func__);
                        retval = -EINVAL;
                        /* free instances */
                        kfree(coreInstances);
                        kfree(instsByCore);
                        kfree(qat_instances.instances);
                        return retval;
                }

                instsByCore[j].instances[instsByCore[j].total] = qat_instances.instances[i];
                instsByCore[j].total++;
        }


        /* Fill in rest of multidimmensional array with instances mapped to cores */
        for (i = 0; i < numOfCores; i++)
        {
                if (NULL == instsByCore[i].instances[0])
                {
                        /* Get an instance round robin fashion */
                        instsByCore[i].instances[0] = get_instance_rr();
                        instsByCore[i].total++;
                }
        }

    
        /* Start all instances */
        for (startCtr = 0; startCtr < num; startCtr++) {
                status = cpaCyStartInstance(qat_instances.instances[startCtr]);
                if (CPA_STATUS_SUCCESS != status) {
                        printk(KERN_ERR "%s: Failed to start instance %d\n",
                               __func__, startCtr);
                        retval = -EINVAL;
                        /* stop instances */
                        for (j = 0; j < startCtr; j++) {
                                cpaCyStopInstance(qat_instances.instances[j]);
                        }
                        kfree(coreInstances);
                        kfree(instsByCore);
                        kfree(qat_instances.instances);
                        return retval;
                }
        }

        /* Initialize algorithm drivers */
        i = 0;
        j = 0;
        while (alg_drivers[i] != NULL) {
                if ((err = alg_drivers[i]->driver_init())) {
                        printk(KERN_ERR
                               "%s: Init alg failed for Quick Assist %s\n",
                               __func__, alg_drivers[i]->driver_name);
                        retval = err;
                        /* Stop drivers */
                        while (alg_drivers[j] != NULL) {
                                alg_drivers[j]->driver_exit();
                                j++;
                        }
                        /* stop instances */
                        for (j = 0; j < startCtr; j++) {
                                cpaCyStopInstance(qat_instances.instances[j]);
                        }
                        kfree(coreInstances);
                        kfree(instsByCore);
                        kfree(qat_instances.instances);
                        return retval;
                }
                printk(KERN_INFO "Quick Assist %s loaded\n",
                       alg_drivers[i]->driver_name);
                i++;
        }

        return icp_netkey_hash_module_init();
}

/* This function calls driver_exit functions and frees instance memory */
static void __exit icp_netkey_exit(void)
{
        int j = 0;

        icp_netkey_hash_module_exit();

        /* Un-register from LKCF */
        while (alg_drivers[j] != NULL) {
                if (alg_drivers[j]->driver_exit() != 0) {
                        printk(KERN_ERR "%s: Unable to unload %s\n",
                               __func__, alg_drivers[j]->driver_name);
                }
                j++;
        }

        /* Stop all instances */
        for (j = 0; j < qat_instances.total; j++) {
                cpaCyStopInstance(qat_instances.instances[j]);
        }

        /* Free resources */
        kfree(coreInstances);
        kfree(instsByCore);
        kfree(qat_instances.instances);
}

module_init(icp_netkey_init);
module_exit(icp_netkey_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LKCF Driver for Intel Quick Assist crypto acceleration");
MODULE_AUTHOR("Intel Corporation");
MODULE_VERSION("0.4.2");
