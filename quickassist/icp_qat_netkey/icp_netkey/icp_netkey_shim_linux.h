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

#ifndef ICP_NETKEY_SHIM_LINUX_H
#define ICP_NETKEY_SHIM_LINUX_H

#define ICP_QAT_ALG_DRIVER_NAME_MAX 64

/* Structure holding pointers to driver_init and driver_exit. */
struct icp_qat_alg_driver {
        char driver_name[ICP_QAT_ALG_DRIVER_NAME_MAX];
        int (*driver_init) (void);
        int (*driver_exit) (void);
};

/* structure for storing info on instances used by QAT */
struct icp_qat_instances {
        /* Total number of instances available */
        u32 total;
        int index;
        /* pointer to array of instance handles */
        CpaInstanceHandle *instances;
};

extern struct icp_qat_instances qat_instances;
extern struct icp_qat_instances *instsByCore;

/* Function to allow drivers to get instances */
CpaInstanceHandle get_instance(Cpa32U core);

extern struct icp_qat_alg_driver aes_cbc_sha1_hmac_driver;
extern struct icp_qat_alg_driver aes_cbc_md5_hmac_driver;
extern struct icp_qat_alg_driver aes_cbc_sha256_hmac_driver;
extern struct icp_qat_alg_driver aes_cbc_sha512_hmac_driver;
extern struct icp_qat_alg_driver des3_cbc_sha1_hmac_driver;
extern struct icp_qat_alg_driver des3_cbc_md5_hmac_driver;
extern struct icp_qat_alg_driver des3_cbc_sha256_hmac_driver;
extern struct icp_qat_alg_driver des3_cbc_sha512_hmac_driver;
extern struct icp_qat_alg_driver hmac_sha1_driver;
extern struct icp_qat_alg_driver hmac_sha256_driver;
extern struct icp_qat_alg_driver authenc_rfc3686_aes_ctr_sha1_hmac_driver;

#endif /* ICP_NETKEY_SHIM_LINUX_H */

