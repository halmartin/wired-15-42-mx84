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
 * icp_netkey_shim_aes_sha512.c
 *
 * This is an implementation of authenc(hmac(sha512),cbc(aes)) using
 *  Quick Assist API for the Linux Kernel Crypto Framework
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/crypto.h>
#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/algapi.h>
#include <linux/module.h> /* Needed for Linux backward compatibility */

#include "icp_netkey_shim.h"
#include "icp_netkey_shim_linux.h"

/* Generate IV aes_cbc_hmac_sha512 encryption */
static int geniv_encrypt_aes_cbc_hmac_sha512(struct aead_givcrypt_request *req)
{
        int retval = geniv_encrypt_netkey_shim (req, AES_BLOCK_SIZE);
        return retval;
}

/* This function initiates aes_cbc_hmac_sha512 decryption */
static int icp_aes_cbc_hmac_sha512_dec(struct aead_request *req)
{
        int retval = icp_netkey_shim_dec (req, AES_BLOCK_SIZE);
        return retval;
}

/* This function initiates aes_cbc_hmac_sha512 encryption */
static int icp_aes_cbc_hmac_sha512_enc(struct aead_request *req)
{
        int retval = icp_netkey_shim_enc(req, AES_BLOCK_SIZE);
        return retval;
}

/* This function initialises encrypt and decrypt sessions with the key
 */
static int icp_aes_cbc_hmac_sha512_setkey(struct crypto_aead *aead_tfm,
                                        const Cpa8U * key, unsigned int keylen)
{
        int retval = icp_netkey_shim_setkey (aead_tfm, key, keylen,
                                             CPA_CY_SYM_CIPHER_AES_CBC,
                                             CPA_CY_SYM_HASH_SHA512, AES_BLOCK_SIZE);
        return retval;
}

/* This function is used to set up icp_authenc_aes_cbc_hmac_sha512_ctx */
static int icp_authenc_aes_cbc_hmac_sha512_init(struct crypto_tfm *tfm)
{
        int retval = icp_netkey_shim_init (tfm);
        return retval;
}

/* This function is used to remove aes_cbc_hmac_sha512 encrypt/decrypt session */
static void icp_authenc_aes_cbc_hmac_sha512_exit(struct crypto_tfm *tfm)
{
        icp_netkey_shim_exit (tfm);
}

/* This structure contains the crypto algorithm details for aes_cbc_hmac_sha512 */
static struct crypto_alg icp_authenc_aes_cbc_hmac_sha512 = {
        .cra_name = "authenc(hmac(sha512),cbc(aes))",
        .cra_driver_name = "icp_qat_aes_cbc_sha512_hmac",
        /* '4001' assigned in order to pick a higher i
           driver than "authenc(hmac(sha512-generic),cbc-aes-aesni)" */
        .cra_priority = 4001,
        .cra_flags = CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
        .cra_blocksize = AES_BLOCK_SIZE,
        .cra_ctxsize = sizeof(struct icp_netkey_ctx),
        .cra_alignmask = 0,
        .cra_type = &crypto_aead_type,
        .cra_module = THIS_MODULE,
        .cra_list = LIST_HEAD_INIT(icp_authenc_aes_cbc_hmac_sha512.cra_list),
        .cra_init = icp_authenc_aes_cbc_hmac_sha512_init,
        .cra_exit = icp_authenc_aes_cbc_hmac_sha512_exit,
        .cra_u = {
                .aead = {
                        .setkey = icp_aes_cbc_hmac_sha512_setkey,
						.encrypt = icp_aes_cbc_hmac_sha512_enc,
                        .decrypt = icp_aes_cbc_hmac_sha512_dec,
                        .givencrypt = geniv_encrypt_aes_cbc_hmac_sha512,
                        .ivsize = AES_BLOCK_SIZE,
                        .maxauthsize = SHA512_DIGEST_SIZE,
                },
        },
};

static int icp_authenc_aes_cbc_hmac_sha512_driver_init(void)
{
        int retval = icp_netkey_driver_init(AES_BLOCK_SIZE, &icp_authenc_aes_cbc_hmac_sha512);
        return retval;
}

static int icp_authenc_aes_cbc_hmac_sha512_driver_exit(void)
{
        int retval = icp_netkey_driver_exit(&icp_authenc_aes_cbc_hmac_sha512);
        return retval;
}

struct icp_qat_alg_driver aes_cbc_sha512_hmac_driver = {
        .driver_name = "icp_authenc_aes_cbc_hmac_sha512",
        .driver_init = icp_authenc_aes_cbc_hmac_sha512_driver_init,
        .driver_exit = icp_authenc_aes_cbc_hmac_sha512_driver_exit,
};
