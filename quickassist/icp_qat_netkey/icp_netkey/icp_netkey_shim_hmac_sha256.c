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

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/crypto.h>
#include <crypto/authenc.h>
#include <linux/rtnetlink.h>
#include <crypto/algapi.h>
#include <crypto/rng.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <crypto/hash.h>
#include <crypto/sha.h>
#include <crypto/internal/hash.h>

#include "icp_netkey_hash.h"
#include "icp_netkey_shim_linux.h"

static int icp_hmac_sha256_setkey(struct crypto_ahash *tfm,
                                  const u8 *key, unsigned int keylen)
{
        return icp_netkey_hash_setkey(tfm, key, keylen,
                                      CPA_CY_SYM_HASH_SHA256,
                                      CPA_CY_SYM_HASH_MODE_AUTH);
}

static struct ahash_alg icp_hmac_sha256 = {
        .init = icp_netkey_hash_init,
        .update = icp_netkey_hash_update,
        .final = icp_netkey_hash_final,
        .finup = icp_netkey_hash_finup,
        .digest = icp_netkey_hash_digest,
        .setkey = icp_hmac_sha256_setkey,
        .halg = {
                .digestsize = SHA256_DIGEST_SIZE,
                .statesize = sizeof(struct icp_netkey_hash_tfm_ctx),
                .base = {
                        .cra_name = "hmac(sha256)",
                        .cra_driver_name = "icp_qat_sha256_hmac",
                        .cra_priority = 0,
                        .cra_flags = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                        .cra_blocksize = SHA256_BLOCK_SIZE,
                        .cra_ctxsize = sizeof(struct icp_netkey_hash_tfm_ctx),
                        .cra_init = icp_netkey_hash_cra_init,
                        .cra_exit = icp_netkey_hash_cra_exit,
                        .cra_module = THIS_MODULE,
                }
        }
};

static int icp_hmac_sha256_driver_init(void)
{
        return crypto_register_ahash(&icp_hmac_sha256);
}

static int icp_hmac_sha256_driver_exit(void)
{
        return crypto_unregister_ahash(&icp_hmac_sha256);
}

struct icp_qat_alg_driver hmac_sha256_driver = {
        .driver_name = "icp_qat_sha256_hmac",
        .driver_init = icp_hmac_sha256_driver_init,
        .driver_exit = icp_hmac_sha256_driver_exit,
};
