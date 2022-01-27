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

#ifndef ICP_NETKEY_HASH_H
#define ICP_NETKEY_HASH_H

#include <linux/types.h>
#include <linux/crypto.h>
#include <linux/spinlock_types.h>
#include <linux/interrupt.h>
#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/hash.h>
#include "cpa.h"
#include "cpa_cy_sym.h"
#include "icp_netkey_shim.h"

struct icp_netkey_hash_tfm_ctx {
        u8 *session_ctx;
        CpaInstanceHandle instance;
        u8 *key;
        Cpa32U digestResultLenInBytes;
        bool initialized;
};

struct icp_netkey_hash_req_ctx {
        CpaBufferList bufferList __attribute__((aligned(ICP_QAT_SRC_BUFF_ALIGN)));
        CpaFlatBuffer flatBuffers[ICP_MAX_NUM_BUFFERS];
        CpaCySymOpData opData;
        u8 *data;
        unsigned int len;
        struct crypto_async_request *async_req;

        void *__pBufferMeta[] CRYPTO_MINALIGN_ATTR;
};

int icp_netkey_hash_module_init(void);
void icp_netkey_hash_module_exit(void);
int icp_netkey_hash_cra_init(struct crypto_tfm *tfm);
void icp_netkey_hash_cra_exit(struct crypto_tfm *tfm);
int icp_netkey_hash_init(struct ahash_request *req);
int icp_netkey_hash_update(struct ahash_request *req);
int icp_netkey_hash_final(struct ahash_request *req);
int icp_netkey_hash_finup(struct ahash_request *req);
int icp_netkey_hash_digest(struct ahash_request *req);
int icp_netkey_hash_setkey(struct crypto_ahash *tfm,
                           const u8 *key, unsigned int keylen,
                           CpaCySymHashAlgorithm hashAlgorithm,
                           CpaCySymHashMode hashMode);

#endif
