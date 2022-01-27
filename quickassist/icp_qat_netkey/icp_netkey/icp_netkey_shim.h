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

#ifndef ICP_NETKEY_SHIM_H
#define ICP_NETKEY_SHIM_H

#include <linux/types.h>
#include <linux/crypto.h>
#include <linux/spinlock_types.h>
#include <linux/rtnetlink.h>
#include <crypto/aead.h>
#include <crypto/aes.h>
#include "cpa.h"
#include "cpa_cy_sym.h"

/* max block size for cipher algorithms */
#define ICP_MAX_BLOCKSIZE AES_BLOCK_SIZE

/* max key size for cipher */
#define ICP_MAX_CIPHER_KEYSIZE 32

/* max key size for authentication */
#define ICP_MAX_AUTH_KEYSIZE 64


/* initialised session status */
#define CONFIGURE   0
#define CONFIGURED  1


/* TODO - how will this max digest be defined with multiple algos */
#define ICP_MAX_DIGEST_SIZE    SHA512_DIGEST_SIZE
#define ICP_QAT_SRC_BUFF_ALIGN 8

/* This shim supports up to 8 buffers in a buffer list */
#define ICP_MAX_NUM_BUFFERS    8

/* The max number of times we try to remove the session */
#define ICP_RETRY_COUNT 10
/* This is the amount of time we wait between remove session re-tries */
#define ICP_SESSION_REMOVE_TIME 2*HZ


/* ctx structure - created per SA */
struct icp_netkey_ctx {
	u8 *session_ctx;        /* session */
	/* qat instance assigned to this ctx */
	CpaInstanceHandle instance;
	/* salt for iv generation */
	u8 salt[ICP_MAX_BLOCKSIZE];
	/* set if sessions have been initialised for this ctx */
	atomic_t ctr;
	u8 *pIvZero;
	Cpa8U cipherkey[ICP_MAX_CIPHER_KEYSIZE];
	Cpa8U authkey[ICP_MAX_AUTH_KEYSIZE];
	unsigned int enckeylen;
	unsigned int authkeylen;
	CpaCySymCipherAlgorithm cipherAlgorithm;
	CpaCySymHashAlgorithm hashAlgorithm;
	spinlock_t saInitSpinLock;
	Cpa32U metaSize;
};

int sl_to_bl(CpaBufferList * pBufferList, const struct scatterlist *const pSrc);
int geniv_encrypt_netkey_shim(struct aead_givcrypt_request *req, u16 blocksize);
int icp_netkey_shim_dec(struct aead_request *req, u16 blocksize);
int icp_netkey_shim_enc(struct aead_request *req, u16 blocksize);
int icp_netkey_shim_extractkeys(struct crypto_aead *aead_tfm,
                                const Cpa8U *key,
                                unsigned int keylen,
                                struct rtattr **ppRta,
                                const Cpa8U **pauthkey,
                                unsigned int *pauthkeylen,
                                const Cpa8U **penckey,
                                unsigned int *penckeylen);
int icp_netkey_shim_setkey(struct crypto_aead *aead_tfm,
                           const Cpa8U * key, unsigned int keylen,
                           CpaCySymCipherAlgorithm cipherAlgorithm,
                           CpaCySymHashAlgorithm hashAlgorithm,
                           u16 blocksize);
int icp_netkey_shim_init(struct crypto_tfm *tfm);
void icp_netkey_shim_exit(struct crypto_tfm *tfm);
int icp_netkey_driver_init(u16 blocksize, struct crypto_alg *alg);
int icp_netkey_driver_exit(struct crypto_alg *alg);


#endif /* ICP_NETKEY_SHIM_H */
