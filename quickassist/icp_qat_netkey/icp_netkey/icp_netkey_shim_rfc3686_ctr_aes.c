/***************************************************************************
 *
 * GPL LICENSE SUMMARY
 *
 *   Copyright (c) 2016 Meraki, Inc.
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
#include <crypto/ctr.h>
#include <crypto/hash.h>
#include <crypto/sha.h>

#include "cpa.h"
#include "cpa_cy_sym.h"
#include "cpa_cy_im.h"
#include "icp_netkey_shim.h"
#include "icp_netkey_shim_linux.h"

struct icp_rfc3686_tfm_ctx {
        char key[CTR_RFC3686_BLOCK_SIZE];
        char nonce[CTR_RFC3686_NONCE_SIZE];

        u8 *session_ctx;
        CpaInstanceHandle instance;
        Cpa32U digestResultLenInBytes;
        Cpa32U metaSize;
};

struct icp_rfc3686_req_ctx {
        char counter[CTR_RFC3686_BLOCK_SIZE];
        struct crypto_async_request *async_req;

        CpaBufferList srcBufferList
        __attribute__ ((aligned(ICP_QAT_SRC_BUFF_ALIGN)));
        CpaFlatBuffer srcFlatBuffers[ICP_MAX_NUM_BUFFERS];
        CpaBufferList dstBufferList
        __attribute__ ((aligned(ICP_QAT_SRC_BUFF_ALIGN)));
        CpaFlatBuffer dstFlatBuffers[ICP_MAX_NUM_BUFFERS];
        CpaCySymOpData opData;

        char __pBufferMeta[] CRYPTO_MINALIGN_ATTR;
};

static int icp_rfc3686_ctr_aes_cra_init(struct crypto_tfm *tfm)
{
        struct icp_rfc3686_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);
        CpaStatus status = CPA_STATUS_SUCCESS;

        tfm_ctx->session_ctx = NULL;
        tfm_ctx->instance = get_instance(current_thread_info()->cpu);

        status = cpaCyBufferListGetMetaSize(
                tfm_ctx->instance, ICP_MAX_NUM_BUFFERS, &tfm_ctx->metaSize);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed to get meta size\n", __func__);
                return -EINVAL;
        }
        tfm->crt_ablkcipher.reqsize =
                sizeof(struct icp_rfc3686_req_ctx) +
                tfm_ctx->metaSize + // src
                tfm_ctx->metaSize;  // dst
        tfm->crt_ablkcipher.ivsize = CTR_RFC3686_IV_SIZE;
        return 0;
}

static void icp_rfc3686_ctr_aes_cra_exit(struct crypto_tfm *tfm)
{
        struct icp_rfc3686_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);
        kfree(tfm_ctx->session_ctx);
}

static void
icp_rfc3686_callback(void *pCallbackTag,
                     CpaStatus status,
                     const CpaCySymOp operationType,
                     void *pOpData,
                     CpaBufferList *pDstBuffer,
                     CpaBoolean verifyResult)

{
        struct icp_rfc3686_req_ctx *req_ctx =
                (struct icp_rfc3686_req_ctx *)pCallbackTag;
        req_ctx->async_req->complete(req_ctx->async_req, 0);
}


static int icp_rfc3686_ctr_aes_setkey(struct crypto_ablkcipher *crypto_ablkcipher,
                                       const u8 *key,
                                       unsigned int keylen)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        CpaCySymSessionSetupData sessionSetupData = {0};
        Cpa32U sessionSize = 0;
        struct icp_rfc3686_tfm_ctx *tfm_ctx =
                crypto_tfm_ctx(crypto_ablkcipher_tfm(crypto_ablkcipher));

        status = cpaCySymSessionCtxGetSize(
                tfm_ctx->instance, &sessionSetupData, &sessionSize);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed to get session size\n", __func__);
                goto fail;
        }
        tfm_ctx->session_ctx = kzalloc(sessionSize, GFP_KERNEL);
        if (NULL == tfm_ctx->session_ctx) {
                printk(KERN_ERR "%s: Failed to alloc session\n", __func__);
                goto fail;
        }
        memcpy(tfm_ctx->key, key, sizeof(tfm_ctx->key));
        memcpy(tfm_ctx->nonce,
               &key[sizeof(tfm_ctx->key)],
               sizeof(tfm_ctx->nonce));

        sessionSetupData.sessionPriority = CPA_CY_PRIORITY_NORMAL;
        sessionSetupData.symOperation = CPA_CY_SYM_OP_CIPHER;
        sessionSetupData.cipherSetupData.cipherAlgorithm =
                CPA_CY_SYM_CIPHER_AES_CTR;
        sessionSetupData.cipherSetupData.cipherKeyLenInBytes =
                CTR_RFC3686_BLOCK_SIZE;
        sessionSetupData.cipherSetupData.pCipherKey = tfm_ctx->key;
        sessionSetupData.cipherSetupData.cipherDirection =
                CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT;
        sessionSetupData.digestIsAppended = CPA_FALSE;
        sessionSetupData.verifyDigest = CPA_FALSE;

        status = cpaCySymInitSession(tfm_ctx->instance,
                                     icp_rfc3686_callback,
                                     &sessionSetupData, tfm_ctx->session_ctx);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed cpaCySymInitSession\n", __func__);
                goto fail;
        }

        return 0;

fail:
        kfree(tfm_ctx->session_ctx);
        tfm_ctx->session_ctx = NULL;
        return -EINVAL;
}

static int icp_rfc3686_ctr_aes_crypt(struct ablkcipher_request *req)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        void *iv = req->info;
        struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
        struct icp_rfc3686_req_ctx *req_ctx = ablkcipher_request_ctx(req);
        struct icp_rfc3686_tfm_ctx *tfm_ctx =
                crypto_tfm_ctx(crypto_ablkcipher_tfm(ablkcipher));
        uint32_t count =
                cpu_to_be32(!(req->base.flags & CRYPTO_TFM_REQ_MERAKI_MODE));

        req_ctx->srcBufferList.pBuffers = req_ctx->srcFlatBuffers;
        req_ctx->srcBufferList.pPrivateMetaData = req_ctx->__pBufferMeta;
        req_ctx->srcBufferList.numBuffers = 0;
        if (sl_to_bl(&req_ctx->srcBufferList, req->src) != CPA_STATUS_SUCCESS) {
                goto fail;
        }

        req_ctx->dstBufferList.pBuffers = req_ctx->dstFlatBuffers;
        req_ctx->dstBufferList.pPrivateMetaData =
                &req_ctx->__pBufferMeta[tfm_ctx->metaSize];
        req_ctx->dstBufferList.numBuffers = 0;
        if (sl_to_bl(&req_ctx->dstBufferList, req->dst) != CPA_STATUS_SUCCESS) {
                goto fail;
        }

        req_ctx->async_req = &req->base;
        memcpy(&req_ctx->counter[0],
               tfm_ctx->nonce, CTR_RFC3686_NONCE_SIZE);
        memcpy(&req_ctx->counter[CTR_RFC3686_NONCE_SIZE],
               iv, CTR_RFC3686_IV_SIZE);
        memcpy(&req_ctx->counter[CTR_RFC3686_NONCE_SIZE + CTR_RFC3686_IV_SIZE],
               &count, sizeof(count));

        req_ctx->opData.sessionCtx = tfm_ctx->session_ctx;
        req_ctx->opData.packetType = CPA_CY_SYM_PACKET_TYPE_FULL;
        // "For block ciphers in CTR mode, this is the counter"
        req_ctx->opData.pIv = req_ctx->counter;
        req_ctx->opData.ivLenInBytes = CTR_RFC3686_BLOCK_SIZE;
        req_ctx->opData.cryptoStartSrcOffsetInBytes = 0;
        req_ctx->opData.messageLenToCipherInBytes = req->nbytes;
        req_ctx->opData.pDigestResult = NULL;

        status = cpaCySymPerformOp(
                tfm_ctx->instance,
                (void *)req_ctx,
                &req_ctx->opData,
                &req_ctx->srcBufferList,
                &req_ctx->dstBufferList,
                NULL);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: cpaCySymPerformOp failed\n", __func__);
                goto fail;
        }

        return -EINPROGRESS;

fail:
        return -EINVAL;
}

static struct crypto_alg icp_rfc3686_ctr_aes = {
        .cra_name        = "rfc3686(ctr(aes))",
        .cra_driver_name = "icp_qat_rfc3686_ctr_aes",
        .cra_priority    = 4001,
        .cra_flags       = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
        .cra_blocksize   = AES_BLOCK_SIZE,
        .cra_ctxsize     = sizeof(struct icp_rfc3686_tfm_ctx),
        .cra_type        = &crypto_ablkcipher_type,
        .cra_module      = THIS_MODULE,
        .cra_init        = icp_rfc3686_ctr_aes_cra_init,
        .cra_exit        = icp_rfc3686_ctr_aes_cra_exit,
        .cra_u           = {
                .ablkcipher = {
                        .setkey  = icp_rfc3686_ctr_aes_setkey,
                        .encrypt = icp_rfc3686_ctr_aes_crypt,
                        .decrypt = icp_rfc3686_ctr_aes_crypt,
                        .ivsize = CTR_RFC3686_IV_SIZE,
                        .min_keysize = CTR_RFC3686_BLOCK_SIZE +
                                         CTR_RFC3686_NONCE_SIZE,
                        .max_keysize = CTR_RFC3686_BLOCK_SIZE +
                                         CTR_RFC3686_NONCE_SIZE,
                        .geniv = "seqiv",
                },
        },
};

static int icp_rfc3686_ctr_aes_driver_init(void)
{
        return crypto_register_alg(&icp_rfc3686_ctr_aes);
}

static int icp_rfc3686_ctr_aes_driver_exit(void)
{
        return crypto_unregister_alg(&icp_rfc3686_ctr_aes);
}

struct icp_qat_alg_driver rfc3686_ctr_aes_driver = {
        .driver_name = "icp_qat_rfc3686_ctr_aes",
        .driver_init = icp_rfc3686_ctr_aes_driver_init,
        .driver_exit = icp_rfc3686_ctr_aes_driver_exit,
};
