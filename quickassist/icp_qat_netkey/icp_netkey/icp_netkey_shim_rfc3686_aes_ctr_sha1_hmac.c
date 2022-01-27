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

struct icp_authenc_rfc3686_tfm_ctx {
        char authkey[ICP_MAX_AUTH_KEYSIZE];
        unsigned int authkeylen;
        char enckey[ICP_MAX_CIPHER_KEYSIZE];
        unsigned int enckeylen;
        char nonce[CTR_RFC3686_NONCE_SIZE];

        u8 *session_ctx;
        CpaInstanceHandle instance;
        Cpa32U digestResultLenInBytes;
        Cpa32U metaSize;

        /* set if sessions have been initialised for this ctx */
        atomic_t ctr;

        spinlock_t saInitSpinLock;
};

struct icp_authenc_rfc3686_req_ctx {
        char counter[CTR_RFC3686_BLOCK_SIZE];
        struct crypto_async_request *async_req;

        CpaBufferList bufferList
        __attribute__ ((aligned(ICP_QAT_SRC_BUFF_ALIGN)));
        CpaFlatBuffer flatBuffer;

        CpaCySymOpData opData;

        char __pBufferMeta[] CRYPTO_MINALIGN_ATTR;
};

static int icp_authenc_rfc3686_aes_ctr_sha1_hmac_cra_init(struct crypto_tfm *tfm)
{
        struct icp_authenc_rfc3686_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);
        CpaStatus status = CPA_STATUS_SUCCESS;

        tfm_ctx->session_ctx = NULL;
        tfm_ctx->instance = NULL;

        status = cpaCyBufferListGetMetaSize(
                tfm_ctx->instance, ICP_MAX_NUM_BUFFERS, &tfm_ctx->metaSize);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed to get meta size\n", __func__);
                return -EINVAL;
        }
        tfm->crt_aead.reqsize =
                sizeof(struct icp_authenc_rfc3686_req_ctx) +
                tfm_ctx->metaSize;
        tfm->crt_aead.ivsize = CTR_RFC3686_IV_SIZE;

        /* initialise session specific spinlock */
        spin_lock_init(&tfm_ctx->saInitSpinLock);
        atomic_set(&tfm_ctx->ctr, 0);

        return 0;
}

static void icp_authenc_rfc3686_aes_ctr_sha1_hmac_cra_exit(struct crypto_tfm *tfm)
{
        struct icp_authenc_rfc3686_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);
        CpaCySymSessionCtx session_ctx = tfm_ctx->session_ctx;
        unsigned int count = 0;
        CpaStatus status = CPA_STATUS_SUCCESS;

        if (atomic_dec_and_test(&tfm_ctx->ctr)) {

            status = cpaCySymRemoveSession(tfm_ctx->instance, session_ctx);

            while ((CPA_STATUS_RETRY == status)
                   && (count < ICP_RETRY_COUNT)) {
                /* Wait a while before retry */
                set_current_state((long)TASK_INTERRUPTIBLE);
                schedule_timeout(ICP_SESSION_REMOVE_TIME);

                count++;
                status = cpaCySymRemoveSession(tfm_ctx->instance, session_ctx);
            }
            if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR
                       "%s: Failed to remove session\n", __func__);
            }
        }

        /* Free session memory */
        if (session_ctx) {
            kfree(session_ctx);
            tfm_ctx->session_ctx = NULL;
        }
}

static void
icp_authenc_rfc3686_callback(void *pCallbackTag,
                             CpaStatus status,
                             const CpaCySymOp operationType,
                             void *pOpData,
                             CpaBufferList *pDstBuffer,
                             CpaBoolean verifyResult)

{
        struct icp_authenc_rfc3686_req_ctx *req_ctx =
                (struct icp_authenc_rfc3686_req_ctx *)pCallbackTag;
        int err = (status == CPA_STATUS_SUCCESS && verifyResult) ? 0 : -EBADMSG;
        req_ctx->async_req->complete(req_ctx->async_req, err);
}


static int icp_authenc_rfc3686_aes_ctr_sha1_hmac_setkey(
    struct crypto_aead *crypto_aead,
    const u8 *key,
    unsigned int keylen)
{
        CpaStatus status;
        Cpa32U sessionSize = 0;
        CpaCySymSessionSetupData sd = { 0 };
        struct crypto_tfm *aead_tfm = crypto_aead_tfm(crypto_aead);
        struct icp_authenc_rfc3686_tfm_ctx *tfm_ctx = crypto_tfm_ctx(aead_tfm);
        CpaCySymSessionCtx session_ctx = tfm_ctx->session_ctx;
        unsigned int count = 0;

        struct rtattr *rta;
        const Cpa8U *authkey, *enckey;
        unsigned int authkeylen, enckeylen, fullenckeylen;
        int rc = icp_netkey_shim_extractkeys(crypto_aead,
                                             key,
                                             keylen,
                                             &rta,
                                             &authkey,
                                             &authkeylen,
                                             &enckey,
                                             &fullenckeylen);
        if (rc < 0) {
            goto fail;
        }

        if (atomic_dec_and_test(&tfm_ctx->ctr)) {
            printk(KERN_DEBUG
                   "%s: Re-initialising previous session\n", __func__);

            status = cpaCySymRemoveSession(tfm_ctx->instance, session_ctx);
            while ((CPA_STATUS_RETRY == status)
                   && (count < ICP_RETRY_COUNT))
            {
                /* Wait a while before retry */
                set_current_state((long)TASK_INTERRUPTIBLE);
                schedule_timeout(ICP_SESSION_REMOVE_TIME);
                count++;
                status = cpaCySymRemoveSession(tfm_ctx->instance, session_ctx);
            }
            if (CPA_STATUS_SUCCESS != status)
            {
                printk(KERN_ERR
                       "%s: Failed to remove session\n", __func__);
                goto fail;
            }
        }

        if (fullenckeylen <= CTR_RFC3686_NONCE_SIZE) {
            printk(KERN_ERR "%s: Enc key too short\n", __func__);
            goto fail;
        }
        enckeylen = fullenckeylen - CTR_RFC3686_NONCE_SIZE;

        status = cpaCySymSessionCtxGetSize(
                tfm_ctx->instance, &sd, &sessionSize);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed to get session size\n", __func__);
                goto fail;
        }
        tfm_ctx->session_ctx = kzalloc(sessionSize, GFP_KERNEL);
        if (NULL == tfm_ctx->session_ctx) {
                printk(KERN_ERR "%s: Failed to alloc session\n", __func__);
                goto fail;
        }

        atomic_set(&tfm_ctx->ctr, CONFIGURE);

        /* Stash all of the key data for use at session initialization time (on
         * first encrypt/decrypt). */
        memcpy(tfm_ctx->authkey, authkey, authkeylen);
        tfm_ctx->authkeylen = authkeylen;

        memcpy(tfm_ctx->enckey, enckey, enckeylen);
        tfm_ctx->enckeylen = enckeylen;

        memcpy(tfm_ctx->nonce, enckey + enckeylen, CTR_RFC3686_NONCE_SIZE);

        return 0;

fail:
        kfree(tfm_ctx->session_ctx);
        tfm_ctx->session_ctx = NULL;
        return -EINVAL;
}

static int setup_instance_and_sessions(struct crypto_aead *crypto_aead,
                                       CpaCySymCipherDirection direction)
{
        struct crypto_tfm *aead_tfm = crypto_aead_tfm(crypto_aead);
        struct icp_authenc_rfc3686_tfm_ctx *tfm_ctx = crypto_tfm_ctx(aead_tfm);

        CpaStatus status = CPA_STATUS_SUCCESS;
        CpaCySymSessionSetupData sd = { 0 };

        /* Get instance */
        tfm_ctx->instance = get_instance(current_thread_info()->cpu);

        sd.sessionPriority = CPA_CY_PRIORITY_NORMAL;
        sd.symOperation = CPA_CY_SYM_OP_ALGORITHM_CHAINING;

        sd.cipherSetupData.cipherAlgorithm = CPA_CY_SYM_CIPHER_AES_CTR;
        sd.cipherSetupData.pCipherKey = tfm_ctx->enckey;
        sd.cipherSetupData.cipherKeyLenInBytes = tfm_ctx->enckeylen;
        sd.cipherSetupData.cipherDirection = direction;

        sd.hashSetupData.hashAlgorithm = CPA_CY_SYM_HASH_SHA1;
        sd.hashSetupData.hashMode = CPA_CY_SYM_HASH_MODE_AUTH;
        sd.hashSetupData.authModeSetupData.authKey = tfm_ctx->authkey;
        sd.hashSetupData.authModeSetupData.authKeyLenInBytes =
                tfm_ctx->authkeylen;
        sd.hashSetupData.digestResultLenInBytes =
                crypto_aead_crt(crypto_aead)->authsize;

        if (direction == CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT) {
                sd.algChainOrder = CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH;
                sd.verifyDigest = CPA_FALSE;
                sd.digestIsAppended = CPA_FALSE;
        } else {
                sd.algChainOrder = CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER;
                sd.verifyDigest = CPA_TRUE;
                sd.digestIsAppended = CPA_TRUE;
        }

        status = cpaCySymInitSession(tfm_ctx->instance,
                                     icp_authenc_rfc3686_callback,
                                     &sd,
                                     tfm_ctx->session_ctx);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed cpaCySymInitSession\n", __func__);
                return -EINVAL;
        }
        atomic_set(&tfm_ctx->ctr, CONFIGURED);
        return status;
}

static int icp_authenc_rfc3686_aes_ctr_sha1_hmac_crypt(
        struct aead_request *req,
        CpaCySymCipherDirection direction)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        void *iv = req->iv;
        struct crypto_aead *aead = crypto_aead_reqtfm(req);
        struct aead_tfm *aead_crt = crypto_aead_crt(aead);
        struct icp_authenc_rfc3686_req_ctx *req_ctx = aead_request_ctx(req);
        struct icp_authenc_rfc3686_tfm_ctx *tfm_ctx =
                crypto_tfm_ctx(crypto_aead_tfm(aead));
        uint32_t count =
                cpu_to_be32(!(req->base.flags & CRYPTO_TFM_REQ_MERAKI_MODE));
        size_t assoc_separation;

        /* Make sure the client understands the contiguity requirement. */
        if (!(req->base.flags & CRYPTO_TFM_REQ_MERAKI_QAT_CONTIG)) {
                goto fail;
        }

        /* Sanity-check contiguity - we require that the transformation occur
         * in-place, and that the associated data, per-packet IV and payload all
         * occur contiguously. */
        if (req->src != req->dst || sg_next(req->src)) {
                goto fail;
        }

        assoc_separation =
                ((Cpa8U *)sg_virt(req->src)) - ((Cpa8U *)sg_virt(req->assoc));
        if (assoc_separation != req->assoc->length + CTR_RFC3686_IV_SIZE) {
                goto fail;
        }

        /* get QAT instance & setup sessions for the first packet in the flow */
        if (CONFIGURE == atomic_read(&tfm_ctx->ctr)) {
            spin_lock(&tfm_ctx->saInitSpinLock);
            status = CPA_STATUS_FAIL;
            if (CONFIGURE == atomic_read(&tfm_ctx->ctr)) {
                status = setup_instance_and_sessions(aead, direction);
            }
            spin_unlock(&tfm_ctx->saInitSpinLock);
            if (status != CPA_STATUS_SUCCESS) {
                goto fail;
            }
        }

        req_ctx->bufferList.pBuffers = &req_ctx->flatBuffer;
        req_ctx->bufferList.pPrivateMetaData = req_ctx->__pBufferMeta;
        req_ctx->bufferList.numBuffers = 1;
        req_ctx->flatBuffer.pData = sg_virt(req->assoc);
        req_ctx->flatBuffer.dataLenInBytes =
                req->assoc->length + CTR_RFC3686_IV_SIZE + req->src->length;

        req_ctx->async_req = &req->base;
        memcpy(&req_ctx->counter[0],
               tfm_ctx->nonce,
               CTR_RFC3686_NONCE_SIZE);
        memcpy(&req_ctx->counter[CTR_RFC3686_NONCE_SIZE],
               iv,
               CTR_RFC3686_IV_SIZE);
        memcpy(&req_ctx->counter[CTR_RFC3686_NONCE_SIZE + CTR_RFC3686_IV_SIZE],
               &count,
               sizeof(count));

        req_ctx->opData.sessionCtx = tfm_ctx->session_ctx;
        req_ctx->opData.packetType = CPA_CY_SYM_PACKET_TYPE_FULL;
        // "For block ciphers in CTR mode, this is the counter"
        req_ctx->opData.pIv = req_ctx->counter;
        req_ctx->opData.ivLenInBytes = CTR_RFC3686_BLOCK_SIZE;
        req_ctx->opData.hashStartSrcOffsetInBytes = 0;
        req_ctx->opData.pAdditionalAuthData = NULL;
        req_ctx->opData.cryptoStartSrcOffsetInBytes =
                req->assoclen + CTR_RFC3686_IV_SIZE;

        if (direction == CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT) {
                /* The message digest length isn't included, so we don't need to
                 * exclude it. */
                req_ctx->opData.messageLenToCipherInBytes = req->cryptlen;
                req_ctx->opData.messageLenToHashInBytes =
                        req->assoclen + CTR_RFC3686_IV_SIZE + req->cryptlen;
                /* As per the manual, work around IXA00378322. */
                req_ctx->opData.pDigestResult =
                        sg_virt(req->src) + req->cryptlen;
        } else {
                /* Exclude the message digest length, which is included. */
                Cpa32U messageLen = req->cryptlen - aead_crt->authsize;
                req_ctx->opData.messageLenToCipherInBytes = messageLen;
                req_ctx->opData.messageLenToHashInBytes =
                        req->assoclen + CTR_RFC3686_IV_SIZE + messageLen;
        }

        status = cpaCySymPerformOp(
                tfm_ctx->instance,
                (void *)req_ctx,
                &req_ctx->opData,
                &req_ctx->bufferList,
                &req_ctx->bufferList,
                NULL);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: cpaCySymPerformOp failed\n", __func__);
                goto fail;
        }

        return -EINPROGRESS;

fail:
        return -EINVAL;
}

static int icp_authenc_rfc3686_aes_ctr_sha1_hmac_encrypt(struct aead_request *req)
{
    return icp_authenc_rfc3686_aes_ctr_sha1_hmac_crypt(
            req,
            CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT);
}

static int icp_authenc_rfc3686_aes_ctr_sha1_hmac_decrypt(struct aead_request *req)
{
    return icp_authenc_rfc3686_aes_ctr_sha1_hmac_crypt(
            req,
            CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT);
}

static struct crypto_alg icp_authenc_rfc3686_aes_ctr_sha1_hmac = {
        .cra_name        = "authenc(hmac(sha1),rfc3686(ctr(aes)))",
        .cra_driver_name = "icp_qat_rfc3686_aes_ctr_sha1_hmac",
        .cra_priority    = 4001,
        .cra_flags       = CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
        .cra_blocksize   = AES_BLOCK_SIZE,
        .cra_ctxsize     = sizeof(struct icp_authenc_rfc3686_tfm_ctx),
        .cra_type        = &crypto_aead_type,
        .cra_module      = THIS_MODULE,
        .cra_init        = icp_authenc_rfc3686_aes_ctr_sha1_hmac_cra_init,
        .cra_exit        = icp_authenc_rfc3686_aes_ctr_sha1_hmac_cra_exit,
        .cra_u           = {
                .aead = {
                        .setkey  = icp_authenc_rfc3686_aes_ctr_sha1_hmac_setkey,
                        .encrypt = icp_authenc_rfc3686_aes_ctr_sha1_hmac_encrypt,
                        .decrypt = icp_authenc_rfc3686_aes_ctr_sha1_hmac_decrypt,
                        .ivsize = CTR_RFC3686_IV_SIZE,
                        .maxauthsize = SHA1_DIGEST_SIZE
                },
        },
};

static int icp_authenc_rfc3686_aes_ctr_sha1_hmac_driver_init(void)
{
        return crypto_register_alg(&icp_authenc_rfc3686_aes_ctr_sha1_hmac);
}

static int icp_authenc_rfc3686_aes_ctr_sha1_hmac_driver_exit(void)
{
        return crypto_unregister_alg(&icp_authenc_rfc3686_aes_ctr_sha1_hmac);
}

struct icp_qat_alg_driver authenc_rfc3686_aes_ctr_sha1_hmac_driver = {
        .driver_name = "icp_qat_authenc_rfc3686_aes_ctr_sha1_hmac",
        .driver_init = icp_authenc_rfc3686_aes_ctr_sha1_hmac_driver_init,
        .driver_exit = icp_authenc_rfc3686_aes_ctr_sha1_hmac_driver_exit,
};
