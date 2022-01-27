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
 * icp_netkey_hash.c
 *
 * This is an implementation of netkey using
 *  Quick Assist API for the Linux Kernel Crypto Framework
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <crypto/authenc.h>
#include <linux/rtnetlink.h>
#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/algapi.h>
#include <crypto/rng.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>

#include "cpa.h"
#include "cpa_cy_sym.h"
#include "cpa_cy_im.h"
#include "icp_netkey_hash.h"
#include "icp_netkey_shim_linux.h"

int icp_netkey_hash_module_init(void)
{
        return 0;
}

void icp_netkey_hash_module_exit(void)
{
}

int icp_netkey_hash_cra_init(struct crypto_tfm *tfm)
{
        struct icp_netkey_hash_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);

        tfm_ctx->initialized = false;

        tfm_ctx->session_ctx = NULL;
        tfm_ctx->instance = get_instance(current_thread_info()->cpu);
        tfm_ctx->key = NULL;

        return 0;
}


void icp_netkey_hash_cra_exit(struct crypto_tfm *tfm)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        unsigned int count = 0;
        struct icp_netkey_hash_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);

        if (tfm_ctx->initialized) {
                status =
                        cpaCySymRemoveSession(tfm_ctx->instance,
                                              tfm_ctx->session_ctx);
                while (CPA_STATUS_RETRY == status && count < ICP_RETRY_COUNT) {
                        /* Wait a while before retry */
                        set_current_state((long)TASK_INTERRUPTIBLE);
                        schedule_timeout(ICP_SESSION_REMOVE_TIME);
                        count++;
                        status = cpaCySymRemoveSession(
                                tfm_ctx->instance, tfm_ctx->session_ctx);
                }
                if (CPA_STATUS_SUCCESS != status) {
                        printk(KERN_ERR "%s: Failed to remove session\n",
                               __func__);
                }
        }
        kfree(tfm_ctx->session_ctx);
        kfree(tfm_ctx->key);
}


int icp_netkey_hash_init(struct ahash_request *req)
{
        struct icp_netkey_hash_req_ctx *req_ctx = ahash_request_ctx(req);
        req_ctx->bufferList.pBuffers = req_ctx->flatBuffers;
        req_ctx->bufferList.numBuffers = 0;
        req_ctx->bufferList.pPrivateMetaData = req_ctx->__pBufferMeta;
        req_ctx->data = NULL;
        req_ctx->len = 0;
        return 0;
}


static void icp_netkey_hash_destroy(struct icp_netkey_hash_req_ctx *req_ctx)
{
        kfree(req_ctx->data);
        req_ctx->data = NULL;
}

int icp_netkey_hash_update(struct ahash_request *req)
{
        struct icp_netkey_hash_req_ctx *req_ctx = ahash_request_ctx(req);
        u8 *tmp;
        unsigned int total;
        size_t copied;

        if (req->nbytes > 0) {
                total = req_ctx->len + req->nbytes;
                if (total > PAGE_SIZE) {
                        icp_netkey_hash_destroy(req_ctx);
                        printk(KERN_ERR
                               "%s: hash scatterlist would exceed page size\n",
                               __func__);
                        return -ENOMEM;
                }

                tmp = krealloc(req_ctx->data, total, GFP_ATOMIC);
                if (!tmp) {
                        icp_netkey_hash_destroy(req_ctx);
                        printk(KERN_ERR "%s: krealloc failed\n", __func__);
                        return -ENOMEM;
                }

                req_ctx->data = tmp;
                copied = sg_copy_to_buffer(req->src,
                                           sg_nents(req->src),
                                           &req_ctx->data[req_ctx->len],
                                           req->nbytes);
                if (copied < req->nbytes) {
                        icp_netkey_hash_destroy(req_ctx);
                        printk(KERN_ERR "%s: req->src too short\n", __func__);
                        return -EINVAL;
                }

                req_ctx->len += req->nbytes;

                req_ctx->flatBuffers[0].pData = req_ctx->data;
                req_ctx->flatBuffers[0].dataLenInBytes = req_ctx->len;
                req_ctx->bufferList.numBuffers = 1;
        }

        return 0;
}

int icp_netkey_hash_final(struct ahash_request *req)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
        struct icp_netkey_hash_req_ctx *req_ctx = ahash_request_ctx(req);
        struct icp_netkey_hash_tfm_ctx *tfm_ctx =
                crypto_tfm_ctx(crypto_ahash_tfm(ahash));

        req_ctx->async_req = &req->base;

        /* At this point req_ctx->bufferList has already been initialized. */

        req_ctx->opData.sessionCtx = tfm_ctx->session_ctx;
        req_ctx->opData.packetType = CPA_CY_SYM_PACKET_TYPE_FULL;
        req_ctx->opData.hashStartSrcOffsetInBytes = 0;
        req_ctx->opData.messageLenToHashInBytes = req_ctx->len;
        req_ctx->opData.pDigestResult = req->result;

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
        icp_netkey_hash_destroy(req_ctx);
        icp_netkey_hash_init(req);
        return status;
}

int icp_netkey_hash_finup(struct ahash_request *req)
{
        /* Because this is the final update, we can avoid stashing the contents
         * of the input scatterlist and instead convert it directly to an Intel
         * buffer list. */
        struct icp_netkey_hash_req_ctx *req_ctx = ahash_request_ctx(req);
        struct scatterlist *sg;
        CpaFlatBuffer *cpabuf;
        unsigned int remaining = req->nbytes, take;
        for (sg = req->src; remaining; sg = sg_next(sg)) {
                if (!sg) {
                        icp_netkey_hash_destroy(req_ctx);
                        printk(KERN_ERR "%s: req->src too short\n", __func__);
                        return -EINVAL;
                }

                if (req_ctx->bufferList.numBuffers == ICP_MAX_NUM_BUFFERS) {
                        icp_netkey_hash_destroy(req_ctx);
                        printk(KERN_ERR "%s: req->src too long\n", __func__);
                        return -EINVAL;
                }

                take = min(sg->length, remaining);

                cpabuf = &req_ctx->flatBuffers[req_ctx->bufferList.numBuffers];
                cpabuf->pData = sg_virt(sg);
                cpabuf->dataLenInBytes = take;

                remaining -= take;
                req_ctx->len += take;
                req_ctx->bufferList.numBuffers++;
        }

        return icp_netkey_hash_final(req);
}

int icp_netkey_hash_digest(struct ahash_request *req)
{
        return icp_netkey_hash_init(req) ?: icp_netkey_hash_finup(req);
}

static void
icp_netkey_hash_callback(void *pCallbackTag,
                         CpaStatus status,
                         const CpaCySymOp operationType,
                         void *pOpData,
                         CpaBufferList *pDstBuffer,
                         CpaBoolean verifyResult)
{
        struct icp_netkey_hash_req_ctx *req_ctx;
        req_ctx = (struct icp_netkey_hash_req_ctx *)pCallbackTag;
        icp_netkey_hash_destroy(req_ctx);
        req_ctx->async_req->complete(req_ctx->async_req, 0);
}

int icp_netkey_hash_setkey(struct crypto_ahash *tfm,
                           const u8 *key, unsigned int keylen,
                           CpaCySymHashAlgorithm hashAlgorithm,
                           CpaCySymHashMode hashMode)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        CpaCySymSessionSetupData sessionSetupData = {0};
        Cpa32U sessionSize = 0;
        Cpa32U metaSize = 0;
        struct icp_netkey_hash_tfm_ctx *tfm_ctx =
                crypto_tfm_ctx(crypto_ahash_tfm(tfm));

        status = cpaCyBufferListGetMetaSize(
                tfm_ctx->instance, ICP_MAX_NUM_BUFFERS, &metaSize);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed to get meta size\n", __func__);
                return -EINVAL;
        }
        crypto_ahash_set_reqsize(tfm,
                                 (sizeof(struct icp_netkey_hash_req_ctx) +
                                  metaSize));


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
        if ((tfm_ctx->key = kmalloc(keylen, GFP_KERNEL)) == NULL) {
                printk(KERN_ERR "%s: Failed kmalloc key\n", __func__);
                goto fail;
        }
        memcpy(tfm_ctx->key, key, keylen);

        sessionSetupData.sessionPriority = CPA_CY_PRIORITY_NORMAL;
        sessionSetupData.symOperation = CPA_CY_SYM_OP_HASH;
        sessionSetupData.hashSetupData.hashAlgorithm = hashAlgorithm;
        sessionSetupData.hashSetupData.hashMode = hashMode;
        sessionSetupData.hashSetupData.digestResultLenInBytes =
                crypto_ahash_digestsize(tfm);
        sessionSetupData.hashSetupData.authModeSetupData.authKey = tfm_ctx->key;
        sessionSetupData.hashSetupData.authModeSetupData.authKeyLenInBytes =
                keylen;
        sessionSetupData.digestIsAppended = CPA_FALSE;
        sessionSetupData.verifyDigest = CPA_FALSE;

        status = cpaCySymInitSession(tfm_ctx->instance,
                                     icp_netkey_hash_callback,
                                     &sessionSetupData, tfm_ctx->session_ctx);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed cpaCySymInitSession\n", __func__);
                goto fail;
        }
        tfm_ctx->initialized = true;

        return 0;

fail:
        kfree(tfm_ctx->session_ctx);
        kfree(tfm_ctx->key);
        return -EINVAL;
}
