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
 * icp_netkey_shim.c
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

#include "cpa.h"
#include "cpa_cy_sym.h"
#include "cpa_cy_im.h"
#include "icp_netkey_shim.h"
#include "icp_netkey_shim_linux.h"

/* request ctx structure allocated for every request */
struct icp_netkey_req_ctx {
	/* ptr to meta data memory required by src CpaBufferList */
	u8 *pSrcMetaData;
	/* ptr to meta data memory required by dst CpaBufferList */
	u8 *pDstMetaData;
	/* Src CpaBufferList */
	CpaBufferList srcBufferList
	__attribute__ ((aligned(ICP_QAT_SRC_BUFF_ALIGN)));
	/* Dst CpaBufferList */
	CpaBufferList dstBufferList
	__attribute__ ((aligned(ICP_QAT_SRC_BUFF_ALIGN)));
	/* Flatbuffers to describe src and dst data */
	CpaFlatBuffer srcFlatBuffers[ICP_MAX_NUM_BUFFERS];
	CpaFlatBuffer dstFlatBuffers[ICP_MAX_NUM_BUFFERS];
	CpaCySymOpData sym_op_data;     /* QA API request */
	struct aead_request *req;
};

CpaInstanceHandle get_instance(Cpa32U core)
{
        CpaInstanceHandle cpaInst;
        struct icp_qat_instances *inst = &(instsByCore[core]);

        cpaInst = inst->instances[(inst->index)++ % inst->total];
        return cpaInst;
}

/*  Callback function */
static void symCallback(void *pCallbackTag,
                        CpaStatus status,
                        const CpaCySymOp operationType,
                        void *pOpData, CpaBufferList * pDstBuffer,
                        CpaBoolean verifyResult)
{
        struct icp_netkey_req_ctx *req_ctx =
                (struct icp_netkey_req_ctx *)pCallbackTag;
        struct aead_request *req = NULL;
        int res = -EBADMSG;

        req = req_ctx->req;

        if (unlikely(CPA_STATUS_SUCCESS != status)) {
                printk(KERN_ERR "%s: symCallback failed, status = (%d)\n",
                       __func__, status);
        } else {
                if (CPA_TRUE == verifyResult) {
                        /* set the result to success */
                        res = 0;
                } else {
                        printk(KERN_ERR "%s: verifyResult == CPA_FALSE (%d)\n",
                               __func__, verifyResult);
                }
        }

        req->base.complete(&(req->base), res);
}

/* Function used to convert scatterlist to bufferlist */
int
sl_to_bl(CpaBufferList * pBufferList, const struct scatterlist *const pSrc)
{
        struct scatterlist *pCurr = (struct scatterlist *)pSrc;
        while ((pCurr != NULL)
               && (pBufferList->numBuffers < ICP_MAX_NUM_BUFFERS)) {

                pBufferList->pBuffers[pBufferList->numBuffers].pData =
                        sg_virt(pCurr);
                pBufferList->pBuffers[pBufferList->numBuffers].dataLenInBytes =
                        pCurr->length;

                pBufferList->numBuffers++;
                pCurr = sg_next(pCurr);
        }

        if (pBufferList->numBuffers == ICP_MAX_NUM_BUFFERS) {
                printk(KERN_ERR
                       "%s: numBuffers in BufferList has exceeded max "
                       "value of %d\n", __func__, ICP_MAX_NUM_BUFFERS);
                return -ENOMEM;
        }
        return 0;

}

/* Initialise CpaBufferList based on pAssoc, iv and pSrc value */
static int icp_init_bufflist_from_assoc_iv_src(struct icp_netkey_req_ctx *const opdata,
                                               struct scatterlist *const pAssoc,
                                               struct scatterlist *const pSrc,
											   struct scatterlist * const pDst,
                                               const Cpa8U * const iv,
                                               const Cpa32U iv_len)
{
	CpaBufferList *pSrcBuffList = &(opdata->srcBufferList);
	CpaBufferList *pDstBuffList = &(opdata->dstBufferList);
	pSrcBuffList->pPrivateMetaData = opdata->pSrcMetaData;
	pDstBuffList->pPrivateMetaData = opdata->pDstMetaData;

	if (pDst == pSrc)
	{
		/* If pAssoc, iv and pSrc are described in 1 contiguous section of
		   memory then we should create the CpaBufferList with 1 buffer to
		   ensure the optimized path is chosen */
		/* iv is just after the assoc data */
		if ((sg_virt(pAssoc) + pAssoc->length == iv)
			&& (iv + iv_len == sg_virt(pSrc))   /* src data is just after iv */
			&&(NULL == sg_next(pSrc)))
		{        /* src data in 1 sg */
			/* Can use optimized path for better performance */
			pSrcBuffList->numBuffers = 1;
			pSrcBuffList->pBuffers = opdata->srcFlatBuffers;
			pSrcBuffList->pBuffers[0].pData = sg_virt(pAssoc);
			pSrcBuffList->pBuffers[0].dataLenInBytes =
				pAssoc->length + iv_len + pSrc->length;
			return 0;
		}
	}
    /* Note that for the case of (pDst!=pSrc), no test of whether 'pAssoc', 'iv' and 'pSrc' are described in 1 contiguous */
	/* section of memory is made, and so if indeed 'pAssoc', 'iv' and 'pSrc' are in 1 contiguous block of memory */
    /* we do NOT ensure the optimised path is chosen, which is in contrast to the (pDst==pSrc) case above. */
	if (pAssoc->length > 0)
	{
		/* 2 buffers will be initialised before calling
		   sl_to_bl: pAssoc and pIV */
		pSrcBuffList->numBuffers = 2;
		pSrcBuffList->pBuffers = opdata->srcFlatBuffers;
		pSrcBuffList->pBuffers[0].pData = sg_virt(pAssoc);
		pSrcBuffList->pBuffers[0].dataLenInBytes = pAssoc->length;
		pSrcBuffList->pBuffers[1].pData = (Cpa8U *)iv;
		pSrcBuffList->pBuffers[1].dataLenInBytes = iv_len;
		if (sl_to_bl(pSrcBuffList, pSrc) != 0)
		{
			printk(KERN_ERR "%s: Failure to populate pSrcBuffList.\n", __func__);
			return -ENOMEM;
		}
		if (pDst != pSrc)
		{
			pDstBuffList->numBuffers = 2;
			pDstBuffList->pBuffers = opdata->dstFlatBuffers;
			pDstBuffList->pUserData = NULL;
			pDstBuffList->pBuffers[0].pData = sg_virt(pAssoc);
			pDstBuffList->pBuffers[0].dataLenInBytes = pAssoc->length;
			pDstBuffList->pBuffers[1].pData = (Cpa8U *)iv;
			pDstBuffList->pBuffers[1].dataLenInBytes = iv_len;
			if (sl_to_bl(pDstBuffList, pDst) != 0)
			{
				printk(KERN_ERR "%s: Failure to populate pDstBuffList.\n", __func__);
				return -ENOMEM;
			}
			return 0;
		}
		else return 0;
	}
	else if (pAssoc->length == 0)
	{
		/* Just 1 buffer will be initialised before calling
		   sl_to_bl: pIV since pAssoc->length == 0 */
		pSrcBuffList->numBuffers = 1;
		pSrcBuffList->pBuffers = opdata->srcFlatBuffers;
		pSrcBuffList->pBuffers[0].pData = (Cpa8U *)iv;
		pSrcBuffList->pBuffers[0].dataLenInBytes = iv_len;
		if (sl_to_bl(pSrcBuffList, pSrc) != 0)
		{
			printk(KERN_ERR "%s: Failure to populate pSrcBuffList.\n", __func__);
			return -ENOMEM;
		}
		if (pDst != pSrc)
		{
			pDstBuffList->numBuffers = 1;
			pDstBuffList->pBuffers = opdata->dstFlatBuffers;
			pDstBuffList->pUserData = NULL;
			pDstBuffList->pBuffers[0].pData = (Cpa8U *)iv;
			pDstBuffList->pBuffers[0].dataLenInBytes = iv_len;
			if (sl_to_bl(pDstBuffList, pDst) != 0)
			{
				printk(KERN_ERR "%s: Failure to populate pDstBuffList.\n", __func__);
				return -ENOMEM;
			}
			return 0;
		}
		else return 0;
	}
	else
	{
		printk(KERN_ERR
			   "%s: None of the conditions needed to initialise the BufferList have been met.\n", __func__);
		return -ENOMEM;
	}
}

/* Perform sym chaining */
static int perform_sym_chaining(struct aead_request *req,
                                Cpa8U * iv,
                                const Cpa32U iv_len,
                                const CpaCySymCipherDirection cipherDirection,
                                CpaBoolean geniv)
{
	CpaStatus status = CPA_STATUS_SUCCESS;
	struct crypto_aead *aead_tfm = crypto_aead_reqtfm(req);
	struct icp_netkey_ctx *ctx = crypto_tfm_ctx(crypto_aead_tfm(aead_tfm));
	struct icp_netkey_req_ctx *pReqCtx = NULL;
	CpaCySymSessionCtx pSessionCtx = ctx->session_ctx;
	Cpa32U messageLen = 0;

	pReqCtx = aead_request_ctx(req);
	pReqCtx->req = req;
	/* space for meta data after icp_netkey_req_ctx */
	pReqCtx->pSrcMetaData = ((u8 *) pReqCtx) + sizeof(struct icp_netkey_req_ctx);
	pReqCtx->pDstMetaData = pReqCtx->pSrcMetaData + ctx->metaSize;

	/* Describe input SGLs as one CpaBufferList */
	if (0 != icp_init_bufflist_from_assoc_iv_src(pReqCtx,
												 req->assoc,
												 req->src,
												 req->dst,
												 iv,
												 iv_len))
	{
		return -ENOMEM;
	}

	/* Setup CpaCySymOpData */
	pReqCtx->sym_op_data.packetType = CPA_CY_SYM_PACKET_TYPE_FULL;
	pReqCtx->sym_op_data.sessionCtx = pSessionCtx;
	pReqCtx->sym_op_data.ivLenInBytes = iv_len;
	pReqCtx->sym_op_data.hashStartSrcOffsetInBytes = 0;
	pReqCtx->sym_op_data.pAdditionalAuthData = NULL;

	if ((CPA_TRUE == geniv) && (CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT == cipherDirection))
	{
		/* Generating IV - set iv=0 and encrypt an extra block */
		pReqCtx->sym_op_data.pIv = ctx->pIvZero;
		messageLen = req->cryptlen;
		pReqCtx->sym_op_data.cryptoStartSrcOffsetInBytes =
			req->assoclen;
		pReqCtx->sym_op_data.messageLenToCipherInBytes =
			messageLen + iv_len;
		pReqCtx->sym_op_data.messageLenToHashInBytes =
			req->assoclen + messageLen + iv_len;
	}
	else if ((CPA_FALSE == geniv) && (CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT == cipherDirection))
	{
		/* Use given iv */
		pReqCtx->sym_op_data.pIv = iv;
		messageLen =
			req->cryptlen - crypto_aead_crt(aead_tfm)->authsize;
		pReqCtx->sym_op_data.cryptoStartSrcOffsetInBytes =
			req->assoclen + iv_len;
		pReqCtx->sym_op_data.messageLenToCipherInBytes =
			messageLen;
		pReqCtx->sym_op_data.messageLenToHashInBytes =
			req->assoclen + iv_len + messageLen;
	}
	else if ((CPA_FALSE == geniv) && (CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT == cipherDirection))
	{
		/* Use given iv */
		pReqCtx->sym_op_data.pIv = iv;
		messageLen = req->cryptlen;
		pReqCtx->sym_op_data.cryptoStartSrcOffsetInBytes = req->assoclen + iv_len;
		pReqCtx->sym_op_data.messageLenToCipherInBytes = messageLen;
		pReqCtx->sym_op_data.messageLenToHashInBytes = req->assoclen + iv_len + messageLen;
	}
	else
	{
		printk(KERN_ERR
			   "%s: Operation not supported. "
			   "encrypt, givencrypt and decrypt operation supported"
			   " in this release.\n", __func__);
		return -EINVAL;
	}

	if (req->dst == req->src)
	{
		status = cpaCySymPerformOp(ctx->instance,
								   (void *)pReqCtx,
								   &(pReqCtx->sym_op_data),
								   &(pReqCtx->srcBufferList),
								   &(pReqCtx->srcBufferList), NULL);
	}
	else /* req->dst != req->src */
	{
		status = cpaCySymPerformOp(ctx->instance,
								   (void *)pReqCtx,
								   &(pReqCtx->sym_op_data),
								   &(pReqCtx->srcBufferList),
								   &(pReqCtx->dstBufferList), NULL);
	}
	if (CPA_STATUS_RETRY == status) {
		return -EBUSY;
	}
	if (CPA_STATUS_SUCCESS != status) {
		printk(KERN_ERR
			   "%s: cpaCySymPerformOp failed. "
			   "(status = (%d))\n", __func__, status);
		return -EINVAL;
	}
	return -EINPROGRESS;
}

/* Setup QAT instance and initialise the enc/dec sessions */
static int setup_instance_and_sessions(struct crypto_aead *aead_tfm,
                                       CpaCySymCipherDirection direction)
{
        struct crypto_tfm *tfm = crypto_aead_tfm(aead_tfm);
        struct icp_netkey_ctx *ctx = crypto_tfm_ctx(tfm);
        CpaCySymSessionCtx session_ctx = ctx->session_ctx;
        CpaCySymSessionSetupData sessionSetupData = { 0 };
        CpaStatus status = CPA_STATUS_SUCCESS;
        unsigned int authkeylen = 0;
        struct thread_info *currThread;

        currThread = current_thread_info();
        /* Get instance */
        ctx->instance = get_instance(currThread->cpu);

        /* Initialise sessions */
        authkeylen = ctx->authkeylen;
        sessionSetupData.sessionPriority = CPA_CY_PRIORITY_HIGH;
        sessionSetupData.cipherSetupData.cipherAlgorithm = ctx->cipherAlgorithm;
        sessionSetupData.cipherSetupData.pCipherKey = ctx->cipherkey;
        sessionSetupData.cipherSetupData.cipherKeyLenInBytes = ctx->enckeylen;
        sessionSetupData.cipherSetupData.cipherDirection = direction;
        sessionSetupData.symOperation = CPA_CY_SYM_OP_ALGORITHM_CHAINING;

        sessionSetupData.hashSetupData.hashAlgorithm = ctx->hashAlgorithm;
        sessionSetupData.hashSetupData.hashMode = CPA_CY_SYM_HASH_MODE_AUTH;
        sessionSetupData.hashSetupData.authModeSetupData.authKey = ctx->authkey;
        sessionSetupData.hashSetupData.authModeSetupData.authKeyLenInBytes =
                ctx->authkeylen;
        sessionSetupData.hashSetupData.digestResultLenInBytes =
                crypto_aead_crt(aead_tfm)->authsize;

        sessionSetupData.digestIsAppended = CPA_TRUE;

        if (CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT == direction) {
                sessionSetupData.algChainOrder =
                        CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH;
                sessionSetupData.verifyDigest = CPA_FALSE;
        } else {
                sessionSetupData.algChainOrder =
                        CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER;
                sessionSetupData.verifyDigest = CPA_TRUE;
        }

        /* Initialize the session */
        status = cpaCySymInitSession(ctx->instance,
                                     symCallback, &sessionSetupData,
                                     session_ctx);
        if (CPA_STATUS_SUCCESS != status) {
                printk(KERN_ERR "%s: Failed to initialize session\n", __func__);
                return -EINVAL;
        }
        atomic_set(&ctx->ctr, CONFIGURED);
        return status;
}

/* Generate IV netkey encryption */
int geniv_encrypt_netkey_shim(struct aead_givcrypt_request *req, u16 blocksize)
{

        struct crypto_aead *aead_tfm = crypto_aead_reqtfm(&req->areq);
        struct icp_netkey_ctx *ctx = crypto_tfm_ctx(crypto_aead_tfm(aead_tfm));
        unsigned int len = 0;
        u64 seq = 0;
        int res = 0;

        /* get QAT instance & setup sessions for the first packet in the flow */
        if (CONFIGURE == atomic_read(&ctx->ctr)) {
                spin_lock(&ctx->saInitSpinLock);
                if (CONFIGURE == atomic_read(&ctx->ctr)) {
                        res = setup_instance_and_sessions(aead_tfm,
                                                          CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT);
                }
                spin_unlock(&ctx->saInitSpinLock);
                if (CPA_STATUS_SUCCESS != res) {
                        printk(KERN_ERR
                               "%s: Failed to get instance and setup sessions\n",
                               __func__);
                        return -EINVAL;
                }
        }

        memcpy(req->giv, ctx->salt, blocksize);
        len = sizeof(u64);
        seq = cpu_to_be64(req->seq);
        memcpy(req->giv + blocksize - len, &seq, len);

        return perform_sym_chaining(&req->areq, req->giv,
                                    blocksize,
                                    CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT,
                                    CPA_TRUE);
}

/* This function initiates netkey decryption */
int icp_netkey_shim_dec(struct aead_request *req, u16 blocksize)
{
        struct crypto_aead *aead_tfm = crypto_aead_reqtfm(req);
        struct icp_netkey_ctx *ctx = crypto_tfm_ctx(crypto_aead_tfm(aead_tfm));
        int res = 0;

        /* get QAT instance & setup sessions for the first packet in the flow */
        if (CONFIGURE == atomic_read(&ctx->ctr)) {
                spin_lock(&ctx->saInitSpinLock);
                if (CONFIGURE == atomic_read(&ctx->ctr)) {
                        res = setup_instance_and_sessions(aead_tfm,
                                                          CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT);
                }
                spin_unlock(&ctx->saInitSpinLock);
                if (CPA_STATUS_SUCCESS != res) {
                        printk(KERN_ERR
                               "%s: Failed to get instance and setup sessions\n",
                               __func__);
                        return -EINVAL;
                }
        }
        return perform_sym_chaining(req, req->iv,
                                    blocksize,
                                    CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT,
                                    CPA_FALSE);
}

/* This function initiates netkey encryption.*/
int icp_netkey_shim_enc(struct aead_request *req, u16 blocksize)
{
	struct crypto_aead *aead_tfm = crypto_aead_reqtfm(req);
	struct icp_netkey_ctx *ctx = crypto_tfm_ctx(crypto_aead_tfm(aead_tfm));
	int res = 0;

	/* get QAT instance & setup sessions for the first packet in the flow */
	if (CONFIGURE == atomic_read(&ctx->ctr)) {
		spin_lock(&ctx->saInitSpinLock);
		if (CONFIGURE == atomic_read(&ctx->ctr)) {
			res = setup_instance_and_sessions(aead_tfm,
											  CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT);
		}
		spin_unlock(&ctx->saInitSpinLock);
		if (CPA_STATUS_SUCCESS != res) {
			printk(KERN_ERR
				   "%s: Failed to get instance and setup sessions\n",
				   __func__);
			return -EINVAL;
		}
	}

	return perform_sym_chaining(req, req->iv,
								blocksize,
								CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT,
								CPA_FALSE);
}

/* Given an aead key blob, extract its rtattr and auth/enc keys. */
int icp_netkey_shim_extractkeys(struct crypto_aead *aead_tfm,
                                const Cpa8U *key,
                                unsigned int keylen,
                                struct rtattr **ppRta,
                                const Cpa8U **pauthkey,
                                unsigned int *pauthkeylen,
                                const Cpa8U **penckey,
                                unsigned int *penckeylen)
{
        struct rtattr *pRta = (void *)key;
        struct crypto_authenc_key_param *pParam = NULL;
        unsigned int authkeylen, enckeylen;

        /* The key we receive is the concatenation of:
           - flags (length is RTA_ALIGN(pRta->rta_len))
           - authentication key (length can be computed)
           - cipher key (length is given by the flags)
        */
        if (!RTA_OK(pRta, keylen)) {
                crypto_aead_set_flags(aead_tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
                printk(KERN_ERR "%s: Bad key\n", __func__);
                return -EINVAL;
        }

        if (pRta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM) {
                crypto_aead_set_flags(aead_tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
                printk(KERN_ERR "%s: Bad key rta_type\n", __func__);
                return -EINVAL;
        }

        if (RTA_PAYLOAD(pRta) < sizeof(*pParam)) {
                crypto_aead_set_flags(aead_tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
                printk(KERN_ERR "%s: Bad key rta_payload\n", __func__);
                return -EINVAL;
        }

        pParam = RTA_DATA(pRta);
        enckeylen = be32_to_cpu(pParam->enckeylen);

        key += RTA_ALIGN(pRta->rta_len);
        keylen -= RTA_ALIGN(pRta->rta_len);

        if (keylen < enckeylen) {
                crypto_aead_set_flags(aead_tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
                printk(KERN_ERR
                       "%s: Bad key, enckeylen is greater than keylen\n",
                       __func__);
                return -EINVAL;
        }

        authkeylen = keylen - enckeylen;

        if (authkeylen > ICP_MAX_AUTH_KEYSIZE) {
                printk(KERN_ERR "%s: Auth key exceeds defined size\n", __func__);
                return -EINVAL;
        }

        if (enckeylen > ICP_MAX_CIPHER_KEYSIZE) {
                printk(KERN_ERR "%s: Cipher key exceeds defined size\n", __func__);
                return -EINVAL;
        }


        *ppRta = pRta;
        *pauthkey = key;
        *penckey = key + authkeylen;
        *pauthkeylen = authkeylen;
        *penckeylen = enckeylen;

        return 0;
}

/* This function creates the key
 */
int icp_netkey_shim_setkey(struct crypto_aead *aead_tfm,
                           const Cpa8U * key, unsigned int keylen,
                           CpaCySymCipherAlgorithm cipherAlgorithm,
                           CpaCySymHashAlgorithm hashAlgorithm, u16 blocksize)
{
        struct rtattr *pRta;
        struct icp_netkey_ctx *ctx = crypto_tfm_ctx(crypto_aead_tfm(aead_tfm));
        CpaCySymSessionCtx session_ctx = ctx->session_ctx;
        const Cpa8U *authkey, *enckey;
        int res = 0;
        unsigned int count = 0;
        CpaStatus status = CPA_STATUS_SUCCESS;

        res = icp_netkey_shim_extractkeys(aead_tfm,
                                          key,
                                          keylen,
                                          &pRta,
                                          &authkey,
                                          &ctx->authkeylen,
                                          &enckey,
                                          &ctx->enckeylen);
        if (res < 0) {
                return res;
        }

        if (atomic_dec_and_test(&ctx->ctr)) {
            printk(KERN_INFO
                   "%s: Re-initialising previous session\n", __func__);

            status = cpaCySymRemoveSession(ctx->instance, session_ctx);
            while ((CPA_STATUS_RETRY == status)
                   && (count < ICP_RETRY_COUNT))
            {
                /* Wait a while before retry */
                set_current_state((long)TASK_INTERRUPTIBLE);
                schedule_timeout(ICP_SESSION_REMOVE_TIME);
                count++;
                status = cpaCySymRemoveSession(ctx->instance, session_ctx);
            }
            if (CPA_STATUS_SUCCESS != status)
            {
                printk(KERN_ERR
                       "%s: Failed to remove session\n", __func__);
                return -EINVAL;
            }
            if (ctx->pIvZero)
            {
                kfree(ctx->pIvZero);
                ctx->pIvZero = NULL;
            }
        }

        /* Get salt */
        res = crypto_rng_get_bytes(crypto_default_rng, ctx->salt, blocksize);
        if (res != 0) {
                printk(KERN_ERR "%s: crypto_rng_get_bytes error\n", __func__);
                return res;
        }

        /* Allocate pIvZero memory */
        ctx->pIvZero = kzalloc(blocksize, GFP_KERNEL);
        if (NULL == ctx->pIvZero) {
                printk(KERN_ERR "%s: Failed to alloc pIvZero\n", __func__);
                return -ENOMEM;
        }

        /* capture details for use on session initialisation later */
        ctx->cipherAlgorithm = cipherAlgorithm;
        ctx->hashAlgorithm = hashAlgorithm;

        memcpy(ctx->authkey, authkey, ctx->authkeylen);
        memcpy(ctx->cipherkey, enckey, ctx->enckeylen);

        atomic_set(&ctx->ctr, CONFIGURE);

        return 0;
}

/* This function is used to set up icp_netkey_ctx */
int icp_netkey_shim_init(struct crypto_tfm *tfm)
{
	struct icp_netkey_ctx *ctx = crypto_tfm_ctx(tfm);
	CpaStatus status = CPA_STATUS_SUCCESS;
	CpaCySymSessionSetupData sessionSetupData = { 0 };
	Cpa32U sessionSize = 0;
	int res = 0;
	CpaInstanceHandle firstInst = qat_instances.instances[0];

	ctx->instance = NULL;
	ctx->metaSize = 0;

	/* get meta data size */
	status = cpaCyBufferListGetMetaSize(firstInst, ICP_MAX_NUM_BUFFERS,
										&ctx->metaSize);
	if (CPA_STATUS_SUCCESS != status) {
		printk(KERN_ERR "%s: Failed to get meta size\n", __func__);
		return -EINVAL;
	}

	/* Set reqsize - allow 2 metaSizes - one for srcBufferList's privateMetaData, the other for dstBufferList' privateMetaData. */
	tfm->crt_aead.reqsize = sizeof(struct icp_netkey_req_ctx) + (2 * ctx->metaSize);
	atomic_set(&ctx->ctr, CONFIGURE);

	/* alloc session memory */
	/* Note: for current driver the session size does not depend on
	 *  the setup data */
	status = cpaCySymSessionCtxGetSize(firstInst, &sessionSetupData,
									   &sessionSize);
	if (CPA_STATUS_SUCCESS != status) {
		printk(KERN_ERR "%s: Failed to get session size\n", __func__);
		return -EINVAL;
	}
	ctx->session_ctx = kzalloc(sessionSize, GFP_KERNEL);
	if (NULL == ctx->session_ctx) {
		printk(KERN_ERR "%s: Failed to alloc session\n", __func__);
		return -ENOMEM;
	}

	/* Get default rng */
	res = crypto_get_default_rng();
	if (res != 0) {
		printk(KERN_ERR "%s: Failed to get default rng\n", __func__);
		kfree(ctx->session_ctx);
		return res;
	}

	/* initialise session specific spinlock */
	spin_lock_init(&ctx->saInitSpinLock);

	return 0;
}

/* This function is used to remove netkey encrypt/decrypt session */
void icp_netkey_shim_exit(struct crypto_tfm *tfm)
{
        struct icp_netkey_ctx *ctx =
                (struct icp_netkey_ctx *)crypto_tfm_ctx(tfm);
        CpaCySymSessionCtx session_ctx = ctx->session_ctx;
        CpaStatus status = CPA_STATUS_SUCCESS;

        unsigned int count = 0;

        if (atomic_dec_and_test(&ctx->ctr)) {

                status = cpaCySymRemoveSession(ctx->instance, session_ctx);

                while ((CPA_STATUS_RETRY == status)
                       && (count < ICP_RETRY_COUNT)) {
                        /* Wait a while before retry */
                        set_current_state((long)TASK_INTERRUPTIBLE);
                        schedule_timeout(ICP_SESSION_REMOVE_TIME);

                        count++;
                        status =
                                cpaCySymRemoveSession(ctx->instance, session_ctx);
                }
                if (CPA_STATUS_SUCCESS != status) {
                        printk(KERN_ERR
                               "%s: Failed to remove session\n", __func__);
                }
        }

        /* Free default rng */
        crypto_put_default_rng();

        /* Free session memory */
		if (session_ctx)
		{
			kfree(session_ctx);
			ctx->session_ctx = NULL;
		}

        /* Free pIvZero memory */
		if (ctx->pIvZero)
		{
			kfree(ctx->pIvZero);
			ctx->pIvZero = NULL;
		}
}

int icp_netkey_driver_init(u16 blocksize, struct crypto_alg *alg)
{
	return crypto_register_alg(alg);
}

int icp_netkey_driver_exit(struct crypto_alg *alg)
{
	return crypto_unregister_alg(alg);
}
