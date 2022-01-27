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

#include <linux/random.h>
#include <linux/crypto.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <linux/scatterlist.h>
#include <linux/rtnetlink.h>

#include "icp_aead_perf.h"

#define ICP_AEAD_PERF_HDR_LEN 8 /* header length in bytes */

#define RESPONSE_TIMEOUT_MS         5000
#define RESPONSE_TIMEOUT_JIFFY      (RESPONSE_TIMEOUT_MS*HZ/100)

extern unsigned int kp_numThreads;
extern atomic_t arrived;
extern atomic_t bGo;

static int perf_aead_execute_tests(sa_context_t * cs, u32 dataLength,
                                   u64 totalReqs, int dir, u32 keysize, u32 digestsize, 
                                   u32 auth_key_len, u32 blocksize, const char *alg_name);
static int perf_aead_performLoop(sa_context_t * cs, u64 totalReqsToSend, int dir);
static int perf_aead_init_data(sa_context_t * cs, u32 dataLength, int dir, u32 keysize, u32 digestsize,
                               u32 auth_key_len, u32 blocksize, const char *alg_name);
static int perf_aead_init_session(sa_context_t * cs, int dir, u32 digestsize, const char *alg_name);
static int perf_aead_cleanup(sa_context_t * cs);

/* Callback function */
static void perf_op_done(struct crypto_async_request *pReq, int err)
{
        sa_context_t *cs = pReq->data;

        if (err != 0) {
                printk("\n Callback error!\n");
        }

        cs->totalResponsesReceived++;

        wake_up_interruptible(&cs->tx_wait_queue);
}

int icp_aead_givencrypt(void *testData)
{
        icp_aead_test_data_t *data = (icp_aead_test_data_t *) testData;
        u64 totalReqsToSend = data->numReq;
        u32 pktSize = data->pktSize;
        u32 keysize = data->keysize;
        u32 digestsize = data->digestsize;
        u32 auth_key_len = data->auth_key_len;
        u32 blocksize = data->blocksize;
        const char *alg_name = data->alg_name;

        if (perf_aead_execute_tests(&data->cs, pktSize, totalReqsToSend, 0, keysize, digestsize,
                                    auth_key_len, blocksize, alg_name)) {
                printk("\nFAILED: AEAD givencrypt performance test\n");
                return -1;
        }

        return 0;

}

int icp_aead_decrypt(void *testData)
{
        icp_aead_test_data_t *data = (icp_aead_test_data_t *) testData;
        u64 totalReqsToSend = data->numReq;
        u32 pktSize = data->pktSize;
        u32 keysize = data->keysize;
        u32 digestsize = data->digestsize;
        u32 auth_key_len = data->auth_key_len;
        u32 blocksize = data->blocksize;
        const char *alg_name = data->alg_name;

        if (perf_aead_execute_tests(&data->cs, pktSize, totalReqsToSend, 1, keysize, digestsize,
                                    auth_key_len, blocksize, alg_name)) {
                printk("\nFAILED: AEAD decrypt performance test\n");
                return -1;
        }

        return 0;

}

int perf_aead_execute_tests(sa_context_t * cs, u32 dataLength, u64 totalReqs,
                            int dir, u32 keysize, u32 digestsize, u32 auth_key_len, u32 blocksize, const char *alg_name)
{
        int res = -1;           /* Non-zero is fail */

        res = perf_aead_init_data(cs, dataLength, dir, keysize, digestsize, auth_key_len,
                                  blocksize, alg_name);
        if (res != 0) {
                return res;
        }
        res = perf_aead_init_session(cs, dir, digestsize, alg_name);
        if (res != 0) {
                /*clean up */
                perf_aead_cleanup(cs);
                return res;
        }
        res = perf_aead_performLoop(cs, totalReqs, dir);

        perf_aead_cleanup(cs);

        return res;

}

int perf_aead_performLoop(sa_context_t * cs, u64 totalReqsToSend, int dir)
{
        int ret;
        u32 curr_offset = 0;


        /* Wait are barrier for all threads */
        atomic_inc(&arrived);
        if (atomic_read(&arrived) < kp_numThreads) {
                while (atomic_read(&bGo) == 0) {
                        yield();
                }
        } else {
                atomic_inc(&bGo);
        }

        rdtscll(cs->startCycleCount);
        /* Send all requests */
        while (cs->totalRequestsSent < totalReqsToSend) {

                ret = wait_event_interruptible_timeout(cs->tx_wait_queue,
                                                       ((cs->totalRequestsSent -
                                                         cs->
                                                         totalResponsesReceived)
                                                        <
                                                        MAX_INFLIGHT_REQUESTS),
                                                       RESPONSE_TIMEOUT_JIFFY);
                if (ret < 0) {
                        printk("\n*** Wait for response interrupted\n");
                        return -1;
                } else
                        if (!
                            ((cs->totalRequestsSent - cs->totalResponsesReceived) <
                             MAX_INFLIGHT_REQUESTS)) {
                                printk("\n*** Failed to receive response from CPM\n");
                                return -1;
                        }
                if (0 == dir) {
                        ret = crypto_aead_givencrypt(cs->pReq[curr_offset]);
                } else {
                        memcpy(cs->pBuff[curr_offset], cs->pTempBuf,
                               cs->buffLen);
                        ret =
                                crypto_aead_decrypt(&(cs->pReq[curr_offset]->areq));

                }
                if (ret == -EBUSY) {
                        /* Retry (multiple threads using same instance) */
                        cs->totalRetries++;
                        yield();
                } else {
                        if (ret == 0) {
                                /* Sync case - no callback */
                                cs->totalResponsesReceived++;
                        } else if (ret != -EINPROGRESS) {
                                printk("\n Result not as expected %d\n", ret);
                                return -1;
                        }
                        curr_offset = (curr_offset + 1) % MAX_INFLIGHT_REQUESTS;
                        cs->totalRequestsSent++;
                }
        }

        /* Wait for inflights */
        while (cs->totalResponsesReceived < totalReqsToSend) {
                ret = wait_event_interruptible_timeout(cs->tx_wait_queue,
                                                       (cs->
                                                        totalResponsesReceived
                                                        == totalReqsToSend),
                                                       RESPONSE_TIMEOUT_JIFFY);
                if (ret < 0) {
                        printk("\n*** Wait for response interrupted\n");
                        return -1;
                } else if (cs->totalResponsesReceived != totalReqsToSend) {
                        printk("\n*** Failed to receive response from CPM\n");
                        return -1;
                }
        }

        rdtscll(cs->endCycleCount);


        return 0;
}

int perf_aead_init_data(sa_context_t * cs, u32 dataLength, int dir, u32 keysize, u32 digestsize,
                        u32 auth_key_len, u32 blocksize, const char *alg_name)
{

        u32 dataIndex = 0;
        struct crypto_aead *tfm2;
        struct aead_givcrypt_request *pReq;
        u8 *pTemp = NULL;
        u8 *pReqMem = NULL;
        u8 *pIv = NULL;
        struct rtattr *rta = NULL;
        struct crypto_authenc_key_param *param;
        u32 i = 0, j = 0;
        u32 len = 0;
        struct scatterlist sg[1];
        struct scatterlist asg[1];

        /* Clear structure */
        memset(cs, 0, sizeof(sa_context_t));

        /* SA Key */
        cs->keylen = keysize + auth_key_len
                + RTA_SPACE(sizeof(*param));
        cs->pKey = kmalloc(cs->keylen, GFP_KERNEL);
        if (NULL == cs->pKey) {
                printk("Could not allocate key\n");
                return -1;
        }
        pTemp = cs->pKey;
        rta = (struct rtattr *)pTemp;
        rta->rta_type = CRYPTO_AUTHENC_KEYA_PARAM;
        rta->rta_len = RTA_LENGTH(sizeof(*param));
        param = RTA_DATA(rta);
        param->enckeylen = cpu_to_be32(keysize);
        pTemp += RTA_SPACE(sizeof(*param));
        /* random key for this context */
        get_random_bytes(pTemp, (keysize + auth_key_len));

        cs->pTempBuf = NULL;

        cs->dataLengthInBytes = dataLength;

        cs->buffLen = ICP_AEAD_PERF_HDR_LEN + blocksize
                + dataLength + digestsize;
        if (1 == dir) {
                /* decrypt direction - need to ensure digest
                   verify passes */
                /* For each context get a valid input */
                cs->pTempBuf = kmalloc(cs->buffLen, GFP_KERNEL);
                if (NULL == cs->pTempBuf) {
                        printk("Could not allocate pTempBuf\n");
                        kfree(cs->pKey);
                        return -1;
                }
                /* random input to encrypt */
                get_random_bytes(cs->pTempBuf, cs->buffLen);


                tfm2 = perf_aead_load_algo(alg_name);
                if (NULL == tfm2){
                    kfree(cs->pTempBuf);
                    kfree(cs->pKey);
                    return -1;
                }
                crypto_aead_setauthsize(tfm2, digestsize);
                crypto_aead_setkey(tfm2, cs->pKey, cs->keylen);

                sg_init_one(&sg[0],
                            (cs->pTempBuf + ICP_AEAD_PERF_HDR_LEN +
                             blocksize), dataLength + digestsize);
                sg_init_one(&asg[0], cs->pTempBuf, ICP_AEAD_PERF_HDR_LEN);
                pIv = cs->pTempBuf + ICP_AEAD_PERF_HDR_LEN;

                /* alloc givcrypt request */
                len =
                        sizeof(struct aead_givcrypt_request) +
                        crypto_aead_reqsize(tfm2);
                pReqMem = kmalloc(len, GFP_ATOMIC);
                if (NULL == pReqMem) {
                        printk("Could not allocate pReqMem\n");
                        crypto_free_aead(tfm2);
                        kfree(cs->pTempBuf);
                        kfree(cs->pKey);
                        return -1;
                }

                pReq = (struct aead_givcrypt_request *)pReqMem;
                aead_givcrypt_set_tfm(pReq, tfm2);

                aead_givcrypt_set_callback(pReq, CRYPTO_TFM_REQ_MAY_BACKLOG, 0,
                                           0);
                aead_givcrypt_set_crypt(pReq, sg, sg, dataLength, pIv);
                aead_givcrypt_set_assoc(pReq, asg, ICP_AEAD_PERF_HDR_LEN);
                aead_givcrypt_set_giv(pReq, pIv, 0);

                if (0 != crypto_aead_givencrypt(pReq)) {
                        printk("Result not as expected\n");
                        kfree(pReqMem);
                        crypto_free_aead(tfm2);
                        kfree(cs->pTempBuf);
                        kfree(cs->pKey);
                        return -1;
                }
                crypto_free_aead(tfm2);
                kfree(pReqMem);

        }

        for (i = 0; i < MAX_INFLIGHT_REQUESTS; i++) {
                /* Allocate memory for Src/Dst */
                cs->pBuff[i] = kmalloc(cs->buffLen, GFP_KERNEL);
                if (NULL == cs->pBuff[i]) {
                        printk("Error allocating src buffer %d\n", i);
                        for (j = 0; j < i; j++) {
                                kfree(cs->pBuff[j]);
                        }
                        if (NULL != cs->pTempBuf) {
                                kfree(cs->pTempBuf);
                        }
                        kfree(cs->pKey);
                        return -1;
                }

                if (1 == dir) {
                        /* Fill buffer with result from givencrypt */
                        memcpy(cs->pBuff[i], cs->pTempBuf, cs->buffLen);
                } else {
                        /* Fill buffer with stuff */
                        for (dataIndex = 0; dataIndex < cs->buffLen;
                             dataIndex++) {
                                *((cs->pBuff[i]) + dataIndex) = (u8) dataIndex;
                        }
                }

                /* describe hdr as a sgl */
                sg_init_one(&(cs->sglHdr[i]), cs->pBuff[i],
                            ICP_AEAD_PERF_HDR_LEN);
                /* describe data as a sgl */
                sg_init_one(&(cs->sglData[i]),
                            (cs->pBuff[i] + ICP_AEAD_PERF_HDR_LEN +
                             blocksize), dataLength + digestsize);
                /* Set where IV is */
                cs->pIv[i] = (cs->pBuff[i] + ICP_AEAD_PERF_HDR_LEN);
        }
        return 0;

}

int perf_aead_init_session(sa_context_t * cs, int dir, u32 digestsize, const char *alg_name)
{
        u32 i = 0;

        /* Crypto api - init SA */
        cs->tfm = crypto_alloc_aead(alg_name, 0, 0);
        if (NULL == cs->tfm) {
                printk("Error allocation tfm\n");
                return -1;
        }
        crypto_aead_setauthsize(cs->tfm, digestsize);
        crypto_aead_setkey(cs->tfm, cs->pKey, cs->keylen);

        /* Prep Crypto Requests */
        for (i = 0; i < MAX_INFLIGHT_REQUESTS; i++) {
                /* our implementation requires user to allocate crypto_aead_reqsize() */
                cs->pReq[i] = kmalloc((sizeof(struct aead_givcrypt_request)
                                       + crypto_aead_reqsize(cs->tfm)),
                                      GFP_KERNEL);
                if (NULL == cs->pReq[i]) {
                        printk("Error allocating request %d\n", i);
                        /* Calling function will free resources */
                        return -1;
                }
                if (0 == dir) {
                        aead_givcrypt_set_tfm(cs->pReq[i], cs->tfm);
                        aead_givcrypt_set_callback(cs->pReq[i], 0, perf_op_done,
                                                   cs);
                        aead_givcrypt_set_crypt(cs->pReq[i], &cs->sglData[i],
                                                &cs->sglData[i],
                                                cs->dataLengthInBytes,
                                                cs->pIv[i]);
                        aead_givcrypt_set_assoc(cs->pReq[i], &cs->sglHdr[i],
                                                ICP_AEAD_PERF_HDR_LEN);
                        aead_givcrypt_set_giv(cs->pReq[i], cs->pIv[i], 0);
                } else {
                        aead_request_set_tfm(&(cs->pReq[i]->areq), cs->tfm);
                        aead_request_set_callback(&(cs->pReq[i]->areq), 0,
                                                  perf_op_done, cs);
                        aead_request_set_crypt(&(cs->pReq[i]->areq),
                                               &cs->sglData[i], &cs->sglData[i],
                                               cs->dataLengthInBytes +
                                               digestsize, cs->pIv[i]);
                        aead_request_set_assoc(&(cs->pReq[i]->areq),
                                               &cs->sglHdr[i],
                                               ICP_AEAD_PERF_HDR_LEN);
                }

        }

        /* Zero Counters */
        cs->totalRequestsSent = 0;
        cs->totalResponsesReceived = 0;

        init_waitqueue_head(&cs->tx_wait_queue);

        return 0;
}

int perf_aead_cleanup(sa_context_t * cs)
{
        u32 i = 0;

        if (NULL != cs->tfm) {
                crypto_free_aead(cs->tfm);
        }

        if (NULL != cs->pKey) {
                kfree(cs->pKey);
        }

        if (NULL != cs->pTempBuf) {
                kfree(cs->pTempBuf);
        }
        for (i = 0; i < MAX_INFLIGHT_REQUESTS; i++) {
                if (NULL != cs->pBuff[i]) {
                        kfree(cs->pBuff[i]);
                }
                if (NULL != cs->pReq[i]) {
                        kfree(cs->pReq[i]);
                }

        }

        return 0;

}

struct crypto_aead * perf_aead_load_algo(const char * alg_name){
	struct crypto_aead * tfm2;
	if(0 == strcmp(alg_name, "authenc(hmac(sha1),cbc(aes))"))
	                {
	                        tfm2 =
	                                crypto_alloc_aead
	                                ("authenc(hmac(sha1-generic),cbc-aes-aesni)", 0, 0);
	                        if (IS_ERR(tfm2)) {
	                                /* "authenc(hmac(sha1-generic),cbc-aes-aesni)" driver not available try
	                                   "authenc(hmac(sha1-generic),cbc(aes-asm))" driver */
	                                tfm2 = crypto_alloc_aead
	                                        ("authenc(hmac(sha1-generic),cbc(aes-asm))", 0, 0);
	                                if (IS_ERR(tfm2)) {
	                                        printk
	                                                ("authenc(hmac(sha1-generic),cbc-aes-aesni) and \
	                        authenc(hmac(sha1-generic),cbc(aes-asm)) drivers not available\n");
	                                        return NULL;
	                                }
	                        }
	                }
	                else if(0 == strcmp(alg_name, "authenc(hmac(md5),cbc(aes))"))
	                {
	                        tfm2 =
	                                crypto_alloc_aead
	                                ("authenc(hmac(md5-generic),cbc-aes-aesni)", 0, 0);
	                        if (IS_ERR(tfm2)) {
	                                /* "authenc(hmac(md5-generic),cbc-aes-aesni)" driver not available try
	                                   "authenc(hmac(md5-generic),cbc(aes-asm))" driver */
	                                tfm2 = crypto_alloc_aead
	                                        ("authenc(hmac(md5-generic),cbc(aes-asm))", 0, 0);
	                                if (IS_ERR(tfm2)) {
	                                        printk
	                                                ("authenc(hmac(md5-generic),cbc-aes-aesni) and \
	                        authenc(hmac(md5-generic),cbc(aes-asm)) drivers not available\n");
	                                        return NULL;
	                                }
	                        }
	                }
	                else if(0 == strcmp(alg_name, "authenc(hmac(sha256),cbc(aes))"))
	                {
	                        tfm2 =
	                                crypto_alloc_aead
	                                ("authenc(hmac(sha256-generic),cbc-aes-aesni)", 0, 0);
	                        if (IS_ERR(tfm2)) {
	                                /* "authenc(hmac(sha256-generic),cbc-aes-aesni)" driver not available try
	                                   "authenc(hmac(sha256-generic),cbc(aes-asm))" driver */
	                                tfm2 = crypto_alloc_aead
	                                        ("authenc(hmac(sha256-generic),cbc(aes-asm))", 0, 0);
	                                if (IS_ERR(tfm2)) {
	                                        printk
	                                                ("authenc(hmac(sha256-generic),cbc-aes-aesni) and \
	                        authenc(hmac(sha1-generic),cbc(aes-asm)) drivers not available\n");
	                                        return NULL;
	                                }
	                        }
	                }
	                else if(0 == strcmp(alg_name, "authenc(hmac(sha512),cbc(aes))"))
	                {
	                        tfm2 =
	                                crypto_alloc_aead
	                                ("authenc(hmac(sha512-generic),cbc-aes-aesni)", 0, 0);
	                        if (IS_ERR(tfm2)) {
	                                /* "authenc(hmac(sha512-generic),cbc-aes-aesni)" driver not available try
	                                   "authenc(hmac(sha512-generic),cbc(aes-asm))" driver */
	                                tfm2 = crypto_alloc_aead
	                                        ("authenc(hmac(sha512-generic),cbc(aes-asm))", 0, 0);
	                                if (IS_ERR(tfm2)) {
	                                        printk
	                                                ("authenc(hmac(sha512-generic),cbc-aes-aesni) and \
	                        authenc(hmac(sha1-generic),cbc(aes-asm)) drivers not available\n");
	                                        return NULL;
	                                }
	                        }
	                }
	                else if(0 == strcmp(alg_name, "authenc(hmac(sha1),cbc(des3_ede))"))
	                {
	                        tfm2 =
	                                crypto_alloc_aead
	                                ("authenc(hmac(sha1-generic),cbc(des3_ede-generic))", 0, 0);
	                        if (IS_ERR(tfm2)) {
	                                /* "authenc(hmac(sha1-generic),cbc(des3_ede-generic))" driver not available
	                                   try "authenc(hmac(sha1-generic),cbc(des-generic))" driver */
	                                tfm2 = crypto_alloc_aead
	                                        ("authenc(hmac(sha1-generic),cbc(des-generic))", 0, 0);
	                                if (IS_ERR(tfm2)) {
	                                        printk
	                                                ("authenc(hmac(sha1),cbc(des3_ede-generic)) and \
	                       authenc(hmac(sha1-generic),cbc(des-generic)) drivers not available\n");
	                                        return NULL;
	                                }
	                        }
	                }
	                else if(0 == strcmp(alg_name, "authenc(hmac(md5),cbc(des3_ede))"))
	                {
	                        tfm2 =
	                                crypto_alloc_aead
	                                ("authenc(hmac(md5-generic),cbc(des3_ede-generic))", 0, 0);
	                        if (IS_ERR(tfm2)) {
	                                /* "authenc(hmac(md5-generic),cbc(des3_ede-generic))" driver not available
	                                   try "authenc(hmac(md5-generic),cbc(des-generic))" driver */
	                                tfm2 = crypto_alloc_aead
	                                        ("authenc(hmac(md5-generic),cbc(des-generic))", 0, 0);
	                                if (IS_ERR(tfm2)) {
	                                        printk
	                                                ("authenc(hmac(md5),cbc(des3_ede-generic)) and \
	                       authenc(hmac(md5-generic),cbc(des-generic)) drivers not available\n");
	                                        return NULL;
	                                }
	                        }
	                }
	                else if(0 == strcmp(alg_name, "authenc(hmac(sha256),cbc(des3_ede))"))
	                {
	                        tfm2 =
	                                crypto_alloc_aead
	                                ("authenc(hmac(sha256-generic),cbc(des3_ede-generic))", 0, 0);
	                        if (IS_ERR(tfm2)) {
	                                /* "authenc(hmac(sha256-generic),cbc(des3_ede-generic))" driver not available
	                                   try "authenc(hmac(sha256-generic),cbc(des-generic))" driver */
	                                tfm2 = crypto_alloc_aead
	                                        ("authenc(hmac(sha256-generic),cbc(des-generic))", 0, 0);
	                                if (IS_ERR(tfm2)) {
	                                        printk
	                                                ("authenc(hmac(sha256),cbc(des3_ede-generic)) and \
	                       authenc(hmac(sha256-generic),cbc(des-generic)) drivers not available\n");
	                                        return NULL;
	                                }
	                        }
	                }
	                else if(0 == strcmp(alg_name, "authenc(hmac(sha512),cbc(des3_ede))"))
	                {
	                        tfm2 =
	                                crypto_alloc_aead
	                                ("authenc(hmac(sha512-generic),cbc(des3_ede-generic))", 0, 0);
	                        if (IS_ERR(tfm2)) {
	                                /* "authenc(hmac(sha512-generic),cbc(des3_ede-generic))" driver not available
	                                   try "authenc(hmac(sha512-generic),cbc(des-generic))" driver */
	                                tfm2 = crypto_alloc_aead
	                                        ("authenc(hmac(sha512-generic),cbc(des-generic))", 0, 0);
	                                if (IS_ERR(tfm2)) {
	                                        printk
	                                                ("authenc(hmac(sha512),cbc(des3_ede-generic)) and \
	                       authenc(hmac(sha512-generic),cbc(des-generic)) drivers not available\n");
	                                        return NULL;
	                                }
	                        }
	                }
	                else
	                {
	                        printk("ERROR: no algorithm defined\n");
	                        return NULL;
	                }
	return tfm2;
}
