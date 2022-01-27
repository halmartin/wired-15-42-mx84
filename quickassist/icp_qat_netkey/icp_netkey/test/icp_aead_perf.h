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

#define MAX_INFLIGHT_REQUESTS 200
#define ICP_AEAD_PERF_AUTH_KEY_LEN 20   /* auth key length in bytes */

typedef struct _sa_context_s {
        /* SA info */
        struct crypto_aead *tfm;
        u8 *pKey;
        u8 *pTempBuf;           /* Used for decrypt testing to hold the known answer vector */
        u32 keylen;

        /* Per request */
        struct aead_givcrypt_request *pReq[MAX_INFLIGHT_REQUESTS];
        struct scatterlist sglData[MAX_INFLIGHT_REQUESTS];
        struct scatterlist sglHdr[MAX_INFLIGHT_REQUESTS];
        u8 *pIv[MAX_INFLIGHT_REQUESTS];
        u8 *pBuff[MAX_INFLIGHT_REQUESTS];       /* Data buffer */
        u32 buffLen;            /* Length of data buffer = dataLengthInBytes + hdr +iv + digest */
        u32 dataLengthInBytes;

        /* wait queue */
        wait_queue_head_t tx_wait_queue;

        /* Counters */
        u64 totalRequestsSent;
        u64 totalResponsesReceived;
        u64 totalRetries;

        /* Results */
        u64 startCycleCount;
        u64 endCycleCount;
        u64 dummyCycles;

} sa_context_t;

typedef struct icp_aead_test_data_s {
        u64 numReq;
        u32 pktSize;
        sa_context_t cs;        /* per thread session info */
        u32 keysize;
        u32 digestsize;
        u32 auth_key_len;
        u32 blocksize;
        const char *alg_name;
} icp_aead_test_data_t;

/* Tests givencrypt function */
int icp_aead_givencrypt(void *pData);

/* Tests decrypt function */
int icp_aead_decrypt(void *pData);

/* Initialise data and session */
int perf_aead_init(sa_context_t * cs, u32 dataLength,
                   u64 totalReqs, int dir, u32 keysize, u32 digestsize, 
                   u32 auth_key_len, u32 blocksize, const char *alg_name);
struct crypto_aead * perf_aead_load_algo(const char * alg_name);
