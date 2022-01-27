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
 * icp_aead_perf_module.c
 *
 * This module tests the performance at the lkcf api for AEAD algorithm 
 * authenc(hmac(sha1),cbc(aes))
 *
 * The performace test runs when the module is insmod-ed. The following
 * module parameters can be set:
 *
 * @param kp_numThreads   The number of threads. Each thread will be assigned to 
 *                        the available cores on the system in a round robin 
 *                        fashion.
 * @param kp_pktSize      The packet size. This is the size in bytes of the plaintext.
 *                        Similar to the IPSec use case this is the length to cipher
 *                        The size of the data to auth is 24 bytes (8 for header and 
 *                        16 for IV) greater than this.
 * @param kp_numReqs      The number of packets to send per thread. Note at least
 *                        1000 requests per thread should be sent to get reliable
 *                        throughput numbers.
 * @param kp_givenc       If set givencrypt function is tested.
 * @param kp_dec          If set decrypt function is tested.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/crypto.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <linux/kthread.h>
#include <linux/scatterlist.h>
#include <linux/cpufreq.h>
#include <crypto/sha.h>
#include <crypto/md5.h>

#include "icp_aead_perf.h"

#define DIGEST_SIZE_96bits 12        /* Digest size = 96 bits -> 12 bytes */

MODULE_DESCRIPTION("ICP AEAD perf test module");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL");

unsigned int kp_numThreads = 4;         /* Number of threads */
static unsigned int kp_pktSize = 1024;  /* Pkt size */
static unsigned int kp_numReqs = 5000000;       /* Number reqs to send per thread - needs to be
                                                   greater than 1000 for accurate results */
static unsigned int kp_givenc = 1;      /* If set, test givencrypt function */
static unsigned int kp_dec = 1;         /* If set, test decrypt function */
static unsigned int alg_type = 0;       /* If set, executes chosen algorithm to run performance test on */

module_param(kp_numThreads, uint, 0);
module_param(kp_pktSize, uint, 0);
module_param(kp_numReqs, uint, 0);
module_param(kp_givenc, uint, 0);
module_param(kp_dec, uint, 0);
module_param(alg_type, uint, 0);

atomic_t arrived;
atomic_t bGo;

static void icp_aead_output_res(icp_aead_test_data_t * pThreadData)
{
        u64 minStartCycles = 0, maxEndCycles = 0;
        u64 totalCycles = 0;
        u64 throughput = 0, time = 0;
        int i = 0;
        u64 cpu_freq = 0;
        icp_aead_test_data_t *pTemp = pThreadData;

        minStartCycles = pTemp->cs.startCycleCount;
        maxEndCycles = pTemp->cs.endCycleCount;
        pTemp++;

        for (i = 1; i < kp_numThreads; i++) {
                if (pTemp->cs.startCycleCount < minStartCycles) {
                        minStartCycles = pTemp->cs.startCycleCount;
                }
                if (pTemp->cs.endCycleCount > maxEndCycles) {
                        maxEndCycles = pTemp->cs.endCycleCount;
                }
                pTemp++;
        }

        totalCycles = maxEndCycles - minStartCycles;
        cpu_freq = (u64) cpufreq_quick_get(0);
        if (0 == cpu_freq) {
                cpu_freq = (u64) cpu_khz;
        }
        do_div(cpu_freq, 1000); /* in MHz */


        /* Number of bits processed */
        throughput =
                8 * pThreadData->cs.totalRequestsSent * kp_numThreads *
                pThreadData->cs.dataLengthInBytes;
        /* Time taken */
        time = totalCycles;

        if (cpu_freq != 0) {
                do_div(time, cpu_freq);
        } else {
                printk("Cpu frequency unknown, cannot calculate throughput\n");
        }

        /* divide by time taken */
        if (time != 0) {
                do_div(throughput, time);
        } else {
                printk("Time taken 0s, cannot calculate throughput\n");
        }

        printk("-----------------------------------------------\n");
        printk("Number threads:                            %d\n",
               kp_numThreads);
        printk("Number of requests per thread:             %llu\n",
               pThreadData->cs.totalRequestsSent);
        printk("Pkt Size:                                  %u\n",
               pThreadData->cs.dataLengthInBytes);
        if (pThreadData->cs.totalRequestsSent > 1000) {
                printk("Total number of Cycles:                    %llu\n",
                       totalCycles);
                if ((cpu_freq != 0) && (time != 0)) {
                        printk
                                ("CPU frequency:                         %llu MHz\n",
                                 cpu_freq);
                        printk
                                ("Throughput:                            %llu Mbps\n",
                                 throughput);
                }
        }
        printk("-----------------------------------------------\n");

}

static int icp_aead_perf_run(u32 keysize, u32 digestsize, u32 auth_key_len, u32 blocksize, const char *alg_name)
{
        icp_aead_test_data_t *pThreadData = NULL;
        icp_aead_test_data_t *pThreadDataTemp = NULL;
        struct task_struct **pThread = NULL;
        int i = 0;
        u32 pktSize = kp_pktSize;
        u32 numCpus = 0;
        struct crypto_aead *tmp_tfm;

        /* Param Checking */
        if (kp_numThreads < 1) {
                printk("kp_numThreads must be greater than or equal to 1\n");
                return -1;
        }

        /* Round pktSize up to multiple of block size */
        if (pktSize % blocksize != 0) {
                pktSize =
                        kp_pktSize + (blocksize -
                                      (kp_pktSize % blocksize));
                printk("Rounding pktSize up to multiple of block size\n");
                printk("New pktSize %d\n", pktSize);
        }

        numCpus = num_online_cpus();

        /* Alloc generic implementation of alg_name to workaround a bug in LKCF */
        tmp_tfm = perf_aead_load_algo(alg_name);
        if (tmp_tfm){
            crypto_free_aead(tmp_tfm);
        }
        /* Initialise barrier */
        atomic_set(&arrived, 0);
        atomic_set(&bGo, 0);

        pThread =
                kmalloc((kp_numThreads * sizeof(struct task_struct *)), GFP_KERNEL);
        if (NULL == pThread) {
                printk("Memory allocation failure (pThread)\n");
                return -1;
        }
        pThreadData =
                kmalloc((kp_numThreads * sizeof(icp_aead_test_data_t)), GFP_KERNEL);
        if (NULL == pThreadData) {
                printk("Memory allocation failure (pThreadData)\n");
                kfree(pThread);
                return -1;
        }

        if (kp_givenc) {
                printk
                        ("\nAEAD givencrypt performance: alg %s\n", alg_name);

                pThreadDataTemp = pThreadData;
                /* Create and start threads */
                for (i = 0; i < kp_numThreads; i++) {
                        pThreadDataTemp->numReq = kp_numReqs;
                        pThreadDataTemp->pktSize = pktSize;
                        pThreadDataTemp->keysize = keysize;
                        pThreadDataTemp->digestsize = digestsize;
                        pThreadDataTemp->auth_key_len = auth_key_len;
                        pThreadDataTemp->blocksize = blocksize;
                        pThreadDataTemp->alg_name = alg_name;
                        pThread[i] =
                                kthread_create(icp_aead_givencrypt,
                                               (void *)pThreadDataTemp, "GIVENC-%d",
                                               i);
                        if (IS_ERR(pThread[i])) {
                                printk("Failed to create thread GIVENC-%d\n",
                                       i);
                                kfree(pThreadData);
                                kfree(pThread);
                                return -1;
                        }
                        /* bind to a core */
                        kthread_bind(pThread[i], (i % numCpus));
                        wake_up_process(pThread[i]);

                        pThreadDataTemp++;
                }

                /* Wait to complete */
                for (i = 0; i < kp_numThreads; i++) {
                        while (!(pThread[i]->exit_state != 0)) {
                                set_current_state((long)TASK_INTERRUPTIBLE);
                                schedule_timeout(10 * HZ);
                        }
                }

                /* Output results */
                icp_aead_output_res(pThreadData);

                /* Reset barrier */
                atomic_set(&arrived, 0);
                atomic_set(&bGo, 0);
        }
        if (kp_dec) {
                printk
                        ("\nAEAD decrypt performance: alg %s\n", alg_name);

                pThreadDataTemp = pThreadData;
                /* Create and start threads */
                for (i = 0; i < kp_numThreads; i++) {
                        pThreadDataTemp->numReq = kp_numReqs;
                        pThreadDataTemp->pktSize = pktSize;
                        pThreadDataTemp->keysize = keysize;
                        pThreadDataTemp->digestsize = digestsize;
                        pThreadDataTemp->auth_key_len = auth_key_len;
                        pThreadDataTemp->blocksize = blocksize;
                        pThreadDataTemp->alg_name = alg_name;
                        pThread[i] =
                                kthread_create(icp_aead_decrypt,
                                               (void *)pThreadDataTemp, "DEC-%d",
                                               i);
                        if (IS_ERR(pThread[i])) {
                                printk("Failed to create thread GIVENC-%d\n",
                                       i);
                                kfree(pThreadData);
                                kfree(pThread);
                                return -1;
                        }
                        /* bind to a core */
                        kthread_bind(pThread[i], (i % numCpus));
                        wake_up_process(pThread[i]);

                        pThreadDataTemp++;
                }

                /* Wait to complete */
                for (i = 0; i < kp_numThreads; i++) {
                        while (!(pThread[i]->exit_state != 0)) {
                                set_current_state((long)TASK_INTERRUPTIBLE);
                                schedule_timeout(10 * HZ);
                        }
                }

                /* Output results */
                icp_aead_output_res(pThreadData);

                /* Reset barrier */
                atomic_set(&arrived, 0);
                atomic_set(&bGo, 0);

        }

        kfree(pThread);
        kfree(pThreadData);

        return 0;
}

static int _icp_aead_perf_init(void)
{

        printk("Loading AEAD Performance Test module ...\n");
        switch (alg_type) {
        case 1:
                icp_aead_perf_run(AES_KEYSIZE_128, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha1),cbc(aes))");
                break; 
        case 2:
                icp_aead_perf_run(AES_KEYSIZE_128, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(md5),cbc(aes))");
                break;
        case 3:
                icp_aead_perf_run(AES_KEYSIZE_128, SHA256_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha256),cbc(aes))");
                break;
        case 4:
                icp_aead_perf_run(AES_KEYSIZE_128, SHA512_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha512),cbc(aes))");
                break;
        case 5:
                icp_aead_perf_run(DES3_EDE_KEY_SIZE, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  DES3_EDE_BLOCK_SIZE, "authenc(hmac(sha1),cbc(des3_ede))");
                break;
        case 6:
                icp_aead_perf_run(DES3_EDE_KEY_SIZE, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  DES3_EDE_BLOCK_SIZE, "authenc(hmac(md5),cbc(des3_ede))");
                break;
        case 7:
                icp_aead_perf_run(DES3_EDE_KEY_SIZE, SHA256_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  DES3_EDE_BLOCK_SIZE, "authenc(hmac(sha256),cbc(des3_ede))");
                break;
        case 8:
                icp_aead_perf_run(DES3_EDE_KEY_SIZE, SHA512_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  DES3_EDE_BLOCK_SIZE, "authenc(hmac(sha512),cbc(des3_ede))");
                break;
        case 9:
                icp_aead_perf_run(AES_KEYSIZE_192, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha1),cbc(aes))");
                break;
        case 10:
                icp_aead_perf_run(AES_KEYSIZE_192, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(md5),cbc(aes))");
                break;
        case 11:
                icp_aead_perf_run(AES_KEYSIZE_192, SHA256_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha256),cbc(aes))");
                break;
        case 12:
                icp_aead_perf_run(AES_KEYSIZE_192, SHA512_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha512),cbc(aes))");
                break;
        case 13:
                icp_aead_perf_run(AES_KEYSIZE_256, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha1),cbc(aes))");
                break;
        case 14:
                icp_aead_perf_run(AES_KEYSIZE_256, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(md5),cbc(aes))");
                break;
        case 15:
                icp_aead_perf_run(AES_KEYSIZE_256, SHA256_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha256),cbc(aes))");
                break;
        case 16:
                icp_aead_perf_run(AES_KEYSIZE_256, SHA512_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha512),cbc(aes))");
                break;
        default:       
                icp_aead_perf_run(AES_KEYSIZE_128, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha1),cbc(aes))");
                icp_aead_perf_run(AES_KEYSIZE_128, SHA256_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha256),cbc(aes))");
                icp_aead_perf_run(AES_KEYSIZE_128, SHA512_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(sha512),cbc(aes))");
                icp_aead_perf_run(AES_KEYSIZE_128, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  AES_BLOCK_SIZE, "authenc(hmac(md5),cbc(aes))");
                icp_aead_perf_run(DES3_EDE_KEY_SIZE, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  DES3_EDE_BLOCK_SIZE, "authenc(hmac(sha1),cbc(des3_ede))");
                icp_aead_perf_run(DES3_EDE_KEY_SIZE, SHA256_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  DES3_EDE_BLOCK_SIZE, "authenc(hmac(sha256),cbc(des3_ede))");
                icp_aead_perf_run(DES3_EDE_KEY_SIZE, SHA512_DIGEST_SIZE, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  DES3_EDE_BLOCK_SIZE, "authenc(hmac(sha512),cbc(des3_ede))");
                icp_aead_perf_run(DES3_EDE_KEY_SIZE, DIGEST_SIZE_96bits, ICP_AEAD_PERF_AUTH_KEY_LEN,
                                  DES3_EDE_BLOCK_SIZE, "authenc(hmac(md5),cbc(des3_ede))");
                break;
        }

        return 0;
}

static void _icp_aead_perf_exit(void)
{
        /* Done */
}

module_init(_icp_aead_perf_init);
module_exit(_icp_aead_perf_exit);
