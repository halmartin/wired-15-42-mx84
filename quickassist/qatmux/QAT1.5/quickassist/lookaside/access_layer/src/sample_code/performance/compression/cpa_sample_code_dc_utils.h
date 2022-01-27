/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
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
 *   BSD LICENSE 
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without 
 *   modification, are permitted provided that the following conditions 
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided with the 
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *  version: QAT1.5.L.1.11.0-36
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file cpa_sample_code_dc_utils.h
 *
 * @defgroup compressionThreads
 *
 * @ingroup compressionThreads
 *
 * @description
 * Contains function prototypes and #defines used throughout code
 * and macros
 *
 ***************************************************************************/
#ifndef CPA_SAMPLE_CODE_DC_UTILS_H_
#define CPA_SAMPLE_CODE_DC_UTILS_H_

#include "cpa_sample_code_dc_perf.h"
#include "cpa.h"
#include "cpa_dc.h"
#include "cpa_dc_dp.h"
#include "cpa_sample_code_framework.h"
#ifdef USER_SPACE
#include "zlib.h"
#else
#include <linux/zlib.h>
#include <linux/crypto.h>
#include <linux/string.h>
#endif

/* Common macro definitions */
#define TEMP_NUM_BUFFS (5)
#define MIN_BUFFER_SIZE (15)
#define KILOBITS_IN_MEGABITS    (1000)

/* corpus Data structure */
extern corpus_data_t corpus_g;
extern CpaBoolean useZlib_g;
extern Cpa32U expansionFactor_g;
#ifdef ZERO_BYTE_LAST_REQUEST
extern CpaBoolean zeroByteLastRequest_g;
CpaStatus enableZeroByteRequest(void);
CpaStatus disableZeroByteRequest(void);
#endif
/* Used by ZLIB */
#define DEFLATE_DEF_LEVEL (Z_DEFAULT_COMPRESSION)
#define DEFLATE_DEF_WINBITS (15)
#define DEFLATE_DEF_MEMLEVEL (8)


#ifndef DO_CRYPTO
/**
 *****************************************************************************
 * @ingroup compressionThreads
 *      getThroughput
 *
 * @description
 *      get the throughput in Megabits per second
 *      =(numPackets*packetSize)*(cycles/cpu_frequency)
 *****************************************************************************/
Cpa32U getThroughput(Cpa64U numPackets, Cpa32U packetSize,
        perf_cycles_t cycles);
#endif
/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  populateCorpus
 *
 *  @description
 *      this API populates the specified corpus to the corpus data structure
 *  @threadSafe
 *      No
 *
 *  @param[out]  status
 *
 *  @param[in]  buffSize size of the flat buffer
 *  @param[in]  corpusType corpus type to load
 *
 ******************************************************************************/
CpaStatus populateCorpus(Cpa32U buffSize, corpus_type_t corpusType);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  startDcServices
 *
 *  @description
 *      this API starts the Data compression services if not already started
 *  @threadSafe
 *      No
 *
 *
 *  @param[in]  buffsize buffer size for intermittent buffers
 *  @param[in]  numBuffs number of intermittent Buffers required
 *
 ******************************************************************************/
CpaStatus startDcServices(Cpa32U buffSize, Cpa32U numBuffs);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  freeCorpus
 *
 *  @description
 *      this API frees the corpus global data structure.
 *  @threadSafe
 *      No
 *
 ******************************************************************************/
void freeCorpus(void);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  startDcServices
 *
 *  @description
 *      this API stops the Data compression services.
 *  @threadSafe
 *      No
 *  @param[in] dcSetup compression_test_params_t test setup structure
 *
 ******************************************************************************/
CpaStatus stopDcServices(compression_test_params_t *dcSetup);
/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  claculateRequireBuffers
 *
 *  @description
 *      this API calculates the number of flat buffer required for the corpus
 *      and populates the setup->numberOfBuffers array
 *  @threadSafe
 *      No
 *
 *  @param[out] status
 *
 *  @param[in]  pointer to compression_test_params_t
 *
 ******************************************************************************/
CpaStatus calculateRequireBuffers(compression_test_params_t * setup);

/**
 *****************************************************************************
 * @ingroup compressionThreads
 *      dcCreatePollingThreadsIfPollingIsEnabled
 *
 * @description
 *      This function checks whether each instance handle is set for polling
 *      and will allocate create and start the same number polling threads
 *      as they are polling instances.
 * @pre numInstances_g is set and all instances have been started.
 * @post numPolledInstances_g is set by the function to the number of polling
 *         instances available.
 *
 *****************************************************************************/
CpaStatus dcCreatePollingThreadsIfPollingIsEnabled(void);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcDpPollNumOperations
 *
 *  @description
 *      Poll for remaining operations, this function will timeout after
 *      SAMPLE_CODE_WAIT_THIRTY_SEC have elapsed.
 *
 *  @threadSafe
 *      No
 *
 *  @param[in] perfData         pointer to performance structure
 *  @param[in] instanceHandle   API CpaInstanceHandle
 *  @param[in] numOperations   Number of operations to poll for
 *
 *  @retval CPA_STATUS_SUCCESS  No operations to poll for or all remaining
 *                              operations have been polled.
 *  @retval CPA_STATUS_FAIL     Failure from polling operation or timeout.
 ******************************************************************************/
CpaStatus dcDpPollNumOperations(perf_data_t *pPerfData,
                CpaInstanceHandle instanceHandle, Cpa64U numOperations);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  wait for semaphore
 *
 *  @description
 *      this API waits for the semaphore for 30 seconds and checks
 *      if the responses are still pending, if the reponses are still pending
 *      wait for another 30 secs and returns error if the responses are not
 *      increased
 *
 *  @threadSafe
 *      Yes
 *
 *  @param[in] perfData  pointer to performance structure
 *
 ******************************************************************************/
CpaStatus waitForSemaphore(perf_data_t *perfData);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  sampleCodeDcGetNode
 *
 *  @description
 *      this API determines the CPU node  for the instance handle
 *
 *  @threadSafe
 *      No
 *
 *  @param[in]  instanceHandle DC instance Handle
 *  @param[out] node node ID of the instance
 *
 ******************************************************************************/
CpaStatus sampleCodeDcGetNode(CpaInstanceHandle instanceHandle, Cpa32U *node);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  sampleCodePoll
 *
 *  @description
 *      this API repeatedly calls the QA API poll function
 *  @threadSafe
 *      No
 *
 *  @param[in]  instanceHandle DC instance Handle
 *
 ******************************************************************************/
void sampleCodeDcPoll(CpaInstanceHandle instanceHandle_in);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcPrintstats
 *
 *  @description
 *      this API prints performance data like number of CPU cycles
 *      consumed for an Operation
 *  @threadSafe
 *      No
 *
 *  @param[in]  data  pointer to test data structure
 *
 ******************************************************************************/
CpaStatus dcPrintStats(thread_creation_data_t* data);
/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcPrintTestData
 *
 *  @description
 *      this API prints the setup data used to execute the compression test
 *  @threadSafe
 *      No
 *
 *  @param[in]  setup  pointer to compression test data structure
 *
 ******************************************************************************/
void dcPrintTestData(compression_test_params_t* setup);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcDpSetBytesProducedAndConsumed
 *
 *  @description
 *      This function aggregates the total bytes consumed and produced as
 *      reported by the driver in the opData structures (a separate opData
 *      structure is used for each buffer submitted) and stores them in the
 *      perfData totalBytesConsumed and totalBytesProduced fields.
 *  @threadSafe
 *      No
 *
 *  @param[in]      opData  2D array of pointers to a CpaDcDpOpData structure
 *  @param[in,out]  perfData  pointer to performance structure
 *  @param[in]      pointer to compression_test_params_t structure.
 *
 *
 ******************************************************************************/
void dcDpSetBytesProducedAndConsumed(CpaDcDpOpData ***opData,
        perf_data_t *perfData, compression_test_params_t* setup);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcSetBytesProducedAndConsumed
 *
 *  @description
 *      This function aggregates the total bytes consumed and produced as
 *      reported by the driver in the cmpResult structures (a separate cmpResult
 *      structure is used for each buffer submitted) and stores them in the
 *      perfData totalBytesConsumed and totalBytesProduced fields.
 *  @threadSafe
 *      No
 *
 *  @param[in]  cmpResult  2D array of pointers to a CpaDcRqResults structure
 *  @param[in]  perfData  pointer to performance structure
 *  @param[in] pointer to compression_test_params_t structure.
 *
 *
 ******************************************************************************/
void dcSetBytesProducedAndConsumed(CpaDcRqResults ***cmpResult,
        perf_data_t *perfData, compression_test_params_t* setup);
/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcCalculateAndPrintCompressionRatio
 *
 *  @description
 *      this API calculates and prints the compression ratio
 *  @threadSafe
 *      No
 *
 *  @param[in]  bytesProduced  Total number of bytes produced after compression
 *  @param[in]  bytesConsumed  Total number of bytes consumed for compression
 *
 ******************************************************************************/
CpaStatus dcCalculateAndPrintCompressionRatio(Cpa32U bytesProduced,
        Cpa32U bytesConsumed);

/**
 *****************************************************************************
 * @ingroup compressionThreads
 *      getDcThroughput
 *
 * @description
 *      get the throughput in Megabits per second
 *
 * @param[in]   totalBytes  Total number of bytes consumed by all threads
 * @param[in]   cycles      Total number of clock cycles
 * @param[in]   numOfLoops  Number of loops specified for compression operation
 *****************************************************************************/
Cpa32U getDcThroughput(Cpa32U totalBytes, perf_cycles_t cycles,
                Cpa32U numOfLoops);

/**
 *****************************************************************************
 * @ingroup compressionThreads
 *      dynamicHuffmanEnabled
 *
 * @description
 *      Determines if a given instance has Dynamic Huffman enabled.
 *
 * @param[in]       dcInstanceHandle  Pointer to CpaInstanceHandle. If
 *                          dcInstanceHandle is NULL a local instance, i.e. the
 *                          first instance from cpaDcGetNumInstances() will be
 *                          used to determine if dynamic Huffman is enabled.
 * @param[in,out]   isEnabled     Pointer to CpaBoolean. When set to CPA_TRUE
 *                          the instance handle has dynamic Huffman enabled.
 *                          Will be set to CPA_FALSE for all error conditions
 *                          or where CpaDcInstanceCapabilities.dynamicHuffman
 *                          is CPA_FALSE.
 * @retval CPA_STATUS_SUCCESS  Instance query for dynamic compression succeeded
 * @retval CPA_STATUS_FAIL     Instance query for dynamic compression failed
 *****************************************************************************/
CpaStatus dynamicHuffmanEnabled(CpaInstanceHandle *dcInstanceHandle,
        CpaBoolean *isEnabled);

/*This function tells the compression sample code to use zLib software to
 * compress the data prior to calling the decompression*/
CpaStatus useZlib(void);

/*This function tells the compression sample code to use zLib software to
 * compress the data prior to calling the decompression*/
CpaStatus useAccelCompression(void);


CpaStatus dcPollNumOperations(perf_data_t *pPerfData,
                CpaInstanceHandle instanceHandle, Cpa64U numOperations);


#endif /* CPA_SAMPLE_CODE_DC_UTILS_H_ */
