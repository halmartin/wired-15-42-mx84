/******************************************************************************
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
 *****************************************************************************/

/*
 * This file exports performance code kernel symbols*/
#include "cpa_sample_code_utils_common.h"
#include "cpa_sample_code_framework.h"
#include "cpa_sample_code_crypto_utils.h"

#include "qae_mem.h"

#include "cpa_sample_code_dc_perf.h"
#include "cpa_sample_code_sym_perf_dp.h"
#include "cpa_sample_code_dc_dp.h"

extern void createStartandWaitForCompletion(void);
extern void printSymTestType(symmetric_test_params_t *setup);
extern CpaStatus getCurveData(ecdsa_test_params_t* setup);
extern int numCreatedThreads_g;
extern Cpa16U numInstances_g;
extern Cpa32U *cyInstMap_g;
extern Cpa32U *dcInstMap_g;
extern Cpa32U instMap_g;
extern Cpa16U numInst_g;
extern Cpa16U *inst_g;

/*Feature specific functions exported*/
#ifdef DO_CRYPTO
EXPORT_SYMBOL(setupEcdsaTest);
EXPORT_SYMBOL(setupSymmetricTest);
EXPORT_SYMBOL(setupCipherTest);
EXPORT_SYMBOL(setupHashTest);
EXPORT_SYMBOL(setupAlgChainTest);
EXPORT_SYMBOL(setupAlgChainTestNestedMode);
EXPORT_SYMBOL(setupAlgChainTestHPAsync);
EXPORT_SYMBOL(setupRsaTest);
EXPORT_SYMBOL(setupDsaTest);
EXPORT_SYMBOL(setupDhTest);
EXPORT_SYMBOL(setupCipherDpTest);
EXPORT_SYMBOL(setupAlgChainDpTest);
EXPORT_SYMBOL(setupHashDpTest);
EXPORT_SYMBOL(setupAlgChainTestDpNestedMode);
#endif
#ifdef INCLUDE_COMPRESSION
EXPORT_SYMBOL(setupDcTest);
EXPORT_SYMBOL(setupDcDpTest);
#endif


EXPORT_SYMBOL(createStartandWaitForCompletion);

/*Framework Functions Exported*/
EXPORT_SYMBOL(setCoreLimit);
EXPORT_SYMBOL(createPerfomanceThreads);
EXPORT_SYMBOL(startThreads);
EXPORT_SYMBOL(waitForThreadCompletion);
EXPORT_SYMBOL(sampleCodeSleep);
EXPORT_SYMBOL(sampleCodeSleepMilliSec);
EXPORT_SYMBOL(testSetupData_g);
EXPORT_SYMBOL(singleThreadData_g);
EXPORT_SYMBOL(sampleCodeTimestamp);
EXPORT_SYMBOL(sampleCodeSemaphoreInit);
EXPORT_SYMBOL(sampleCodeSemaphorePost);
EXPORT_SYMBOL(sampleCodeSemaphoreWait);
EXPORT_SYMBOL(generateRandomData);
EXPORT_SYMBOL(qaeMemFree);
EXPORT_SYMBOL(qaeMemFreeNUMA);
EXPORT_SYMBOL(qaeVirtToPhysNUMA);
EXPORT_SYMBOL(sampleCodeBarrier);
EXPORT_SYMBOL(sampleCodeSemaphoreDestroy);
EXPORT_SYMBOL(thread_setup_g);
EXPORT_SYMBOL(thread_name_g);
EXPORT_SYMBOL(sampleCodeGetNumberOfCpus);
EXPORT_SYMBOL(testTypeCount_g);
EXPORT_SYMBOL(sampleCodeGetCpuFreq);
EXPORT_SYMBOL(qaeMemAllocNUMA);
EXPORT_SYMBOL(getLongestCycleCount);
EXPORT_SYMBOL(startBarrier);
EXPORT_SYMBOL(killCreatedThreads);
EXPORT_SYMBOL(qaeMemAlloc);
EXPORT_SYMBOL(clearPerfStats);
EXPORT_SYMBOL(numCreatedThreads_g);
#ifdef DO_CRYPTO
EXPORT_SYMBOL(symPerformCallback);
EXPORT_SYMBOL(sampleCreateBuffers);
EXPORT_SYMBOL(bufferDataMemAlloc);
EXPORT_SYMBOL(processCallback);
EXPORT_SYMBOL(sampleCodeCyGetNode);
EXPORT_SYMBOL(getCurveData);
EXPORT_SYMBOL(getThroughput);
EXPORT_SYMBOL(stopCyServices);
EXPORT_SYMBOL(waitForResponses);
EXPORT_SYMBOL(printSymTestType);
EXPORT_SYMBOL(makeParam1SmallerThanParam2);
EXPORT_SYMBOL(printAsymStatsAndStopServices);
EXPORT_SYMBOL(numInstances_g);
EXPORT_SYMBOL(allocArrayOfPointers);
EXPORT_SYMBOL(sampleFreeBuffers);
EXPORT_SYMBOL(printSymmetricPerfDataAndStopCyService);
EXPORT_SYMBOL(startCyServices);
#endif
EXPORT_SYMBOL(cyInstMap_g);
EXPORT_SYMBOL(dcInstMap_g);
EXPORT_SYMBOL(instMap_g);
EXPORT_SYMBOL(numInst_g);
EXPORT_SYMBOL(inst_g);
EXPORT_SYMBOL(poll_inline_g);

