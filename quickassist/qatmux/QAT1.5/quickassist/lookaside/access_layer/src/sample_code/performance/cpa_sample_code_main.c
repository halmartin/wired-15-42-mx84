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
 **************************************************************************/

/**
 ***************************************************************************
 * @file cpa_sample_code_main.c
 *
 * This file provides the main function, which calls out what to measure the
 * performance of
 *
 **************************************************************************/

// Local define to allow CRYPTO to be included or excluded from compilation
// Redundant here (except for test purposes)as set by performance sample code Makefile.
//#define DO_CRYPTO

#include "cpa_sample_code_framework.h"
#include "cpa_sample_code_crypto_utils.h"
#ifdef INCLUDE_COMPRESSION
#include "cpa_dc.h"
#include "cpa_sample_code_dc_perf.h"
#include "cpa_sample_code_dc_utils.h"
#include "cpa_sample_code_dc_dp.h"
#endif
#include "cpa_sample_code_sym_perf_dp.h"
#include "icp_sal_versions.h"

#ifndef INCLUDE_COMPRESSION
/*define this just so that sample code will build without compression code*/
#define MIN_DC_LOOPS            (1)
#ifdef __x86_64__
#define DEFAULT_DC_LOOPS        (100)
#else
#define DEFAULT_DC_LOOPS        (200)
#endif
#define SINGLE_REQUEST          (1)
#else
corpus_type_t sampleCorpus;
#endif

#ifdef USER_SPACE
#include "icp_sal_user.h"


extern CpaStatus qaeMemInit(void);
extern void qaeMemDestroy(void);


int runTests;
int signOfLife;
int cyNumBuffers;
int cyAsymLoops;
int cySymLoops;
int dcLoops;
int wirelessFirmware;
int configFileVersion;
int runStateful;
int verboseOutput = 1;

option_t optArray[MAX_NUMOPT] ={
        {"signOfLife", DEFAULT_SIGN_OF_LIFE},
        {"runTests", RUN_ALL_TESTS},
        {"cyNumBuffers", DEFAULT_CY_BUFFERS},
        {"cyAsymLoops", DEFAULT_ASYM_LOOPS},
        {"cySymLoops", DEFAULT_SYM_LOOPS},
        {"dcLoops", DEFAULT_DC_LOOPS},
        {"wirelessFirmware", DEFAULT_WIRELESS_FIRMWARE},
        {"configFileVer",USE_V2_CONFIG_FILE},
        {"runStateful",0},
        {"verboseOutput",1}};


#define SIGN_OF_LIFE_OPT_ARRAY_POS      (0)
#define RUN_TEST_OPT_ARRAY_POS          (1)
#define NUM_BUFFERS_OPT_ARRAY_POS       (2)
#define ASYM_LOOPS_OPT_ARRAY_POS        (3)
#define SYM_LOOPS_OPT_ARRAY_POS         (4)
#define DC_LOOPS_OPT_ARRAY_POS          (5)
#define WIRELESS_FIRMWARE_OPT_ARRAY_POS (6)
#define CONFIG_FILE_OPT_ARRAY_POS       (7)
#define RUN_STATEFUL_ARRAY_POS          (8)

#else

extern int cyNumBuffers;
extern int cySymLoops;
extern int cyAsymLoops;
extern int dcLoops;
extern int runTests;
extern int wirelessFirmware;
extern int signOfLife;
extern int runStateful;
extern int verboseOutput;
#endif



/*this function contains the top level tests, it is called main to support
 * user space executable, otherwise it is called in kernel space by the kernel
 * space init upon loading of the kernel module
 *
 * A test is made of of the following:
 *
 * 1. User calls their setup Test function, to set the feature specific
 * parameters that will be used when threads are created
 *
 * 2. user calls createPerfomanceThreads to create the test as setup in step2
 * across IA cores and acceleration units
 *
 * 3. User repeats steps 1-2 to setup more tests, ie to test co-existence of
 * crypto and datacompression, we call step 1-2 to setup crypto threads, then
 * step 1-2 again to setup data compression threads.
 *
 * 4. User calls startThreads so that the OS can start all the created threads
 *
 * 5. User waits for all threads to complete, when complete the statsPrintOut
 * function is called for each type of thread (the user is expect to provide
 * a statsPrintFunc in implementation of there setup Function used in step 1)
 *
 * 6. Repeat 1-5 for each test
 * */


CpaInstanceHandle *inst_g = NULL;
Cpa32U *cyInstMap_g = NULL;
Cpa32U *dcInstMap_g = NULL;
Cpa32U instMap_g;
Cpa16U numInst_g = 0;
extern Cpa32U packageIdCount_g;
extern CpaBoolean devicesCounted_g;

/*get the core affinity of a given instance handle*/
CpaStatus getCoreAffinity(CpaInstanceHandle instance,
        Cpa32U*  coreAffinity,
        Cpa32U instType)
{
    CpaInstanceInfo2 info = {0};
    Cpa32U i = 0;
    CpaStatus status = CPA_STATUS_FAIL;

	switch (instType)
	{
#ifdef DO_CRYPTO
		case CRYPTO:
		{
			status = cpaCyInstanceGetInfo2(instance, &info);
			if(CPA_STATUS_SUCCESS != status)
			{
				PRINT_ERR("could not get instance info\n");
				return status;
			}
		}
		break;
#endif /* DO_CRYPTO */

#ifdef INCLUDE_COMPRESSION
		case COMPRESSION:
		{
			status = cpaDcInstanceGetInfo2(instance, &info);
			if(CPA_STATUS_SUCCESS != status)
			{
				PRINT_ERR("could not get instance info\n");
				return status;
			}
		}
		break;
#endif /*INCLUDE_COMPRESSION*/

		default:
		{
			PRINT_ERR("Unknown Instance type\n");
			return CPA_STATUS_FAIL;
		}
		break;
	}/* End Switch */


    /*loop of the instanceInfo coreAffinity bitmask to find the core affinity*/
    for(i = 0; i< CPA_MAX_CORES; i++)
    {
        if(CPA_BITMAP_BIT_TEST(info.coreAffinity,i))
        {
            *coreAffinity = i;
            return CPA_STATUS_SUCCESS;
        }
    }
    PRINT_ERR("Could not find core affinity\n");
    return CPA_STATUS_FAIL;
}


void freeInstanceMapping(void)
{
    if(NULL != inst_g)
    {
        qaeMemFree((void**)&inst_g);
    }
    if(NULL != cyInstMap_g)
    {
        qaeMemFree((void**)&cyInstMap_g);
    }
    if(NULL != dcInstMap_g)
    {
        qaeMemFree((void**)&dcInstMap_g);
    }
    numInst_g = 0;
}

/*get the instance to core affinity mapping of the crypto instances and
 * populate the InstMap structure*/


CpaStatus getCryptoInstanceMapping(void)
{

	CpaStatus status = CPA_STATUS_FAIL;
#ifdef DO_CRYPTO /* Called from within this file */
    Cpa32U i = 0;
    Cpa32U coreAffinity = 0;
    CpaInstanceInfo2 info = {0};

    /*get the number of crypto instances*/
    status = cpaCyGetNumInstances(&numInst_g);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCyGetNumInstances failed with status: %d\n", status);
        return status;
    }
    if(numInst_g > 0)
    {
        /*allocate memory to store the instance handles*/
        inst_g = qaeMemAlloc(sizeof(CpaInstanceHandle)*numInst_g);
        if(inst_g == NULL)
        {
            PRINT_ERR("Failed to allocate memory for instances\n");
            freeInstanceMapping();
            return CPA_STATUS_FAIL;
        }
        /*get the instances handles and place in allocated memory*/
        status = cpaCyGetInstances(numInst_g, inst_g);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("cpaCyGetInstances failed with status: %d\n", status);
            freeInstanceMapping();
            return status;
        }
        /*allocate memory for the instance core mapping*/
        cyInstMap_g = qaeMemAlloc(sizeof(Cpa32U)*numInst_g);
        if(cyInstMap_g == NULL)
        {
            PRINT_ERR("Failed to allocate memory for instance mapping\n");
            freeInstanceMapping();
            return CPA_STATUS_FAIL;
        }

        for(i = 0; i<numInst_g; i++)
        {
            status = cpaCyInstanceGetInfo2(inst_g[i], &info);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("could not get instance info\n");
                return status;
            }
            if(CPA_STATUS_SUCCESS == getCoreAffinity(inst_g[i],
                    &coreAffinity,
                    CRYPTO))
            {
                if((packageIdCount_g  < info.physInstId.packageId) &&
                        (info.operState==CPA_OPER_STATE_UP)&&
                        (CPA_FALSE == devicesCounted_g))
                {
                    packageIdCount_g =  info.physInstId.packageId;
                }
                if(verboseOutput)
                {
                    PRINT("Inst %u, Affin: %u, Dev: %u, Accel %u",
                            i, coreAffinity,
                            info.physInstId.packageId,
                            info.physInstId.acceleratorId);
                    PRINT(", EE %u, BDF %02X:%02X:%02X\n",
                            info.physInstId.executionEngineId,
                            (Cpa8U)((info.physInstId.busAddress) >> 8),
                            (Cpa8U)((info.physInstId.busAddress) & 0xFF) >> 3,
                            (Cpa8U)((info.physInstId.busAddress) & 3));
                }
                cyInstMap_g[i] = coreAffinity;
            }
            else
            {
                freeInstanceMapping();
                return CPA_STATUS_FAIL;
            }
        }
        devicesCounted_g = CPA_TRUE;
    }
    else
    {
        PRINT("There are no crypto instances\n");
        return CPA_STATUS_FAIL;
    }
#endif /* DO_CRYPTO*/
	return status;
}
EXPORT_SYMBOL(getCryptoInstanceMapping);


#ifdef INCLUDE_COMPRESSION
/*get the instance to core affinity mapping of the compression instances and
 * populate the InstMap structure*/
CpaStatus getCompressionInstanceMapping(void)
{
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32U i = 0;
    Cpa32U coreAffinity = 0;
    CpaInstanceInfo2 info = {0};

    /*get the number of compression instances*/
    status = cpaDcGetNumInstances(&numInst_g);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaDcGetNumInstances failed with status: %d\n", status);
        return status;
    }
    if(numInst_g > 0)
    {
        /*allocate memory to store the instance handles*/
        inst_g = qaeMemAlloc(sizeof(CpaInstanceHandle)*numInst_g);
        if(inst_g == NULL)
        {
            PRINT_ERR("Failed to allocate memory for instances\n");
            freeInstanceMapping();
            return CPA_STATUS_FAIL;
        }
        /*get the instances handles and place in allocated memory*/
        status = cpaDcGetInstances(numInst_g, inst_g);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("cpaDcGetInstances failed with status: %d\n", status);
            freeInstanceMapping();
            return status;
        }
        /*allocate memory for the instance core mapping*/
        dcInstMap_g = qaeMemAlloc(sizeof(Cpa32U)*numInst_g);
        if(dcInstMap_g == NULL)
        {
            PRINT_ERR("Failed to allocate memory for instance mapping\n");
            freeInstanceMapping();
            return CPA_STATUS_FAIL;
        }

        for(i = 0; i<numInst_g; i++)
        {
            status = cpaDcInstanceGetInfo2(inst_g[i], &info);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("could not get instance info\n");
                return status;
            }
            if(CPA_STATUS_SUCCESS == getCoreAffinity(inst_g[i],
                    &coreAffinity,
                    COMPRESSION))
            {
                if((packageIdCount_g < info.physInstId.packageId) &&
                        (info.operState==CPA_OPER_STATE_UP) &&
                        CPA_FALSE == devicesCounted_g)
                {
                    packageIdCount_g = info.physInstId.packageId;
                }
                if(verboseOutput)
                {
                    PRINT("Inst %u, Affin: %u, Dev: %u, Accel %u",
                            i, coreAffinity,
                            info.physInstId.packageId,
                            info.physInstId.acceleratorId);
                    PRINT(", EE %u, BDF %02X:%02X:%02X\n",
                            info.physInstId.executionEngineId,
                            (Cpa8U)((info.physInstId.busAddress) >> 8),
                            (Cpa8U)((info.physInstId.busAddress) & 0xFF) >> 3,
                            (Cpa8U)((info.physInstId.busAddress) & 3));
                }
                dcInstMap_g[i] = coreAffinity;
            }
            else
            {
                freeInstanceMapping();
                return CPA_STATUS_FAIL;
            }
        }
        devicesCounted_g = CPA_TRUE;
    }
    else
    {
        PRINT("There are no compression instances\n");
        return CPA_STATUS_FAIL;
    }
    return status;
}
EXPORT_SYMBOL(getCompressionInstanceMapping);
#endif


/*this function can only be called after the last setup function has been
 * called, it creates the threads for the last setup function called, then
 * starts all threads and waits for all threads to complete. */
CpaStatus createStartandWaitForCompletion(Cpa32U instType)
{
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32U *instMap = NULL;

    switch (instType)
	{
#ifdef DO_CRYPTO
	case CRYPTO:
    {
        if(CPA_STATUS_SUCCESS != getCryptoInstanceMapping())
        {
            PRINT_ERR("Could not get Crypto Instance mapping\n");
            return CPA_STATUS_FAIL;
        }
        instMap = cyInstMap_g;
    }
	break;
#endif /* DO_CRYPTO */

#ifdef INCLUDE_COMPRESSION
    case COMPRESSION:
    {
        if(CPA_STATUS_SUCCESS != getCompressionInstanceMapping())
        {
            PRINT_ERR("Could not get Crypto Instance mapping\n");
            return CPA_STATUS_FAIL;
        }
        instMap = dcInstMap_g;
    }
	break;
#endif
	default:
	{
		PRINT_ERR("Invalid instance type\n");
		return CPA_STATUS_FAIL;
	}
	break;
	}/* End Switch */


    status = createPerfomanceThreads(
            numInst_g,
            instMap,
            USE_ALL_QA_LOGICAL_INSTANCES,
            DEFAULT_LOGICAL_INST_INSTANCE_OFFSET);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("Could not create threads, status: %d\n", status);
        return status;
    }

    status = startThreads();
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("Could not start threads, status: %d\n", status);
        return status;
    }
    /*wait for threads_g to finished and print out stats*/
    status = waitForThreadCompletion();
    freeInstanceMapping();
    return status;
}

CpaStatus printDriverVersion(Cpa32U deviceNum)
{
#ifdef WITH_UPSTREAM
    icp_sal_dev_version_info_t qaVersionInfo =
    {
            .devId = 0,
            .softwareVersion = {0}
    };
#else
    icp_sal_dev_version_info_t qaVersionInfo =
    {
            .devId = 0,
            .firmwareVersion = {0},
            .mmpVersion = {0},
            .softwareVersion = {0},
            .hardwareVersion = {0}
    };
#endif
    if(CPA_STATUS_SUCCESS != icp_sal_getDevVersionInfo(deviceNum, &qaVersionInfo))
    {
        PRINT_ERR("Could not get QA version information\n");
    }
    else
    {
#ifdef WITH_UPSTREAM
        PRINT("*** QA version information ***\n");
        PRINT("device ID\t\t= %d\n", qaVersionInfo.devId);
        PRINT("software \t\t= %s\n", qaVersionInfo.softwareVersion);
        PRINT("*** END QA version information ***\n");
#else
        PRINT("*** QA version information ***\n");
        PRINT("device ID\t\t= %d\n", qaVersionInfo.devId);
        PRINT("firmware \t\t= %s\n", qaVersionInfo.firmwareVersion);
        PRINT("mmp      \t\t= %s\n", qaVersionInfo.mmpVersion);
        PRINT("software \t\t= %s\n", qaVersionInfo.softwareVersion);
        PRINT("hardware \t\t= %s\n", qaVersionInfo.hardwareVersion);
        PRINT("*** END QA version information ***\n");
#endif
    }
    return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(printDriverVersion);

/***************************************************************************
 * what code to run
 **************************************************************************/

#define SYMMETRIC_CODE          (1)
#define RSA_CODE                (2)
#define DSA_CODE                (4)
#define ECDSA_CODE              (8)
#define DH_CODE                 (16)
#define COMPRESSION_CODE        (32)
#define FIRST_INSTANCE          (1)

/***************************************************************************
 * number of simultaneous threads to run for stateful compression
 **************************************************************************/
#define NUMBER_SIMILTANEOUS_THREADS (16)
#define NUMBER_OF_CORES_TO_USE      (8)

#define KASUMI_40_BYTE_BUFFER   (40)

int main(int argc,char *argv[])
{

    /*These are the packet sizes to measure performance off PACKET_IMIX is an
     * average of a mix of packet sizes - see PACKET_IMIX defintion for this mix
     * */
    Cpa32U lv_count = 0;
    Cpa16U i = 0;
    Cpa16S prevDevId = -1;
#ifdef INCLUDE_COMPRESSION
    Cpa16U numDcInst = 0;
    Cpa32U statefulMultiThreadCoreMap[NUMBER_SIMILTANEOUS_THREADS];
    Cpa32U dcBufferSize = 0;
    CpaBoolean dynamicEnabled = CPA_FALSE;
#endif

    CpaStatus status = CPA_STATUS_FAIL;
    CpaStatus retStatus = CPA_STATUS_SUCCESS;
#ifdef WITH_UPSTREAM
    icp_sal_dev_version_info_t qaVersionInfo =
    {
            .devId = 0,
            .softwareVersion = {0}
    };
#else
    icp_sal_dev_version_info_t qaVersionInfo =
    {
            .devId = 0,
            .firmwareVersion = {0},
            .mmpVersion = {0},
            .softwareVersion = {0},
            .hardwareVersion = {0}
    };
#endif
    CpaInstanceInfo2 info = {0};


#ifdef USER_SPACE
    char *processName = NULL;
#ifdef DO_CRYPTO
	Cpa32U cySymDpNumBuffers = CY_SYM_DP_NUM_BUFFERS;
    Cpa32U cySymDpNumLoops = CY_SYM_DP_NUM_LOOPS;
#endif /* DO_CRYPTO*/


    if(0 != parseArg(argc, argv,  optArray, MAX_NUMOPT))
    {
        return 0;
    }
    signOfLife = optArray[SIGN_OF_LIFE_OPT_ARRAY_POS].optValue;
    runTests = optArray[RUN_TEST_OPT_ARRAY_POS].optValue;
    cyNumBuffers = optArray[NUM_BUFFERS_OPT_ARRAY_POS].optValue;
    cyAsymLoops = optArray[ASYM_LOOPS_OPT_ARRAY_POS].optValue;
    cySymLoops = optArray[SYM_LOOPS_OPT_ARRAY_POS].optValue;
    dcLoops = optArray[DC_LOOPS_OPT_ARRAY_POS].optValue;
    wirelessFirmware = optArray[WIRELESS_FIRMWARE_OPT_ARRAY_POS].optValue;
    configFileVersion = optArray[CONFIG_FILE_OPT_ARRAY_POS].optValue;
    runStateful = optArray[RUN_STATEFUL_ARRAY_POS].optValue;

    if(CPA_STATUS_SUCCESS != qaeMemInit())
    {
        PRINT_ERR("Could not start qae mem for user space\n");
        PRINT("Has the qae module been loaded?\n");
        return CPA_STATUS_FAIL;
    }
    else
    {
        PRINT("qaeMemInit started\n");
    }



	if(signOfLife)
    {
#ifdef DO_CRYPTO
		cySymDpNumBuffers = MIN_CY_BUFFERS;
        cySymDpNumLoops = MIN_SYM_LOOPS;
#endif /* DO_CRYPTO */
	}

    if(wirelessFirmware && USE_V1_CONFIG_FILE == configFileVersion )
    {
        processName = "WIRELESS_INT_0";
    }
    else if(wirelessFirmware && USE_V2_CONFIG_FILE == configFileVersion)
    {
        processName = "WIRELESS";
    }
    else
    {
        processName = "SSL";
    }

    if(USE_V1_CONFIG_FILE == configFileVersion)
    {
        if(CPA_STATUS_SUCCESS != icp_sal_userStart(processName))
        {
            PRINT_ERR("Could not start sal for user space\n");
            return CPA_STATUS_FAIL;
        }
        else
        {
            PRINT("icp_sal_userStart(\"%s\") started\n",processName);
        }
    }
    else if(USE_V2_CONFIG_FILE == configFileVersion)
    {
        if(CPA_STATUS_SUCCESS != icp_sal_userStartMultiProcess(processName,
                                                                CPA_FALSE))
        {
            PRINT_ERR("Could not start sal for user space\n");
            return CPA_STATUS_FAIL;
        }
        else
        {
            PRINT("icp_sal_userStartMultiProcess(\"%s\") started\n",processName);
        }
    }
    else
    {
        PRINT("Invalid configFileVer specified(%d) "
                "valid values are 1 or 2\n",configFileVersion);
        return CPA_STATUS_FAIL;
    }

#ifdef DO_CRYPTO
    if(wirelessFirmware)
    {
        numPacketSizes = WIRELESS_PACKET_LIMIT;
    }
#endif

#endif //USER_SPACE

    if(signOfLife)
    {
        cyNumBuffers = MIN_CY_BUFFERS;
        cySymLoops = MIN_SYM_LOOPS;
        cyAsymLoops = MIN_ASYM_LOOPS;
        dcLoops = MIN_DC_LOOPS;
    }
#ifdef DO_CRYPTO
    status = cpaCyGetNumInstances(&numInst_g);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCyGetNumInstances failed with status: %d\n", status);
        return status;
    }
    if(numInst_g > 0)
    {
        /*allocate memory to store the instance handles*/
        inst_g = qaeMemAlloc(sizeof(CpaInstanceHandle)*numInst_g);
        if(inst_g == NULL)
        {
            PRINT_ERR("Failed to allocate memory for instances\n");
            freeInstanceMapping();
            return CPA_STATUS_FAIL;
        }

        /*get the instances handles and place in allocated memory*/
        status = cpaCyGetInstances(numInst_g, inst_g);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("cpaCyGetInstances failed with status: %d\n", status);
            freeInstanceMapping();
            return status;
        }

        /*allocate memory for the instance core mapping*/
        cyInstMap_g = qaeMemAlloc(sizeof(Cpa32U)*numInst_g);
        if(cyInstMap_g == NULL)
        {
            PRINT_ERR("Failed to allocate memory for instance mapping\n");
            freeInstanceMapping();
            return CPA_STATUS_FAIL;
        }

        for(i = 0; i<numInst_g; i++)
        {
            status = cpaCyInstanceGetInfo2(inst_g[i], &info);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("could not get instance info\n");
                return status;
            }
            if(prevDevId == info.physInstId.packageId)
            {
                  continue;
            }
            prevDevId = info.physInstId.packageId;

            if(CPA_STATUS_SUCCESS !=
            icp_sal_getDevVersionInfo
                 (info.physInstId.packageId, &qaVersionInfo))
           {
                PRINT_ERR("Could not get QA version information\n");
           }
           else
           {
#ifdef WITH_UPSTREAM
               PRINT("*** QA version information ***\n");
               PRINT("device ID\t\t= %d\n", qaVersionInfo.devId);
               PRINT("software \t\t= %s\n", qaVersionInfo.softwareVersion);
               PRINT("*** END QA version information ***\n");
#else
               PRINT("*** QA version information ***\n");
               PRINT("device ID\t\t= %d\n", qaVersionInfo.devId);
               PRINT("firmware \t\t= %s\n", qaVersionInfo.firmwareVersion);
               PRINT("mmp      \t\t= %s\n", qaVersionInfo.mmpVersion);
               PRINT("software \t\t= %s\n", qaVersionInfo.softwareVersion);
               PRINT("hardware \t\t= %s\n", qaVersionInfo.hardwareVersion);
               PRINT("*** END QA version information ***\n");
#endif
           }

       }
    }
    else
    {
        PRINT("There are no crypto instances\n");
    }
#endif /* DO_CRYPTO */

#ifndef DO_CRYPTO
#ifdef INCLUDE_COMPRESSION
    status = cpaDcGetNumInstances(&numInst_g);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaDcGetNumInstances failed with status: %d\n", status);
        return status;
    }
    if(numInst_g > 0)
    {
        /*allocate memory to store the instance handles*/
        inst_g = qaeMemAlloc(sizeof(CpaInstanceHandle)*numInst_g);
        if(inst_g == NULL)
        {
            PRINT_ERR("Failed to allocate memory for instances\n");
            freeInstanceMapping();
            return CPA_STATUS_FAIL;
        }

        /*get the instances handles and place in allocated memory*/
        status = cpaDcGetInstances(numInst_g, inst_g);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("cpaDcGetInstances failed with status: %d\n", status);
            freeInstanceMapping();
            return status;
        }

        /*allocate memory for the instance core mapping*/
        dcInstMap_g = qaeMemAlloc(sizeof(Cpa32U)*numInst_g);
        if(dcInstMap_g == NULL)
        {
            PRINT_ERR("Failed to allocate memory for instance mapping\n");
            freeInstanceMapping();
            return CPA_STATUS_FAIL;
        }

        for(i = 0; i<numInst_g; i++)
        {
            status = cpaDcInstanceGetInfo2(inst_g[i], &info);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("could not get instance info\n");
                return status;
            }
            if(prevDevId == info.physInstId.packageId)
            {
                  continue;
            }
            prevDevId = info.physInstId.packageId;

            if(CPA_STATUS_SUCCESS !=
            icp_sal_getDevVersionInfo
                 (info.physInstId.packageId, &qaVersionInfo))
           {
                PRINT_ERR("Could not get QA version information\n");
           }
           else
           {
               PRINT("*** QA version information ***\n");
               PRINT("device ID\t\t= %d\n", qaVersionInfo.devId);
               PRINT("firmware \t\t= %s\n", qaVersionInfo.firmwareVersion);
               PRINT("mmp      \t\t= %s\n", qaVersionInfo.mmpVersion);
               PRINT("software \t\t= %s\n", qaVersionInfo.softwareVersion);
               PRINT("hardware \t\t= %s\n", qaVersionInfo.hardwareVersion);
               PRINT("*** END QA version information ***\n");
           }

       }
    }
    else
    {
        PRINT("There are no crypto instances\n");
    }


#endif /* INCLUDE_COMPRESSION */
#endif /* DO_CRYPTO */
    freeInstanceMapping();

#ifdef DO_CRYPTO
    /***************************************************************************
     * SYMMETRIC PERFORMANCE
     **************************************************************************/
    if((SYMMETRIC_CODE & runTests)== SYMMETRIC_CODE)
    {
        if(!wirelessFirmware)
        {
            /*AES128-CBC TEST*/
            for(lv_count=0; lv_count < numPacketSizes; lv_count++)
            {
                status = setupCipherTest(
                            CPA_CY_SYM_CIPHER_AES_CBC,
                            KEY_SIZE_128_IN_BYTES,
                            CPA_CY_PRIORITY_NORMAL,
                            ASYNC,
                            packetSizes[lv_count],
                            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                            cyNumBuffers,
                            cySymLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupCipherTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }

            /*AES256-CBC TEST*/
            for(lv_count=0; lv_count < numPacketSizes; lv_count++)
            {
                status = setupCipherTest(
                            CPA_CY_SYM_CIPHER_AES_CBC,
                            KEY_SIZE_256_IN_BYTES,
                            CPA_CY_PRIORITY_NORMAL,
                            ASYNC,
                            packetSizes[lv_count],
                            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                            cyNumBuffers,
                            cySymLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupCipherTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

            }

            /*KASUMI TEST*/
            for(lv_count=0; lv_count < numWirelessPacketSizes; lv_count++)
            {
                status = setupCipherTest(
                        CPA_CY_SYM_CIPHER_KASUMI_F8,
                        KEY_SIZE_128_IN_BYTES,
                        CPA_CY_PRIORITY_HIGH,
                        ASYNC,
                        wirelessPacketSizes[lv_count],
                        DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                        cyNumBuffers,
                        cySymLoops
                 );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupCipherDpTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

            }

            /*AES128-CBC HMAC-SHA1 test*/
            for(lv_count=0; lv_count < numPacketSizes; lv_count++)
            {
                status = setupAlgChainTest(
                            CPA_CY_SYM_CIPHER_AES_CBC,
                            KEY_SIZE_128_IN_BYTES,
                            CPA_CY_SYM_HASH_SHA1,
                            CPA_CY_SYM_HASH_MODE_AUTH,
                            SHA1_AUTH_KEY_LENGTH_IN_BYTES,
                            CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH,
                            CPA_CY_PRIORITY_NORMAL,
                            ASYNC,
                            packetSizes[lv_count],
                            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                            cyNumBuffers,
                            cySymLoops
                );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupAlgChainTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }/*End of test*/

            /*AES256-CBC HMAC-SHA512 test*/
            for(lv_count=0; lv_count < numPacketSizes; lv_count++)
            {
                status = setupAlgChainTest(
                            CPA_CY_SYM_CIPHER_AES_CBC,
                            KEY_SIZE_256_IN_BYTES,
                            CPA_CY_SYM_HASH_SHA512,
                            CPA_CY_SYM_HASH_MODE_AUTH,
                            SHA512_AUTH_KEY_LENGTH_IN_BYTES,
                            CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH,
                            CPA_CY_PRIORITY_NORMAL,
                            ASYNC,
                            packetSizes[lv_count],
                            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                            cyNumBuffers,
                            cySymLoops
                );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupAlgChainTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }/*End of test*/

            /*AES256-CBC AES-XCBC-MAC test*/
            for(lv_count=0; lv_count < numPacketSizes; lv_count++)
            {
                status = setupAlgChainTest(
                            CPA_CY_SYM_CIPHER_AES_CBC,
                            KEY_SIZE_256_IN_BYTES,
                            CPA_CY_SYM_HASH_AES_XCBC,
                            CPA_CY_SYM_HASH_MODE_AUTH,
                            AES_XCBC_DIGEST_LENGTH_IN_BYTES,
                            CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH,
                            CPA_CY_PRIORITY_NORMAL,
                            ASYNC,
                            packetSizes[lv_count],
                            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                            cyNumBuffers,
                            cySymLoops
                );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupAlgChainTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }/*End of test*/

        }/*end of tests not on wirelessFirmware*/

#ifdef USER_SPACE /* Data Plane support in User Space only */
        /*AES128-CBC TEST*/
        for(lv_count=0; lv_count < numPacketSizes; lv_count++)
        {

            status = setupCipherDpTest(
                        CPA_CY_SYM_CIPHER_AES_CBC,
                        KEY_SIZE_128_IN_BYTES,
                        CPA_CY_PRIORITY_HIGH,
                        ASYNC,
                        packetSizes[lv_count],
                        SYM_DP_ENQUEUEING,
                        DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                        SINGLE_REQUEST,
                        SYM_DP_SINGLE_SESSION,
                        cySymDpNumBuffers,
                        cySymDpNumLoops);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("Error calling setupCipherDpTest\n");
                return CPA_STATUS_FAIL;
            }
            status = createStartandWaitForCompletion(CRYPTO);
            if(status == CPA_STATUS_FAIL)
            {
                retStatus = CPA_STATUS_FAIL;
            }
        }


        /*AES256-CBC TEST*/
        for(lv_count=0; lv_count < numPacketSizes; lv_count++)
        {

            status = setupCipherDpTest(
                    CPA_CY_SYM_CIPHER_AES_CBC,
                    KEY_SIZE_256_IN_BYTES,
                    CPA_CY_PRIORITY_HIGH,
                    ASYNC,
                    packetSizes[lv_count],
                    SYM_DP_ENQUEUEING,
                    DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                    SINGLE_REQUEST,
                    SYM_DP_SINGLE_SESSION,
                    cySymDpNumBuffers,
                    cySymDpNumLoops
             );
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("Error calling setupCipherDpTest\n");
                return CPA_STATUS_FAIL;
            }
            status = createStartandWaitForCompletion(CRYPTO);
            if(status == CPA_STATUS_FAIL)
            {
                retStatus = CPA_STATUS_FAIL;
            }
        }

        /*KASUMI TEST*/
        for(lv_count=0; lv_count < numWirelessPacketSizes; lv_count++)
        {

            status = setupCipherDpTest(
                    CPA_CY_SYM_CIPHER_KASUMI_F8,
                    KEY_SIZE_128_IN_BYTES,
                    CPA_CY_PRIORITY_HIGH,
                    ASYNC,
                    wirelessPacketSizes[lv_count],
                    SYM_DP_ENQUEUEING,
                    DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                    SINGLE_REQUEST,
                    SYM_DP_SINGLE_SESSION,
                    cySymDpNumBuffers,
                    cySymDpNumLoops
             );
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("Error calling setupCipherDpTest\n");
                return CPA_STATUS_FAIL;
            }
            status = createStartandWaitForCompletion(CRYPTO);
            if(status == CPA_STATUS_FAIL)
            {
                retStatus = CPA_STATUS_FAIL;
            }
        }
        if(!wirelessFirmware)
        {
            /*AES128-CBC HMAC-SHA1 test*/
            for(lv_count=0; lv_count < numPacketSizes; lv_count++)
            {

                status = setupAlgChainDpTest(
                            CPA_CY_SYM_CIPHER_AES_CBC,
                            KEY_SIZE_128_IN_BYTES,
                            CPA_CY_SYM_HASH_SHA1,
                            CPA_CY_SYM_HASH_MODE_AUTH,
                            SHA1_AUTH_KEY_LENGTH_IN_BYTES,
                            CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH,
                            CPA_CY_PRIORITY_HIGH,
                            ASYNC,
                            packetSizes[lv_count],
                            SYM_DP_ENQUEUEING,
                            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                            SINGLE_REQUEST,
                            SYM_DP_SINGLE_SESSION,
                            cySymDpNumBuffers,
                            cySymDpNumLoops
                );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupAlgChainDpTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }/*End of test*/

            /*AES256-CBC HMAC-SHA512 test*/
            for(lv_count=0; lv_count < numPacketSizes; lv_count++)
            {

                status = setupAlgChainDpTest(
                            CPA_CY_SYM_CIPHER_AES_CBC,
                            KEY_SIZE_256_IN_BYTES,
                            CPA_CY_SYM_HASH_SHA512,
                            CPA_CY_SYM_HASH_MODE_AUTH,
                            SHA512_AUTH_KEY_LENGTH_IN_BYTES,
                            CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH,
                            CPA_CY_PRIORITY_HIGH,
                            ASYNC,
                            packetSizes[lv_count],
                            SYM_DP_ENQUEUEING,
                            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                            SINGLE_REQUEST,
                            SYM_DP_SINGLE_SESSION,
                            cySymDpNumBuffers,
                            cySymDpNumLoops
                );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupAlgChainDpTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }/*End of test*/

            /*AES256-CBC AES-XCBC-MAC test*/
            for(lv_count=0; lv_count < numPacketSizes; lv_count++)
            {

                status = setupAlgChainDpTest(
                            CPA_CY_SYM_CIPHER_AES_CBC,
                            KEY_SIZE_256_IN_BYTES,
                            CPA_CY_SYM_HASH_AES_XCBC,
                            CPA_CY_SYM_HASH_MODE_AUTH,
                            AES_XCBC_DIGEST_LENGTH_IN_BYTES,
                            CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH,
                            CPA_CY_PRIORITY_HIGH,
                            ASYNC,
                            packetSizes[lv_count],
                            SYM_DP_ENQUEUEING,
                            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                            SINGLE_REQUEST,
                            SYM_DP_SINGLE_SESSION,
                            cySymDpNumBuffers,
                            cySymDpNumLoops
                );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupAlgChainDpTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }/*End of test*/

        }/*end of test !wireless firmware*/


#endif /* USER_SPACE */
    }
#endif /* DO_CRYPTO */



    if(!wirelessFirmware)
    {
#ifdef DO_CRYPTO
		/**************************************************************************
		* RSA PERFORMANCE
		**************************************************************************/
		if((RSA_CODE & runTests)== RSA_CODE)
        {
            if(signOfLife)
            {
                numModSizes = ONE_PACKET;
            }
            for(lv_count=0; lv_count < numModSizes; lv_count++)
            {
                status = setupRsaTest(modSizes[lv_count],
                            CPA_CY_RSA_PRIVATE_KEY_REP_TYPE_2,
                            ASYNC,
                            cyNumBuffers,
                            cyAsymLoops );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupRsaTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }
        }
#endif /*DO_CRYPTO*/

#ifdef DO_CRYPTO
    /**************************************************************************
     * DH PERFORMANCE
     **************************************************************************/

        if((DH_CODE & runTests)== DH_CODE)
        {
            /*test the 2nd packet size of ModSize only for signOfLife*/
            if(signOfLife)
            {
                numModSizes = ONE_PACKET+ONE_PACKET;
            }
            /*modSize starts at 512 which is not supported by DH, so we skip the
             * 1st modSize*/
            for(lv_count=ONE_PACKET; lv_count < numModSizes; lv_count++)
            {
                status = setupDhTest(modSizes[lv_count],
                        EXPONENT_180_BIT,
                        ASYNC,
                        DH_PHASE_2,
                        cyNumBuffers,
                        cyAsymLoops );
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDhTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(CRYPTO);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
            }
        }
#endif /*DO_CRYPTO*/

#ifdef DO_CRYPTO
    /***************************************************************************
     * DSA CRYPTO TESTS
     **************************************************************************/
        if((DSA_CODE & runTests)== DSA_CODE)
        {
            status = setupDsaTest(MODULUS_1024_BIT,
                        EXPONENT_160_BIT,
                        ASYNC,
                        cyNumBuffers,
                        cyAsymLoops);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("Error calling setupDsaTest\n");
                return CPA_STATUS_FAIL;
            }
            status = createStartandWaitForCompletion(CRYPTO);
            if(status == CPA_STATUS_FAIL)
            {
                retStatus = CPA_STATUS_FAIL;
            }
        }
#endif /*DO_CRYPTO*/

#ifdef DO_CRYPTO
    /***************************************************************************
     * ECDSA CRYPTO TESTS
     **************************************************************************/
        if((ECDSA_CODE & runTests)== ECDSA_CODE)
        {
            status = setupEcdsaTest(GFP_P384_SIZE_IN_BITS /*GFP_P192_SIZE_IN_BITS*/,
                        CPA_CY_EC_FIELD_TYPE_PRIME,
                        ASYNC,
                        ECDSA_STEP_VERIFY,
                        cyNumBuffers,
                        cyAsymLoops);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("Error calling setupEcdsaTest\n");
                return CPA_STATUS_FAIL;
            }
            status = createStartandWaitForCompletion(CRYPTO);
            if(status == CPA_STATUS_FAIL)
            {
                retStatus = CPA_STATUS_FAIL;
            }
        }
#endif /*DO_CRYPTO*/

#ifdef INCLUDE_COMPRESSION
        if(signOfLife)
        {
            sampleCorpus = SIGN_OF_LIFE_CORPUS;
            dcBufferSize = BUFFER_SIZE_32768;
        }
        else
        {
            sampleCorpus = CALGARY_CORPUS;
            dcBufferSize = BUFFER_SIZE_65536;
        }
    /***************************************************************************
     * COMPRESSION TESTS CALGARY CORPUS
     **************************************************************************/
        if((COMPRESSION_CODE & runTests)== COMPRESSION_CODE)
        {

		status = cpaDcGetNumInstances(&numDcInst);
		/* Check the status */
		if(CPA_STATUS_SUCCESS != status)
		{
			PRINT_ERR("Unable to Get Number of DC instances\n");
			return CPA_STATUS_FAIL;
		}
		if(numDcInst>0)
		{

                dynamicHuffmanEnabled(NULL, &dynamicEnabled);

                /*STATIC L1 & L3 COMPRESSION*/
                status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_STATIC,
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

                status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_STATIC,
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

                status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L3,
                            CPA_DC_HT_STATIC,
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

                status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L3,
                            CPA_DC_HT_STATIC,
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

                /*DYNAMIC L1 & L3 COMPRESSION*/
                if(dynamicEnabled)
                {
                    status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_FULL_DYNAMIC,
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Error calling setupDcTest\n");
                        return CPA_STATUS_FAIL;
                    }
                    status = createStartandWaitForCompletion(COMPRESSION);
                    if(status == CPA_STATUS_FAIL)
                    {
                        retStatus = CPA_STATUS_FAIL;
                    }
                    status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_FULL_DYNAMIC,
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Error calling setupDcTest\n");
                        return CPA_STATUS_FAIL;
                    }
                    status = createStartandWaitForCompletion(COMPRESSION);
                    if(status == CPA_STATUS_FAIL)
                    {
                        retStatus = CPA_STATUS_FAIL;
                    }

                    status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L3,
                            CPA_DC_HT_FULL_DYNAMIC,
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Error calling setupDcTest\n");
                        return CPA_STATUS_FAIL;
                    }
                    status = createStartandWaitForCompletion(COMPRESSION);
                    if(status == CPA_STATUS_FAIL)
                    {
                        retStatus = CPA_STATUS_FAIL;
                    }

                    status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L3,
                            CPA_DC_HT_FULL_DYNAMIC,
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Error calling setupDcTest\n");
                        return CPA_STATUS_FAIL;
                    }
                    status = createStartandWaitForCompletion(COMPRESSION);
                    if(status == CPA_STATUS_FAIL)
                    {
                        retStatus = CPA_STATUS_FAIL;
                    }
                }

                /*DECOMPRESSION OF ZLIB COMPRESSED DATA*/
                useZlib();
                status = setupDcTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L1, /*not used in this test*/
                            CPA_DC_HT_STATIC, /*not used in this test*/
                            CPA_DC_FT_ASCII,
                            CPA_DC_STATELESS,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            dcBufferSize,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
                useAccelCompression();

                if(runStateful && dynamicEnabled)
                {
                    /*STATEFUL COMPRESSION TEST*/
                    status = setupDcStatefulTest(CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_FULL_DYNAMIC,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            SYNC,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Stateful setup failed\n");
                    }

                    /*set the array of cores to create threads on*/
                    for(lv_count = 0;
                        lv_count < sizeof(statefulMultiThreadCoreMap)/
                        sizeof(Cpa32U);
                        lv_count++)
                    {
                        statefulMultiThreadCoreMap[lv_count] =
                            lv_count % NUMBER_OF_CORES_TO_USE;
                    }
                    /*create the threads using all available instances*/
                    createPerfomanceThreads
                    (
                        sizeof(statefulMultiThreadCoreMap)/sizeof(Cpa32U),
                        statefulMultiThreadCoreMap,
                        numInst_g,
                        0
                    );
                    status = startThreads();
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT("Error starting threads\n");
                        return status;
                    }
                    status = waitForThreadCompletion();
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT("Stateful Compression thread(s) failed\n");
                        return status;
                    }
                }

#ifdef USER_SPACE
                /* Data Plane API Sample Code Test */
                /*STATIC DP_API L1 & L3 COMPRESS & DECOMPRESS*/
                status = setupDcDpTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_STATIC,
                            CPA_DC_FT_ASCII,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            DC_DP_ENQUEUEING,
                            SINGLE_REQUEST,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcDpTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

                status = setupDcDpTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_STATIC,
                            CPA_DC_FT_ASCII,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            DC_DP_ENQUEUEING,
                            SINGLE_REQUEST,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcDpTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

                status = setupDcDpTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L3,
                            CPA_DC_HT_STATIC,
                            CPA_DC_FT_ASCII,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            DC_DP_ENQUEUEING,
                            SINGLE_REQUEST,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcDpTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }

                status = setupDcDpTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L3,
                            CPA_DC_HT_STATIC,
                            CPA_DC_FT_ASCII,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            DC_DP_ENQUEUEING,
                            SINGLE_REQUEST,
                            dcLoops);
                if(CPA_STATUS_SUCCESS != status)
                {
                    PRINT_ERR("Error calling setupDcDpTest\n");
                    return CPA_STATUS_FAIL;
                }
                status = createStartandWaitForCompletion(COMPRESSION);
                if(status == CPA_STATUS_FAIL)
                {
                    retStatus = CPA_STATUS_FAIL;
                }
                if(dynamicEnabled)
                {
                    /*DYNAMIC DP_API L1 & L3 COMPRESS & DECOMPRESS*/
                    status = setupDcDpTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_FULL_DYNAMIC,
                            CPA_DC_FT_ASCII,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            DC_DP_ENQUEUEING,
                            SINGLE_REQUEST,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Error calling setupDcDpTest\n");
                        return CPA_STATUS_FAIL;
                    }
                    status = createStartandWaitForCompletion(COMPRESSION);
                    if(status == CPA_STATUS_FAIL)
                    {
                        retStatus = CPA_STATUS_FAIL;
                    }

                    status = setupDcDpTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L1,
                            CPA_DC_HT_FULL_DYNAMIC,
                            CPA_DC_FT_ASCII,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            DC_DP_ENQUEUEING,
                            SINGLE_REQUEST,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Error calling setupDcDpTest\n");
                        return CPA_STATUS_FAIL;
                    }
                    status = createStartandWaitForCompletion(COMPRESSION);
                    if(status == CPA_STATUS_FAIL)
                    {
                        retStatus = CPA_STATUS_FAIL;
                    }

                    status = setupDcDpTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_COMPRESS,
                            SAMPLE_CODE_CPA_DC_L3,
                            CPA_DC_HT_FULL_DYNAMIC,
                            CPA_DC_FT_ASCII,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            DC_DP_ENQUEUEING,
                            SINGLE_REQUEST,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Error calling setupDcDpTest\n");
                        return CPA_STATUS_FAIL;
                    }
                    status = createStartandWaitForCompletion(COMPRESSION);
                    if(status == CPA_STATUS_FAIL)
                    {
                        retStatus = CPA_STATUS_FAIL;
                    }

                    status = setupDcDpTest( CPA_DC_DEFLATE,
                            CPA_DC_DIR_DECOMPRESS,
                            SAMPLE_CODE_CPA_DC_L3,
                            CPA_DC_HT_FULL_DYNAMIC,
                            CPA_DC_FT_ASCII,
                            DEFAULT_COMPRESSION_WINDOW_SIZE,
                            BUFFER_SIZE_8192,
                            sampleCorpus,
                            CPA_SAMPLE_ASYNCHRONOUS,
                            DC_DP_ENQUEUEING,
                            SINGLE_REQUEST,
                            dcLoops);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        PRINT_ERR("Error calling setupDcDpTest\n");
                        return CPA_STATUS_FAIL;
                    }
                    status = createStartandWaitForCompletion(COMPRESSION);
                    if(status == CPA_STATUS_FAIL)
                    {
                        retStatus = CPA_STATUS_FAIL;
                    }
                }
#endif /* End USER_SPACE */
            }//End of if(numDcInst>0)
        }//End of if((COMPRESSION_CODE & runTests)== COMPRESSION_CODE
#endif
    }/*end if not wireless firmware*/
#ifdef USER_SPACE
    if(CPA_STATUS_SUCCESS != icp_sal_userStop())
    {
        PRINT_ERR("Could not stop sal for user space\n");
        return CPA_STATUS_FAIL;
    }
    qaeMemDestroy();
#endif /* USER_SPACE */
    return retStatus;
}
