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
 *  version: QAT1.6.L.2.6.0-65
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file cpa_sample_utils.c
 *
 * @ingroup sampleCode
 *
 * @description
 * Defines functions to get an instance and poll an instance
 *
 ***************************************************************************/

#include "cpa_sample_utils.h"
#include "cpa_dc.h"
#include "icp_sal_poll.h"

/*
 * Maximum number of instances to query from the API
 */
#define MAX_INSTANCES 1

#ifdef DO_CRYPTO
static sampleThread gPollingThread;
static int gPollingCy = 0;
#endif

static sampleThread gPollingThreadDc;
static int gPollingDc = 0;

/* *************************************************************
 *
 * Common instance funcitons
 *
 * ************************************************************* */

/*
 * This function returns a handle to an instance of the cryptographic
 * API.  It does this by querying the API for all instances and
 * returning the first such instance.
 */
//<snippet name="getInstance">
#ifdef DO_CRYPTO
void
sampleCyGetInstance(CpaInstanceHandle* pCyInstHandle)
{
    CpaInstanceHandle cyInstHandles[MAX_INSTANCES];
    Cpa16U numInstances = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;

    *pCyInstHandle = NULL;

    status = cpaCyGetNumInstances(&numInstances);
    if ((status == CPA_STATUS_SUCCESS) && (numInstances > 0))
    {
        status = cpaCyGetInstances(MAX_INSTANCES, cyInstHandles);
        if (status == CPA_STATUS_SUCCESS)
        {
            *pCyInstHandle = cyInstHandles[0];
        }
    }

    if (0 == numInstances)
    {
        PRINT_ERR("No instances found for 'SSL'\n");
        PRINT_ERR("Please check your section names in the config file.\n");
        PRINT_ERR("Also make sure to use config file version 2.\n");
    }
   
}
#endif
//</snippet>

/* 
 * This function polls a crypto instance.
 *
 */
#ifdef DO_CRYPTO
static void
sal_polling(CpaInstanceHandle cyInstHandle)
{
   gPollingCy = 1;
   while(gPollingCy)
   {
       icp_sal_CyPollInstance(cyInstHandle, 0);
       OS_SLEEP(10);
   }

   sampleThreadExit();
}
#endif
/*
 * This function checks the instance info. If the instance is
 * required to be polled then it starts a polling thread.
 */

#ifdef DO_CRYPTO
void
sampleCyStartPolling(CpaInstanceHandle cyInstHandle)
{
   CpaInstanceInfo2 info2 = {0};
   CpaStatus status = CPA_STATUS_SUCCESS;

   status = cpaCyInstanceGetInfo2(cyInstHandle, &info2);
   if((status == CPA_STATUS_SUCCESS) && (info2.isPolled == CPA_TRUE))
   {
      /* Start thread to poll instance */
      sampleThreadCreate(&gPollingThread, sal_polling, cyInstHandle);
   }

}
#endif
/*
 * This function stops the polling of a crypto instance. 
 */
#ifdef DO_CRYPTO
void
sampleCyStopPolling(void)
{

   gPollingCy = 0;
}
#endif


/*
 * This function returns a handle to an instance of the data
 * compression API.  It does this by querying the API for all
 * instances and returning the first such instance.
 */
//<snippet name="getInstanceDc">
void
sampleDcGetInstance(CpaInstanceHandle* pDcInstHandle)
{
    CpaInstanceHandle dcInstHandles[MAX_INSTANCES];
    Cpa16U numInstances = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;

    *pDcInstHandle = NULL;

    status = cpaDcGetNumInstances(&numInstances);
    if ((status == CPA_STATUS_SUCCESS) && (numInstances > 0))
    {
        status = cpaDcGetInstances(MAX_INSTANCES, dcInstHandles);
        if (status == CPA_STATUS_SUCCESS)
        {
            *pDcInstHandle = dcInstHandles[0];
        }
    }

    if (0 == numInstances)
    {
        PRINT_ERR("No instances found for 'SSL'\n");
        PRINT_ERR("Please check your section names in the config file.\n");
        PRINT_ERR("Also make sure to use config file version 2.\n");
    }

}
//</snippet>

/*
 * This function polls a compression instance.
 *
 */
static void
sal_dc_polling(CpaInstanceHandle dcInstHandle)
{

   gPollingDc = 1;
   while(gPollingDc)
   {
       icp_sal_DcPollInstance(dcInstHandle, 0);
       OS_SLEEP(10);
   }

   sampleThreadExit();
}

/*
 * This function checks the instance info. If the instance is
 * required to be polled then it starts a polling thread.
 */
void
sampleDcStartPolling(CpaInstanceHandle dcInstHandle)
{
   CpaInstanceInfo2 info2 = {0};
   CpaStatus status = CPA_STATUS_SUCCESS;

   status = cpaDcInstanceGetInfo2(dcInstHandle, &info2);
   if((status == CPA_STATUS_SUCCESS) && (info2.isPolled == CPA_TRUE))
   {
      /* Start thread to poll instance */
      sampleThreadCreate(&gPollingThreadDc, sal_dc_polling, dcInstHandle);
   }

}

/*
 * This function stops the thread polling the compression instance.
 */
void
sampleDcStopPolling(void)
{

   gPollingDc = 0;
}
