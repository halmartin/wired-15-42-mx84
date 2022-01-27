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
 ***************************************************************************
 * @file cpa_impl_mux.c
 *    Implementation of the mux/demux shim layer for mulitple implementations
 *    of the QA API.
 *
 * @ingroup cpaImplMux
 ***************************************************************************/

/*
******************************************************************************
* Include public/global header files
******************************************************************************
*/
#ifdef USER_SPACE
#include <stdio.h>
#include <dlfcn.h>
#include <string.h>
#endif
#include "cpa_mux_common.h"

typedef struct
{
    ListEntry l;
    CpaCyDrbgSessionHandle sessionHandle;
    CpaInstanceHandle instanceHandle;
} CpaMuxDrbgSessionInfo;

typedef struct
{
    const char *name;
    Cpa32U offset;
} CpaFuncMembers_t;
volatile Cpa16U cpaMuxNumImpls = 0;

#ifdef USER_SPACE
STATIC void *handle_qat_1_6 = NULL;
STATIC void *handle_qat_1_5 = NULL;
#endif

CpaMuxImpl cpaMuxImpls[MAX_NUM_IMPLS];

/** Create an array describing each member of CpaFuncPtrs, and its offset
 * within CpaFuncPtrs.  This makes the code which does symbol lookup for
 * user space, and structure validation for kernel space, much cleaner.
 */

#define MEMBER(name) { #name, offsetof(CpaFuncPtrs, name) }
STATIC CpaFuncMembers_t CpaFuncMembers[] = {
    MEMBER (icp_sal_getDevVersionInfo),
    MEMBER (cpaCySymDpEnqueueOp),
    MEMBER (cpaCySymDpEnqueueOpBatch),
    MEMBER (cpaCySymDpRegCbFunc),
    MEMBER (cpaCySymDpSessionCtxGetSize),
    MEMBER (cpaCySymDpSessionCtxGetDynamicSize),
    MEMBER (cpaCySymDpInitSession),
    MEMBER (cpaCySymDpRemoveSession),
    MEMBER (cpaCySymDpPerformOpNow),
    MEMBER (cpaCySymInitSession),
    MEMBER (cpaCySymRemoveSession),
    MEMBER (cpaCySymPerformOp),
    MEMBER (cpaCySymQueryStats),
    MEMBER (cpaCySymQueryStats64),
    MEMBER (cpaCySymQueryCapabilities),
    MEMBER (cpaCySymSessionCtxGetSize),
    MEMBER (cpaCySymSessionCtxGetDynamicSize),
    MEMBER (cpaCyDrbgInitSession),
    MEMBER (cpaCyDrbgReseed),
    MEMBER (cpaCyDrbgGen),
    MEMBER (cpaCyDrbgSessionGetSize),
    MEMBER (cpaCyDrbgRemoveSession),
    MEMBER (cpaCyDrbgQueryStats64),
    MEMBER (icp_sal_drbgGetEntropyInputFuncRegister),
    MEMBER (icp_sal_drbgGetNonceFuncRegister),
    MEMBER (icp_sal_drbgIsDFReqFuncRegister),
    MEMBER (icp_sal_drbgGetInstance),
    MEMBER (icp_sal_drbgHTGetTestSessionSize),
    MEMBER (icp_sal_drbgHTInstantiate),
    MEMBER (icp_sal_drbgHTGenerate),
    MEMBER (icp_sal_drbgHTReseed),
    MEMBER (cpaCyNrbgGetEntropy),
    MEMBER (icp_sal_nrbgHealthTest),
    MEMBER (cpaCyDhKeyGenPhase1),
    MEMBER (cpaCyDhKeyGenPhase2Secret),
    MEMBER (cpaCyDhQueryStats),
    MEMBER (cpaCyDhQueryStats64),
    MEMBER (cpaCyKeyGenSsl),
    MEMBER (cpaCyKeyGenTls),
    MEMBER (cpaCyKeyGenTls2),
    MEMBER (cpaCyKeyGenMgf),
    MEMBER (cpaCyKeyGenMgfExt),
    MEMBER (cpaCyKeyGenQueryStats),
    MEMBER (cpaCyKeyGenQueryStats64),
    MEMBER (cpaCyLnModExp),
    MEMBER (cpaCyLnModInv),
    MEMBER (cpaCyLnStatsQuery),
    MEMBER (cpaCyLnStatsQuery64),
    MEMBER (cpaCyPrimeTest),
    MEMBER (cpaCyPrimeQueryStats),
    MEMBER (cpaCyPrimeQueryStats64),
    MEMBER (cpaCyDsaGenPParam),
    MEMBER (cpaCyDsaGenGParam),
    MEMBER (cpaCyDsaGenYParam),
    MEMBER (cpaCyDsaSignR),
    MEMBER (cpaCyDsaSignS),
    MEMBER (cpaCyDsaSignRS),
    MEMBER (cpaCyDsaVerify),
    MEMBER (cpaCyDsaQueryStats),
    MEMBER (cpaCyDsaQueryStats64),
    MEMBER (cpaCyRsaGenKey),
    MEMBER (cpaCyRsaEncrypt),
    MEMBER (cpaCyRsaDecrypt),
    MEMBER (cpaCyRsaQueryStats),
    MEMBER (cpaCyRsaQueryStats64),
    MEMBER (cpaCyEcPointMultiply),
    MEMBER (cpaCyEcPointVerify),
    MEMBER (cpaCyEcQueryStats64),
    MEMBER (cpaCyEcdhPointMultiply),
    MEMBER (cpaCyEcdhQueryStats64),
    MEMBER (cpaCyEcdsaSignR),
    MEMBER (cpaCyEcdsaSignS),
    MEMBER (cpaCyEcdsaSignRS),
    MEMBER (cpaCyEcdsaVerify),
    MEMBER (cpaCyEcdsaQueryStats64),
    MEMBER (cpaDcInitSession),
    MEMBER (cpaDcRemoveSession),
    MEMBER (cpaDcResetSession),
    MEMBER (cpaDcCompressData),
    MEMBER (cpaDcDecompressData),
    MEMBER (cpaDcGenerateHeader),
    MEMBER (cpaDcGenerateFooter),
    MEMBER (cpaDcGetStats),
    MEMBER (cpaDcDpGetSessionSize),
    MEMBER (cpaDcDpInitSession),
    MEMBER (cpaDcDpRemoveSession),
    MEMBER (cpaDcDpRegCbFunc),
    MEMBER (cpaDcDpEnqueueOp),
    MEMBER (cpaDcDpEnqueueOpBatch),
    MEMBER (cpaDcDpPerformOpNow),
    MEMBER (cpaCyBufferListGetMetaSize),
    MEMBER (cpaCyGetInstances),
    MEMBER (cpaCyGetNumInstances),
    MEMBER (cpaCyGetStatusText),
    MEMBER (cpaCyInstanceGetInfo),
    MEMBER (cpaCyInstanceGetInfo2),
    MEMBER (cpaCyInstanceSetNotificationCb),
    MEMBER (cpaCySetAddressTranslation),
    MEMBER (cpaCyStartInstance),
    MEMBER (cpaCyStopInstance),
    MEMBER (cpaCyQueryCapabilities),
    MEMBER (cpaDcGetInstances),
    MEMBER (cpaDcGetNumInstances),
    MEMBER (cpaDcGetNumIntermediateBuffers),
    MEMBER (cpaDcGetSessionSize),
    MEMBER (cpaDcGetStatusText),
    MEMBER (cpaDcBufferListGetMetaSize),
    MEMBER (cpaDcInstanceGetInfo2),
    MEMBER (cpaDcInstanceSetNotificationCb),
    MEMBER (cpaDcQueryCapabilities),
    MEMBER (cpaDcSetAddressTranslation),
    MEMBER (cpaDcStartInstance),
    MEMBER (cpaDcStopInstance),
    MEMBER (icp_sal_CyPollInstance),
    MEMBER (icp_sal_CyPollDpInstance),
    MEMBER (icp_sal_DcPollInstance),
    MEMBER (icp_sal_DcPollDpInstance),
#ifdef USER_SPACE
    MEMBER (icp_sal_userStartMultiProcess),
    MEMBER (icp_sal_userStart),
    MEMBER (icp_sal_userStop),
    MEMBER (icp_sal_userCyGetAvailableNumDynInstances),
    MEMBER (icp_sal_userDcGetAvailableNumDynInstances),
    MEMBER (icp_sal_userCyInstancesAlloc),
    MEMBER (icp_sal_userDcInstancesAlloc),
    MEMBER (icp_sal_userCyFreeInstances),
    MEMBER (icp_sal_userDcFreeInstances),
    MEMBER (icp_sal_userCyGetAvailableNumDynInstancesByDevPkg),
    MEMBER (icp_sal_userDcGetAvailableNumDynInstancesByDevPkg),
    MEMBER (icp_sal_userCyGetAvailableNumDynInstancesByPkgAccel),
    MEMBER (icp_sal_userCyInstancesAllocByPkgAccel),
    MEMBER (icp_sal_userCyInstancesAllocByDevPkg),
    MEMBER (icp_sal_userDcInstancesAllocByDevPkg),
    MEMBER (icp_sal_check_device),
    MEMBER (icp_sal_check_all_devices),
    MEMBER (icp_sal_userGetPfVfcommsStatus),
    MEMBER (icp_sal_userGetMsgFromPf),
    MEMBER (icp_sal_userGetMsgFromVf),
    MEMBER (icp_sal_userSendMsgToPf),
    MEMBER (icp_sal_userSendMsgToVf),
    MEMBER (icp_sal_reset_device),
    MEMBER (icp_sal_poll_device_events),
    MEMBER (icp_sal_find_new_devices),
    MEMBER (icp_sal_CyGetFileDescriptor),
    MEMBER (icp_sal_CyPutFileDescriptor),
    MEMBER (icp_sal_DcGetFileDescriptor),
    MEMBER (icp_sal_DcPutFileDescriptor),
#endif
#ifdef ONE_KO_RELEASE_PACKAGE
    MEMBER (icp_sal_pollBank),
    MEMBER (icp_sal_pollAllBanks),
#endif
    MEMBER (icp_sal_iommu_get_remap_size),
    MEMBER (icp_sal_iommu_map),
    MEMBER (icp_sal_iommu_unmap),
    MEMBER (icp_amgr_getNumInstances),
    {0, 0}
};

#undef MEMBER

/**
 * List management functions for doubly linked lists.  We have list of
 * registered implementations, each implementation has a list of cyInstances
 * and a list of dcInstances.
 */

static inline void LIST_INIT(ListEntry *p)
{
    p->next = p->prev = p;
}

static inline void LIST_INSERT_AFTER(ListEntry *pIns, ListEntry *pAft)
{
    pIns->next = pAft->next;
    pIns->prev = pAft;
    pAft->next->prev = pIns;
    pAft->next = pIns;
}

static inline void LIST_REMOVE(ListEntry *p)
{
    p->next->prev = p->prev;
    p->prev->next = p->next;
    p->prev = p->next = NULL;
}

/**
 * APIs to register and deregister implementations
 */

CpaStatus
cpaMuxRegisterImpl (CpaMuxDriverType driverType,
                    char *pDymLibPath,
                    CpaFuncPtrs * pImplFuncts,
                    void **implHandle)
{
    CpaFuncPtrs *pFuncts = NULL;
    CpaMuxImpl *impl = NULL;
    Cpa16U i = 0;

    CHECK_NULL_PARAM (implHandle);

    if (cpaMuxNumImpls >= MAX_NUM_IMPLS)
    {
        MUX_ERROR("Exceeded the max number of implementations that "
                           "can be registered");
        return CPA_STATUS_INVALID_PARAM;
    }

    if ((driverType < 0) || (driverType >= MAX_NUM_IMPLS))
    {
        MUX_ERROR("Invalid driverType specified");
        return CPA_STATUS_INVALID_PARAM;
    }

    if (cpaMuxImpls[driverType].registered)
    {
        MUX_ERROR
             ("Implementation already registered for specified device type");
        return CPA_STATUS_RESOURCE;
    }

    impl = &cpaMuxImpls[driverType];
    LIST_INIT(&impl->drbgSessions);
    pFuncts = &(impl->implFunctPtrs);

#ifdef USER_SPACE
    CHECK_NULL_PARAM (pDymLibPath);

    if (NULL == (impl->dlHandle = dlopen (pDymLibPath, RTLD_LAZY)))
    {
        MUX_WARN1 ("%s Library not loaded", pDymLibPath);
        return CPA_STATUS_INVALID_PARAM;
    }
    else
    {
        void *addr = NULL;

        for (i = 0; CpaFuncMembers[i].name; i++)
        {
            addr = dlsym (impl->dlHandle, CpaFuncMembers[i].name);
            if (NULL == addr)
            {
                dlclose (impl->dlHandle);
                MUX_PRINT ("CpaMux: Symbol %s not found in %s\n",
                           CpaFuncMembers[i].name, pDymLibPath);
                return CPA_STATUS_INVALID_PARAM;
            }
            *(void **) ((unsigned long) pFuncts + CpaFuncMembers[i].offset) =
                addr;
        }
    }
#else
    CHECK_NULL_PARAM (pImplFuncts);
    {
        *pFuncts = *pImplFuncts;
        for (i = 0; CpaFuncMembers[i].name; i++)
        {
            if (*(void **)
                ((unsigned long) pFuncts + CpaFuncMembers[i].offset) == NULL)
            {
                MUX_PRINT
                    ("CpaMux: Member %s not provided with CpaFuncPtrs %p\n",
                     CpaFuncMembers[i].name, pImplFuncts);
                return CPA_STATUS_INVALID_PARAM;
            }
        }
    }
#endif

    /**
     * Note: Do not wish to get the instances here as we can do this in the 
     * individual getinstances.
     */
    cpaMuxNumImpls++;
    impl->registered = 1;

    *implHandle = impl;

    return CPA_STATUS_SUCCESS;
}

CpaStatus
cpaMuxDeRegisterImpl (void *implHandle)
{
    CpaMuxImpl *impl = NULL;
    CHECK_NULL_PARAM (implHandle);
    impl = (CpaMuxImpl *)implHandle;

    if (!impl->registered)
    {
        return CPA_STATUS_INVALID_PARAM;
    }

#ifdef USER_SPACE
    dlclose (impl->dlHandle);
#endif
    impl->registered = 0;
    cpaMuxNumImpls--;

    return CPA_STATUS_SUCCESS;
}

/**
 * ========================================================================
 * API functions that are not instance-specific.
 * ========================================================================
 */
STATIC CpaStatus
cpaMuxGetNumInstances (Cpa16U * pNumInstances, CpaMuxInstanceType type)
{
    Cpa16U curNumInstances = 0;
    Cpa16U i = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;

    if (pNumInstances == NULL)
    {
        MUX_ERROR("pNumInstances parameter cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }

    *pNumInstances = 0;

    /**
     * Iterate across all registered implementations 
     */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            switch (type)
            {
            case CPA_MUX_INST_TYPE_CY:
                status = impl->implFunctPtrs.cpaCyGetNumInstances(
                                                      &curNumInstances);
                break;
            case CPA_MUX_INST_TYPE_DC:
                status = impl->implFunctPtrs.cpaDcGetNumInstances(
                                                      &curNumInstances);
                break;
            default:
                MUX_ERROR("Invalid instance type");
                return CPA_STATUS_FAIL;
            }

            if (CPA_STATUS_SUCCESS != status)
            {  
                return status;
            }

            *pNumInstances += curNumInstances;
        }
    }
    return status;
}

CpaStatus
cpaCyGetNumInstances (Cpa16U * pNumInstances)
{
    return cpaMuxGetNumInstances (pNumInstances, CPA_MUX_INST_TYPE_CY);
}

CpaStatus
cpaDcGetNumInstances (Cpa16U * pNumInstances)
{
    return cpaMuxGetNumInstances (pNumInstances, CPA_MUX_INST_TYPE_DC);
}

/**
 * Common GetInstances function, shared by Cy and Dc
 */
STATIC CpaStatus
cpaMuxGetInstances (Cpa16U numInstances, CpaInstanceHandle *instances,
                    CpaMuxInstanceType type)
{
    Cpa16U curNumInstances = 0;
    Cpa16U totalNumInstances = 0;
    Cpa16U i = 0;     
    Cpa16U j = 0;     
    CpaStatus status = CPA_STATUS_SUCCESS;
    
    if (0 == numInstances)
    {
        MUX_ERROR("numInstances is 0");
        return CPA_STATUS_INVALID_PARAM;
    }

    if (NULL == instances)
    {
        MUX_ERROR("instances parameter cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }

    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            switch (type)
            {
            case CPA_MUX_INST_TYPE_CY:
                status = impl->implFunctPtrs.cpaCyGetNumInstances(
                                                     &curNumInstances);
                break;
            case CPA_MUX_INST_TYPE_DC:
                status = impl->implFunctPtrs.cpaDcGetNumInstances(
                                                     &curNumInstances);
                break;
            default:
                MUX_ERROR("Invalid instance type");
                return CPA_STATUS_FAIL;
            }
            
            if (CPA_STATUS_SUCCESS != status)
            {
                MUX_ERROR
                    ("Failed to get no of instances for a registered impl");
                return status;
            }

            /* Make sure we don't overflow the 'instances' array provided */
            if (curNumInstances > (numInstances - totalNumInstances))
            {    
                curNumInstances = numInstances - totalNumInstances;
            }

            if (curNumInstances)
            {

                switch (type)
                {
                case CPA_MUX_INST_TYPE_CY:
                    status = impl->implFunctPtrs.cpaCyGetInstances(
                        curNumInstances, instances + totalNumInstances);
                    break;
                case CPA_MUX_INST_TYPE_DC:
                    status = impl->implFunctPtrs.cpaDcGetInstances(
                        curNumInstances, instances + totalNumInstances);
                    break;
                default:
                    MUX_ERROR("Internal error: Invalid instance type");
                    return CPA_STATUS_FAIL;
                }
                if (CPA_STATUS_SUCCESS != status)
                {
                    MUX_ERROR
                     ("Failed to get instances for registered implemenation");
                    return status;
                }
            
                for (j = 0; j < curNumInstances; j++)
                {
                    CpaMuxInstance * pMuxInst =
                        (CpaMuxInstance *) instances[totalNumInstances + j];
                
                    if (!pMuxInst)
                    {
                        MUX_ERROR("Failed to insert implementation ref");
                        return CPA_STATUS_INVALID_PARAM;
                    }
                
                    /** Store a reference to the implementation within the
                     * InstanceHandle that we can use later
                     */
                    pMuxInst->pMuxImpl = impl;
                }
            }

            totalNumInstances += curNumInstances;
        }
    }

    if (numInstances > totalNumInstances)
    {
        MUX_PRINT("Only %d instances available (%d requested)\n",
                   totalNumInstances, numInstances);
        return CPA_STATUS_RESOURCE;
    }
    return status;
}

CpaStatus
cpaCyGetInstances (Cpa16U numInstances, CpaInstanceHandle * cyInstances)
{
    return cpaMuxGetInstances(numInstances, cyInstances, CPA_MUX_INST_TYPE_CY);
}

CpaStatus
cpaDcGetInstances (Cpa16U numInstances, CpaInstanceHandle * cyInstances)
{
    return cpaMuxGetInstances(numInstances, cyInstances, CPA_MUX_INST_TYPE_DC);
}

static CpaMuxImpl *
cpaMuxAccelIdToImpl(Cpa32U *accelId)
{
    Cpa16U i, devAccelIdOffset = 0;
    CpaMuxImpl *impl = NULL; 
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa16U numDevs = 0;
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {

        if (cpaMuxImpls[i].registered)
        {
            impl = &cpaMuxImpls[i]; 
            
            status = impl->implFunctPtrs.icp_amgr_getNumInstances(&numDevs);
            if (status != CPA_STATUS_SUCCESS)
            {
                MUX_ERROR("Failed to get numInstances");
                continue;
            }
            if (*accelId - devAccelIdOffset < numDevs)
            {
                *accelId -= devAccelIdOffset;
                break;
            }
            devAccelIdOffset += numDevs;
        }
    }
    return impl;
}

#ifdef USER_SPACE
/**
 * QA implmentation specific APIs - not defined for all APIs.
 */

CpaStatus
icp_sal_userStartMultiProcess (const char *pProcessName,
                               CpaBoolean limitDevAccess)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaBoolean validInstanceFound = CPA_FALSE;
    Cpa16U i = 0;
    
    status = cpaMuxRegisterImpl(CPA_MUX_DRIVER_TYPE_QAT_1_6,
                                "libqat_1_6_mux_s.so", NULL, &handle_qat_1_6);
    if (CPA_STATUS_SUCCESS != status)
    {
        MUX_WARN("qat_1_6 driver not registered with qat_mux");
    }  
    status = cpaMuxRegisterImpl(CPA_MUX_DRIVER_TYPE_QAT_1_5,
                        "libqat_1_5_mux_s.so", NULL, &handle_qat_1_5);
    if (CPA_STATUS_SUCCESS != status)
    {
        MUX_WARN("qat_1_5 driver not registered with qat_mux");
    }  

    if (0 == cpaMuxNumImpls)
    {
        MUX_ERROR("No implementations have been registered");
        return CPA_STATUS_INVALID_PARAM;
    }

/**
 * Best we can do is loop through all implementations and call start on each.
 */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            status = impl->implFunctPtrs.
                icp_sal_userStartMultiProcess (pProcessName, limitDevAccess);
            if (CPA_STATUS_SUCCESS == status)
            { 
                validInstanceFound = CPA_TRUE;
            }
        }
    }

    return ((validInstanceFound ==
             CPA_FALSE) ? CPA_STATUS_FAIL : CPA_STATUS_SUCCESS);
}

CpaStatus
icp_sal_userStart (const char *pProcessName)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaBoolean validInstanceFound = CPA_FALSE;
    Cpa16U i = 0;
   
    status = cpaMuxRegisterImpl(CPA_MUX_DRIVER_TYPE_QAT_1_6,
                                "libqat_1_6_mux_s.so", NULL, &handle_qat_1_6);
    if (CPA_STATUS_SUCCESS != status)
    {
        MUX_WARN("qat_1_6 driver not registered with qat_mux");
    }  
    status = cpaMuxRegisterImpl(CPA_MUX_DRIVER_TYPE_QAT_1_5,
                                "libqat_1_5_mux_s.so", NULL, &handle_qat_1_5);
    if (CPA_STATUS_SUCCESS != status)
    {
        MUX_WARN("qat_1_5 driver not registered with qat_mux");
    }  

    if (0 == cpaMuxNumImpls)
    {
        MUX_ERROR("No implementations have been registered");
        return CPA_STATUS_INVALID_PARAM;
    }

/**
 * Best we can do is loop through all implementations and call start on each.
 */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];
            
            status = impl->implFunctPtrs.icp_sal_userStart (pProcessName);
            if (CPA_STATUS_SUCCESS == status)
            {
                validInstanceFound = CPA_TRUE;
            }
        }
    }

    return ((validInstanceFound ==
             CPA_FALSE) ? CPA_STATUS_FAIL : CPA_STATUS_SUCCESS);
}

CpaStatus
icp_sal_userStop (void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaBoolean validInstanceFound = CPA_FALSE;
    Cpa16U i = 0;

    if (0 == cpaMuxNumImpls)
    {
        MUX_ERROR("No implementations have been registered");
        return CPA_STATUS_INVALID_PARAM;
    }

/**
 * Best we can do is loop through all implementations and call stop on each.
 */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];
            
            status = impl->implFunctPtrs.icp_sal_userStop ();
            if (CPA_STATUS_SUCCESS == status)
            {
                validInstanceFound = CPA_TRUE;
            }
        }
    }
    
    if (NULL != handle_qat_1_6)
    {
        status = cpaMuxDeRegisterImpl(handle_qat_1_6);
        if (CPA_STATUS_SUCCESS != status)
        {
            MUX_WARN("qat_1_6 driver not deRegistered from qat_mux");
        }
    }  
    if (NULL != handle_qat_1_5)
    {
        status = cpaMuxDeRegisterImpl(handle_qat_1_5);
      
        if (CPA_STATUS_SUCCESS != status)
        {
            MUX_WARN("qat_1_5 driver not deRegistered from qat_mux");
        }  
    }
    return ((validInstanceFound ==
             CPA_FALSE) ? CPA_STATUS_FAIL : CPA_STATUS_SUCCESS);
}

STATIC CpaStatus
cpaMuxGetAvailNumDynInstances (Cpa32U * pNumInstances, CpaMuxInstanceType type)
{
    Cpa32U curNumInstances = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa16U i = 0;

    if (NULL == pNumInstances)
    {
        MUX_ERROR("pNumInstances param cannot be NULL\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    *pNumInstances = 0;

    /**
     * Iterate across all registered implementations 
     */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            switch (type)
            {
            case CPA_MUX_INST_TYPE_CY:
                status = impl->implFunctPtrs.
                            icp_sal_userCyGetAvailableNumDynInstances(
                                                      &curNumInstances);
                break;
            case CPA_MUX_INST_TYPE_DC:
                status = impl->implFunctPtrs.
                            icp_sal_userDcGetAvailableNumDynInstances(
                                                      &curNumInstances);
                break;
            default:
                MUX_ERROR("Invalid inst type");
                return CPA_STATUS_FAIL;
            }

            if (CPA_STATUS_SUCCESS != status)
            {  
                return status;
            }

            *pNumInstances += curNumInstances;
        }
    }
    return status;
}

STATIC CpaStatus
cpaMuxGetAvailNumDynInstancesByDevPkg (Cpa32U * pNumInstances,
                                        Cpa32U devPkgID,
                                        CpaMuxInstanceType type)
{
    Cpa32U curNumInstances = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa16U i = 0;

    if (NULL == pNumInstances)
    {
        MUX_ERROR("pNumInstances param cannot be NULL\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    *pNumInstances = 0;

    /**
     * Iterate across all registered implementations 
     */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            switch (type)
            {
            case CPA_MUX_INST_TYPE_CY:
                status = impl->implFunctPtrs.
                            icp_sal_userCyGetAvailableNumDynInstancesByDevPkg(
                                                    &curNumInstances, devPkgID);
                break;
            case CPA_MUX_INST_TYPE_DC:
                status = impl->implFunctPtrs.
                            icp_sal_userDcGetAvailableNumDynInstancesByDevPkg(
                                                    &curNumInstances, devPkgID);
                break;
            default:
                MUX_ERROR("Invalid inst type");
                status = CPA_STATUS_FAIL;
                break;
            }

            if (CPA_STATUS_SUCCESS != status)
            {  
                MUX_ERROR
                   ("Failed to get avail no of insts for a registered impl");
                break;
            }

            *pNumInstances += curNumInstances;
        }
    }
    return status;
}

STATIC CpaStatus
cpaMuxGetAvailNumDynInstancesByPkgAccel(Cpa32U *pNumInstances,
                                        Cpa32U devPkgID,
                                        Cpa32U accelerator_number,
                                        CpaMuxInstanceType type)
{
    Cpa32U curNumInstances = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa16U i = 0;

    if (NULL == pNumInstances)
    {
        MUX_ERROR("pNumInstances param cannot be NULL\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    *pNumInstances = 0;

    /**
     * Iterate across all registered implementations 
     */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            switch (type)
            {
            case CPA_MUX_INST_TYPE_CY:
                status = impl->implFunctPtrs.
                            icp_sal_userCyGetAvailableNumDynInstancesByPkgAccel(
                                &curNumInstances, devPkgID, accelerator_number);
                break;
            default:
                MUX_ERROR("Invalid instance type");
                status = CPA_STATUS_FAIL;
                break;
            }

            if (CPA_STATUS_SUCCESS != status)
            {  
                MUX_ERROR
                   ("Failed to get avail no of insts for a registered impl");
                break;
            }

            *pNumInstances += curNumInstances;
        }
    }
    return status;
}

STATIC CpaStatus
cpaMuxFreeDynInstances (Cpa32U numInstances, CpaInstanceHandle *instances,
                    CpaMuxInstanceType type)
{
    Cpa32U i = 0;
    CpaMuxImpl * impl = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus retStatus = CPA_STATUS_SUCCESS;
    
    if (0 == numInstances)
    {
        MUX_ERROR("numInstances to free is 0");
        return CPA_STATUS_INVALID_PARAM;
    }
    if (NULL == instances)
    {
        MUX_ERROR("instances param to free cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }
    for (i = 0; i < numInstances; i++)
    {
       if (NULL != instances[i])
       {
          impl = ((CpaMuxInstance *)instances[i])->pMuxImpl;
       }
       else
       { 
           impl = NULL;
           MUX_PRINT
             ("Error:API call with invalid insatance handle for inst[%d]\n", i);
       }
       if ((NULL != impl)&&(impl->registered))
       {
          switch (type)
          { 
                case CPA_MUX_INST_TYPE_CY:
                    status = impl->implFunctPtrs.icp_sal_userCyFreeInstances(
                                                              1, &instances[i]);
                    break;
                case CPA_MUX_INST_TYPE_DC:
                    status = impl->implFunctPtrs.icp_sal_userDcFreeInstances(
                                                              1, &instances[i]);
                    break;
                default:
                    MUX_ERROR("Invalid instance type");
                    return CPA_STATUS_FAIL;
           }
           if (CPA_STATUS_SUCCESS != status)
           {
              MUX_PRINT("Failed to Free instances[%d] for registered impl", i);
              retStatus = CPA_STATUS_FAIL;
           }
           impl = NULL;  
       }             
    }
    return retStatus;
}

STATIC CpaStatus
cpaMuxAllocDynInstances (Cpa32U numInstances, CpaInstanceHandle *instances,
                    CpaMuxInstanceType type)
{
    Cpa32U curNumInstances = 0;
    Cpa32U totalNumInstances = 0;
    Cpa32U totalAllocInstances = 0;
    Cpa16U i = 0;
    Cpa16U j = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus retStatus = CPA_STATUS_SUCCESS;
    
    if (0 == numInstances)
    {
        MUX_ERROR("numInstances to alloc is 0");
        return CPA_STATUS_INVALID_PARAM;
    }
    if (NULL == instances)
    {
        MUX_ERROR("instances param cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            switch (type)
            {
            case CPA_MUX_INST_TYPE_CY:
                status = impl->implFunctPtrs.
                            icp_sal_userCyGetAvailableNumDynInstances(
                                                     &curNumInstances);
                break;
            case CPA_MUX_INST_TYPE_DC:
                status = impl->implFunctPtrs.
                            icp_sal_userDcGetAvailableNumDynInstances(
                                                     &curNumInstances);
                break;
            default:
                MUX_ERROR("Invalid instance type");
                return CPA_STATUS_FAIL;
            }
            
            if (CPA_STATUS_SUCCESS != status)
            {
                MUX_ERROR
                   ("Failed to get avail no of insts for a registered impl");
                goto exit;
            }

            /* Make sure we don't overflow the 'instances' array provided */
            if (curNumInstances > (numInstances - totalNumInstances))
            {    
                curNumInstances = numInstances - totalNumInstances;
            }

            if (curNumInstances)
            {
                switch (type)
                {
                case CPA_MUX_INST_TYPE_CY:
                    status = impl->implFunctPtrs.icp_sal_userCyInstancesAlloc(
                        curNumInstances, instances + totalNumInstances);
                    break;
                case CPA_MUX_INST_TYPE_DC:
                    status = impl->implFunctPtrs.icp_sal_userDcInstancesAlloc(
                        curNumInstances, instances + totalNumInstances);
                    break;
                default:
                    MUX_ERROR("Invalid dyn instance type");
                    return CPA_STATUS_FAIL;
                }
                if (CPA_STATUS_SUCCESS != status)
                {
                    MUX_ERROR("Failed to Alloc instances for registered impl");
                    goto exit;
                }

                for (j = 0; j < curNumInstances; j++)
                {
                    CpaMuxInstance * pMuxInst =
                        (CpaMuxInstance *) instances[totalNumInstances + j];

                    if (!pMuxInst)
                    {
                        MUX_ERROR("Failed to insert impl ref");
                        switch (type)
                        {
                           case CPA_MUX_INST_TYPE_CY:
                                status = impl->implFunctPtrs.
                                          icp_sal_userCyFreeInstances(
                                          1,&instances[totalNumInstances + j]);
                           break;
                           case CPA_MUX_INST_TYPE_DC:
                                status = impl->implFunctPtrs.
                                           icp_sal_userDcFreeInstances(
                                           1,&instances[totalNumInstances + j]);
                           break;
                           default:
                                MUX_ERROR("Invalid dyn inst type");
                                return CPA_STATUS_FAIL;
                         }
                         status = CPA_STATUS_INVALID_PARAM;
                         goto exit;
                    }

                    /** Store a reference to the implementation within the
                     * InstanceHandle that we can use later
                     */
                    pMuxInst->pMuxImpl = impl;
                    ++totalAllocInstances;
                }
            }
            totalNumInstances += curNumInstances;
        }
    }

    if (numInstances > totalNumInstances)
    {
        MUX_PRINT("Only %d instances available for dyn alloc (%d requested)\n",
                   totalNumInstances, numInstances);
        status = CPA_STATUS_RESOURCE;
    }

exit:
    if (((CPA_STATUS_SUCCESS != status) 
         &&(CPA_STATUS_RESOURCE != status))
         && (totalAllocInstances > 0))
    {
       retStatus = cpaMuxFreeDynInstances(totalAllocInstances, instances, type);
       if (CPA_STATUS_SUCCESS == retStatus)
       {
          MUX_PRINT("Freed [%d] dynamic alloc instances\n",
                                              totalAllocInstances);
       }         
    }
    return status;
}

STATIC CpaStatus
cpaMuxAllocDynInstancesByDevPkg (Cpa32U numInstances,
                    CpaInstanceHandle *instances,
                    Cpa32U devPkgID,
                    CpaMuxInstanceType type)
{
    Cpa32U curNumInstances = 0;
    Cpa32U totalNumInstances = 0;
    Cpa32U totalAllocInstances = 0;
    Cpa16U i = 0;
    Cpa16U j = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus retStatus = CPA_STATUS_SUCCESS;
    
    if (0 == numInstances)
    {
        MUX_ERROR("numInstances to alloc is 0");
        return CPA_STATUS_INVALID_PARAM;
    }
    if (NULL == instances)
    {
        MUX_ERROR("instances param cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            switch (type)
            {
            case CPA_MUX_INST_TYPE_CY:
                status = impl->implFunctPtrs.
                            icp_sal_userCyGetAvailableNumDynInstancesByDevPkg(
                                                     &curNumInstances,
                                                     devPkgID);
                break;
            case CPA_MUX_INST_TYPE_DC:
                status = impl->implFunctPtrs.
                            icp_sal_userDcGetAvailableNumDynInstancesByDevPkg(
                                                     &curNumInstances,
                                                     devPkgID);
                break;
            default:
                MUX_ERROR("Invalid instance type");
                return CPA_STATUS_FAIL;
            }
            
            if (CPA_STATUS_SUCCESS != status)
            {
                MUX_ERROR
                   ("Failed to get avail no of insts for a registered impl");
                goto exit;
            }

            /* Make sure we don't overflow the 'instances' array provided */
            if (curNumInstances > (numInstances - totalNumInstances))
            {    
                curNumInstances = numInstances - totalNumInstances;
            }

            if (curNumInstances)
            {
                switch (type)
                {
                case CPA_MUX_INST_TYPE_CY:
                    status = impl->implFunctPtrs.
                            icp_sal_userCyInstancesAllocByDevPkg(
                            curNumInstances,
                            instances + totalNumInstances,
                            devPkgID);
                    break;
                case CPA_MUX_INST_TYPE_DC:
                    status = impl->implFunctPtrs.
                            icp_sal_userDcInstancesAllocByDevPkg(
                            curNumInstances,
                            instances + totalNumInstances,
                            devPkgID);
                    break;
                default:
                    MUX_ERROR("Invalid dyn instance type");
                    return CPA_STATUS_FAIL;
                }
                if (CPA_STATUS_SUCCESS != status)
                {
                    MUX_ERROR("Failed to Alloc instances for registered impl");
                    goto exit;
                }
                /* Verify we have correct number of instances
                 * and contiguous in array.Free allocated instance,
                 * when CpaMuxInstance derived from instance array is Null */
                for (j = 0; j < curNumInstances; j++)
                {
                    CpaMuxInstance * pMuxInst =
                        (CpaMuxInstance *) instances[totalNumInstances + j];

                    if (!pMuxInst)
                    {
                        MUX_ERROR("Failed to insert impl ref");
                        switch (type)
                        {
                           case CPA_MUX_INST_TYPE_CY:
                                status = impl->implFunctPtrs.
                                          icp_sal_userCyFreeInstances(
                                          1,&instances[totalNumInstances + j]);
                           break;
                           case CPA_MUX_INST_TYPE_DC:
                                status = impl->implFunctPtrs.
                                           icp_sal_userDcFreeInstances(
                                           1,&instances[totalNumInstances + j]);
                           break;
                           default:
                                MUX_ERROR("Invalid dyn inst type");
                                return CPA_STATUS_FAIL;
                         }
                         status = CPA_STATUS_INVALID_PARAM;
                         goto exit;
                    }

                    /** Store a reference to the implementation within the
                     * InstanceHandle that we can use later
                     */
                    pMuxInst->pMuxImpl = impl;
                    ++totalAllocInstances;
                }
            }
            totalNumInstances += curNumInstances;
        }
    }

    if (numInstances > totalNumInstances)
    {
        MUX_PRINT("Only %d instances available for dyn alloc (%d requested)\n",
                   totalNumInstances, numInstances);
        status = CPA_STATUS_RESOURCE;
    }

exit:
    if (((CPA_STATUS_SUCCESS != status) 
         &&(CPA_STATUS_RESOURCE != status))
         && (totalAllocInstances > 0))
    {
       retStatus = cpaMuxFreeDynInstances(totalAllocInstances, instances, type);
       if (CPA_STATUS_SUCCESS == retStatus)
       {
          MUX_PRINT("Freed [%d] dynamic alloc instances\n",
                                              totalAllocInstances);
       }         
    }
    return status;
}


STATIC CpaStatus
cpaMuxAllocDynInstancesByPkgAccel(Cpa32U numInstances,
                                  CpaInstanceHandle *instances,
                                  Cpa32U devPkgID,
                                  Cpa32U accelerator_number,
                                  CpaMuxInstanceType type)
{
    Cpa32U curNumInstances = 0;
    Cpa32U totalNumInstances = 0;
    Cpa32U totalAllocInstances = 0;
    Cpa16U i = 0;
    Cpa16U j = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus retStatus = CPA_STATUS_SUCCESS;

    if (0 == numInstances)
    {
        MUX_ERROR("numInstances to alloc is 0");
        return CPA_STATUS_INVALID_PARAM;
    }
    if (NULL == instances)
    {
        MUX_ERROR("instances param cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            switch (type)
            {
            case CPA_MUX_INST_TYPE_CY:
                status = impl->implFunctPtrs.
                            icp_sal_userCyGetAvailableNumDynInstancesByPkgAccel(
                                &curNumInstances, devPkgID, accelerator_number);
                break;
            default:
                MUX_ERROR("Invalid instance type");
                return CPA_STATUS_FAIL;
            }

            if (CPA_STATUS_SUCCESS != status)
            {
                MUX_ERROR
                   ("Failed to get avail no of insts for a registered impl");
                goto exit;
            }

            /* Make sure we don't overflow the 'instances' array provided */
            if (curNumInstances > (numInstances - totalNumInstances))
            {
                curNumInstances = numInstances - totalNumInstances;
            }

            if (curNumInstances)
            {
                switch (type)
                {
                case CPA_MUX_INST_TYPE_CY:
                    status = impl->implFunctPtrs.
                            icp_sal_userCyInstancesAllocByPkgAccel(
                            curNumInstances,
                            instances + totalNumInstances,
                            devPkgID, accelerator_number);
                    break;
                default:
                    MUX_ERROR("Invalid dynamic instance type");
                    return CPA_STATUS_FAIL;
                }
                if (CPA_STATUS_SUCCESS != status)
                {
                    MUX_ERROR("Failed to Alloc instances for registered impl");
                    goto exit;
                }
                /* Verify we have correct number of instances
                 * and contiguous in array.Free allocated instance,
                 * when CpaMuxInstance derived from instance array is Null */
                for (j = 0; j < curNumInstances; j++)
                {
                    CpaMuxInstance * pMuxInst =
                        (CpaMuxInstance *) instances[totalNumInstances + j];

                    if (!pMuxInst)
                    {
                        MUX_ERROR("Failed to insert impl ref");
                        switch (type)
                        {
                           case CPA_MUX_INST_TYPE_CY:
                                status = impl->implFunctPtrs.
                                          icp_sal_userCyFreeInstances(
                                          1,&instances[totalNumInstances + j]);
                           break;
                           default:
                                MUX_ERROR("Invalid dynamic instance type");
                                return CPA_STATUS_FAIL;
                         }
                         status = CPA_STATUS_INVALID_PARAM;
                         goto exit;
                    }

                    /** Store a reference to the implementation within the
                     * InstanceHandle that we can use later
                     */
                    pMuxInst->pMuxImpl = impl;
                    ++totalAllocInstances;
                }
            }
            totalNumInstances += curNumInstances;
        }
    }

    if (numInstances > totalNumInstances)
    {
        MUX_PRINT("Only %d instances available for dyn alloc (%d requested)\n",
                   totalNumInstances, numInstances);
        status = CPA_STATUS_RESOURCE;
    }

exit:
    if (((CPA_STATUS_SUCCESS != status)
         &&(CPA_STATUS_RESOURCE != status))
         && (totalAllocInstances > 0))
    {
       retStatus = cpaMuxFreeDynInstances(totalAllocInstances, instances, type);
       if (CPA_STATUS_SUCCESS == retStatus)
       {
          MUX_PRINT("Freed [%d] dynamic alloc instances\n",
                                              totalAllocInstances);
       }
    }
    return status;
}

CpaStatus
icp_sal_userCyGetAvailableNumDynInstances(Cpa32U *pNumCyInstances)
{
    return cpaMuxGetAvailNumDynInstances(pNumCyInstances, CPA_MUX_INST_TYPE_CY);
}

CpaStatus
icp_sal_userDcGetAvailableNumDynInstances(Cpa32U *pNumDcInstances)
{
    return cpaMuxGetAvailNumDynInstances(pNumDcInstances,CPA_MUX_INST_TYPE_DC);
}

CpaStatus 
icp_sal_userCyInstancesAlloc(Cpa32U numCyInstances,
                                       CpaInstanceHandle *pCyInstances)
{
    return cpaMuxAllocDynInstances(numCyInstances, pCyInstances,
                                        CPA_MUX_INST_TYPE_CY);
}

CpaStatus 
icp_sal_userDcInstancesAlloc(Cpa32U numDcInstances,
                                       CpaInstanceHandle *pDcInstances)
{
    return cpaMuxAllocDynInstances(numDcInstances, pDcInstances,
                                    CPA_MUX_INST_TYPE_DC);
}

CpaStatus 
icp_sal_userCyInstancesAllocByDevPkg(Cpa32U numCyInstances,
                                        CpaInstanceHandle *pCyInstances,
                                        Cpa32U devPkgID)
{
    return cpaMuxAllocDynInstancesByDevPkg (numCyInstances,
                            pCyInstances, devPkgID, CPA_MUX_INST_TYPE_CY);
}

CpaStatus 
icp_sal_userDcInstancesAllocByDevPkg(Cpa32U numDcInstances,
                                        CpaInstanceHandle *pDcInstances,
                                        Cpa32U devPkgID)
{
    return cpaMuxAllocDynInstancesByDevPkg (numDcInstances,
                                pDcInstances, devPkgID, CPA_MUX_INST_TYPE_DC);
}

CpaStatus
icp_sal_userCyInstancesAllocByPkgAccel(Cpa32U numCyInstances,
                                       CpaInstanceHandle *pCyInstances,
                                       Cpa32U devPkgID,
                                       Cpa32U accelerator_number)
{
    return cpaMuxAllocDynInstancesByPkgAccel(numCyInstances,
                                 pCyInstances, devPkgID,
                                 accelerator_number, CPA_MUX_INST_TYPE_CY);
}

CpaStatus 
icp_sal_userCyFreeInstances(Cpa32U numCyInstances,
                                      CpaInstanceHandle *pCyInstances)
{
    return cpaMuxFreeDynInstances(numCyInstances, pCyInstances,
                                        CPA_MUX_INST_TYPE_CY);
}

CpaStatus 
icp_sal_userDcFreeInstances(Cpa32U numDcInstances,
                                       CpaInstanceHandle *pDcInstances)
{
    return cpaMuxFreeDynInstances(numDcInstances, pDcInstances,
                                    CPA_MUX_INST_TYPE_DC);
}

CpaStatus
icp_sal_userCyGetAvailableNumDynInstancesByDevPkg(Cpa32U *pNumCyInstances,
                                                            Cpa32U devPkgID)
{
    return cpaMuxGetAvailNumDynInstancesByDevPkg (pNumCyInstances,
                                                    devPkgID,
                                                    CPA_MUX_INST_TYPE_CY);
}

CpaStatus
icp_sal_userDcGetAvailableNumDynInstancesByDevPkg(Cpa32U *pNumCyInstances,
                                                            Cpa32U devPkgID)
{
    return cpaMuxGetAvailNumDynInstancesByDevPkg (pNumCyInstances,
                                                    devPkgID,
                                                    CPA_MUX_INST_TYPE_DC);
}

CpaStatus
icp_sal_userCyGetAvailableNumDynInstancesByPkgAccel(Cpa32U *pNumCyInstances,
                                                    Cpa32U devPkgID,
                                                    Cpa32U accelerator_number)
{
    return cpaMuxGetAvailNumDynInstancesByPkgAccel(pNumCyInstances,
                                                   devPkgID,
                                                   accelerator_number,
                                                   CPA_MUX_INST_TYPE_CY);
}

CpaStatus
icp_sal_check_device (Cpa32U accelId)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.icp_sal_check_device (accelId);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }

}
CpaStatus
icp_sal_check_all_devices (void)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaBoolean validDev = CPA_FALSE;
    Cpa16U i = 0;

    if (0 == cpaMuxNumImpls)
    {
        MUX_ERROR("No implementations have been registered");
        return CPA_STATUS_INVALID_PARAM;
    }

/**
 * Best we can do is loop through all implementations and call
 * check device on each.
 */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];
            
            status = impl->implFunctPtrs.icp_sal_check_all_devices ();
            if (CPA_STATUS_SUCCESS != status)
            {
                MUX_PRINT
                ("WARN: icp_sal_check_all_devices fails for [%d] impl\n", i);
            }
            else
            {
                validDev = CPA_TRUE;
            } 
        }
    }
    return ((validDev == CPA_FALSE) ? CPA_STATUS_FAIL : CPA_STATUS_SUCCESS);
}
CpaStatus 
icp_sal_userGetPfVfcommsStatus(CpaBoolean *unreadMessage)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaBoolean bPfVfCommsStatus = CPA_FALSE;
    CpaBoolean bUnReadMsg = CPA_FALSE;
    Cpa16U i = 0;

    if (0 == cpaMuxNumImpls)
    {
        MUX_ERROR("No implementations have been registered");
        return CPA_STATUS_INVALID_PARAM;
    }
    if (NULL == unreadMessage)
    {
        MUX_ERROR("Invalid input param - unreadMessage");
        return CPA_STATUS_INVALID_PARAM;
    }
/**
 * Best we can do is loop through all implementations and call
 * get Pf/vf comms status on each.
 */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];
            
            status = impl->implFunctPtrs.
                        icp_sal_userGetPfVfcommsStatus (unreadMessage);
            if (CPA_STATUS_SUCCESS != status)
            {
                MUX_PRINT
                ("WARN:icp_sal_userGetPfVfcommsStatus fails for [%d]impl\n",i);
            }
            else
            {
                bPfVfCommsStatus = CPA_TRUE;
                if (bUnReadMsg || (*unreadMessage))
                {
                   bUnReadMsg = *unreadMessage;
                } 
            } 
        }
    }
    if (bUnReadMsg)
    {
        *unreadMessage = bUnReadMsg;
    }
    return ((bPfVfCommsStatus == CPA_FALSE) ?
                CPA_STATUS_FAIL : CPA_STATUS_SUCCESS);
}

CpaStatus 
icp_sal_userGetMsgFromPf(Cpa32U accelId, Cpa32U * message,
                                   Cpa32U *messageCounter)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.
                    icp_sal_userGetMsgFromPf (accelId, message,
                                                messageCounter);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }
}

CpaStatus 
icp_sal_userGetMsgFromVf (Cpa32U accelId, Cpa32U vfNum,
                            Cpa32U *message, Cpa32U *messageCounter)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.
                    icp_sal_userGetMsgFromVf (accelId, vfNum, message,
                                                messageCounter);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }
}

CpaStatus 
icp_sal_userSendMsgToPf(Cpa32U accelId, Cpa32U message)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.
                    icp_sal_userSendMsgToPf (accelId, message);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }
}

CpaStatus 
icp_sal_userSendMsgToVf(Cpa32U accelId, Cpa32U vfNum, Cpa32U message)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.
                    icp_sal_userSendMsgToVf (accelId, vfNum, message);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }
}
CpaStatus 
icp_sal_reset_device(Cpa32U accelId)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.
                    icp_sal_reset_device (accelId);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }
}

CpaStatus 
icp_sal_poll_device_events (void)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaBoolean ret = CPA_FALSE;
    Cpa16U i = 0;

    if (0 == cpaMuxNumImpls)
    {
        MUX_ERROR("No implementations have been registered");
        return CPA_STATUS_INVALID_PARAM;
    }

/**
 * Best we can do is loop through all implementations and call
 * poll device events on each.
 */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];
            
            status = impl->implFunctPtrs.icp_sal_poll_device_events();
            if (CPA_STATUS_SUCCESS != status)
            {
                MUX_PRINT
                ("WARN: icp_sal_poll_device_events fails for [%d] impl\n", i);
            }
            else
            {
                ret = CPA_TRUE;
            } 
        }
    }
    return ((ret == CPA_FALSE) ? CPA_STATUS_FAIL : CPA_STATUS_SUCCESS);
}

CpaStatus 
icp_sal_find_new_devices (void)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaBoolean ret = CPA_FALSE;
    Cpa16U i = 0;

    if (0 == cpaMuxNumImpls)
    {
        MUX_ERROR("No implementations have been registered");
        return CPA_STATUS_INVALID_PARAM;
    }

/**
 * Best we can do is loop through all implementations and call
 * find new devices on each.
 */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];
            
            status = impl->implFunctPtrs.icp_sal_find_new_devices();
            if (CPA_STATUS_SUCCESS != status)
            {
                MUX_PRINT
                ("WARN: icp_sal_find_new_devices fails for [%d] impl\n", i);
            }
            else
            {
                ret = CPA_TRUE;
            } 
        }
    }
    return ((ret == CPA_FALSE) ? CPA_STATUS_FAIL : CPA_STATUS_SUCCESS);
}

/**
 * icp_sal_CyGetFileDescriptor:
 */
CpaStatus icp_sal_CyGetFileDescriptor(CpaInstanceHandle instanceHandle_in, int *fd)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_CyGetFileDescriptor, instanceHandle_in,
                         instanceHandle_in, fd);
}

/**
 * icp_sal_CyPutFileDescriptor:
 */
CpaStatus icp_sal_CyPutFileDescriptor(CpaInstanceHandle instanceHandle_in, int fd)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_CyPutFileDescriptor, instanceHandle_in,
                         instanceHandle_in, fd);
}

/**
 * icp_sal_DcGetFileDescriptor:
 */
CpaStatus icp_sal_DcGetFileDescriptor(CpaInstanceHandle instanceHandle_in, int *fd)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_DcGetFileDescriptor, instanceHandle_in,
                         instanceHandle_in, fd);
}

/**
 * icp_sal_DcPutFileDescriptor:
 */
CpaStatus icp_sal_DcPutFileDescriptor(CpaInstanceHandle instanceHandle_in, int fd)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_DcPutFileDescriptor, instanceHandle_in,
                         instanceHandle_in, fd);
}
#endif

/**
 * icp_sal_getDevVersionInfo:
 */
CpaStatus
icp_sal_getDevVersionInfo (Cpa32U accelId,
                           icp_sal_dev_version_info_t * pVerInfo)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.icp_sal_getDevVersionInfo (accelId,
                                                              pVerInfo);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }
}

/**
 * icp_sal_drbgGetEntropyInputFuncRegister:
 */
IcpSalDrbgGetEntropyInputFunc
icp_sal_drbgGetEntropyInputFuncRegister (IcpSalDrbgGetEntropyInputFunc func)
{
    IcpSalDrbgGetEntropyInputFunc res = NULL;
    Cpa16U i = 0;

    /**
     * Best we can do is loop through all implementations and call each one,
     * but only return the result of the last one.  That should be ok provided
     * all implemetations are registered before making any calls to this
     * function.
     */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            res = impl->implFunctPtrs.
                icp_sal_drbgGetEntropyInputFuncRegister(func);
        }
    }

    return res;
}

/**
 * icp_sal_drbgGetNonceFuncRegister:
 */
IcpSalDrbgGetNonceFunc
icp_sal_drbgGetNonceFuncRegister (IcpSalDrbgGetNonceFunc func)
{
    IcpSalDrbgGetNonceFunc res = NULL;
    Cpa16U i = 0;

    /**
     * Best we can do is loop through all implementations and call each one,
     * but only return the result of the last one.  That should be ok provided
     * all implemetations are registered before making any calls to this
     * function.
     */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];

            res = impl->implFunctPtrs.icp_sal_drbgGetNonceFuncRegister(func);
        }
    }

    return res;
}

/**
 * icp_sal_drbgIsDFReqFuncRegister:
 */
IcpSalDrbgIsDFReqFunc
icp_sal_drbgIsDFReqFuncRegister (IcpSalDrbgIsDFReqFunc func)
{
    IcpSalDrbgIsDFReqFunc res = NULL;
    Cpa16U i = 0;

    /**
     * Best we can do is loop through all implementations and call each one,
     * but only return the result of the last one.  That should be ok provided
     * all implemetations are registered before making any calls to this
     * function.
     */
    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];
            
            res = impl->implFunctPtrs.icp_sal_drbgIsDFReqFuncRegister(func);
        }
    }

    return res;
}

#ifdef ONE_KO_RELEASE_PACKAGE
/**
 * icp_sal_pollBank:
 */
CpaStatus
icp_sal_pollBank (Cpa32U accelId, Cpa32U bank_number, Cpa32U response_quota)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.icp_sal_pollBank (accelId, bank_number,
                                                     response_quota);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }
}

/**
 * icp_sal_pollAllBanks:
 */
CpaStatus
icp_sal_pollAllBanks (Cpa32U accelId, Cpa32U response_quota)
{
    CpaMuxImpl *impl = NULL;

    impl = cpaMuxAccelIdToImpl(&accelId);
    if (NULL != impl)
    {
        return impl->implFunctPtrs.icp_sal_pollAllBanks (accelId,
                                                         response_quota);
    }
    else
    {
        MUX_ERROR("Invalid accelId parameter specified");
        return CPA_STATUS_FAIL;
    }
}
#endif

static CpaStatus
__cpaCyDrbgSessionGetSize (const CpaInstanceHandle instanceHandle,
                           const CpaCyDrbgSessionSetupData * pSetupData,
                           Cpa32U * pSize)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDrbgSessionGetSize,
                         instanceHandle, instanceHandle,
                         pSetupData, pSize);
}

static CpaStatus
__cpaCyDrbgInitSession (const CpaInstanceHandle instanceHandle,
                        const CpaCyGenFlatBufCbFunc pGenCb,
                        const CpaCyGenericCbFunc pReseedCb,
                        const CpaCyDrbgSessionSetupData * pSetupData,
                        CpaCyDrbgSessionHandle sessionHandle,Cpa32U * pSeedLen)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDrbgInitSession,
                         instanceHandle, instanceHandle, pGenCb,
                         pReseedCb, pSetupData, sessionHandle, pSeedLen);
}

CpaStatus
__cpaCyDrbgRemoveSession (const CpaInstanceHandle instanceHandle,
                          CpaCyDrbgSessionHandle sessionHandle)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDrbgRemoveSession,
                         instanceHandle, instanceHandle, sessionHandle);
}

static inline CpaMuxDrbgSessionInfo *
cpaMuxDrbgSessionFind(CpaCyDrbgSessionHandle sessionHandle)
{
    Cpa16U i=0;
    
    if (NULL == sessionHandle)
    {
       return NULL;
    }

    for (i = 0; i < MAX_NUM_IMPLS; i++)
    {
        if (cpaMuxImpls[i].registered)
        {
            CpaMuxImpl *impl = &cpaMuxImpls[i];
            ListEntry *l = &(impl->drbgSessions);
            ListEntry *p = l->next;

            while (p != l)
            {
                if(((CpaMuxDrbgSessionInfo *)p)->sessionHandle == sessionHandle)
                {
                    return (CpaMuxDrbgSessionInfo *)p;
                }
                
                p = p->next;
            }
        }
    }

    return NULL;
}

static CpaStatus
cpaMuxDrbgSessionRemove(CpaCyDrbgSessionHandle sessionHandle)
{
    CpaMuxDrbgSessionInfo *pSessionInfo = NULL;
     
    pSessionInfo = cpaMuxDrbgSessionFind(sessionHandle);
    if (NULL == pSessionInfo)
    { 
        return CPA_STATUS_FAIL;
    }

    /** No need to free the memory for pSessionInfo, because
     *  that's owned by the user application
     */
    LIST_REMOVE((ListEntry *)pSessionInfo);

    return CPA_STATUS_SUCCESS;
}

static CpaStatus
cpaMuxDrbgSessionAdd(const CpaInstanceHandle instanceHandle,
                     const CpaCyDrbgSessionSetupData * pSetupData,
                     CpaCyDrbgSessionHandle sessionHandle)
{
    CpaMuxImpl *impl = cpaMuxGetImpl(instanceHandle);
    CpaMuxDrbgSessionInfo *pMuxSessionInfo = NULL;
    Cpa32U size = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;

    if (NULL == sessionHandle)
    {
        MUX_ERROR("sessionHandle cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }
    
    if (NULL == impl)
    {
        MUX_ERROR("Invalid instance handle");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Make sure it doesn't already exist, remove it otherwise */
    cpaMuxDrbgSessionRemove(sessionHandle);
    
    status = __cpaCyDrbgSessionGetSize (instanceHandle,
                                        pSetupData,
                                        &size);
    if (CPA_STATUS_SUCCESS != status)
    {
        MUX_ERROR("Failed to get DRBG session size");
        return status;
    }

    /* We will "borrow" some memory from the allocated session
     * descriptor to store our sessionInfo.  That way, we avoid
     * needing to do a blocking alloc, which isn't allowed for
     * the cpaCyDrbgInitSession API
     *
     * We bumped up the memory size returned by 
     * cpaCyDrbgSessionGetSize() to make space for it.
     */
    pMuxSessionInfo =
        (CpaMuxDrbgSessionInfo *)((Cpa8U *)sessionHandle + size);
    pMuxSessionInfo->sessionHandle = sessionHandle;
    pMuxSessionInfo->instanceHandle = instanceHandle;
    
    LIST_INSERT_AFTER((ListEntry *)pMuxSessionInfo, impl->drbgSessions.prev);

    return CPA_STATUS_SUCCESS;
}

/**
 * cpaCyDrbgInitSession:
 */
CpaStatus
cpaCyDrbgInitSession (const CpaInstanceHandle instanceHandle,
                      const CpaCyGenFlatBufCbFunc pGenCb,
                      const CpaCyGenericCbFunc pReseedCb,
                      const CpaCyDrbgSessionSetupData * pSetupData,
                      CpaCyDrbgSessionHandle sessionHandle, Cpa32U * pSeedLen)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    if (NULL == sessionHandle)
    {
        MUX_ERROR("sessionHandle cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }

    if (NULL != instanceHandle)
    {
        /* We add this sessionHandle and instanceHandle to an internal list
         * in the Mux layer so that we can implement icp_sal_drbgGetInstance()
         * But this function appears to be called from cpaCyDrbgInitSession
         * as well so we need to add it to our list first
         */

        status = cpaMuxDrbgSessionAdd(instanceHandle, pSetupData,sessionHandle);
        if (CPA_STATUS_SUCCESS != status)
        {
            MUX_ERROR("Failed to add DRBG session");
            return status;
        }
    }

    status = __cpaCyDrbgInitSession (instanceHandle, pGenCb, pReseedCb,
                                     pSetupData, sessionHandle, pSeedLen);
    if (CPA_STATUS_SUCCESS != status)
    {
        cpaMuxDrbgSessionRemove(sessionHandle);
    }

    return status;
}

/**
 * cpaCyDrbgSessionGetSize:
 */
CpaStatus
cpaCyDrbgSessionGetSize (const CpaInstanceHandle instanceHandle,
                         const CpaCyDrbgSessionSetupData * pSetupData,
                         Cpa32U * pSize)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    if (NULL == pSize)
    {
        MUX_ERROR("pSize parameter cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }
    
    status = __cpaCyDrbgSessionGetSize(instanceHandle, pSetupData, pSize);

    /* Add some extra space for the CPA-Mux to store meta-data needed later */
    if (CPA_STATUS_SUCCESS == status)
    {
        *pSize += sizeof (CpaMuxDrbgSessionInfo);
    }

    return status;
}

/**
 * cpaCyDrbgRemoveSession:
 */
CpaStatus
cpaCyDrbgRemoveSession (const CpaInstanceHandle instanceHandle,
                        CpaCyDrbgSessionHandle sessionHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    status = __cpaCyDrbgRemoveSession(instanceHandle, sessionHandle);
    if (CPA_STATUS_SUCCESS == status)
    { 
        cpaMuxDrbgSessionRemove(sessionHandle);
    }

    return status;
}

/**
 * icp_sal_drbgGetInstance:
 */
void icp_sal_drbgGetInstance (CpaCyDrbgSessionHandle sessionHandle,
                              CpaInstanceHandle **pDrbgInstance)
{
    CpaMuxDrbgSessionInfo *pSessionInfo = NULL;
    if (NULL == pDrbgInstance)
    {
       return;
    } 
    pSessionInfo = cpaMuxDrbgSessionFind(sessionHandle);
    if (NULL == pSessionInfo)
    {
        *pDrbgInstance = NULL;
        return;
    }
    
    *pDrbgInstance = pSessionInfo->instanceHandle;
}
