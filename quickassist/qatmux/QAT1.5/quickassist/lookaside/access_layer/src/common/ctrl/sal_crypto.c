/*****************************************************************************
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

/**
 ***************************************************************************
 * @file sal_crypto.c     Instance handling functions for crypto
 *
 * @ingroup SalCtrl
 *
 ***************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/

/* QAT-API includes */
#include "cpa.h"
#include "cpa_types.h"
#include "cpa_cy_common.h"
#include "cpa_cy_im.h"
#include "cpa_cy_drbg.h"
#include "cpa_cy_ln.h"
#include "cpa_cy_dh.h"
#include "cpa_cy_dsa.h"
#include "cpa_cy_rsa.h"
#include "cpa_cy_ec.h"
#include "cpa_cy_ecdh.h"
#include "cpa_cy_ecdsa.h"
#include "cpa_cy_prime.h"
#include "cpa_cy_key.h"
#include "cpa_cy_sym.h"

/* Osal includes */
#include "Osal.h"

/* ADF includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_cfg.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_poll.h"
#include "icp_adf_debug.h"

/* SAL includes */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "sal_statistics.h"
#include "lac_common.h"
#include "lac_list.h"
#include "lac_hooks.h"
#include "lac_sym_qat_hash_defs_lookup.h"
#include "lac_sym.h"
#include "lac_sym_key.h"
#include "lac_sym_hash.h"
#include "lac_sym_cb.h"
#include "lac_sym_stats.h"
#include "lac_pke_utils.h"
#include "lac_pke_qat_comms.h"
#include "lac_ec.h"
#include "lac_sal_types_crypto.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"
#include "sal_string_parse.h"
#include "sal_service_state.h"
#include "icp_sal_poll.h"
#include "icp_sal_drbg_impl.h"
#include "icp_sal_drbg_ht.h"
#include "lac_sym_drbg.h"
#include "lac_sym_drbg_ht.h"
#include "lac_sync.h"

#define HMAC_MODE_1     1
#define HMAC_MODE_2     2
#define NUM_CRYPTO_RX_RINGS 3
#define TH_SYM_RX_HI    0
#define TH_SYM_RX_LO    1
#define TH_ASYM_RX      2
#define DOUBLE_INCR     2

/* Function to release the handles. */
STATIC CpaStatus
SalCtrl_ReleaseTransHandle(sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus ret_status = CPA_STATUS_SUCCESS;
    sal_crypto_service_t* pCryptoService = (sal_crypto_service_t*)service;

    if(NULL != pCryptoService->trans_handle_asym_tx)
    {
       status = icp_adf_transReleaseHandle(
                      pCryptoService->trans_handle_asym_tx);
       if(CPA_STATUS_SUCCESS != status)
       {
          ret_status = status;
       }
    }

    if(NULL != pCryptoService->trans_handle_asym_rx)
    {
       status = icp_adf_transReleaseHandle(
                      pCryptoService->trans_handle_asym_rx);
       if(CPA_STATUS_SUCCESS != status)
       {
          ret_status = status;
       }
    }
    if(NULL != pCryptoService->trans_handle_sym_rx_hi)
    {
       status = icp_adf_transReleaseHandle(
                      pCryptoService->trans_handle_sym_rx_hi);
       if(CPA_STATUS_SUCCESS != status)
       {
          ret_status = status;
       }
    }
    if(NULL != pCryptoService->trans_handle_sym_rx_lo)
    {
       status = icp_adf_transReleaseHandle(
                      pCryptoService->trans_handle_sym_rx_lo);
       if(CPA_STATUS_SUCCESS != status)
       {
          ret_status = status;
       }
    }
    if(NULL != pCryptoService->trans_handle_sym_tx_hi)
    {
       status = icp_adf_transReleaseHandle(
                      pCryptoService->trans_handle_sym_tx_hi);
       if(CPA_STATUS_SUCCESS != status)
       {
          ret_status = status;
       }
    }
    if(NULL != pCryptoService->trans_handle_sym_tx_lo)
    {
       status = icp_adf_transReleaseHandle(
                      pCryptoService->trans_handle_sym_tx_lo);
       if(CPA_STATUS_SUCCESS != status)
       {
          ret_status = status;
       }
    }
    return ret_status;
}


/*
 * @ingroup sal_crypto
 *     Frees resources (memory and transhandles) if allocated
 *
 * @param[in]  pCryptoService       Pointer to crypto service instance
 * @retval                          _SUCCESS if transhandles resleased
 *                                  successfully
*/
STATIC CpaStatus
SalCtrl_CryptoFreeResources(sal_crypto_service_t* pCryptoService)
{

    CpaStatus status = CPA_STATUS_SUCCESS;

    /* 1. Free memory pools if not NULL */
    Lac_MemPoolDestroy(pCryptoService->lac_sym_cookie_pool);
    Lac_MemPoolDestroy(pCryptoService->lac_pke_align_pool);
    Lac_MemPoolDestroy(pCryptoService->lac_pke_req_pool);
    Lac_MemPoolDestroy(pCryptoService->lac_ec_pool);
    Lac_MemPoolDestroy(pCryptoService->lac_prime_pool);

    /* 2. Free misc memory if allocated */
    /* Frees memory allocated for Hmac precomputes */
    LacSymHash_HmacPrecompShutdown(pCryptoService);
    /* Frees memory allocated for key labels
       Also clears key stats  */
    LacSymKey_Shutdown(pCryptoService);
    /* Free hash lookup table if allocated */
    if(NULL != pCryptoService->pLacHashLookupDefs)
    {
        LAC_OS_FREE(pCryptoService->pLacHashLookupDefs);
    }

    /* 3. Free transport handles */
    status = SalCtrl_ReleaseTransHandle((sal_service_t*)pCryptoService);
    LAC_CHECK_STATUS(status);

    /* 4. Free statistics */
    LacSym_StatsFree(pCryptoService);
    LacSymKey_Shutdown(pCryptoService);
    LacDh_StatsFree(pCryptoService);
    LacDsa_StatsFree(pCryptoService);
    LacRsa_StatsFree(pCryptoService);
    LacEc_StatsFree(pCryptoService);
    LacPrime_StatsFree(pCryptoService);
    LacLn_StatsFree(pCryptoService);
    LacDrbg_StatsFree(pCryptoService);

    return status;

}

/**
 ***********************************************************************
 * @ingroup SalCtrl
 *   This macro verifies that the status is _SUCCESS
 *   If status is not _SUCCESS then Crypto Instance resources are
 *   freed before the function returns the error
 *
 * @param[in] status    status we are checking
 *
 * @return void         status is ok (CPA_STATUS_SUCCESS)
 * @return status       The value in the status paramater is an error one
 *
 ******************************************************************************/
#define LAC_CHECK_STATUS_CY_INIT(status)               \
do {                                                   \
    if(CPA_STATUS_SUCCESS != status){                  \
        SalCtrl_CryptoFreeResources(pCryptoService);   \
        return status;                                 \
    }                                                  \
} while(0)


/* Function that creates the Crypto Handles. */
STATIC CpaStatus
SalCtrl_CreateTransHandle(icp_accel_dev_t* device, sal_service_t* service,
                          Cpa32U numSymRequests, Cpa32U numAsymRequests,
                          Cpa32U executionEngine)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char temp_string[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    sal_crypto_service_t* pCryptoService = (sal_crypto_service_t*)service;
    icp_resp_deliv_method rx_resp_type = ICP_RESP_TYPE_IRQ;
    Cpa32U sym_hi_ring_type = 0, sym_lo_ring_type = 0,
           asym_ring_type = 0;
    Cpa32U ring_number_tx = 0;
    Cpa32U ring_number_rx = 0;
    Cpa32U ring_offset = 0;

    char * section = DYN_SEC;

    if(executionEngine)
    {
        sym_hi_ring_type = SAL_RING_TYPE_B_SYM_HI;
        sym_lo_ring_type = SAL_RING_TYPE_B_SYM_LO;
        asym_ring_type = SAL_RING_TYPE_B_ASYM;
    }
    else
    {
        sym_hi_ring_type = SAL_RING_TYPE_A_SYM_HI;
        sym_lo_ring_type = SAL_RING_TYPE_A_SYM_LO;
        asym_ring_type = SAL_RING_TYPE_A_ASYM;
    }

    if(SAL_RESP_POLL_CFG_FILE == pCryptoService->isPolled)
    {
         rx_resp_type = ICP_RESP_TYPE_POLL;
    }

    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "RingSymTxHi", temp_string);
    LAC_CHECK_STATUS(status);

    if(CPA_FALSE == pCryptoService->generic_service_info.is_dyn)
    {
        section = icpGetProcessName();
    }

    status = icp_adf_transCreateHandle(device,
                                ICP_TRANS_TYPE_ETR,
                                section,
                                pCryptoService->acceleratorNum,
                                pCryptoService->bankNum,
                                temp_string,
                                lac_getRingType(sym_hi_ring_type),
                                NULL,
                                ICP_RESP_TYPE_NONE,
                                numSymRequests,
                                (icp_comms_trans_handle *)
                                 &(pCryptoService->trans_handle_sym_tx_hi));
    LAC_CHECK_STATUS(status);
    if(icp_adf_transGetRingNum(pCryptoService->trans_handle_sym_tx_hi,
                               &ring_number_tx) != CPA_STATUS_SUCCESS ||
                               (ring_number_tx & 0x1))
    {
        LAC_LOG_ERROR("Invalid ring number. Tx ring number has to be even\n");
        status = CPA_STATUS_FAIL;
    }
    LAC_CHECK_STATUS_CY_INIT(status);

    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "RingSymTxLo", temp_string);
    /* Need to free resources in case not _SUCCESS from here */
    LAC_CHECK_STATUS_CY_INIT(status);

    status = icp_adf_transCreateHandle(device,
                                ICP_TRANS_TYPE_ETR,
                                section,
                                pCryptoService->acceleratorNum,
                                pCryptoService->bankNum,
                                temp_string,
                                lac_getRingType(sym_lo_ring_type),
                                NULL,
                                ICP_RESP_TYPE_NONE,
                                numSymRequests,
                                (icp_comms_trans_handle *)
                                &(pCryptoService->trans_handle_sym_tx_lo));
    LAC_CHECK_STATUS_CY_INIT(status);
    if(icp_adf_transGetRingNum(pCryptoService->trans_handle_sym_tx_lo,
                               &ring_number_tx) != CPA_STATUS_SUCCESS ||
                               (ring_number_tx & 0x1))
    {
        LAC_LOG_ERROR("Invalid ring number. Tx ring number has to be even\n");
        status = CPA_STATUS_FAIL;
    }

    LAC_CHECK_STATUS_CY_INIT(status);
    /* Sym Rx handles are created in LacSymQat_Init() function below */
    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "RingAsymTx",
                    temp_string);
    LAC_CHECK_STATUS_CY_INIT(status);

    status = icp_adf_transCreateHandle(device,
                                  ICP_TRANS_TYPE_ETR,
                                  section,
                                  pCryptoService->acceleratorNum,
                                  pCryptoService->bankNum,
                                  temp_string,
                                  lac_getRingType(asym_ring_type),
                                  NULL,
                                  ICP_RESP_TYPE_NONE,
                                  numAsymRequests,
                                  (icp_comms_trans_handle *)
                                  &(pCryptoService->trans_handle_asym_tx));
    LAC_CHECK_STATUS_CY_INIT(status);
    if(icp_adf_transGetRingNum(pCryptoService->trans_handle_asym_tx,
                               &ring_number_tx) != CPA_STATUS_SUCCESS ||
                               (ring_number_tx & 0x1))
    {
        LAC_LOG_ERROR("Invalid ring number. Tx ring number has to be even\n");
        status = CPA_STATUS_FAIL;
    }
    LAC_CHECK_STATUS_CY_INIT(status);

    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "RingAsymRx", temp_string);
    LAC_CHECK_STATUS_CY_INIT(status);

    status = icp_adf_transCreateHandle(device,
                                   ICP_TRANS_TYPE_ETR,
                                   section,
                                   pCryptoService->acceleratorNum,
                                   pCryptoService->bankNum,
                                   temp_string,
                                   lac_getRingType(SAL_RING_TYPE_NONE),
                                   LacPke_MsgCallback,
                                   rx_resp_type,
                                   numAsymRequests,
                                   (icp_comms_trans_handle *)
                                   &(pCryptoService->trans_handle_asym_rx));
    LAC_CHECK_STATUS_CY_INIT(status);
    if(icp_adf_transGetRingNum(pCryptoService->trans_handle_asym_rx,
                               &ring_number_rx) != CPA_STATUS_SUCCESS ||
                               (!(ring_number_rx & 0x1)) ||
                               (ring_number_tx + 1 != ring_number_rx))
    {
        LAC_LOG_ERROR("Invalid ring number. Rx ring number has to be odd\n");
        LAC_LOG_ERROR("Rx ring number has to be Tx + 1");
        status = CPA_STATUS_FAIL;
    }
    LAC_CHECK_STATUS_CY_INIT(status);
    pCryptoService->asymResponseRingId = ring_number_tx;
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_VF_RING_OFFSET_KEY,
                                      temp_string);
    LAC_CHECK_STATUS_CY_INIT(status);
    ring_offset = Sal_Strtoul(temp_string, NULL, SAL_CFG_BASE_DEC);
    pCryptoService->asymResponseRingId += ring_offset;
    return status;
}

STATIC int SalCtrl_CryptoDebug(void* private_data,
                           char* data, int size, int offset)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U len = 0;
    sal_crypto_service_t* pCryptoService = (sal_crypto_service_t*)private_data;

    switch (offset)
    {
        case SAL_STATS_SYM:
        {
            CpaCySymStats64 symStats = {0};
            if(CPA_TRUE !=
                  pCryptoService->generic_service_info.stats->bSymStatsEnabled)
            {
                break;
            }
            status = cpaCySymQueryStats64(pCryptoService, &symStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCySymQueryStats64 returned error\n");
                return 0;
            }

            /* Engine Info */
            len += snprintf(data+len, size-len,
               SEPARATOR
               BORDER " Statistics for Instance %24s |\n"
               BORDER " Symmetric Stats                                  "
               BORDER "\n"
               SEPARATOR, pCryptoService->debug_file->name);

            /* Session Info */
            len += snprintf(data+len, size-len,
               BORDER " Sessions Initialized:           %16llu " BORDER "\n"
               BORDER " Sessions Removed:               %16llu " BORDER "\n"
               BORDER " Session Errors:                 %16llu " BORDER "\n"
               SEPARATOR,
               (long long unsigned int)symStats.numSessionsInitialized,
               (long long unsigned int)symStats.numSessionsRemoved,
               (long long unsigned int)symStats.numSessionErrors);

            /* Session info */
            len += snprintf(data+len, size-len,
               BORDER " Symmetric Requests:             %16llu " BORDER "\n"
               BORDER " Symmetric Request Errors:       %16llu " BORDER "\n"
               BORDER " Symmetric Completed:            %16llu " BORDER "\n"
               BORDER " Symmetric Completed Errors:     %16llu " BORDER "\n"
               BORDER " Symmetric Verify Failures:      %16llu " BORDER "\n",
               (long long unsigned int)symStats.numSymOpRequests,
               (long long unsigned int)symStats.numSymOpRequestErrors,
               (long long unsigned int)symStats.numSymOpCompleted,
               (long long unsigned int)symStats.numSymOpCompletedErrors,
               (long long unsigned int)symStats.numSymOpVerifyFailures);
            break;
        }
        case SAL_STATS_DSA:
        {
             CpaCyDsaStats64 dsaStats = {0};
            if(CPA_TRUE !=
                  pCryptoService->generic_service_info.stats->bDsaStatsEnabled)
            {
                ++offset;
                break;
            }

             status = cpaCyDsaQueryStats64(pCryptoService, &dsaStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyDsaQueryStats4 returned error\n");
                return 0;
            }
            /* engine info */
            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " DSA Stats                                        "
                BORDER "\n"
                SEPARATOR);

            /* p parameter generation requests */
            len += snprintf(data+len, size-len,
            BORDER " DSA P Param Gen Requests-Succ:  %16llu " BORDER "\n"
            BORDER " DSA P Param Gen Requests-Err:   %16llu " BORDER "\n"
            BORDER " DSA P Param Gen Completed-Succ: %16llu " BORDER "\n"
            BORDER " DSA P Param Gen Completed-Err:  %16llu " BORDER "\n"
            SEPARATOR,
            (long long unsigned int)dsaStats.numDsaPParamGenRequests,
            (long long unsigned int)dsaStats.numDsaPParamGenRequestErrors,
            (long long unsigned int)dsaStats.numDsaPParamGenCompleted,
            (long long unsigned int)dsaStats.numDsaPParamGenCompletedErrors);

            /* g parameter generation requests */
            len += snprintf(data+len, size-len,
            BORDER " DSA G Param Gen Requests-Succ:  %16llu " BORDER "\n"
            BORDER " DSA G Param Gen Requests-Err:   %16llu " BORDER "\n"
            BORDER " DSA G Param Gen Completed-Succ: %16llu " BORDER "\n"
            BORDER " DSA G Param Gen Completed-Err:  %16llu " BORDER "\n"
            SEPARATOR,
            (long long unsigned int)dsaStats.numDsaGParamGenRequests,
            (long long unsigned int)dsaStats.numDsaGParamGenRequestErrors,
            (long long unsigned int)dsaStats.numDsaGParamGenCompleted,
            (long long unsigned int)dsaStats.numDsaGParamGenCompletedErrors);

            /* y parameter generation requests */
            len += snprintf(data+len, size-len,
            BORDER " DSA Y Param Gen Requests-Succ:  %16llu " BORDER "\n"
            BORDER " DSA Y Param Gen Requests-Err:   %16llu " BORDER "\n"
            BORDER " DSA Y Param Gen Completed-Succ: %16llu " BORDER "\n"
            BORDER " DSA Y Param Gen Completed-Err:  %16llu " BORDER "\n"
            SEPARATOR,
            (long long unsigned int)dsaStats.numDsaYParamGenRequests,
            (long long unsigned int)dsaStats.numDsaYParamGenRequestErrors,
            (long long unsigned int)dsaStats.numDsaYParamGenCompleted,
            (long long unsigned int)dsaStats.numDsaYParamGenCompletedErrors);
            break;
        }
        case SAL_STATS_DSA2:
        {
             CpaCyDsaStats64 dsaStats = {0};
             status = cpaCyDsaQueryStats64(pCryptoService, &dsaStats);
             if(status != CPA_STATUS_SUCCESS)
             {
                 LAC_LOG_ERROR("cpaCyDsaQueryStats4 returned error\n");
                 return 0;
             }
            /* r sign requests */
            len += snprintf(data+len, size-len,
            BORDER " DSA R Sign Requests-Succ:       %16llu " BORDER "\n"
            BORDER " DSA R Sign Request-Err:         %16llu " BORDER "\n"
            BORDER " DSA R Sign Completed-Succ:      %16llu " BORDER "\n"
            BORDER " DSA R Sign Completed-Err:       %16llu " BORDER "\n"
            SEPARATOR,
            (long long unsigned int)dsaStats.numDsaRSignRequests,
            (long long unsigned int)dsaStats.numDsaRSignRequestErrors,
            (long long unsigned int)dsaStats.numDsaRSignCompleted,
            (long long unsigned int)dsaStats.numDsaRSignCompletedErrors);

            /* s sign requests */
            len += snprintf(data+len, size-len,
            BORDER " DSA S Sign Requests-Succ:       %16llu " BORDER "\n"
            BORDER " DSA S Sign Request-Err:         %16llu " BORDER "\n"
            BORDER " DSA S Sign Completed-Succ:      %16llu " BORDER "\n"
            BORDER " DSA S Sign Completed-Err:       %16llu " BORDER "\n"
            SEPARATOR,
            (long long unsigned int)dsaStats.numDsaSSignRequests,
            (long long unsigned int)dsaStats.numDsaSSignRequestErrors,
            (long long unsigned int)dsaStats.numDsaSSignCompleted,
            (long long unsigned int)dsaStats.numDsaSSignCompletedErrors);

            /* rs sign requests */
            len += snprintf(data+len, size-len,
            BORDER " DSA RS Sign Requests-Succ:      %16llu " BORDER "\n"
            BORDER " DSA RS Sign Request-Err:        %16llu " BORDER "\n"
            BORDER " DSA RS Sign Completed-Succ:     %16llu " BORDER "\n"
            BORDER " DSA RS Sign Completed-Err:      %16llu " BORDER "\n"
            SEPARATOR,
            (long long unsigned int)dsaStats.numDsaRSSignRequests,
            (long long unsigned int)dsaStats.numDsaRSSignRequestErrors,
            (long long unsigned int)dsaStats.numDsaRSSignCompleted,
            (long long unsigned int)dsaStats.numDsaRSSignCompletedErrors);

            /* verify requests */
            len += snprintf(data+len, size-len,
            BORDER " DSA Verify Requests-Succ:       %16llu " BORDER "\n"
            BORDER " DSA Verify Request-Err:         %16llu " BORDER "\n"
            BORDER " DSA Verify Completed-Succ:      %16llu " BORDER "\n"
            BORDER " DSA Verify Completed-Err:       %16llu " BORDER "\n"
            BORDER " DSA Verify Completed-Failure:   %16llu " BORDER "\n",
            (long long unsigned int)dsaStats.numDsaVerifyRequests,
            (long long unsigned int)dsaStats.numDsaVerifyRequestErrors,
            (long long unsigned int)dsaStats.numDsaVerifyCompleted,
            (long long unsigned int)dsaStats.numDsaVerifyCompletedErrors,
            (long long unsigned int)dsaStats.numDsaVerifyFailures);
            break;
        }
        case SAL_STATS_RSA:
        {
            CpaCyRsaStats64 rsaStats = {0};
            if(CPA_TRUE !=
                  pCryptoService->generic_service_info.stats->bRsaStatsEnabled)
            {
                break;
            }

            status = cpaCyRsaQueryStats64(pCryptoService, &rsaStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyRsaQueryStats64 returned error\n");
                return 0;
            }

            /* Engine Info */
            len += snprintf(data+len, size-len,
            SEPARATOR
            BORDER " RSA Stats                                        "
            BORDER "\n"
            SEPARATOR);

            /*rsa keygen Info*/
            len += snprintf(data+len, size-len,
            BORDER " RSA Key Gen Requests:           %16llu " BORDER "\n"
            BORDER " RSA Key Gen Request Errors      %16llu " BORDER "\n"
            BORDER " RSA Key Gen Completed:          %16llu " BORDER "\n"
            BORDER " RSA Key Gen Completed Errors:   %16llu " BORDER "\n"
            SEPARATOR,
            (long long unsigned int)rsaStats.numRsaKeyGenRequests,
            (long long unsigned int)rsaStats.numRsaKeyGenRequestErrors,
            (long long unsigned int)rsaStats.numRsaKeyGenCompleted,
            (long long unsigned int)rsaStats.numRsaKeyGenCompletedErrors);

            /*rsa enc Info*/
            len += snprintf(data+len, size-len,
            BORDER " RSA Encrypt Requests:           %16llu " BORDER "\n"
            BORDER " RSA Encrypt Request Errors:     %16llu " BORDER "\n"
            BORDER " RSA Encrypt Completed:          %16llu " BORDER "\n"
            BORDER " RSA Encrypt Completed Errors:   %16llu " BORDER "\n"
            SEPARATOR,
            (long long unsigned int)rsaStats.numRsaEncryptRequests,
            (long long unsigned int)rsaStats.numRsaEncryptRequestErrors,
            (long long unsigned int)rsaStats.numRsaEncryptCompleted,
            (long long unsigned int)rsaStats.numRsaEncryptCompletedErrors);

            /*rsa dec Info*/
            len += snprintf(data+len, size-len,
            BORDER " RSA Decrypt Requests:           %16llu " BORDER "\n"
            BORDER " RSA Decrypt Request Errors:     %16llu " BORDER "\n"
            BORDER " RSA Decrypt Completed:          %16llu " BORDER "\n"
            BORDER " RSA Decrypt Completed Errors:   %16llu " BORDER "\n",
            (long long unsigned int)rsaStats.numRsaDecryptRequests,
            (long long unsigned int)rsaStats.numRsaDecryptRequestErrors,
            (long long unsigned int)rsaStats.numRsaDecryptCompleted,
            (long long unsigned int)rsaStats.numRsaDecryptCompletedErrors);
            break;
        }
        case SAL_STATS_DH:
        {
            CpaCyDhStats64 dhStats = {0};
            if(CPA_TRUE !=
                  pCryptoService->generic_service_info.stats->bDhStatsEnabled)
            {
                break;
            }
            status = cpaCyDhQueryStats64(pCryptoService, &dhStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyDhQueryStats returned error\n");
                return 0;
            }

            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " Diffie Hellman Stats                             "
                BORDER "\n"
                SEPARATOR);

            /* perform Info */
            len += snprintf(data+len, size-len,
                BORDER " DH Phase1 Key Gen Requests:     %16llu " BORDER "\n"
                BORDER " DH Phase1 Key Gen Request Err:  %16llu " BORDER "\n"
                BORDER " DH Phase1 Key Gen Completed:    %16llu " BORDER "\n"
                BORDER " DH Phase1 Key Gen Completed Err:%16llu " BORDER "\n"
                SEPARATOR,
                (long long unsigned int)dhStats.numDhPhase1KeyGenRequests,
                (long long unsigned int)dhStats.numDhPhase1KeyGenRequestErrors,
                (long long unsigned int)dhStats.numDhPhase1KeyGenCompleted,
                (long long unsigned int)
                    dhStats.numDhPhase1KeyGenCompletedErrors);

            len += snprintf(data+len, size-len,
                BORDER " DH Phase2 Key Gen Requests:     %16llu " BORDER "\n"
                BORDER " DH Phase2 Key Gen Request Err:  %16llu " BORDER "\n"
                BORDER " DH Phase2 Key Gen Completed:    %16llu " BORDER "\n"
                BORDER " DH Phase2 Key Gen Completed Err:%16llu " BORDER "\n",
                (long long unsigned int)dhStats.numDhPhase2KeyGenRequests,
                (long long unsigned int)dhStats.numDhPhase2KeyGenRequestErrors,
                (long long unsigned int)dhStats.numDhPhase2KeyGenCompleted,
                (long long unsigned int)
                    dhStats.numDhPhase2KeyGenCompletedErrors);
                break;
        }
        case SAL_STATS_KEYGEN:
        {
            CpaCyKeyGenStats64 keyStats = {0};
            if(CPA_TRUE !=
              pCryptoService->generic_service_info.stats->bKeyGenStatsEnabled)
            {
                break;
            }
            status = cpaCyKeyGenQueryStats64(pCryptoService, &keyStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyKeyGenQueryStats64 returned error\n");
                return 0;
            }

            /* Key Gen stats */
            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " Key Stats                                        "
                BORDER "\n"
                SEPARATOR);

            len += snprintf(data+len, size-len,
               BORDER " SSL Key Requests:               %16llu " BORDER "\n"
               BORDER " SSL Key Request Errors:         %16llu " BORDER "\n"
               BORDER " SSL Key Completed               %16llu " BORDER "\n"
               BORDER " SSL Key Complete Errors:        %16llu " BORDER "\n"
               SEPARATOR,
               (long long unsigned int)keyStats.numSslKeyGenRequests,
               (long long unsigned int)keyStats.numSslKeyGenRequestErrors,
               (long long unsigned int)keyStats.numSslKeyGenCompleted,
               (long long unsigned int)keyStats.numSslKeyGenCompletedErrors);

            len += snprintf(data+len, size-len,
               BORDER " TLS Key Requests:               %16llu " BORDER "\n"
               BORDER " TLS Key Request Errors:         %16llu " BORDER "\n"
               BORDER " TLS Key Completed               %16llu " BORDER "\n"
               BORDER " TLS Key Complete Errors:        %16llu " BORDER "\n"
               SEPARATOR,
               (long long unsigned int)keyStats.numTlsKeyGenRequests,
               (long long unsigned int)keyStats.numTlsKeyGenRequestErrors,
               (long long unsigned int)keyStats.numTlsKeyGenCompleted,
               (long long unsigned int)keyStats.numTlsKeyGenCompletedErrors);

            len += snprintf(data+len, size-len,
               BORDER " MGF Key Requests:               %16llu " BORDER "\n"
               BORDER " MGF Key Request Errors:         %16llu " BORDER "\n"
               BORDER " MGF Key Completed               %16llu " BORDER "\n"
               BORDER " MGF Key Complete Errors:        %16llu " BORDER "\n",
               (long long unsigned int)keyStats.numMgfKeyGenRequests,
               (long long unsigned int)keyStats.numMgfKeyGenRequestErrors,
               (long long unsigned int)keyStats.numMgfKeyGenCompleted,
               (long long unsigned int)keyStats.numMgfKeyGenCompletedErrors);
            break;
        }
        case SAL_STATS_LN:
        {
            CpaCyLnStats64 lnStats = {0};
            if(CPA_TRUE !=
              pCryptoService->generic_service_info.stats->bLnStatsEnabled)
            {
                break;
            }
            status = cpaCyLnStatsQuery64(pCryptoService, &lnStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyLnStatsQuery64 returned error\n");
                return 0;
            }

            /* Engine Info */
            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " LN ModExp/ModInv Stats                           "
                BORDER "\n"
                SEPARATOR);

            /* Large Number Modular Exponentationstats operations stats */
            len += snprintf(data+len, size-len,
                BORDER " LN ModEXP successful requests:  %16llu " BORDER "\n"
                BORDER " LN ModEXP requests with error:  %16llu " BORDER "\n"
                BORDER " LN ModEXP completed operations: %16llu " BORDER "\n"
                BORDER " LN ModEXP not completed-errors: %16llu " BORDER "\n"
                SEPARATOR,
                (long long unsigned int)lnStats.numLnModExpRequests,
                (long long unsigned int)lnStats.numLnModExpRequestErrors,
                (long long unsigned int)lnStats.numLnModExpCompleted,
                (long long unsigned int)lnStats.numLnModExpCompletedErrors);

            /*  Large Number Modular Inversion operations stats */
            len += snprintf(data+len, size-len,
                BORDER " LN ModINV successful requests:  %16llu " BORDER "\n"
                BORDER " LN ModINV requests with error:  %16llu " BORDER "\n"
                BORDER " LN ModINV completed operations: %16llu " BORDER "\n"
                BORDER " LN ModINV not completed-errors: %16llu " BORDER "\n",
                (long long unsigned int)lnStats.numLnModInvRequests,
                (long long unsigned int)lnStats.numLnModInvRequestErrors,
                (long long unsigned int)lnStats.numLnModInvCompleted,
                (long long unsigned int)lnStats.numLnModInvCompletedErrors);


            break;
        }
        case SAL_STATS_PRIME:
        {
            CpaCyPrimeStats64 primeStats = {0};
            if(CPA_TRUE !=
              pCryptoService->generic_service_info.stats->bPrimeStatsEnabled)
            {
                break;
            }
            status = cpaCyPrimeQueryStats64(pCryptoService, &primeStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyPrimeQueryStats64 returned error\n");
                return 0;
            }

            /* Engine Info */
            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " PRIME Stats                                      "
                BORDER "\n"
                SEPARATOR);

            /* Parameter generation requests - PRIME stats */
            len += snprintf(data+len, size-len,
                BORDER " PRIME successfull requests:     %16llu " BORDER "\n"
                BORDER " PRIME failed requests:          %16llu " BORDER "\n"
                BORDER " PRIME successfully completed:   %16llu " BORDER "\n"
                BORDER " PRIME failed completion:        %16llu " BORDER "\n"
                BORDER " PRIME completed - not a prime:  %16llu " BORDER "\n",
                (long long unsigned int)primeStats.numPrimeTestRequests,
                (long long unsigned int)primeStats.numPrimeTestRequestErrors,
                (long long unsigned int)primeStats.numPrimeTestCompleted,
                (long long unsigned int)primeStats.numPrimeTestCompletedErrors,
                (long long unsigned int)primeStats.numPrimeTestFailures);
            break;
        }
        case SAL_STATS_ECC:
        {
            CpaCyEcStats64 ecStats = {0};
            if(CPA_TRUE !=
              pCryptoService->generic_service_info.stats->bEccStatsEnabled)
            {
                offset +=DOUBLE_INCR;
                break;
            }
            status = cpaCyEcQueryStats64(pCryptoService, &ecStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyEcQueryStats64 returned error\n");
                return 0;
            }

            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " EC Stats                                         "
                BORDER "\n"
                SEPARATOR);

            len += snprintf(data+len, size-len,
                BORDER " EC Pt Multiply Requests-Succ:   %16llu " BORDER "\n"
                BORDER " EC Pt Multiply Request-Err:     %16llu " BORDER "\n"
                BORDER " EC Pt Multiply Completed-Succ:  %16llu " BORDER "\n"
                BORDER " EC Pt Multiply Completed-Err:   %16llu " BORDER "\n"
                BORDER " EC Pt Multiply Output Invalid:  %16llu " BORDER "\n"
                SEPARATOR,
                (long long unsigned int)ecStats.numEcPointMultiplyRequests,
                (long long unsigned int)ecStats.numEcPointMultiplyRequestErrors,
                (long long unsigned int)ecStats.numEcPointMultiplyCompleted,
                (long long unsigned int)
                    ecStats.numEcPointMultiplyCompletedError,
                (long long unsigned int)
                    ecStats.numEcPointMultiplyCompletedOutputInvalid);
            len += snprintf(data+len, size-len,
                BORDER " EC Pt Verify Requests-Succ:     %16llu " BORDER "\n"
                BORDER " EC Pt Verify Request-Err:       %16llu " BORDER "\n"
                BORDER " EC Pt Verify Completed-Succ:    %16llu " BORDER "\n"
                BORDER " EC Pt Verify Completed-Err:     %16llu " BORDER "\n"
                BORDER " EC Pt Verify Output Invalid:    %16llu " BORDER "\n",
                (long long unsigned int)ecStats.numEcPointVerifyRequests,
                (long long unsigned int)ecStats.numEcPointVerifyRequestErrors,
                (long long unsigned int)ecStats.numEcPointVerifyCompleted,
                (long long unsigned int)ecStats.numEcPointVerifyCompletedErrors,
                (long long unsigned int)
                    ecStats.numEcPointVerifyCompletedOutputInvalid);
            break;
        }
        case SAL_STATS_ECDH:
        {
            CpaCyEcdhStats64 ecdhStats = {0};
            status = cpaCyEcdhQueryStats64(pCryptoService, &ecdhStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyEcdhQueryStats64 returned error\n");
                return 0;
            }

            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " ECDH Stats                                       "
                BORDER "\n"
                SEPARATOR);
            len += snprintf(data+len, size-len,
                BORDER " ECDH Pt Multiply Requests-Succ: %16llu " BORDER "\n"
                BORDER " ECDH Pt Multiply Request-Err:   %16llu " BORDER "\n"
                BORDER " ECDH Pt Multiply Completed-Succ:%16llu " BORDER "\n"
                BORDER " ECDH Pt Multiply Completed-Err: %16llu " BORDER "\n"
                BORDER " ECDH Output Invalid:            %16llu " BORDER "\n",
                (long long unsigned int)ecdhStats.numEcdhPointMultiplyRequests,
                (long long unsigned int)
                    ecdhStats.numEcdhPointMultiplyRequestErrors,
                (long long unsigned int)ecdhStats.numEcdhPointMultiplyCompleted,
                (long long unsigned int)
                    ecdhStats.numEcdhPointMultiplyCompletedError,
                (long long unsigned int)
                    ecdhStats.numEcdhRequestCompletedOutputInvalid);
            break;
        }
        case SAL_STATS_ECDSA:
        {
            CpaCyEcdsaStats64 ecdsaStats = {0};
            status = cpaCyEcdsaQueryStats64(pCryptoService, &ecdsaStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyEcdsaQueryStats64 returned error\n");
                return 0;
            }

            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " ECDSA Stats                                      "
                BORDER "\n"
                SEPARATOR);
            len += snprintf(data+len, size-len,
                BORDER " ECDSA Sign R Requests-Succ:     %16llu " BORDER "\n"
                BORDER " ECDSA Sign R Request-Err:       %16llu " BORDER "\n"
                BORDER " ECDSA Sign R Completed-Succ:    %16llu " BORDER "\n"
                BORDER " ECDSA Sign R Completed-Err:     %16llu " BORDER "\n"
                BORDER " ECDSA Sign R Output Invalid:    %16llu " BORDER "\n"
                SEPARATOR,
                (long long unsigned int)ecdsaStats.numEcdsaSignRRequests,
                (long long unsigned int)ecdsaStats.numEcdsaSignRRequestErrors,
                (long long unsigned int)ecdsaStats.numEcdsaSignRCompleted,
                (long long unsigned int)ecdsaStats.numEcdsaSignRCompletedErrors,
                (long long unsigned int)
                    ecdsaStats.numEcdsaSignRCompletedOutputInvalid);
            len += snprintf(data+len, size-len,
                BORDER " ECDSA Sign S Requests-Succ:     %16llu " BORDER "\n"
                BORDER " ECDSA Sign S Request-Err:       %16llu " BORDER "\n"
                BORDER " ECDSA Sign S Completed-Succ:    %16llu " BORDER "\n"
                BORDER " ECDSA Sign S Completed-Err:     %16llu " BORDER "\n"
                BORDER " ECDSA Sign S Output Invalid:    %16llu " BORDER "\n"
                SEPARATOR,
                (long long unsigned int)ecdsaStats.numEcdsaSignSRequests,
                (long long unsigned int)ecdsaStats.numEcdsaSignSRequestErrors,
                (long long unsigned int)ecdsaStats.numEcdsaSignSCompleted,
                (long long unsigned int)ecdsaStats.numEcdsaSignSCompletedErrors,
                (long long unsigned int)
                    ecdsaStats.numEcdsaSignSCompletedOutputInvalid);
            len += snprintf(data+len, size-len,
                BORDER " ECDSA Sign RS Requests-Succ:    %16llu " BORDER "\n"
                BORDER " ECDSA Sign RS Request-Err:      %16llu " BORDER "\n"
                BORDER " ECDSA Sign RS Completed-Succ:   %16llu " BORDER "\n"
                BORDER " ECDSA Sign RS Completed-Err:    %16llu " BORDER "\n"
                BORDER " ECDSA Sign RS Output Invalid:   %16llu " BORDER "\n"
                SEPARATOR,
                (long long unsigned int)ecdsaStats.numEcdsaSignRSRequests,
                (long long unsigned int)ecdsaStats.numEcdsaSignRSRequestErrors,
                (long long unsigned int)ecdsaStats.numEcdsaSignRSCompleted,
                (long long unsigned int)
                    ecdsaStats.numEcdsaSignRSCompletedErrors,
                (long long unsigned int)
                    ecdsaStats.numEcdsaSignRSCompletedOutputInvalid);
            len += snprintf(data+len, size-len,
                BORDER " ECDSA Verify Requests-Succ:     %16llu " BORDER "\n"
                BORDER " ECDSA Verify Request-Err:       %16llu " BORDER "\n"
                BORDER " ECDSA Verify Completed-Succ:    %16llu " BORDER "\n"
                BORDER " ECDSA Verify Completed-Err:     %16llu " BORDER "\n"
                BORDER " ECDSA Verify Output Invalid:    %16llu " BORDER "\n",
                (long long unsigned int)ecdsaStats.numEcdsaVerifyRequests,
                (long long unsigned int)ecdsaStats.numEcdsaVerifyRequestErrors,
                (long long unsigned int)ecdsaStats.numEcdsaVerifyCompleted,
                (long long unsigned int)
                    ecdsaStats.numEcdsaVerifyCompletedErrors,
                (long long unsigned int)
                    ecdsaStats.numEcdsaVerifyCompletedOutputInvalid);
            break;
        }
        case SAL_STATS_DRBG:
        {
            CpaCyDrbgStats64 drbgStats = {0};
            if(CPA_TRUE !=
              pCryptoService->generic_service_info.stats->bDrbgStatsEnabled)
            {
                break;
            }
            status = cpaCyDrbgQueryStats64(pCryptoService, &drbgStats);
            if(status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("cpaCyDrbgQueryStats returned error\n");
                return 0;
            }

            /* Engine Info */
            len += snprintf(data+len, size-len,
                SEPARATOR
                BORDER " DRBG Stats                                       "
                BORDER "\n"
                SEPARATOR );

            /* Random Info */
            len += snprintf(data+len, size-len,
                BORDER " DRBG Gen Requests:              %16llu " BORDER "\n"
                BORDER " DRBG Gen Request Errors:        %16llu " BORDER "\n"
                BORDER " DRBG Gen Completed:             %16llu " BORDER "\n"
                BORDER " DRBG Gen Completed Errors:      %16llu " BORDER "\n"
                BORDER " DRBG Reseed Requests:           %16llu " BORDER "\n"
                BORDER " DRBG Reseed Request Errors:     %16llu " BORDER "\n"
                BORDER " DRBG Reseeds Completed:         %16llu " BORDER "\n"
                BORDER " DRBG Reseeds Completed Errors:  %16llu " BORDER "\n"
                BORDER " DRBG Sessions Initialized:      %16llu " BORDER "\n"
                BORDER " DRBG Sessions Removed:          %16llu " BORDER "\n"
                BORDER " DRBG Session Errors:            %16llu " BORDER "\n",
                (long long unsigned int)drbgStats.numGenRequests,
                (long long unsigned int)drbgStats.numGenRequestErrors,
                (long long unsigned int)drbgStats.numGenCompleted,
                (long long unsigned int)drbgStats.numGenCompletedErrors,
                (long long unsigned int)drbgStats.numReseedRequests,
                (long long unsigned int)drbgStats.numReseedRequestErrors,
                (long long unsigned int)drbgStats.numReseedCompleted,
                (long long unsigned int)drbgStats.numReseedCompletedErrors,
                (long long unsigned int)drbgStats.numSessionsInitialized,
                (long long unsigned int)drbgStats.numSessionsRemoved,
                (long long unsigned int)drbgStats.numSessionErrors);
            break;
        }
        default:
        {
            len += snprintf(data+len, size-len, SEPARATOR);
            return 0;
        }
    }
    return ++offset;
}

CpaStatus validateConcurrRequest(Cpa32U numConcurrRequests)
{
    Cpa32U baseReq=0;

    if (numConcurrRequests < SAL_64_CONCURR_REQUESTS)
    {
        return CPA_STATUS_FAIL;
    }

    baseReq = SAL_64_CONCURR_REQUESTS;
    while (baseReq <= SAL_MAX_CONCURR_REQUESTS)
    {
        if (baseReq != numConcurrRequests)
        {
            baseReq = baseReq << 1;
        }
        else
        {
            break;
        }
    }
    if (baseReq > SAL_MAX_CONCURR_REQUESTS)
    {
        return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}

CpaStatus
SalCtrl_CryptoInit(icp_accel_dev_t* device, sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    Cpa32U qatHmacMode = 0;
    Cpa32U numSymConcurrentReq = 0;
    Cpa32U numAsymConcurrentReq = 0;
    Cpa32U drbgPollAndWaitTime = 0;
    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char temp_string[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char temp_string2[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char *instance_name = NULL;
    sal_crypto_service_t* pCryptoService = (sal_crypto_service_t*) service;
    sal_statistics_collection_t *pStatsCollection =
        (sal_statistics_collection_t*)device->pQatStats;

    char * section = DYN_SEC;

    SAL_SERVICE_GOOD_FOR_INIT(pCryptoService);
    pCryptoService->generic_service_info.state = SAL_SERVICE_STATE_INITIALIZING;

    /* This function:
     * 1. Creates sym and asym transport handles
     * 2. Allocates memory pools required by sym and asym services
     * 3. Clears the sym and asym stats counters
     */

    /* Register callbacks for the symmetric services
    * (Hash, Cipher, Algorithm-Chaining) (returns void)*/
    LacSymCb_CallbacksRegister();


    /*get the HmacMode*/
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      "cyHmacAuthMode",
                                      adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to get cyHmacAuthMode");
        return status;
    }

    qatHmacMode = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    switch(qatHmacMode)
    {
        case HMAC_MODE_1: pCryptoService->qatHmacMode = ICP_QAT_HW_AUTH_MODE1;
                break;
        case HMAC_MODE_2: pCryptoService->qatHmacMode = ICP_QAT_HW_AUTH_MODE2;
                break;
        default:
                pCryptoService->qatHmacMode = ICP_QAT_HW_AUTH_MODE1;
                LAC_LOG_ERROR("Invalid Auth Mode, Mode 1 used per default");
                break;
    }

    /*get the drbgPollAndWaitTime */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      "drbgPollAndWaitTimeMS",
                                      adfGetParam);
    if(CPA_STATUS_SUCCESS == status)
    {
        drbgPollAndWaitTime = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
        if (drbgPollAndWaitTime == 0)
        {
            LAC_LOG_ERROR("DrbgPollAndWaitTime in GENERAL section must be greater than zero.");
            return CPA_STATUS_INVALID_PARAM;
        }
        pCryptoService->drbgPollAndWaitTime = drbgPollAndWaitTime;
    } else {
        pCryptoService->drbgPollAndWaitTime = LAC_SYM_DRBG_POLL_AND_WAIT_TIME_MS;
    }

    if(CPA_FALSE == pCryptoService->generic_service_info.is_dyn)
    {
        section = icpGetProcessName();
    }

   /* Get Config Info: Accel Num, bank Num, packageID,
                            coreAffinity, nodeAffinity and response mode */

    status = Sal_StringParsing("Cy",
            pCryptoService->generic_service_info.instance,
            "AcceleratorNumber", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
            section,
            temp_string,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        return status;
    }
    pCryptoService->acceleratorNum = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

    status = Sal_StringParsing("Cy",
            pCryptoService->generic_service_info.instance,
            "BankNumber", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
            section,
            temp_string,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        return status;
    }
    pCryptoService->bankNum = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

    status = Sal_StringParsing("Cy",
            pCryptoService->generic_service_info.instance,
            "IsPolled", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
            section,
            temp_string,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        return status;
    }
    pCryptoService->isPolled = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

    status = icp_adf_cfgGetParamValue(device,
            LAC_CFG_SECTION_GENERAL,
            ADF_DEV_PKG_ID,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            ADF_DEV_PKG_ID);
        return status;
    }
    pCryptoService->pkgID = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

    status = icp_adf_cfgGetParamValue(device,
            LAC_CFG_SECTION_GENERAL,
            ADF_DEV_NODE_ID,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            ADF_DEV_NODE_ID);
        return status;
    }
    pCryptoService->nodeAffinity = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

   /* Next need to read the [AcceleratorX] section of the config file */
    status = Sal_StringParsing("Accelerator",
            pCryptoService->acceleratorNum,
            "", temp_string2);
    LAC_CHECK_STATUS(status);
    status = Sal_StringParsing("Bank",
            pCryptoService->bankNum,
            "CoreIDAffinity", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
            temp_string2,
            temp_string,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        return status;
    }
    pCryptoService->coreAffinity = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);
    status = Sal_StringParsing("Cy",
            pCryptoService->generic_service_info.instance,
            "ExecutionEngine", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
            section,
            temp_string,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        return status;
    }
    pCryptoService->executionEngine = (Cpa8U)Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

    status = Sal_StringParsing("Cy",
            pCryptoService->generic_service_info.instance,
            "CmnRespOrder", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
            section,
            temp_string,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        pCryptoService->cmnRespOrder = 1; // default ordering
    }
    else {
        pCryptoService->cmnRespOrder = (Cpa8U)Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);
    }

    status = Sal_StringParsing("Cy",
            pCryptoService->generic_service_info.instance,
            "PkeRespOrder", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
            section,
            temp_string,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        pCryptoService->pkeRespOrder = 1; // default ordering
    }
    else {
        pCryptoService->pkeRespOrder = (Cpa8U)Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);
    }

    /* Build common request flags and pke flags for requests sent
     * on this instance */
    if(0 == pCryptoService->executionEngine)
    {
         pCryptoService->cmnReqFlags =
         ICP_QAT_FW_COMN_FLAGS_BUILD(((pCryptoService->cmnRespOrder == ICP_QAT_FW_ORDER_NONE) ? \
                                   ICP_QAT_FW_COMN_ORD_FLAG_NONE : \
                                   ICP_QAT_FW_COMN_ORD_FLAG_STRICT),
                                   QAT_COMN_PTR_TYPE_SGL,
                                   ((pCryptoService->cmnRespOrder == ICP_QAT_FW_EXTEND_ORDER) ? \
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_STRICT : \
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE),
                                   QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                                   QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                                   QAT_COMN_XLAT_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CPR_SLICE_NOT_REQUIRED,
                                   QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                                   QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                                   QAT_COMN_RND_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH0_SLICE_REQUIRED,
                                   QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER0_SLICE_REQUIRED);
     pCryptoService->keyCmnReqFlags =
         ICP_QAT_FW_COMN_FLAGS_BUILD(ICP_QAT_FW_COMN_ORD_FLAG_NONE,
                                   QAT_COMN_PTR_TYPE_FLAT,
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE,
                                   QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                                   QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                                   QAT_COMN_XLAT_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CPR_SLICE_NOT_REQUIRED,
                                   QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                                   QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                                   QAT_COMN_RND_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH0_SLICE_REQUIRED,
                                   QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER0_SLICE_REQUIRED);
        pCryptoService->pkeCmnReqFlags =
        ICP_QAT_FW_COMN_FLAGS_BUILD(((pCryptoService->pkeRespOrder == ICP_QAT_FW_ORDER_NONE) ? \
                                   ICP_QAT_FW_COMN_ORD_FLAG_NONE : \
                                   ICP_QAT_FW_COMN_ORD_FLAG_STRICT),
                                   QAT_COMN_PTR_TYPE_FLAT,
                                   ((pCryptoService->pkeRespOrder == ICP_QAT_FW_EXTEND_ORDER) ? \
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_STRICT : \
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE),
                                   QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                                   QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                                   QAT_COMN_XLAT_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CPR_SLICE_NOT_REQUIRED,
                                   QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                                   QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                                   QAT_COMN_RND_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE0_SLICE_REQUIRED,
                                   QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED);
    }
    else if(1 == pCryptoService->executionEngine)
    {
         pCryptoService->cmnReqFlags =
         ICP_QAT_FW_COMN_FLAGS_BUILD(((pCryptoService->cmnRespOrder == ICP_QAT_FW_ORDER_NONE) ? \
                                   ICP_QAT_FW_COMN_ORD_FLAG_NONE : \
                                   ICP_QAT_FW_COMN_ORD_FLAG_STRICT),
                                   QAT_COMN_PTR_TYPE_SGL,
                                   ((pCryptoService->cmnRespOrder == ICP_QAT_FW_EXTEND_ORDER) ? \
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_STRICT : \
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE),
                                   QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                                   QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                                   QAT_COMN_XLAT_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CPR_SLICE_NOT_REQUIRED,
                                   QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                                   QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                                   QAT_COMN_RND_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH1_SLICE_REQUIRED,
                                   QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER1_SLICE_REQUIRED,
                                   QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED);
     pCryptoService->keyCmnReqFlags =
         ICP_QAT_FW_COMN_FLAGS_BUILD(ICP_QAT_FW_COMN_ORD_FLAG_NONE,
                                   QAT_COMN_PTR_TYPE_FLAT,
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE,
                                   QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                                   QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                                   QAT_COMN_XLAT_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CPR_SLICE_NOT_REQUIRED,
                                   QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                                   QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                                   QAT_COMN_RND_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH1_SLICE_REQUIRED,
                                   QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER1_SLICE_REQUIRED,
                                   QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED);
        pCryptoService->pkeCmnReqFlags =
        ICP_QAT_FW_COMN_FLAGS_BUILD(((pCryptoService->pkeRespOrder == ICP_QAT_FW_ORDER_NONE) ? \
                                   ICP_QAT_FW_COMN_ORD_FLAG_NONE : \
                                   ICP_QAT_FW_COMN_ORD_FLAG_STRICT),
                                   QAT_COMN_PTR_TYPE_FLAT,
                                   ((pCryptoService->pkeRespOrder == ICP_QAT_FW_EXTEND_ORDER) ? \
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_STRICT : \
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE),
                                   QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                                   QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                                   QAT_COMN_XLAT_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CPR_SLICE_NOT_REQUIRED,
                                   QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                                   QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                                   QAT_COMN_RND_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE1_SLICE_REQUIRED,
                                   QAT_COMN_PKE0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED);
    }
    else
    {
            LAC_LOG_ERROR1("Invalid execution engine %d",
                              pCryptoService->executionEngine);
            return CPA_STATUS_FAIL;
    }

    /* num concurrent requests from config file */
    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "NumConcurrentSymRequests", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
                                      section,
                                      temp_string,
                                      adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        return status;
    }

    numSymConcurrentReq = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    if (CPA_STATUS_FAIL == validateConcurrRequest(numSymConcurrentReq))
    {
       LAC_LOG_ERROR("Invalid NumConcurrentSymRequests, valid "\
                      "values {64, 128, 256, ... 32768, 65536}");
       return CPA_STATUS_FAIL;
    }

    /* ADF does not allow us to completely fill the ring for batch requests */
    pCryptoService->maxNumSymReqBatch =
        ((numSymConcurrentReq) - SAL_BATCH_SUBMIT_FREE_SPACE);

    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "NumConcurrentAsymRequests", temp_string);
    LAC_CHECK_STATUS(status);
    status = icp_adf_cfgGetParamValue(device,
                                      section,
                                      temp_string,
                                      adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        return status;
    }

    numAsymConcurrentReq = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    if (CPA_STATUS_FAIL == validateConcurrRequest(numAsymConcurrentReq))
    {
       LAC_LOG_ERROR("Invalid NumConcurrentAsymRequests, valid "\
                      "values {64, 128, 256, 512, .. 32768, 65536}");
       return CPA_STATUS_FAIL;
    }

    /* 1. Create transport handles */
    status = SalCtrl_CreateTransHandle(device, service, numSymConcurrentReq,
                                       numAsymConcurrentReq,
                                       pCryptoService->executionEngine);
    LAC_CHECK_STATUS(status);

    /* 2. Allocates memory pools */

    /* Create and initialise symmetric cookie memory pool */
    pCryptoService->lac_sym_cookie_pool = LAC_MEM_POOL_INIT_POOL_ID;
    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "SymPool", temp_string);
    LAC_CHECK_STATUS_CY_INIT(status);
    /* Note we need twice (i.e. <<1) the number of sym cookies to
       support hi and lo priority ring pairs (and some, for partials) */
    status = Lac_MemPoolCreate(&pCryptoService->lac_sym_cookie_pool,
                    temp_string,
                    ((numSymConcurrentReq + numSymConcurrentReq + 1) <<1),
                    sizeof(lac_sym_cookie_t),
                    LAC_64BYTE_ALIGNMENT,
                    CPA_FALSE,
                    pCryptoService->nodeAffinity);
    LAC_CHECK_STATUS_CY_INIT(status);
    /* For all sym cookies fill out the physical address of data that
       will be set to QAT */
    Lac_MemPoolInitSymCookiesPhyAddr(
                                    pCryptoService->lac_sym_cookie_pool);

    /* Memory for input buffers that need to be resized to be sent to PKE */
    pCryptoService->lac_pke_align_pool = LAC_MEM_POOL_INIT_POOL_ID;
    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "AsymResizePool", temp_string);
    LAC_CHECK_STATUS_CY_INIT(status);
    status = Lac_MemPoolCreate(&pCryptoService->lac_pke_align_pool,
                           temp_string,
                           ((numAsymConcurrentReq + 1)
                           * LAC_PKE_BUFFERS_PER_OP_MAX),
                           LAC_BITS_TO_BYTES(LAC_MAX_OP_SIZE_IN_BITS),
                           LAC_64BYTE_ALIGNMENT,
                           CPA_FALSE,
                           pCryptoService->nodeAffinity);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Allocate pke request memory pool */
    pCryptoService->lac_pke_req_pool = LAC_MEM_POOL_INIT_POOL_ID;
    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "AsymReqPool", temp_string);
    LAC_CHECK_STATUS_CY_INIT(status);
    status = Lac_MemPoolCreate(&(pCryptoService->lac_pke_req_pool),
        temp_string,
        ((numAsymConcurrentReq + 1)
        * LAC_PKE_MAX_CHAIN_LENGTH),
        sizeof(lac_pke_qat_req_data_t),
        LAC_64BYTE_ALIGNMENT, CPA_FALSE,
        pCryptoService->nodeAffinity);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Allocate prime memory pool */
    pCryptoService->lac_prime_pool = LAC_MEM_POOL_INIT_POOL_ID;
    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "AsymPrimePool", temp_string);
    LAC_CHECK_STATUS_CY_INIT(status);
    status = Lac_MemPoolCreate(&pCryptoService->lac_prime_pool, temp_string,
            (numAsymConcurrentReq * 2 + 1),
            (sizeof(CpaFlatBuffer) * (LAC_PRIME_MAX_MR + 1)),
            LAC_64BYTE_ALIGNMENT, CPA_FALSE, pCryptoService->nodeAffinity);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Allocate EC memory pool */
    pCryptoService->lac_ec_pool = LAC_MEM_POOL_INIT_POOL_ID;
    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "AsymEcMemPool", temp_string);
    LAC_CHECK_STATUS_CY_INIT(status);
    status = Lac_MemPoolCreate(
            &pCryptoService->lac_ec_pool,
            temp_string,
            (numAsymConcurrentReq + 1),
            ((LAC_EC_NUM_CONCAT_INPUTS * LAC_EC_SIZE_BYTES_MAX)
                                       + sizeof(CpaFlatBuffer)),
            LAC_64BYTE_ALIGNMENT,
            CPA_FALSE,
            pCryptoService->nodeAffinity);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* 3. Clear stats */
    /* Clears Key stats and alloctes memory of SSL and TLS labels
        These labels are initialised to standard values */
    status = LacSymKey_Init(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

#ifdef NRGB_DISABLED
    LAC_LOG("NRGB is disabled\n");
#else
    /* Set NRGB Callback functions */
    LacSymNrbg_Init();
#endif

    /* Creates sym Rx transport handle and initialises the hash lookup table*/
    status = LacSymQat_Init(device, pCryptoService, numSymConcurrentReq);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Fills out content descriptor for precomputes and registers the
       hash precompute callback */
    status = LacSymHash_HmacPrecompInit(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Init the Sym stats */
    status = LacSym_StatsInit(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Init DH stats */
    status = LacDh_Init(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Init DRBG stats */
    status = LacDrbg_StatsInit(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Init Dsa stats */
    status = LacDsa_Init(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Init Ec stats */
    status = LacEc_Init(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Init Ln Stats */
    status = LacLn_Init(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Init Prime stats */
    status = LacPrime_Init(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Init Rsa Stats */
    status = LacRsa_Init(pCryptoService);
    LAC_CHECK_STATUS_CY_INIT(status);

    /* Get MMPLib physical address */
    status = icp_adf_cfgGetParamValue(device,
            LAC_CFG_SECTION_INTERNAL,
            ICP_CFG_MMP_PHYS_ADDRESS_KEY,
            adfGetParam);
    LAC_CHECK_STATUS_CY_INIT(status);
    pCryptoService->mmpPhysAddr = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_HEX);

   /* Build Flow ID for all pke request sent on this instance */
    pCryptoService->pkeFlowId = ( LAC_PKE_FLOW_ID_TAG |
              (pCryptoService->acceleratorNum << LAC_PKE_ACCEL_ID_BIT_POS) |
              (pCryptoService->executionEngine << LAC_PKE_SLICE_ID_BIT_POS));

    /* For all asym requests fill out known data */
    Lac_MemPoolInitAsymCookies(pCryptoService->lac_pke_req_pool,
                                                       pCryptoService);

    if(CPA_TRUE == pStatsCollection->bStatsEnabled)
    {
       /* Get instance name for stats */
        status = LAC_OS_MALLOC(&instance_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
        LAC_CHECK_STATUS_CY_INIT(status);

        status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "Name", temp_string);
        if(CPA_STATUS_SUCCESS != status)
        {
            LAC_OS_FREE(instance_name);
            SalCtrl_CryptoFreeResources(pCryptoService);
            return status;
        }
        status = icp_adf_cfgGetParamValue(device,
                section,
                temp_string,
                adfGetParam);
        if(CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
                temp_string);
            LAC_OS_FREE(instance_name);
            SalCtrl_CryptoFreeResources(pCryptoService);
            return status;
        }
        snprintf(instance_name,ADF_CFG_MAX_VAL_LEN_IN_BYTES,"%s",adfGetParam);

        status = LAC_OS_MALLOC(&pCryptoService->debug_file,
                              sizeof(debug_file_info_t));
        if(CPA_STATUS_SUCCESS != status)
        {
            LAC_OS_FREE(instance_name);
            SalCtrl_CryptoFreeResources(pCryptoService);
            return status;
        }
        osalMemSet(pCryptoService->debug_file, 0, sizeof(debug_file_info_t));
        pCryptoService->debug_file->name = instance_name;
        pCryptoService->debug_file->seq_read = SalCtrl_CryptoDebug;
        pCryptoService->debug_file->private_data = pCryptoService;
        pCryptoService->debug_file->parent =
            pCryptoService->generic_service_info.debug_parent_dir;

        status = icp_adf_debugAddFile(device,
                      pCryptoService->debug_file);
        if(CPA_STATUS_SUCCESS != status)
        {
            LAC_OS_FREE(instance_name);
            LAC_OS_FREE(pCryptoService->debug_file);
            SalCtrl_CryptoFreeResources(pCryptoService);
            return status;
        }
    }
    pCryptoService->generic_service_info.stats = pStatsCollection;
    pCryptoService->generic_service_info.state = SAL_SERVICE_STATE_INITIALIZED;
    return status;
}

CpaStatus
SalCtrl_CryptoStart(icp_accel_dev_t* device, sal_service_t* service)
{
    sal_crypto_service_t* pCryptoService = (sal_crypto_service_t*) service;
    CpaStatus status = CPA_STATUS_SUCCESS;
#ifndef PKE_DISABLED
    Cpa32U no_wireless_procs = 0;
    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
#endif

    if(pCryptoService->generic_service_info.state !=
        SAL_SERVICE_STATE_INITIALIZED)
    {
        LAC_LOG_ERROR("Not in the correct state to call start\n");
        return CPA_STATUS_FAIL;
    }

#ifndef PKE_DISABLED
    /* Get the number of Wireless Procs */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      "NumberOfWirelessProcs",
                                      adfGetParam);
    if (CPA_STATUS_SUCCESS == status)
    {
        no_wireless_procs = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    }
    else
    {
        no_wireless_procs = 0;
        status = CPA_STATUS_SUCCESS;
    }

    if (no_wireless_procs == 0)
    {
        /* Not running wireless firmware, so we send PKE liveness message */
        status = LacPke_Init(pCryptoService);
        LAC_CHECK_STATUS(status);
    }
#endif
    pCryptoService->generic_service_info.state = SAL_SERVICE_STATE_RUNNING;
    return status;
}

CpaStatus
SalCtrl_CryptoStop(icp_accel_dev_t* device, sal_service_t* service)
{
    sal_crypto_service_t* pCryptoService = (sal_crypto_service_t*) service;

    if (SAL_SERVICE_STATE_RUNNING != pCryptoService->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call stop");
    }

    if(icp_adf_is_dev_in_reset(device))
    {
        pCryptoService->generic_service_info.state =
          SAL_SERVICE_STATE_RESTARTING;
        return CPA_STATUS_SUCCESS;
    }
    pCryptoService->generic_service_info.state =
          SAL_SERVICE_STATE_SHUTTING_DOWN;
    return CPA_STATUS_RETRY;
}

CpaStatus
SalCtrl_CryptoShutdown(icp_accel_dev_t* device, sal_service_t* service)
{
    sal_crypto_service_t* pCryptoService = (sal_crypto_service_t*) service;
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_statistics_collection_t *pStatsCollection =
        (sal_statistics_collection_t*)device->pQatStats;

    if ((SAL_SERVICE_STATE_INITIALIZED !=
         pCryptoService->generic_service_info.state) &&
        (SAL_SERVICE_STATE_SHUTTING_DOWN !=
         pCryptoService->generic_service_info.state) &&
        (SAL_SERVICE_STATE_RESTARTING !=
         pCryptoService->generic_service_info.state))
    {
        LAC_LOG_ERROR("Not in the correct state to call shutdown \n");
        return CPA_STATUS_FAIL;
    }

     /* Free memory and transhandles */
    status = SalCtrl_CryptoFreeResources(pCryptoService);

    if(CPA_TRUE == pStatsCollection->bStatsEnabled)
    {
       /* Clean stats */
        icp_adf_debugRemoveFile(pCryptoService->debug_file);
        LAC_OS_FREE(pCryptoService->debug_file->name);
        LAC_OS_FREE(pCryptoService->debug_file);
        pCryptoService->debug_file = NULL;
    }
    pCryptoService->generic_service_info.stats = NULL;

    if(icp_adf_is_dev_in_reset(device))
    {
        pCryptoService->generic_service_info.state =
          SAL_SERVICE_STATE_RESTARTING;
        return CPA_STATUS_SUCCESS;
    }
    pCryptoService->generic_service_info.state =
      SAL_SERVICE_STATE_SHUTDOWN;
    return status;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/
CpaStatus
cpaCyGetStatusText(const CpaInstanceHandle instanceHandle,
        CpaStatus errStatus,
        Cpa8S *pStatusText)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

#ifdef ICP_TRACE
	LAC_LOG3("Called with params (0x%lx, %d, 0x%lx)\n",
			 (LAC_ARCH_UINT)instanceHandle,
			 errStatus,
			 (LAC_ARCH_UINT)pStatusText);
#endif

    LAC_CHECK_NULL_PARAM(pStatusText);

    switch(errStatus)
    {
        case CPA_STATUS_SUCCESS:
            LAC_COPY_STRING(pStatusText, CPA_STATUS_STR_SUCCESS);
            break;
        case CPA_STATUS_FAIL:
            LAC_COPY_STRING(pStatusText, CPA_STATUS_STR_FAIL);
            break;
        case CPA_STATUS_RETRY:
            LAC_COPY_STRING(pStatusText, CPA_STATUS_STR_RETRY);
            break;
        case CPA_STATUS_RESOURCE:
            LAC_COPY_STRING(pStatusText, CPA_STATUS_STR_RESOURCE);
            break;
        case CPA_STATUS_INVALID_PARAM:
            LAC_COPY_STRING(pStatusText, CPA_STATUS_STR_INVALID_PARAM);
            break;
        case CPA_STATUS_FATAL:
            LAC_COPY_STRING(pStatusText, CPA_STATUS_STR_FATAL);
            break;
        default:
            status = CPA_STATUS_INVALID_PARAM;
            break;
    }
    return status;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/
CpaStatus
cpaCyStartInstance(CpaInstanceHandle instanceHandle_in)
{
    CpaInstanceHandle instanceHandle = NULL;
    /* Structure initializer is part of C99, but it is
     * not supported by some old Intel compiler
    */
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(disable: 188)
#endif
    CpaInstanceInfo2 info = {0};
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(enable)
#endif
    icp_accel_dev_t *dev = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;

#ifdef ICP_TRACE
    LAC_LOG1("Called with params (0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in);
#endif

    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in) {
        instanceHandle = Lac_GetFirstHandle();
    }
    else {
        instanceHandle = instanceHandle_in;
    }
    LAC_CHECK_NULL_PARAM(instanceHandle);

    status = cpaCyInstanceGetInfo2(instanceHandle, &info);
    if (CPA_STATUS_SUCCESS != status) {
        LAC_LOG_ERROR("Can not get instance info\n");
        return status;
    }
    dev = icp_adf_getAccelDevByAccelId(info.physInstId.packageId);
    if (NULL == dev) {
        LAC_LOG_ERROR("Can not find device for the instance\n");
        return CPA_STATUS_FAIL;
    }
    /* Increment dev ref counter */
    icp_qa_dev_get(dev);
    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/
CpaStatus
cpaCyStopInstance(CpaInstanceHandle instanceHandle_in)
{
    CpaInstanceHandle instanceHandle = NULL;
    /* Structure initializer is part of C99, but it is
     * not supported by some old Intel compiler
    */
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(disable: 188)
#endif
    CpaInstanceInfo2 info = {0};
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(enable)
#endif
    icp_accel_dev_t *dev = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;

#ifdef ICP_TRACE
    LAC_LOG1("Called with params (0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in);
#endif

    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in) {
        instanceHandle = Lac_GetFirstHandle();
    }
    else {
        instanceHandle = instanceHandle_in;
    }
    LAC_CHECK_NULL_PARAM(instanceHandle);

    status = cpaCyInstanceGetInfo2(instanceHandle, &info);
    if (CPA_STATUS_SUCCESS != status) {
        LAC_LOG_ERROR("Can not get instance info\n");
        return status;
    }
    dev = icp_adf_getAccelDevByAccelId(info.physInstId.packageId);
    if (NULL == dev) {
        LAC_LOG_ERROR("Can not find device for the instance\n");
        return CPA_STATUS_FAIL;
    }
    /* Decrement dev ref counter */
    icp_qa_dev_put(dev);
    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/
CpaStatus
cpaCyInstanceSetNotificationCb(
    const CpaInstanceHandle instanceHandle,
    const CpaCyInstanceNotificationCbFunc pInstanceNotificationCb,
    void *pCallbackTag)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_service_t *gen_handle = instanceHandle;

#ifdef ICP_TRACE
    LAC_LOG3("Called with params (0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle,
            (LAC_ARCH_UINT)pInstanceNotificationCb,
            (Cpa64U)pCallbackTag);
#endif

    LAC_CHECK_NULL_PARAM(gen_handle);
    gen_handle->notification_cb = pInstanceNotificationCb;
    gen_handle->cb_tag = pCallbackTag;
    return status;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/
CpaStatus
cpaCyGetNumInstances(Cpa16U *pNumInstances)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_accel_dev_t **pAdfInsts = NULL;
    icp_accel_dev_t* dev_addr = NULL;
    sal_t* base_addr = NULL;
    sal_list_t* list_temp =NULL;
    Cpa16U num_accel_dev = 0;
    Cpa16U num=0;
    Cpa16U i=0;

    LAC_CHECK_NULL_PARAM(pNumInstances);

    /* get the number of accel_dev in the system */
    status = icp_amgr_getNumInstances(&num_accel_dev);
    LAC_CHECK_STATUS(status);

    /* Allocate memory to store addr of accel_devs */
    pAdfInsts = osalMemAlloc(num_accel_dev*sizeof(icp_accel_dev_t*));
    if(NULL == pAdfInsts)
    {
        LAC_LOG_ERROR("Failed to allocate dev instance memory");
        return CPA_STATUS_RESOURCE;
    }
    num_accel_dev = 0;
    /* Get ADF to return accel_devs with cy enabled */
    status = icp_amgr_getAllAccelDevByCapabilities(
                                 ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC,
                                                pAdfInsts, &num_accel_dev);
    if(CPA_STATUS_SUCCESS == status)
    {
        for(i=0; i<num_accel_dev; i++)
        {
            dev_addr = (icp_accel_dev_t*) pAdfInsts[i];
            if(NULL != dev_addr)
            {
                base_addr = dev_addr->pSalHandle;
                if(NULL != base_addr)
                {
                    list_temp = base_addr->crypto_services;
                    while(NULL != list_temp)
                    {
                       num++;
                       list_temp=SalList_next(list_temp);
                    }
                }
            }
        }
    }
    *pNumInstances = num;
    osalMemFree(pAdfInsts);

#ifdef ICP_TRACE
    if (NULL != pNumInstances)
    {
		LAC_LOG2("Called with params (0x%lx[%d])\n",
				(LAC_ARCH_UINT)pNumInstances,
				*pNumInstances);
    }
    else
    {
    	LAC_LOG1("Called with params (0x%lx)\n",
				(LAC_ARCH_UINT)pNumInstances);
    }
#endif
    return status;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/
CpaStatus
cpaCyGetInstances(Cpa16U numInstances,
                  CpaInstanceHandle *pCyInstances)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_accel_dev_t **pAdfInsts = NULL;
    icp_accel_dev_t* dev_addr = NULL;
    sal_t* base_addr = NULL;
    sal_list_t* list_temp =NULL;
    Cpa16U num_accel_dev = 0;
    Cpa16U index=0;
    Cpa16U i=0;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (%d, 0x%lx)\n",
            numInstances,
            (LAC_ARCH_UINT)pCyInstances);
#endif

    LAC_CHECK_NULL_PARAM(pCyInstances);
    if (0 == numInstances)
    {
        LAC_INVALID_PARAM_LOG("numInstances is 0");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* get the number of accel_dev in the system */
    status = icp_amgr_getNumInstances(&num_accel_dev);
    LAC_CHECK_STATUS(status);

    /* Allocate memory to store addr of accel_devs */
    pAdfInsts = osalMemAlloc(num_accel_dev*sizeof(icp_accel_dev_t*));
    if(NULL == pAdfInsts)
    {
        LAC_LOG_ERROR("Failed to allocate dev instance memory");
        return CPA_STATUS_RESOURCE;
    }

    num_accel_dev=0;
    /* Get ADF to return accel_devs with cy enabled */
    status = icp_amgr_getAllAccelDevByCapabilities(
                       ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC,
                                     pAdfInsts, &num_accel_dev);

    if(CPA_STATUS_SUCCESS == status)
    {
        /* First check the number of instances in the system */
        for(i=0; i<num_accel_dev; i++)
        {
            dev_addr = (icp_accel_dev_t*) pAdfInsts[i];
            if(NULL != dev_addr)
            {
                base_addr = dev_addr->pSalHandle;
                if(NULL != base_addr)
                {
                    list_temp = base_addr->crypto_services;
                    while(NULL != list_temp)
                    {
                        if(index > (numInstances-1))
                        {
                            break;
                        }
                        list_temp=SalList_next(list_temp);
                        index++;
                    }
                }
            }
        }
        if(numInstances > index)
        {
            LAC_LOG_ERROR1("Only %d cy instances available", index);
            status = CPA_STATUS_RESOURCE;
        }
    }
    if(CPA_STATUS_SUCCESS == status)
    {
        index = 0;
        for(i=0; i<num_accel_dev; i++)
        {
            dev_addr = (icp_accel_dev_t*) pAdfInsts[i];
            /* Note dev_addr cannot be NULL here as numInstances=0
               is not valid and if dev_addr=NULL then index=0 (which
               is less than numInstances and status is set to _RESOURCE
               above */
            base_addr = dev_addr->pSalHandle;
            if(NULL != base_addr)
            {
                list_temp = base_addr->crypto_services;
                while(NULL != list_temp)
                {
                    if(index > (numInstances-1))
                    {
                        break;
                    }
                    pCyInstances[index] = SalList_getObject(list_temp);
                    list_temp=SalList_next(list_temp);
                    index++;
                }
            }
        }
    }
    osalMemFree(pAdfInsts);
    return status;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/
CpaStatus
cpaCyInstanceGetInfo(const CpaInstanceHandle instanceHandle_in,
                     struct _CpaInstanceInfo *pInstanceInfo)
{
    CpaInstanceHandle instanceHandle = NULL;
    sal_crypto_service_t* pCryptoService = NULL;
    sal_service_t* pGenericService = NULL;
    Cpa8U deviceName[SAL_DEVICE_NAME_SIZE] = {};

    Cpa8U name[CPA_INST_NAME_SIZE] =
                       "Intel(R) %s instance number: %02x, type: Crypto";
#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pInstanceInfo);
#endif

    snprintf((char *) deviceName, SAL_DEVICE_NAME_SIZE,
              "%s", SAL_DEVICE_DH89XXCC);
    deviceName[SAL_DEVICE_NAME_SIZE - 1] = '\0';

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in){
        instanceHandle = Lac_GetFirstHandle();
    }else{
        instanceHandle = instanceHandle_in;
    }


    LAC_CHECK_NULL_PARAM(instanceHandle);
    LAC_CHECK_NULL_PARAM(pInstanceInfo);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    pGenericService = (sal_service_t*) instanceHandle;
    if(!(ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER &
           pGenericService->capabilitiesMask))
    {
        snprintf((char *) deviceName,
                  SAL_DEVICE_NAME_SIZE,
                   "%s", SAL_DEVICE_C2XXX);
        deviceName[SAL_DEVICE_NAME_SIZE - 1] = '\0';
    }

    pCryptoService = (sal_crypto_service_t *) instanceHandle;

    pInstanceInfo->type = CPA_INSTANCE_TYPE_CRYPTO;

   /* According to cpa.h instance state is initialized and ready for use
    * or shutdown. Therefore need to map our running state to initialised
    * or shutdown */
    if(SAL_SERVICE_STATE_RUNNING ==
            pCryptoService->generic_service_info.state){

         pInstanceInfo->state = CPA_INSTANCE_STATE_INITIALISED;
    }else{
         pInstanceInfo->state = CPA_INSTANCE_STATE_SHUTDOWN;
    }

    snprintf((char *) pInstanceInfo->name,
               CPA_INST_NAME_SIZE,
               (char *)name, (char *)deviceName, pGenericService->instance);

    pInstanceInfo->name[CPA_INST_NAME_SIZE - 1]
                    = '\0';

    snprintf((char *)pInstanceInfo->version,
               CPA_INSTANCE_MAX_NAME_SIZE_IN_BYTES,
               "%d.%d", CPA_CY_API_VERSION_NUM_MAJOR,
                CPA_CY_API_VERSION_NUM_MINOR);

    pInstanceInfo->version[CPA_INSTANCE_MAX_VERSION_SIZE_IN_BYTES - 1]
                    = '\0';
    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/
CpaStatus
cpaCyInstanceGetInfo2(const CpaInstanceHandle instanceHandle_in,
                     CpaInstanceInfo2 *pInstanceInfo2)
{
    CpaInstanceHandle instanceHandle = NULL;
    sal_crypto_service_t* pCryptoService = NULL;
    sal_service_t* pGenericService = NULL;
    Cpa8U deviceName[SAL_DEVICE_NAME_SIZE] = {};
    icp_accel_dev_t *dev = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    char keyStr[ADF_CFG_MAX_KEY_LEN_IN_BYTES] = {0};
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char * section = DYN_SEC;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pInstanceInfo2);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in){
        instanceHandle = Lac_GetFirstHandle();
    } else {
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_NULL_PARAM(instanceHandle);
    LAC_CHECK_NULL_PARAM(pInstanceInfo2);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    snprintf((char *) deviceName,
                SAL_DEVICE_NAME_SIZE,
                "%s", SAL_DEVICE_DH89XXCC);
    deviceName[SAL_DEVICE_NAME_SIZE - 1] = '\0';

    memset(pInstanceInfo2, '\0', sizeof(CpaInstanceInfo2));
    pGenericService = (sal_service_t*) instanceHandle;

    if(!(ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER &
           pGenericService->capabilitiesMask))
    {
        snprintf((char *) deviceName,
                    SAL_DEVICE_NAME_SIZE,
                    "%s", SAL_DEVICE_C2XXX);
        deviceName[SAL_DEVICE_NAME_SIZE - 1] = '\0';
    }

    pInstanceInfo2->accelerationServiceType = CPA_ACC_SVC_TYPE_CRYPTO;
    snprintf((char *) pInstanceInfo2->vendorName,
               CPA_INST_VENDOR_NAME_SIZE,
               "%s", SAL_INFO2_VENDOR_NAME);
    pInstanceInfo2->vendorName[CPA_INST_VENDOR_NAME_SIZE - 1] = '\0';
    snprintf((char *) pInstanceInfo2->partName,
               CPA_INST_PART_NAME_SIZE,
               SAL_INFO2_PART_NAME, (char *)deviceName);
    pInstanceInfo2->partName[CPA_INST_PART_NAME_SIZE - 1] = '\0';
    snprintf((char *) pInstanceInfo2->swVersion,
               CPA_INST_SW_VERSION_SIZE,
               "Version %d.%d", SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
                                SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER);
    pInstanceInfo2->swVersion[CPA_INST_SW_VERSION_SIZE - 1]  = '\0';

    /* Note we can safely read the contents of the crypto service instance
       here because icp_amgr_getAllAccelDevByCapabilities() only returns devs
       that have started */
    pCryptoService = (sal_crypto_service_t *) instanceHandle;
    pInstanceInfo2->physInstId.packageId = pCryptoService->pkgID;
    pInstanceInfo2->physInstId.acceleratorId = pCryptoService->acceleratorNum;
    pInstanceInfo2->physInstId.executionEngineId =
                     pCryptoService->executionEngine;
    pInstanceInfo2->physInstId.busAddress =
                 icp_adf_get_busAddress(pInstanceInfo2->physInstId.packageId);

    /*set coreAffinity to zero before use */
    LAC_OS_BZERO(pInstanceInfo2->coreAffinity,
                                          sizeof(pInstanceInfo2->coreAffinity));
    CPA_BITMAP_BIT_SET(pInstanceInfo2->coreAffinity,
                                                 pCryptoService->coreAffinity);
    pInstanceInfo2->nodeAffinity = pCryptoService->nodeAffinity;

    if(SAL_SERVICE_STATE_RUNNING ==
           pCryptoService->generic_service_info.state)
    {
         pInstanceInfo2->operState = CPA_OPER_STATE_UP;
    }
    else
    {
         pInstanceInfo2->operState = CPA_OPER_STATE_DOWN;
    }

    pInstanceInfo2->requiresPhysicallyContiguousMemory = CPA_TRUE;
    if(SAL_RESP_POLL_CFG_FILE == pCryptoService->isPolled)
    {
        pInstanceInfo2->isPolled = CPA_TRUE;
    }
    else
    {
        pInstanceInfo2->isPolled = CPA_FALSE;
    }
    pInstanceInfo2->isOffloaded = CPA_TRUE;

    /* Get the instance name */
    dev = icp_adf_getAccelDevByAccelId(pCryptoService->pkgID);
    if (NULL == dev) {
        LAC_LOG_ERROR("Can not find device for the instance\n");
        return CPA_STATUS_FAIL;
    }
    status = Sal_StringParsing("Cy",
                    pCryptoService->generic_service_info.instance,
                    "Name", keyStr);
    LAC_CHECK_STATUS(status);

    if(CPA_FALSE == pCryptoService->generic_service_info.is_dyn)
    {
        section = icpGetProcessName();
    }

    status = icp_adf_cfgGetParamValue(dev, section,
                                      keyStr, valStr);
    LAC_CHECK_STATUS(status);

    strncpy((char*)pInstanceInfo2->instName, valStr, CPA_INST_NAME_SIZE);
    strncpy((char*)pInstanceInfo2->instID, section,
                                                     CPA_INST_NAME_SIZE);
    strncat((char*)pInstanceInfo2->instID, "_", 1);
    strncat((char*)pInstanceInfo2->instID, valStr, CPA_INST_NAME_SIZE);
    return CPA_STATUS_SUCCESS;
}
/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/

CpaStatus cpaCyQueryCapabilities(const CpaInstanceHandle instanceHandle_in,
                            CpaCyCapabilitiesInfo * pCapInfo)
{
    /* Verify Instance exists */
    CpaInstanceHandle instanceHandle = NULL;
    sal_crypto_service_t* pCryptoService = NULL;
    sal_service_t* pGenericService = NULL;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pCapInfo);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in){
        instanceHandle = Lac_GetFirstHandle();
    }else{
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_NULL_PARAM(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
    LAC_CHECK_NULL_PARAM(pCapInfo);
    memset(pCapInfo, 0, sizeof(CpaCyCapabilitiesInfo));
    pCryptoService  = (sal_crypto_service_t *) instanceHandle;
    pGenericService = &(pCryptoService->generic_service_info);

    pCapInfo->symSupported = CPA_TRUE;
    pCapInfo->dhSupported = CPA_TRUE;
    pCapInfo->dsaSupported = CPA_TRUE;
    pCapInfo->rsaSupported = CPA_TRUE;
    pCapInfo->ecSupported = CPA_TRUE;
    pCapInfo->ecdhSupported = CPA_TRUE;
    pCapInfo->ecdsaSupported = CPA_TRUE;
    pCapInfo->keySupported = CPA_TRUE;
    pCapInfo->lnSupported = CPA_TRUE;
    pCapInfo->primeSupported = CPA_TRUE;
    pCapInfo->drbgSupported = CPA_TRUE;
    pCapInfo->nrbgSupported = CPA_FALSE;

    if ((0 == pCryptoService->executionEngine) &&
            (pGenericService->capabilitiesMask &
                ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER)) {
        pCapInfo->nrbgSupported = CPA_TRUE;
    }

    pCapInfo->randSupported = CPA_FALSE;
    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup cpaCySym
 *****************************************************************************/
CpaStatus
cpaCySymQueryCapabilities(const CpaInstanceHandle instanceHandle_in,
        CpaCySymCapabilitiesInfo *pCapInfo)
{
    /* Verify Instance exists */
    CpaInstanceHandle instanceHandle = NULL;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pCapInfo);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in){
        instanceHandle = Lac_GetFirstHandle();
    }else{
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_NULL_PARAM(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
    LAC_CHECK_NULL_PARAM(pCapInfo);

    memset(pCapInfo, '\0', sizeof(CpaCySymCapabilitiesInfo));
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_NULL);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_ARC4);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_AES_ECB);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_AES_CBC);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_AES_CTR);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_AES_CCM);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_AES_GCM);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_DES_ECB);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_DES_CBC);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_3DES_ECB);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_3DES_CBC);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_3DES_CTR);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_KASUMI_F8);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_SNOW3G_UEA2);
    CPA_BITMAP_BIT_SET(pCapInfo->ciphers, CPA_CY_SYM_CIPHER_AES_F8);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_MD5);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_SHA1);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_SHA224);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_SHA256);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_SHA384);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_SHA512);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_AES_XCBC);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_AES_CCM);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_AES_GCM);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_KASUMI_F9);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_SNOW3G_UIA2);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_AES_CMAC);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_AES_GMAC);
    CPA_BITMAP_BIT_SET(pCapInfo->hashes, CPA_CY_SYM_HASH_AES_CBC_MAC); 
    pCapInfo->partialPacketSupported = CPA_TRUE;
    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 *****************************************************************************/

CpaStatus
cpaCySetAddressTranslation(const CpaInstanceHandle instanceHandle_in,
                           CpaVirtualToPhysical virtual2physical)
{
    CpaInstanceHandle instanceHandle = NULL;
    sal_service_t * pService = NULL;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)virtual2physical);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in){
        instanceHandle = Lac_GetFirstHandle();
    }else{
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_NULL_PARAM(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
    LAC_CHECK_NULL_PARAM(virtual2physical);
    pService = (sal_service_t *) instanceHandle;
    pService->virt2PhysClient = virtual2physical;
    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup cpaCyCommon
 * Crypto specific polling function which polls a crypto instance.
 *****************************************************************************/

CpaStatus
icp_sal_CyPollInstance(CpaInstanceHandle instanceHandle_in,
                       Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_crypto_service_t *crypto_handle=NULL;
    sal_service_t *gen_handle=NULL;
    icp_comms_trans_handle trans_hndTable[NUM_CRYPTO_RX_RINGS];

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        crypto_handle = (sal_crypto_service_t *)Lac_GetFirstHandle();
    }
    else
    {
        crypto_handle = (sal_crypto_service_t *)instanceHandle_in;
    }
    LAC_CHECK_NULL_PARAM(crypto_handle);
    SAL_RUNNING_CHECK(crypto_handle);

    gen_handle = &(crypto_handle->generic_service_info);
    if(!(SAL_SERVICE_TYPE_CRYPTO & gen_handle->type))
    {
        LAC_LOG_ERROR("The instance handle is the wrong type");
        return CPA_STATUS_FAIL;
    }

    /*
     * From the instanceHandle we must get the trans_handle and send
     * down to adf for polling.
     * Populate our trans handle table with the appropriate handles.
     */
    trans_hndTable[TH_SYM_RX_HI] = crypto_handle->trans_handle_sym_rx_hi;
    trans_hndTable[TH_SYM_RX_LO] = crypto_handle->trans_handle_sym_rx_lo;
    trans_hndTable[TH_ASYM_RX] = crypto_handle->trans_handle_asym_rx;
    /* Call adf to do the polling. */
    status = icp_adf_pollInstance(trans_hndTable, NUM_CRYPTO_RX_RINGS,
                                  response_quota);
    return status;
}

CpaInstanceHandle
Lac_GetFirstHandle()
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    static icp_accel_dev_t *adfInsts[ADF_MAX_DEVICES] = {0};
    CpaInstanceHandle cyInst = NULL;
    icp_accel_dev_t* dev_addr = NULL;
    sal_t* base_addr = NULL;
    sal_list_t* list_temp =NULL;
    Cpa16U num_cy  = 0;
    Cpa16U i  = 0;


   /* Only need 1 dev with crypto enabled - so check all devices*/
    status = icp_amgr_getAllAccelDevByCapabilities(
                             ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC,
                             adfInsts, &num_cy);
    if((0 == num_cy) || (CPA_STATUS_SUCCESS!=status))
    {
        LAC_LOG_ERROR("No crypto devices enabled in the system\n");
        return cyInst;
    }

    if(num_cy > ADF_MAX_DEVICES)
    {
        num_cy = ADF_MAX_DEVICES;
    }
    for(i=0; i<num_cy; i++)
    {
        dev_addr = (icp_accel_dev_t*) adfInsts[i];
        base_addr = dev_addr->pSalHandle;
        if(NULL != base_addr)
        {
            list_temp = base_addr->crypto_services;
            if(NULL != list_temp)
            {
                cyInst = SalList_getObject(list_temp);
                break;
            }
        }
    }
    if (NULL == cyInst)
    {
        LAC_LOG_ERROR("No remaining crypto instances available\n");
    }
    return cyInst;
}

/* The stub for event based poll */
CpaStatus icp_sal_CyGetFileDescriptor(CpaInstanceHandle instanceHandle_in, int *fd)
{
    return CPA_STATUS_UNSUPPORTED;
}

/* The stub for event based poll */
CpaStatus icp_sal_CyPutFileDescriptor(CpaInstanceHandle instanceHandle_in, int fd)
{
    return CPA_STATUS_UNSUPPORTED;
}
