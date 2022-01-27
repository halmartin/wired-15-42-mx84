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
 * @file sal_compression.c
 *
 * @ingroup SalCtrl
 *
 * @description
 *    This file contains the sal implementation for compression.
 *
 *****************************************************************************/

/* QAT-API includes */
#include "cpa.h"
#include "cpa_dc.h"

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
#include "icp_adf_esram.h"

/* SAL includes */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_common.h"
#include "lac_mem_pools.h"
#include "sal_statistics.h"
#include "lac_list.h"
#include "icp_sal_poll.h"
#include "sal_types_compression.h"
#include "dc_session.h"
#include "dc_datapath.h"
#include "dc_stats.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"
#include "sal_string_parse.h"
#include "sal_service_state.h"
#include "lac_buffer_desc.h"
#include "icp_qat_fw_comp.h"
#include "icp_sal_versions.h"

#define WINDOW_SIZE_8K_MASK     (0x20)
#define WINDOW_SIZE_32K_MASK    (0x80)

/*
 * Get parity report key
 */
CpaStatus 
SalCtrl_GetParityReportValue(const CpaInstanceHandle instanceHandle,
                                   CpaBoolean* parity_report)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    sal_compression_service_t* pCompressionService = NULL;
    icp_accel_dev_t *dev = NULL;

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(parity_report);
#endif

    /* Get the compression service via a cast of the instance. */
    pCompressionService = (sal_compression_service_t*)instanceHandle;

    /* The device is held in the package. */
    dev = icp_adf_getAccelDevByAccelId(pCompressionService->pkgID);

    status = icp_adf_cfgGetParamValue(dev, LAC_CFG_SECTION_GENERAL, 
                              ICP_CFG_REPORT_DC_PARITY_ERROR_KEY, valStr);

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Problem getting PARITY_ERROR key - default to false");
        /* If the parity field is not present then default to false */
        *parity_report = CPA_FALSE;
    }
    else
    {
        *parity_report = 
            (CpaBoolean)Sal_Strtoul(valStr, NULL, SAL_CFG_BASE_DEC);
    }

    return status;
}

/*
 * Prints statistics for a compresion instance
 */
STATIC int SalCtrl_CompresionDebug(void* private_data,
                           char* data, int size, int offset)
{
    sal_compression_service_t* pCompressionService =
                           (sal_compression_service_t*)private_data;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaDcStats dcStats = {0};
    Cpa32S len = 0;

    status = cpaDcGetStats(pCompressionService, &dcStats);
    if(status != CPA_STATUS_SUCCESS)
    {
        LAC_LOG_ERROR("cpaDcGetStats returned error\n");
        return (-1);
    }

    /* Engine Info */
    len += snprintf(data+len, size-len,
         SEPARATOR
         BORDER " Statistics for Instance %24s | \n"
         SEPARATOR, pCompressionService->debug_file->name);

    /*Perform Info*/
    len += snprintf(data+len, size-len,
         BORDER " DC comp Requests:               %16llu " BORDER "\n"
         BORDER " DC comp Request Errors:         %16llu " BORDER "\n"
         BORDER " DC comp Completed:              %16llu " BORDER "\n"
         BORDER " DC comp Completed Errors:       %16llu " BORDER "\n"
         SEPARATOR,
         (long long unsigned int)dcStats.numCompRequests,
         (long long unsigned int)dcStats.numCompRequestsErrors,
         (long long unsigned int)dcStats.numCompCompleted,
         (long long unsigned int)dcStats.numCompCompletedErrors);

    /*Perform Info*/
    len += snprintf(data+len, size-len,
         BORDER " DC decomp Requests:             %16llu " BORDER "\n"
         BORDER " DC decomp Request Errors:       %16llu " BORDER "\n"
         BORDER " DC decomp Completed:            %16llu " BORDER "\n"
         BORDER " DC decomp Completed Errors:     %16llu " BORDER "\n"
         SEPARATOR,
         (long long unsigned int)dcStats.numDecompRequests,
         (long long unsigned int)dcStats.numDecompRequestsErrors,
         (long long unsigned int)dcStats.numDecompCompleted,
         (long long unsigned int)dcStats.numDecompCompletedErrors);
    return 0;
}

CpaStatus
SalCtrl_CompressionInit(icp_accel_dev_t* device, sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U numCompConcurrentReq = 0;
    Cpa32U request_ring_id = 0;
    Cpa32U response_ring_id = 0;
    Cpa32U ring_number_in_bank = 0;

    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
    char compMemPool[SAL_CFG_MAX_VAL_LEN_IN_BYTES];
    char temp_string[SAL_CFG_MAX_VAL_LEN_IN_BYTES]={0};
    char temp_string2[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char *instance_name = NULL;
    sal_statistics_collection_t *pStatsCollection =
        (sal_statistics_collection_t*)device->pQatStats;
    icp_resp_deliv_method rx_resp_type = ICP_RESP_TYPE_IRQ;
    sal_compression_service_t* pCompressionService =
            (sal_compression_service_t*) service;
    Cpa32U msgSize=0;
    char * section = DYN_SEC;

    SAL_SERVICE_GOOD_FOR_INIT(pCompressionService);

    pCompressionService->generic_service_info.state =
      SAL_SERVICE_STATE_INITIALIZING;

    if(CPA_FALSE == pCompressionService->generic_service_info.is_dyn)
    {
        section = icpGetProcessName();
    }

    /* Get Config Info: Accel Num, bank Num, packageID,
                                coreAffinity, nodeAffinity and response mode */

    pCompressionService->acceleratorNum = 0;

    /*
     * Set number of Intermediate Buffers to the maximum number of
     * Intermediate Buffers available
     */
    pCompressionService->numInterBuffs = DC_MAX_NUM_INTERMEDIATE_BUFFERS;

    status = Sal_StringParsing("Dc",
            pCompressionService->generic_service_info.instance,
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
    pCompressionService->bankNum = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

    status = Sal_StringParsing("Dc",
            pCompressionService->generic_service_info.instance,
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
    pCompressionService->isPolled = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

    if(SAL_RESP_POLL_CFG_FILE == pCompressionService->isPolled)
    {
         rx_resp_type = ICP_RESP_TYPE_POLL;
    }

    /*
     * This code is redundant, but we keep it here to make it more readable:
     *   Internally, we treat the epoll mode as the legacy interrupt mode.
     */
    if (SAL_RESP_EPOLL_CFG_FILE == pCompressionService->isPolled) {
         rx_resp_type = ICP_RESP_TYPE_IRQ;
    }

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
    pCompressionService->pkgID = Sal_Strtoul(adfGetParam,
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
    pCompressionService->nodeAffinity = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

   /* Next need to read the [AcceleratorX] section of the config file */
    status = Sal_StringParsing("Accelerator",
            pCompressionService->acceleratorNum,
            "", temp_string2);
    LAC_CHECK_STATUS(status);
    status = Sal_StringParsing("Bank",
            pCompressionService->bankNum,
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
    pCompressionService->coreAffinity = Sal_Strtoul(adfGetParam,
                                               NULL, SAL_CFG_BASE_DEC);

    status = Sal_StringParsing("Dc",
                    pCompressionService->generic_service_info.instance,
                    "NumConcurrentRequests", temp_string);
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

    numCompConcurrentReq = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    if (validateConcurrRequest(numCompConcurrentReq))
    {
        LAC_LOG_ERROR("Invalid NumConcurrentRequests, valid "\
                     "values {64, 128, 256, ... 32768, 65536}");
        return CPA_STATUS_FAIL;
    }

    /* ADF does not allow us to completely fill the ring for batch requests */
    pCompressionService->maxNumCompConcurrentReq =
        (numCompConcurrentReq - SAL_BATCH_SUBMIT_FREE_SPACE);

    /* 1. Create transport handles */
    status = Sal_StringParsing("Dc",
            pCompressionService->generic_service_info.instance, "RingTx",
            temp_string);
    LAC_CHECK_STATUS(status);

    /* Get msgSize and pass in to createHandle */
    status = icp_adf_cfgGetParamValue(device,
            LAC_CFG_SECTION_GENERAL,
            ICP_CFG_FW_MSG_SIZE_DC_TX_KEY,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Problem getting DC_TX size string from config table");
        return CPA_STATUS_FAIL;
    }
    msgSize = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

    if (msgSize != (LAC_QAT_DC_REQ_SZ_LW*LAC_LONG_WORD_IN_BYTES))
     {
         /* Data in config table (or in .conf file) is out of sync with
          * this version of SAL. Ignore value from config table */
         LAC_LOG_DEBUG2("This SAL builds DC msgs of %d bytes, "
                 "so ignore conf value (%d) for transCreateHandle",
                 (LAC_QAT_DC_REQ_SZ_LW*LAC_LONG_WORD_IN_BYTES),
                 msgSize);
         msgSize = (LAC_QAT_DC_REQ_SZ_LW*LAC_LONG_WORD_IN_BYTES);
     }

    status = icp_adf_transCreateHandle(device,
                 ICP_TRANS_TYPE_ETR,
                 section,
                 pCompressionService->acceleratorNum,
                 pCompressionService->bankNum,
                 temp_string,
                 lac_getRingType(SAL_RING_TYPE_DC),
                 NULL,
                 ICP_RESP_TYPE_NONE,
                 numCompConcurrentReq,
                 msgSize,
                 (icp_comms_trans_handle *)
                     &(pCompressionService->trans_handle_compression_tx));
    LAC_CHECK_STATUS(status);

    if(icp_adf_transGetRingNum(
                        pCompressionService->trans_handle_compression_tx,
                        &request_ring_id) != CPA_STATUS_SUCCESS)
    {
       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_tx);

       LAC_LOG_ERROR("Failed to get DC TX ring number");
       return CPA_STATUS_FAIL;
    }

    ring_number_in_bank = request_ring_id % ICP_DH895xCC_RINGS_PER_BANK;
    if((ring_number_in_bank != ICP_DH895xCC_DC_TX_RING_0) &&
       (ring_number_in_bank != ICP_DH895xCC_DC_TX_RING_1))
    {
        LAC_LOG_ERROR("Invalid ring number. Ring number must be 6 or 7\n");
    }

    status = Sal_StringParsing("Dc",
            pCompressionService->generic_service_info.instance, "RingRx",
            temp_string);
    if(CPA_STATUS_SUCCESS != status)
    {
        icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_tx);
        return status;
    }

    /* Get msgSize and pass in to createHandle */
    status = icp_adf_cfgGetParamValue(device,
            LAC_CFG_SECTION_GENERAL,
            ICP_CFG_FW_MSG_SIZE_DC_RX_KEY,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Problem getting DC_RX size string from config table");
        icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_tx);
        return status;
    };
    msgSize = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

    if (msgSize != (LAC_QAT_DC_RESP_SZ_LW*LAC_LONG_WORD_IN_BYTES))
     {
         /* Data in config table (or in .conf file) is out of sync with
          * this version of SAL. Ignore value from config table */
         LAC_LOG_DEBUG2("This SAL expects DC responses of %d bytes, "
                 "so ignore conf value (%d) for transCreateHandle",
                 (LAC_QAT_DC_RESP_SZ_LW*LAC_LONG_WORD_IN_BYTES),
                 msgSize);
         msgSize = (LAC_QAT_DC_RESP_SZ_LW*LAC_LONG_WORD_IN_BYTES);
     }

    status = icp_adf_transCreateHandle(device,
                 ICP_TRANS_TYPE_ETR,
                 section,
                 pCompressionService->acceleratorNum,
                 pCompressionService->bankNum,
                 temp_string,
                 lac_getRingType(SAL_RING_TYPE_NONE),
                 (icp_trans_callback)dcCompression_ProcessCallback,
                 rx_resp_type,
                 numCompConcurrentReq,
                 msgSize,
                 (icp_comms_trans_handle *)
                     &(pCompressionService->trans_handle_compression_rx));
    if(CPA_STATUS_SUCCESS != status)
    {
       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_tx);
       return status;
    }

    if(icp_adf_transGetRingNum(pCompressionService->trans_handle_compression_rx,
                                &response_ring_id) !=
                                CPA_STATUS_SUCCESS)
    {
       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_tx);

       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_rx);

       LAC_LOG_ERROR("Failed to get DC RX ring number");
       return CPA_STATUS_FAIL;
    }

    if((request_ring_id + ICP_DH895xCC_RX_RINGS_OFFSET) != response_ring_id)
    {
       LAC_LOG_ERROR("Rx ring number has to be Tx + 8");
       return CPA_STATUS_FAIL;
    }

    if(CPA_STATUS_SUCCESS != status)
    {
       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_tx);

       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_rx);
       return CPA_STATUS_FAIL;
    }

    /* 2. Allocates memory pools */

    /* Valid initialisation value for a pool ID */
    pCompressionService->compression_mem_pool = LAC_MEM_POOL_INIT_POOL_ID;

    status = Sal_StringParsing("Comp",
            pCompressionService->generic_service_info.instance, "_MemPool",
            compMemPool);
    if(CPA_STATUS_SUCCESS != status)
    {
       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_tx);

       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_rx);

       return status;
    }

    status = Lac_MemPoolCreate(&pCompressionService->compression_mem_pool,
                               compMemPool,
                               (numCompConcurrentReq+1),
                               sizeof(dc_compression_cookie_t),
                               LAC_64BYTE_ALIGNMENT,
                               CPA_FALSE,
                               pCompressionService->nodeAffinity);
    if(CPA_STATUS_SUCCESS != status)
    {
       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_tx);

       icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_rx);

       return status;
    }

    /* Init compression statistics */
    status = dcStatsInit(pCompressionService);
    if(CPA_STATUS_SUCCESS != status)
    {
        Lac_MemPoolDestroy(pCompressionService->compression_mem_pool);

        icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_tx);

        icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_rx);

        return status;
    }
    if(CPA_TRUE == pStatsCollection->bDcStatsEnabled)
    {
       /* Get instance name for stats */
        status = LAC_OS_MALLOC(&instance_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
        if(CPA_STATUS_SUCCESS != status)
        {
           Lac_MemPoolDestroy(pCompressionService->compression_mem_pool);

           icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_tx);

           icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_rx);

           return status;
        }

        status = Sal_StringParsing("Dc",
                    pCompressionService->generic_service_info.instance,
                    "Name", temp_string);
        if(CPA_STATUS_SUCCESS != status)
        {
            Lac_MemPoolDestroy(pCompressionService->compression_mem_pool);

            icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_tx);

            icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_rx);
            LAC_OS_FREE(instance_name);
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

            Lac_MemPoolDestroy(pCompressionService->compression_mem_pool);

            icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_tx);

            icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_rx);
            LAC_OS_FREE(instance_name);
            return status;
        }

        snprintf(instance_name,ADF_CFG_MAX_VAL_LEN_IN_BYTES,"%s",adfGetParam);

        status = LAC_OS_MALLOC(&pCompressionService->debug_file,
                              sizeof(debug_file_info_t));
        if(CPA_STATUS_SUCCESS != status)
        {
            Lac_MemPoolDestroy(pCompressionService->compression_mem_pool);

            icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_tx);

            icp_adf_transReleaseHandle(
                pCompressionService->trans_handle_compression_rx);
            LAC_OS_FREE(instance_name);
            return status;
        }

        osalMemSet(pCompressionService->debug_file,0,sizeof(debug_file_info_t));
        pCompressionService->debug_file->name = instance_name;
        pCompressionService->debug_file->seq_read = SalCtrl_CompresionDebug;
        pCompressionService->debug_file->private_data = pCompressionService;
        pCompressionService->debug_file->parent =
            pCompressionService->generic_service_info.debug_parent_dir;

        status = icp_adf_debugAddFile(device,
                      pCompressionService->debug_file);
        if(CPA_STATUS_SUCCESS != status)
        {
           Lac_MemPoolDestroy(pCompressionService->compression_mem_pool);

           icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_tx);

           icp_adf_transReleaseHandle(
               pCompressionService->trans_handle_compression_rx);
           LAC_OS_FREE(instance_name);
           LAC_OS_FREE(pCompressionService->debug_file);
           return status;
        }
    }
    pCompressionService->generic_service_info.stats = pStatsCollection;
    pCompressionService->generic_service_info.state =
      SAL_SERVICE_STATE_INITIALIZED;

    return status;
}

CpaStatus
SalCtrl_CompressionStart(icp_accel_dev_t* device, sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    sal_compression_service_t* pCompressionService =
            (sal_compression_service_t*) service;

    if (SAL_SERVICE_STATE_INITIALIZED !=
            pCompressionService->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call start\n");
        return CPA_STATUS_FAIL;
    }

    pCompressionService->generic_service_info.state = SAL_SERVICE_STATE_RUNNING;

    return status;
}

CpaStatus
SalCtrl_CompressionStop(icp_accel_dev_t* device, sal_service_t* service)
{
    sal_compression_service_t* pCompressionService =
            (sal_compression_service_t*) service;

    if (SAL_SERVICE_STATE_RUNNING !=
        pCompressionService->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call stop");
        return CPA_STATUS_FAIL;
    }

    if(icp_adf_is_dev_in_reset(device))
    {
        pCompressionService->generic_service_info.state =
          SAL_SERVICE_STATE_RESTARTING;
        return CPA_STATUS_SUCCESS;
    }

    pCompressionService->generic_service_info.state =
      SAL_SERVICE_STATE_SHUTTING_DOWN;
    return CPA_STATUS_RETRY;
}

CpaStatus
SalCtrl_CompressionShutdown(icp_accel_dev_t* device,
                            sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    sal_compression_service_t* pCompressionService =
            (sal_compression_service_t*) service;
    sal_statistics_collection_t *pStatsCollection =
        (sal_statistics_collection_t*)device->pQatStats;


    if ((SAL_SERVICE_STATE_INITIALIZED !=
          pCompressionService->generic_service_info.state) &&
        (SAL_SERVICE_STATE_SHUTTING_DOWN !=
          pCompressionService->generic_service_info.state) &&
        (SAL_SERVICE_STATE_RESTARTING !=
          pCompressionService->generic_service_info.state))
    {
        LAC_LOG_ERROR("Not in the correct state to call shutdown");
        return CPA_STATUS_FAIL;
    }

    Lac_MemPoolDestroy(pCompressionService->compression_mem_pool);

    status = icp_adf_transReleaseHandle(
            pCompressionService->trans_handle_compression_tx);
    LAC_CHECK_STATUS(status);

    status = icp_adf_transReleaseHandle(
            pCompressionService->trans_handle_compression_rx);
    LAC_CHECK_STATUS(status);

    if(CPA_TRUE == pStatsCollection->bDcStatsEnabled)
    {
       /* Clean stats */
        icp_adf_debugRemoveFile(pCompressionService->debug_file);
        LAC_OS_FREE(pCompressionService->debug_file->name);
        LAC_OS_FREE(pCompressionService->debug_file);
        pCompressionService->debug_file = NULL;
    }
    pCompressionService->generic_service_info.stats = NULL;
    dcStatsFree(pCompressionService);

    if(icp_adf_is_dev_in_reset(device))
    {
        pCompressionService->generic_service_info.state =
          SAL_SERVICE_STATE_RESTARTING;
        return CPA_STATUS_SUCCESS;
    }
    pCompressionService->generic_service_info.state =
      SAL_SERVICE_STATE_SHUTDOWN;
    return status;
}

CpaStatus
cpaDcGetStatusText(const CpaInstanceHandle dcInstance,
        const CpaStatus errStatus,
        Cpa8S *pStatusText)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(pStatusText);
#endif

#ifdef ICP_TRACE
    LAC_LOG3("Called with params (0x%lx, %d, 0x%lx)\n",
            (LAC_ARCH_UINT)dcInstance,
            errStatus,
            (LAC_ARCH_UINT)pStatusText);
#endif

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

CpaStatus cpaDcGetNumIntermediateBuffers(CpaInstanceHandle dcInstance,
    Cpa16U *pNumBuffers)
{
    CpaInstanceHandle insHandle = NULL;
    sal_compression_service_t* pService = NULL;

    if (CPA_INSTANCE_HANDLE_SINGLE == dcInstance) {
        insHandle = dcGetFirstHandle();
    }
    else {
        insHandle = dcInstance;
    }

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(insHandle);
    LAC_CHECK_NULL_PARAM(pNumBuffers);
#endif

    pService = (sal_compression_service_t*) insHandle;
    *pNumBuffers = pService->numInterBuffs;

#ifdef ICP_TRACE
    LAC_LOG3("Called with params (0x%lx, 0x%lx[%d])\n",
            (LAC_ARCH_UINT)insHandle,
            (LAC_ARCH_UINT)pNumBuffers,
            *pNumBuffers);
#endif
    return CPA_STATUS_SUCCESS;
}

CpaStatus
cpaDcStartInstance(CpaInstanceHandle instanceHandle,
    Cpa16U numBuffers,
    CpaBufferList **pIntermediateBufferPtrsArray)
{
    icp_qat_addr_width_t *pInterBuffPtrsArray = NULL;
    icp_qat_addr_width_t pArrayBufferListDescPhyAddr = 0;
    icp_qat_addr_width_t bufListDescPhyAddr;
    icp_qat_addr_width_t bufListAlignedPhyAddr;
    CpaFlatBuffer *pClientCurrFlatBuffer = NULL;
    icp_buffer_list_desc_t *pBufferListDesc = NULL;
    icp_flat_buffer_desc_t *pCurrFlatBufDesc = NULL;
   /* Structure initializer is supported by C99, but it is
    * not supported by some former Intel compiler.
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
    sal_compression_service_t* pService = NULL;
    CpaInstanceHandle insHandle = NULL;
    Cpa16U bufferIndex = 0;
    Cpa32U numFlatBuffers = 0;
    Cpa64U clientListSize = 0;
    CpaBufferList *pClientCurrentIntermediateBuffer = NULL;

    /* Check parameters */
#ifdef ICP_PARAM_CHECK
    Cpa16U bufferIndex2 = 0;
    CpaBufferList **pTempIntermediateBufferPtrsArray;
    Cpa64U lastClientListSize = 0;
#endif

    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle) {
        insHandle = dcGetFirstHandle();
    }
    else {
        insHandle = instanceHandle;
    }
    LAC_CHECK_NULL_PARAM(insHandle);

    status = cpaDcInstanceGetInfo2(insHandle, &info);
    if (CPA_STATUS_SUCCESS != status) {
        LAC_LOG_ERROR("Can not get instance info\n");
        return status;
    }

    dev = icp_adf_getAccelDevByAccelId(info.physInstId.packageId);
    if (NULL == dev) {
        LAC_LOG_ERROR("Can not find device for the instance\n");
        return CPA_STATUS_FAIL;
    }

    if (NULL == pIntermediateBufferPtrsArray) {
        /* Increment dev ref counter and return - DRAM is not used */
        icp_qa_dev_get(dev);
        return CPA_STATUS_SUCCESS;
    }

    if (0 == numBuffers)
    {
        /* Increment dev ref counter and return - DRAM is not used */
        icp_qa_dev_get(dev);
        return CPA_STATUS_SUCCESS;
    }

    pService = (sal_compression_service_t*) insHandle;

    /* Check parameters */
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(insHandle);

    if((numBuffers > 0) && (NULL == pIntermediateBufferPtrsArray))
    {
        LAC_LOG_ERROR("Invalid Intermediate Buffers Array pointer\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Check number of intermediate buffers allocated by user */
    if((pService->numInterBuffs != numBuffers))
    {
        LAC_LOG_ERROR("Invalid number of buffers\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    pTempIntermediateBufferPtrsArray = pIntermediateBufferPtrsArray;
    for(bufferIndex = 0; bufferIndex < numBuffers; bufferIndex++)
    {
        if(NULL == *pTempIntermediateBufferPtrsArray)
        {
            LAC_LOG_ERROR("Intermediate Buffer - Invalid Buffer List pointer\n");
            return CPA_STATUS_INVALID_PARAM;
        }

        if(NULL == (*pTempIntermediateBufferPtrsArray)->pBuffers)
        {
            LAC_LOG_ERROR("Intermediate Buffer - Invalid Flat Buffer descriptor pointer\n");
            return CPA_STATUS_INVALID_PARAM;
        }

        if(NULL == (*pTempIntermediateBufferPtrsArray)->pPrivateMetaData)
        {
            LAC_LOG_ERROR("Intermediate Buffer - Invalid Private MetaData descriptor pointer\n");
            return CPA_STATUS_INVALID_PARAM;
        }

        clientListSize = 0;
        for(bufferIndex2 = 0; bufferIndex2 < (*pTempIntermediateBufferPtrsArray)->numBuffers; bufferIndex2++)
        {

            if((0 != (*pTempIntermediateBufferPtrsArray)->pBuffers[bufferIndex2].dataLenInBytes) &&
                    NULL == (*pTempIntermediateBufferPtrsArray)->pBuffers[bufferIndex2].pData)
            {
                LAC_LOG_ERROR("Intermediate Buffer - Invalid Flat Buffer pointer\n");
                return CPA_STATUS_INVALID_PARAM;
            }

            clientListSize += (*pTempIntermediateBufferPtrsArray)->pBuffers[bufferIndex2].dataLenInBytes;
        }

        if(bufferIndex != 0)
        {
            if(lastClientListSize != clientListSize)
            {
                LAC_LOG_ERROR("SGLs have to be of the same size\n");
                return CPA_STATUS_INVALID_PARAM;
            }
        }
        else
        {
            lastClientListSize = clientListSize;
        }
        pTempIntermediateBufferPtrsArray++;
    }

    SAL_CHECK_ADDR_TRANS_SETUP(insHandle);
#endif

    /* Allocate array of physical pointers to icp_buffer_list_desc_t */
    status = LAC_OS_CAMALLOC(&pInterBuffPtrsArray,
            (numBuffers * sizeof(icp_qat_addr_width_t)),
                                LAC_64BYTE_ALIGNMENT, pService->nodeAffinity);
    if (CPA_STATUS_SUCCESS != status) {
        LAC_LOG_ERROR("Can not allocate Intermediate Buffers array\n");
        return status;
    }

    /* Get physical address of the intermediate buffer pointers array */
    pArrayBufferListDescPhyAddr = LAC_MEM_CAST_PTR_TO_UINT64(
                    LAC_OS_VIRT_TO_PHYS_INTERNAL(pInterBuffPtrsArray));

    pService->pInterBuffPtrsArray = pInterBuffPtrsArray;
    pService->pInterBuffPtrsArrayPhyAddr = pArrayBufferListDescPhyAddr;

    /* Get the full size of the buffer list */
    /* Assumption: all the SGLs allocated by the user have the same size */
    clientListSize = 0;
    for(bufferIndex = 0; bufferIndex < (*pIntermediateBufferPtrsArray)->numBuffers; bufferIndex++)
    {
        clientListSize += ((*pIntermediateBufferPtrsArray)->pBuffers[bufferIndex].dataLenInBytes);
    }
    pService->minInterBuffSizeInBytes = clientListSize;

    for(bufferIndex = 0; bufferIndex < numBuffers; bufferIndex++)
    {

        /* Get pointer to the client Intermediate Buffer List (CpaBufferList) */
        pClientCurrentIntermediateBuffer = *pIntermediateBufferPtrsArray;

        /* Get number of flat buffers in the buffer list */
        numFlatBuffers = pClientCurrentIntermediateBuffer->numBuffers;

        /* Get pointer to the client array of CpaFlatBuffers */
        pClientCurrFlatBuffer = pClientCurrentIntermediateBuffer->pBuffers;

        /* Calculate Physical address of current private SGL */
        bufListDescPhyAddr = LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                                         (*pService), pClientCurrentIntermediateBuffer->pPrivateMetaData);
        if (bufListDescPhyAddr == 0) {
            LAC_LOG_ERROR("Unable to get the physical address of the metadata\n");
            return CPA_STATUS_FAIL;
        }

        /* Align SGL physical address */
        bufListAlignedPhyAddr = LAC_ALIGN_POW2_ROUNDUP(bufListDescPhyAddr,
                                                ICP_DESCRIPTOR_ALIGNMENT_BYTES);

        /* Set physical address of the Intermediate Buffer SGL in the SGLs array */
        *pInterBuffPtrsArray = LAC_MEM_CAST_PTR_TO_UINT64(bufListAlignedPhyAddr);

        /* Calculate (virtual) offset to the buffer list descriptor */
        pBufferListDesc = (icp_buffer_list_desc_t *)(
                        (LAC_ARCH_UINT)pClientCurrentIntermediateBuffer->pPrivateMetaData +
                        (LAC_ARCH_UINT)(bufListAlignedPhyAddr - bufListDescPhyAddr));

        /* Set number of flat buffers in the physical Buffer List descriptor */
        pBufferListDesc->numBuffers = numFlatBuffers;

        /* Go past the Buffer List descriptor to the list of buffer descriptors */
        pCurrFlatBufDesc = (icp_flat_buffer_desc_t *)(
                                            (pBufferListDesc->phyBuffers));

        /* Loop for each flat buffer in the SGL */
        while (0 != numFlatBuffers)
        {
            /* Set length of the current flat buffer */
            pCurrFlatBufDesc->dataLenInBytes =
                                pClientCurrFlatBuffer->dataLenInBytes;

            /* Set physical address of the flat buffer */
            pCurrFlatBufDesc->phyBuffer = LAC_MEM_CAST_PTR_TO_UINT64(
                LAC_OS_VIRT_TO_PHYS_EXTERNAL((*pService),
                    pClientCurrFlatBuffer->pData));

            if (pCurrFlatBufDesc->phyBuffer == 0)
            {
                LAC_LOG_ERROR("Unable to get the physical address of the flat"
                 " buffer\n");
                return CPA_STATUS_FAIL;
            }

            pCurrFlatBufDesc++;
            pClientCurrFlatBuffer++;
            numFlatBuffers--;

        }
        pIntermediateBufferPtrsArray++;
        pInterBuffPtrsArray++;
    }

    pService->generic_service_info.isInstanceStarted = CPA_TRUE;

    /* Increment dev ref counter */
    icp_qa_dev_get(dev);
    return CPA_STATUS_SUCCESS;
}

CpaStatus
cpaDcStopInstance(CpaInstanceHandle instanceHandle)
{
    CpaInstanceHandle insHandle = NULL;
   /* Structure initializer is supported by C99, but it is
    * not supported by some former Intel compiler.
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
    sal_compression_service_t *pService = NULL;

#ifdef ICP_TRACE
    LAC_LOG1("Called with params (0x%lx)\n", (LAC_ARCH_UINT)instanceHandle);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
        insHandle = dcGetFirstHandle();
    }
    else
    {
        insHandle = instanceHandle;
    }

    LAC_CHECK_NULL_PARAM(insHandle);
    pService = (sal_compression_service_t*) insHandle;

    /* Free Intermediate Buffer Pointers Array */
    if(pService->pInterBuffPtrsArray != NULL)
    {
        LAC_OS_CAFREE(pService->pInterBuffPtrsArray);
        pService->pInterBuffPtrsArray = 0;
    }

    pService->pInterBuffPtrsArrayPhyAddr = 0;

    status = cpaDcInstanceGetInfo2(insHandle, &info);
    if (CPA_STATUS_SUCCESS != status) {
        LAC_LOG_ERROR("Can not get instance info\n");
        return status;
    }
    dev = icp_adf_getAccelDevByAccelId(info.physInstId.packageId);
    if (NULL == dev) {
        LAC_LOG_ERROR("Can not find device for the instance\n");
        return CPA_STATUS_FAIL;
    }

    pService->generic_service_info.isInstanceStarted = CPA_FALSE;

    /* Decrement dev ref counter */
    icp_qa_dev_put(dev);
    return CPA_STATUS_SUCCESS;
}

CpaStatus
cpaDcGetNumInstances(Cpa16U *pNumInstances)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_accel_dev_t **pAdfInsts = NULL;
    icp_accel_dev_t* dev_addr = NULL;
    sal_t* base_addr = NULL;
    sal_list_t* list_temp = NULL;
    Cpa16U num_accel_dev = 0;
    Cpa16U num = 0;
    Cpa16U i = 0;

    LAC_CHECK_NULL_PARAM(pNumInstances);

    /* Get the number of accel_dev in the system */
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

    /* Get ADF to return accel_devs with dc enabled */
    status = icp_amgr_getAllAccelDevByCapabilities(
                                        ICP_ACCEL_CAPABILITIES_COMPRESSION,
                                                pAdfInsts, &num_accel_dev);
    if(CPA_STATUS_SUCCESS == status)
    {
        for(i = 0; i < num_accel_dev; i++)
        {
            dev_addr = (icp_accel_dev_t*) pAdfInsts[i];
            if(NULL != dev_addr)
            {
                base_addr = dev_addr->pSalHandle;
                if(NULL != base_addr)
                {
                    list_temp = base_addr->compression_services;
                    while(NULL != list_temp)
                    {
                        num++;
                        list_temp=SalList_next(list_temp);
                    }
                }
            }
        }

        *pNumInstances = num;
    }

    osalMemFree(pAdfInsts);

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx[%d])\n", (LAC_ARCH_UINT)pNumInstances,
            *pNumInstances);
#endif
    return status;
}

CpaStatus
cpaDcGetInstances(Cpa16U numInstances, CpaInstanceHandle* dcInstances)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_accel_dev_t **pAdfInsts = NULL;
    icp_accel_dev_t* dev_addr = NULL;
    sal_t* base_addr = NULL;
    sal_list_t* list_temp = NULL;
    Cpa16U num_accel_dev = 0;
    Cpa16U index = 0;
    Cpa16U i = 0;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (%d, 0x%lx)\n",
            numInstances,
            (LAC_ARCH_UINT)dcInstances);
#endif

    LAC_CHECK_NULL_PARAM(dcInstances);
    if (0 == numInstances)
    {
        LAC_INVALID_PARAM_LOG("numInstances is 0");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Get the number of accel_dev in the system */
    status = icp_amgr_getNumInstances(&num_accel_dev);
    LAC_CHECK_STATUS(status);

    /* Allocate memory to store addr of accel_devs */
    pAdfInsts = osalMemAlloc(num_accel_dev*sizeof(icp_accel_dev_t*));
    if(pAdfInsts == NULL)
    {
        LAC_LOG_ERROR("Failed to allocate dev instance memory");
        return CPA_STATUS_RESOURCE;
    }

    num_accel_dev=0;
    /* Get ADF to return accel_devs with dc enabled */
    status = icp_amgr_getAllAccelDevByCapabilities(
                            ICP_ACCEL_CAPABILITIES_COMPRESSION,
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
                    list_temp = base_addr->compression_services;
                    while(NULL != list_temp)
                    {
                        if(index > (numInstances-1))
                        {
                           break;
                        }

                        dcInstances[index++] = SalList_getObject(list_temp);
                        list_temp=SalList_next(list_temp);
                        index++;
                    }
                }
            }
        }

        if(numInstances > index)
        {
            LAC_LOG_ERROR1("Only %d dc instances available", index);
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
                list_temp = base_addr->compression_services;
                while(NULL != list_temp)
                {
                    if(index > (numInstances-1))
                    {
                        break;
                    }

                    dcInstances[index] = SalList_getObject(list_temp);
                    list_temp=SalList_next(list_temp);
                    index++;
                }
            }
        }
    }

    osalMemFree(pAdfInsts);

    return status;
}

CpaStatus
cpaDcInstanceGetInfo2(const CpaInstanceHandle instanceHandle,
                     CpaInstanceInfo2 *pInstanceInfo2)
{
    sal_compression_service_t* pCompressionService = NULL;
    CpaInstanceHandle insHandle = NULL;
    icp_accel_dev_t *dev = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    char keyStr[ADF_CFG_MAX_KEY_LEN_IN_BYTES] = {0};
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char * section = DYN_SEC;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle,
            (LAC_ARCH_UINT)pInstanceInfo2);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
         insHandle = dcGetFirstHandle();
    }
    else
    {
         insHandle = instanceHandle;
    }

    LAC_CHECK_NULL_PARAM(insHandle);
    SAL_CHECK_INSTANCE_TYPE(insHandle, SAL_SERVICE_TYPE_COMPRESSION);
    LAC_CHECK_NULL_PARAM(pInstanceInfo2);

    LAC_OS_BZERO(pInstanceInfo2, sizeof(CpaInstanceInfo2));
    pInstanceInfo2->accelerationServiceType =
        CPA_ACC_SVC_TYPE_DATA_COMPRESSION;

    snprintf((char *) pInstanceInfo2->vendorName,
               CPA_INST_VENDOR_NAME_SIZE,
               "%s", SAL_INFO2_VENDOR_NAME);
    pInstanceInfo2->vendorName[CPA_INST_VENDOR_NAME_SIZE - 1] = '\0';

    snprintf((char *) pInstanceInfo2->partName,
               CPA_INST_PART_NAME_SIZE,
               "%s", SAL_INFO2_PART_NAME);
    pInstanceInfo2->partName[CPA_INST_PART_NAME_SIZE - 1] = '\0';

    snprintf((char *) pInstanceInfo2->swVersion,
                   CPA_INST_SW_VERSION_SIZE,
                   "Version %d.%d", SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
                                    SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER);
    pInstanceInfo2->swVersion[CPA_INST_SW_VERSION_SIZE - 1] = '\0';

    /* Note we can safely read the contents of the compression service instance
       here because icp_amgr_getAccelDevByCapabilities() only returns devs
       that have started */
    pCompressionService = (sal_compression_service_t*) insHandle;
    pInstanceInfo2->physInstId.packageId = pCompressionService->pkgID;
    pInstanceInfo2->physInstId.acceleratorId =
        pCompressionService->acceleratorNum;
    pInstanceInfo2->physInstId.executionEngineId = 0;
    pInstanceInfo2->physInstId.busAddress =
                 icp_adf_get_busAddress(pInstanceInfo2->physInstId.packageId);

    /*set coreAffinity to zero before use */
    LAC_OS_BZERO(pInstanceInfo2->coreAffinity,
                                          sizeof(pInstanceInfo2->coreAffinity));
    CPA_BITMAP_BIT_SET(pInstanceInfo2->coreAffinity,
        pCompressionService->coreAffinity);

    pInstanceInfo2->nodeAffinity = pCompressionService->nodeAffinity;

    if(SAL_SERVICE_STATE_RUNNING ==
       pCompressionService->generic_service_info.state)
    {
        pInstanceInfo2->operState = CPA_OPER_STATE_UP;
    }
    else
    {
        pInstanceInfo2->operState = CPA_OPER_STATE_DOWN;
    }

    pInstanceInfo2->requiresPhysicallyContiguousMemory = CPA_TRUE;

    if(SAL_RESP_POLL_CFG_FILE == pCompressionService->isPolled)
    {
        pInstanceInfo2->isPolled = CPA_TRUE;
    }
    else if(SAL_RESP_EPOLL_CFG_FILE == pCompressionService->isPolled)
    {
        /*
         * For the SAL_RESP_EPOLL_CFG_FILE case, we also return CPA_TRUE
         * for @pInstanceInfo2->isPolled.
         */
        pInstanceInfo2->isPolled = CPA_TRUE;
    }

    pInstanceInfo2->isOffloaded = CPA_TRUE;
    /* Get the instance name from the config file */
    dev = icp_adf_getAccelDevByAccelId(pCompressionService->pkgID);
    if (NULL == dev || NULL == dev->type) {
        LAC_LOG_ERROR("Can not find device for the instance\n");
        LAC_OS_BZERO(pInstanceInfo2, sizeof(CpaInstanceInfo2));
        return CPA_STATUS_FAIL;
    }

    snprintf((char *) pInstanceInfo2->partName,
                CPA_INST_PART_NAME_SIZE,
                SAL_INFO2_PART_NAME, dev->type);
    pInstanceInfo2->partName[CPA_INST_PART_NAME_SIZE - 1] = '\0';

    if(CPA_FALSE == pCompressionService->generic_service_info.is_dyn)
    {
        section = icpGetProcessName();
    }

    status = Sal_StringParsing("Dc",
                    pCompressionService->generic_service_info.instance,
                    "Name", keyStr);
    LAC_CHECK_STATUS(status);
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

CpaStatus
cpaDcQueryCapabilities(CpaInstanceHandle dcInstance,
        CpaDcInstanceCapabilities *pInstanceCapabilities)
{
    CpaInstanceHandle insHandle = NULL;
    CpaStatus status;
    CpaBoolean report_parity = CPA_FALSE;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)dcInstance,
            (LAC_ARCH_UINT)pInstanceCapabilities);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == dcInstance)
    {
         insHandle = dcGetFirstHandle();
    }
    else
    {
         insHandle = dcInstance;
    }

    /* Check parameters */
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(insHandle);
    SAL_CHECK_INSTANCE_TYPE(insHandle, SAL_SERVICE_TYPE_COMPRESSION);
    LAC_CHECK_NULL_PARAM(pInstanceCapabilities);
#endif

    /* Set compression capabilities */
    pInstanceCapabilities->statefulLZSCompression = CPA_FALSE;
    pInstanceCapabilities->statefulLZSDecompression = CPA_FALSE;
    pInstanceCapabilities->statelessLZSCompression = CPA_TRUE;
    pInstanceCapabilities->statelessLZSDecompression = CPA_TRUE;
    pInstanceCapabilities->statefulLZSSCompression = CPA_FALSE;
    pInstanceCapabilities->statefulLZSSDecompression = CPA_FALSE;
    pInstanceCapabilities->statelessLZSSCompression = CPA_FALSE;
    pInstanceCapabilities->statelessLZSSDecompression = CPA_FALSE;
    pInstanceCapabilities->statefulELZSCompression = CPA_FALSE;
    pInstanceCapabilities->statefulELZSDecompression = CPA_FALSE;
    pInstanceCapabilities->statelessELZSCompression = CPA_FALSE;
    pInstanceCapabilities->statelessELZSDecompression = CPA_FALSE;
    pInstanceCapabilities->statefulDeflateCompression = CPA_TRUE;
    pInstanceCapabilities->statefulDeflateDecompression = CPA_TRUE;
    pInstanceCapabilities->statelessDeflateCompression = CPA_TRUE;
    pInstanceCapabilities->statelessDeflateDecompression = CPA_TRUE;
    pInstanceCapabilities->checksumCRC32 = CPA_TRUE;
    pInstanceCapabilities->checksumAdler32 = CPA_TRUE;
#ifndef ICP_DC_DYN_NOT_SUPPORTED
    pInstanceCapabilities->dynamicHuffman = CPA_TRUE;
#else
    pInstanceCapabilities->dynamicHuffman = CPA_FALSE;
#endif
    pInstanceCapabilities->precompiledHuffman = CPA_FALSE;
    pInstanceCapabilities->dynamicHuffmanBufferReq = CPA_TRUE;
    pInstanceCapabilities->autoSelectBestHuffmanTree = CPA_TRUE;
    pInstanceCapabilities->validWindowSizeMaskCompression = (WINDOW_SIZE_8K_MASK | WINDOW_SIZE_32K_MASK);
    pInstanceCapabilities->validWindowSizeMaskDecompression = (WINDOW_SIZE_8K_MASK | WINDOW_SIZE_32K_MASK);

    /* Read key value from adf and configure capability value */
    status = SalCtrl_GetParityReportValue(insHandle, &report_parity);

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Problem getting PARITY_ERROR key - default to false");
        pInstanceCapabilities->reportParityError = CPA_FALSE;
    }
    else
    {
        pInstanceCapabilities->reportParityError = report_parity;
    }

    return CPA_STATUS_SUCCESS;
}

CpaStatus
cpaDcSetAddressTranslation(const CpaInstanceHandle instanceHandle,
        CpaVirtualToPhysical virtual2Physical)
{
    sal_service_t *pService = NULL;
    CpaInstanceHandle insHandle = NULL;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle,
            (LAC_ARCH_UINT)virtual2Physical);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
        insHandle = dcGetFirstHandle();
    }
    else
    {
        insHandle = instanceHandle;
    }

    /* Check parameters */
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(insHandle);
    SAL_CHECK_INSTANCE_TYPE(insHandle, SAL_SERVICE_TYPE_COMPRESSION);
    LAC_CHECK_NULL_PARAM(virtual2Physical);
#endif

    pService = (sal_service_t *) insHandle;

    pService->virt2PhysClient = virtual2Physical;

    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup cpaDcCommon
 * Data compression specific polling function which polls a DC instance.
 *****************************************************************************/

CpaStatus
icp_sal_DcPollInstance(CpaInstanceHandle instanceHandle_in,
                  Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_compression_service_t *dc_handle = NULL;
    sal_service_t *gen_handle = NULL;
    icp_comms_trans_handle trans_hndTable[DC_NUM_RX_RINGS];

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        dc_handle = (sal_compression_service_t *)dcGetFirstHandle();
    }
    else
    {
        dc_handle = (sal_compression_service_t *)instanceHandle_in;
    }

    LAC_CHECK_NULL_PARAM(dc_handle);
    SAL_RUNNING_CHECK(dc_handle);

    gen_handle = &(dc_handle->generic_service_info);
    if(SAL_SERVICE_TYPE_COMPRESSION != gen_handle->type)
    {
        LAC_LOG_ERROR("The instance handle is the wrong type");
        return CPA_STATUS_FAIL;
    }

    /*
     * From the instanceHandle we must get the trans_handle and send
     * down to adf for polling.
     * Populate our trans handle table with the appropriate handles.
     */
    trans_hndTable[0] = dc_handle->trans_handle_compression_rx;

    /* Call adf to do the polling. */
    status = icp_adf_pollInstance(trans_hndTable, DC_NUM_RX_RINGS,
                                  response_quota);
    return status;
}

/**
 ******************************************************************************
 * @ingroup cpaDcCommon
 *****************************************************************************/
CpaStatus
cpaDcInstanceSetNotificationCb(
    const CpaInstanceHandle instanceHandle,
    const CpaDcInstanceNotificationCbFunc pInstanceNotificationCb,
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

CpaInstanceHandle
dcGetFirstHandle(void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    static icp_accel_dev_t *adfInsts[ADF_MAX_DEVICES] = {0};
    CpaInstanceHandle dcInst = NULL;
    icp_accel_dev_t* dev_addr = NULL;
    sal_t* base_addr = NULL;
    sal_list_t* list_temp = NULL;
    Cpa16U i, num_dc = 0;


    /* Only need 1 dev with compression enabled - so check all devices */
    status = icp_amgr_getAllAccelDevByCapabilities(
                             ICP_ACCEL_CAPABILITIES_COMPRESSION,
                             adfInsts, &num_dc);
    if((0 == num_dc) || (CPA_STATUS_SUCCESS != status))
    {
         LAC_LOG("No compression devices enabled in the system\n");
         return dcInst;
    }

    for(i=0; i<num_dc; i++)
    {
        dev_addr = (icp_accel_dev_t*) adfInsts[i];
        if(NULL != dev_addr)
        {
            base_addr = dev_addr->pSalHandle;
            if(NULL != base_addr)
            {
                list_temp = base_addr->compression_services;
                if(NULL != list_temp)
                {
                    dcInst = SalList_getObject(list_temp);
                    break;
                }
            }
        }
    }
    return dcInst;
}

/**
 ******************************************************************************
 * @ingroup cpaDcCommon
 * Data compression specific function which returns a file descriptor
 * corresponding to a DC instance.
 *****************************************************************************/
CpaStatus icp_sal_DcGetFileDescriptor(CpaInstanceHandle instanceHandle_in, int *fd)
{
    sal_compression_service_t *dc_handle = NULL;
    sal_service_t *gen_handle=NULL;

    LAC_CHECK_NULL_PARAM(fd);
    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        dc_handle = (sal_compression_service_t *)dcGetFirstHandle();
    }
    else
    {
        dc_handle = (sal_compression_service_t *)instanceHandle_in;
    }
    LAC_CHECK_NULL_PARAM(dc_handle);
    SAL_RUNNING_CHECK(dc_handle);

    gen_handle = &(dc_handle->generic_service_info);
    if(SAL_SERVICE_TYPE_COMPRESSION != gen_handle->type)
    {
        LAC_LOG_ERROR("The instance handle is the wrong type");
        return CPA_STATUS_FAIL;
    }

    if (SAL_RESP_EPOLL_CFG_FILE != dc_handle->isPolled) {
        return CPA_STATUS_UNSUPPORTED;
    }

    return icp_adf_getTransHandleFileDesc(dc_handle->trans_handle_compression_rx,
                                     fd);
}

CpaStatus icp_sal_DcPutFileDescriptor(CpaInstanceHandle instanceHandle_in, int fd)
{
    sal_compression_service_t *dc_handle = NULL;
    sal_service_t *gen_handle=NULL;

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        dc_handle = (sal_compression_service_t *)dcGetFirstHandle();
    }
    else
    {
        dc_handle = (sal_compression_service_t *)instanceHandle_in;
    }
    LAC_CHECK_NULL_PARAM(dc_handle);

    gen_handle = &(dc_handle->generic_service_info);
    if(SAL_SERVICE_TYPE_COMPRESSION != gen_handle->type)
    {
        LAC_LOG_ERROR("The instance handle is the wrong type");
        return CPA_STATUS_FAIL;
    }

    return icp_adf_putTransHandleFileDesc(dc_handle->trans_handle_compression_rx,
                                     fd);
}
