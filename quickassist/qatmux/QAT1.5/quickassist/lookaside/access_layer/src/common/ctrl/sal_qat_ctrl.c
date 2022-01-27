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

/**
 ***************************************************************************
 * @file sal_qat_ctrl.c     Qat instance handling functions
 *
 * @ingroup SalCtrl
 *
 ***************************************************************************/
/* Include cpa header file. */
#include "cpa.h"

/* Log and Osal includes. */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "Osal.h"
#include "lac_list.h"

/* ADF includes */
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_ae.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_init.h"
#include "icp_qat_fw_admin.h"

/* SAL includes */
#include "lac_sal_types.h"
#include "lac_sal_types_qat_ctrl.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"

/* QAT includes */
#include "qat_priority.h"
#include "qat_ctrl.h"
#include "qat_init.h"
#include "qat_rings.h"
#include "qat_admin.h"
#include "qat_init_defs.h"
#include "sal_statistics.h"
#include "sal_string_parse.h"

/* Size of the Ring table in bytes (128 bits) */
#define INIT_RING_MASK_TABLE_BYTES_SZ 16

/* Admin ring numbers */
#define ADMIN_RING_TX 0
#define ADMIN_RING_RX 1

/* Proc debug defines */
#define DEBUG_QAT_NAME "qat%d"
#define DEBUG_QAT_NAME_LEN 5

/* First usable AE in case when a separate
 * AE is used to run dispatcher threads
 * and all the others are running working
 * threads. If that will be firmware model
 * this needs to be changes to 1 */
#define SAL_FIRST_USABLE_AE 0

/**
*******************************************************************************
 * @ingroup SalCtrl
 * @description
 *      Initialize the Ring Table and allocate resources
 *      depending on which services are enabled and on the
 *      number of AEs available. The (Master) Ring table is of size
 *      a sub ring table times the number of AEs. The Ring Table
 *      is then sent down to the AE so that it knows which service
 *      it supports and the rings it will use. Depending on the
 *      the services enabled we then set the service specific
 *      ring tables to specific access points. For example if we have
 *      four AEs on this instance and all services enabled we have
 *      the following situation.
 *
 *                                   pMasterRingTable
 *                                 ____________________
 * pCyARingTable----------------->|                    |
 *                                |                    |
 *                                |    CyA             |
 *                                |                    |
 *                                |____________________|
 * pCyBRingTable----------------->|                    |
 *                                |                    |
 *                                |    CyB             |
 *                                |                    |
 *                                |____________________|
 * pDcRingTable------------------>|                    |
 *                                |    Dc              |
 *                                |                    |
 *                                |                    |
 *                                |____________________|
 *
 *
 *
 * @context
 *      This function is called from SalCtrl_QatEventInit.
 *
 * @assumptions
 *      A call must first be made to get the services enabled and at least
 *      one service must be enabled.
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @param[in]  instance    The specific logical instance which will
 *                         run the service
 * @param[in]  cya_enabled  Flag for first crypto engine enabled
 * @param[in]  cyb_enabled  Flag for second crypto engine enabled
 * @param[in]  dc_enabled  Flag for compression enabled
 *
 * @retVal                 None
 *
 *****************************************************************************/

STATIC CpaStatus
QatCtrl_InitializeRingTable(CpaInstanceHandle instanceHandle,
                            CpaBoolean cya_enabled,
                            CpaBoolean cyb_enabled,
                            CpaBoolean dc_enabled)
{

    Cpa32U ae_one_index=0, ae_two_index=1;
    sal_qat_service_t *qat_instance = (sal_qat_service_t *)instanceHandle;
    /* The ring number to build the ring polling mask needs to be in the
       range 0-127 */
    Cpa32U ringNumPolling = ((qat_instance->ringNumTx)%SAL_RINGS_NUM_PER_QAT);

    /*
     * The first case is the case of one AE.
     * Any and all services enabled will run on
     * this AE and hence the weights are set
     * accordingly.
     */
    if(NUM_AES_ONE == qat_instance->num_aes_per_qat)
    {
        if(cya_enabled)
        {
            qat_instance->pCyARingTable =
              &(qat_instance->pMasterRingTable[ae_one_index]);
            ICP_QAT_FW_INIT_RING_MASK_SET((
                      *(qat_instance->pCyARingTable)), ringNumPolling);
            qat_instance->serviceMask[ae_one_index]
                                          |= SAL_SERVICE_TYPE_CRYPTO_A;
        }
        if(cyb_enabled)
        {
            qat_instance->pCyBRingTable =
              &(qat_instance->pMasterRingTable[ae_one_index]);
            ICP_QAT_FW_INIT_RING_MASK_SET((
                      *(qat_instance->pCyBRingTable)), ringNumPolling);
            qat_instance->serviceMask[ae_one_index]
                                          |= SAL_SERVICE_TYPE_CRYPTO_B;
        }
    }
    /*
     * For the case of 2 AEs things are more complex.
     * If cyA is enabled then it gets it's own AE.
     * If cyB and Dc are enabled as well then they share AE 1
     * if cyB is not enabled then Dc is on AE 1 (if dc is enabled)
     * The 3 scenario is for HAPs only and CKK will always have 4AEs per accelerator
     * and nano is crypto only
     */
    else if(NUM_AES_TWO == qat_instance->num_aes_per_qat)
    {
        if(cya_enabled)
        {
            qat_instance->pCyARingTable =
              &(qat_instance->pMasterRingTable[ae_one_index]);
            ICP_QAT_FW_INIT_RING_MASK_SET((
                      *(qat_instance->pCyARingTable)), ringNumPolling);
            qat_instance->serviceMask[ae_one_index]
                                          |= SAL_SERVICE_TYPE_CRYPTO_A;
        }
        if(cyb_enabled)
        {
            qat_instance->pCyBRingTable =
              &(qat_instance->pMasterRingTable[ae_two_index]);
            ICP_QAT_FW_INIT_RING_MASK_SET((
                      *(qat_instance->pCyBRingTable)), ringNumPolling);
            qat_instance->serviceMask[ae_two_index]
                                          |= SAL_SERVICE_TYPE_CRYPTO_B;
        }
        /* Only for HAPs testing
         * Remove when B0 will be there */
        if(dc_enabled)
        {
            qat_instance->pDcRingTable =
              &(qat_instance->pMasterRingTable[ae_two_index]);
            ICP_QAT_FW_INIT_RING_MASK_SET((
                      *(qat_instance->pDcRingTable)), ringNumPolling);
            qat_instance->serviceMask[ae_two_index] |=
              SAL_SERVICE_TYPE_COMPRESSION;
        }
    }
    /*
     * The case of 4 AEs is simple. Each service enabled
     * gets its own AE.
     */
    else if(NUM_AES_FOUR == qat_instance->num_aes_per_qat)
    {
        Cpa32U current_ae = SAL_FIRST_USABLE_AE;
        if(cya_enabled)
        {
            qat_instance->pCyARingTable =
              &(qat_instance->pMasterRingTable[current_ae]);
            ICP_QAT_FW_INIT_RING_MASK_SET((
                      *(qat_instance->pCyARingTable)), ringNumPolling);
            qat_instance->serviceMask[current_ae] |= SAL_SERVICE_TYPE_CRYPTO_A;
            current_ae++;
        }
        if(cyb_enabled)
        {
            qat_instance->pCyBRingTable =
              &(qat_instance->pMasterRingTable[current_ae]);
            ICP_QAT_FW_INIT_RING_MASK_SET((
                      *(qat_instance->pCyBRingTable)), ringNumPolling);
            qat_instance->serviceMask[current_ae] |= SAL_SERVICE_TYPE_CRYPTO_B;
            current_ae++;
        }
        if(dc_enabled)
        {
            qat_instance->pDcRingTable =
              &(qat_instance->pMasterRingTable[current_ae]);
            ICP_QAT_FW_INIT_RING_MASK_SET((
                      *(qat_instance->pDcRingTable)), ringNumPolling);
            qat_instance->serviceMask[current_ae] |=
                                                 SAL_SERVICE_TYPE_COMPRESSION;
            current_ae++;
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Prints statistics for a QAT instance
 */
STATIC int QatCtrl_Debug(void* private_data, char* data, int size, int offset)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    qat_admin_stats_t qatStats = {NULL, NULL};
    sal_qat_service_t *qatInstance = (sal_qat_service_t*)private_data;
    Cpa32S len = 0;
    int i=0;

    if ((SAL_SERVICE_STATE_RUNNING != qatInstance->generic_service_info.state) ||
	(CPA_FALSE == icp_adf_isDevStarted(qatInstance->debug->accel_dev)))
    {
        len += snprintf(data+len, size-len, "Device not started\n");
        return 0;
    }
    /* Alloc memory for the stats */
    status = LAC_OS_CAMALLOC(&(qatStats.numSent),
                            (qatInstance->num_aes_per_qat)
                            * sizeof(*(qatStats.numSent)),
                            LAC_64BYTE_ALIGNMENT, qatInstance->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate qatInstance->numSent memory\n");
        return (-1);
    }

    status = LAC_OS_CAMALLOC(&(qatStats.numRec),
                            (qatInstance->num_aes_per_qat)
                            * sizeof(*(qatStats.numRec)),
                            LAC_64BYTE_ALIGNMENT, qatInstance->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate qatInstance->numRec memory\n");
        LAC_OS_CAFREE(qatStats.numSent);
        return (-1);
    }

    /* Zero the stats */
    for(i=0; i< qatInstance->num_aes_per_qat; i++)
    {
        qatStats.numRec[i] = 0;
        qatStats.numSent[i] = 0;
    }

    status = QatCtrl_FWCountGet(qatInstance->debug->accel_dev,
                                       qatInstance, &qatStats);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to get counters from Firmware\n");
        LAC_LOG_ERROR("Device will be restarted\n");
        LAC_OS_CAFREE(qatStats.numSent);
        LAC_OS_CAFREE(qatStats.numRec);
#ifdef ICP_HEARTBEAT
        len += snprintf(data+len, size-len,
                      " ERROR: Qat is not responding and will be restarted\n");
        status = icp_adf_reset_dev(qatInstance->debug->accel_dev,
                                        ICP_ADF_DEV_RESET_ASYNC);
#else
        len += snprintf(data+len, size-len,
                 " ERROR: Qat is not responding. Please restart the device\n");
#endif
        return 0;
    }

    len = snprintf(data+len, size-len,
                   SEPARATOR
                   BORDER " Statistics for Qat Instance %u                    "
                   BORDER"\n"
                   SEPARATOR,
                   qatInstance->generic_service_info.instance );

    for(i=0; i< qatInstance->num_aes_per_qat; i++)
    {
        len += snprintf(data+len, size-len,
                       BORDER " Firmware Requests[AE %d]:        %16llu "
                       BORDER "\n"
                       BORDER " Firmware Responses[AE %d]:       %16llu "
                       BORDER "\n"
                       SEPARATOR,
                       i, (long long unsigned int)qatStats.numRec[i],
                       i, (long long unsigned int)qatStats.numSent[i]);
    }

    LAC_OS_CAFREE(qatStats.numSent);
    LAC_OS_CAFREE(qatStats.numRec);
    return 0;
}

/*
 * Function validates services enabled on different SKUs
 */
STATIC CpaStatus SalCtrl_ValidateServices(Cpa32U serviceMask,
                                          Cpa32U numberAEs)
{
    switch(numberAEs)
    {
        /* First SKU - only crypto A or B or both are valid */
        case NUM_AES_ONE:
            switch (serviceMask)
            {
                case SAL_SERVICE_TYPE_CRYPTO_A:
                case SAL_SERVICE_TYPE_CRYPTO_B:
                case SAL_SERVICE_TYPE_CRYPTO  :
                    return CPA_STATUS_SUCCESS;
                default:
                    return CPA_STATUS_FAIL;
            }
        /* Second SKU - only crypto A or B or compression or all are valid */
        case NUM_AES_TWO:
            switch (serviceMask)
            {
                case SAL_SERVICE_TYPE_CRYPTO_A:
                case SAL_SERVICE_TYPE_CRYPTO_B:
                case SAL_SERVICE_TYPE_CRYPTO:
                case SAL_SERVICE_TYPE_COMPRESSION:
                case SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_COMPRESSION:
                    return CPA_STATUS_SUCCESS;
                default:
                    return CPA_STATUS_FAIL;
            }
        /* Four AEs SKU - all services are valid */
        case NUM_AES_FOUR:
                    return CPA_STATUS_SUCCESS;
        default:
            return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_FAIL;
}

/*
 * Free all memory allocated for the QAT instance
 */
void SalCtrl_QatClean(sal_qat_service_t *qat_instance)
{
    LAC_OS_CAFREE(qat_instance->aeTargetIds);
    LAC_OS_CAFREE(qat_instance->qatAeMask);
    LAC_OS_CAFREE(qat_instance->serviceMask);
}

/*
 * The Init function takes care of initializing all
 * of the memory and arrays that will be needed to enable
 * the AEs on this QAT accelerator.
 */
CpaStatus
SalCtrl_QatInit(icp_accel_dev_t* device,
                sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0, j = 0;
    Cpa32U enabled_services=0;
    CpaBoolean cya_enabled = CPA_FALSE;
    CpaBoolean cyb_enabled = CPA_FALSE;
    CpaBoolean dc_enabled = CPA_FALSE;
    char temp_string[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char temp_value[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U accel_num = 0, bank_num = 0;
    Cpa32U num_qats=0;
    Cpa32U aeIndex=0;
    icp_qat_fw_init_ring_table_t *pRTtemp=NULL;
    sal_qat_service_t *qat_instance = (sal_qat_service_t*) service;
    Cpa32U ringNumRx=0, ringNumTx=0;
    sal_statistics_collection_t *pStatsCollection =
        (sal_statistics_collection_t*)device->pQatStats;

    if (SAL_SERVICE_STATE_UNINITIALIZED !=
          qat_instance->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call init\n");
        return CPA_STATUS_FAIL;
    }

    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_INITIALIZING;

    /*
     * In order to set up the Master Ring Table
     * we will need to know the number of AEs per
     * QAT instance.
     */
    status = SalCtrl_getNumberOfAE(device,
                                   &(qat_instance->num_aes));
    LAC_CHECK_STATUS_LOG(status, "Failed to get number of accel engines");
    status = SalCtrl_getNumberOfQat(device, &num_qats);
    LAC_CHECK_STATUS_LOG(status, "Failed to get number of accelerators");
    if(0 == num_qats)
    {
        LAC_LOG_ERROR("Number of accelerators out of range-num_qats is zero\n");
        return CPA_STATUS_FAIL;
    }

    qat_instance->num_aes_per_qat = (qat_instance->num_aes)/num_qats;

    /*
     * We will also need to know the Node Id so that we can
     * allocate memory from the corect node.
     */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_NODE_ID,
                                      temp_value);
    LAC_CHECK_STATUS(status);
    qat_instance->nodeId = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);

    /*
     * Within the possible SKU configurations we can only have either
     * 1 or 2 or 4 AEs per QAT.
     */
    if(!(NUM_AES_ONE == qat_instance->num_aes_per_qat
      || NUM_AES_TWO == qat_instance->num_aes_per_qat
      || NUM_AES_FOUR == qat_instance->num_aes_per_qat))
    {
         LAC_LOG_ERROR("Invalid param found - possible SKU \n"
                       "configuration error\n");
         return CPA_STATUS_FAIL;
    }

    /* Get max QAT */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_MAX_ACCEL,
                                      temp_value);
    LAC_CHECK_STATUS(status);
    qat_instance->max_qat = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);

    /* Get max AEs */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_MAX_AE,
                                      temp_value);
    LAC_CHECK_STATUS(status);
    qat_instance->max_aes = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);

    /* Get memory for the master ring table.
     * Note: size of the buffer pointed to by both pMasterRingTable
     * and pQatDmaMem is 4KB (one page) and the ring table for the device
     * can't be bigger that this. If in the future the size required
     * will be bigger then the allocation in adf_probe() function
     * in the driver should reflect this. */
    qat_instance->pMasterRingTable =
        device->pQatCpuMem[qat_instance->generic_service_info.instance];
    qat_instance->RingTableDmaAddr =
        device->pQatDmaMem[qat_instance->generic_service_info.instance];

    /* allocate memory for service mask */
    status = LAC_OS_CAMALLOC(&(qat_instance->serviceMask),
                            qat_instance->max_aes/qat_instance->max_qat,
                            LAC_64BYTE_ALIGNMENT, qat_instance->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate master serviceMask memory\n");
        return CPA_STATUS_RESOURCE;
    }

    /* Initialise the Master Ring Table */
    for (j = 0; j < qat_instance->num_aes_per_qat; j++)
    {
        /*
         * pRTtemp is a pointer to the start of the
         * table dedicated to a specific AE.
         */
        pRTtemp = &(qat_instance->pMasterRingTable[j]);
        for (i = 0; i < INIT_RING_TABLE_SZ; i++)
        {
            /*Set reserved field to zero*/
            pRTtemp->bulk_rings[i].resrvd    = 0;
            pRTtemp->bulk_rings[i].curr_weight = QATAL_DEFAULT_RING_WEIGHTING;
            pRTtemp->bulk_rings[i].init_weight = QATAL_DEFAULT_RING_WEIGHTING;
            pRTtemp->bulk_rings[i].ring_pvl    = QATAL_DEFAULT_PVL;
        }
        osalMemSet((void *)pRTtemp->ring_mask, 0,
                   INIT_RING_MASK_TABLE_BYTES_SZ);
        qat_instance->serviceMask[j] = 0;
    }
    pRTtemp = NULL;

    /* Get the skuing information */
    status = LAC_OS_CAMALLOC(&(qat_instance->qatAeMask),
                            qat_instance->max_qat * sizeof(Cpa32U),
                            LAC_64BYTE_ALIGNMENT, qat_instance->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate master qatAeMask memory\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_RESOURCE;
    }

    status = LAC_OS_CAMALLOC(&(qat_instance->aeTargetIds),
                            qat_instance->max_aes,
                            LAC_64BYTE_ALIGNMENT, qat_instance->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate master aeTargetIds memory\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_RESOURCE;
    }

    status = SalCtrl_getQatAeMask(device, qat_instance->qatAeMask);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to get skuing information\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }
    /* Using the qatAeMask get the aeTargetIds of the AEs in the system */
    for(i=0; i<qat_instance->max_qat; i++)
    {
        for(j=0; j<qat_instance->max_aes; j++)
        {

            if((qat_instance->qatAeMask[i] >> j) & 0x1)
            {
               qat_instance->aeTargetIds[aeIndex] = j;
               aeIndex++;
            }
        }
    }

    if(aeIndex != qat_instance->num_aes)
    {
        LAC_LOG_ERROR("qatAemask not as expected\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    /* What services are enabled */
    status = SalCtrl_GetEnabledServices(device, &enabled_services);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to get enabled services\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    /* Validate services enabled on different SKUs */
    status = SalCtrl_ValidateServices(enabled_services,
                                      qat_instance->num_aes_per_qat);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Invalid services configuration for current SKU\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }


    /* Check if the crypto A service is enabled */
    cya_enabled = SalCtrl_IsServiceEnabled(enabled_services,
                                           SAL_SERVICE_TYPE_CRYPTO_A);
    /* Check if the crypto B service is enabled */
    cyb_enabled = SalCtrl_IsServiceEnabled(enabled_services,
                                           SAL_SERVICE_TYPE_CRYPTO_B);
     /* Check if the compression service is enabled */
    dc_enabled = SalCtrl_IsServiceEnabled(enabled_services,
                                          SAL_SERVICE_TYPE_COMPRESSION);


    /* Create the Admin Rings. */
    /* Check that the accelerator number in the config table
     * matches what it should be. */
    status = Sal_StringParsing("Accel",
                      qat_instance->generic_service_info.instance,
                      "AcceleratorNumber",
                      temp_string);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to parse string\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      temp_string,
                                      temp_value);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    accel_num = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);
    if(accel_num != qat_instance->generic_service_info.instance)
    {
        LAC_LOG_ERROR("The accelerator number in the config table\n"
                      " does not match the instance number expected.");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    status = Sal_StringParsing("Accel",
                      qat_instance->generic_service_info.instance,
                      "AdminBankNumber", temp_string);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to parse string\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      temp_string,
                                      temp_value);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    bank_num = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);
    if(0 != bank_num)
    {
        LAC_LOG_ERROR("The bank number in the config file should be 0\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }
    status = Sal_StringParsing("Accel",
                               qat_instance->generic_service_info.instance,
                               "AdminTx", temp_string);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to parse string\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      temp_string,
                                      temp_value);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }
    ringNumTx = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);

    /* Param Check the Admin TX Rings found. */
    if(ADMIN_RING_TX != ringNumTx)
    {
        LAC_LOG_ERROR("Ring number Admin Tx incorrect\n"
                      " should be 0\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    status =
      icp_adf_transCreateHandle(device,
                                ICP_TRANS_TYPE_ETR,
                                LAC_CFG_SECTION_GENERAL,
                                accel_num,
                                bank_num,
                                temp_string,
                                lac_getRingType(SAL_RING_TYPE_ADMIN),
                                NULL,
                                ICP_RESP_TYPE_NONE, /* for the request the
                                                       response type doesn't
                                                       matter */
                                SAL_DEFAULT_RING_SIZE,
                                (icp_comms_trans_handle *)
                                &(qat_instance->trans_handle_qat_admin_tx));
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create handle for Admin Tx\n");
        SalCtrl_QatClean(qat_instance);
        return status;
    }

    /* The TX ring may need to be updated depending on the QAT instance. */
    status = icp_adf_transGetRingNum(qat_instance->trans_handle_qat_admin_tx,
                                     &ringNumTx);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create handle for Admin Tx\n");
        SalCtrl_QatClean(qat_instance);
        return status;
    }
    /* Update the transmit ring in the QAT structure. */
    qat_instance->ringNumTx = ringNumTx;

    status = Sal_StringParsing("Accel",
                               qat_instance->generic_service_info.instance,
                               "AdminRx", temp_string);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to parse string\n");
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }

    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      temp_string,
                                      temp_value);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_STRING_ERROR1("Failed to get %s from configuration file",
            temp_string);
        SalCtrl_QatClean(qat_instance);
        return CPA_STATUS_FAIL;
    }
    ringNumRx = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);
    /* Param Check the Admin Rx Rings found. */
    if(ADMIN_RING_RX != ringNumRx)
    {
        LAC_LOG_ERROR("Ring number Admin Rx incorrect\n"
                      " should be 1\n");
        SalCtrl_QatClean(qat_instance);
        icp_adf_transReleaseHandle(qat_instance->trans_handle_qat_admin_tx);
        return CPA_STATUS_FAIL;
    }

    status =
      icp_adf_transCreateHandle(device,
                                ICP_TRANS_TYPE_ETR,
                                LAC_CFG_SECTION_GENERAL,
                                accel_num,
                                bank_num,
                                temp_string,
                                lac_getRingType(SAL_RING_TYPE_NONE),
                                QatCtrl_ResponseMsgHandler,
                                ICP_RESP_TYPE_IRQ,/* needs to be triggered by
                                                     an interrupt for callback
                                                     hence IRQ */
                                SAL_DEFAULT_RING_SIZE,
                                (icp_comms_trans_handle *)
                                &(qat_instance->trans_handle_qat_admin_rx));
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create Admin Ring Handle.\n");
        SalCtrl_QatClean(qat_instance);
        icp_adf_transReleaseHandle(qat_instance->trans_handle_qat_admin_tx);
        return CPA_STATUS_FAIL;
    }

    status = icp_adf_transGetRingNum(qat_instance->trans_handle_qat_admin_rx,
                                     &ringNumRx);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create Admin Ring Handle.\n");
        SalCtrl_QatClean(qat_instance);
        icp_adf_transReleaseHandle(qat_instance->trans_handle_qat_admin_tx);
        icp_adf_transReleaseHandle(qat_instance->trans_handle_qat_admin_rx);
        return CPA_STATUS_FAIL;
    }

    /* Update the receive ring in the QAT structure. */
    qat_instance->ringNumRx = ringNumRx;

    /* set the QATAL init msg response callback handler */
    QatCtrl_ResponseCbSet(qat_instance);

    /*
     * Finally set up the service indices into the Master Ring Table
     * and convenient ring table pointers for each service enabled.
     * Only the Admin rings are initialized.
     */
    status = QatCtrl_InitializeRingTable(qat_instance, cya_enabled,
                               cyb_enabled, dc_enabled);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to initialize ring table\n");
        SalCtrl_QatClean(qat_instance);
        icp_adf_transReleaseHandle(qat_instance->trans_handle_qat_admin_tx);
        icp_adf_transReleaseHandle(qat_instance->trans_handle_qat_admin_rx);
        return status;
    }
    /* Calculate the number of active Aes per QAT
                               - needed for Shram partitioning. */
    for(i=0; i<qat_instance->num_aes_per_qat; i++)
    {
        if(qat_instance->serviceMask[i])
        {
            qat_instance->active_aes_per_qat++;
        }
    }

    if(CPA_TRUE == pStatsCollection->bStatsEnabled)
    {
        status = LAC_OS_MALLOC(&qat_instance->debug,
                              sizeof(sal_service_debug_t));
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to create service debug handler.\n");
            SalCtrl_QatClean(qat_instance);
            icp_adf_transReleaseHandle(
                     qat_instance->trans_handle_qat_admin_tx);
            icp_adf_transReleaseHandle(
                     qat_instance->trans_handle_qat_admin_rx);
            return status;
        }

        osalMemSet(qat_instance->debug, 0, sizeof(sal_service_debug_t));
        status = LAC_OS_MALLOC(&qat_instance->debug->debug_file.name,
                                                    DEBUG_QAT_NAME_LEN);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to create service debug handler.\n");
            SalCtrl_QatClean(qat_instance);
            icp_adf_transReleaseHandle(
                     qat_instance->trans_handle_qat_admin_tx);
            icp_adf_transReleaseHandle(
                     qat_instance->trans_handle_qat_admin_rx);
            LAC_OS_FREE(qat_instance->debug);
            return status;
        }
        snprintf(qat_instance->debug->debug_file.name, DEBUG_QAT_NAME_LEN,
             DEBUG_QAT_NAME, qat_instance->generic_service_info.instance);

        qat_instance->debug->debug_file.seq_read = QatCtrl_Debug;
        qat_instance->debug->debug_file.private_data = service;
        qat_instance->debug->accel_dev = device;

        status = icp_adf_debugAddFile(device,
                      &qat_instance->debug->debug_file);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to add service debug handler.\n");
            SalCtrl_QatClean(qat_instance);
            icp_adf_transReleaseHandle(
                     qat_instance->trans_handle_qat_admin_tx);
            icp_adf_transReleaseHandle(
                     qat_instance->trans_handle_qat_admin_rx);
            LAC_OS_FREE(qat_instance->debug->debug_file.name);
            LAC_OS_FREE(qat_instance->debug);
            return status;
        }
    }
    qat_instance->generic_service_info.stats = pStatsCollection;
    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_INITIALIZED;
    /* Initialize the admin callback counter to zero. */
    osalAtomicSet(0,
                  &(qat_instance->adminCallbackPending));
    return status;
}

/*
 * The start function sends the init messages to the AEs.
 */
CpaStatus SalCtrl_QatStart(icp_accel_dev_t* device,
                        sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_qat_service_t* qat_instance = (sal_qat_service_t*) service;
#ifndef TRNG_DISABLED
    Cpa32U capabilitiesMask = 0;
    Cpa32U no_wireless_procs = 0;
    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
#endif
    if (SAL_SERVICE_STATE_INITIALIZED !=
         qat_instance->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call start\n");
        return CPA_STATUS_FAIL;
    }

    /******************************************************/
    /* Send init msgs to AEs */
    /******************************************************/

    /*
     * SET_AE_INFO Msgs are only sent using instance 0.
     * Each AE polls the admin ring looking for its own
     * unique SET_AE_INFO msg.
     */
    if(0 == qat_instance->generic_service_info.instance)
    {
        /* The number of messages sent is equal to the number of
           active AEs for the device */
        status = QatCtrl_SendInitMsg(device,
                                     qat_instance,
                                     ICP_QAT_FW_INIT_CMD_SET_AE_INFO);
    }

    /* Checking AE_Info Msg */
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Sending SET_AE_INFO msg Failed\n");
        return CPA_STATUS_FAIL;
    }

    /*
     * Send the RING_INFO msg next.
     * A SET_RING_INFO msg will be sent to each active AE
     * per QAT instance.
     */
    status = QatCtrl_SendInitMsg(device,
                                 qat_instance,
                                 ICP_QAT_FW_INIT_CMD_SET_RING_INFO);

     /* Checking RING_INFO Msg */
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Sending SET_RING_INFO msg Failed\n");
        return status;
    }

#ifdef TRNG_DISABLED
    LAC_LOG("TRNG is disabled\n");
#else
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
        /* Not running wireless firmware, so we send TRNG_ENABLE message */
        status = icp_amgr_getAccelDevCapabilities(device,
                &capabilitiesMask);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to get device capabilities\n");
            return status;
        }

        if (capabilitiesMask & ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER)
        {
            /*
             * Send the TRNG_ENABLE message
             * A TRNG_ENABLE msg will be sent to each AE
             * per QAT instance that has crypto enabled.
             */
            status = QatCtrl_SendInitMsg(device,
                                         qat_instance,
                                         ICP_QAT_FW_INIT_CMD_TRNG_ENABLE);
            if (CPA_STATUS_SUCCESS != status)
            {
                LAC_LOG_ERROR("Sending TRNG_ENABLE msg Failed\n");
                return status;
            }
        }
    }
#endif
    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_RUNNING;
    return status;
}

/*
 * The Stop function sends the TRNG disable msg to each AE
 * in this QAT instance.
 */
CpaStatus
SalCtrl_QatStop(icp_accel_dev_t* device, sal_service_t* service)
{
#ifndef TRNG_DISABLED
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U no_wireless_procs = 0;
    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U capabilitiesMask  = 0;
#endif
    sal_qat_service_t* qat_instance = (sal_qat_service_t*) service;

    if (SAL_SERVICE_STATE_RUNNING !=
            qat_instance->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call stop");
    }

#ifdef TRNG_DISABLED
    LAC_LOG("TRNG is disabled\n");
#else
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

    if ((no_wireless_procs == 0) && (!icp_adf_is_dev_in_reset(device)))
    {
        /* Not running wireless firmware, so we send TRNG_DISABLE message */
        status = icp_amgr_getAccelDevCapabilities(device,
                &capabilitiesMask);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to get device capabilities\n");
        }

        if (capabilitiesMask & ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER)
        {
            /*
             * Send the TRNG_DISABLE message to each AE
             * per QAT instance that has crypto enabled
             */
            status = QatCtrl_SendInitMsg(device,
                                         qat_instance,
                                         ICP_QAT_FW_INIT_CMD_TRNG_DISABLE);
            if (CPA_STATUS_SUCCESS != status)
            {
                LAC_LOG_ERROR("Sending TRNG_DISABLE msg Failed\n");
            }
        }
    }
#endif

    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_SHUTTING_DOWN;
    /* If there are admin messages in-flight return RETRY */
    if(osalAtomicGet(&(qat_instance->adminCallbackPending)) &&
                                            (!icp_adf_is_dev_in_reset(device)))
    {
        return CPA_STATUS_RETRY;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * The shutdown releases the rings and frees all the memory allocated
 * during the init.
 */
CpaStatus
SalCtrl_QatShutdown(icp_accel_dev_t* device, sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus ret_status = CPA_STATUS_SUCCESS;
    sal_qat_service_t* qat_instance = (sal_qat_service_t*) service;
    sal_statistics_collection_t *pStatsCollection =
    (sal_statistics_collection_t*)device->pQatStats;


    if ((SAL_SERVICE_STATE_INITIALIZED !=
          qat_instance->generic_service_info.state) &&
        (SAL_SERVICE_STATE_SHUTTING_DOWN !=
         qat_instance->generic_service_info.state))
    {
        LAC_LOG_ERROR("Not in the correct state to call shutdown \n");
        return CPA_STATUS_FAIL;
    }

    status
        = icp_adf_transReleaseHandle(qat_instance->trans_handle_qat_admin_tx);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to release handle for Admin Tx\n");
        ret_status = status;
    }

    status
        = icp_adf_transReleaseHandle(qat_instance->trans_handle_qat_admin_rx);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to release handle for Admin Rx\n");
        ret_status = status;
    }
    /* free allocated memory */
    if (NULL != qat_instance->aeTargetIds)
    {
        LAC_OS_CAFREE(qat_instance->aeTargetIds);
    }
    if (NULL != qat_instance->qatAeMask)
    {
        LAC_OS_CAFREE(qat_instance->qatAeMask);
    }
    if (NULL != qat_instance->serviceMask)
    {
        LAC_OS_CAFREE(qat_instance->serviceMask);
    }

    if(CPA_TRUE == pStatsCollection->bStatsEnabled &&
       qat_instance->debug &&
       qat_instance->debug->debug_file.name)
    {
        icp_adf_debugRemoveFile(&qat_instance->debug->debug_file);
        LAC_OS_FREE(qat_instance->debug->debug_file.name);
        LAC_OS_FREE(qat_instance->debug);
    }

    qat_instance->generic_service_info.stats = NULL;
    /* change state to be shutdown */
    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_SHUTDOWN;

    return ret_status;
}
