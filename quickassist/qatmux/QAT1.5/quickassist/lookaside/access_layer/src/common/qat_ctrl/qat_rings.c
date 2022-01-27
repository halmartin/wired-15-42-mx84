/*
 ***************************************************************************
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
 * @file  qat_rings.c
 *
 * @ingroup qat_ctrl
 *
 * @description
 *        This file contains the functions to be used to set and update
 *        ring polling masks for qat.
 *
 *****************************************************************************/

/* include cpa APIs. */
#include "cpa.h"

/* include header files for osal and memory macros. */
#include "Osal.h"
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_list.h"

/* include necessary ADF headers. */
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/* include FW headers. */
#include "icp_qat_fw_init.h"
#include "icp_qat_fw_admin.h"

/* include SAL headers. */
#include "lac_sal_types.h"
#include "lac_sal_types_qat_ctrl.h"
#include "lac_sal.h"
#include "sal_string_parse.h"

/* QAT headers. */
#include "qat_priority.h"
#include "qat_init_defs.h"

#include "qat_rings.h"

/*
 *  Build the kernel ring table. There is one master table which
 *  contains all of the sub tables. The sub tables are the
 *  crypto table and the compression table.
 *  The service mask is used to check if a service is
 *  enabled for the given AE before the table is built for that
 *  AE.
 */

CpaStatus QatCtrl_buildKernelRingTable(icp_accel_dev_t* device,
                                       sal_qat_service_t* qatInstance,
                                       Cpa32U ae_index_per_qat)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U ring_num = 0, execution_engine = 0;
    Cpa32U no_cy_inst = 0, no_dc_inst = 0;
    Cpa32U no_wireless_procs = 0;
    Cpa32U i = 0, j = 0;
    Cpa32U accel_num = 0, bank_num = 0, instance_num = 0;
    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char temp_string[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char wireless_section[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    instance_num = qatInstance->generic_service_info.instance;
    /*
     * Check that the crypto service is actually enabled on
     * this AE.
     */
    no_cy_inst = 0;
    if(SAL_SERVICE_TYPE_CRYPTO & qatInstance->serviceMask[ae_index_per_qat])
    {
        /* Get the number of Crypto Instances */
        status = icp_adf_cfgGetParamValue(device,
                                          icpGetProcessName(),
                                          "NumberCyInstances",
                                          adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
            "Can't find NumberCyInstances in config file in %s section",
             icpGetProcessName());
        no_cy_inst = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    }
    /*
     * Loop over the Crypto instance getting the accel number,
     * bank number and appropriate ring numbers. Set the
     * ring weights accordingly.
     */
    for(i=0; i<no_cy_inst; i++)
    {
        /*
         * Sal_StringParsing concatenates the first three arguments
         * and places the result into temp_string.
         */
        status = Sal_StringParsing("Cy",i,
                "AcceleratorNumber", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>AcceleratorNumber");

        status = icp_adf_cfgGetParamValue(device,
                    icpGetProcessName(),
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get %s from config file in %s section",
                temp_string, icpGetProcessName());

        accel_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
        if(accel_num != instance_num)
        {
            /* Crypto instance is not on this accelerator, continue */
            continue;
        }

        status = Sal_StringParsing("Cy", i,
                "BankNumber", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>BankNumber");
        status = icp_adf_cfgGetParamValue(device,
                    icpGetProcessName(),
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get %s from config file in %s section",
                temp_string, icpGetProcessName());

        bank_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

        status = Sal_StringParsing("Cy", i, "RingSymTxHi", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingSymTxHi");
        status = icp_adf_cfgGetRingNumber(device,
                    icpGetProcessName(),
                    accel_num,
                    bank_num,
                    temp_string,
                    &ring_num);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get ring number for %s", temp_string);

        /* ring_num needs to be in range [0,127] for ring polling */
        ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

        status = Sal_StringParsing("Cy", i,
                "ExecutionEngine", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>ExecutionEngine");
        status = icp_adf_cfgGetParamValue(device,
                    icpGetProcessName(),
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get %s from config file in %s section",
                temp_string, icpGetProcessName());

        execution_engine = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

        /*
         * Initialize the weights for the bulk rings according to
         * the usage model. The Ring mask is also set for this ring number.
         */
        if(0 == execution_engine)
        {
            if(!(SAL_SERVICE_TYPE_CRYPTO_A &
                            qatInstance->serviceMask[ae_index_per_qat]))
            {
                continue;
            }
            qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

            qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                                  ring_num);
        }
        else if(1 == execution_engine)
        {
            if(!(SAL_SERVICE_TYPE_CRYPTO_B &
                            qatInstance->serviceMask[ae_index_per_qat]))
            {
                continue;
            }
            qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

            qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                                  ring_num);
        }
        else
        {
            LAC_LOG_ERROR_PARAMS("Invalid execution engine %d for inst Cy%d",
                        execution_engine, i);
            return CPA_STATUS_FAIL;
        }

        status = Sal_StringParsing("Cy", i, "RingSymTxLo", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingSymTxLo");

        status = icp_adf_cfgGetRingNumber(device,
                    icpGetProcessName(),
                    accel_num,
                    bank_num,
                    temp_string,
                    &ring_num);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get ring number for %s", temp_string);

        ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

        if(0 == execution_engine)
        {
            qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

            qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                                 ring_num);
        }
        else
        {
            qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

            qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                                 ring_num);
        }

        status = Sal_StringParsing("Cy", i, "RingAsymTx", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingAsymTx");

        status = icp_adf_cfgGetRingNumber(device,
                    icpGetProcessName(),
                    accel_num,
                    bank_num,
                    temp_string,
                    &ring_num);

        LAC_CHECK_STATUS_LOG(status,
                "Failed to get ring number for %s", temp_string);

        ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

        if(0 == execution_engine)
        {
            qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                = QATAL_PKE_RING_WEIGHTING;

            qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                = QATAL_PKE_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                               ring_num);
        }
        else
        {
            qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                = QATAL_PKE_RING_WEIGHTING;

            qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                = QATAL_PKE_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                               ring_num);
        }
    }

    /*
     * Get the number of Wireless sections in the config file. We will set the bits
     * for the AE now at init time rather than later when the processes start.
     */
    if(SAL_SERVICE_TYPE_CRYPTO & qatInstance->serviceMask[ae_index_per_qat])
    {
        /* Get the number of Wireless Procs */
        status = icp_adf_cfgGetParamValue(device,
                                          "GENERAL",
                                          "NumberOfWirelessProcs",
                                          adfGetParam);
        if (CPA_STATUS_SUCCESS == status)
        {
            no_wireless_procs = Sal_Strtoul(adfGetParam,
                                            NULL,
                                            SAL_CFG_BASE_DEC);
        }
        else
        {
            no_wireless_procs = 0;
        }
    }

    for (j=0; j<no_wireless_procs; j++)
    {
        status = Sal_StringParsing("WIRELESS_INT_", j, "", wireless_section);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse WIRELESS_<x>");

        /*
         * Check that the crypto service is actually enabled on
         * this AE.
         */
        no_cy_inst = 0;
        if(SAL_SERVICE_TYPE_CRYPTO & qatInstance->serviceMask[ae_index_per_qat])
        {
            /* Get the number of Crypto Instances */
            status = icp_adf_cfgGetParamValue(device,
                                              wireless_section,
                                              "NumberCyInstances",
                                              adfGetParam);
            LAC_CHECK_STATUS_LOG(status,
                "Can't find NumberCyInstances in config file in %s section",
                 wireless_section);
            no_cy_inst = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
        }
        /*
         * Loop over the Crypto instance getting the accel number,
         * bank number and appropriate ring numbers. Set the
         * ring weights accordingly.
         */
        for(i=0; i<no_cy_inst; i++)
        {
            /*
             * Sal_StringParsing concatenates the first three arguments
             * and places the result into temp_string.
             */
            status = Sal_StringParsing("Cy",i,
                    "AcceleratorNumber", temp_string);
            LAC_CHECK_STATUS_LOG(status,
                                 "Failed to parse Cy<x>AcceleratorNumber");

            status = icp_adf_cfgGetParamValue(device,
                        wireless_section,
                        temp_string,
                        adfGetParam);
            LAC_CHECK_STATUS_LOG(status,
                    "Failed to get %s from config file in %s section",
                    temp_string, wireless_section);

            accel_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
            if(accel_num != instance_num)
            {
                /* Crypto instance is not on this accelerator, continue */
                continue;
            }

            status = Sal_StringParsing("Cy", i,
                    "BankNumber", temp_string);
            LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>BankNumber");
            status = icp_adf_cfgGetParamValue(device,
                        wireless_section,
                        temp_string,
                        adfGetParam);
            LAC_CHECK_STATUS_LOG(status,
                    "Failed to get %s from config file in %s section",
                    temp_string, wireless_section);

            bank_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

            status = Sal_StringParsing("Cy", i, "RingSymTxHi", temp_string);
            LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingSymTxHi");
            status = icp_adf_cfgGetRingNumber(device,
                        wireless_section,
                        accel_num,
                        bank_num,
                        temp_string,
                        &ring_num);
            LAC_CHECK_STATUS_LOG(status,
                    "Failed to get ring number for %s", temp_string);

            /* ring_num needs to be in range [0,127] for ring polling */
            ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

            status = Sal_StringParsing("Cy", i,
                    "ExecutionEngine", temp_string);
            LAC_CHECK_STATUS_LOG(status,
                                 "Failed to parse Cy<x>ExecutionEngine");
            status = icp_adf_cfgGetParamValue(device,
                        wireless_section,
                        temp_string,
                        adfGetParam);
            LAC_CHECK_STATUS_LOG(status,
                    "Failed to get %s from config file in %s section",
                    temp_string, wireless_section);

            execution_engine = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

            /*
             * Initialize the weights for the bulk rings according to
             * the usage model. The Ring mask is also set for this ring number.
             */
            if(0 == execution_engine)
            {
                if(!(SAL_SERVICE_TYPE_CRYPTO_A &
                            qatInstance->serviceMask[ae_index_per_qat]))
                {
                    continue;
                }
                qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                    = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

                qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                    = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

                ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                                      ring_num);
            }
            else if(1 == execution_engine)
            {
                if(!(SAL_SERVICE_TYPE_CRYPTO_B &
                                qatInstance->serviceMask[ae_index_per_qat]))
                {
                    continue;
                }
                qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                    = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

                qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                    = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

                ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                                      ring_num);
            }
            else
            {
                LAC_LOG_ERROR_PARAMS(
                            "Invalid execution engine %d for inst Cy%d",
                            execution_engine, i);
                return CPA_STATUS_FAIL;
            }

            status = Sal_StringParsing("Cy", i, "RingSymTxLo", temp_string);
            LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingSymTxLo");

            status = icp_adf_cfgGetRingNumber(device,
                        wireless_section,
                        accel_num,
                        bank_num,
                        temp_string,
                        &ring_num);
            LAC_CHECK_STATUS_LOG(status,
                    "Failed to get ring number for %s", temp_string);

            ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

            if(0 == execution_engine)
            {
                qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                    = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

                qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                    = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

                ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                                     ring_num);
            }
            else
            {
                qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                    = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

                qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                    = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

                ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                                     ring_num);
            }

            status = Sal_StringParsing("Cy", i, "RingAsymTx", temp_string);
            LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingAsymTx");

            status = icp_adf_cfgGetRingNumber(device,
                        wireless_section,
                        accel_num,
                        bank_num,
                        temp_string,
                        &ring_num);

            LAC_CHECK_STATUS_LOG(status,
                    "Failed to get ring number for %s", temp_string);

            ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

            if(0 == execution_engine)
            {
                qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                    = QATAL_PKE_RING_WEIGHTING;

                qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                    = QATAL_PKE_RING_WEIGHTING;

                ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                                   ring_num);
            }
            else
            {
                qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                    = QATAL_PKE_RING_WEIGHTING;

                qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                    = QATAL_PKE_RING_WEIGHTING;

                ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                                   ring_num);
            }
        }
    }

    /*
     * For Dyn section, We will set the bits
     * for the AE now at init time rather than later when the processes start.
     */
    no_cy_inst = 0;
    if(SAL_SERVICE_TYPE_CRYPTO & qatInstance->serviceMask[ae_index_per_qat])
    {
        /* Get the number of Crypto Instances */
        status = icp_adf_cfgGetParamValue(device,
                                          DYN_SEC,
                                          "NumberCyInstances",
                                          adfGetParam);
        if(CPA_STATUS_SUCCESS != status)
        {
            no_cy_inst = 0;
            status = CPA_STATUS_SUCCESS;
        }
        else
        {
            no_cy_inst = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
        }
    }
    /*
     * Loop over the Crypto instance getting the accel number,
     * bank number and appropriate ring numbers. Set the
     * ring weights accordingly.
     */
    for(i=0; i<no_cy_inst; i++)
    {
        /*
         * Sal_StringParsing concatenates the first three arguments
         * and places the result into temp_string.
         */
        status = Sal_StringParsing("Cy",i,
                "AcceleratorNumber", temp_string);
        LAC_CHECK_STATUS_LOG(status,
                             "Failed to parse Cy<x>AcceleratorNumber");

        status = icp_adf_cfgGetParamValue(device,
                    DYN_SEC,
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get %s from config file in %s section",
                temp_string, DYN_SEC);

        accel_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
        if(accel_num != instance_num)
        {
            /* Crypto instance is not on this accelerator, continue */
            continue;
        }

        status = Sal_StringParsing("Cy", i,
               "BankNumber", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>BankNumber");
        status = icp_adf_cfgGetParamValue(device,
                    DYN_SEC,
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get %s from config file in %s section",
                temp_string, DYN_SEC);

        bank_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

        status = Sal_StringParsing("Cy", i, "RingSymTxHi", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingSymTxHi");
        status = icp_adf_cfgGetRingNumber(device,
                    DYN_SEC,
                    accel_num,
                    bank_num,
                    temp_string,
                    &ring_num);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get ring number for %s", temp_string);

        /* ring_num needs to be in range [0,127] for ring polling */
        ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

        status = Sal_StringParsing("Cy", i,
                "ExecutionEngine", temp_string);
        LAC_CHECK_STATUS_LOG(status,
                            "Failed to parse Cy<x>ExecutionEngine");
        status = icp_adf_cfgGetParamValue(device,
                    DYN_SEC,
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get %s from config file in %s section",
                temp_string, DYN_SEC);

        execution_engine = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

        /*
         * Initialize the weights for the bulk rings according to
         * the usage model. The Ring mask is also set for this ring number.
         */
        if(0 == execution_engine)
        {
            if(!(SAL_SERVICE_TYPE_CRYPTO_A &
                        qatInstance->serviceMask[ae_index_per_qat]))
            {
                continue;
            }
            qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

            qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                                  ring_num);
        }
        else if(1 == execution_engine)
        {
            if(!(SAL_SERVICE_TYPE_CRYPTO_B &
                            qatInstance->serviceMask[ae_index_per_qat]))
            {
                continue;
            }
            qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

            qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                = QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                                  ring_num);
        }
        else
        {
            LAC_LOG_ERROR_PARAMS(
                        "Invalid execution engine %d for inst Cy%d",
                        execution_engine, i);
            return CPA_STATUS_FAIL;
        }

        status = Sal_StringParsing("Cy", i, "RingSymTxLo", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingSymTxLo");

        status = icp_adf_cfgGetRingNumber(device,
                    DYN_SEC,
                    accel_num,
                    bank_num,
                    temp_string,
                    &ring_num);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get ring number for %s", temp_string);

        ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

        if(0 == execution_engine)
        {
            qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

            qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                                 ring_num);
        }
        else
        {
            qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

            qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                                     ring_num);
        }

        status = Sal_StringParsing("Cy", i, "RingAsymTx", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Cy<x>RingAsymTx");

        status = icp_adf_cfgGetRingNumber(device,
                    DYN_SEC,
                    accel_num,
                    bank_num,
                    temp_string,
                    &ring_num);

        LAC_CHECK_STATUS_LOG(status,
                "Failed to get ring number for %s", temp_string);

        ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

        if(0 == execution_engine)
        {
            qatInstance->pCyARingTable->bulk_rings[ring_num].curr_weight
                = QATAL_PKE_RING_WEIGHTING;

            qatInstance->pCyARingTable->bulk_rings[ring_num].init_weight
                = QATAL_PKE_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyARingTable)),
                                                               ring_num);
        }
        else
        {
            qatInstance->pCyBRingTable->bulk_rings[ring_num].curr_weight
                = QATAL_PKE_RING_WEIGHTING;

            qatInstance->pCyBRingTable->bulk_rings[ring_num].init_weight
                = QATAL_PKE_RING_WEIGHTING;

            ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pCyBRingTable)),
                                                               ring_num);
        }
    }

    /*
     * Check that the compression service is actually enabled on
     * this AE.
     */
    no_dc_inst = 0;
    if(SAL_SERVICE_TYPE_COMPRESSION & qatInstance->serviceMask[ae_index_per_qat])
    {
        /* Get the number of Compression Instances */
        status = icp_adf_cfgGetParamValue(device,
                                          DYN_SEC,
                                          "NumberDcInstances",
                                          adfGetParam);
        if(CPA_STATUS_SUCCESS != status)
        {
            no_dc_inst = 0;
            status = CPA_STATUS_SUCCESS;
        }
        else
        {
            no_dc_inst = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
        }
    }

    /*
     * Loop over the Compression instance getting the accel number,
     * bank number and appropriate ring numbers. Set the
     * ring weights accordingly.
     */
    for(i=0; i<no_dc_inst; i++)
    {
        status = Sal_StringParsing("Dc",i,
                "AcceleratorNumber", temp_string);

        LAC_CHECK_STATUS_LOG(status, "Failed to parse Dc<x>AcceleratorNumber");
        status = icp_adf_cfgGetParamValue(device,
                    DYN_SEC,
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
              "Failed to get %s from config file in %s section",
               temp_string, DYN_SEC);

        accel_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
        if(accel_num != instance_num)
        {
            /* Compression instance is not on this accelerator, continue */
            continue;
        }

        status = Sal_StringParsing("Dc", i,
                "BankNumber", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Dc<x>BankNumber");
        status = icp_adf_cfgGetParamValue(device,
                    DYN_SEC,
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
              "Failed to get %s from config file in %s section",
               temp_string, DYN_SEC);

        bank_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

        status = Sal_StringParsing("Dc", i, "RingTx", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Dc<x>RingTx");
        status = icp_adf_cfgGetRingNumber(device,
                    DYN_SEC,
                    accel_num,
                    bank_num,
                    temp_string,
                    &ring_num);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get ring number for %s", temp_string);

        ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

        qatInstance->pDcRingTable->bulk_rings[ring_num].curr_weight
            = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;
        qatInstance->pDcRingTable->bulk_rings[ring_num].init_weight
            = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

        ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pDcRingTable)), ring_num);
    }

    /*
     * Check that the compression service is actually enabled on
     * this AE.
     */
    no_dc_inst = 0;
    if(SAL_SERVICE_TYPE_COMPRESSION &
       qatInstance->serviceMask[ae_index_per_qat])
    {
        /* Get the number of Compression Instances */
        status = icp_adf_cfgGetParamValue(device,
                                          icpGetProcessName(),
                                          "NumberDcInstances",
                                          adfGetParam);

        LAC_CHECK_STATUS_LOG(status,
              "Failed to get NumberDcInstances from config file in %s section",
               icpGetProcessName());

        no_dc_inst = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    }
    /*
     * Loop over the Compression instance getting the accel number,
     * bank number and appropriate ring numbers. Set the
     * ring weights accordingly.
     */
    for(i=0; i<no_dc_inst; i++)
    {
        status = Sal_StringParsing("Dc",i,
                "AcceleratorNumber", temp_string);

        LAC_CHECK_STATUS_LOG(status, "Failed to parse Dc<x>AcceleratorNumber");
        status = icp_adf_cfgGetParamValue(device,
                    icpGetProcessName(),
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
              "Failed to get %s from config file in %s section",
               temp_string, icpGetProcessName());

        accel_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
        if(accel_num != instance_num)
        {
            /* Compression instance is not on this accelerator, continue */
            continue;
        }

        status = Sal_StringParsing("Dc", i,
                "BankNumber", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Dc<x>BankNumber");
        status = icp_adf_cfgGetParamValue(device,
                    icpGetProcessName(),
                    temp_string,
                    adfGetParam);
        LAC_CHECK_STATUS_LOG(status,
              "Failed to get %s from config file in %s section",
               temp_string, icpGetProcessName());

        bank_num = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);

        status = Sal_StringParsing("Dc", i, "RingTx", temp_string);
        LAC_CHECK_STATUS_LOG(status, "Failed to parse Dc<x>RingTx");
        status = icp_adf_cfgGetRingNumber(device,
                    icpGetProcessName(),
                    accel_num,
                    bank_num,
                    temp_string,
                    &ring_num);
        LAC_CHECK_STATUS_LOG(status,
                "Failed to get ring number for %s", temp_string);

        ring_num = (ring_num % SAL_RINGS_NUM_PER_QAT);

        qatInstance->pDcRingTable->bulk_rings[ring_num].curr_weight
            = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;
        qatInstance->pDcRingTable->bulk_rings[ring_num].init_weight
            = QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING;

        ICP_QAT_FW_INIT_RING_MASK_SET((*(qatInstance->pDcRingTable)), ring_num);
    }
    /* Ring table (or tables) and masks now updated for all kernel instances
       associated with this physical qat */

    return status;
}

/*
 * Update the ring table as Rings are enabled or disabled.
 * This function called when user space processes create
 * and release ring handles.
 */

CpaStatus QatCtrl_updateRingTable(icp_accel_dev_t* device,
                                sal_qat_service_t* qatInstance,
                                Cpa32U ringNumber,
                                icp_adf_ringInfoOperation_t operation,
                                Cpa32U ring_service_type)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    switch(ring_service_type)
    {
    case SAL_RING_TYPE_A_SYM_HI:
        /* update ring mask and crypto ring table */
        if(ICP_ADF_RING_ENABLE == operation)
        {
            ENABLE_CY_A_RING_TABLE(ringNumber,
                     QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING);
        }
        else
        {
            DISABLE_CY_A_RING_TABLE(ringNumber,
                      QATAL_DEFAULT_RING_WEIGHTING);

        }
        break;
    case SAL_RING_TYPE_A_SYM_LO:
        /* update ring mask and crypto ring table */
        if(ICP_ADF_RING_ENABLE == operation)
        {
            ENABLE_CY_A_RING_TABLE(ringNumber,
                     QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING);
        }
        else
        {
            DISABLE_CY_A_RING_TABLE(ringNumber,
                      QATAL_DEFAULT_RING_WEIGHTING);
        }
        break;
    case SAL_RING_TYPE_A_ASYM:
        /* update ring mask and crypto ring table */
        if(ICP_ADF_RING_ENABLE == operation)
        {
            ENABLE_CY_A_RING_TABLE(ringNumber,
                     QATAL_PKE_RING_WEIGHTING);
        }
        else
        {
            DISABLE_CY_A_RING_TABLE(ringNumber,
                      QATAL_DEFAULT_RING_WEIGHTING);
        }
        break;
    case SAL_RING_TYPE_B_SYM_HI:
        /* update ring mask and crypto ring table */
        if(ICP_ADF_RING_ENABLE == operation)
        {
            ENABLE_CY_B_RING_TABLE(ringNumber,
                     QATAL_LAC_HIGH_PRIORITY_RING_WEIGHTING);
        }
        else
        {
            DISABLE_CY_B_RING_TABLE(ringNumber,
                      QATAL_DEFAULT_RING_WEIGHTING);

        }
        break;
    case SAL_RING_TYPE_B_SYM_LO:
        /* update ring mask and crypto ring table */
        if(ICP_ADF_RING_ENABLE == operation)
        {
            ENABLE_CY_B_RING_TABLE(ringNumber,
                     QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING);
        }
        else
        {
            DISABLE_CY_B_RING_TABLE(ringNumber,
                      QATAL_DEFAULT_RING_WEIGHTING);
        }
        break;
    case SAL_RING_TYPE_B_ASYM:
        /* update ring mask and crypto ring table */
        if(ICP_ADF_RING_ENABLE == operation)
        {
            ENABLE_CY_B_RING_TABLE(ringNumber,
                     QATAL_PKE_RING_WEIGHTING);
        }
        else
        {
            DISABLE_CY_B_RING_TABLE(ringNumber,
                      QATAL_DEFAULT_RING_WEIGHTING);
        }
        break;

    case SAL_RING_TYPE_DC:
        /* update ring mask and compression ring table */
        if(ICP_ADF_RING_ENABLE == operation)
        {
            ENABLE_DC_RING_TABLE(ringNumber,
                     QATAL_LAC_LOW_PRIORITY_RING_WEIGHTING);
        }
        else
        {
            DISABLE_DC_RING_TABLE(ringNumber,
                      QATAL_DEFAULT_RING_WEIGHTING);
        }
        break;
    default:
        {
            LAC_LOG_ERROR("Invalid ring type detected.\n");
            status = CPA_STATUS_FAIL;
        }
        break;
    }

    return status;
}
