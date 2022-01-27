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
 *  version: QAT1.6.L.2.6.0-65
 *
******************************************************************************/

/******************************************************************************
 * @file adf_dh895x_config.c
 *
 * @ingroup adf_cfg
 *
 * @description
 *      Functions to generate dh895x parameters that are not in the config file
 *
******************************************************************************/

#include "icp_accel_devices.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "adf_config.h"

#define ADF_CHECK_STATUS_RETURN_ON_FAIL(status, args...) \
do { \
    if(CPA_STATUS_SUCCESS != status) \
    { \
        ADF_ERROR(args); \
        return status; \
    } \
} while(0)

/* 
 * Masks to track DH895xCC ring allocations
 *
 * Re-using the 'firstAvailableRing' value of the adf_bank_info_t structure
 * from DH89xxCC setup to track DH895xCC allocated rings. This means using a
 * mask since the ring locations are static.
 */
#define ALLOCATED_CY_INSTANCE_0_BIT (1)
#define ALLOCATED_CY_INSTANCE_1_BIT (2)
#define ALLOCATED_DC_INSTANCE_0_BIT (4)
#define ALLOCATED_DC_INSTANCE_1_BIT (8)

STATIC adf_bank_info_t *dh895_banks = NULL;

STATIC int dh895_num_banks_per_accel = 0;
STATIC int dh895_num_rings_per_bank = 0;

/*
 * adf_set_cy_rings_dh895x
 *
 * This function sets the crypto rings in the given bank
 */
STATIC CpaStatus adf_set_cy_rings_dh895x (Cpa32U bank_idx,
                                          char * name,
                                          adf_cfg_section_t *pSection,
                                          Cpa8U processNum,
                                          Cpa32U instanceOffset)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_bank_info_t *bank = NULL;

    ICP_CHECK_PARAM_RANGE(bank_idx, 0, dh895_num_banks_per_accel);
    bank = &dh895_banks[bank_idx];

    status = adf_add_param_from_string(bank_idx,
                                       BANK_NUM_STR,
                                       name,
                                       pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to add bank parameter\n");

    status = adf_add_param_from_string(
                                ICP_DH895xCC_ASYM_TX_RING_0 + instanceOffset,
                                RING_ASYM_TX_STR,
                                name,
                                pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to create crypto rings\n");

    status = adf_add_param_from_string(
                                /*response ring is #req + 8*/
                                ICP_DH895xCC_ASYM_TX_RING_0 +
                                  instanceOffset + ICP_DH895xCC_RX_RINGS_OFFSET,
                                RING_ASYM_RX_STR,
                                name,
                                pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to create crypto rings\n");

    status = adf_add_param_from_string(
                                ICP_DH895xCC_SYM_TX_RING_0 + instanceOffset,
                                RING_SYM_TX_STR,
                                name,
                                pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to create crypto rings\n");

    status = adf_add_param_from_string(
                                ICP_DH895xCC_SYM_TX_RING_0 +
                                  instanceOffset + ICP_DH895xCC_RX_RINGS_OFFSET,
                                RING_SYM_RX_STR,
                                name,
                                pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to create crypto rings\n");

    status = adf_add_param_from_string(
                                ICP_DH895xCC_NRBG_TX_RING_0 + instanceOffset,
                                RING_NRBG_TX_STR,
                                name,
                                pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to create crypto rings\n");

    status = adf_add_param_from_string(
                                ICP_DH895xCC_NRBG_TX_RING_0 +
                                  instanceOffset + ICP_DH895xCC_RX_RINGS_OFFSET,
                                RING_NRBG_RX_STR,
                                name,
                                pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to create crypto rings\n");

    bank->processNum[instanceOffset + ADF_CY_PROCESS_OFFSET] = processNum;
    bank->firstAvailableRing |= ((instanceOffset == 0) ?
                ALLOCATED_CY_INSTANCE_0_BIT : ALLOCATED_CY_INSTANCE_1_BIT);
    return status;
}

/*
 * adf_set_dc_rings_dh895x
 *
 * This function sets the compression rings in the given bank
 */
STATIC CpaStatus adf_set_dc_rings_dh895x (Cpa32U bank_idx,
                                          char* name,
                                          adf_cfg_section_t *pSection,
                                          Cpa8U processNum,
                                          Cpa32U instanceOffset)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_bank_info_t *bank = NULL;

    ICP_CHECK_PARAM_RANGE(bank_idx, 0, dh895_num_banks_per_accel);
    bank = &dh895_banks[bank_idx];
    status = adf_add_param_from_string(bank_idx,
                                       BANK_NUM_STR,
                                       name,
                                       pSection);
     ADF_CHECK_STATUS_RETURN_ON_FAIL(status,
                                    "Failed to add bank parameter %s\n");

    status = adf_add_param_from_string(
                                ICP_DH895xCC_DC_TX_RING_0 + instanceOffset,
                                RING_DC_TX_STR,
                                name,
                                pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to create dc rings\n");

    status = adf_add_param_from_string(
                                /*response ring is #req + 8*/
                                ICP_DH895xCC_DC_TX_RING_0 +
                                  instanceOffset + ICP_DH895xCC_RX_RINGS_OFFSET,
                                RING_DC_RX_STR,
                                name,
                                pSection);
    ADF_CHECK_STATUS_RETURN_ON_FAIL(status, "Failed to create dc rings\n");

    bank->processNum[instanceOffset + ADF_DC_PROCESS_OFFSET] = processNum;
    bank->firstAvailableRing |= ((instanceOffset == 0) ?
                ALLOCATED_DC_INSTANCE_0_BIT : ALLOCATED_DC_INSTANCE_1_BIT);
    return status;
}

/*
 * adf_bank_process_avail
 *
 * This function checks if the bank is available for storing a process for
 * certain @type.
 *
 * Return the bit offset for the @type:
 *   For Crypto:
 *         0 ==> ALLOCATED_CY_INSTANCE_0_BIT,
 *         1 ==> ALLOCATED_CY_INSTANCE_1_BIT,
 *
 *   For Compression:
 *         0 ==> ALLOCATED_DC_INSTANCE_0_BIT,
 *         1 ==> ALLOCATED_DC_INSTANCE_1_BIT,
 *
 * Return ADF_EACH_TYPE_MAX means the bank is full for this @type.
 */
STATIC Cpa32U adf_bank_process_avail(adf_bank_info_t *bank,
                                        enum adf_process_type type)
{
    Cpa32U i = 0, type_offset = ADF_CY_PROCESS_OFFSET;

    if (ADF_DC_PROCESS == type)
        type_offset = ADF_DC_PROCESS_OFFSET;

    for (i = 0; i < ADF_EACH_TYPE_MAX; i++) {
        if (bank->processNum[i + type_offset] == ADF_BANK_PROCESS_NUM_NOT_SET)
            break;
    }
    return i;
}

/*
 * adf_generate_config_params_section_type
 *
 * This function generates the config params for the given section and the
 * given @type.
 */
STATIC CpaStatus adf_generate_config_params_section_type(
                                adf_config_subsection_t *adf_config_subsection,
                                adf_cfg_section_t *pSection,
                                adf_config_default_t* adf_config_default,
                                adf_dev_status_info_t *dev_status,
								Cpa8U number_of_banks_to_skip,
                                Cpa8U processNum,
                                enum adf_process_type type)
{
    Cpa8U i=0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U offset = 0;
    adf_bank_info_t *bank = NULL;
    CpaStatus (*handle)(Cpa32U idx, char *name,
	                    adf_cfg_section_t *pSection,
                        Cpa8U processNum, Cpa32U instanceOffset);

    if (ADF_CY_PROCESS == type) {
        if (!adf_config_default->numConcurrentSymRequestsOverwritten) {
            /* use the default value*/
            status = adf_add_param_from_string(
                        adf_config_default->cyNumConcurrentSymRequests,
                        "%sNumConcurrentSymRequests",
                        adf_config_subsection->name, pSection);

            ADF_CHECK_STATUS_RETURN_ON_FAIL(status,
                        "Failed to add parameter NumConcurrentSymRequests\n");
        }

        if (!adf_config_default->numConcurrentAsymRequestsOverwritten) {
            /* use the default value*/
            status = adf_add_param_from_string(
                        adf_config_default->cyNumConcurrentAsymRequests,
                        "%sNumConcurrentAsymRequests",
                        adf_config_subsection->name, pSection);

            ADF_CHECK_STATUS_RETURN_ON_FAIL(status,"Failed to add parameter"
                        " NumConcurrentAsymRequests\n");
        }

        handle = adf_set_cy_rings_dh895x;
    } else { /* ADF_DC_PROCESS == type */
        if (!adf_config_default->numConcurrentDcRequestsOverwritten) {
            /* use the default value*/
            status = adf_add_param_from_string(
                            adf_config_default->dcNumConcurrentRequests,
                            "%sNumConcurrentRequests",
                            adf_config_subsection->name, pSection);

            ADF_CHECK_STATUS_RETURN_ON_FAIL(status,
                        "Failed to add parameter NumConcurrentRequests\n");
        }

        handle = adf_set_dc_rings_dh895x;
    }

    /*
     * Firstly, we should use the empty bank whose "coreAffinitySet" is false.
     *
     * This keeps the processes use the banks evenly.
     * If there is no empty bank, we will start to use the "used" bank to
     * put two "Cy" (or two "Dc") into one bank.
     */
     for (i = number_of_banks_to_skip; i < dh895_num_banks_per_accel; i++) {
        bank = &dh895_banks[i];
        ICP_CHECK_FOR_NULL_PARAM(bank);

        if (CPA_FALSE != bank->coreAffinitySet) 
            continue; 

        bank->coreAffinitySet = CPA_TRUE;
        bank->coreAffinity = adf_config_subsection->coreAffinity;

        status = adf_add_new_bank_default(i,
                             0, /*only one accelerator*/
                             adf_config_subsection, adf_config_default,
                             dev_status);

        ADF_CHECK_STATUS_RETURN_ON_FAIL(status,
                        "Failed to add default parameters for bank %d\n", i);

        return handle(i, adf_config_subsection->name, pSection, processNum, 0);
    }

    for (i = number_of_banks_to_skip; i < dh895_num_banks_per_accel; i++) {
        bank = &dh895_banks[i];
        ICP_CHECK_FOR_NULL_PARAM(bank);

        offset = adf_bank_process_avail(bank, type);
        if (ADF_EACH_TYPE_MAX == offset)
            continue;

        if ((CPA_TRUE == bank->coreAffinitySet) &&
            (bank->coreAffinity == adf_config_subsection->coreAffinity))
            return handle(i, adf_config_subsection->name, pSection,
                           processNum, offset);
    }

    /* could not affinitize any bank - just use any available bank */
    for (i = number_of_banks_to_skip; i < dh895_num_banks_per_accel; i++) {
        bank = &dh895_banks[i];
        ICP_CHECK_FOR_NULL_PARAM(bank);

        offset = adf_bank_process_avail(bank, type);
        if (ADF_EACH_TYPE_MAX == offset)
            continue;

        ADF_ERROR("Warning: could not find an unallocated bank "
                      "with affinity %d for %s;"
                      " using bank %d, coreAffinity %d instead\n",
                      adf_config_subsection->coreAffinity,
                      adf_config_subsection->name, i, bank->coreAffinity);

        return handle(i, adf_config_subsection->name, pSection,
                processNum, offset);
    }

    return CPA_STATUS_FAIL;
}

/*
 * adf_generate_config_params_section_dh895x
 *
 * This function generates the config params for the given section
 */
STATIC CpaStatus adf_generate_config_params_section_dh895x(
                adf_config_subsection_t *adf_config_subsection,
                adf_cfg_section_t *pSection,
                adf_config_default_t* adf_config_default,
                adf_dev_status_info_t *dev_status,
                Cpa8U processNum,
                Cpa8U number_of_banks_to_skip)
{
    enum adf_process_type type = ADF_DC_PROCESS;

     /* set the core affinity*/
    if (adf_config_subsection->numCoreAffinitySet >= 1)
    {
        adf_config_subsection->coreAffinity =
           adf_config_subsection->coreAffinityArray
            [processNum % adf_config_subsection->numCoreAffinitySet];
    }

    if (0 == strncmp(adf_config_subsection->name, CY, strlen(CY)))
    {
        type = ADF_CY_PROCESS;
    }
    return adf_generate_config_params_section_type(adf_config_subsection,
                        pSection, adf_config_default, dev_status,
                        number_of_banks_to_skip, processNum, type);
}


/*
 * adf_generate_config_params_dh895x
 * This function generates the config params that are not in the config file
 */
CpaStatus adf_generate_config_params_dh895x(
                                adf_config_subsection_t *adf_config_subsection,
                                Cpa32U numProcesses,
                                Cpa8U number_of_banks_to_skip,
                                adf_config_default_t* adf_config_default,
                                adf_dev_status_info_t *dev_status)
{
    adf_cfg_section_t *pSection = ctl_data.config_section;

    Cpa8U i = 0;
    Cpa8U processNum = 0;

    CpaStatus status = CPA_STATUS_SUCCESS;

    if (0 == adf_config_subsection->numCoreAffinitySet)
    {
        adf_config_subsection->coreAffinity = 0;
    }

    if (0 == memcmp(pSection->name, KERNEL, strlen(KERNEL)))
    {
        /* only one process for kernel sections*/
        numProcesses = 1;
        processNum = ADF_BANK_KERN_PROCESS_NUM;
    }
    if (0 == memcmp(pSection->name, DYN, strlen(DYN)))
    {
        /* only one process for DYN section*/
        numProcesses = 1;
        processNum = ADF_BANK_DYN_PROCESS_NUM;
    }

    for (i = 0; i < numProcesses; i++)
    {
        /* find rings and banks for this process */
        status = adf_generate_config_params_section_dh895x(
                        adf_config_subsection,
                        pSection,
                        adf_config_default,
                        dev_status,
                        (
                         (processNum == ADF_BANK_KERN_PROCESS_NUM) ||
                         (processNum == ADF_BANK_DYN_PROCESS_NUM)
                        ) ? processNum : i,
                        number_of_banks_to_skip);

        ADF_CHECK_STATUS_RETURN_ON_FAIL(status,
                "ERROR: Hardware resources unavailable for requested "
                "configuration.\n");
        pSection = pSection->pNext;
    }

    /* reset the instance for the calling function */
    adf_config_subsection->numAcceleratorNumberSet = 0;
    adf_config_subsection->numCoreAffinitySet = 0;
    adf_config_subsection->coreAffinity = 0;
    adf_config_subsection->acceleratorNumber = 0;
    adf_config_default->numConcurrentSymRequestsOverwritten = CPA_FALSE;
    adf_config_default->numConcurrentAsymRequestsOverwritten = CPA_FALSE;
    adf_config_default->numConcurrentDcRequestsOverwritten = CPA_FALSE;
    return status;
}

 /*
 * adf_init_bank_info_dh895x
 *
 * This function created the bank array and initializes it.
 */
 CpaStatus adf_init_bank_info_dh895x(adf_dev_status_info_t *dev_status)
 {
    int i=0;
    int j = 0;

    /* store in global variables */
    dh895_num_banks_per_accel = dev_status->numBanksPerAccel;
    dh895_num_rings_per_bank = dev_status->numRingsPerBank;
    dh895_banks = ICP_MALLOC_GEN(
                dh895_num_banks_per_accel * sizeof(adf_bank_info_t));
    if (NULL == dh895_banks)
    {
        ADF_ERROR("Failed to allocate bank array\n");
        return CPA_STATUS_FAIL;
    }
    for (i=0; i<dh895_num_banks_per_accel; i++)
    {

        for (j = 0; j < DH895xCC_MAX_PROCESS_NUM_PER_BANK; j++)
            dh895_banks[i].processNum[j] = ADF_BANK_PROCESS_NUM_NOT_SET;

        dh895_banks[i].coreAffinitySet = CPA_FALSE;
        dh895_banks[i].firstAvailableRing = 0;
    }
    return CPA_STATUS_SUCCESS;
}

void adf_free_bank_info_dh895x(adf_dev_status_info_t *dev_status)
{
    ICP_FREE(dh895_banks);
    dh895_banks = NULL;
    dh895_num_banks_per_accel = 0;
    dh895_num_rings_per_bank = 0;
}


