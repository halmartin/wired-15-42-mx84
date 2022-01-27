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
******************************************************************************/

/******************************************************************************
 * @file adf_config_rings.c
 *
 * @ingroup adf_cfg
 *
 * @description Generates parameters that are not in the config file
 *
******************************************************************************/

#include "icp_accel_devices.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "adf_config.h"

#define ADMIN_BANK_1 0
#define ADMIN_BANK_2 8
#define NUM_ADMIN_RINGS 2

#define ACCEL_NUM_0 0
#define ACCEL_NUM_1 1
#define ACCEL_NUM_2 2
#define ACCEL_NUM_3 3
/*
 * Internal structure storing affinity and available rings informations in
 * the banks
 */
typedef struct adf_bank_info_s
{
    CpaBoolean coreAffinitySet;
    Cpa8U coreAffinity;
    Cpa8U firstAvailableRing;
} adf_bank_info_t;

adf_bank_info_t **banks = NULL;

STATIC int num_banks_per_accel = 0;
STATIC int num_rings_per_bank = 0;

STATIC char* cy_ring_name_array[NUM_OF_CY_RINGS] = {
    RING_ASYM_TX_STR,
    RING_ASYM_RX_STR,
    RING_SYM_TX_HI_STR,
    RING_SYM_RX_HI_STR,
    RING_SYM_TX_LO_STR,
    RING_SYM_RX_LO_STR};

STATIC char* dc_ring_name_array[NUM_OF_DC_RINGS] = {
    DC_RING_TX_STR, DC_RING_RX_STR};

/*
 * adf_add_param_from_string
 *
 * This function adds the parameter with param_value and with the name composed
 * of the string format and the name in the section passed in.
 */
STATIC CpaStatus adf_add_param_from_string(Cpa32U param_value,
                                           char* string_format,
                                           char* name,
                                           adf_cfg_section_t *pSection)

{
    char key_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    char val_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};

    sprintf(key_str, string_format, name);
    sprintf(val_str, "%d", param_value);
    return adf_add_param(key_str, val_str, pSection);
}

/*
 * adf_add_param_from_string_int
 *
 * This function adds the parameter with param_value and with the name composed
 * of the string format and the string parameter in the section passed in.
 */
STATIC CpaStatus adf_add_param_from_string_int(Cpa32U param_value,
                                   char* string_format,
                                   Cpa8U string_param,
                                   adf_cfg_section_t *pSection)

{
    char key_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    char val_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};

    sprintf(key_str, string_format, string_param);
    sprintf(val_str, "%d", param_value);
    return adf_add_param(key_str, val_str, pSection);
}

/*
 * adf_add_new_bank_default
 *
 * This function adds the default parameters associated with a new bank
 */
STATIC CpaStatus adf_add_new_bank_default(Cpa8U bank,
                                Cpa8U accelNumber,
                                adf_config_subsection_t *adf_config_subsection,
                                adf_config_default_t* adf_config_default)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char val_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    adf_cfg_section_t *pAccelSection = NULL;

    sprintf(val_str, ACCEL_STR, accelNumber);

    status = adf_find_section(ctl_data.config_section,
                             val_str,
                             &pAccelSection);

    if (status != CPA_STATUS_SUCCESS)
    {
        status = adf_add_section_end(&(ctl_data.config_section),
                                                val_str,
                                                &pAccelSection);
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to create the accelerator section\n");
            return status;
        }
    }
    status = adf_add_param_from_string_int(adf_config_subsection->coreAffinity,
                                   ADF_ETRMGR_COREID_AFFINITY_FORMAT,
                                   bank,
                                   pAccelSection);

    if (status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Failed to add core affinity parameter\n");
        return CPA_STATUS_FAIL;
    }

    if (0 > adf_set_core_affinity_gen(ctl_data.device_id, accelNumber, bank,
                                          adf_config_subsection->coreAffinity))
    {
        ADF_ERROR("Failed to set core affinity\n");
        return CPA_STATUS_FAIL;
    }

    status = adf_add_param_from_string_int(
                                adf_config_default->interruptCoalescingEnabled,
                                ADF_ETRMGR_COALESCING_ENABLED_FORMAT,
                                bank,
                                pAccelSection);

    if (status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Failed to add interrupt coalescing parameter\n");
        return CPA_STATUS_FAIL;
    }

    status = adf_add_param_from_string_int(
                                adf_config_default->interruptCoalescingTimerNs,
                                ADF_ETRMGR_COALESCE_TIMER_FORMAT,
                                bank,
                                pAccelSection);

    if (status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Failed to add bank coalescing timer parameter\n");
        return CPA_STATUS_FAIL;
    }

    status = adf_add_param_from_string_int(
                            adf_config_default->interruptCoalescingNumResponses,
                            ADF_NEARLY_FULL,
                            bank,
                            pAccelSection);

    if (status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Failed to add coalescing num responses parameter\n");
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_set_cy_rings
 *
 * This function sets the crypto rings in the given bank
 */
STATIC CpaStatus adf_set_cy_rings(Cpa8U* first_ring,
                                  Cpa32U bank,
                                  char * name,
                                  adf_cfg_section_t *pSection)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    int i=0;

    status = adf_add_param_from_string(bank,
                                       BANK_NUM_STR,
                                       name,
                                       pSection);

    if (status != CPA_STATUS_SUCCESS)
    {
         ADF_ERROR("Failed to add bank parameter\n");
         return status;
     }

     for (i=0; i<NUM_OF_CY_RINGS; i++)
     {
        status = adf_add_param_from_string(*first_ring + i,
                                cy_ring_name_array[i],
                                name,
                                pSection);

        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to create crypto rings\n");
            return status;
        }
    }

    *first_ring += NUM_OF_CY_RINGS;
    return status;
}

/*
 * adf_set_dc_rings
 *
 * This function sets the compression rings in the given bank
 */
STATIC CpaStatus adf_set_dc_rings(Cpa8U* first_ring,
                                  Cpa32U bank, char* name,
                                  adf_cfg_section_t *pSection)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    int i=0;

    status = adf_add_param_from_string(bank,
                                       BANK_NUM_STR,
                                       name,
                                       pSection);
    if (status != CPA_STATUS_SUCCESS)
    {
         ADF_ERROR("Failed to add bank parameter %s\n");
         return status;
    }

    for (i=0; i<NUM_OF_DC_RINGS; i++)
    {
        status = adf_add_param_from_string(*first_ring + i,
                                dc_ring_name_array[i],
                                name,
                                pSection);

        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to create dc rings\n");
            return status;
        }
    }

    *first_ring += NUM_OF_DC_RINGS;
    return status;
}

/*
 * adf_generate_config_params_section
 *
 * This function generates the config params for the given section
 */
STATIC CpaStatus adf_generate_config_params_section(
                adf_config_subsection_t *adf_config_subsection,
                adf_cfg_section_t *pSection,
                Cpa8U number_of_banks_to_skip,
                adf_config_default_t* adf_config_default)
{
    Cpa8U i=0,j=0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U accelNumber = 0;
    Cpa8U executionEngine = 0;
    Cpa8U otherAcceleratorNumber = 0;

    if (strncmp(adf_config_subsection->name, CY, strlen(CY)) == 0)
    {
        /* regenerate execution engine and accel number for accel number
         * acceleratorNumber and executionEngine have been merged with the
         * following mapping:
         ---------------------------------------------------------
         | New format (version 2)      | Old format (version 1)   |
         ----------------------------------------------------------
         | acceleratorNumber = 0       | AcceleratorNumber = 0    |
         |                             | ExecutionEngine = 0      |
         ----------------------------------------------------------
         | acceleratorNumber = 1       | AcceleratorNumber = 0    |
         |                             | ExecutionEngine = 1      |
         ----------------------------------------------------------
         | acceleratorNumber = 2       | AcceleratorNumber = 1    |
         |                             | ExecutionEngine = 0      |
         ----------------------------------------------------------
         | acceleratorNumber = 3       | AcceleratorNumber = 1    |
         |                             | ExecutionEngine = 1      |
         ---------------------------------------------------------- */
        switch(adf_config_subsection->acceleratorNumber)
        {
            case ACCEL_NUM_0:
                accelNumber = 0;
                executionEngine = 0;
                break;
            case ACCEL_NUM_1:
                accelNumber = 0;
                executionEngine = 1;
                break;
            case ACCEL_NUM_2:
                accelNumber = 1;
                executionEngine = 0;
                break;
            case ACCEL_NUM_3:
                accelNumber = 1;
                executionEngine = 1;
                break;
            default:
                ADF_ERROR("ERROR: AcceleratorNumber out of range "
                        "[%s]\n", adf_config_subsection->name);
                return CPA_STATUS_FAIL;
        }

        /* accelNumber is being validated against SKU */
        if(NULL == banks[accelNumber])
        {
            ADF_ERROR("ERROR: Invalid accelerator number (%d) \n ",accelNumber);
            return CPA_STATUS_FAIL;;
        }

        status = adf_add_param_from_string(accelNumber,
                                           "%sAcceleratorNumber",
                                           adf_config_subsection->name,
                                           pSection);
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to add accelerator number parameter\n");
            return status;
        }
        status = adf_add_param_from_string(executionEngine,
                                       "%sExecutionEngine",
                                       adf_config_subsection->name,
                                       pSection);
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to add parameter execution engine\n");
            return status;
        }
        if (adf_config_default->numConcurrentSymRequestsOverwritten
            == CPA_FALSE)
        {
            /* use the default value*/
            status = adf_add_param_from_string(
                            adf_config_default->cyNumConcurrentSymRequests,
                            "%sNumConcurrentSymRequests",
                            adf_config_subsection->name,
                            pSection);
            if (status != CPA_STATUS_SUCCESS)
            {
                ADF_ERROR("Failed to add parameter NumConcurrentSymRequests\n");
                return status;
            }
        }

        if (adf_config_default->numConcurrentAsymRequestsOverwritten
            == CPA_FALSE)
        {
            /* use the default value*/
            status = adf_add_param_from_string(
                            adf_config_default->cyNumConcurrentAsymRequests,
                            "%sNumConcurrentAsymRequests",
                            adf_config_subsection->name,
                            pSection);
            if (status != CPA_STATUS_SUCCESS)
            {
                ADF_ERROR("Failed to add parameter"
                    " NumConcurrentAsymRequests\n");
                return status;
            }
        }
        for (i=number_of_banks_to_skip; i<num_banks_per_accel; i++)
        {
            adf_bank_info_t * pCurrentbank = &(banks[accelNumber][i]);
            if ((pCurrentbank->coreAffinitySet == CPA_TRUE) &&
                (pCurrentbank->coreAffinity ==
                    adf_config_subsection->coreAffinity))
            {
                if ((num_rings_per_bank - pCurrentbank->firstAvailableRing)
                        >= NUM_OF_CY_RINGS)
                {
                    return adf_set_cy_rings(&(pCurrentbank->firstAvailableRing),
                                            i,
                                            adf_config_subsection->name,
                                            pSection);
                }
            }
        }
        for (i=number_of_banks_to_skip; i<num_banks_per_accel; i++)
        {
            adf_bank_info_t * pCurrentbank = &(banks[accelNumber][i]);
            if (pCurrentbank->coreAffinitySet == CPA_FALSE)
            {
                pCurrentbank->coreAffinitySet = CPA_TRUE;
                pCurrentbank->coreAffinity =
                    adf_config_subsection->coreAffinity;
                status = adf_add_new_bank_default(
                                         i,
                                         accelNumber,
                                         adf_config_subsection,
                                         adf_config_default);
                if (status != CPA_STATUS_SUCCESS)
                {
                    ADF_ERROR("Failed to add default parameters for bank %d\n",
                                i);
                    return status;
                }
                return  adf_set_cy_rings(
                                        &(pCurrentbank->firstAvailableRing),
                                        i,
                                        adf_config_subsection->name,
                                        pSection);
            }
        }


        /* could not affinitize any bank on current accel - check other accels*/
        for (j=0; j<adf_config_subsection->numAcceleratorNumberSet; j++)
        {
            if (j == adf_config_subsection->acceleratorNumber)
            {
                /* No need to check current accelerator again */
                j++;
            }
            /* set the accelerator number*/
            if (adf_config_subsection->numAcceleratorNumberSet >= 1)
            {
                otherAcceleratorNumber =
                       adf_config_subsection->acceleratorNumberArray [j];
            }
            /* get the execution engine/accelerator combo */
            if (otherAcceleratorNumber == ACCEL_NUM_0)
            {
                accelNumber = 0;
                executionEngine = 0;
            }
            else if (otherAcceleratorNumber == ACCEL_NUM_1)
            {
                accelNumber = 0;
                executionEngine = 1;
            }
            else if (otherAcceleratorNumber == ACCEL_NUM_2)
            {
                accelNumber = 1;
                executionEngine = 0;
            }
            else if (otherAcceleratorNumber == ACCEL_NUM_3)
            {
                accelNumber = 1;
                executionEngine = 1;
            }
            else
            {
                ADF_ERROR("ERROR: AcceleratorNumber out of range "
                      "[%s]\n", adf_config_subsection->name);
                return CPA_STATUS_FAIL;
            }

            for (i=number_of_banks_to_skip; i<num_banks_per_accel; i++)
            {
                adf_bank_info_t * pCurrentbank = &(banks[accelNumber][i]);
                if ((num_rings_per_bank - pCurrentbank->firstAvailableRing)
                            >= NUM_OF_CY_RINGS)
                {
                    ADF_ERROR("Warning: could not find any bank with affinity "
                        "%d for %s; using %d instead\n",
                        adf_config_subsection->coreAffinity,
                        adf_config_subsection->name,
                        pCurrentbank->coreAffinity);
                    return adf_set_cy_rings(&(pCurrentbank->firstAvailableRing),
                                            i,
                                            adf_config_subsection->name,
                                            pSection);
                }
            }
        }
    }
    else /* DC */
    {
        if (adf_config_subsection->acceleratorNumber == ACCEL_NUM_0)
        {
            accelNumber = 0;
        }
        else if (adf_config_subsection->acceleratorNumber == ACCEL_NUM_1)
        {
            accelNumber = 1;
        }
        else
        {
            ADF_ERROR("ERROR: AcceleratorNumber out of range "
                  "[%s]\n", adf_config_subsection->name);
            return CPA_STATUS_FAIL;
        }

        /* accelNumber is being validated against SKU */
        if(NULL == banks[accelNumber])
        {
            ADF_ERROR("ERROR: Invalid accelerator number (%d) \n ",accelNumber);
            return CPA_STATUS_FAIL;;
        }

        status = adf_add_param_from_string(accelNumber,
                                           "%sAcceleratorNumber",
                                           adf_config_subsection->name,
                                           pSection);
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to add accelerator number parameter\n");
            return status;
        }

        if (adf_config_default->numConcurrentDcRequestsOverwritten
            == CPA_FALSE)
        {
            /* use the default value*/
            status = adf_add_param_from_string(
                            adf_config_default->dcNumConcurrentRequests,
                            "%sNumConcurrentRequests",
                            adf_config_subsection->name,
                            pSection);

            if (status != CPA_STATUS_SUCCESS)
            {
                ADF_ERROR("Failed to add parameter sNumConcurrentRequests\n");
                return status;
            }
        }

        for (i=number_of_banks_to_skip; i<num_banks_per_accel; i++)
        {
            adf_bank_info_t * pCurrentbank = &(banks[accelNumber][i]);
            if ((pCurrentbank->coreAffinitySet == CPA_TRUE) &&
                (pCurrentbank->coreAffinity ==
                    adf_config_subsection->coreAffinity))
            {
                if ((num_rings_per_bank - pCurrentbank->firstAvailableRing)
                        >= NUM_OF_DC_RINGS)
                {
                    return adf_set_dc_rings(&(pCurrentbank->firstAvailableRing),
                                            i,
                                            adf_config_subsection->name,
                                            pSection);
                }
            }
        }
        for (i=number_of_banks_to_skip; i<num_banks_per_accel; i++)
        {
            adf_bank_info_t * pCurrentbank = &(banks[accelNumber][i]);
            if (pCurrentbank->coreAffinitySet == CPA_FALSE)
            {
                pCurrentbank->coreAffinitySet = CPA_TRUE;
                pCurrentbank->coreAffinity =
                    adf_config_subsection->coreAffinity;

                status = adf_add_new_bank_default(i,
                                                  accelNumber,
                                                  adf_config_subsection,
                                                  adf_config_default);

                if (status != CPA_STATUS_SUCCESS)
                {
                    ADF_ERROR("Failed to add default parameters for bank %d\n",
                        i);
                    return status;
                }
                return adf_set_dc_rings(&(pCurrentbank->firstAvailableRing),
                                        i,
                                        adf_config_subsection->name,
                                        pSection);
            }
        }
        /* could not affinitize any bank on current accel - check other accels*/
        for (j=0; j<adf_config_subsection->numAcceleratorNumberSet; j++)
        {
            if (j == adf_config_subsection->acceleratorNumber)
            {
                /* No need to check current accelerator again */
                j++;
            }
            /* set the accelerator number*/
            if (adf_config_subsection->numAcceleratorNumberSet >= 1)
            {
                otherAcceleratorNumber =
                       adf_config_subsection->acceleratorNumberArray [j];
            }
            if (adf_config_subsection->acceleratorNumber == ACCEL_NUM_0)
            {
                accelNumber = 0;
            }
            else if (adf_config_subsection->acceleratorNumber == ACCEL_NUM_1)
            {
                accelNumber = 1;
            }
            else
            {
                ADF_ERROR("ERROR: AcceleratorNumber out of range "
                      "[%s]\n", adf_config_subsection->name);
                return CPA_STATUS_FAIL;
            }

            for (i=number_of_banks_to_skip; i<num_banks_per_accel; i++)
            {
                adf_bank_info_t * pCurrentbank = &(banks[accelNumber][i]);

               if ((num_rings_per_bank - pCurrentbank->firstAvailableRing)
                            >= NUM_OF_DC_RINGS)
               {
                    ADF_ERROR("Warning: could not find any bank with affinity "
                        "%d for %s; using %d instead\n",
                        adf_config_subsection->coreAffinity,
                        adf_config_subsection->name,
                        pCurrentbank->coreAffinity);

                    return adf_set_dc_rings(&(pCurrentbank->firstAvailableRing),
                                            i,
                                            adf_config_subsection->name,
                                            pSection);
                }
            }
        }
    }
    return CPA_STATUS_FAIL;
}

/*
 * adf_generate_config_params
 * This function generates the config params that are not in the config file
 */
CpaStatus adf_generate_config_params(
                                adf_config_subsection_t *adf_config_subsection,
                                Cpa32U numProcesses,
                                Cpa8U number_of_banks_to_skip,
                                adf_config_default_t* adf_config_default,
                                adf_dev_status_info_t *dev_status)
{
    Cpa32U i = 0;
    adf_cfg_section_t *pSection = ctl_data.config_section;
    CpaStatus status = CPA_STATUS_SUCCESS;

    /* default missing params to 0 */
    if (adf_config_subsection->numAcceleratorNumberSet == 0)
    {
        ADF_DEBUG("Warning acceleratorNumber not set for instance %s ",
                      adf_config_subsection->name);
        ADF_DEBUG("defaulting to 0\n");
        adf_config_subsection->acceleratorNumber = 0;
    }

    if (adf_config_subsection->numCoreAffinitySet == 0)
    {
        ADF_DEBUG("Warning coreAffinity not set for instance %s ",
                        adf_config_subsection->name);
        ADF_DEBUG("defaulting to 0\n");
        adf_config_subsection->coreAffinity = 0;
    }

    if (memcmp(pSection->name, KERNEL, strlen(KERNEL)) == 0)
    {
        /* only one process for kernel sections*/
        numProcesses = 1;
    }
    if (memcmp(pSection->name, DYN, strlen(DYN)) == 0)
    {
        /* only one process for DYN section*/
        numProcesses = 1;
    }

    for (i = 0; i < numProcesses; i++)
    {
        /* set the core affinity*/
        if (adf_config_subsection->numCoreAffinitySet >= 1)
        {
            adf_config_subsection->coreAffinity =
               adf_config_subsection->coreAffinityArray
                [i % adf_config_subsection->numCoreAffinitySet];
        }

        /* set the accelerator number*/
        if (adf_config_subsection->numAcceleratorNumberSet >= 1)
        {
            adf_config_subsection->acceleratorNumber =
               adf_config_subsection->acceleratorNumberArray
                [i % adf_config_subsection->numAcceleratorNumberSet];
        }

        /* Make sure we do not access banks on
         * an accelerator that doesn't exist
         */
        if ((adf_config_subsection->acceleratorNumber>>1) <
                dev_status->numAccel)
        {
            /* find rings and banks for this process */
            status = adf_generate_config_params_section(adf_config_subsection,
                                                        pSection,
                                                        number_of_banks_to_skip,
                                                        adf_config_default);
        }
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("ERROR: Hardware resources unavailable for requested "
             "configuration.\n");
            return status;
        }
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
 * adf_set_admin_param
 * This function is used to set the admin parameters for all the accelerators
 */
CpaStatus adf_set_admin_param(int nb_accel)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U i=0;
    adf_cfg_section_t *section = NULL;
    status = adf_find_section(ctl_data.config_section,
                              GENERAL_SEC,
                              &section);

    if ( status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("ERROR: Could not find GENERAL section\n");
        return status;
    }

    for (i=0; i < nb_accel; i++)
    {
        status = adf_add_param_from_string_int(0, /* use bank 0*/
                                               ACCEL_ADMINBANK_STR,
                                               i,
                                               section);
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to add admin ring for accelerator %d\n", i);
            return status;
        }
        status = adf_add_param_from_string_int(i,
                                               ACCEL_ACCELNUM_STR,
                                               i,
                                               section);
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to add accelerator number for accelerator %d\n",
                i);
            return status;
        }
        status = adf_add_param_from_string_int(0, /* use ring 0*/
                                               ACCEL_ADMIN_TX_STR,
                                               i,
                                               section);
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to add accelerator number for accelerator %d\n",
                i);
            return status;
        }
        status = adf_add_param_from_string_int(1, /* use ring 1*/
                                               ACCEL_ADMIN_RX_STR,
                                               i,
                                               section);
        if (status != CPA_STATUS_SUCCESS)
        {
            ADF_ERROR("Failed to add accelerator number for accelerator %d\n",
                i);
            return status;
        }
    }
    return status;
}

 /*
 * adf_init_bank_info
 *
 * This function created the bank array and initializes it.
 */
 CpaStatus adf_init_bank_info(adf_dev_status_info_t *dev_status)
 {
    int i=0;
    int j=0;

    /* store in global variables */
    num_banks_per_accel = dev_status->numBanksPerAccel;
    num_rings_per_bank = dev_status->numRingsPerBank;
    banks = ICP_MALLOC_GEN(dev_status->numAccel * sizeof(adf_bank_info_t *));
    if (banks == NULL)
    {
        ADF_ERROR("Failed to allocate bank array\n");
        return CPA_STATUS_FAIL;
    }
    for (i=0; i< dev_status->numAccel; i++)
    {
        banks[i] = malloc(num_banks_per_accel * sizeof(adf_bank_info_t));
        if (banks[i] == NULL)
        {
            ADF_ERROR("Failed to allocate bank array\n");
            return CPA_STATUS_FAIL;
        }
        for (j=0; j<num_banks_per_accel; j++)
        {
            banks[i][j].coreAffinitySet = CPA_FALSE;
            if (((DEV_DH89XXCC == dev_status->type) ||
                 (DEV_C2XXX == dev_status->type)) &&
                ((j == ADMIN_BANK_1) ||
                 (j == ADMIN_BANK_2)) )
            {
                /* the 2 admin rings are not available */
                banks[i][j].firstAvailableRing = NUM_ADMIN_RINGS;
            }
            else
            {
                banks[i][j].firstAvailableRing = 0;
            }
        }
    }
    return CPA_STATUS_SUCCESS;
}

 void adf_free_bank_info(adf_dev_status_info_t *dev_status)
 {
    int i=0;
    for (i=0; i< dev_status->numAccel; i++)
    {
        ICP_FREE(banks[i]);
    }
    ICP_FREE(banks);
    banks = NULL;
    num_banks_per_accel = 0;
    num_rings_per_bank = 0;
}
