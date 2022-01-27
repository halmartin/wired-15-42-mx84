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
 * @file adf_config_affinity.c
 *
 * @ingroup adf_cfg
 *
 * @description Core affinity setup functions
 *
******************************************************************************/
#include "icp_accel_devices.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "adf_config.h"

#define MSIX_BUNDLE_FORMAT "dev%dBundle%d"
#define ACCELERATOR_FORMAT "Accelerator%d"
#define ACCELERATOR_STRING "Accelerator"
#define MSIX_ENTRY_STR_LEN 16
#define LINE_LEN 1024
#define INTERRUPTS "/proc/interrupts"
#define CPU_INFO "/proc/cpuinfo"
#define PROCESSOR "processor"
#define IRQ_AFFINITY_FILE "/proc/irq/%d/smp_affinity"
#define MAX_CPUS 256
#define SETUP_INIT 0xFFFF

STATIC adf_dev_setup_t dev_setup[ADF_MAX_DEVICES] = {{0}};
STATIC int accelerator = 0;
STATIC int system_max_cpu = 0;

void adf_get_system_max_cpu(void)
{
    FILE *cpu_info_file = NULL;
    char line[LINE_LEN];
    char *processor_str = NULL;

    cpu_info_file = fopen(CPU_INFO, "r");
    if (!cpu_info_file)
    {
        ADF_ERROR("Can not open "CPU_INFO" file\n");
        return;
    }

    while (NULL != fgets(line, LINE_LEN, cpu_info_file))
    {
        processor_str = strstr(line, PROCESSOR);
        if (NULL != processor_str)
        {
            if ((processor_str = strstr(line, ":")) != NULL)
            {
                processor_str++;
                system_max_cpu = atoi(processor_str);
            }
        }
    }
    fclose(cpu_info_file);
}

/*
 * adf_core_affinity_init
 *
 * Init setup
 */
int adf_core_affinity_init(adf_dev_status_info_t *dev_status)
{
    int i = 0, x = 0;
    adf_dev_status_info_t *dev_stat = NULL;
    int size = 0;

    for(i = 0; i < ADF_MAX_DEVICES; i++)
    {
        dev_stat = &dev_status[i];

        memset(&dev_setup[i], 0, sizeof(adf_dev_setup_t));
        size = sizeof(uint16_t) * dev_stat->maxNumBanks;
        dev_setup[i].setup = ICP_MALLOC_GEN(size);
        if (NULL == dev_setup[i].setup)
        {
                ADF_ERROR("Failed to allocate setup buffer.\n");
                return 0;
        }

        dev_setup[i].accelId = dev_stat->accelId;
        dev_setup[i].numAccel = dev_stat->numAccel;
        dev_setup[i].maxNumAccel = dev_stat->maxNumAccel;
        dev_setup[i].maxNumBanks = dev_stat->maxNumBanks;
        dev_setup[i].numBanksPerAccel = dev_stat->numBanksPerAccel;

        for(x = 0; x < dev_setup[i].maxNumBanks; x++)
        {
            dev_setup[i].setup[x] = SETUP_INIT;
        }
    }
    adf_get_system_max_cpu();

    return 1;
}

void adf_core_affinity_clean(void)
{
    int i = 0;

    for (i = 0; i < ADF_MAX_DEVICES; i++)
    {
        ICP_FREE(dev_setup[i].setup);
        memset(&dev_setup[i], 0, sizeof(adf_dev_setup_t));
    }
}

/*
 * adf_core_affinity_section_accelerator
 *
 * Set accel number offset
 */
void adf_core_affinity_section_accelerator(const char *param_name)
{
    if (param_name && strstr(param_name, ACCELERATOR_STRING))
    {
        sscanf(param_name, ACCELERATOR_FORMAT, &accelerator);
    }
}

/*
 * adf_param_core_affinity
 *
 * Returns 1 if the param is bank core affinity
 */
int adf_param_core_affinity(int dev, const char *param_name, int *bank_nr)
{
    int i = 0;
    char param_aff_name[ADF_CFG_MAX_CONFIG_STRING_LENGTH];

    if (ADF_MAX_DEVICES < dev)
    {
        ADF_ERROR("Invalid device\n");
        return 0;
    }

    for (i = 0; i < dev_setup[dev].maxNumBanks; i++)
    {
        sprintf(param_aff_name, ADF_ETRMGR_COREID_AFFINITY_FORMAT, i);
        if (strncmp(param_aff_name, param_name, strlen(param_aff_name)) == 0)
        {
            *bank_nr = i;
            return 1;
        }
    }
    return 0;
}

/*
 * adf_set_core_affinity
 *
 * Function sets the core affinity
 */
int adf_set_core_affinity(int dev, int bank, int core)
{
    uint32_t dev_bank_id =0;

    if (ADF_MAX_DEVICES <= dev || MAX_CPUS <= core)
    {
        ADF_ERROR("Invalid device or CPU id\n");
        return -1;
    }

    if (dev_setup[dev].maxNumBanks <= bank)
    {
        ADF_ERROR("Invalid bank number\n");
        return -1;
    }

    if (dev_setup[dev].maxNumAccel <= accelerator)
    {
        ADF_ERROR("Invalid accelerator number\n");
        return -1;
    }

    /* Get device bank id */
    dev_bank_id = (accelerator * dev_setup[dev].numBanksPerAccel) + bank;

    if (core > system_max_cpu)
    {
        ADF_ERROR("Invalid core affinity settings for accelerator %d bank %d."
                  " Max cpu number is %d. Defaulting to 0\n",
                  accelerator, bank, system_max_cpu);
        return dev_setup[dev].setup[dev_bank_id] = 0;
    }
    else
    {
        return dev_setup[dev].setup[dev_bank_id] = core;
    }
}

/*
 * adf_set_core_affinity_gen
 *
 * Function sets the core affinity for auto generated values.
 */
int adf_set_core_affinity_gen(int dev, int accel, int bank, int core)
{
    uint32_t dev_bank_id =0;

    if (ADF_MAX_DEVICES <= dev || MAX_CPUS <= core)
    {
        ADF_ERROR("Invalid device or CPU id\n");
        return -1;
    }

    if (dev_setup[dev].maxNumBanks <= bank)
    {
        ADF_ERROR("Invalid bank number\n");
        return -1;
    }

    if (dev_setup[dev].maxNumAccel <= accel)
    {
        ADF_ERROR("Invalid accelerator number\n");
        return -1;
    }

    /* Get device bank id */
    dev_bank_id = (accel * dev_setup[dev].numBanksPerAccel) + bank;

    if (core > system_max_cpu)
    {
        ADF_ERROR("Invalid core affinity settings for accelerator %d bank %d."
                  " Max cpu number is %d. Defaulting to 0\n",
                  accel, bank, system_max_cpu);
        return dev_setup[dev].setup[dev_bank_id] = 0;
    }
    else
    {
        return dev_setup[dev].setup[dev_bank_id] = core;
    }
}

/*
 * adf_config_core_affinity
 *
 * Function sets the core affinity
 */
void adf_config_core_affinity(int dev, adf_dev_status_info_t *dev_status)
{
    int i = 0, irq = 0, bank = 0;
    char buff[MSIX_ENTRY_STR_LEN];
    char affinity_file_name[LINE_LEN];
    char line[LINE_LEN];
    FILE *itr_file = NULL, *affinity_file = NULL;
    int ret = 0;

    for(i = 0; i < dev_setup[dev].maxNumBanks; i++)
    {
        bank = i;
        if (SETUP_INIT == dev_setup[dev].setup[bank])
        {
            continue;
        }
        itr_file = fopen(INTERRUPTS , "r");
        if (!itr_file)
        {
            ADF_ERROR("Can not open "INTERRUPTS" file\n");
            return;
        }

        sprintf(buff, MSIX_BUNDLE_FORMAT, dev, bank);
        while (NULL != fgets(line, LINE_LEN, itr_file))
        {
            irq = atoi(line);
            if (0 == irq)
            {
                continue;
            }
            if (NULL != strstr(line, buff))
            {
                sprintf(affinity_file_name, IRQ_AFFINITY_FILE, irq);
                affinity_file = fopen(affinity_file_name, "w");
                if (!affinity_file)
                {
                    ADF_ERROR("Can not open %s file\n", affinity_file_name);
                    fclose(itr_file);
                    return;
                }
                ret = fprintf(affinity_file, "%x", 1 <<
                    dev_setup[dev].setup[bank]);
                if (ret < 0)
                {
                    ADF_ERROR("Failed updating core affinity. Error %d\n", ret);
                }
                fclose(affinity_file);
                break;
            }
        }
        fclose(itr_file);
    }
}

