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

/*****************************************************************************
 * @file adf_transport_debug.c
 *
 * @description
 *      Transport debug code in which ADF uses the debug feature to show the
 *      contents of the rings in the proc filesystem
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_platform.h"
#include "adf_cfg.h"
#include "icp_adf_debug.h"

/* use bits 0-31 */
#define ADF_DEBUG_REG_BITS        32
/* size in one line*/
#define ADF_DEBUG_LINE_LEN        60
/* ring dir name */
#define ADF_DEBUG_ENTRY          "et_ring_ctrl"
/* num bytes allocated for dir names */
#define ADF_DEBUG_DIR_NAME_LEN    8
/* naming for banks */
#define ADF_DEBUG_BANK_DIR       "bank_%d"
/* naming for rings  */
#define ADF_DEBUG_RING           "ring_%d"
/* naming for config */
#define ADF_DEBUG_CONF           "conf"

typedef struct debug_private_s{
    icp_accel_dev_t *accel_dev;
    Cpa32U bankNumber;
} debug_private_t;

/*
 * adf_debug_print_bank_rings
 * Print the ring info of a bank
 */
STATIC int adf_debug_print_bank_rings(void *prv, char* data,
                               int size, int offset)
{
    Cpa32S len = 0, i = 0, bits_to_show = 0;
    Cpa32U head = 0, tail = 0, space = 0, irq_csr = 0;
    Cpa32U empty_stat = 0, nearly_empty_stat = 0;
    Cpa32U *csr_base_addr = NULL;
    icp_et_ring_data_t *ring = NULL;
    icp_accel_dev_t *accel_dev = NULL;
    icp_et_ring_bank_data_t *bank = NULL;
    debug_private_t *dbg_data = (debug_private_t*) prv;
    icp_etr_priv_data_t *bankData = NULL;

    accel_dev = dbg_data->accel_dev;

    bankData = (icp_etr_priv_data_t*)accel_dev->pCommsHandle;
    bank = &(bankData->banks[dbg_data->bankNumber]);

    len = snprintf(data+len, size-len,"------- Bank %d Configuration -------\n",
                                                            bank->bankNumber);
    if(bank->timedCoalescEnabled)
    {
        len += snprintf(data+len, size-len,
              " Interrupt Coalescing Enabled\n");
        len += snprintf(data+len, size-len,
              " Interrupt Coalescing Counter = %d\n",
                            bank->coalIntrDelayCount);
    }
    else
    {
        len += snprintf(data+len, size-len,
              " Interrupt Coalescing Disabled\n");
    }
    if(bank->numberMsgCoalescEnabled)
    {
        len += snprintf(data+len, size-len,
              " Interrupt Coalescing based on number of messages enabled\n");
    }

    bits_to_show = ADF_DEBUG_REG_BITS -
        (ADF_DEBUG_REG_BITS - GET_NUM_RINGS_PER_BANK(accel_dev)) - 1;

    len += snprintf(data+len, size-len, " Interrupt mask:\t");

    for(i = bits_to_show; i >= 0; i--)
    {
        if( bank->interruptMask & (1 << i))
        {
            len += snprintf(data+len, size-len, " 1");
        }
        else
        {
          len += snprintf(data+len, size-len, " 0");
        }
    }
    len += snprintf(data+len, size-len, "\n");
    len += snprintf(data+len, size-len, " User interrupt mask:\t");
    for(i = bits_to_show; i >= 0; i-- )
    {
        if( bank->interruptUserMask & (1 << i))
        {
            len += snprintf(data+len, size-len, " 1");
        }
        else
        {
            len += snprintf(data+len, size-len, " 0");
        }
    }
    len += snprintf(data+len, size-len, "\n");
    len += snprintf(data+len, size-len, " Polling mask:\t\t");
    for(i = bits_to_show; i >= 0; i-- )
    {
        if( bank->pollingMask & (1 << i))
        {
            len += snprintf(data+len, size-len, " 1");
        }
        else
        {
            len += snprintf(data+len, size-len, " 0");
        }
    }
    if(bank->allocatedRings)
    {
        Cpa32U ring_nr = ADF_DEBUG_REG_BITS;
        for(i = 0; i < ADF_DEBUG_REG_BITS-1; i++)
        {
            if(bank->allocatedRings & 1 << i)
            {
                ring_nr = i;
                break;
            }
        }

        if(ADF_DEBUG_REG_BITS != ring_nr)
        {
            ring = &(bank->rings[ring_nr]);
            csr_base_addr = ring->ringCSRAddress;
        }
    }
    if(csr_base_addr)
    {
        if(bank->timedCoalescEnabled)
        {
            irq_csr = READ_CSR_INT_COL_EN(bank->bankNumber);
            len += snprintf(data+len, size-len, "\n");
            len += snprintf(data+len, size-len, " Coalesc reg:\t\t");
            for(i = bits_to_show; i >= 0; i-- )
            {
                if( irq_csr & (1 << i))
                {
                    len += snprintf(data+len, size-len, " 1");
                }
                else
                {
                    len += snprintf(data+len, size-len, " 0");
                }
            }
        }
        if(!bank->timedCoalescEnabled ||
               bank->numberMsgCoalescEnabled)
        {
            irq_csr = READ_CSR_INT_EN(bank->bankNumber);
            len += snprintf(data+len, size-len, "\n");
            len += snprintf(data+len, size-len, " Interrupt reg:\t\t");
            for(i = bits_to_show; i >= 0; i-- )
            {
                if( irq_csr & (1 << i))
                {
                    len += snprintf(data+len, size-len, " 1");
                }
                else
                {
                    len += snprintf(data+len, size-len, " 0");
                }
            }
        }
    }

    if(!bank->allocatedRings)
    {
        len += snprintf(data+len, size-len,"\nThere are no rings allocated.\n");
        return 0;
    }

    for (i = 0; i < GET_NUM_RINGS_PER_BANK(accel_dev); i++)
    {
        if(bank->allocatedRings & (1 << i))
        {
            ring = &(bank->rings[i]);
            csr_base_addr = ring->ringCSRAddress;
            empty_stat = READ_CSR_E_STAT(bank->bankNumber);
            nearly_empty_stat = READ_CSR_NE_STAT(bank->bankNumber);
        }
    }
    len += snprintf(data+len, size-len, "\n");
    len += snprintf(data+len, size-len, " Bank empty stat:\t");
    for(i = bits_to_show; i >= 0; i-- )
    {
        if( empty_stat & (1 << i))
        {
            len += snprintf(data+len, size-len, " 1");
        }
        else
        {
            len += snprintf(data+len, size-len, " 0");
        }
    }
    len += snprintf(data+len, size-len, "\n");
    len += snprintf(data+len, size-len, " Bank nempty stat:\t");
    for(i = bits_to_show; i >= 0; i-- )
    {
        if( nearly_empty_stat & (1 << i))
        {
            len += snprintf(data+len, size-len, " 1");
        }
        else
        {
            len += snprintf(data+len, size-len, " 0");
        }
    }

    len += snprintf(data+len, size-len, "\n------- Rings: \n");

    for (i = 0; i < GET_NUM_RINGS_PER_BANK(accel_dev); i++)
    {
        if(bank->allocatedRings & (1 << i))
        {
            OsalAtomic *pRingInflight;

            ring = &(bank->rings[i]);
            pRingInflight = ((OsalAtomic*)accel_dev->pRingInflight) +
                                                     ring->inFlightIndex;
            csr_base_addr = ring->ringCSRAddress;
            head = READ_CSR_RING_HEAD(ring->bankNumber,
                                            ring->ringNumber);
            tail = READ_CSR_RING_TAIL(ring->bankNumber,
                                            ring->ringNumber);
            if(head == tail)
            {
                /* Need to reread empty status */
                empty_stat = READ_CSR_E_STAT(bank->bankNumber);
                space = ((empty_stat & (1 << i))) ? ring->sizeInBytes : 0;
            }
            else
            {
                space = (tail > head) ? (ring->sizeInBytes - (tail - head)) :
                   (ring->sizeInBytes - (ring->sizeInBytes - (head - tail)));
            }

            len += snprintf(data+len, size-len, "Ring Number: %2d,"
             " Config: %4x, Base Addr: %p"
             " Head: %4x, Tail: %4x, Space: %4x, inflights: %4d, Name: %s\n",
                            i, ring->ringConfig,
                            ring->ringBaseAddress,
                            head, tail,
                            space,
                            (int)osalAtomicGet(pRingInflight),
                            ring->serviceName);
        }
    }
    len += snprintf(data+len, size-len,
            "-------------------------------------\n");
    return 0;
}

/*
 * adf_debug_print_ring_data
 * Print the data to the ring debug file
 */
STATIC int adf_debug_print_ring_data(void *prv, char* data,
                                             int size, int offset)
{
    Cpa32U len = 0, i = 0, x = 0;
    Cpa32U *ptr = NULL;
    icp_et_ring_data_t *ring = (icp_et_ring_data_t*) prv;
    Cpa32U *csr_base_addr = ring->ringCSRAddress;
    Cpa32U head = 0, tail = 0, space = 0,
           empty_stat = 0, nearly_empty_stat = 0;

    if(offset == 0)
    {
        empty_stat = READ_CSR_E_STAT(ring->bankNumber);
        nearly_empty_stat = READ_CSR_NE_STAT(ring->bankNumber);

        head = READ_CSR_RING_HEAD(ring->bankNumber,
                                  ring->ringNumber);
        tail = READ_CSR_RING_TAIL(ring->bankNumber,
                                  ring->ringNumber);

        if(head == tail)
        {
            /* Need to reread empty status */
            empty_stat = READ_CSR_E_STAT(ring->bankNumber);
            space = ((empty_stat & (1 << ring->ringNumber))) ?
                                                      ring->sizeInBytes : 0;
        }
        else
        {
            space = (tail > head) ? (ring->sizeInBytes - (tail - head)) :
               (ring->sizeInBytes - (ring->sizeInBytes - (head - tail)));
        }

        empty_stat = (empty_stat & (1<<ring->ringNumber)) ? 1 : 0;
        nearly_empty_stat = (nearly_empty_stat & (1<<ring->ringNumber)) ?
                                                                      1 : 0;
        len = snprintf(data+len, size-len,
                           "------- Ring Configuration -------\n");
        len += snprintf(data+len, size-len, " Service Name:\t%s\n",
                                                  ring->serviceName);
        len += snprintf(data+len, size-len,
                   " Accelerator Number:\t%d, Bank Number:\t %d,"
                   " Ring Number:\t %d\n",
                    ring->accelNumber,
                    ring->bankNumber,
                    ring->ringNumber
                    );
        len += snprintf(data+len, size-len," Ring Config: %x, Base Address: %p,"
                    " Head: %x, Tail: %x, Space: %x\n",
                    ring->ringConfig,
                    ring->ringBaseAddress,
                    head, tail, space);

        len += snprintf(data+len, size-len,
                " Ring Empty flag: %d, Ring Nearly Empty flag: %d\n",
                    empty_stat, nearly_empty_stat);

        if(ring->userSpaceRing)
        {
            len += snprintf(data+len, size-len, " This is a userspace Ring\n");
            if(ring->orphanRing)
            {
                len += snprintf(data+len, size-len, " The userspace Ring is an"
                                    " orphan ring\n");
            }
        }

        len += snprintf(data+len, size-len,
                              "----------- Ring Data -----------\n");
    }
    ptr = ring->ringBaseAddress + (offset / sizeof(Cpa32U));
    for(i = offset; i < ring->sizeInBytes; )
    {
        if(len + ADF_DEBUG_LINE_LEN > size)
        {
            return offset;
        }
        len += snprintf(data+len, size-len, "%p:", ptr);
        for(x = 0 ; x < sizeof(Cpa32U) && i< ring->sizeInBytes ;
                                                x++, i+=sizeof(Cpa32U))
        {
            len += snprintf(data+len, size-len, " %08X", *ptr);
            ptr++;
            offset+=sizeof(Cpa32U);
        }
        len += snprintf(data+len, size-len, "\n");
    }
    return 0;
}

/*
 * adf_init_ETRings_proc_debug
 * Create ring debug directory if enabled.
 */
CpaStatus adf_init_ETRings_proc_debug(icp_accel_dev_t *accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    Cpa8S val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32S enabled = 0;

    stat = icp_adf_cfgGetParamValue(accel_dev,
                                      GENERAL_SEC,
                                      ADF_PROC_DEBUG,
                                      (char*)val);
    if (CPA_STATUS_SUCCESS != stat)
    {
        /*
         * There is no debug entry in the config file
         * so don't need to do anything just return success
         */
        return CPA_STATUS_SUCCESS;
    }
    /* check the parent proc dir */
    if(NULL == accel_dev->base_dir)
    {
        return stat;
    }

    enabled = (Cpa32S)ICP_STRTOUL((char*)val, NULL, ADF_CFG_BASE_DEC);

    if(enabled)
    {
        icp_etr_priv_data_t *priv_data = (icp_etr_priv_data_t*)
                                                  accel_dev->pCommsHandle;
        debug_dir_info_t *dir = ICP_ZALLOC_GEN( sizeof(debug_dir_info_t));
        if(NULL == dir)
        {
            ADF_ERROR("Failed allocating memory for debug\n");
            return CPA_STATUS_RESOURCE;
        }

        dir->parent = accel_dev->base_dir;
        dir->name = ADF_DEBUG_ENTRY;

        if( CPA_STATUS_SUCCESS != icp_adf_debugAddDir(accel_dev, dir))
        {
            ADF_ERROR("Failed creating ETRing debug entry\n");
            ICP_FREE(dir);
            return CPA_STATUS_FAIL;
        }
        else
        {
            priv_data->debug = (void*)dir;
        }
    }
    return stat;
}

/*
 * adf_clean_ETRings_proc_debug
 * Clean ring debug directory.
 */
void adf_clean_ETRings_proc_debug(icp_accel_dev_t *accel_dev)
{
    icp_etr_priv_data_t *priv_data = (icp_etr_priv_data_t*)
                                                  accel_dev->pCommsHandle;

    if(priv_data->debug)
    {
        debug_dir_info_t *dir = (debug_dir_info_t*)priv_data->debug;
        icp_adf_debugRemoveDir(dir);
        ICP_FREE(dir);
        priv_data->debug = NULL;
    }
}

/*
 * adf_debug_transport_create_dir
 * Create debug dir and debug config file for bank
 */
CpaStatus adf_debug_transport_create_dir(icp_accel_dev_t *accel_dev,
                                 icp_et_ring_bank_data_t *bank_data)
{
    icp_etr_priv_data_t *priv_data = (icp_etr_priv_data_t*)
                                                  accel_dev->pCommsHandle;
    debug_private_t *dbg_private = NULL;

    if(priv_data && priv_data->debug)
    {
        debug_dir_info_t *dir = NULL;
        debug_file_info_t *file = NULL;

        dir = ICP_MALLOC_GEN(sizeof(debug_dir_info_t));
        if(NULL == dir)
        {
            ADF_ERROR("Failed to allocate memory for debug dir\n");
            return CPA_STATUS_FAIL;
        }
        dir->parent = priv_data->debug;
        dir->name = ICP_MALLOC_GEN(ADF_DEBUG_DIR_NAME_LEN);
        if(NULL == dir->name)
        {
            ADF_ERROR("Failed to allocate memory for debug dir name\n");
            ICP_FREE(dir);
            return CPA_STATUS_FAIL;
        }
        sprintf(dir->name, ADF_DEBUG_BANK_DIR, bank_data->bankNumber);
        if(CPA_STATUS_SUCCESS != icp_adf_debugAddDir(accel_dev, dir))
        {
            ADF_ERROR("Failed to add debug directory\n");
            ICP_FREE(dir->name);
            ICP_FREE(dir);
            return CPA_STATUS_FAIL;
        }
        file = ICP_MALLOC_GEN(sizeof(debug_file_info_t));
        if(NULL == file)
        {
            ADF_ERROR("Failed to allocate memory for debug file\n");
            icp_adf_debugRemoveDir(dir);
            ICP_FREE(dir->name);
            ICP_FREE(dir);
            return CPA_STATUS_FAIL;
        }
        file->name = ADF_DEBUG_CONF;
        file->parent = dir;
        file->seq_read = adf_debug_print_bank_rings;

        dbg_private = ICP_MALLOC_GEN(sizeof (debug_private_t));
        if (NULL == dbg_private){
            ADF_ERROR("Failed to allocate memory for debug file\n");
            icp_adf_debugRemoveDir(dir);
            ICP_FREE(file);
            ICP_FREE(dir->name);
            ICP_FREE(dir);
            return CPA_STATUS_FAIL;
        }

        dbg_private->accel_dev  = accel_dev;
        dbg_private->bankNumber = bank_data->bankNumber;
        file->private_data      = dbg_private;
        if(CPA_STATUS_SUCCESS != icp_adf_debugAddFile(accel_dev, file))
        {
            ADF_ERROR("Failed to add debug file\n");
            icp_adf_debugRemoveDir(dir);
            ICP_FREE(dir->name);
            ICP_FREE(dir);
            ICP_FREE(file);
            ICP_FREE(dbg_private);
            return CPA_STATUS_FAIL;
        }
        bank_data->debug_rings_dir = dir;
        bank_data->debug_bank_conf = file;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_debug_transport_remove_dir
 * Remove debug directory and return resources
 */
void adf_debug_transport_remove_dir(icp_accel_dev_t *accel_dev,
                                 icp_et_ring_bank_data_t* bank_data)
{
    icp_etr_priv_data_t *priv_data = (icp_etr_priv_data_t*)
                                                  accel_dev->pCommsHandle;
    if(priv_data && priv_data->debug && bank_data &&
            bank_data->debug_rings_dir )
    {
        debug_dir_info_t *dir = bank_data->debug_rings_dir;
        debug_file_info_t *file = bank_data->debug_bank_conf;
        icp_adf_debugRemoveFile(file);
        icp_adf_debugRemoveDir(dir);
        ICP_FREE(dir->name);
        ICP_FREE(dir);
        ICP_FREE(file->private_data);
        ICP_FREE(file);
        bank_data->debug_rings_dir = NULL;
        bank_data->debug_bank_conf = NULL;
    }
}

/*
 * adf_debug_create_ring
 * Create a debug file for ring
 */
CpaStatus adf_debug_create_ring(icp_accel_dev_t *accel_dev,
                                icp_et_ring_bank_data_t *bank_data,
                                icp_et_ring_data_t *ring_data)
{
    icp_etr_priv_data_t *priv_data = (icp_etr_priv_data_t*)
                                                  accel_dev->pCommsHandle;

    if(priv_data && priv_data->debug && bank_data->debug_rings_dir)
    {
        debug_dir_info_t *dir = bank_data->debug_rings_dir;
        debug_file_info_t *file = ICP_MALLOC_GEN(sizeof(debug_file_info_t));
        if(NULL == file)
        {
            ADF_ERROR("Failed to allocate memory for ring debug file\n");
            return CPA_STATUS_FAIL;
        }
        file->name = ICP_MALLOC_GEN(ADF_DEBUG_DIR_NAME_LEN);
        if(NULL == file->name)
        {
            ADF_ERROR("Failed to allocate memory for ring debug file->name\n");
            ICP_FREE(file);
            return CPA_STATUS_FAIL;
        }
        sprintf(file->name, ADF_DEBUG_RING, ring_data->ringNumber);
        file->parent = dir;
        file->seq_read = adf_debug_print_ring_data;
        file->private_data = ring_data;
        if(CPA_STATUS_SUCCESS != icp_adf_debugAddFile(accel_dev, file))
        {
            ADF_ERROR("Failed to create debug file for ring\n");
            ICP_FREE(file->name);
            ICP_FREE(file);
            return CPA_STATUS_FAIL;
        }
        ring_data->debug_conf = file;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_debug_remove_ring
 * Remove debug file for ring
 */
void adf_debug_remove_ring(icp_et_ring_data_t *ring_data)
{
    debug_file_info_t *file = ring_data->debug_conf;
    if(file && file->name)
    {
        icp_adf_debugRemoveFile(file);
        ICP_FREE(file->name);
        ICP_FREE(file);
        ring_data->debug_conf = NULL;
    }
}


