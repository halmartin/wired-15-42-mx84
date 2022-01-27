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
 * @file adf_ETring_mgr.h
 *
 * @description
 *      File contains declaration of ring manipulation routines and structures
 *
 *****************************************************************************/
#ifndef ADF_ETRING_MGR_H
#define ADF_ETRING_MGR_H

#include "icp_platform.h"
#include "adf_platform.h"

/* Structure describing Eagletail Ring */
typedef struct icp_et_ring_data_s
{
    Cpa64U     dma_addr;
    Cpa32U*    ringBaseAddress;
    Cpa32U*    ringCSRAddress;
    Cpa32U     msgSizeInBytes;
    Cpa32U     ringConfig;
    Cpa64U     ringBase; /* for use with Wireless or dyn */
    Cpa32U     sizeInBytes;
    Cpa32U     modulo;
    Cpa32U     head;
    Cpa32U     tail;
    Cpa32U     ringNumber;
    Cpa32U     bankNumber;
    Cpa32U     accelNumber;
    Cpa32U     ringmaskId;
    Cpa32U     flags;
    Cpa32U     orphanResponseCount;
    Cpa32U     ringResponseQuota;
    Cpa32U     inFlightIndex;   /* inflight counter */
    Cpa32U     maxRequestsInflight;
    Cpa32U     minRespsPerHeadWrite;
    icp_trans_callback callbackFn; /* Callback function */
    icp_trans_handle*  parent;
    void*      userSpacePrivateData;
    char*      serviceName;
    void*      ETR_ring_tasklet_poll;
    icp_adf_ringInfoService_t info;
    ICP_SPINLOCK       ringLock;   /* Lock per ring */
    CpaBoolean userSpaceRing;
    CpaBoolean orphanRing;
    OsalAtomic pollingTaskletScheduled;
    void* debug_conf;
} icp_et_ring_data_t;

/* Structure describing Eagletail Rings Bank */
typedef struct icp_et_ring_bank_data_s
{
    icp_et_ring_data_t rings[ICP_ETR_MAX_RINGS_PER_BANK];
    Cpa32U allocatedRings;
    Cpa32U interruptMask;
    Cpa32U interruptUserMask;
    Cpa32U interruptPendingUserMask;
    Cpa32U pollingMask;
    Cpa32U coalIntrDelayCount;
    /* Bitfields for different coalescing
     * methods on/off settings */
    Cpa32U timedCoalescEnabled : 1;
    Cpa32U numberMsgCoalescEnabled : 1;
    Cpa32U bankNumber;
    Cpa32U irq_src_val_hi;
    Cpa32U irq_src_val_low;
    Cpa32U bankResponseQuota;
    Cpa32U ringsFullMask;
    /* Mask to keep track of pre-created, run-time assigned, rings
     * can only be set to used if bit in 'allocatedRings' is set */
    Cpa32U ringsInUseMask;
    ICP_SPINLOCK bankLock;
    OsalAtomic pollingTaskletScheduled;
    void *ETR_bank_tasklet;
    void *ETR_bank_tasklet_poll;
    void *debug_rings_dir;
    void *debug_bank_conf;
} icp_et_ring_bank_data_t;

/* Autopush bank data structure */
typedef struct icp_et_ring_ap_bank_data_s
{
    Cpa32U apNFMask;
    Cpa32U apNFDest;
    Cpa32U apNEMask;
    Cpa32U apNEDest;
} icp_et_ring_ap_bank_data_t;

/* Private Data structure for the rings. */
typedef struct icp_etr_priv_data_s
{
    /* ET Ring private data is organized on a per-bank basis. */
    icp_et_ring_bank_data_t *banks;
    /* Autopush bank */
    icp_et_ring_ap_bank_data_t *apBanks;
    /* Autopush delay */
    Cpa32U apDelay;
    /* Base address of the CSRs for this ring controller device */
    Cpa64U csrBaseAddress;
    /*ETRing debug entry*/
    void* debug;
}icp_etr_priv_data_t;

/******************************************************************************
 * @description
 * Initialize and register the Eagletail Rings Manager in the Transport Manager
 *****************************************************************************/
CpaStatus adf_init_ETManager(void);

/******************************************************************************
 * @description
 * Unregister the Eagletail Rings manager.
 *****************************************************************************/
CpaStatus adf_shutdown_ETManager(void);

/******************************************************************************
 * @description
 * Initialise the Eagletail Rings manager data.
 *****************************************************************************/
CpaStatus adf_init_ETR_Data(icp_accel_dev_t *accelHandle);

/******************************************************************************
 * @description
 * Clean Eagletail Rings manager data.
 *****************************************************************************/
CpaStatus adf_cleanup_ETR_Data(icp_accel_dev_t *accelHandle);

/******************************************************************************
 * @description
 * Eagletail Ring transport bottom half handler.
 *****************************************************************************/
void adf_ringResponseHandler(void *private_data, Cpa32U bankNumber);

/******************************************************************************
 * @description
 * Find the Eagletail Ring handle associated with the transport ID
 *****************************************************************************/
CpaStatus adf_findETHandle(icp_accel_dev_t *accel_dev,
                           icp_trans_identifier trans_id,
                           icp_trans_handle** trans_handle);

/******************************************************************************
 * @description
 * Eagletail Ring transport handler function for the Ring Polling.
 *****************************************************************************/
void adf_ringResponsePolling(void *ringHandle);

/******************************************************************************
 * @description
 * Eagletail Ring transport handler function for the bank Polling. This is
 * done on a per bank basis.
 *****************************************************************************/
void adf_bankResponsePolling(icp_accel_dev_t *accelHandle, Cpa32U bankNumber);

/******************************************************************************
 * @description
 * Internal function which takes care of scheduling
 * the tasklet for polling and checking that the
 * bank is not already being polled.
 *
 * Returns:
 *
 * CPA_STATUS_SUCCESS on success
 * CPA_STATUS_RETRY   in the case where the bank is already being polled.
 *****************************************************************************/
CpaStatus adf_pollBank(icp_et_ring_bank_data_t *bank,
                       Cpa32U response_quota,
                       Cpa32U ringsFull);

/******************************************************************************
 * @description
 * Internal polling function which reads the messages and calls
 * the user callback
 *
 *****************************************************************************/

/******************************************************************************
 * @description
 * Initializes transport debug.
 *
 *****************************************************************************/
CpaStatus adf_init_ETRings_proc_debug(icp_accel_dev_t *accel_dev);

/******************************************************************************
 * @description
 * Clean ring debug directory.
 *
 *****************************************************************************/
void adf_clean_ETRings_proc_debug(icp_accel_dev_t *accel_dev);

/******************************************************************************
 * @description
 * Create proc dir entry for a bank
 *
 *****************************************************************************/
CpaStatus adf_debug_transport_create_dir(icp_accel_dev_t *accel_dev,
                                         icp_et_ring_bank_data_t* bank_data);
/******************************************************************************
 * @description
 * Clean proc dir entry for a bank
 *
 *****************************************************************************/
void adf_debug_transport_remove_dir(icp_accel_dev_t *accel_dev,
                                    icp_et_ring_bank_data_t* bank_data);
/******************************************************************************
 * @description
 * Create proc entry for a ring
 *
 *****************************************************************************/
CpaStatus adf_debug_create_ring(icp_accel_dev_t *accel_dev,
                                icp_et_ring_bank_data_t *bank_data,
                                icp_et_ring_data_t *ring_data);
/******************************************************************************
 * @description
 * Clean proc entry for a ring
 *
 *****************************************************************************/
void adf_debug_remove_ring(icp_et_ring_data_t *ring_data);

/******************************************************************************
 * @description
 *
 * Returns true if the ring is a polled ring
 *****************************************************************************/
CpaBoolean adf_is_ring_polled(icp_et_ring_data_t *ring_data);

/******************************************************************************
 * @description
 *
 * Sets InUse flag for this ring so dyn or wireless inst can use it.
 * Ring must be allocated already.
 *
 *****************************************************************************/
CpaStatus adf_setRingInUse(icp_et_ring_bank_data_t* bank_data,
                                                    Cpa32U ring_number);

/*
 * adf_process_msgs
 *
 * Description
 * Process the Message, trigger the associated callback, update the 
 * message counter and update head
 */
static inline void adf_process_msg(icp_trans_handle *trans_handle,
        volatile Cpa32U **msg, Cpa32U *msg_counter, OsalAtomic *pRingInflight)
{
    icp_et_ring_data_t *ringData = (icp_et_ring_data_t *)
        trans_handle->trans_data;

    osalAtomicDec(pRingInflight);
    /* Invoke the callback for the message */
    ringData->callbackFn(trans_handle, (Cpa32U*)*msg);

    /* Mark the message as processed */
    **msg = EMPTY_RING_ENTRY_SIG;

    /* Advance the head offset and handle wraparound */
    ringData->head =
        modulo((ringData->head + ICP_ET_DEFAULT_MSG_SIZE),
                ringData->modulo);
    (*msg_counter)++;

    /* Point to where the next message should be */
    *msg = (Cpa32U*)
        (((UARCH_INT)ringData->ringBaseAddress) + ringData->head);
}
#endif /* ADF_ETRING_MGR_H */
