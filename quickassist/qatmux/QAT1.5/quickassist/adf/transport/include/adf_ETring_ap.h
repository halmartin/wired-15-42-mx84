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
 * @file adf_ETring_ap.h
 *
 * @description
 *      This header file that contains the prototypes and definitions required
 *      for ETR Autopush configuration.
 *
 *****************************************************************************/
#ifndef ADF_ETRING_AP_H
#define ADF_ETRING_AP_H

/* Autopush maximum rings per bank */
#define ETR_MAX_RINGS_PER_AP_BANK       32

/* Maximum mailbox per acclerator */
#define ETR_MAX_MAILBOX_PER_ACCELERATOR 4

/* Maximum AEs per mailbox */
#define MAX_AE_PER_MAILBOX              4

/* Maximum AEs per accelerator */
#define MAX_AE_PER_ACCELERATOR          (MAX_ACCELENGINES/MAX_ACCELERATORS)

/* Macro to get the ring's autopush bank number */
#define RING_AP_BANK_NUMBER(ring)       (ring >> 5)

/* Macro to get the ring's autopush bank number */
#define RING_AP_BUNDLE_NUMBER(ring)       (ring >> 4)

/* Macro to get the ring's accelerator number */
#define RING_AP_ACCEL_NUMBER(ring)      (ring >> 7)

/* Macro to get the ring's autopush mailbox number */
#define RING_AP_MAILBOX_NUMBER(ring)    \
    (RING_AP_BANK_NUMBER(ring) % ETR_MAX_MAILBOX_PER_ACCELERATOR)

/* Macro to get the ring number in the autopush bank */
#define RING_NUMBER_IN_AP_BANK(ring)    (ring % ETR_MAX_RINGS_PER_AP_BANK)

/* Autopush destination AE bit offset */
#define AP_DEST_AE_OFFSET               2

/* Autopush destination enable bit offset */
#define AP_DEST_ENABLE_OFFSET           7

/******************************************************************************
 * @description
 * Sets the specified ring settings in the auto-push registers.
 *****************************************************************************/
CpaStatus adf_etrApSetRingRegister(icp_accel_dev_t *pAccelDev,
                                    Cpa32U ringNumInDev,
                                    Cpa32U flags);

/******************************************************************************
 * @description
 * Remove specified ring settings from the auto-push registers.
 *****************************************************************************/
CpaStatus adf_etrApClrRingRegister(icp_accel_dev_t *pAccelDev,
                                    Cpa32U ringNumInDev,
                                    Cpa32U flags);

/******************************************************************************
 * @description
 * Clears the auto-push bank and registers.
 *****************************************************************************/
CpaStatus adf_etrApClrBanks(icp_accel_dev_t *pAccelDev, CpaBoolean clrReg);

#endif /* ADF_ETRING_AP_H */
