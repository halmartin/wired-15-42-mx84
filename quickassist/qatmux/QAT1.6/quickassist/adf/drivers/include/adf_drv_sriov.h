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
 *  version: QAT1.6.L.2.6.0-65
 *
 *****************************************************************************/

/*****************************************************************************
 * @file adf_drv_sriov.h
 *
 * @description
 *      This header file contains the macros and functions for VF to PF
 *      communication.
 *
 *****************************************************************************/
#ifndef ADF_DRV_SRIOV_H
#define ADF_DRV_SRIOV_H

#include "icp_adf_init.h"

/*
 * VF2PF & PF2VF communication
 * There is one 32bit register for each VF for two directional VF2PF
 * communication.
 * PF has access to all 16 registers but each VF sees just its one.
 * The register funnctionally is split into two parts.
 * The bottom half is for PF to VF communication. In particular when
 * the first bit of this register (bit 0) gets set an IRQ will be triggered
 * in VF.
 * The top half is for VF to PF communication. In particular when
 * the first bit of this half of register (bit 16) gets set an IRQ
 * will be triggered in PF. PF then needs to read
 * Error Source Mask Register 3 to find out what VF triggered the IRQ.
 * The other bits within this register don't have any other function
 * and can be used to encode messages.
 *
 *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *                                            ^   ^
 *                               Msg Origin---|   |
 *                                              VF2PF IRQ
 *
 *  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *                                             ^  ^
 *                                Msg Origin---|  |
 *                                               PF2VF IRQ
 *
 * Currently the communication is implemented as follows.
 * 
 * Msg Origin
 * 
 * System = 1 , user message = 0
 * The VF or PF when it receives a message needs to know whether the message
 * sent was from the system (kernel) or came from the user space API called
 * by a user context. messages in the repective bottom halves are handled 
 * differently depending on this bit
 *
 * SYSTEM MESSAGES
 * ============================================================================
 * ==> VF->PF:
 * 1. Version Request 
 * 
 *  *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *    _______________________________________________
 *    |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *    -----------------------------------------------
 *            \_______/   \_______/   \_______/  ^   ^
 *                ^           ^           ^      |   |
 *                |           |           |      |    VF2PF IRQ
 *                |           |           |      |
 *                |      Minor version    |      Msg Origin 
 *                |                       |
 *         Major version            System message Type
 * 
 * 2. VF init, Type = System
 *
 * Notify PF that VF is initialized.
 *
 * 3. VF shutdown, Type = System
 *
 * Notify PF that VF is shutdown.
 *
 * ==> PF->VF:
 * 
 * 1. version request response
 * 
 *     15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *    _______________________________________________
 *    |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *    -----------------------------------------------
 *      \_/   \_______/   \_______/   \_______/  ^   ^
 *       ^        ^           ^           ^      |   |
 *       |        |           |           |      |    PF2VF IRQ
 *  Supported     |           |           |      |
 *                |      Minor version    |      Msg Origin 
 *                |                       |
 *         Major version            System message Type
 * 
 * support = 1 - PF and VF compatible
 *         = 0 - Not supported 
 *         = 2 - Unsure let VF descide
 * 
 * 2. notify restarting
 * 
 * When device hangs or uncorrectable error detected,
 * PF driver will send the restarting event to all mapped VFs
 * before PF reset, and trigger an IRQ by setting bit1, and start polling on
 * the IRQ bit for ACK or untill timeout expires. 
 * And then, PF driver will continue resetting
 * and restarting, guests will die, we restart guests.
 *
 * To encode the message the lower 15 bits are split into fields.
 * One field is used encode type of the message.
 * The type of the message in encoded in three most significant bits of the
 * register.
 * Restarting event is encoded in the type bits (Type = 1) and
 * not other bits are used.
 *
 *  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *                                 \_______/  ^   ^
 *                                     ^      |   |
 *                                     |      |   PF2VF IRQ
 *                                     |      |
 *                                     |      Msg Origin 
 *                                     |
 *                             System message Type
 *
 *  VF driver responds by setting bit17 to 1 with response message type=7.
 *  In this way, PF knows that the message has been received and processed
 *  successfully, and VF has distribute the restarting event to all apps,
 *  and apps are ready to be restarted.
 *  In case of error VF writes 0xFFFC value the the register.
 *
 *
 * USER MESSAGES
 * ============================================================================
 * 
 * Using userspace API's it is possible for a user context to send a message
 * over PF2VF comms. both from the PF and the VF - the basic protocal is below
 * 
 * 
 *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *   \_____________________________________/  ^   ^
 *                      ^                     |   |
 *                      |                     |   VF2PF IRQ
 *                   message                  Msg Origin 
 * 
 *  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *   \_____________________________________/  ^   ^
 *                      ^                     |   |
 *                      |                     |   PF2VF IRQ
 *                   message                   Msg Origin 
 *                                     
 * Bits 2 - 15 and 18 - 31 are for the message.
 * The Msg origin will be 0 for user.                        
 * 
 * 
 * The below functions are used to set and get the fields.
 */
/* IRQ helper fns */
static inline void adf_vf2pf_set_irq(Cpa32U *csr_value)
{
        *csr_value |= ICP_VF2PF_MSGMASK_IRQ;
}

static inline void adf_pf2vf_ack(Cpa32U *csr_value)
{
        /* To ack, clear the irq bit set by the other side */
        *csr_value = *csr_value &~ ICP_VF2PF_MSGMASK_IRQ;
}

static inline void adf_pf2vf_set_irq(Cpa32U *csr_value)
{
        *csr_value |= ICP_PF2VF_MSGMASK_IRQ;
}

static inline void adf_vf2pf_ack(Cpa32U *csr_value)
{
        /* To ack, clear the irq bit set by the other side */
        *csr_value = *csr_value &~ ICP_PF2VF_MSGMASK_IRQ;
}

/* MSGORIGIN helper fns */
static inline void adf_vf2pf_set_msg_origin(Cpa32U *csr_value, Cpa32U origin)
{
        /* clear field first */
        *csr_value = *csr_value & (~(ICP_PFVF_MSGORIGIN_MASK <<
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGORIGIN_SHIFT)));
        /* Now mask off only the bits we want from origin, then set field
         * to this. If origin is 0 this has no effect.*/
        *csr_value |= ((origin & ICP_PFVF_MSGORIGIN_MASK)
                 <<(ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGORIGIN_SHIFT));
}
static inline void adf_pf2vf_set_msg_origin(Cpa32U *csr_value, Cpa32U origin)
{
        /* clear field first */
        *csr_value = *csr_value &
                (~(ICP_PFVF_MSGORIGIN_MASK << ICP_PFVF_MSGORIGIN_SHIFT));
        /* Now mask off only the bits we want from origin, then set field
         * to this. If origin is 0 this has no effect.*/
        *csr_value |= ((origin & ICP_PFVF_MSGORIGIN_MASK)
                << ICP_PFVF_MSGORIGIN_SHIFT);
}
static inline Cpa32U adf_vf2pf_get_msg_origin(Cpa32U csr_value)
{
        return ((csr_value >>
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGORIGIN_SHIFT))
                & ICP_PFVF_MSGORIGIN_MASK);
}
static inline Cpa32U adf_pf2vf_get_msg_origin(Cpa32U csr_value)
{
        return ((csr_value >> ICP_PFVF_MSGORIGIN_SHIFT)
                & ICP_PFVF_MSGORIGIN_MASK);
}

/* MSGCONTENT helper fns */
static inline void adf_vf2pf_set_msg_content(Cpa32U *csr_value, Cpa32U content)
{
        /* clear field first */
        *csr_value = *csr_value & (~(ICP_PFVF_MSGCONTENT_MASK <<
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGCONTENT_SHIFT)));
        /* Now mask off only the bits we want from content, then set field
         * to this. If content is 0 this has no effect.*/
        *csr_value |= ((content & ICP_PFVF_MSGCONTENT_MASK)
                 <<(ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGCONTENT_SHIFT));
}
static inline void adf_pf2vf_set_msg_content(Cpa32U *csr_value, Cpa32U content)
{
        /* clear field first */
        *csr_value = *csr_value &
                (~(ICP_PFVF_MSGCONTENT_MASK << ICP_PFVF_MSGCONTENT_SHIFT));
        /* Now mask off only the bits we want from content, then set field
         * to this. If content is 0 this has no effect.*/
        *csr_value |= ((content & ICP_PFVF_MSGCONTENT_MASK)
                << ICP_PFVF_MSGCONTENT_SHIFT);
}
static inline Cpa32U adf_vf2pf_get_msg_content(Cpa32U csr_value)
{
        return ((csr_value >>
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGCONTENT_SHIFT))
                & ICP_PFVF_MSGCONTENT_MASK);
}
static inline Cpa32U adf_pf2vf_get_msg_content(Cpa32U csr_value)
{
        return ((csr_value >> ICP_PFVF_MSGCONTENT_SHIFT)
                & ICP_PFVF_MSGCONTENT_MASK);
}


/* MSGTYPE helper fns */
static inline void adf_vf2pf_set_msg_type(Cpa32U *csr_value, Cpa32U type)
{
        /* clear field first */
        *csr_value = *csr_value & (~(ICP_PFVF_MSGTYPE_MASK <<
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGTYPE_SHIFT)));
        /* Now mask off only the bits we want from type, then set field
         * to this. If type is 0 this has no effect.*/
        *csr_value |= ((type & ICP_PFVF_MSGTYPE_MASK)
                 <<(ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGTYPE_SHIFT));
}
static inline void adf_pf2vf_set_msg_type(Cpa32U *csr_value, Cpa32U type)
{
        /* clear field first */
        *csr_value = *csr_value &
                (~(ICP_PFVF_MSGTYPE_MASK << ICP_PFVF_MSGTYPE_SHIFT));
        /* Now mask off only the bits we want from type, then set field
         * to this. If type is 0 this has no effect.*/
        *csr_value |= ((type & ICP_PFVF_MSGTYPE_MASK)
                << ICP_PFVF_MSGTYPE_SHIFT);
}
static inline Cpa32U adf_vf2pf_get_msg_type(Cpa32U csr_value)
{
        return ((csr_value >>
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGTYPE_SHIFT))
                & ICP_PFVF_MSGTYPE_MASK);
}
static inline Cpa32U adf_pf2vf_get_msg_type(Cpa32U csr_value)
{
        return ((csr_value >> ICP_PFVF_MSGTYPE_SHIFT)
                & ICP_PFVF_MSGTYPE_MASK);
}

/* VERSION helper fns */
static inline void adf_vf2pf_set_version(Cpa32U *csr_value,
                                            Cpa32U major, Cpa32U minor)
{
        /* clear fields first */
        *csr_value = *csr_value & (~(ICP_PFVF_MAJORVERSION_MASK <<
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MAJORVERSION_SHIFT)));
        *csr_value = *csr_value & (~(ICP_PFVF_MINORVERSION_MASK <<
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MINORVERSION_SHIFT)));
        /* Now mask off only the bits we want from major and minor
         * then set fields to these. */
        *csr_value |= ((major & ICP_PFVF_MAJORVERSION_MASK)
                 <<(ICP_VF2PF_START_SHIFT + ICP_PFVF_MAJORVERSION_SHIFT));
        *csr_value |= ((minor & ICP_PFVF_MINORVERSION_MASK)
                 <<(ICP_VF2PF_START_SHIFT + ICP_PFVF_MINORVERSION_SHIFT));
}
static inline void adf_pf2vf_set_version(Cpa32U *csr_value,
                                        Cpa32U major, Cpa32U minor)
{
        /* clear field first */
        *csr_value = *csr_value &
                (~(ICP_PFVF_MAJORVERSION_MASK << ICP_PFVF_MAJORVERSION_SHIFT));
        *csr_value = *csr_value &
                (~(ICP_PFVF_MINORVERSION_MASK << ICP_PFVF_MINORVERSION_SHIFT));
        /* Now mask off only the bits we want from major and minor
         * then set fields to these. */
        *csr_value |= ((major & ICP_PFVF_MAJORVERSION_MASK)
                << ICP_PFVF_MAJORVERSION_SHIFT);
        *csr_value |= ((minor & ICP_PFVF_MINORVERSION_MASK)
                     << ICP_PFVF_MINORVERSION_SHIFT);
}
static inline Cpa32U adf_vf2pf_get_major_version(Cpa32U csr_value)
{
        return ((csr_value >>
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MAJORVERSION_SHIFT))
                & ICP_PFVF_MAJORVERSION_MASK);
}
static inline Cpa32U adf_pf2vf_get_major_version(Cpa32U csr_value)
{
        return ((csr_value >> ICP_PFVF_MAJORVERSION_SHIFT)
                & ICP_PFVF_MAJORVERSION_MASK);
}
static inline Cpa32U adf_vf2pf_get_minor_version(Cpa32U csr_value)
{
        return ((csr_value >>
                (ICP_VF2PF_START_SHIFT + ICP_PFVF_MINORVERSION_SHIFT))
                & ICP_PFVF_MINORVERSION_MASK);
}
static inline Cpa32U adf_pf2vf_get_minor_version(Cpa32U csr_value)
{
        return ((csr_value >> ICP_PFVF_MINORVERSION_SHIFT)
                & ICP_PFVF_MINORVERSION_MASK);
}

/* VF_SUPPORTED helper fns */
static inline void adf_pf2vf_set_vf_supported(Cpa32U *csr_value)
{
        /* clear field first */
        *csr_value = *csr_value &
                (~(ICP_PFVF_VF_SUPPORTED_MASK << ICP_PFVF_VF_SUPPORTED_SHIFT));
        /* Now set field */
        *csr_value |= (ICP_PFVF_VF_SUPPORTED<<ICP_PFVF_VF_SUPPORTED_SHIFT);
}
static inline void adf_pf2vf_set_vf_notSupported(Cpa32U *csr_value)
{
        /* clear field first */
        *csr_value = *csr_value &
                (~(ICP_PFVF_VF_SUPPORTED_MASK << ICP_PFVF_VF_SUPPORTED_SHIFT));
        /* Now set field */
        *csr_value |= (ICP_PFVF_VF_NOTSUPPORTED<<ICP_PFVF_VF_SUPPORTED_SHIFT);
}
static inline void adf_pf2vf_set_vf_unknown(Cpa32U *csr_value)
{
        /* clear field first */
        *csr_value = *csr_value &
                (~(ICP_PFVF_VF_SUPPORTED_MASK << ICP_PFVF_VF_SUPPORTED_SHIFT));
        /* Now set field */
        *csr_value |= (ICP_PFVF_VF_SUPPORTUNKNOWN<<ICP_PFVF_VF_SUPPORTED_SHIFT);
}

static inline Cpa32U adf_pf2vf_get_version_supported(Cpa32U csr_value)
{
        return ((csr_value >> ICP_PFVF_VF_SUPPORTED_SHIFT)
                & ICP_PFVF_VF_SUPPORTED_MASK);
}

static inline CpaBoolean adf_pfvfcomms_isCSRInUseByVF(Cpa32U reg)
{
        Cpa32U val =0;

        val = reg & ICP_IN_USE_BY_VF_MASK;

        if (val == ICP_IN_USE_BY_VF)
                return CPA_TRUE;
        else
                return CPA_FALSE;
}

static inline CpaBoolean adf_pfvfcomms_isCSRInUseByPF(Cpa32U reg)
{
        Cpa32U val = 0;

        val = reg & ICP_IN_USE_BY_PF_MASK;

        if (val == ICP_IN_USE_BY_PF)
                return CPA_TRUE;
        else
                return CPA_FALSE;
}

static inline void adf_pfvfcomms_setVFInUse(Cpa32U *msg)
{
        *msg &= ~ICP_IN_USE_BY_VF_MASK;
        *msg |= ICP_IN_USE_BY_VF;
}

static inline void adf_pfvfcomms_setPFInUse(Cpa32U *msg)
{
        *msg &= ~ICP_IN_USE_BY_PF_MASK;
        *msg |= ICP_IN_USE_BY_PF;
}

static inline void adf_pfvfcomms_clearVFInUse(Cpa32U *msg)
{
        *msg &= (~ICP_IN_USE_BY_VF_MASK);
}

static inline void adf_pfvfcomms_clearPFInUse(Cpa32U *msg)
{
        *msg &= (~ICP_IN_USE_BY_PF_MASK);
}


static inline Cpa16U adf_vf2pf_get_userSpace_msg(Cpa32U msg)
{
        return (msg >> (ICP_VF2PF_START_SHIFT + ICP_PFVF_MSGCONTENT_SHIFT));
}

static inline Cpa16U adf_pf2vf_get_userSpace_msg(Cpa32U msg)
{
        return ((msg & ICP_PF2VF_CSR_MASK) >> ICP_PFVF_MSGCONTENT_SHIFT);
}

/*
 * Notify PF about restarting event
 */
CpaStatus adf_PF2VF_notify_restarting(icp_accel_dev_t *accel_dev);

/*
 * Notify PF about VF init
 */
CpaStatus adf_VF2PF_init(icp_accel_dev_t *accel_dev);

/*
 * Notify PF about VF shutdown
 */
CpaStatus adf_VF2PF_shutdown(icp_accel_dev_t *accel_dev);

/*
 * requests the PF version per device 
 */
CpaStatus adf_VF2PF_RequestVersion(icp_accel_dev_t *accel_dev);

/*
 * sets the PF version per device 
 */
CpaStatus adf_VFPF_setVersion(Cpa32U accelId, Cpa32U major , Cpa32U minor, 
                              Cpa32U supported );

/*
 * adf_vfbh_handler
 * VF2PF IRQ bottom half handler
 */
void adf_vf_bh_handler(void *handle);
#endif
