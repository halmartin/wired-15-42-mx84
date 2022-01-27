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
 * @file adf_drv_sriov.h
 *
 * @description
 *      This header file contains the macros and functions for VF to PF
 *      communication.
 *
 *****************************************************************************/
#ifndef ADF_DRV_SRIOV_H
#define ADF_DRV_SRIOV_H

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
 *                                               ^
 *                                               |
 *                                          VF2PF IRQ
 *
 *  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *                                               ^
 *                                               |
 *                                          PF2VF IRQ
 *
 * Currently the communication is implemented as follows.
 *
 * ==> VF->PF:
 * VF driver when needs a service from PF it encodes message in the
 * top half of the register, triggers an IRQ by setting 16 bit, and start
 * polling on the bottom half of the register waiting for PF response
 * until it gets the response or timeout expires.
 *
 * To encode the message the upper 15 bits are split into fields.
 * One field is used encode type of the message.
 * The type of the message in encoded in three most significant bits of the
 * register.
 *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *  \______/                                     ^
 *     ^                                         |
 *     |                                     VF2PF IRQ
 *  Type of the Request Message
 *
 *
 *  PF responds by setting the second bit to 1 with response message type=7.
 *  In this way, VF knows that the message has been received and processed
 *  successfully. When additional info needs to be send back to VF as a result
 *  of the request it is encoded in the remaining bits in the register.
 *  In case of error PF writes 0xFFFC value the the register.
 *
 *  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *  \______/                                  ^  ^
 *     ^                                      |  |
 *     |                                     Msg PF2VF IRQ
 *  Type of the Response Message = 0         Ack
 *
 * Currently there are three type of VF2PF request messages in use.
 * The three operation are as follows:
 *
 * 1. Ring operation, Type = 1
 *
 * Sent everytime a VF wants to create a ring. To create a ring
 * the following information needs to be sent
 * Ring Operation - add ring = 0 or remove ring = 1
 * Ring Number - number in 0-15 range
 * Ring Info - decodes the type of the ring so QAT knows
 *             what ring mask needs to be updated.
 * Flag - 0 for request ring, 1 for response ring.
 * This information is encoded as follows:
 *
 *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *  \______/       ^  \_________/ \_________/ ^
 *     ^           |       ^           ^      |
 *     |          flag     |           |     Operation
 *  Ring                  Ring        Ring   Add ring = 0
 *  Operation = 1         Type       Number  remove ring = 1
 *
 * 2. VF ring offset query, Type = 2
 *
 * When VF configures its response pipes id it doesnt know
 * what bank it is running on so it needs to take into
 * account the offset to calculate the global ring number
 * that is sent as response pipe_id field in each message.
 *
 * This information is encoded in the Ring operation bit and
 * not other bits are used.
 *
 * 3. ESRam address query, Type = 3
 *
 * This information is encoded in the Ring operation bit and
 * not other bits are used.
 *
 * 3. VF init, Type = 4
 *
 * Notify PF that VF is initialized.
 *
 * 4. VF shutdown, Type = 5
 *
 * Notify PF that VF is shutdown.
 *
 * ==> PF->VF:
 * When device hangs or uncorrectable error detected,
 * PF driver will send the restarting event to all mapped VFs
 * before PF reset, and trigger an IRQ by setting bit1, and start polling on
 * the top half the register waiting for VF response until it gets the response
 * or timeout expires. And then, PF driver will continue resetting
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
 *  \______/                                     ^
 *     ^                                         |
 *     |                                     PF2VF IRQ
 *  Type of the Request Message
 *
 *  VF driver responds by setting bit17 to 1 with response message type=7.
 *  In this way, PF knows that the message has been received and processed
 *  successfully, and VF has distribute the restarting event to all apps,
 *  and apps are ready to be restarted.
 *  In case of error VF writes 0xFFFC value the the register.
 *
 *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *  -----------------------------------------------
 *  \______/                                  ^  ^
 *     ^                                      |  |
 *     |                                     Msg VF2VP IRQ
 *  Type of the Response Message = 0         Ack
 *
 * The bellow functions are used to set and get the fields.
 */

static inline Cpa32U adf_vf2pf_get_ring_number(Cpa32U reg)
{
    Cpa32U ring_nr = reg & ICP_PMISCBAR_VF2PFMSGMASKNR;
    ring_nr >>= ICP_PMISCBAR_VF2PFMSGSHIFTRINGNR;
    return ring_nr;
}

static inline void adf_vf2pf_set_ring_number(Cpa32U *reg,
                                             Cpa32U ring_number)
{
    ring_number <<= ICP_PMISCBAR_VF2PFMSGSHIFTRINGNR;
    *reg |= ring_number;
}

static inline icp_adf_ringInfoOperation_t adf_vf2pf_get_operation(Cpa32U reg)
{
    reg &= ICP_PMISCBAR_VF2PFMSGMASKOP;
    reg >>= ICP_PMISCBAR_VF2PFMSGSHIFTRINGOP;
    return (icp_adf_ringInfoOperation_t) reg;
}

static inline void adf_vf2pf_set_operation(Cpa32U *reg,
                                           icp_adf_ringInfoOperation_t op)
{
    Cpa32U opmsk = (Cpa32U) op;
    opmsk <<= ICP_PMISCBAR_VF2PFMSGSHIFTRINGOP;
    *reg |= opmsk;
}

static inline icp_adf_ringInfoService_t adf_vf2pf_get_ring_info(Cpa32U reg)
{
    reg &= ICP_PMISCBAR_VF2PFMSGMASKINFO;
    reg >>= ICP_PMISCBAR_VF2PFMSGSHIFTRINGINFO;
    return (icp_adf_ringInfoService_t) reg;
}

static inline void adf_vf2pf_set_ring_info(Cpa32U *reg,
                                           icp_adf_ringInfoService_t info)
{
    Cpa32U infomsk = (Cpa32U) info;
    infomsk <<= ICP_PMISCBAR_VF2PFMSGSHIFTRINGINFO;
    *reg |= infomsk;
}

static inline void adf_vf2pf_set_irq(Cpa32U *reg)
{
    *reg |= (1<<ICP_PMISCBAR_VF2PFMSGSHIFTRINGIRQ);
}

static inline void adf_pf2vf_set_irq(Cpa32U *reg)
{
    *reg |= (1<<ICP_PMISCBAR_PF2VFMSGSHIFTIRQ);
}

static inline void adf_vf2pf_set_response_ring(Cpa32U *reg)
{
    *reg |= (1<<ICP_PMISCBAR_VF2PFMSGSHIFTRINGFLAG);
}

static inline int adf_vf2pf_get_response_ring(Cpa32U reg)
{
    reg &= ICP_PMISCBAR_VF2PFMSGMASKFLAG;
    reg >>= ICP_PMISCBAR_VF2PFMSGSHIFTRINGFLAG;
    return (int) reg;
}

static inline void adf_vf2pf_set_msg_type(Cpa32U *reg, Cpa32U type)
{
    *reg |= (type<<ICP_PMISCBAR_VF2PFMSGSHIFTMSGTYPE);
}

static inline void adf_pf2vf_set_msg_type(Cpa32U *reg, Cpa32U type)
{
    *reg |= (type<<ICP_PMISCBAR_PF2VFMSGSHIFTMSGTYPE);
}

static inline int adf_vf2pf_get_msg_type(Cpa32U reg)
{
    reg &= ICP_PMISCBAR_VF2PFMSGMASKMSGTYPE;
    reg >>= ICP_PMISCBAR_VF2PFMSGSHIFTMSGTYPE;
    return (int) reg;
}

static inline int adf_pf2vf_get_msg_type(Cpa32U reg)
{
    reg &= ICP_PMISCBAR_PF2VFMSGMASKMSGTYPE;
    reg >>= ICP_PMISCBAR_PF2VFMSGSHIFTMSGTYPE;
    return (int) reg;
}

/*
 * Notify PF about restarting event
 */
CpaStatus adf_PF2VF_notify(icp_accel_dev_t *accel_dev);

/*
 * Notify PF about new transport handle
 */
CpaStatus adf_VF2PF_notify(icp_accel_dev_t *accel_dev,
                                    void* data,
                                    icp_adf_ringInfoService_t info,
                                    icp_adf_ringInfoOperation_t op,
                                    Cpa32U flags);

/*
 * Notify PF about VF init
 */
CpaStatus adf_VF2PF_init(icp_accel_dev_t *accel_dev);

/*
 * Notify PF about VF shutdown
 */
CpaStatus adf_VF2PF_shutdown(icp_accel_dev_t *accel_dev);

/*
 * Gets VF ring offset
 */
Cpa32U adf_VF2PF_get_ring_offset(icp_accel_dev_t *accel_dev);

/*
 * adf_vfbh_handler
 * VF2PF IRQ bottom half handler
 */
void adf_vf_bh_handler(void *handle);
#endif
