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
 * @file adf_platform_dh89xxcciov.h
 *
 * @description
 *      This file contains the platform specific macros for DH89xxCC processor
 *
 *****************************************************************************/
#ifndef ADF_PLATFORM_DH89xxCCIOV_H
#define ADF_PLATFORM_DH89xxCCIOV_H

/*****************************************************************************
 * Define Constants and Macros
 *****************************************************************************/

/* PCIe configuration space */
#define ICP_DH89xxCCIOV_PETRINGCSR_BAR          0
#define ICP_DH89xxCCIOV_PETRINGCSR_BAR_SIZE     0x1000  /* (4KB) */
#define ICP_DH89xxCCIOV_VFTOPF_BAR              1
#define ICP_DH89xxCCIOV_VFTOPF_BAR_SIZE         0x1000  /* (4KB) */
#define ICP_DH89xxCCIOV_MAX_PCI_BARS            2

/* Clock */
#define ICP_DH89xxCCIOV_AE_CLOCK_IN_MHZ         1066

/* ETR */
#define ICP_DH89xxCCIOV_ETR_MAX_BANKS           1
#define ICP_DH89xxCCIOV_ETR_MAX_RINGS_PER_BANK  16
#define ICP_DH89xxCCIOV_BANKS_PER_ACCELERATOR   1
#define ICP_DH89xxCCIOV_MAX_ET_RINGS   \
       (ICP_DH89xxCCIOV_ETR_MAX_BANKS * ICP_DH89xxCCIOV_ETR_MAX_RINGS_PER_BANK)
#define ICP_DH89xxCCIOV_ETR_MAX_AP_BANKS        1

/* Fuse Conrol */
#define ICP_DH89xxCCIOV_MAX_ACCELERATORS        1
#define ICP_DH89xxCCIOV_MAX_ACCELENGINES        1
#define ICP_DH89xxCCIOV_ACCELERATORS_MASK       0x1
#define ICP_DH89xxCCIOV_ACCELENGINES_MASK       0x1

/* PF-to-VF */
#define ICP_DH89xxCCIOV_PF2VFDBINT_OFFSET       0x200
#define ICP_DH89xxCCIOV_VINTSRC_OFFSET          0x204
#define ICP_DH89xxCCIOV_VINTMSK_OFFSET          0x208
#define ICP_DH89xxCCIOV_VINTMSK_DEFAULT         0x02

#define ICP_DH89xxCCIOV_BUNDLE_IRQ_MASK         0x1
#define ICP_DH89xxCCIOV_PF2VF_IRQ_MASK          0x2

/* PETRING BAR offsets */
#define ICP_DH89xxCCIOV_ETRING_CSR_OFFSET       0x00

/* Ring configuration space size mapped to userspace */
#define ICP_DH89xxCCIOV_ETRING_CSR_SIZE         0x200

/* User space */
#define ICP_DH89xxCCIOV_USER_ETRING_CSR_SIZE    0x1000

/*
 * irq_bun_source
 * Function returns the value of bundle interrupt source
 */
static inline Cpa32U irq_bun_source(Cpa32U vint)
{
    Cpa32U bun_irq = 0;
    bun_irq = vint & ICP_DH89xxCCIOV_BUNDLE_IRQ_MASK;
    return bun_irq;
}

/*
 * irq_pf2vf_source
 * Function returns the value of PF2VF interrupt source
 */
static inline Cpa32U irq_pf2vf_source(Cpa32U vint)
{
    Cpa32U pf2vf_irq = 0;
    pf2vf_irq = vint & ICP_DH89xxCCIOV_PF2VF_IRQ_MASK;
    return pf2vf_irq;
}

#endif /* ADF_PLATFORM_DH89xxCCIOV_H */
