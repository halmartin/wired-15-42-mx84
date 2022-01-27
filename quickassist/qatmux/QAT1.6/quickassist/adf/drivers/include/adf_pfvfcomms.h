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
 *****************************************************************************/

/******************************************************************************
 * @file adf_pfvfcomms.h
 *
 * @description
 *     Header file for char device driver code for managing the user space
 *     PF VF comms.
 *
 *****************************************************************************/
#ifndef ADF_VFPFCOMMS_H
#define ADF_VFPFCOMMS_H

typedef struct adf_user_msgElement_s{
        char*    name;
        Cpa32U*  buffer;
        Cpa16U*  msgCount;
}adf_user_msgElement_t;

typedef struct adf_sendPfVfMsg_args_s {
	    Cpa32U accelId;
	    Cpa32U vfNum;
	    Cpa32U message;
	    Cpa32U *messageCounter;
	    CpaBoolean *unreadMessage;
	    adf_user_msgElement_t msgElement;
}adf_sendPfVfMsg_args_t;

CpaStatus adf_vfpf_storeMsgForUser(icp_accel_dev_t *accel_dev, Cpa32U vfNum ,Cpa16U msg );

/* IOCTL number for use between the kernel and the user space application */
#define ADF_PFVFCOMMS_MAGIC             'v'
#define ADF_SENDMSG_TO_VF_FROM_PF       (0)
#define ADF_SENDMSG_TO_PF_FROM_VF       (1)

/* IOCTL commands for requesting kernel memory */
#define ADF_PFVF_IOC_SENDMSG_TO_VF_FROM_PF \
        _IOWR(ADF_PFVFCOMMS_MAGIC, ADF_SENDMSG_TO_VF_FROM_PF, \
        		adf_sendPfVfMsg_args_t)

#define ADF_PFVF_IOC_SENDMSG_TO_PF_FROM_VF \
        _IOR(ADF_PFVFCOMMS_MAGIC, ADF_SENDMSG_TO_PF_FROM_VF, \
        		adf_sendPfVfMsg_args_t)

/* Macro to force consistent indexing into the adf_pfvf_usermsg_buffer */
#define adf_vfpf_messageInBuffer(buffer, accelId, vfNum) \
    *(buffer + accelId * ICP_ADF_MAX_NUM_VF + vfNum)

#endif /* ADF_VFPFCOMMS_H */
