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
 * @file adf_user_vf2pf_comms.c
 *
 * @description
 * User space interface to VFPF comms driver in kernel space
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_adf_user_proxy.h"
#include "adf_platform_common.h"
#include "adf_platform_acceldev_common.h"
#include <linux/ioctl.h>
#include "adf_pfvfcomms.h"
#include "icp_adf_init.h"
#include "icp_platform.h"
#include "adf_user_vf2pf_comms.h"
#include "icp_adf_accel_mgr.h"
#include "adf_cfg.h"
#include "adf_config.h"
#include "adf_devmgr.h"

#define ADF_PFVF_USERSPACE_MSG_MASK     0x3FFF
#define ADF_PFVF_MSGCOUNT_MEMORY_OFFSET 32768
#define ADF_PFVF_USERSPACE_COUNT_MASK   0xFFFF
#define ADF_PFVF_USERSPACE_MSG_SHIFT    0x10

#define MAX_MSG_BUFFER_SIZE \
    ICP_ADF_MAX_NUM_VF * sizeof(Cpa32U) * ADF_MAX_DEVICES 

STATIC volatile Cpa32U *adf_pfvf_usermsg_buffer = NULL;
STATIC Cpa16U messageCount[ADF_MAX_DEVICES][ICP_ADF_MAX_NUM_VF] = {{0,0}};

STATIC int pfvfcomms_file = -1;


STATIC CpaStatus icp_adf_initialise_pfvfcomms_counts(Cpa32U);

STATIC inline CpaStatus adf_user_pfvf_paramCheck(Cpa32U accelId,
                                                 CpaBoolean isVfApi,
                                                 Cpa32U pfvfNum)
{
    CpaBoolean isVf = CPA_FALSE, isPf = CPA_FALSE;
    icp_accel_dev_t *accel_dev = NULL;

    accel_dev = icp_adf_getAccelDevByAccelId(accelId);

    if (NULL == accel_dev)
    {
        ADF_ERROR("Cannot get accel device from accelId %d in pfvfcomms API\n",
                                                                        accelId);
        return CPA_STATUS_FAIL;
    }

    /* Check if virtualised device and if so if Pf or Vf driver*/
    /* if neither then it's a non-virtualised device */
    isPf = accel_dev->virtualization.enabled ? CPA_TRUE : CPA_FALSE;
    isVf = accel_dev->virtualization.virtualized ?  CPA_TRUE : CPA_FALSE;
    if (!isPf && !isVf)
    {
        ADF_ERROR("Call to pfvfcomms API not supported on non-virtualized device\n");
        return CPA_STATUS_UNSUPPORTED;
    }

   if (isVfApi)
   {
       /* check running on a VF and valid number for PF */
       if (!isVf)
       {
            ADF_ERROR("VF specific API called on PF\n");
            return CPA_STATUS_INVALID_PARAM;
       }
       if (pfvfNum != 0)
       {
            ADF_ERROR("PF Num %d must be 0 on VF API call\n", pfvfNum);
            return CPA_STATUS_INVALID_PARAM;
       }
   }
   else
   {
       /* check running on a PF and VF num is within range */
       if (!isPf)
       {
            ADF_ERROR("PF specific API called on VF\n");
            return CPA_STATUS_INVALID_PARAM;
       }
       if ((pfvfNum == 0) || (pfvfNum > accel_dev->maxNumVf))
       {
            ADF_ERROR("VF Num %d must be 1..%d on PF API call\n", pfvfNum,
                                                        accel_dev->maxNumVf);
            return CPA_STATUS_INVALID_PARAM;
       }

   }

   if (pfvfcomms_file <= 0)
   {
       ADF_ERROR("Process access to pfvfcomms not set up\n");
       return CPA_STATUS_FAIL;
   }

   return CPA_STATUS_SUCCESS;
}

/*
 * adf_user_vfpf_removeProcess
 * 
 */
CpaStatus adf_user_vfpf_removeProcess()
{
    
    if (NULL != adf_pfvf_usermsg_buffer )
    {
        ICP_MUNMAP((void *)adf_pfvf_usermsg_buffer, MAX_MSG_BUFFER_SIZE) ;
        adf_pfvf_usermsg_buffer = NULL;
    }

    if (pfvfcomms_file > 0)
    {
        close(pfvfcomms_file);
        pfvfcomms_file = -1;
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * adf_user_vfpf_registerProcess
 * 
 * every time we create a new user process we register the process
 * with the pfvf_comms device.
 */
CpaStatus adf_user_vfpf_registerProcess(Cpa32U accelId)
{
    Cpa32U maxPfvfBufIndex = 0;
    icp_accel_dev_t *accel_dev = NULL;

    accel_dev = icp_adf_getAccelDevByAccelId(accelId);

    if (NULL == accel_dev)
    {
        ADF_ERROR("Cannot get accel device from accelId %d in pfvfcomms API\n",
            accelId);
        return CPA_STATUS_FAIL;
    }
 
    if(!accel_dev->virtualization.virtualized && 
                                      !accel_dev->virtualization.enabled)
    {
        /* This is not a virtualised device so no need for pfvfcomms */
        ADF_DEBUG("Not registered for pfvfcomms as non-virt system\n");
        return CPA_STATUS_SUCCESS;
    }
    else
    {
        /* registration is once per process,not per device, so just 
         * do on first device */
        if (-1 != pfvfcomms_file)
        {
            ADF_DEBUG("process already registered for pfvfcomms. accelId %d\n",
                                                                 accelId );
            return CPA_STATUS_SUCCESS;
        }

        pfvfcomms_file = open(ADF_DEV_PFVFCOMMS_PATH, O_RDWR);
        if (pfvfcomms_file < 0)
        {
            ADF_ERROR("Cannot open ""%s"" file\n", ADF_DEV_PFVFCOMMS_PATH );
            return CPA_STATUS_FAIL;
        }

        adf_pfvf_usermsg_buffer = ICP_MMAP(0,
                MAX_MSG_BUFFER_SIZE,
                PROT_READ,
                MAP_FILE|MAP_SHARED|MAP_LOCKED,
                pfvfcomms_file,
                0);
        if (MAP_FAILED == adf_pfvf_usermsg_buffer)
        {
            ADF_ERROR("pfvfcomms mmap failed\n");
            close(pfvfcomms_file);
            pfvfcomms_file = -1;
            return CPA_STATUS_FAIL;
        }
        
        maxPfvfBufIndex =
                   (accel_dev->virtualization.virtualized ?
                           ADF_PFVF_MAX_NUM_PF : accel_dev->maxNumVf);
        icp_adf_initialise_pfvfcomms_counts(maxPfvfBufIndex);
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_pfvf_sendMsg
 * 
 * common function for both VF and PF messaging. sends a message to either
 * the PF or the VF. calls the IOCTL function in the vfpf_comms char device 
 */
STATIC CpaStatus adf_pfvf_sendMsg(Cpa32U accelId, Cpa32U vfNum,
                    Cpa32U message, CpaBoolean isVfApi)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U ret = 0;
    adf_sendPfVfMsg_args_t* args = NULL;

    /* check params commom to all APIs */
    status = adf_user_pfvf_paramCheck(accelId, isVfApi, vfNum);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("invalid API call accelId %d, %s, pf/vfNum %d\n",
                accelId, isVfApi ? "VF API" :"PF API", vfNum);
    }
    else
    {
        args = ICP_MALLOC_GEN (sizeof(adf_sendPfVfMsg_args_t));

        if (NULL == args)
        {
            ADF_ERROR("unable to allocate args buffer\n");
            status = CPA_STATUS_FAIL;
        }

        if (status == CPA_STATUS_SUCCESS)
        {

            memset(args, 0, sizeof(adf_sendPfVfMsg_args_t));
            args->accelId = accelId;
            args->vfNum = vfNum;
            args->message = message & ADF_PFVF_USERSPACE_MSG_MASK;

            if (isVfApi == CPA_FALSE)
            {
                ret = ioctl(pfvfcomms_file, ADF_PFVF_IOC_SENDMSG_TO_VF_FROM_PF, args);
            }
            else
            {
                ret = ioctl(pfvfcomms_file, ADF_PFVF_IOC_SENDMSG_TO_PF_FROM_VF, args);
            }

            switch (ret) {
            case -EAGAIN:       status = CPA_STATUS_RETRY;
                                break;
            case 0:             status = CPA_STATUS_SUCCESS;
                                break;
            default:            status = CPA_STATUS_FAIL;
                                ADF_ERROR("pfvf ioctl failed, ret=%d\n", ret);
                                break;
            }
        }
    }

    if (NULL != args)
    {
        ICP_FREE(args);
    }
    return status;
}

/*
 * icp_adf_send_msg_to_vf
 * 
 * Called from the Host(PF)side to send a message to a given VF 
 */
CpaStatus icp_adf_send_msg_to_vf(Cpa32U accelId, Cpa32U vfNum, Cpa32U message )
{
    return  adf_pfvf_sendMsg(accelId, vfNum, message, CPA_FALSE);
}

/*
 * icp_adf_send_msg_to_pf
 * 
 * Called from the VM(VF)side to send a message to the PF 
 */
CpaStatus icp_adf_send_msg_to_pf(Cpa32U accelId, Cpa32U message )
{
    return adf_pfvf_sendMsg(accelId, 0, message, CPA_TRUE);
}




/*
 * icp_adf_get_msg
 * 
 * This collects a message from the kernel space buffer mapped to this
 * process if a new message is available.
 */
STATIC CpaStatus icp_adf_get_msg(Cpa32U accelId, Cpa32U pfvfNum,
                                 Cpa32U *message, Cpa32U *messageCounter,
                                CpaBoolean isVfApi)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U numNewMsgs = 0, pfvfBufIndex = 0;
    Cpa32U messageAndCounter = 0;
    Cpa16U lastMsgReceived = 0, lastMsgCollected = 0;

    /* check params commom to all APIs */
    status = adf_user_pfvf_paramCheck(accelId, isVfApi, pfvfNum);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("invalid API call accelId %d, %s, pf/vfNum %d\n",
                accelId, isVfApi ? "VF API" :"PF API", pfvfNum);
        return status;
    }

    if (NULL == message)
    {
        ADF_ERROR("NULL message pointer\n");
        return CPA_STATUS_INVALID_PARAM;
    }
    if (NULL == messageCounter)
    {
        ADF_ERROR("NULL messageCounter pointer\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    if (NULL == adf_pfvf_usermsg_buffer)
    {
        ADF_ERROR("adf_pfvf_usermsg_buffer is not mapped\n");
        return CPA_STATUS_FAIL;
    }

    if (isVfApi)
    {
        pfvfBufIndex = pfvfNum;
    }
    else
    {
        /* use 0-31 when indexing buffer. vfNum is 1-32 */
        pfvfBufIndex = pfvfNum-1;
    }
    messageAndCounter = adf_vfpf_messageInBuffer(adf_pfvf_usermsg_buffer,
                                            accelId, pfvfBufIndex);

    lastMsgReceived = messageAndCounter & ADF_PFVF_USERSPACE_COUNT_MASK;
    lastMsgCollected = messageCount[accelId][pfvfBufIndex];

    if (lastMsgReceived >= lastMsgCollected)
    {
        numNewMsgs = lastMsgReceived - lastMsgCollected;
    }
    else
    {
        /* kernel counter wrapped */
        numNewMsgs = lastMsgReceived + UINT16_MAX + 1 - lastMsgCollected;
    }

    *messageCounter = numNewMsgs;
    if (numNewMsgs > 0 )
    {
        *message = messageAndCounter >> ADF_PFVF_USERSPACE_MSG_SHIFT;
    }
    else
    {
        *message = 0;
    }

    /* Update the array local to this process with the number
     * of the message collected, to avoid picking up the same msg twice.
     */
    messageCount[accelId][pfvfBufIndex] = lastMsgReceived;

    return status;
}

/*
 * icp_adf_get_msg_from_vf
 * 
 * This function should only be called on a Host(PF)
 */
CpaStatus icp_adf_get_msg_from_vf(Cpa32U accelId, Cpa32U vfNum,
        Cpa32U *message, Cpa32U *messageCounter)
{
    return icp_adf_get_msg(accelId, vfNum, message, messageCounter, CPA_FALSE);
}

/*
 * icp_adf_get_msg_from_pf
 * 
 * This function should only be called on a Guest(VF)
 */
CpaStatus icp_adf_get_msg_from_pf(Cpa32U accelId,
                        Cpa32U *message, Cpa32U *messageCounter)
{
    return icp_adf_get_msg(accelId, 0, message, messageCounter, CPA_TRUE);
}

/*
 * icp_adf_get_pfvfcomms_status
 * sets the unreadMessage flag to true if a message exists
 * (in the kernel space buffer mapped to this process) which has not
 * been collected yet in this process by a get_msg API call
 */
CpaStatus icp_adf_get_pfvfcomms_status(CpaBoolean *unreadMessage)
{
    int accelId=0, pfvfBufIndex=0, maxBufIndex=0;;
    Cpa16U lastMsgReceived = 0, lastMsgCollected = 0;
    Cpa32U messageAndCounter;
    icp_accel_dev_t *accel_dev = NULL;

    if (NULL == unreadMessage)
    {
        ADF_ERROR("unreadMessage null pointer\n");
        return CPA_STATUS_INVALID_PARAM;
    }
    *unreadMessage = CPA_FALSE;
    
    if (NULL == adf_pfvf_usermsg_buffer)
    {
        ADF_ERROR("pfvfcomms API call failed. "
                "adf_pfvf_usermsg_buffer is not mapped\n");
        return CPA_STATUS_FAIL;
    }

    for (accelId=0; accelId<ADF_MAX_DEVICES; accelId++)
    {
        if (NULL == (accel_dev = icp_adf_getAccelDevByAccelId(accelId)))
        {
            ADF_DEBUG("skipping accelId %d as not available\n",accelId);
            continue;
        }

        maxBufIndex = 
            (accel_dev->virtualization.virtualized ? ADF_PFVF_MAX_NUM_PF : accel_dev->maxNumVf);
        for (pfvfBufIndex = 0; pfvfBufIndex < maxBufIndex; pfvfBufIndex++)
        {
            messageAndCounter = adf_vfpf_messageInBuffer(adf_pfvf_usermsg_buffer, accelId, pfvfBufIndex);
            lastMsgReceived = messageAndCounter & ADF_PFVF_USERSPACE_COUNT_MASK;
            lastMsgCollected = messageCount[accelId][pfvfBufIndex];

            if (lastMsgCollected != lastMsgReceived)
            {
                *unreadMessage = CPA_TRUE;
                break;
            }
        }
        /* No need to check more devices if we already found one */
        if (*unreadMessage)
        {
            break;
        }
    }
    return CPA_STATUS_SUCCESS;
}

STATIC CpaStatus icp_adf_initialise_pfvfcomms_counts(Cpa32U maxBufIndex)
{
    int accelId=0, pfvfBufIndex=0;
    Cpa32U messageAndCounter;

    if (NULL == adf_pfvf_usermsg_buffer)
    {
        ADF_ERROR("adf_pfvf_usermsg_buffer is not mapped\n");
        return CPA_STATUS_FAIL;
    }

    for (accelId=0; accelId<ADF_MAX_DEVICES; accelId++)
    {
        for (pfvfBufIndex = 0; pfvfBufIndex < maxBufIndex; pfvfBufIndex++)
        {
            messageAndCounter = adf_vfpf_messageInBuffer(
                    adf_pfvf_usermsg_buffer, accelId, pfvfBufIndex);
            messageCount[accelId][pfvfBufIndex] =
                    messageAndCounter & ADF_PFVF_USERSPACE_COUNT_MASK;
        }
    }
    return CPA_STATUS_SUCCESS;
}
