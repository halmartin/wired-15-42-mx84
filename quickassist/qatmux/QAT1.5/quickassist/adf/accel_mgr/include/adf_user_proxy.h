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
 *****************************************************************************/

/******************************************************************************
 * @file adf_user_proxy.h
 *
 * @description
 *      This is the header file contains user space proxy structures
 *      and functions declarations.
 *
 *****************************************************************************/
#ifndef ADF_USER_PROXY_H
#define ADF_USER_PROXY_H

/******************************************************************************
 * Section for #define's & typedef's
 *****************************************************************************/
#define ADF_MAX_PENDING_EVENT 10
#define ADF_PROCESS_MUST_WAIT 1
#define ADF_PROCESS_TOO_MANY  2
/* That meny characters + the name of the process will be in each
 * line of process info file */
#define ADF_PROCESS_ADDITIONAL_STR_LEN 12
#define ADF_USER_PROCESS_IOCTL_RELEASE 0xF /* IOCTL CMD */
#define ADF_PROCESS_PROXY_RUNNING 1 /* adf user process running  val */
#define ADF_PROCESS_PROXY_STOPPED 0 /* adf user process stop val */



/******************************************************************************
 * Section for struct definitions
 *****************************************************************************/

/******************************************************************************
 * @description
 * Structure defining user space process ring list
 *
 *****************************************************************************/
typedef struct adf_user_process_rings_list_s
{
    ICP_SPINLOCK ringListLock;
    icp_trans_handle *listHead;
    icp_trans_handle *listTail;
} adf_user_process_rings_list_t;

/******************************************************************************
 * @description
 * Structure defining user space process dynamic instance list
 *
 *****************************************************************************/
typedef struct adf_user_process_dyn_instances_list_s
{
    icp_dyn_instance_handle_t *listHead;
    icp_dyn_instance_handle_t *listTail;
} adf_user_process_dyn_instances_list_t;

/******************************************************************************
 * @description
 * Structure defining user space process
 *
 *****************************************************************************/
typedef struct adf_user_process_s
{
    Cpa32U processId;
    icp_accel_dev_t* accelHandle;
    adf_user_process_rings_list_t *pUserRings;
    adf_user_process_dyn_instances_list_t *pDynInstances;
    Cpa32U pendingEvents;
    Cpa32U pendingEventsQueue[ADF_MAX_PENDING_EVENT];
    Cpa32U processUsageCounter;
    Cpa8U processProxyState;
    void* processWaitQueue;
    OsalSemaphore processDisconnectedFlag;
    ICP_SPINLOCK userProcLock;
    struct adf_user_process_s *pPrev;
    struct adf_user_process_s *pNext;
} adf_user_process_t;

/******************************************************************************
 * @description
 * Structure defining user space process list
 *
 *****************************************************************************/
typedef struct adf_user_process_list_s
{
    ICP_MUTEX procListLock;
    adf_user_process_t *listHead;
    adf_user_process_t *listTail;
} adf_user_process_list_t;

/******************************************************************************
 * Section for interface function prototypes
 *****************************************************************************/
/******************************************************************************
 * adf_userProcessCreateRing
 * @description
 * Creates ring handle within user proxy that is used to manage rings created
 * from userspace.
 *
 *****************************************************************************/
CpaStatus adf_userProcessCreateRing( icp_accel_dev_t *accel_hdl,
                                     icp_trans_handle *ring_hdl,
                                     Cpa32U pId );

/******************************************************************************
 * adf_userProcessDeleteRing
 * @description
 * Removes ring handle from user proxy.
 *
 *****************************************************************************/
CpaStatus adf_userProcessDeleteRing( icp_accel_dev_t *accel_hdl,
                                     icp_trans_handle *ring_hdl,
                                     Cpa32U pId );

/******************************************************************************
 * adf_userProcessCreateDynInstance
 * @description
 * Creates dyn instance handle within user proxy that is used to
 * manage dyn instances created from userspace.
 *
 *****************************************************************************/
CpaStatus adf_userProcessCreateDynInstance(icp_accel_dev_t *accel_hdl,
                                           Cpa32U inst_id,
                                           adf_service_type_t stype,
                                           Cpa32U pid);

/******************************************************************************
 * adf_userProcessDeleteDynInstance
 * @description
 * Removes dyn instance handle from user proxy.
 *
 *****************************************************************************/
CpaStatus adf_userProcessDeleteDynInstance(icp_accel_dev_t *accel_hdl,
                                           Cpa32U inst_id,
                                           adf_service_type_t stype,
                                           Cpa32U pid);

/******************************************************************************
 * adf_userProcessConnect
 * @description
 * Creates user space process handle within user proxy.
 *
 *****************************************************************************/
CpaStatus adf_userProcessConnect( void** proxyPrivateData,
                                  Cpa32U accelDevId, Cpa32U procId );

/******************************************************************************
 * adf_userProcessDisconnet
 * @description
 * Removes user space process handle from user proxy.
 *
 *****************************************************************************/
CpaStatus adf_userProcessDisconnect( void** proxyPrivateData );

/******************************************************************************
 * icp_adf_userSpaceRegister
 * @description
 * Register user space proxy subsystem
 *
 *****************************************************************************/
CpaStatus adf_userSpaceRegister( void );

/******************************************************************************
 * icp_adf_userSpaceUnregister
 * @description
 * Unregister user space proxy subsystem
 *
 *****************************************************************************/
CpaStatus adf_userSpaceUnregister( void );

/******************************************************************************
 * icp_adf_userSpaceInit
 * @description
 * Initialise user space proxy subsystem
 *
 *****************************************************************************/
CpaStatus adf_user_proxyInit( icp_accel_dev_t* accel_dev );

/******************************************************************************
 * icp_adf_userSpaceShutdown
 * @description
 * Shutdown user space proxy subsystem
 *
 *****************************************************************************/
CpaStatus adf_user_proxyShutdown( icp_accel_dev_t* accel_dev );

/******************************************************************************
 * adf_user_proxyEventGetPut
 * @description
 * Function increments the device usage counter
 *
 *****************************************************************************/
CpaStatus adf_user_proxyEventGetPut( icp_accel_dev_t *accel_dev,
                                     Cpa32U pid, Cpa32U cmd );

#endif /* ADF_USER_PROXY_H */
