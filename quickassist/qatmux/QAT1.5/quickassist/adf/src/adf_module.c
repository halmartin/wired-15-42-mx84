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
 * @file adf_module.c
 *
 * @description
 *      This file contains the device driver's initialization and release
 *      functions. This also contain the list of exported symbols.
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_firml_interface.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_transport_dp.h"
#include "icp_adf_ae.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_esram.h"
#include "icp_adf_poll.h"
#include "adf_drv.h"
#include "adf_cfg.h"
#include "icp_adf_cfg_dp.h"
#include "adf_init.h"
#include "adf_ae.h"
#include "adf_ae_fw.h"
#include "adf_dram.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_cfg.h"
#include "adf_devmgr.h"
#include "adf_user_proxy.h"
#include "icp_adf_debug.h"
#include "adf_proc_debug.h"
#include "OsalDevDrv.h"
#include "adf_wireless.h"
#include "adf_dyn.h"

/*
 * Module static variables
 */
char *icp_module_name = THIS_MODULE->name;

MODULE_DESCRIPTION("ICP Acceleration Device Framework");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE(LICENSE_TYPE);


extern CpaStatus icp_sal_pollBank(Cpa32U accelId,
                           Cpa32U bank_number,
                           Cpa32U response_quota);

extern CpaStatus icp_sal_pollAllBanks(Cpa32U accelId,
                               Cpa32U response_quota);
/*
 * Module exports
 */
EXPORT_SYMBOL( icp_adf_putDynInstance );
EXPORT_SYMBOL( icp_adf_transGetRingNum );
EXPORT_SYMBOL( icp_adf_aePatchSymbol );
EXPORT_SYMBOL( icp_adf_transReleaseHandle );
EXPORT_SYMBOL( icp_adf_transPutMsg );
EXPORT_SYMBOL( icp_adf_transCreateHandle );
EXPORT_SYMBOL( icp_adf_subsystemUnregister );
EXPORT_SYMBOL( icp_adf_subsystemRegister );
EXPORT_SYMBOL( icp_adf_cfgGetParamValue );
EXPORT_SYMBOL( icp_adf_cfgGetParamValueList );
EXPORT_SYMBOL( icp_adf_accesLayerRingInfoCbRegister );
EXPORT_SYMBOL( icp_adf_accesLayerRingInfoCbUnregister );
EXPORT_SYMBOL( icp_adf_isSubsystemStarted );
EXPORT_SYMBOL( icp_amgr_getNumInstances );
EXPORT_SYMBOL( icp_amgr_getInstances );
EXPORT_SYMBOL( icp_amgr_getAccelDevByName );
EXPORT_SYMBOL( icp_amgr_getAccelDevByCapabilities );
EXPORT_SYMBOL( icp_amgr_getAllAccelDevByCapabilities );
EXPORT_SYMBOL( icp_amgr_getAccelDevCapabilities );
EXPORT_SYMBOL( icp_adf_getAccelDevByAccelId );
EXPORT_SYMBOL( icp_adf_esramGetAddress );
EXPORT_SYMBOL( icp_adf_cfgGetRingNumber );
EXPORT_SYMBOL( icp_adf_get_busAddress );
EXPORT_SYMBOL( icp_sal_pollBank );
EXPORT_SYMBOL( icp_sal_pollAllBanks );
EXPORT_SYMBOL( icp_adf_pollInstance );
EXPORT_SYMBOL( icp_qa_dev_get );
EXPORT_SYMBOL( icp_qa_dev_put );

EXPORT_SYMBOL( icp_adf_debugAddFile );
EXPORT_SYMBOL( icp_adf_debugRemoveFile );
EXPORT_SYMBOL( icp_adf_debugAddDir );
EXPORT_SYMBOL( icp_adf_debugRemoveDir );

EXPORT_SYMBOL( icp_adf_cfgGetSharedMemSize );
EXPORT_SYMBOL( icp_adf_cfgExportData );

EXPORT_SYMBOL( icp_adf_updateQueueTail);
EXPORT_SYMBOL( icp_adf_isRingEmpty);
EXPORT_SYMBOL( icp_adf_getSingleQueueAddr);
EXPORT_SYMBOL( icp_adf_getQueueMemory);
EXPORT_SYMBOL( icp_adf_pollQueue);
EXPORT_SYMBOL( icp_adf_getQueueNext);
EXPORT_SYMBOL( icp_adf_queueDataToSend);

/*
 * Osal exports
 */
EXPORT_SYMBOL (osalMemAlloc);
EXPORT_SYMBOL (osalMemAllocAtomic);
EXPORT_SYMBOL (osalMemAllocContiguousNUMA);
EXPORT_SYMBOL (osalMemAllocPage);
EXPORT_SYMBOL (osalMemFree);
EXPORT_SYMBOL (osalMemFreePage);
EXPORT_SYMBOL (osalMemFreeNUMA);
EXPORT_SYMBOL (osalMemCopy);
EXPORT_SYMBOL (osalMemSet);
EXPORT_SYMBOL (osalMemAllocAligned);
EXPORT_SYMBOL (osalMemAlignedFree);
EXPORT_SYMBOL (osalVirtToPhysNUMA);

EXPORT_SYMBOL (osalThreadCreate);
EXPORT_SYMBOL (osalThreadBind);
EXPORT_SYMBOL (osalThreadStart);
EXPORT_SYMBOL (osalThreadPrioritySet);
EXPORT_SYMBOL (osalThreadSetPolicyAndPriority);
EXPORT_SYMBOL (osalThreadKill);
EXPORT_SYMBOL (osalThreadExit);

EXPORT_SYMBOL (osalMutexInit);
EXPORT_SYMBOL (osalMutexLock);
EXPORT_SYMBOL (osalMutexUnlock);
EXPORT_SYMBOL (osalMutexDestroy);

EXPORT_SYMBOL (osalSemaphoreInit);
EXPORT_SYMBOL (osalSemaphoreDestroy);
EXPORT_SYMBOL (osalSemaphoreWaitInterruptible);
EXPORT_SYMBOL (osalSemaphorePostWakeup);
EXPORT_SYMBOL (osalSemaphorePost);
EXPORT_SYMBOL (osalSemaphoreWait);
EXPORT_SYMBOL (osalSemaphoreTryWait);
EXPORT_SYMBOL (osalSemaphoreGetValue);

EXPORT_SYMBOL (osalSleep);
EXPORT_SYMBOL (osalYield);
EXPORT_SYMBOL (osalTimeGet);
EXPORT_SYMBOL (osalTicksToTimeval);
EXPORT_SYMBOL (osalTimevalToTicks);
EXPORT_SYMBOL (osalTimestampGet);

EXPORT_SYMBOL (osalLog);
EXPORT_SYMBOL (osalLog64);
EXPORT_SYMBOL (osalLogString);
EXPORT_SYMBOL (osalLogLevelSet);
EXPORT_SYMBOL (osalStdLog);

EXPORT_SYMBOL (osalLockInit);
EXPORT_SYMBOL (osalLock);
EXPORT_SYMBOL (osalLockBh);
EXPORT_SYMBOL (osalUnlock);
EXPORT_SYMBOL (osalUnlockBh);
EXPORT_SYMBOL (osalLockDestroy);
EXPORT_SYMBOL (osalLockIrqSave);
EXPORT_SYMBOL (osalUnlockIrqRestore);

EXPORT_SYMBOL (osalAtomicGet);
EXPORT_SYMBOL (osalAtomicSet);
EXPORT_SYMBOL (osalAtomicAdd);
EXPORT_SYMBOL (osalAtomicSub);
EXPORT_SYMBOL (osalAtomicInc);
EXPORT_SYMBOL (osalAtomicDec);
EXPORT_SYMBOL (osalAtomicDecAndTest);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34))
EXPORT_SYMBOL (osalHashMD5);
#endif
EXPORT_SYMBOL (osalHashSHA1);
EXPORT_SYMBOL (osalHashSHA224);
EXPORT_SYMBOL (osalHashSHA256);
EXPORT_SYMBOL (osalHashSHA384);
EXPORT_SYMBOL (osalHashSHA512);
EXPORT_SYMBOL (osalAESEncrypt);

EXPORT_SYMBOL (osalIOMMUMap);
EXPORT_SYMBOL (osalIOMMUUnmap);
EXPORT_SYMBOL (osalIOMMUVirtToPhys);
EXPORT_SYMBOL (osalIOMMUgetRemappingSize);

/*
 * Osal lib entry points
 */
extern int  __init osal_init(char* path);
extern void __exit osal_exit(void);
extern int adf_init_sriov(void);
extern void adf_exit_sriov(void);
/*
 * Module inc/dec usage functions
 */
inline int adf_get_module(void)
{
        return try_module_get(THIS_MODULE);
}

inline void adf_put_module(void)
{
        module_put(THIS_MODULE);
}

/*
 * Module init/exit prototypes
 */
STATIC int __init adf_module_init(void)
{
        Cpa32U status = SUCCESS;

#ifndef ADF_PLATFORM_ACCELDEVVF
        status = adf_init_sriov();
        if (SUCCESS == status) {
                status = adf_init_aer();
        }
#endif
        if (SUCCESS == status) {
                status = adf_devmgrInit();
        }
        if (SUCCESS == status) {
                status = adf_init_ETManager();
        }
        if (SUCCESS == status) {
                status = driver_module_init();
        }
        if (SUCCESS == status) {
                status = adf_userSpaceRegister();
        }
        if (SUCCESS == status) {
                status = adf_wirelessRegister();
        }
        if (SUCCESS == status) {
                status = adf_dynRegister();
        }

        /* Initialise osal */
        if (SUCCESS == status) {
#ifdef ONE_KO_RELEASE_PACKAGE
                status = osal_init();
#else
                status = register_mem_device_driver(NULL);
#endif
        }
        return status;
}

/*
 * adfdrv_release
 * ADF Module release function
 */
STATIC void adf_module_release(void)
{
        /* Deinit osal */
#ifdef ONE_KO_RELEASE_PACKAGE
        osal_exit();
#else
        unregister_mem_device_driver();
#endif
        if (SUCCESS != adf_dynUnregister()) {
                ADF_ERROR( "Failed to unregister dyn proxy.\n" );
        }
        if (SUCCESS != adf_wirelessUnregister()) {
                ADF_ERROR( "Failed to unregister wireless proxy.\n" );
        }
        if (SUCCESS != adf_userSpaceUnregister()) {
                ADF_ERROR( "Failed to unregister user space proxy.\n" );
        }
        driver_module_exit();
        adf_shutdown_ETManager();
        adf_devmgrExit();
#ifndef ADF_PLATFORM_ACCELDEVVF
        adf_exit_sriov();
        adf_exit_aer();
#endif
        return;
}

ICP_MODULE_INIT(adf_module_init);
ICP_MODULE_EXIT(adf_module_release);
