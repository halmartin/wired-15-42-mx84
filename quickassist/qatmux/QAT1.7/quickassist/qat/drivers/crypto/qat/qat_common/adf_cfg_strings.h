/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2014 Intel Corporation.
  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  qat-linux@intel.com

  BSD LICENSE
  Copyright(c) 2014 Intel Corporation.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef ADF_CFG_STRINGS_H_
#define ADF_CFG_STRINGS_H_

#define ADF_GENERAL_SEC "GENERAL"
#define ADF_KERNEL_SEC "KERNEL"
#define ADF_KERNEL_SAL_SEC "KERNEL_QAT"
#define ADF_ACCEL_SEC "Accelerator"
#define ADF_NUM_CY "NumberCyInstances"
#define ADF_NUM_DC "NumberDcInstances"
#define ADF_RING_SYM_SIZE "NumConcurrentSymRequests"
#define ADF_RING_ASYM_SIZE "NumConcurrentAsymRequests"
#define ADF_RING_DC_SIZE "NumConcurrentRequests"
#define ADF_RING_ASYM_TX "RingAsymTx"
#define ADF_RING_SYM_TX "RingSymTx"
#define ADF_RING_ASYM_RX "RingAsymRx"
#define ADF_RING_SYM_RX "RingSymRx"
#define ADF_RING_DC_TX "RingTx"
#define ADF_RING_DC_RX "RingRx"
#define ADF_ETRMGR_BANK "Bank"
#define ADF_RING_BANK_NUM "BankNumber"
#define ADF_CY "Cy"
#define ADF_DC "Dc"
#define ADF_ETRMGR_COALESCING_ENABLED "InterruptCoalescingEnabled"
#define ADF_ETRMGR_COALESCING_ENABLED_FORMAT \
	ADF_ETRMGR_BANK "%d" ADF_ETRMGR_COALESCING_ENABLED
#define ADF_ETRMGR_COALESCE_TIMER "InterruptCoalescingTimerNs"
#define ADF_ETRMGR_COALESCE_TIMER_FORMAT \
	ADF_ETRMGR_BANK "%d" ADF_ETRMGR_COALESCE_TIMER
#define ADF_ETRMGR_COALESCING_MSG_ENABLED "InterruptCoalescingNumResponses"
#define ADF_ETRMGR_COALESCING_MSG_ENABLED_FORMAT \
	ADF_ETRMGR_BANK "%d" ADF_ETRMGR_COALESCING_MSG_ENABLED
#define ADF_ETRMGR_CORE_AFFINITY "CoreAffinity"
#define ADF_ETRMGR_CORE_AFFINITY_FORMAT \
	ADF_ETRMGR_BANK "%d" ADF_ETRMGR_CORE_AFFINITY
#define ADF_ACCEL_STR "Accelerator%d"
#define ADF_DEV_SSM_WDT_BULK "CySymAndDcWatchDogTimer"
#define ADF_DEV_SSM_WDT_PKE  "CyAsymWatchDogTimer"
#define ADF_DH895XCC_AE_FW_NAME "icp_qat_ae.uof"
#define ADF_CXXX_AE_FW_NAME "icp_qat_ae.suof"
#define ADF_HEARTBEAT_TIMER  "HeartbeatTimer"
#define ADF_MMP_VER_KEY "Firmware_MmpVer"
#define ADF_UOF_VER_KEY "Firmware_UofVer"
#define ADF_HW_REV_ID_KEY "HW_RevId"
#define ADF_DEV_MAX_BANKS "Device_Max_Banks"
#define ADF_DEV_CAPABILITIES_MASK "Device_Capabilities_Mask"
#define ADF_DEV_NODE_ID "Device_NodeId"
#define ADF_DEV_PKG_ID "Device_PkgId"
#define ADF_FIRST_USER_BUNDLE "FirstUserBundle"
#define ADF_INTERNAL_USERSPACE_SEC_SUFF     "_INT_"
#define ADF_LIMIT_DEV_ACCESS "LimitDevAccess"
#define ADF_NUM_PROCESSES "NumProcesses"
#define ADF_DEV_KPT_ENABLE "KptEnabled"
#define ADF_STORAGE_FIRMWARE_ENABLED "StorageEnabled"
#define ADF_DH895XCC_AE_FW_NAME_STORAGE "icp_qat_ae_storage.uof"
#define ADF_CXXX_AE_FW_NAME_STORAGE "icp_qat_ae_storage.suof"
#define ADF_CXXX_AE_FW_NAME_KPT "icp_qat_ae_kpt.suof"
#define ADF_DC_EXTENDED_FEATURES "Device_DcExtendedFeatures"
#define ADF_PKE_DISABLED "PkeServiceDisabled"
#define ADF_INTER_BUF_SIZE "DcIntermediateBufferSizeInKB"
#define ADF_AUTO_RESET_ON_ERROR "AutoResetOnError"
#define ADF_CFG_CY "cy"
#define ADF_CFG_DC "dc"
#define ADF_CFG_ASYM "asym"
#define ADF_CFG_SYM "sym"
#define ADF_SERVICES_ENABLED "ServicesEnabled"
#define ADF_POLL_MODE "IsPolled"
#define ADF_CY_RING_SYM_SIZE (ADF_CY ADF_RING_SYM_SIZE)
#define ADF_CY_RING_ASYM_SIZE (ADF_CY ADF_RING_ASYM_SIZE)
#define ADF_DC_RING_SIZE (ADF_DC ADF_RING_DC_SIZE)
#define ADF_ASYM "Asym"
#define ADF_SYM "Sym"
#define ADF_CY_CORE_AFFINITY_FORMAT \
	ADF_CY "%d" ADF_ETRMGR_CORE_AFFINITY
#define ADF_DC_CORE_AFFINITY_FORMAT \
	ADF_DC "%d" ADF_ETRMGR_CORE_AFFINITY
#define ADF_CY_BANK_NUM_FORMAT \
	ADF_CY "%d" ADF_RING_BANK_NUM
#define ADF_DC_BANK_NUM_FORMAT \
	ADF_DC "%d" ADF_RING_BANK_NUM
#define ADF_CY_ASYM_TX_FORMAT \
	ADF_CY "%d" ADF_RING_ASYM_TX
#define ADF_CY_SYM_TX_FORMAT \
	ADF_CY "%d" ADF_RING_SYM_TX
#define ADF_CY_ASYM_RX_FORMAT \
	ADF_CY "%d" ADF_RING_ASYM_RX
#define ADF_CY_SYM_RX_FORMAT \
	ADF_CY "%d" ADF_RING_SYM_RX
#define ADF_DC_TX_FORMAT \
	ADF_DC "%d" ADF_RING_DC_TX
#define ADF_DC_RX_FORMAT \
	ADF_DC "%d" ADF_RING_DC_RX
#define ADF_CY_RING_SYM_SIZE_FORMAT \
	ADF_CY "%d" ADF_RING_SYM_SIZE
#define ADF_CY_RING_ASYM_SIZE_FORMAT \
	ADF_CY "%d" ADF_RING_ASYM_SIZE
#define ADF_DC_RING_SIZE_FORMAT \
	ADF_DC "%d" ADF_RING_DC_SIZE
#define ADF_CY_NAME_FORMAT \
	ADF_CY "%dName"
#define ADF_DC_NAME_FORMAT \
	ADF_DC "%dName"
#define ADF_CY_POLL_MODE_FORMAT \
	ADF_CY "%d" ADF_POLL_MODE
#define ADF_DC_POLL_MODE_FORMAT \
	ADF_DC "%d" ADF_POLL_MODE
#define ADF_CONFIG_VERSION "ConfigVersion"
#define ADF_LIMITED_USER_SECTION_NAME_FORMAT "%s_DEV%d_INT_%d"
#define ADF_USER_SECTION_NAME_FORMAT "%s_INT_%d"
#endif
