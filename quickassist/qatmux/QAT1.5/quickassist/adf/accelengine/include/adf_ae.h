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
 * @file adf_ae.h
 *
 * @description
 *      This header file contains the function prototypes for initialising,
 *      patching symbols etc for the acceleration engines. The functions
 *      are wrappers to Uclo and Hal functions which perform the tasks.
 *
 *****************************************************************************/
#ifndef ADF_AE_H
#define ADF_AE_H

#include "icp_accel_devices.h"

/* Accel Engines mask */
#define ADF_AE_CTX_ENABLE_MASK      (0xFF)
/*
 * Type definition and length of MMP firmware version
 */
#define MMP_VERSION_LEN             4
#define MMP_VER_0                   0
#define MMP_VER_1                   1
#define MMP_VER_2                   2

/**
 *****************************************************************************
 * @description
 *      Firmware handle structure.
 *
 *****************************************************************************/
typedef struct adf_fw_loader_handle_s {
   icp_firml_sys_mem_info_t *sysMemInfo;
   void*                     firmLoaderHandle;
} adf_fw_loader_handle_t;

/*
 * adf_accelEngineFwLoad
 *
 * Description
 *  Wrapper functions which loads the fw, gets the address of the UOF fw and
 *  patches the fw
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeFwLoad(icp_accel_dev_t *pAccelDev);

/*
 * adf_aeInit
 *
 * Description
 *  Function to initialize the AccelEngines
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeInit(icp_accel_dev_t *pAccelDev);

/*
 * adf_aeShutdown
 *
 * Description
 *   Shutdown the AccelEngines
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeShutdown(icp_accel_dev_t *pAccelDev);

/*
 * adf_aeStart
 *
 * Description
 *   Function to start the AccelEngine.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */

CpaStatus adf_aeStart(icp_accel_dev_t *pAccelDev);

/*
 * adf_aeStop
 *
 * Description
 *   Function to stop the AccelEngine
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeStop(icp_accel_dev_t *pAccelDev);

/*
 * adf_aeReset
 *
 * Description
 *   Function to reset the specified AccelEngines and conditionally clear the
 *   registers to their power-up state. After this is done
 *   the specified AccelEngines are taken out of reset and inited.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeReset(icp_accel_dev_t *pAccelDev,
                      Cpa32S aeNum,
                      Cpa32S aeClrRegs);

/*
 * adf_aeUcodeMap
 *
 * Description
 *   Maps the Uclo Object File (UOF) objects from the memory location
 *   described by the 'addr' and 'size' input parameters.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeUcodeMap(icp_accel_dev_t *pAccelDev,
                         void* addr,
                         Cpa32U size);

/*
 * adf_aePatchSymbol
 *
 * Description
 * Binds a value to the Ucode symbol in the UOF image.
 *
 * CPA_STATUS_SUCCESS   on success
 * CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aePatchSymbol(icp_accel_dev_t *pAccelDev,
                            Cpa32S aeNum,
                            char* pSymName,
                            Cpa32S symValue);

/*
 * adf_aeStoreSymbols
 *
 * Description
 * Add the symbols into the symbol_info member of the
 * config data structure. The symbols are stored in memory
 * in case a suspend and resume are initiated a some later point.
 * The symbols can in that case be retrieved from the disk.
 *
 * CPA_STATUS_SUCCESS   on success
 * CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeStoreSymbols(icp_accel_dev_t *pAccelDev,
                             Cpa32S aeNum,
                             char* pSymName,
                             Cpa32S symValue);
/*
 * adf_aeUcodeDownload
 *
 * Description
 *   Writes all of the UOF images to the assigned uEngines
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeUcodeDownload(icp_accel_dev_t *pAccelDev);

/*
 * adf_aeUcodeRelease
 *
 * Description
 *   Removes and frees all resource references associated with the handle.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeUcodeRelease(icp_accel_dev_t *pAccelDev);

/*
 * adf_aeReleaseFirmware
 *
 * Description
 *   Removes and frees the Firmware memory.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeReleaseFirmware(icp_accel_dev_t *pAccelDev);


/*
 * adf_aeRemoveSymbols
 *
 * Description
 * Removes the Symbols into the
 * adf_cfg_device_data_t structure
 *
 * Returns
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus adf_aeRemoveSymbols(icp_accel_dev_t *pAccelDev);

/*
 * adf_aefwGetVersion
 *
 * Description
 * Extracts firmware and tools version and adds it to config.
 *
 * Returns
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 *
 */
CpaStatus adf_aefwGetVersion(icp_accel_dev_t *pAccelDev);

#endif /* ADF_AE_H */
