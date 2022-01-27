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
 * @file adf_lib.c
 *
 * @description
 *      This file contains the adf lib initialization and release
 *      functions.
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_init.h"
#include "icp_firml_interface.h"
#include "icp_adf_transport.h"
#include "icp_adf_ae.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_esram.h"
#include "icp_adf_poll.h"
#include "adf_drv.h"
#include "adf_cfg.h"
#include "adf_init.h"
#include "adf_ae.h"
#include "adf_ae_fw.h"
#include "adf_dram.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_cfg.h"
#include "adf_devmgr.h"
#include "adf_user_proxy.h"
#include "adf_wireless.h"
#include "adf_dyn.h"

extern int  icp_qa_get_module(void);
extern void icp_qa_put_module(void);

#ifndef ADF_PLATFORM_ACCELDEVVF
extern int adf_init_sriov(void);
extern void adf_exit_sriov(void);
#endif

/*
 * Module inc/dec usage functions
 */
inline int adf_get_module(void)
{
        return icp_qa_get_module();
}

inline void adf_put_module(void)
{
        icp_qa_put_module();
}

/*
 * adf_init
 * Adf lib init function
 */
int __init adf_init(void)
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
        return status;
}

/*
 * adfdrv_release
 * Adf lib exit function
 */
void adf_exit(void)
{
        if (SUCCESS != adf_dynUnregister()) {
                ADF_ERROR("Failed to unregister dyn proxy.\n");
        }
        if (SUCCESS != adf_wirelessUnregister()) {
                ADF_ERROR("Failed to unregister wireless proxy.\n");
        }
        if (SUCCESS != adf_userSpaceUnregister()) {
                ADF_ERROR("Failed to unregister user space proxy.\n");
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
