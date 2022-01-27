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
 * @file adf_bh.c
 *
 * @description
 *    This file contains ADF code for setting up ISR BH handlers
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/interrupt.h>

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "adf_platform.h"
#include "adf_drv_sriov.h"


/*
 * adf_setup_vf_bh
 * Setup & Initialise the tasklet structs for virtual function
 */
int adf_setup_vf_bh(icp_accel_pci_vf_info_t *vf)
{
        if(NULL == vf) {
                ADF_ERROR("adf_setup_vf_bh invalid param\n");
                return FAIL;
        }
        ADF_DEBUG("Setup bh tasklet for vf%d \n", vf->deviceId);
        vf->tasklet_vf = ICP_ZALLOC_GEN(sizeof(struct tasklet_struct));
        if (!vf->tasklet_vf) {
                ADF_ERROR("Failed to allocate ETR Bank tasklet\n");
                return FAIL;
        }
        tasklet_init(vf->tasklet_vf,
                     (void*)adf_vf_bh_handler,
                     (unsigned long)vf);

        return SUCCESS;
}

/*
 * adf_cleanup_vf_bh
 * Cleanup Tasklet structures for virtual function
 */
int adf_cleanup_vf_bh(icp_accel_pci_vf_info_t *vf)
{
        /*Cleanup the VF tasklets */
        if (vf->tasklet_vf != NULL) {
                ADF_DEBUG("Cleanup bh tasklet for vf%d \n", vf->deviceId);
                tasklet_disable(vf->tasklet_vf);
                tasklet_kill(vf->tasklet_vf);
                ICP_FREE(vf->tasklet_vf);
                vf->tasklet_vf = NULL;
        }
        return SUCCESS;
}
