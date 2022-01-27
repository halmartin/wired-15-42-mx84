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
 * @file  adf_wireless_main.c
 *
 * @description
 *      This file contains the function definitions for wireless control
 *      path layer.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_adf_init.h"
#include "icp_adf_cfg.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "adf_init.h"
#include "adf_wireless_events.h"
#include "adf_transport_ctrl.h"
#include "adf_wireless.h"
#include "adf_platform.h"


STATIC subservice_registation_handle_t wireless_reg_handle;
STATIC char* subsystem_name = WIRELESS_SUBSYSTEM_NAME;

/*
 * wireless_EventHandler - User proxy event dispatcher
 * Call appropriate event handler function in wireless control plane
 */
STATIC CpaStatus wireless_EventHandler( icp_accel_dev_t *accel_dev,
                              icp_adf_subsystemEvent_t event, void* param )
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    switch( event )
    {
        case ICP_ADF_EVENT_INIT :
            stat = adf_wirelessEventInit( accel_dev );
            break;
        case ICP_ADF_EVENT_START :
            stat = CPA_STATUS_SUCCESS;
            break;
        case ICP_ADF_EVENT_STOP :
            stat = CPA_STATUS_SUCCESS;
            break;
        case ICP_ADF_EVENT_SHUTDOWN :
            stat = adf_wirelessEventShutdown( accel_dev );
            break;
        default:
            stat = CPA_STATUS_SUCCESS;
    }
    return stat;
}

/*
 * adf_wirelessRegister
 * Register wireless event dispatcher into ADF
 */
CpaStatus adf_wirelessRegister( void )
{
    wireless_reg_handle.subsystem_name  = subsystem_name;
    wireless_reg_handle.subserviceEventHandler = wireless_EventHandler;
    return icp_adf_subsystemRegister( &wireless_reg_handle );
}

/*
 * adf_wirelessUnregister
 * Unregister wireless event dispatcher from ADF
 */
CpaStatus adf_wirelessUnregister( void )
{
    return icp_adf_subsystemUnregister( &wireless_reg_handle );
}
