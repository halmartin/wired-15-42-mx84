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
 * @file adf_acceldev_bank_bh_handlers.c
 *
 * @description
 *      File contains platform specific bh handlers
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_platform.h"

/*
 * bank 0 handler
 */
void adf_bank0_handler(void* handle)
{
    adf_ringResponseHandler(handle, 0);
}

/*
 * bank 1 handler
 */
void adf_bank1_handler(void* handle)
{
    adf_ringResponseHandler(handle, 1);
}

/*
 * bank 2 handler
 */
void adf_bank2_handler(void* handle)
{
    adf_ringResponseHandler(handle, 2);
}

/*
 * bank 3 handler
 */
void adf_bank3_handler(void* handle)
{
    adf_ringResponseHandler(handle, 3);
}

/*
 * bank 4 handler
 */
void adf_bank4_handler(void* handle)
{
    adf_ringResponseHandler(handle, 4);
}

/*
 * bank 5 handler
 */
void adf_bank5_handler(void* handle)
{
    adf_ringResponseHandler(handle, 5);
}

/*
 * bank 6 handler
 */
void adf_bank6_handler(void* handle)
{
    adf_ringResponseHandler(handle, 6);
}

/*
 * bank 7 handler
 */
void adf_bank7_handler(void* handle)
{
    adf_ringResponseHandler(handle, 7);
}

/*
 * bank 8 handler
 */
void adf_bank8_handler(void* handle)
{
    adf_ringResponseHandler(handle, 8);
}

/*
 * bank 9 handler
 */
void adf_bank9_handler(void* handle)
{
    adf_ringResponseHandler(handle, 9);
}

/*
 * bank 10 handler
 */
void adf_bank10_handler(void* handle)
{
    adf_ringResponseHandler(handle, 10);
}

/*
 * bank 11 handler
 */
void adf_bank11_handler(void* handle)
{
    adf_ringResponseHandler(handle, 11);
}

/*
 * bank 12 handler
 */
void adf_bank12_handler(void* handle)
{
    adf_ringResponseHandler(handle, 12);
}

/*
 * bank 13 handler
 */
void adf_bank13_handler(void* handle)
{
    adf_ringResponseHandler(handle, 13);
}

/*
 * bank 14 handler
 */
void adf_bank14_handler(void* handle)
{
    adf_ringResponseHandler(handle, 14);
}

/*
 * bank 15 handler
 */
void adf_bank15_handler(void* handle)
{
    adf_ringResponseHandler(handle, 15);
}









/* Set the Response handlers for polling - one for each bank. */
/*
 * bank 0 handler for polling
 */
void adf_bank0_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 0);
}

/*
 * bank 1 handler for polling
 */
void adf_bank1_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 1);
}

/*
 * bank 2 handler for polling
 */
void adf_bank2_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 2);
}

/*
 * bank 3 handler for polling
 */
void adf_bank3_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 3);
}

/*
 * bank 4 handler for polling
 */
void adf_bank4_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 4);
}

/*
 * bank 5 handler for polling
 */
void adf_bank5_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 5);
}

/*
 * bank 6 handler for polling
 */
void adf_bank6_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 6);
}

/*
 * bank 7 handler for polling
 */
void adf_bank7_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 7);
}

/*
 * bank 8 handler for polling
 */
void adf_bank8_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 8);
}

/*
 * bank 9 handler for polling
 */
void adf_bank9_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 9);
}

/*
 * bank 10 handler for polling
 */
void adf_bank10_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 10);
}

/*
 * bank 11 handler for polling
 */
void adf_bank11_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 11);
}

/*
 * bank 12 handler for polling
 */
void adf_bank12_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 12);
}

/*
 * bank 13 handler for polling
 */
void adf_bank13_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 13);
}

/*
 * bank 14 handler for polling
 */
void adf_bank14_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 14);
}

/*
 * bank 15 handler for polling
 */
void adf_bank15_polling_handler(void* handle)
{
    adf_bankResponsePolling(handle, 15);
}

