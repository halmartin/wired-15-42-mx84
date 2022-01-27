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

/**
*****************************************************************************
 * @file cpa_sample_code_linux_module.c
 *
 * This file contains the init and shutdown functions for the performance
 * sample code Module.
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>

#include "cpa.h"
#include "cpa_sample_code_utils_common.h"
#include "cpa_sample_code_framework.h"
#include "module_exports.h"
#include <linux/device.h>
#include <linux/firmware.h>

#ifdef INCLUDE_COMPRESSION
extern struct device perf_device;
#endif

extern int main(void);

int runTests = 127;
int signOfLife=0;
int cyNumBuffers=20;
int cyAsymLoops=5000;
int cySymLoops=5000;
int dcLoops=100;
int wirelessFirmware=0;
int runStateful=0;
int verboseOutput=0;


module_param(runTests, int, 0);
module_param(cyNumBuffers, int, 0);
module_param(cyAsymLoops, int, 0);
module_param(cySymLoops, int, 0);
module_param(dcLoops, int, 0);
module_param(signOfLife, int, 0);
module_param(wirelessFirmware, int, 0);
module_param(runStateful, int, 0);
module_param(verboseOutput, int, 0);





/*****************************************************************************/

static CpaStatus sampleCodeInit(void)
{
    PRINT("Loaded Sample code module\n\n\n");
#ifdef INCLUDE_COMPRESSION
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
    dev_set_name(&perf_device, "perf_device");
#endif
    if(device_register(&perf_device) != 0)
    {
        PRINT("The perf_device registration failed\n");
        return CPA_STATUS_FAIL;
    }
#endif
    if(runTests)
    {
        main();
        PRINT("Sample Code Complete\n");
    }

    return CPA_STATUS_SUCCESS;
}

/*****************************************************************************/

static void sampleCodeExit(void)
{
#ifdef INCLUDE_COMPRESSION
    device_unregister(&perf_device);
#endif
    PRINT("Unloading Sample code module\n");
}

module_init(sampleCodeInit);
module_exit(sampleCodeExit);

EXPORT_SYMBOL(cyNumBuffers);
EXPORT_SYMBOL(cyAsymLoops);
EXPORT_SYMBOL(cySymLoops);
EXPORT_SYMBOL(dcLoops);
EXPORT_SYMBOL(runTests);
EXPORT_SYMBOL(wirelessFirmware);
EXPORT_SYMBOL(signOfLife);
EXPORT_SYMBOL(verboseOutput);

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Sample Code");
