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
 * @file icp_qa_module.c
 *
 * This file contains the init and shutdown functions for the entire lookaside
 * acceleration driver. SAL, ADF and OSAL subcomponent init and exit functions
 * are called from here.
 *
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/crypto.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28))
#include <crypto/hash.h>
#include <crypto/sha.h>
#endif   /* End of check for KERNEL_VERSION(2,6,28) */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34))
#include <crypto/md5.h>
#endif   /* End of check for KERNEL_VERSION(2,6,34) */

#include "cpa.h"
#include "lac_sal_ctrl.h"
#include "lac_common.h"

#include "icp_sal_versions.h"
#include "cpa_cy_sym_dp.h"
#include "cpa_cy_drbg.h"
#include "icp_sal_drbg_impl.h"
#include "icp_sal_drbg_ht.h"
#include "cpa_cy_ecdsa.h"
#include "cpa_cy_im.h"
#include "cpa_cy_nrbg.h"
#include "cpa_cy_dh.h"
#include "cpa_dc.h"
#include "cpa_cy_key.h"
#include "cpa_cy_dsa.h"
#include "cpa_cy_ecdh.h"
#include "cpa_cy_ln.h"
#include "cpa_cy_rsa.h"
#include "cpa_dc_dp.h"
#include "cpa_cy_prime.h"

#ifdef WITH_CPA_MUX
#include "cpa_impl_mux.h"
static void *cpa_mux_handle;
extern CpaFuncPtrs cpaMuxFuncPtrs;
#endif

#define SUCCESS 0
#define FAIL 1

MODULE_DESCRIPTION("ICP Look Aside Acceleration driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");

extern int  __init osal_init(char* path);
extern void osal_exit(void);
extern int __init adf_init(void);
extern void adf_exit(void);
extern int lacSymDrbgLock_init(void);
extern void lacSymDrbgLock_exit(void);

char *icp_module_name = "icp_qa_al";

/*
 * Module inc/dec usage functions
 */
inline int icp_qa_get_module(void)
{
    return try_module_get(THIS_MODULE);
}

inline void icp_qa_put_module(void)
{
    module_put(THIS_MODULE);
}

static int __init QaModInit(void)
{
    int ret = -1;

    if(OSAL_SUCCESS != lacSymDrbgLock_init())
    {
        goto failed;
    }

    if(SUCCESS != osal_init(NULL))
    {
        printk("Error initialising osal\n");
        goto failed;
    }

    if(SUCCESS != adf_init())
    {
        printk("Error initialising adf\n");
        osal_exit();
        goto failed;
    }

#ifdef WITH_CPA_MUX
    if (SUCCESS != cpaMuxRegisterImpl(CPA_MUX_DRIVER_TYPE_QAT_1_5,
                                      NULL, &cpaMuxFuncPtrs,
                                      &cpa_mux_handle))
    {
        printk("Error registering with CpaMux\n");
        adf_exit(); 
        osal_exit();
        goto failed;
    }
#endif

    icpSetProcessName(LAC_KERNEL_PROCESS_NAME);
    /* MUST register QAT first as qat instances need to be started before
       service instances */
#ifndef ACCELDEVVF
    SalCtrl_AdfQatRegister();
#endif
    SalCtrl_AdfServicesRegister();
    printk("Loading SAL Module ...\n");
    return 0;

failed:
#ifdef WITH_CPA_MUX
    if(cpa_mux_handle)
        cpaMuxDeRegisterImpl(cpa_mux_handle);
#endif
    return ret;
}


static void __exit QaModExit(void)
{
#ifndef ACCELDEVVF
    SalCtrl_AdfQatUnregister();
#endif
    SalCtrl_AdfServicesUnregister();
    lacSymDrbgLock_exit();
#ifdef WITH_CPA_MUX
    if(cpa_mux_handle)
        cpaMuxDeRegisterImpl(cpa_mux_handle);
#endif
    adf_exit();
    osal_exit();
    printk("Unloading SAL Module ...\n");
}

module_init(QaModInit);
module_exit(QaModExit);
