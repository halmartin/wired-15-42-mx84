/******************************************************************************
 *
 * GPL LICENSE SUMMARY
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
 *  version: QAT1.5.L.1.11.0-36
 *
 *****************************************************************************/


/**
 ******************************************************************************
 * @file  cpa_ssl_sample_linux_kernel_module.c
 *
 *****************************************************************************/

#include "cpa_sample_utils.h"


int gDebugParam = 1;
module_param(gDebugParam, int, S_IRUGO);

extern CpaStatus
algChainSample(void);

static int
sslInit(void)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;

    PRINT_DBG("Loading SSL Sample Code Module ...\n" ) ;

    stat = algChainSample();
    if(CPA_STATUS_SUCCESS != stat)
    {
       PRINT_ERR("\nSSL Sample Code App failed\n");
    }
    else
    {
       PRINT_DBG("\nSSL Sample Code App finished\n");
    }

    PRINT_DBG("\nRemoving Module - ignore insmod error\n");

    /* module does not have any runtime functionality so remove it */
    return -EAGAIN;
}

static void
sslExit(void)
{
    PRINT_DBG("Unloading SSL Sample Code Module ...\n" ) ;
}

module_init(sslInit);
module_exit(sslExit);

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SSL Sample Code");
