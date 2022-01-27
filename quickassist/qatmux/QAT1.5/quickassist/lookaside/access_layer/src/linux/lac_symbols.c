/***************************************************************************
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
 ***************************************************************************/

/**
*****************************************************************************
 * @file lac_symbols.c
 *
 * This file contains all the symbols that are exported by the Look Aside
 * kernel Module.
 *
 *****************************************************************************/

#include <linux/module.h>
#include "cpa.h"
#include "cpa_cy_sym.h"
#include "cpa_cy_common.h"
#include "cpa_cy_dh.h"
#include "cpa_cy_key.h"
#include "cpa_cy_prime.h"
#include "cpa_cy_dsa.h"
#include "cpa_cy_rsa.h"
#include "cpa_cy_ln.h"
#include "cpa_cy_im.h"
#include "cpa_cy_ec.h"
#include "cpa_cy_ecdh.h"
#include "cpa_cy_ecdsa.h"
#include "cpa_dc.h"
#include "cpa_dc_dp.h"
#include "cpa_cy_drbg.h"
#include "cpa_cy_nrbg.h"
#include "cpa_cy_sym_dp.h"
#include "icp_sal_drbg_impl.h"
#include "icp_sal_drbg_ht.h"
#include "icp_sal_nrbg_ht.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_poll.h"
#include "icp_sal_poll.h"
#include "icp_sal_iommu.h"
#include "icp_sal_versions.h"

#ifndef WITH_CPA_MUX
#define PUBLIC_SYMBOL(fn)      EXPORT_SYMBOL(fn);
#else
#include "icp_adf_accel_mgr.h"
#include "cpa_impl_mux.h"

#define PUBLIC_SYMBOL(fn)      fn : fn ,

CpaFuncPtrs cpaMuxFuncPtrs = {

#endif

/* Symbols for getting version information */
PUBLIC_SYMBOL(icp_sal_getDevVersionInfo)

/* Symbols for Symmetric Data Plane API */

PUBLIC_SYMBOL(cpaCySymDpEnqueueOp)
PUBLIC_SYMBOL(cpaCySymDpEnqueueOpBatch)
PUBLIC_SYMBOL(cpaCySymDpRegCbFunc)
PUBLIC_SYMBOL(cpaCySymDpSessionCtxGetSize)
PUBLIC_SYMBOL(cpaCySymDpSessionCtxGetDynamicSize)
PUBLIC_SYMBOL(cpaCySymDpInitSession)
PUBLIC_SYMBOL(cpaCySymDpRemoveSession)
PUBLIC_SYMBOL(cpaCySymDpPerformOpNow)

/* Symbols for Symmetric */
PUBLIC_SYMBOL(cpaCySymInitSession)
PUBLIC_SYMBOL(cpaCySymRemoveSession)
PUBLIC_SYMBOL(cpaCySymPerformOp)
PUBLIC_SYMBOL(cpaCySymQueryStats)
PUBLIC_SYMBOL(cpaCySymQueryStats64)
PUBLIC_SYMBOL(cpaCySymQueryCapabilities)
PUBLIC_SYMBOL(cpaCySymSessionCtxGetSize)
PUBLIC_SYMBOL(cpaCySymSessionCtxGetDynamicSize)

/* Symbols for DRBG */
PUBLIC_SYMBOL(cpaCyDrbgInitSession)
PUBLIC_SYMBOL(cpaCyDrbgReseed)
PUBLIC_SYMBOL(cpaCyDrbgGen)
PUBLIC_SYMBOL(cpaCyDrbgSessionGetSize)
PUBLIC_SYMBOL(cpaCyDrbgRemoveSession)
PUBLIC_SYMBOL(cpaCyDrbgQueryStats64)

PUBLIC_SYMBOL(icp_sal_drbgGetEntropyInputFuncRegister)
PUBLIC_SYMBOL(icp_sal_drbgGetNonceFuncRegister)
PUBLIC_SYMBOL(icp_sal_drbgIsDFReqFuncRegister)
PUBLIC_SYMBOL(icp_sal_drbgGetInstance)

PUBLIC_SYMBOL(icp_sal_drbgHTGetTestSessionSize)
PUBLIC_SYMBOL(icp_sal_drbgHTInstantiate)
PUBLIC_SYMBOL(icp_sal_drbgHTGenerate)
PUBLIC_SYMBOL(icp_sal_drbgHTReseed)

/* Symbols for NRBG */
PUBLIC_SYMBOL(cpaCyNrbgGetEntropy)
PUBLIC_SYMBOL(icp_sal_nrbgHealthTest)

/* Diffie Hellman */
PUBLIC_SYMBOL(cpaCyDhKeyGenPhase1)
PUBLIC_SYMBOL(cpaCyDhKeyGenPhase2Secret)
PUBLIC_SYMBOL(cpaCyDhQueryStats)
PUBLIC_SYMBOL(cpaCyDhQueryStats64)

/*Key Expansion and Generation*/
PUBLIC_SYMBOL(cpaCyKeyGenSsl)
PUBLIC_SYMBOL(cpaCyKeyGenTls)
PUBLIC_SYMBOL(cpaCyKeyGenTls2)
PUBLIC_SYMBOL(cpaCyKeyGenMgf)
PUBLIC_SYMBOL(cpaCyKeyGenMgfExt)
PUBLIC_SYMBOL(cpaCyKeyGenQueryStats)
PUBLIC_SYMBOL(cpaCyKeyGenQueryStats64)

/* Large Number ModExp and ModInv */
PUBLIC_SYMBOL(cpaCyLnModExp)
PUBLIC_SYMBOL(cpaCyLnModInv)
PUBLIC_SYMBOL(cpaCyLnStatsQuery)
PUBLIC_SYMBOL(cpaCyLnStatsQuery64)

/* Prime */
PUBLIC_SYMBOL(cpaCyPrimeTest)
PUBLIC_SYMBOL(cpaCyPrimeQueryStats)
PUBLIC_SYMBOL(cpaCyPrimeQueryStats64)

/* DSA */
PUBLIC_SYMBOL(cpaCyDsaGenPParam)
PUBLIC_SYMBOL(cpaCyDsaGenGParam)
PUBLIC_SYMBOL(cpaCyDsaGenYParam)
PUBLIC_SYMBOL(cpaCyDsaSignR)
PUBLIC_SYMBOL(cpaCyDsaSignS)
PUBLIC_SYMBOL(cpaCyDsaSignRS)
PUBLIC_SYMBOL(cpaCyDsaVerify)
PUBLIC_SYMBOL(cpaCyDsaQueryStats)
PUBLIC_SYMBOL(cpaCyDsaQueryStats64)

/* RSA */
PUBLIC_SYMBOL(cpaCyRsaGenKey)
PUBLIC_SYMBOL(cpaCyRsaEncrypt)
PUBLIC_SYMBOL(cpaCyRsaDecrypt)
PUBLIC_SYMBOL(cpaCyRsaQueryStats)
PUBLIC_SYMBOL(cpaCyRsaQueryStats64)

/* EC */
PUBLIC_SYMBOL(cpaCyEcPointMultiply)
PUBLIC_SYMBOL(cpaCyEcPointVerify)
PUBLIC_SYMBOL(cpaCyEcQueryStats64)

/* ECDH */
PUBLIC_SYMBOL(cpaCyEcdhPointMultiply)
PUBLIC_SYMBOL(cpaCyEcdhQueryStats64)

/* ECDSA */
PUBLIC_SYMBOL(cpaCyEcdsaSignR)
PUBLIC_SYMBOL(cpaCyEcdsaSignS)
PUBLIC_SYMBOL(cpaCyEcdsaSignRS)
PUBLIC_SYMBOL(cpaCyEcdsaVerify)
PUBLIC_SYMBOL(cpaCyEcdsaQueryStats64)

/* DC Compression */
PUBLIC_SYMBOL(cpaDcGetNumIntermediateBuffers)
PUBLIC_SYMBOL(cpaDcInitSession)
PUBLIC_SYMBOL(cpaDcRemoveSession)
PUBLIC_SYMBOL(cpaDcResetSession)
PUBLIC_SYMBOL(cpaDcCompressData)
PUBLIC_SYMBOL(cpaDcDecompressData)
PUBLIC_SYMBOL(cpaDcGenerateHeader)
PUBLIC_SYMBOL(cpaDcGenerateFooter)
PUBLIC_SYMBOL(cpaDcGetStats)
PUBLIC_SYMBOL(cpaDcGetInstances)
PUBLIC_SYMBOL(cpaDcGetNumInstances)
PUBLIC_SYMBOL(cpaDcGetSessionSize)
PUBLIC_SYMBOL(cpaDcGetStatusText)
PUBLIC_SYMBOL(cpaDcBufferListGetMetaSize)
PUBLIC_SYMBOL(cpaDcInstanceGetInfo2)
PUBLIC_SYMBOL(cpaDcQueryCapabilities)
PUBLIC_SYMBOL(cpaDcSetAddressTranslation)
PUBLIC_SYMBOL(cpaDcInstanceSetNotificationCb)
PUBLIC_SYMBOL(cpaDcStartInstance)
PUBLIC_SYMBOL(cpaDcStopInstance)

/* DcDp Compression */
PUBLIC_SYMBOL(cpaDcDpGetSessionSize)
PUBLIC_SYMBOL(cpaDcDpInitSession)
PUBLIC_SYMBOL(cpaDcDpRemoveSession)
PUBLIC_SYMBOL(cpaDcDpRegCbFunc)
PUBLIC_SYMBOL(cpaDcDpEnqueueOp)
PUBLIC_SYMBOL(cpaDcDpEnqueueOpBatch)
PUBLIC_SYMBOL(cpaDcDpPerformOpNow)

/* Symbols for all of LAC */
PUBLIC_SYMBOL(cpaCyBufferListGetMetaSize)
PUBLIC_SYMBOL(cpaCyGetInstances)
PUBLIC_SYMBOL(cpaCyGetNumInstances)
PUBLIC_SYMBOL(cpaCyGetStatusText)
PUBLIC_SYMBOL(cpaCyInstanceGetInfo)
PUBLIC_SYMBOL(cpaCyInstanceGetInfo2)
PUBLIC_SYMBOL(cpaCyInstanceSetNotificationCb)
PUBLIC_SYMBOL(cpaCySetAddressTranslation)
PUBLIC_SYMBOL(cpaCyStartInstance)
PUBLIC_SYMBOL(cpaCyStopInstance)
PUBLIC_SYMBOL(cpaCyQueryCapabilities)

/* Polling symbols */
PUBLIC_SYMBOL(icp_sal_CyPollInstance)
PUBLIC_SYMBOL(icp_sal_CyPollDpInstance)
PUBLIC_SYMBOL(icp_sal_DcPollInstance)
PUBLIC_SYMBOL(icp_sal_DcPollDpInstance)

/* Iommu symbols */
PUBLIC_SYMBOL(icp_sal_iommu_get_remap_size)
PUBLIC_SYMBOL(icp_sal_iommu_map)
PUBLIC_SYMBOL(icp_sal_iommu_unmap)

#ifdef ONE_KO_RELEASE_PACKAGE
PUBLIC_SYMBOL(icp_sal_pollBank)
PUBLIC_SYMBOL(icp_sal_pollAllBanks)
#endif

#ifdef WITH_CPA_MUX
PUBLIC_SYMBOL(icp_amgr_getNumInstances)
};
#endif

