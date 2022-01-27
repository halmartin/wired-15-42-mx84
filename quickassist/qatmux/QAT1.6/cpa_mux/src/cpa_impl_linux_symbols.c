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
 *  version: QAT1.6.L.2.6.0-65
 *
 ***************************************************************************/

/**
*****************************************************************************
 * @file cpa_impl_linux_symbols.c
 *
 * This file contains all the symbols that are exported by the CPA Mux
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
#include "icp_sal_user.h"
#include "icp_sal_versions.h"
#include "icp_sal_iommu.h"

#include "cpa_impl_mux.h"

/* Symbols for getting version information */
EXPORT_SYMBOL(icp_sal_getDevVersionInfo);

/* Symbols for Symmetric Data Plane API */

EXPORT_SYMBOL(cpaCySymDpEnqueueOp);
EXPORT_SYMBOL(cpaCySymDpEnqueueOpBatch);
EXPORT_SYMBOL(cpaCySymDpRegCbFunc);
EXPORT_SYMBOL(cpaCySymDpSessionCtxGetSize);
EXPORT_SYMBOL(cpaCySymDpSessionCtxGetDynamicSize);
EXPORT_SYMBOL(cpaCySymDpInitSession);
EXPORT_SYMBOL(cpaCySymDpRemoveSession);
EXPORT_SYMBOL(cpaCySymDpPerformOpNow);

/* Symbols for Symmetric */
EXPORT_SYMBOL(cpaCySymInitSession);
EXPORT_SYMBOL(cpaCySymRemoveSession);
EXPORT_SYMBOL(cpaCySymPerformOp);
EXPORT_SYMBOL(cpaCySymQueryStats);
EXPORT_SYMBOL(cpaCySymQueryStats64);
EXPORT_SYMBOL(cpaCySymQueryCapabilities);
EXPORT_SYMBOL(cpaCySymSessionCtxGetSize);
EXPORT_SYMBOL(cpaCySymSessionCtxGetDynamicSize);

/* Symbols for DRBG */
EXPORT_SYMBOL(cpaCyDrbgInitSession);
EXPORT_SYMBOL(cpaCyDrbgReseed);
EXPORT_SYMBOL(cpaCyDrbgGen);
EXPORT_SYMBOL(cpaCyDrbgSessionGetSize);
EXPORT_SYMBOL(cpaCyDrbgRemoveSession);
EXPORT_SYMBOL(cpaCyDrbgQueryStats64);

EXPORT_SYMBOL(icp_sal_drbgGetEntropyInputFuncRegister);
EXPORT_SYMBOL(icp_sal_drbgGetNonceFuncRegister);
EXPORT_SYMBOL(icp_sal_drbgIsDFReqFuncRegister);
EXPORT_SYMBOL(icp_sal_drbgGetInstance);

EXPORT_SYMBOL(icp_sal_drbgHTGetTestSessionSize);
EXPORT_SYMBOL(icp_sal_drbgHTInstantiate);
EXPORT_SYMBOL(icp_sal_drbgHTGenerate);
EXPORT_SYMBOL(icp_sal_drbgHTReseed);

/* Symbols for NRBG */
EXPORT_SYMBOL(cpaCyNrbgGetEntropy);
EXPORT_SYMBOL(icp_sal_nrbgHealthTest);

/* Diffie Hellman */
EXPORT_SYMBOL(cpaCyDhKeyGenPhase1);
EXPORT_SYMBOL(cpaCyDhKeyGenPhase2Secret);
EXPORT_SYMBOL(cpaCyDhQueryStats);
EXPORT_SYMBOL(cpaCyDhQueryStats64);

/*Key Expansion and Generation*/
EXPORT_SYMBOL(cpaCyKeyGenSsl);
EXPORT_SYMBOL(cpaCyKeyGenTls);
EXPORT_SYMBOL(cpaCyKeyGenTls2);
EXPORT_SYMBOL(cpaCyKeyGenMgf);
EXPORT_SYMBOL(cpaCyKeyGenMgfExt);
EXPORT_SYMBOL(cpaCyKeyGenQueryStats);
EXPORT_SYMBOL(cpaCyKeyGenQueryStats64);

/* Large Number ModExp and ModInv */
EXPORT_SYMBOL(cpaCyLnModExp);
EXPORT_SYMBOL(cpaCyLnModInv);
EXPORT_SYMBOL(cpaCyLnStatsQuery);
EXPORT_SYMBOL(cpaCyLnStatsQuery64);

/* Prime */
EXPORT_SYMBOL(cpaCyPrimeTest);
EXPORT_SYMBOL(cpaCyPrimeQueryStats);
EXPORT_SYMBOL(cpaCyPrimeQueryStats64);

/* DSA */
EXPORT_SYMBOL(cpaCyDsaGenPParam);
EXPORT_SYMBOL(cpaCyDsaGenGParam);
EXPORT_SYMBOL(cpaCyDsaGenYParam);
EXPORT_SYMBOL(cpaCyDsaSignR);
EXPORT_SYMBOL(cpaCyDsaSignS);
EXPORT_SYMBOL(cpaCyDsaSignRS);
EXPORT_SYMBOL(cpaCyDsaVerify);
EXPORT_SYMBOL(cpaCyDsaQueryStats);
EXPORT_SYMBOL(cpaCyDsaQueryStats64);

/* RSA */
EXPORT_SYMBOL(cpaCyRsaGenKey);
EXPORT_SYMBOL(cpaCyRsaEncrypt);
EXPORT_SYMBOL(cpaCyRsaDecrypt);
EXPORT_SYMBOL(cpaCyRsaQueryStats);
EXPORT_SYMBOL(cpaCyRsaQueryStats64);

/* EC */
EXPORT_SYMBOL(cpaCyEcPointMultiply);
EXPORT_SYMBOL(cpaCyEcPointVerify);
EXPORT_SYMBOL(cpaCyEcQueryStats64);

/* ECDH */
EXPORT_SYMBOL(cpaCyEcdhPointMultiply);
EXPORT_SYMBOL(cpaCyEcdhQueryStats64);

/* ECDSA */
EXPORT_SYMBOL(cpaCyEcdsaSignR);
EXPORT_SYMBOL(cpaCyEcdsaSignS);
EXPORT_SYMBOL(cpaCyEcdsaSignRS);
EXPORT_SYMBOL(cpaCyEcdsaVerify);
EXPORT_SYMBOL(cpaCyEcdsaQueryStats64);

/* DC Compression */
EXPORT_SYMBOL(cpaDcInitSession);
EXPORT_SYMBOL(cpaDcRemoveSession);
EXPORT_SYMBOL(cpaDcResetSession);
EXPORT_SYMBOL(cpaDcCompressData);
EXPORT_SYMBOL(cpaDcDecompressData);
EXPORT_SYMBOL(cpaDcGenerateHeader);
EXPORT_SYMBOL(cpaDcGenerateFooter);
EXPORT_SYMBOL(cpaDcGetStats);

/* DcDp Compression */
EXPORT_SYMBOL(cpaDcDpGetSessionSize);
EXPORT_SYMBOL(cpaDcDpInitSession);
EXPORT_SYMBOL(cpaDcDpRemoveSession);
EXPORT_SYMBOL(cpaDcDpRegCbFunc);
EXPORT_SYMBOL(cpaDcDpEnqueueOp);
EXPORT_SYMBOL(cpaDcDpEnqueueOpBatch);
EXPORT_SYMBOL(cpaDcDpPerformOpNow);

/* Symbols for all of LAC */
EXPORT_SYMBOL(cpaCyBufferListGetMetaSize);

EXPORT_SYMBOL(cpaCyGetInstances);
EXPORT_SYMBOL(cpaCyGetNumInstances);
EXPORT_SYMBOL(cpaCyGetStatusText);
EXPORT_SYMBOL(cpaCyInstanceGetInfo);
EXPORT_SYMBOL(cpaCyInstanceGetInfo2);
EXPORT_SYMBOL(cpaCyInstanceSetNotificationCb);
EXPORT_SYMBOL(cpaCySetAddressTranslation);
EXPORT_SYMBOL(cpaCyStartInstance);
EXPORT_SYMBOL(cpaCyStopInstance);

EXPORT_SYMBOL(cpaCyQueryCapabilities);

/* DC Compression */
EXPORT_SYMBOL(cpaDcGetInstances);
EXPORT_SYMBOL(cpaDcGetNumInstances);
EXPORT_SYMBOL(cpaDcGetNumIntermediateBuffers);
EXPORT_SYMBOL(cpaDcGetSessionSize);
EXPORT_SYMBOL(cpaDcGetStatusText);
EXPORT_SYMBOL(cpaDcBufferListGetMetaSize);
EXPORT_SYMBOL(cpaDcInstanceGetInfo2);
EXPORT_SYMBOL(cpaDcInstanceSetNotificationCb);
EXPORT_SYMBOL(cpaDcQueryCapabilities);
EXPORT_SYMBOL(cpaDcSetAddressTranslation);
EXPORT_SYMBOL(cpaDcStartInstance);
EXPORT_SYMBOL(cpaDcStopInstance);

/* Polling symbols */
EXPORT_SYMBOL(icp_sal_CyPollInstance);
EXPORT_SYMBOL(icp_sal_CyPollDpInstance);
EXPORT_SYMBOL(icp_sal_DcPollInstance);
EXPORT_SYMBOL(icp_sal_DcPollDpInstance);
#ifdef ONE_KO_RELEASE_PACKAGE
EXPORT_SYMBOL(icp_sal_pollBank);
EXPORT_SYMBOL(icp_sal_pollAllBanks);
#endif

/* iommu symbols */
EXPORT_SYMBOL(icp_sal_iommu_get_remap_size);
EXPORT_SYMBOL(icp_sal_iommu_map);
EXPORT_SYMBOL(icp_sal_iommu_unmap);

/*Register*/
EXPORT_SYMBOL(cpaMuxRegisterImpl);
EXPORT_SYMBOL(cpaMuxDeRegisterImpl);
