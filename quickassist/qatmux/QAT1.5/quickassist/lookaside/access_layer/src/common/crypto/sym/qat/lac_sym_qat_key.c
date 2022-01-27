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
 * @file lac_sym_qat_key.c Interfaces for populating the symmetric qat key
 *  structures
 *
 * @ingroup LacSymQatKey
 *
 *****************************************************************************/


#include "cpa.h"
#include "lac_mem.h"
#include "icp_qat_fw_la.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"
#include "lac_list.h"
#include "lac_sal_types.h"
#include "lac_sym_qat_key.h"
#include "lac_sym_hash_defs.h"

void
LacSymQat_KeySslRequestPopulate(
    icp_qat_fw_la_key_gen_req_t *pKeyGenReq,
    Cpa32U generatedKeyLenInBytes,
    Cpa32U labelLenInBytes,
    Cpa32U secretLenInBytes,
    Cpa32U iterations)
{
    /* Rounded to nearest 8 byte boundary */
    Cpa8U outLenRounded = LAC_ALIGN_POW2_ROUNDUP(
        generatedKeyLenInBytes, LAC_QUAD_WORD_IN_BYTES);

    pKeyGenReq->u.keygen_comn.resrvd = 0;
    pKeyGenReq->u.keygen_comn.resrvd1 = 0;
    pKeyGenReq->u.keygen_comn.u1.iter_count = iterations;
    pKeyGenReq->u.keygen_comn.u.secret_len = secretLenInBytes;

    /* Set up the common LA flags */
    pKeyGenReq->comn_la_req.la_cmd_id = ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE;
    pKeyGenReq->comn_la_req.u1.resrvd = 0;
    pKeyGenReq->comn_la_req.u.u.ssl_tls_common.label_len = labelLenInBytes;
    pKeyGenReq->comn_la_req.u.u.ssl_tls_common.out_len = outLenRounded;

}

void
LacSymQat_KeyTlsRequestPopulate(
    icp_qat_fw_la_key_gen_req_t *pKeyGenReq,
    icp_qat_fw_la_auth_req_params_t *pHashReqParams,
    Cpa32U generatedKeyLenInBytes,
    Cpa32U labelLenInBytes,
    Cpa32U secretLenInBytes,
    Cpa8U seedLenInBytes,
    icp_qat_fw_la_cmd_id_t cmdId)
{
    LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_INTERNAL(
        pKeyGenReq->u.req_params_addr,
        pHashReqParams);

    /* Setup the common LA request fields */
    pKeyGenReq->comn_la_req.la_cmd_id = cmdId;
    pKeyGenReq->comn_la_req.u1.req_params_blk_sz =
        LAC_BYTES_TO_QUADWORDS(sizeof(icp_qat_fw_la_auth_req_params_t));
    pKeyGenReq->comn_la_req.u1.tls_seed_len = seedLenInBytes;

    pKeyGenReq->comn_la_req.u.la_flags = 0;
    pKeyGenReq->comn_la_req.u.u.ssl_tls_common.label_len = labelLenInBytes;

    pKeyGenReq->comn_la_req.u.u.ssl_tls_common.out_len =
          LAC_ALIGN_POW2_ROUNDUP(generatedKeyLenInBytes,
            LAC_QUAD_WORD_IN_BYTES);

    /* For TLS u param of auth_req_params is set to secretLen */
    pHashReqParams->u.tls_secret_len = secretLenInBytes;
}

void
LacSymQat_KeyMgfRequestPopulate(
    icp_qat_fw_la_key_gen_req_t *pKeyGenReq,
    Cpa8U seedLenInBytes,
    Cpa16U maskLenInBytes,
    Cpa8U hashLenInBytes)
{
    pKeyGenReq->u.keygen_comn.resrvd = 0;
    pKeyGenReq->u.keygen_comn.resrvd1 = 0;
    pKeyGenReq->u.keygen_comn.u1.iter_count = 0;
    pKeyGenReq->u.keygen_comn.u.mask_len =
            LAC_ALIGN_POW2_ROUNDUP(maskLenInBytes, LAC_QUAD_WORD_IN_BYTES);

    pKeyGenReq->comn_la_req.la_cmd_id = ICP_QAT_FW_LA_CMD_MGF1;
    pKeyGenReq->comn_la_req.u1.resrvd = 0;
    pKeyGenReq->comn_la_req.u.u.mgf_common.seed_len = seedLenInBytes;
    pKeyGenReq->comn_la_req.u.u.mgf_common.hash_len = hashLenInBytes;
}

void
LacSymQat_KeySslKeyMaterialInputPopulate(
    sal_service_t *pService,
    icp_qat_fw_la_ssl_key_material_input_t *pSslKeyMaterialInput,
    void *pSeed,
    Cpa64U labelPhysAddr,
    void *pSecret)
{
    LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_EXTERNAL((*pService),
        pSslKeyMaterialInput->seed_addr,
        pSeed);

    pSslKeyMaterialInput->label_addr = labelPhysAddr;

    LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_EXTERNAL((*pService),
        pSslKeyMaterialInput->secret_addr,
        pSecret);
}

void
LacSymQat_KeyTlsKeyMaterialInputPopulate(
    sal_service_t* pService,
    icp_qat_fw_la_tls_key_material_input_t *pTlsKeyMaterialInput,
    void *pSeed,
    Cpa64U labelPhysAddr)
{
    LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_EXTERNAL((*pService),
        pTlsKeyMaterialInput->seed_addr,
        pSeed);

    pTlsKeyMaterialInput->label_addr = labelPhysAddr;
}
