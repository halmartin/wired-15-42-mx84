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
 * @file lac_sym_qat.c Interfaces for populating the symmetric qat structures
 *
 * @ingroup LacSymQat
 *
 *****************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include "cpa.h"

#include "icp_adf_init.h"
#include "icp_adf_transport.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"
#include "icp_adf_cfg.h"
#include "lac_sym.h"
#include "lac_sym_qat.h"
#include "lac_sal_types_crypto.h"
#include "sal_string_parse.h"
#include "lac_sym_key.h"
#include "lac_sym_qat_hash_defs_lookup.h"

/* sym crypto response handlers */
STATIC
sal_qat_resp_handler_func_t respHandlerSymTbl[ICP_QAT_FW_LA_CMD_DELIMITER];


STATIC void
LacSymQat_SymRespHandler(icp_comms_trans_handle transHandle, void* pRespMsg)
{
    Cpa8U lacCmdId = 0;
    void *pOpaqueData = NULL;
    icp_qat_fw_la_resp_t* pRespMsgFn = NULL;
    Cpa8U opStatus = ICP_QAT_FW_COMN_STATUS_FLAG_OK;

    LAC_ENSURE(transHandle != NULL,
                "LacSymQat_SymRespHandler - transHandle NULL\n");

    pRespMsgFn = (icp_qat_fw_la_resp_t*) pRespMsg;
    LAC_ENSURE(pRespMsgFn != NULL,
                "LacSymQat_SymRespHandler - pRespMsgFn NULL\n");

    LAC_MEM_SHARED_READ_TO_PTR(pRespMsgFn->comn_resp.opaque_data,
                                           pOpaqueData);
    LAC_ENSURE(pOpaqueData != NULL,
                "LacSymQat_SymRespHandler - pOpaqueData NULL\n");

    lacCmdId = pRespMsgFn->comn_resp.serv_cmd_id;
    opStatus = pRespMsgFn->comn_resp.comn_status;
    /* call the response message handler registered for the command ID */
    respHandlerSymTbl[lacCmdId]((icp_qat_fw_la_cmd_id_t)lacCmdId,
                pOpaqueData, (icp_qat_fw_comn_flags)opStatus);
}

CpaStatus
LacSymQat_Init(icp_accel_dev_t* device,
               CpaInstanceHandle instanceHandle,
               Cpa32U numSymRequests)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;
    char temp_string[SAL_CFG_MAX_VAL_LEN_IN_BYTES]={0};
    icp_resp_deliv_method rx_resp_type = ICP_RESP_TYPE_IRQ;
    Cpa32U ring_offset = 0;
    Cpa32U tx_ring_number = 0;
    char * section = DYN_SEC;

    if(SAL_RESP_POLL_CFG_FILE == pService->isPolled)
    {
         rx_resp_type = ICP_RESP_TYPE_POLL;
    }

    if(CPA_FALSE == pService->generic_service_info.is_dyn)
    {
        section = icpGetProcessName();
    }

    status = Sal_StringParsing("Cy",
           pService->generic_service_info.instance,
           "RingSymRxHi",
           temp_string);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Problem while parsing RingSymRxHi string");
        return CPA_STATUS_FAIL;
    }

    status = icp_adf_transCreateHandle(device,
                    ICP_TRANS_TYPE_ETR,
                    section,
                    pService->acceleratorNum,
                    pService->bankNum,
                    temp_string,
                    lac_getRingType(SAL_RING_TYPE_NONE),
                    (icp_trans_callback)LacSymQat_SymRespHandler,
                    rx_resp_type,
                    numSymRequests,
                    (icp_comms_trans_handle *)
                    &(pService->trans_handle_sym_rx_hi));

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create ring RingSymRxHi");
        return CPA_STATUS_FAIL;
    }

    status = Sal_StringParsing("Cy",
           pService->generic_service_info.instance,"RingSymRxLo", temp_string);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Problem while parsing RingSymRxLo string");
        return CPA_STATUS_FAIL;
    }
    status = icp_adf_transCreateHandle(device,
                        ICP_TRANS_TYPE_ETR,
                        section,
                        pService->acceleratorNum,
                        pService->bankNum,
                        temp_string,
                        lac_getRingType(SAL_RING_TYPE_NONE),
                        (icp_trans_callback)LacSymQat_SymRespHandler,
                        rx_resp_type,
                        numSymRequests,
                        (icp_comms_trans_handle *)
                         &(pService->trans_handle_sym_rx_lo));

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create ring RingSymRxLo");
        return CPA_STATUS_FAIL;
    }

    /*
     * Store the response ring number - used to build the QAT message.
     */
    status = icp_adf_transGetRingNum(
                pService->trans_handle_sym_rx_hi,
                &(pService->symHiResponseRingId));
    LAC_CHECK_STATUS(status);

    status = icp_adf_transGetRingNum(
                pService->trans_handle_sym_tx_hi,
                &(tx_ring_number));
    LAC_CHECK_STATUS(status);

    if((!(pService->symHiResponseRingId & 0x1))
            || (pService->symHiResponseRingId != tx_ring_number + 1))
    {
        LAC_LOG_ERROR("Invalid ring number. Rx ring number has to be odd\n");
        LAC_LOG_ERROR("Rx ring number has to be Tx + 1");
        status = CPA_STATUS_FAIL;
    }
    LAC_CHECK_STATUS(status);

    status = icp_adf_transGetRingNum(
                pService->trans_handle_sym_rx_lo,
                &(pService->symLoResponseRingId));
    LAC_CHECK_STATUS(status);

    status = icp_adf_transGetRingNum(
                pService->trans_handle_sym_tx_lo,
                &(tx_ring_number));
    LAC_CHECK_STATUS(status);

    if((!(pService->symLoResponseRingId & 0x1))
            || ((pService->symLoResponseRingId != tx_ring_number + 1)))
    {
        LAC_LOG_ERROR("Invalid ring number. Rx ring number has to be odd\n");
        LAC_LOG_ERROR("Rx ring number has to be Tx + 1");
        status = CPA_STATUS_FAIL;
    }
    LAC_CHECK_STATUS(status);

    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_VF_RING_OFFSET_KEY,
                                      temp_string);
    LAC_CHECK_STATUS(status);

    ring_offset = Sal_Strtoul(temp_string, NULL, SAL_CFG_BASE_DEC);
    pService->symHiResponseRingId += ring_offset;
    pService->symLoResponseRingId += ring_offset;

    /* Initialise the Hash lookup table */
    status = LacSymQat_HashLookupInit(instanceHandle);
    return status;
}

void
LacSymQat_RespHandlerRegister(icp_qat_fw_la_cmd_id_t lacCmdId,
    sal_qat_resp_handler_func_t pCbHandler)
{
    LAC_ENSURE((lacCmdId < ICP_QAT_FW_LA_CMD_DELIMITER), "Invalid Command ID");

    /* set the response handler for the command ID */
    respHandlerSymTbl[lacCmdId] = pCbHandler;
}

void
LacSymQat_LaPacketCommandFlagSet(
    Cpa32U qatPacketType,
    icp_qat_fw_la_cmd_id_t laCmdId,
    Cpa16U *pLaCommandFlags)
{
    ICP_QAT_FW_LA_PARTIAL_SET(
        *pLaCommandFlags, qatPacketType);

    /* update state disabled for full packets and final partials */
    if ((ICP_QAT_FW_LA_PARTIAL_NONE == qatPacketType) ||
        (ICP_QAT_FW_LA_PARTIAL_END == qatPacketType))
    {
        ICP_QAT_FW_LA_WR_STATE_SET(
            *pLaCommandFlags, ICP_QAT_FW_LA_NO_UPDATE_STATE);
    }
    /* For first or middle partials set the update state command flag */
    else
    {
        ICP_QAT_FW_LA_WR_STATE_SET(
            *pLaCommandFlags, ICP_QAT_FW_LA_UPDATE_STATE);

        if (laCmdId == ICP_QAT_FW_LA_CMD_AUTH)
        {
       /* For hash only partial - verify and return auth result are disabled */
            ICP_QAT_FW_LA_RET_AUTH_SET(
                *pLaCommandFlags, ICP_QAT_FW_LA_NO_RET_AUTH_RES);

            ICP_QAT_FW_LA_CMP_AUTH_SET(
                *pLaCommandFlags, ICP_QAT_FW_LA_NO_CMP_AUTH_RES);
        }
    }
}

void
LacSymQat_packetTypeGet(
    CpaCySymPacketType packetType,
    CpaCySymPacketType packetState,
    Cpa32U *pQatPacketType)
{
    /* partial */
    if (CPA_CY_SYM_PACKET_TYPE_PARTIAL == packetType)
    {
        /* if the previous state was full, then this is the first packet */
        if (CPA_CY_SYM_PACKET_TYPE_FULL == packetState)
        {
            *pQatPacketType = ICP_QAT_FW_LA_PARTIAL_START;
        }
        else
        {
            *pQatPacketType = ICP_QAT_FW_LA_PARTIAL_MID;
        }
    }
    /* final partial */
    else if (CPA_CY_SYM_PACKET_TYPE_LAST_PARTIAL == packetType)
    {
        *pQatPacketType = ICP_QAT_FW_LA_PARTIAL_END;
    }
    /* full packet - CPA_CY_SYM_PACKET_TYPE_FULL */
    else
    {
        *pQatPacketType = ICP_QAT_FW_LA_PARTIAL_NONE;
    }
}
