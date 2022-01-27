/***************************************************************************
 *
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
 *
 ***************************************************************************/

/**
 ***************************************************************************
 * @file lac_kpt_ksp_qat_comms.h
 *
 * @defgroup LacKptKspQatCommon
 *
 * @ingroup LacKptKspQatCommon
 *
 * Common definition that are KptKsp specific
 *
 ******************************************************************************/

#ifndef __LAC_KPT_KSP_QAT_COMMS_H__
#define __LAC_KPT_KSP_QAT_COMMS_H__

/*
********************************************************************************
* Include public/global header files
********************************************************************************
*/
#include "cpa.h"
#include "cpa_types.h"
#include "cpa_cy_common.h"
#include "cpa_cy_kpt.h"

/*
********************************************************************************
* Include private header files
********************************************************************************
*/
/* ADF include */
#include "icp_adf_transport.h"

/* QAT include */
#include "icp_qat_fw_kpt_ksp.h"

#define LAC_QAT_KPT_KSP_REQ_SZ_LW (16)
/**< @ingroup LacKptKspQatCommon
 * LAC KSP QAT Request message size) */

#define LAC_QAT_KPT_KSP_RESP_SZ_LW (8)
/**< @ingroup LacKptKspQatCommon
 * LAC KSP QAT Response message size) */

#define LAC_KPT_SERVICE_TYPE (0xa)

#define KPTKSP_REGISTER_HANDLE_CMD (1)
/*KPT KSP register handle cmd */

#define KPTKSP_LOAD_WKY_CMD (2)
/*KPT KSP load wrapping key cmd */

#define KPTKSP_DEL_WKY_CMD (3)
/*KPT KSP delete wrapping key cmd */

#define KPTKSP_REGISTER_SEED_HANDLE_CMD (4)
/*KPT KSP register a handle for RN seed cmd */

#define KPT_KEY_FORMAT_LEN (64)
/*KPT SWK format length*/

#define KPTKSP_REGISTER_KEY_HANDLE_MAXIMUM_NUMBER (30)
/* KPT KSP maximum key handle number can be registered by one application*/

#define KPTKSP_HANDLE_LOW_DWORD_MASK (0xffffffff)

#define KPTKSP_HANDLE_DWORD_LENGTH_INBITS (32)

#define KPTKSP_REQUEST_VALID_MASK (0x80)

#define KPTKSP_WRAPPED_KEY_MAXIMUM_ITERATION_COUNT (0x100)

typedef void *lac_kpt_ksp_request_handle_t;

#define LAC_KPT_KSP_INVALID_HANDLE ((lac_kpt_ksp_request_handle_t)0)

/**
 *****************************************************************************
 * @ingroup LacKptKspQatCommon
 * @description
 *  KPT KSP response client callback opdata
 *****************************************************************************/
typedef struct lac_kpt_ksp_op_cb_data_s
{
    const void *pClientCb;
    void *pCallbackTag;
    Cpa8U cmdID;
    Cpa16U rspStatus;
} lac_kpt_ksp_op_cb_data_t;

/**
 *****************************************************************************
 * @ingroup LacKptKspQatCommon
 * @description
 *  KPT KSP response client callback function
 *****************************************************************************/
typedef void (*lac_kpt_ksp_op_cb_func_t)(CpaStatus status,
                                         Cpa8U cmdID,
                                         Cpa16U rspStatus,
                                         CpaInstanceHandle instanceHandle,
                                         lac_kpt_ksp_op_cb_data_t *pCbData);

/**
 *****************************************************************************
 * @ingroup LacKptKspQatCommon
 * @description
 *  KPT KSP sync mode callback function definition
 *****************************************************************************/
typedef void (*lac_kpt_ksp_sync_cb)(void *pCallbackTag,
                                    CpaStatus status,
                                    Cpa8U cmdID,
                                    Cpa16U rspStatus);

/**
 *****************************************************************************
 * @ingroup LacKptKspQatCommon
 *
 * @description
 *     Contains the input and output buffer data
 *     Will be processed in callback function
 *
 *****************************************************************************/
typedef struct lac_kpt_ksp_req_data_param_info_s
{
    void *keyformat_viradd;
    /*Initialization Vector vitual address*/
} lac_kpt_ksp_req_data_param_info_t;

/**
 *****************************************************************************
 * @ingroup LacKptKspQatCommon
 *
 * @description
 *     Contains the data for a ksp op callback
 *
 *****************************************************************************/
typedef struct lac_kpt_ksp_cb_info_s
{
    lac_kpt_ksp_op_cb_func_t cbFunc;
    lac_kpt_ksp_op_cb_data_t *pcbData;
    CpaInstanceHandle instanceHandle;
} lac_kpt_ksp_cb_info_t;

/**
 ***************************************************************************
 * @ingroup LacKptKspQatCommon
 *      Request data for QAT messages
 *
 * @description
 *      This structure defines the request data for KPT KSP QAT messages. This
 *is
 * used to store data which is known when the message is sent and which we wish
 * to retrieve when the response message is processed.
 **************************************************************************/
typedef struct lac_kpt_ksp_qat_req_data_s
{
    union lac_kpt_ksp_qat_req_data_request_u {
        icp_qat_fw_kpt_ksp_request_t request;
        Cpa8U padding[LAC_QAT_KPT_KSP_REQ_SZ_LW * 4];
    } u1;
    lac_kpt_ksp_cb_info_t cbinfo;
    lac_kpt_ksp_req_data_param_info_t paramInfo;
    struct lac_kpt_ksp_qat_req_data_s *pHeadReqData;
} lac_kpt_ksp_qat_req_data_t;

/**
 *******************************************************************************
 * @ingroup LacKptKspQatCommon
 * Send a single (unchained) KptKsp request to the QAT.
 *
 * @description
 *      This function takes the parameters for a KptKsp QAT request, creates the
 * request, fills in the KptKsp fields and sends it to the QAT. It does block
 * waiting for a response. When the callback function is invoked when the
 * response from the QAT has been processed.
 * @param[in] instanceHandle     InstanceHandle
 * @param[in] cmdID              ksp command id
 * @param[in] kpthandle          A 64bits integer used to indicate wrapping
 *key's identity
 * @param[in] pKptWrappingFormat Pointer of wrapping key format
 * @param[in] keyselflag         Key selection flag, which indicates key loading
 *mode
 * @param[in] keyaction
 * @param[in] pKspOpCbFunc       This function is invoked when the response is
 *                               received from the QAT
 * @param[in] pCbData            callback data to be returned (by copy)
 *                               unchanged in the callback.
 *
 * @retval CPA_STATUS_SUCCESS   No error
 * @retval CPA_STATUS_RESOURCE  Resource error (e.g. failed memory allocation)
 *
 ******************************************************************************/
CpaStatus LacKpt_Ksp_SendSingleRequest(
    CpaInstanceHandle instanceHandle,
    Cpa8U cmdID,
    CpaCyKptHandle kpthandle,
    CpaCyKptWrappingFormat *pKptWrappingFormat,
    CpaCyKptKeySelectionFlags keyselflag,
    CpaCyKptKeyAction keyaction,
    CpaFlatBuffer *pOutputData,
    lac_kpt_ksp_op_cb_func_t pKspOpCbFunc,
    lac_kpt_ksp_op_cb_data_t *pCbData);

/**
 *******************************************************************************
 * @ingroup LacKptKspQatCommon
 * Creates a KptKsp request for the QAT.
 *
 * @description
 *      This function takes the parameters for a KptKsp QAT request, fills in
 *the per
 * request PKE fields.  The request can subsequently be sent to the QAT using
 * SalQatMsg_transPutMsg(). In the event of an error this function will tidy up
 * any resources associated with the request handle and set it to
 * PKE_INVALID_HANDLE.
 *
 *
 * @param[in] pReqestHandl        Pointer of ksp transport handle
 * @param[in] instanceHandle      InstanceHandle
 * @param[in] cmdID               ksp command id
 * @param[in] kpthandle           A 64bits integer used to indicate wrapping
 *key's identity
 * @param[in] pKptWrappingFormat  Pointer of wrapping key format
 * @param[in] keyselflag          Key selection flag, which indicates key
 *loading mode
 * @param[in] keyaction
 * @param[in] pKspOpCbFunc        This function is invoked when the response is
 *                                received from the QAT
 * @param[in] pCbData             Callback data to be returned (by copy)
 *                                unchanged in the callback.
 *
 * @retval CPA_STATUS_SUCCESS   No error
 * @retval CPA_STATUS_RESOURCE  Resource error (e.g. failed memory allocation)
 *
 ******************************************************************************/
CpaStatus LacKpt_Ksp_CreateRequest(lac_kpt_ksp_request_handle_t *pRequestHandle,
                                   CpaInstanceHandle instanceHandle,
                                   Cpa8U cmdID,
                                   CpaCyKptHandle kpthandle,
                                   CpaCyKptWrappingFormat *pKptWrappingFormat,
                                   Cpa16U keyselflag,
                                   Cpa8U keyaction,
                                   CpaFlatBuffer *pOutputData,
                                   lac_kpt_ksp_op_cb_func_t pKspOpCbFunc,
                                   lac_kpt_ksp_op_cb_data_t *pCbData);

/**
 ***************************************************************************
 * @ingroup LacKptKspQatCommon
 *        Destroys a KSP request
 *
 * @description
 *       This function destroys a KPT ksp request that was created using
 * LacKpt_Ksp_CreateRequest().  It should be called if an error occurs during
 * request create or request send, or else as part of the response callback.
 *
 * @param pRequestHandle    IN  Pointer to the request handle that identifies
 *                              the request to be destroyed.  The request
 *                              handle will get set to
 *LAC_KPT_KSP_INVALID_HANDLE.
 *
 * @retval NULL
 ***************************************************************************/
void LacKpt_Ksp_DestroyRequest(lac_kpt_ksp_request_handle_t *pRequestHandle);
#endif
