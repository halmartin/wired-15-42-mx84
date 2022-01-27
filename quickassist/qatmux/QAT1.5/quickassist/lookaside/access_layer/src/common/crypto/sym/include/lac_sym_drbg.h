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
 * @file lac_sym_drbg.h
 *
 * @defgroup LacSym_Drbg  Symmetric Deterministic Random Bit Generator as
 *      specified in NIST SP 800-90
 *
 * @ingroup LacSym
 *
 * @description
 *     Interfaces for DRBG implementation
 *
 *****************************************************************************/

#ifndef _LAC_SYM_DRBG_H_
#define _LAC_SYM_DRBG_H_

/* Following defines were provided to avoid comparison with numerical values
 * while building buffer list structure that are later being sent to
 * LacAlgChain_Perform */
#define LAC_DRBG_TWO_BUFFERS    2
#define LAC_DRBG_THREE_BUFFERS  3
#define LAC_DRBG_FOUR_BUFFERS   4
#define LAC_DRBG_FIVE_BUFFERS   5

#define LAC_DRBG_CHECK_NULL_PARAM(param) \
  do{\
    if( NULL == (param) ) return CPA_STATUS_INVALID_PARAM;\
  }while(0);
/**< @ingroup LacSym_Drbg
 * This is the DRBG specific version of LAC_CHECK_NULL_PARAM macro. It doesn't
 * print error to the system log to avoid user confusion caused by messages
 * comming from health test runs. */

#define LAC_DRBG_OUTLEN_IN_BYTES 16
/**< @ingroup LacSym_Drbg
 * Size of the output block of the AES block cipher used for
 * DRBG implementation */

#define LAC_DRBG_KEYLEN256_IN_BYTES 32
/**< @ingroup LacSym_Drbg
 * Size of the cipher key when AES-256 is the base cipher */

#define LAC_DRBG_KEYLEN128_IN_BYTES 16
/**< @ingroup LacSym_Drbg
 * Size of the cipher key when AES-128 is the base cipher */

#define LAC_DRBG_MAX_SEEDLEN_IN_BYTES \
    (LAC_DRBG_OUTLEN_IN_BYTES + LAC_DRBG_KEYLEN256_IN_BYTES)
/**< @ingroup LacSym_Drbg
 * Maximum length of the seed for possible implementations */

#define LAC_DRBG_MAX_NUM_OF_BYTES (0xFFFF)
/**< @ingroup LacSym_Drbg
 * Maximum number of bytes that might be requested in
 * a single Generate operation, as described in NIST SP 800-90,
 * section 10.2.1, Table 3 */

#define LAC_DRBG_SEED_LIFE (0xFFFFFFFFFFFFLL)
/**< @ingroup LacSym_Drbg
 * Maximum number of generate requests before reseed is required.
 * This is maximum value as described in NIST SP 800-90, section 10.2.1,
 * Table 3 (2^48) */

#define LAC_DRBG_ERROR_STATE    1
/**< @ingroup LacSym_Drbg
 * Flag indicating that instance entered into error state */

#define LAC_DRBG_MAX_RETRIES    0xFF
/**< @ingroup LacSym_Drbg
 * Maximum number of retries to be performed before returning an error */

#define LAC_DRBG_DF_DATA_DELIMITER 0x80
/**< @ingroup LacSym_Drbg
 * Derivation Function data delimiter byte as described in NIST SP 800-90,
 * section 10.4.2 (step 4) */

#define LAC_RBG_SEC_STRENGTH_TO_NUM(strength, num) \
    do{                                            \
        switch(strength){                          \
            case CPA_CY_RBG_SEC_STRENGTH_112:      \
                (num) = 112;                       \
                break;                             \
            case CPA_CY_RBG_SEC_STRENGTH_128:      \
                (num) = 128;                       \
                break;                             \
            case CPA_CY_RBG_SEC_STRENGTH_192:      \
                (num) = 192;                       \
                break;                             \
            case CPA_CY_RBG_SEC_STRENGTH_256:      \
                (num) = 256;                       \
                break;                             \
            default:                               \
                (num) = 0;                         \
                break;                             \
        }                                          \
    }while(0);
/**< @ingroup LacSym_Drbg
 * Converts enum value representing security strength to its numerical
 * representation */

#ifndef DISABLE_STATS
#define LAC_DRBG_STAT_INC(pInternalState, statistic, pService)               \
    do{                                                                      \
        if(CPA_FALSE == pInternalState->isTestRun) {                         \
            if(CPA_TRUE ==                                                   \
                    pService->generic_service_info.stats->bDrbgStatsEnabled) \
            {                                                                \
                osalAtomicInc(&pService->pLacDrbgStatsArr[                   \
                                   offsetof(CpaCyDrbgStats64, statistic)     \
                                   / sizeof(Cpa64U) ]);                      \
            }                                                                \
        }                                                                    \
    }while(0)
/**< @ingroup LacSym_Drbg
 * Statistics incrementation macro. If this macro is called in test mode
 * statistics should not be incremented */
#else
#define LAC_DRBG_STAT_INC(pInternalState, statistic, pService)
#endif

#define LAC_DRBG_BITS_TO_BYTES(num) \
    ( (0 == (num) % 8) ? ((num) >> 3) : (((num) >> 3) + 1))
/**< @ingroup LacSym_Drbg
 * Converts number of bits to number of bytes */

#define LAC_DRBG_NONCE_MIN_ENTROPY(secStren) ((secStren) / 2)
/**< @ingroup LacSym_Drbg
 * Calculates minimum entropy required for nonce basing on security strength */


/**
 *****************************************************************************
 * @ingroup LacSym_Drbg
 *      Next operation callback prototype
 *
 * @description
 *      This is the prototype of callback functions used in DRBG
 *      implementation. Whenever a function is called that should operate in
 *      asynchronous mode a pointer to a LacDrbgNextOpCbFunc type of function
 *      is passed to the called function. This callback function is supposed
 *      to continue processing after a reply to asynchronous request is
 *      received
 *
 * @param[in] pCallbackTag  Pointer to the callback tag; contains information
 *                          required to continue operation
 * @param[in] status        Status of the previous operation
 *
 * @return None
 *
 ****************************************************************************/
typedef void (*LacDrbgNextOpCbFunc)(void *pCallbackTag,
        CpaStatus status);

/**
*******************************************************************************
 * @ingroup LacSym_Drbg
 *      Type of operation currently executed
 *
 * @description
 *      This type describes the type of operation that is currently being
 *      executed
 *****************************************************************************/
typedef enum {
    LAC_DRBG_OPERATION_NONE = 0,
    LAC_DRBG_OPERATION_GENERATE,
    LAC_DRBG_OPERATION_RESEED
}lac_drbg_op_type_t;

/* Forward declaration of lac_drbg_internal_statate_s to use it in
 * lac_drbg_callback_data_t */
typedef struct lac_drbg_internal_state_s lac_drbg_internal_state_t;

/**
*******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback data structure
 *
 * @description
 *      This structure stores information passed to callback functions that
 *      are required for functions to continue operation
 *****************************************************************************/
typedef struct lac_drbg_callback_data_s{
    lac_drbg_internal_state_t * pInternalState;
    /**< Pointer to a DRBG internal state */
    sal_crypto_service_t *pService;
    /**< Pointer to a crypto service structure*/
    union{
        struct{
            lac_sync_op_data_t* pSyncCallbackData;
            /**< Pointer to a structure containing synchronization data for
             * synchronous crypto requests */
        }syncCbData;
        /**< Callback data for synchronous crypto requests */
        struct{
            Cpa8U *pData;
            /**< Pointer to data that was passed as a parameter to Update
             * function  */
        }asyncUpdateCbData;
        /**< Callback data for asynchronous Update call */
        struct{
            CpaFlatBuffer *pPseudoRandomBits;
            /**< Pointer to a user supplied buffer where generated pseudo
             * random bits are to be stored  */
            CpaBufferList *pEncryptResult;
            /**< Pointer to a buffer list where results of encrypt operation
             * are to be stored */
            CpaCyDrbgGenOpData *pOpData;
            /**< Pointer to a user supplied OpData structure for Generate
             * request */
            void *pCallbackTag;
            /**< User supplied callback tag for Generate request */
        }asyncGenerateCbData;
        /**< Callback data for asynchronous Generate call */
        struct{
            CpaCyDrbgReseedOpData *pOpData;
            /**< Pointer to a user supplied OpData structure for Reseed
             * request */
        }asyncReseedCbData;
        /**< Callback data for asynchronous Reseed call */
        struct{
            CpaFlatBuffer* pAdditionalInput;
            /**< Pointer to additional input passed to Generate or Reseed
             * call */
        }asyncGetEntropyInputCbData;
        /**< Callback data for asynchronous Get Enropy Input call */
        struct{
            Cpa8U *pTemp;
            /**< Pointer to a temporary data */
            Cpa32U tempLen;
            /**< Length of data in pTemp */
            Cpa32U i;
            /**< Counter of BCC calls, as used in Step 4. of BCC process
             * described in NIST SP 800-90, section 10.4.3 */
            CpaBufferList *pSrcBuffer;
            /**< Pointer to a buffer list containing input data for derivation
             * function */
            Cpa8U* pRequestedOutputData;
            /**< Pointer to a buffer where output data from derivation function
             * are to be stored */
            Cpa32U requestedOutputDataLen;
            /**< Length of requested output data */
        }asyncBlockCipherDFCbData;
        /**< Callback data for asynchronous derivation function call */
    }cbOpData;
    /**< function specific callback data */
    LacDrbgNextOpCbFunc pNextOpCb;
    /**< Pointer to the next operation in the chain */
    void *pNextOpCallbackTag;
    /**< Callback tag for the next operation in the chain; always pointing to
     * lac_drbg_callback_data_t structure  */
}lac_drbg_callback_data_t;

/**
*******************************************************************************
 * @ingroup LacSym_Drbg
 *      Internal state of DRBG instantiation
 * @description
 *      This structure stores information about internal state of DRBG
 *      instantiation
 *****************************************************************************/
typedef struct lac_drbg_working_state_s{
    Cpa8U v[LAC_DRBG_OUTLEN_IN_BYTES];
    /**< V value as described in NIST SP 800-90 - derived from seed */
    Cpa8U key[LAC_DRBG_KEYLEN256_IN_BYTES];
    /**< Key used in cipher request - derived from seed */
    Cpa64U reseedCounter;
    /**< A count of the number of requests or blocks produced since the
     * instantiation was seeded or reseeded */
}lac_drbg_working_state_t;

/**
*******************************************************************************
 * @ingroup LacSym_Drbg
 *      Administration information about DRBG instantiation
 * @description
 *      This structure stores information about DRBG instantiation
 *****************************************************************************/
typedef struct lac_drbg_admin_info_s{
    CpaCyDrbgSecStrength drbgSecurityStrength;
    /**< Security strength with which DRBG instance was set up */
    CpaBoolean predictionResistanceRequired;
    /**< Stores information if DRBG instance was set up with prediction
     * resistance */
}lac_drbg_admin_info_t;

/* The following set of defines is used to calculate maximum buffer that DRBG
 * consumes while creating buffer list structures required to call
 * LacAlgChain_Perform. Maximum comes from LacDrbg_InstantiateDF function */

#define LAC_DRBG_MAX_BUFFER_NO  LAC_DRBG_FIVE_BUFFERS
/**< @ingroup LacSym_Drbg
 * Maximum number of flat buffers in CpaBufferList structure that is
 * constructed internally to process DRBG requests */

#define LAC_DRBG_MAX_BUFFER_META_SIZE \
    (sizeof(icp_buffer_list_desc_t) +\
                 (sizeof(icp_flat_buffer_desc_t) * LAC_DRBG_MAX_BUFFER_NO) +\
                 ICP_DESCRIPTOR_ALIGNMENT_BYTES)
/**< @ingroup LacSym_Drbg
 * Meta Data size for max number of buffers in CpaCyBufferList used in
 * this DRBG implementation */

#define LAC_DRBG_MAX_BUFFER_USAGE \
    (sizeof(CpaBufferList) + LAC_DRBG_MAX_BUFFER_NO * sizeof(CpaFlatBuffer) + \
     LAC_DRBG_MAX_BUFFER_META_SIZE +                                          \
     (2 * sizeof(Cpa32U) + LAC_DRBG_OUTLEN_IN_BYTES) /* prefix buffer to store\
        L and N as per NIST SP 800-90, 10.4.2, Step 4 */ +                    \
     LAC_DRBG_OUTLEN_IN_BYTES /* suffix buffer to store delimiter and padding \
        zeroes as per NIST SP 800-90, 10.4.2, Steps 4 and 5 */ +              \
     (LAC_DRBG_OUTLEN_IN_BYTES + LAC_DRBG_KEYLEN256_IN_BYTES) /* DF out data*/)
/**< @ingroup LacSym_Drbg
 * Max number of bytes required to create buffers */

/**
*******************************************************************************
 * @ingroup LacSym_Drbg
 *      Internal buffers for DRBG
 * @description
 *      This structure contains buffers for storing data required during
 *      various internal DRBG calls
 *****************************************************************************/
typedef struct lac_drbg_internal_buffers_s{
    CpaCySymOpData opData;
    /**< Op Data structure used in calls to LacAlgChain_Perform */
    icp_sal_drbg_get_entropy_op_data_t getEntropyOpData;
    /**< Op Data structure used in calls to Get Entropy Input function */
    Cpa32U entropyLenReturned;
    /**< Variable in which Get Entropy Input function stores information
     * about length of returned entropy */
    lac_drbg_callback_data_t cbDataForUpdate;
    /**< Callback data for asynchronous LacDrbg_Update function call */
    lac_drbg_callback_data_t cbDataForGenerate;
    /**< Callback data for asynchronous Generate function call */
    lac_drbg_callback_data_t cbDataForReseed;
    /**< Callback data for asynchronous LacDrbg_Reseed function call */
    lac_drbg_callback_data_t cbUserDataForReseed;
    /**< Callback data for asynchronous cpaCyDrbgReseed API function call */
    lac_drbg_callback_data_t cbDataForReseedDF;
    /**< Callback data for asynchronous LacDrbg_ReseedDF function call */
    lac_drbg_callback_data_t cbDataForGetEntropyInput;
    /**< Callback data for asynchronous 'Get Entropy Input' function call */
    lac_drbg_callback_data_t cbDataForBCC;
    /**< Callback data for asynchronous LacDrbg_Bcc function call */
    lac_drbg_callback_data_t cbDataForDF;
    /**< Callback data for asynchronous LacDrbg_BlockCipherDF
     * (Derivation Function) function call */
    lac_drbg_callback_data_t cbDataForEncrypt;
    /**< Callback data for asynchronous LacDrbg_Encrypt function call
     * in BCC function */
    CpaFlatBuffer entropyInputBuffer;
    /**< Flat Buffer passed to 'Get Entropy Input' function to store
     * entropy input */
    CpaFlatBuffer nonceBuffer;
    /**< Flat Buffer passed to Get Nonce function to store nonce */
    Cpa8U entropyInput[LAC_DRBG_MAX_SEEDLEN_IN_BYTES];
    /**< Buffer to store entropy input */
    Cpa8U nonce[LAC_DRBG_MAX_SEEDLEN_IN_BYTES];
    /**< Buffer to store nonce */
    Cpa8U seedMaterial[LAC_DRBG_MAX_SEEDLEN_IN_BYTES];
    /**< Buffer to store seed material obtained from entropy input, nonce,
     * additional input / personalization string  */
    Cpa8U persString[LAC_DRBG_MAX_SEEDLEN_IN_BYTES];
    /**< Buffer to store personalization string */
    Cpa8U additionalInput[LAC_DRBG_MAX_SEEDLEN_IN_BYTES];
    /**< Buffer to store additional input */
    Cpa8U buff[LAC_DRBG_MAX_BUFFER_USAGE];
    /**< Buffer used to prepare buffer list passed to LacAlgChain_Perform,
     * together with all additional flat buffers required for padding and
     * for prefix buffers */
}lac_drbg_internal_buffers_t;

/**
*******************************************************************************
 * @ingroup LacSym_Drbg
 *      Internal state of DRBG instantiation
 * @description
 *      This structure describes internal state of DRBG instantiation
 *****************************************************************************/
struct lac_drbg_internal_state_s{
    lac_drbg_working_state_t workingState;
    /**< Working state of DRBG instantiation.
     * NOTE: this is supposed to be the first field in the internal state as
     * the address of v should 8 byte aligned */

    lac_drbg_internal_buffers_t buffers;
    /**< Internal buffers for DRBG instantiation */

    lac_drbg_admin_info_t adminInfo;
    /**< Administration info about DRBG instantiation */

    CpaBoolean reseedRequired;
    /**< Flag indicating that Reseed is required before next pseudo random bits
     * can be generated */

    lac_drbg_op_type_t operationType;
    /**< Describes type of operation that is currently being executed */

    CpaCyGenFlatBufCbFunc pGenCb;
    /**< Callback function provided by user during DRBG instantiation. If this
     * is NULL the Generate function will operate in synchronous mode */
    CpaCyGenericCbFunc pReseedCb;
    /**< Callback function provided by user during DRBG instantiation. If this
     * is NULL the Reseed function will operate in synchronous mode */

    lac_lock_t sessionLock;
    /**< Lock used to providing exclusive access for number of In-Flight
     * requests update */
    OsalAtomic numInFlightRequests;
    /**< Number of requests being processed at the moment */

    lac_sync_op_data_t* pSyncCallbackData;
    /**< Pointer to a variable used for synchronization when operating in
     * synchronous mode */

    OsalAtomic errorState;
    /**< Represents error state of instantiation; value different than zero
     * indicates occurrence of catastrophic error or failure of health test.
     * Instance that enters error state cannot be used again - re-instantiation
     * is required */

    Cpa32U nImplKeyLen;
    /**< Key length for the implementation for which session was setup */

    Cpa32U nImplSeedLen;
    /**< Seed length for the implementation for which session was setup */

    CpaBoolean isDFRequired;
    /**< Indicates whether derivation function is required for the entropy
     * source used in this session */

    IcpSalDrbgGetEntropyInputFunc pGetEntropyInputFunc;
    /**< Pointer to a function used to obtain entropy input for instantiate or
     * reseed */

    IcpSalDrbgGetNonceFunc pGetNonceFunc;
    /**< Pointer to a function used to obtain nonce during instantiate */

    CpaBoolean isTestRun;
    /**< Indicates whether internal states represents health test session */

    Cpa32U symSessionSize;
    /**< Size used by symmetric session inside allocated DRBG session handle */

    Cpa32U maxBufferListMetaSize;
    /**< Maximum size (in bytes) of meta data required for buffer lists
     * created internally within DRBG implementation   */

    CpaBoolean isWorkingStateModified;
    /**< Indicates whether working state has been modified as a part of
     * processing of current request (reseed or generate). Used to decide
     * whether session should be invalidated when in case of problems with
     * sending requests to firmware (e.g. when rings are full and the maximum
     * number of retries has been reached) */

    CpaInstanceHandle instanceHandle;
    /**< The intanceHandle being used by the DRBG session */
};

#define LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle) \
    (lac_drbg_internal_state_t *) (*(LAC_ARCH_UINT *)sessionHandle)
/**< @ingroup LacSym_Session
 * Retrieve the internal state pointer from the session handle that the user
 * allocated. The pointer to the internally realigned address is stored at
 * the start of the session handle. */


/* FUNCTIONS */
/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Determine if the device possesses random capability
 *
 * @description
 *      This function determines if the device supports random capability
 *      by examining the capabilitiesMask of the device.
 *
 * @param[in]  instanceHandle         Instance handle.
l*
 * @return CPA_TRUE                   Random Capability is supported
 * @return CPA_FALSE                  Random Capability is not supported
 *
 *****************************************************************************/
CpaBoolean
LacDrbg_CheckRandomCapability(const CpaInstanceHandle instanceHandle);
/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Internal implementation of DRBG Init Session / Instantiate
 *
 * @description
 *      This function implements DRBG Init Session / Instantiate process. API
 *      function cpaCyDrbgInitSession, after running self test calls this
 *      function to initialize DRBG instantiation.
 *
 * @param[in]  instanceHandle_in      Instance handle.
 * @param[in]  pGenCb                 User callback for Generate operation. If
 *                                    this is set, cpaCyDrbgGen will operate
 *                                    asynchronously; if the pointer is NULL
 *                                    all calls to  cpaCyDrbgGen will be
 *                                    synchronous
 * @param[in]  pReseedCb              User callback for Reseed operation. If
 *                                    this is set, cpaCyDrbgReseed will operate
 *                                    asynchronously; if the pointer is NULL
 *                                    all calls to  cpaCyDrbgReseed will be
 *                                    synchronous
 * @param[in]  pSetupData             Pointer to session setup data
 * @param[in]  sessionHandle          DRBG session handle
 * @param[out] pSeedLen               Pointer to a variable to which the
 *                                    function will write length of seed that
 *                                    current DRBG session will use
 * @param[in]  testRun                Indicates whether the function is run as
 *                                    a part of a normal call or health test
 *                                    call
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid parameter provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_InitSession(const CpaInstanceHandle instanceHandle_in,
        const CpaCyGenFlatBufCbFunc pGenCb,
        const CpaCyGenericCbFunc pReseedCb,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaCyDrbgSessionHandle sessionHandle,
        Cpa32U* pSeedLen,
        CpaBoolean testRun);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Allocates and initializes statistics array
 *
 * @description
 *      This function clears DRBG statistics
 *
 * @param[in]  pService               Pointer to a crypto service structure
 *
 *****************************************************************************/
CpaStatus
LacDrbg_StatsInit(sal_crypto_service_t* pService);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Frees statistics array
 *
 * @description
 *      This function clears DRBG statistics
 *
 * @param[in]  pService               Pointer to a crypto service structure
 *
 *****************************************************************************/
void
LacDrbg_StatsFree(sal_crypto_service_t* pService);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      BCC DRBG function
 *
 * @description
 *      This function implements the BCC function as described in
 *      NIST SP 800-90, section 10.4.3. It is called from inside the
 *      derivation function.
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pSrcBuffer             Buffer list containing input data
 * @param[out] pOutputBlock           Buffer for the output data
 * @param[in]  pNextOpFunc            Function that needs to be executed after
 *                                    BCC process is completed when operating
 *                                    in asynchronous operation; NULL when
 *                                    operating synchronously
 * @param[in]  pCallbackTag           Pointer to opaque data that is passed to
 *                                    pNextOpFunc when working in asynchronous
 *                                    mode
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_Bcc(sal_crypto_service_t* pService,
        lac_drbg_internal_state_t * pInternalState,
        CpaBufferList *pSrcBuffer,
        Cpa8U* pOutputBlock,
        LacDrbgNextOpCbFunc pNextOpFunc,
        void * pCallbackTag);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      AES cipher function for use in DRBG
 *
 * @description
 *      This function uses symmetric cipher session set up during session
 *      initialization to perform AES cipher operation needed in different DRBG
 *      processes.
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pSymCb                 Callback function used to indicate that
 *                                    symmetric cipher request is finished and
 *                                    requested data is ready
 * @param[in]  pCbData                Pointer to callback data required to
 *                                    continue processing when cipher request
 *                                    is finished. Passed to pSymCb as a
 *                                    pCallbackTag
 * @param[in]  algorithm              Algorithm that is to be used for
 *                                    encryption
 * @param[in]  pKey                   Encryption key to be used in cipher
 *                                    request
 * @param[in]  pIv                    Initialization vector / counter data to
 *                                    be used in cipher request
 * @param[in]  pSrcBuffer             Buffer list containing input data
 * @param[out]  pDstBuffer             Buffer list to store output data
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_Encrypt(sal_crypto_service_t* pService,
               lac_drbg_internal_state_t * pInternalState,
               CpaCySymCbFunc pSymCb,
               lac_drbg_callback_data_t *pCbData,
               CpaCySymCipherAlgorithm algorithm,
               Cpa8U* pKey,
               Cpa8U* pIv,
               CpaBufferList *pSrcBuffer,
               CpaBufferList *pDstBuffer);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Symmetric session callback function for use in synchronous mode of
 *      operation
 *
 * @description
 *      This function is called to notify that the symmetric request is
 *      completed when operating in synchronous mode. It unblocks caller
 *      functions
 *
 * @param[in]  pCallbackTag           Opaque pointer to user data
 * @param[in]  status                 Status of symmetric request
 * @param[in]  operationType          Type of operation this callback refers to
 * @param[in]  pOpData                Pointer to Op Data structure that was
 *                                    passed to LacAlgChain_Perform function
 *                                    that this callback originates from
 * @param[in]  pDstBuffer             Pointer to destination buffer list that
 *                                    was passed to LacAlgChain_Perform
 *                                    function that this callback originates
 *                                    from - contains requested data
 * @param[in]  verifyResult           Result of digest verification if the
 *                                    callback comes from hash request and
 *                                    if verification was requested
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_SyncCb(void *pCallbackTag,
        CpaStatus status,
        const CpaCySymOp operationType,
        void *pOpData,
        CpaBufferList *pDstBuffer,
        CpaBoolean verifyResult);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Symmetric session callback function for use in asynchronous mode of
 *      operation
 *
 * @description
 *      This function is called to notify that the symmetric request is
 *      completed when operating in asynchronous mode. It calls the next
 *      operation in the chain
 *
 * @param[in]  pCallbackTag           Opaque pointer to user data
 * @param[in]  status                 Status of symmetric request
 * @param[in]  operationType          Type of operation this callback refers to
 * @param[in]  pOpData                Pointer to Op Data structure that was
 *                                    passed to LacAlgChain_Perform function
 *                                    that this callback originates from
 * @param[in]  pDstBuffer             Pointer to destination buffer list that
 *                                    was passed to LacAlgChain_Perform
 *                                    function that this callback originates
 *                                    from - contains requested data
 * @param[in]  verifyResult           Result of digest verification if the
 *                                    callback comes from hash request and
 *                                    if verification was requested
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_ASyncCb(void *pCallbackTag,
        CpaStatus status,
        const CpaCySymOp operationType,
        void *pOpData,
        CpaBufferList *pDstBuffer,
        CpaBoolean verifyResult);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Symmetric session callback function for use in asynchronous mode of
 *      operation to continue DRBG Update process
 *
 * @description
 *      This function is called to notify that the symmetric request is
 *      completed when operating in asynchronous mode. It continues the DRBG
 *      Update process (XORs encryption result with the input data) and
 *      calls the next operation in the chain
 *
 * @param[in]  pCallbackTag           Opaque pointer to user data
 * @param[in]  status                 Status of symmetric request
 * @param[in]  operationType          Type of operation this callback refers to
 * @param[in]  pOpData                Pointer to Op Data structure that was
 *                                    passed to LacAlgChain_Perform function
 *                                    that this callback originates from
 * @param[in]  pDstBuffer             Pointer to destination buffer list that
 *                                    was passed to LacAlgChain_Perform
 *                                    function that this callback originates
 *                                    from - contains requested data
 * @param[in]  verifyResult           Result of digest verification if the
 *                                    callback comes from hash request and
 *                                    if verification was requested
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_UpdateASyncCb(void *pCallbackTag,
        CpaStatus status,
        const CpaCySymOp operationType,
        void *pOpData,
        CpaBufferList *pDstBuffer,
        CpaBoolean verifyResult);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Update Function
 *
 * @description
 *      This function implements the CTR_DRBG Update process as described
 *      in NIST SP 800-90, section 10.2.1.2
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pData                  Pointer to input data. It is expected
 *                                    that there are SEEDLEN bytes of valid
 *                                    data.
 * @param[in]  pNextOpCb              Function that needs to be executed after
 *                                    Update Function process is completed
 *                                    when operating in asynchronous operation;
 *                                    NULL when operating synchronously
 * @param[in]  pCallbackTag           Pointer to opaque data that is passed to
 *                                    pNextOpFunc when working in asynchronous
 *                                    mode
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_Update(sal_crypto_service_t* pService,
              lac_drbg_internal_state_t * pInternalState,
              Cpa8U* pData,
              LacDrbgNextOpCbFunc pNextOpCb,
              void * pCallbackTag);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Instantiate Function for entropy sources that do not require
 *      derivation function.
 *
 * @description
 *      This function implements the CTR_DRBG Instantiate process for
 *      entropy sources that do not require derivation function as described
 *      in NIST SP 800-90, section 10.2.1.3.1
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pSetupData             Pointer to setup data passed to
 *                                    cpaCyDrbgInitSession function
 * @param[in]  pEntropyInput          Pointer to a buffer containing entropy
 *                                    input
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_InstantiateNoDF(sal_crypto_service_t* pService,
        lac_drbg_internal_state_t * pInternalState,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaFlatBuffer *pEntropyInput);


/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Instantiate Function for entropy sources that do require
 *      derivation function.
 *
 * @description
 *      This function implements the CTR_DRBG Instantiate process for
 *      entropy sources that do require derivation function as described
 *      in NIST SP 800-90, section 10.2.1.3.2
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pSetupData             Pointer to setup data passed to
 *                                    cpaCyDrbgInitSession function
 * @param[in]  pEntropyInput          Pointer to a buffer containing entropy
 *                                    input
 * @param[in]  pNonceBuffer           Pointer to a buffer containing nonce used
 *                                    to derive seed
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_InstantiateDF(sal_crypto_service_t* pService,
        lac_drbg_internal_state_t * pInternalState,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaFlatBuffer *pEntropyInput,
        CpaFlatBuffer *pNonceBuffer);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Initializes symmetric sessions used in DRBG implementation
 *
 * @description
 *      This function initializes symmetric sessions - cipher and hash(if
 *      needed) - that are used in DRBG
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pSessionCtx            Pointer to memory allocated for storing
 *                                    symmetric session context
 * @param[in]  isCipherSession        Indicates whether session that is to be
 *                                    set up is cipher session or hash session
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_InitSymSession(sal_crypto_service_t* pService,
                      lac_drbg_internal_state_t * pInternalState,
                      CpaCySymSessionCtx pSessionCtx,
                      CpaBoolean isCipherSession);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Reseed Function for entropy sources that do not require
 *      derivation function.
 *
 * @description
 *      This function implements the CTR_DRBG Reseed process for
 *      entropy sources that do not require derivation function as described
 *      in NIST SP 800-90, section 10.2.1.4.1 (steps 1-4).
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pEntropyInput          Pointer to a buffer containing entropy
 *                                    input
 * @param[in]  pAdditionalInput       Pointer to a buffer containing additional
 *                                    input
 * @param[in]  pNextOpFunc            Function that needs to be executed after
 *                                    Reseed Function process is completed
 *                                    when operating in asynchronous operation;
 *                                    NULL when operating synchronously
 * @param[in]  pCallbackTag           Pointer to opaque data that is passed to
 *                                    pNextOpFunc when working in asynchronous
 *                                    mode
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_ReseedNoDF(sal_crypto_service_t* pService,
        lac_drbg_internal_state_t * pInternalState,
        CpaFlatBuffer *pEntropyInput,
        CpaFlatBuffer *pAdditionalInput,
        LacDrbgNextOpCbFunc pNextOpFunc,
        void *pCallbackTag);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Reseed Function for entropy sources that do require
 *      derivation function.
 *
 * @description
 *      This function implements the CTR_DRBG Reseed process for
 *      entropy sources that do require derivation function as described
 *      in NIST SP 800-90, section 10.2.1.4.2 (steps 1-2)
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pEntropyInput          Pointer to a buffer containing entropy
 *                                    input
 * @param[in]  pAdditionalInput       Pointer to a buffer containing additional
 *                                    input
 * @param[in]  pNextOpFunc            Function that needs to be executed after
 *                                    Reseed Function process is completed
 *                                    when operating in asynchronous operation;
 *                                    NULL when operating synchronously
 * @param[in]  pCallbackTag           Pointer to opaque data that is passed to
 *                                    pNextOpFunc when working in asynchronous
 *                                    mode
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_ReseedDF(sal_crypto_service_t* pService,
        lac_drbg_internal_state_t * pInternalState,
        CpaFlatBuffer *pEntropyInput,
        CpaFlatBuffer *pAdditionalInput,
        LacDrbgNextOpCbFunc pNextOpFunc,
        void *pCallbackTag);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Reseed Function
 *
 * @description
 *      This function implements the CTR_DRBG Reseed process as described
 *      in NIST SP 800-90, Section 9.2
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  sessionHandle          DRBG session handle
 * @param[in]  pAdditionalInput       Pointer to a buffer containing additional
 *                                    input
 * @param[in]  pNextOpFunc            Function that needs to be executed after
 *                                    Reseed Function process is completed
 *                                    when operating in asynchronous mode;
 *                                    NULL when operating synchronously
 * @param[in]  pCallbackTag           Pointer to opaque data that is passed to
 *                                    pNextOpFunc when working in asynchronous
 *                                    mode
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_Reseed(sal_crypto_service_t* pService,
              CpaCyDrbgSessionHandle sessionHandle,
              CpaFlatBuffer *pAdditionalInput,
              LacDrbgNextOpCbFunc pNextOpFunc,
              void *pCallbackTag);


/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Generate Function for entropy sources that do not require
 *      derivation function.
 *
 * @description
 *      This function implements the CTR_DRBG Generate process for
 *      entropy sources that do not require derivation function as described
 *      in NIST SP 800-90, section 10.2.1.5.1 .
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  numOfBytesRequested    Number of bytes that user requested to
 *                                    generate
 * @param[in]  pAdditionalInput       Pointer to a buffer containing additional
 *                                    input
 * @param[in]  pPseudoRandomBits      Pointer to a buffer where requested bytes
 *                                    are to be stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_GenerateNoDF(sal_crypto_service_t* pService,
                    lac_drbg_internal_state_t * pInternalState,
                    Cpa32U numOfBytesRequested,
                    const CpaFlatBuffer *pAdditionalInput,
                    CpaFlatBuffer *pPseudoRandomBits);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Generate Function for entropy sources that do require
 *      derivation function.
 *
 * @description
 *      This function implements the CTR_DRBG Generate process for
 *      entropy sources that do require derivation function as described
 *      in NIST SP 800-90, section 10.2.1.5.2.
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  numOfBytesRequested    Number of bytes that user requested to
 *                                    generate
 * @param[in]  pAdditionalInput       Pointer to a buffer containing additional
 *                                    input
 * @param[in]  pPseudoRandomBits      Pointer to a buffer where requested bytes
 *                                    are to be stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_GenerateDF(sal_crypto_service_t* pService,
                    lac_drbg_internal_state_t * pInternalState,
                    Cpa32U numOfBytesRequested,
                    const CpaFlatBuffer *pAdditionalInput,
                    CpaFlatBuffer *pPseudoRandomBits);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Function performing common steps that are common for Generate process
 *      for all entropy sources
 *
 * @description
 *      This function implements the CTR_DRBG Generate process for all
 *      entropy sources as described in NIST SP 800-90, section 10.2.1.5.1
 *      (steps 3-8) and section 10.2.1.5.2 (steps 3-8)
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  numOfBytesRequested    Number of bytes that user requested to
 *                                    generate
 * @param[in]  pPseudoRandomBits      Pointer to a buffer where requested bytes
 *                                    are to be stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_GenerateCommon(sal_crypto_service_t *pService,
        lac_drbg_internal_state_t * pInternalState,
        Cpa32U numOfBytesRequested,
        CpaFlatBuffer *pPseudoRandomBits);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for Get Entropy Input function
 *
 * @description
 *      This function is called from within Get Entropy Input function when
 *      operating in asynchronous mode to notify that entropy input is ready
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to Get
 *                                    Entropy Input function; it contains data
 *                                    required to continue processing
 * @param[in]  status                 Indicates the status of Get Entropy Input
 *                                    operation
 * @param[in]  pOpdata                Pointer to op data passed to Get Entropy
 *                                    Input function
 * @param[in]  lenReturned            Length of entropy input returned by Get
 *                                    Entropy Input function
 * @param[in]  pOut                   Pointer to a buffer to store entropy
 *                                    input that was passed to Get Entropy
 *                                    Input function
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_GetEntropyInputCb(void *pCallbackTag,
        CpaStatus status,
        void *pOpdata,
        Cpa32U lenReturned,
        CpaFlatBuffer *pOut);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Internal Reseed Callback
 *
 * @description
 *      Callback function used to implement synchronous mode of operation.
 *      It is used when cpaCyDrbgReseed is called and session is setup
 *      in synchronous mode.
 *
 * @param[in] pCallbackTag Opaque value provided by user while making
 *                         individual function call.
 * @param[in] status       Status of the operation. Valid values are
 *                         CPA_STATUS_SUCCESS and CPA_STATUS_FAIL.
 * @param[in] pOpData      Opaque Pointer to the operation data that was
 *                         submitted in the request
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_ReseedInternalCb(void *pCallbackTag,
        CpaStatus status,
        void *pOpData);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *     Callback from LacDrbg_Reseed function when API reseed function is called
 *     in asynchronous mode
 *
 * @description
 *     Callback function for LacDrbg_Reseed function when API Reseed
 *     function is called in asynchronous mode - it updates reseed statistics
 *     and calls user callback
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_ReseedCallUserCb(void *pCallbackTag,
                     CpaStatus opStatus);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for LacDrbg_ReseedDF and LacDrbg_ReseedNoDF function
 *
 * @description
 *      Callback function to continue reseed operation after Update request
 *      is finished - pairs with LacDrbg_ReseedDF and LacDrbg_ReseedNoDF,
 *      as described in NIST SP 800-90, section 10.2.1.4.2 (steps 4-5)
 *      and section 10.2.1.4.1 (steps 5-6) respectively.
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_ReseedCb(void *pCallbackTag,
                     CpaStatus opStatus);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for derivation function to continue reseed process
 *
 * @description
 *      This callback continues reseed process after the call to derivation
 *      function is completed. Pairs with LacDrbg_BlockCipherDF.
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_ReseedDFCb(void *pCallbackTag,
                    CpaStatus opStatus);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for Encrypt function called during Generate process
 *
 * @description
 *      This function continues Generate process after Encrypt is finished
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_GenerateNextOpAfterEncryptCb(void *pCallbackTag,
           CpaStatus opStatus);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for Reseed function called during Generate process
 *
 * @description
 *      This callback continues Generate process after Derivation Function.
 *      It is used when Derivation Function is required.
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_GenerateDFNextOpAfterDFCb(void *pCallbackTag,
           CpaStatus opStatus);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for Update function called during Generate process
 *
 * @description
 *      This function continues generate process after Update on
 *      additional input is completed
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_GenerateNextOpAfterUpdateCb(void *pCallbackTag,
           CpaStatus opStatus);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for Reseed function called during Generate process
 *
 * @description
 *      This callback continues Generate process after Reseed step
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_GenerateNextOpAfterReseedCb(void *pCallbackTag,
        CpaStatus opStatus);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for LacDrbg_Bcc function
 *
 * @description
 *      This callback continues derivation function operation after BCC request
 *      is completed. Paired with LacDrbg_Bcc
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_BccCb(void *pCallbackTag,
        CpaStatus opStatus);

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Callback for Encrypt function called during derivation function process
 *
 * @description
 *      This callback continues derivation function operation after the
 *      Encrypt call
 *
 * @param[in]  pCallbackTag           Opaque pointer to data passed to the
 *                                    requesting function; it contains data
 *                                    required to continue processing
 * @param[in]  opStatus               Indicates the status of the previous
 *                                    operation
 *
 * @return None
 *
 *****************************************************************************/
void
LacDrbg_BlockCipherDFEncryptCb(void *pCallbackTag,
        CpaStatus opStatus);

#endif /* _LAC_SYM_DRBG_H_ */
