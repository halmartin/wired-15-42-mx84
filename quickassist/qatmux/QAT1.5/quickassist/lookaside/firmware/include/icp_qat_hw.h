/*
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
 */

/**
 *****************************************************************************
 * @file icp_qat_hw.h
 * @defgroup icp_qat_hw_defs ICP QAT HW definitions
 * @ingroup icp_qat_hw
 * @description
 *      This file documents definitions for the QAT HW
 *
 *****************************************************************************/

#ifndef __ICP_QAT_HW_H__
#define __ICP_QAT_HW_H__

/*
******************************************************************************
* Include public/global header files
******************************************************************************
*/

/* ========================================================================= */
/*                                                  AccelerationEngine       */
/* ========================================================================= */


typedef enum
{
    ICP_QAT_HW_AE_0=0,                 /*!< ID of AE0 */
    ICP_QAT_HW_AE_1=1,                 /*!< ID of AE1 */
    ICP_QAT_HW_AE_2=2,                 /*!< ID of AE2 */
    ICP_QAT_HW_AE_3=3,                 /*!< ID of AE3 */
    ICP_QAT_HW_AE_4=4,                 /*!< ID of AE4 */
    ICP_QAT_HW_AE_5=5,                 /*!< ID of AE5 */
    ICP_QAT_HW_AE_6=6,                 /*!< ID of AE6 */
    ICP_QAT_HW_AE_7=7,                 /*!< ID of AE7 */
    ICP_QAT_HW_AE_DELIMITER=8          /**< Delimiter type */
} icp_qat_hw_ae_id_t;

/* ========================================================================= */
/*                                                                 QAT       */
/* ========================================================================= */

typedef enum
{
  ICP_QAT_HW_QAT_0=0,                 /*!< ID of QAT0 */
  ICP_QAT_HW_QAT_1=1,                 /*!< ID of QAT1 */
  ICP_QAT_HW_QAT_DELIMITER=2          /**< Delimiter type */
} icp_qat_hw_qat_id_t;

/* ========================================================================= */
/*                                                          RingCluster      */
/* ========================================================================= */

typedef enum
{
  ICP_QAT_HW_RING_CLUSTER_0=0,         /*!< ID of RingCluster0 - Rings 0-127 */
  ICP_QAT_HW_RING_CLUSTER_1=1,         /*!< ID of RingCluster1 - Rings 128-255*/
  ICP_QAT_HW_RING_CLUSTER_DELIMITER=2  /**< Delimiter type */
} icp_qat_hw_ring_cluster_id_t;

/* ========================================================================= */
/*                                                  AUTH SLICE               */
/* ========================================================================= */

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Supported Authentication Algorithm types
 * @description
 *      Enumeration which is used to define the authenticate algorithms
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_AUTH_ALGO_NULL=0,                 /*!< Null hashing */
   ICP_QAT_HW_AUTH_ALGO_SHA1=1,                 /*!< SHA1 hashing */
   ICP_QAT_HW_AUTH_ALGO_MD5=2,                  /*!< MD5 hashing */
   ICP_QAT_HW_AUTH_ALGO_SHA224=3,               /*!< SHA-224 hashing */
   ICP_QAT_HW_AUTH_ALGO_SHA256=4,               /*!< SHA-256 hashing */
   ICP_QAT_HW_AUTH_ALGO_SHA384=5,               /*!< SHA-384 hashing */
   ICP_QAT_HW_AUTH_ALGO_SHA512=6,               /*!< SHA-512 hashing */
   ICP_QAT_HW_AUTH_ALGO_AES_XCBC_MAC=7,         /*!< AES-XCBC-MAC hashing */
   ICP_QAT_HW_AUTH_ALGO_AES_CBC_MAC=8,          /*!< AES-CBC-MAC hashing */
   ICP_QAT_HW_AUTH_ALGO_AES_F9=9,               /*!< AES F9 hashing */
   ICP_QAT_HW_AUTH_ALGO_GALOIS_128=10,          /*!< Galois 128 bit hashing */
   ICP_QAT_HW_AUTH_ALGO_GALOIS_64=11,           /*!< Galois 64 hashing */
   ICP_QAT_HW_AUTH_ALGO_KASUMI_F9=12,           /*!< Kasumi F9 hashing */
   ICP_QAT_HW_AUTH_ALGO_SNOW_3G_UIA2=13,        /*!< UIA2/SNOW_3H F9 hashing */
   ICP_QAT_HW_AUTH_ALGO_DELIMITER=14            /**< Delimiter type */
} icp_qat_hw_auth_algo_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported Authentication modes
 * @description
 *      Enumeration which is used to define the authentication slice modes.
 *      The concept of modes is very specific to the QAT implementation. Its
 *      main use is differentiate how the algorithms are used i.e. mode0 SHA1
 *      will configure the QAT Auth Slice to do plain SHA1 hashing while mode1
 *      configures it to do SHA1 HMAC with precomputes and mode2 sets up the
 *      slice to do SHA1 HMAC with no precomputes (uses key directly)
 *
 * @Note
 *      Only some algorithms are valid in some of the modes. If you dont know
 *      what you are doing then refer back to the HW documentation
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_AUTH_MODE0=0,                 /*!< QAT Auth Mode0 configuration */
   ICP_QAT_HW_AUTH_MODE1=1,                 /*!< QAT Auth Mode1 configuration */
   ICP_QAT_HW_AUTH_MODE2=2,                 /*!< QAT AuthMode2 configuration */
   ICP_QAT_HW_AUTH_MODE_DELIMITER=3         /**< Delimiter type */
} icp_qat_hw_auth_mode_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Auth configuration structure
 *
 * @description
 *      Definition of the format of the authentication slice configuration
 *
 *****************************************************************************/
typedef struct icp_qat_hw_auth_config_s
{
   uint32_t config;
   /**< Configuration used for setting up the slice */

   uint32_t reserved;
   /**< Reserved */
} icp_qat_hw_auth_config_t;

/* Private defines */

/* Note: Bit positions have been defined for little endian ordering */

#define QAT_AUTH_MODE_BITPOS            4
/**< @ingroup icp_qat_hw_defs
 * Starting bit position for indicating the Auth mode */

#define QAT_AUTH_MODE_MASK              0xF
/**< @ingroup icp_qat_hw_defs
 * Four bit mask used for determing the Auth mode */

#define QAT_AUTH_ALGO_BITPOS            0
/**< @ingroup icp_qat_hw_defs
 * Starting bit position for indicating the Auth Algo  */

#define QAT_AUTH_ALGO_MASK              0xF
/**< @ingroup icp_qat_hw_defs
 * Four bit mask used for determining the Auth algo */

#define QAT_AUTH_CMP_BITPOS             8
/**< @ingroup icp_qat_hw_defs
 * Starting bit position for indicating the Auth Compare */

#define QAT_AUTH_CMP_MASK               0x7F
/**< @ingroup icp_qat_hw_defs
 * Seven bit mask used for determing the Auth Compare. */

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Returns the configuration word for the auth slice based on the inputs
 *      of mode, algorithm type and compare length. Note for the digest
 *      generation case the compare length is a dont care value. Also if the
 *      client will be doing the digest validation the compare_length will not
 *      be used
 *
 * @param mode      Authentication mode to use
 * @param algo      Auth Algorithm to use
 * @param cmp_len   The length of the digest if the QAT is to the check
 *
 *****************************************************************************/
#define ICP_QAT_HW_AUTH_CONFIG_BUILD(mode,algo,cmp_len)   \
            ((((mode) & QAT_AUTH_MODE_MASK) << QAT_AUTH_MODE_BITPOS ) | \
             (((algo) & QAT_AUTH_ALGO_MASK) << QAT_AUTH_ALGO_BITPOS ) | \
             (((cmp_len) & QAT_AUTH_CMP_MASK) << QAT_AUTH_CMP_BITPOS ) )

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Auth Counter structure
 *
 * @description
 *      32 bit counter that tracks the number of data bytes passed through
 *      the slice. This is used by the padding logic for some algorithms. Note
 *      only the upper 32 bits are set.
 *
 *****************************************************************************/
typedef struct icp_qat_hw_auth_counter_s
{
   uint32_t counter;
   /**< Counter value */
   uint32_t reserved;
   /**< Reserved */
} icp_qat_hw_auth_counter_t;

/* Private defines */
#define QAT_AUTH_COUNT_MASK         0xFFFFFFFF
/**< @ingroup icp_qat_hw_defs
 * Thirty two bit mask used for determining the Auth count */

#define QAT_AUTH_COUNT_BITPOS      0
/**< @ingroup icp_qat_hw_defs
 * Starting bit position indicating the Auth count.  */

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Macro to build the auth counter quad word
 *
 * @param val      Counter value to set
 *
 *****************************************************************************/
#define ICP_QAT_HW_AUTH_COUNT_BUILD(val)                        \
      (((val) & QAT_AUTH_COUNT_MASK) << QAT_AUTH_COUNT_BITPOS)

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the common auth parameters
 * @description
 *      This part of the configuration is constant for each service
 *
 *****************************************************************************/
typedef struct icp_qat_hw_auth_setup_s
{
   icp_qat_hw_auth_config_t auth_config;
   /**< Configuration word for the auth slice */
   icp_qat_hw_auth_counter_t auth_counter;
    /**< Auth counter value for this request */
} icp_qat_hw_auth_setup_t;

/* ************************************************************************* */
/* ************************************************************************* */

#define QAT_HW_DEFAULT_ALIGNMENT            8
#define QAT_HW_ROUND_UP(val, n)                (((val) + ((n)-1)) & (~(n-1)))

/* State1 */
#define ICP_QAT_HW_NULL_STATE1_SZ            32
/**< @ingroup icp_qat_hw_defs
 * State1 block size for NULL hashing */
#define ICP_QAT_HW_MD5_STATE1_SZ            16
/**< @ingroup icp_qat_hw_defs
 * State1 block size for MD5 */
#define ICP_QAT_HW_SHA1_STATE1_SZ            20
/**< @ingroup icp_qat_hw_defs
 * Define the state1 block size for SHA1 - Note that for the QAT HW the state
 * is rounded to the nearest 8 byte multiple */
#define ICP_QAT_HW_SHA224_STATE1_SZ         32
/**< @ingroup icp_qat_hw_defs
 * State1 block size for SHA24 */
#define ICP_QAT_HW_SHA256_STATE1_SZ         32
/**< @ingroup icp_qat_hw_defs
 * State1 block size for SHA256 */
#define ICP_QAT_HW_SHA384_STATE1_SZ         64
/**< @ingroup icp_qat_hw_defs
 * State1 block size for SHA384 */
#define ICP_QAT_HW_SHA512_STATE1_SZ         64
/**< @ingroup icp_qat_hw_defs
 * State1 block size for SHA512 */
#define ICP_QAT_HW_AES_XCBC_MAC_STATE1_SZ       16
/**< @ingroup icp_qat_hw_defs
 * State1 block size for XCBC */
#define ICP_QAT_HW_AES_CBC_MAC_STATE1_SZ        16
/**< @ingroup icp_qat_hw_defs
 * State1 block size for CBC */
#define ICP_QAT_HW_AES_F9_STATE1_SZ             32
/**< @ingroup icp_qat_hw_defs
 * State1 block size for AES F9 */
#define ICP_QAT_HW_KASUMI_F9_STATE1_SZ          16
/**< @ingroup icp_qat_hw_defs
 * State1 block size for Kasumi F9 */
#define ICP_QAT_HW_GALOIS_128_STATE1_SZ         16
/**< @ingroup icp_qat_hw_defs
 * State1 block size for Galois128 */
#define ICP_QAT_HW_SNOW_3G_UIA2_STATE1_SZ       8
/**< @ingroup icp_cpm_hw_defs
 * State1 block size for UIA2 */

/* State2 */
#define ICP_QAT_HW_NULL_STATE2_SZ           32
/**< @ingroup icp_qat_hw_defs
 * State2 block size for NULL hashing */
#define ICP_QAT_HW_MD5_STATE2_SZ            16
/**< @ingroup icp_qat_hw_defs
 * State2 block size for MD5 */
#define ICP_QAT_HW_SHA1_STATE2_SZ           20
/**< @ingroup icp_qat_hw_defs
 * State2 block size for SHA1 - Note that for the QAT HW the state  is rounded
 * to the nearest 8 byte multiple */
#define ICP_QAT_HW_SHA224_STATE2_SZ         32
/**< @ingroup icp_qat_hw_defs
 * State2 block size for SHA224 */
#define ICP_QAT_HW_SHA256_STATE2_SZ         32
/**< @ingroup icp_qat_hw_defs
 * State2 block size for SHA256 */
#define ICP_QAT_HW_SHA384_STATE2_SZ         64
/**< @ingroup icp_qat_hw_defs
 * State2 block size for SHA384 */
#define ICP_QAT_HW_SHA512_STATE2_SZ         64
/**< @ingroup icp_qat_hw_defs
 * State2 block size for SHA512 */
#define ICP_QAT_HW_AES_XCBC_MAC_KEY_SZ      16
/**< @ingroup icp_qat_hw_defs
 * State2 block size for XCBC */
#define ICP_QAT_HW_AES_CBC_MAC_KEY_SZ       16
/**< @ingroup icp_qat_hw_defs
 * State2 block size for CBC */
#define ICP_QAT_HW_AES_CCM_CBC_E_CTR0_SZ    16
/**< @ingroup icp_qat_hw_defs
 * State2 block size for AES Encrypted Counter 0 */
#define ICP_QAT_HW_F9_IK_SZ                   16
/**< @ingroup icp_qat_hw_defs
 * State2 block size for F9 IK */
#define ICP_QAT_HW_F9_FK_SZ                 16
/**< @ingroup icp_qat_hw_defs
 * State2 block size for F9 FK */
#define ICP_QAT_HW_KASUMI_F9_STATE2_SZ      (ICP_QAT_HW_F9_IK_SZ +        \
                                                      ICP_QAT_HW_F9_FK_SZ)
/**< @ingroup icp_qat_hw_defs
 * State2 complete size for Kasumi F9 */
#define ICP_QAT_HW_AES_F9_STATE2_SZ        ICP_QAT_HW_KASUMI_F9_STATE2_SZ
/**< @ingroup icp_qat_hw_defs
 * State2 complete size for AES F9 */
#define ICP_QAT_HW_SNOW_3G_UIA2_STATE2_SZ            24
/**< @ingroup icp_cpm_hw_defs
 * State2 block size for UIA2 */
#define ICP_QAT_HW_GALOIS_H_SZ              16
/**< @ingroup icp_qat_hw_defs
 * State2 block size for Galois Multiplier H */
#define ICP_QAT_HW_GALOIS_LEN_A_SZ          8
/**< @ingroup icp_qat_hw_defs
 * State2 block size for Galois AAD length */
#define ICP_QAT_HW_GALOIS_E_CTR0_SZ         16
/**< @ingroup icp_qat_hw_defs
 * State2 block size for Galois Encrypted Counter 0 */

/* ************************************************************************* */
/* ************************************************************************* */

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of SHA512 auth algorithm processing struct
 * @description
 *      This structs described the parameters to pass to the slice for
 *      configuring it for SHA512 processing. This is the largest possible
 *      setup block for authentication
 *
 *****************************************************************************/
typedef struct icp_qat_hw_auth_sha512_s
{
   icp_qat_hw_auth_setup_t inner_setup;
   /**< Inner loop configuration word for the slice */

   uint8_t   state1[ICP_QAT_HW_SHA512_STATE1_SZ];
   /**< Slice state1 variable */

   icp_qat_hw_auth_setup_t outer_setup;
   /**< Outer configuration word for the slice */

   uint8_t   state2[ICP_QAT_HW_SHA512_STATE2_SZ];
   /**< Slice state2 variable */

} icp_qat_hw_auth_sha512_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Supported hardware authentication algorithms
 * @description
 *      Common grouping of the auth algorithm types supported by the QAT
 *
 *****************************************************************************/
typedef union icp_qat_hw_auth_algo_blk_u
{
   icp_qat_hw_auth_sha512_t sha512;
   /**< SHA512 Hashing */

} icp_qat_hw_auth_algo_blk_t;

#define ICP_QAT_HW_GALOIS_LEN_A_BITPOS      0
/**< @ingroup icp_qat_hw_defs
 * Bit position of the 32 bit A value in the 64 bit A configuration sent to
 * the QAT */

#define ICP_QAT_HW_GALOIS_LEN_A_MASK      0xFFFFFFFF
/**< @ingroup icp_qat_hw_defs
 * Mask value for A value */

/* ========================================================================= */
/*                                                  BULK SLICE */
/* ========================================================================= */

/* To be defined */

/* ========================================================================= */
/*                                                CIPHER SLICE */
/* ========================================================================= */

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported Cipher Algorithm types
 * @description
 *      Enumeration used to define the cipher algorithms
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_CIPHER_ALGO_NULL=0,               /*!< Null ciphering */
   ICP_QAT_HW_CIPHER_ALGO_DES=1,                /*!< DES ciphering */
   ICP_QAT_HW_CIPHER_ALGO_3DES=2,               /*!< 3DES ciphering */
   ICP_QAT_HW_CIPHER_ALGO_AES128=3,             /*!< AES-128 ciphering */
   ICP_QAT_HW_CIPHER_ALGO_AES192=4,             /*!< AES-192 ciphering */
   ICP_QAT_HW_CIPHER_ALGO_AES256=5,             /*!< AES-256 ciphering */
   ICP_QAT_HW_CIPHER_ALGO_ARC4=6,               /*!< ARC4 ciphering */
   ICP_QAT_HW_CIPHER_ALGO_KASUMI=7,             /*!< Kasumi */
   ICP_QAT_HW_CIPHER_ALGO_SNOW_3G_UEA2=8,       /*!< Snow_3G */
   ICP_QAT_HW_CIPHER_DELIMITER=8                /**< Delimiter type */
} icp_qat_hw_cipher_algo_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported cipher modes of operation
 * @description
 *      Enumeration used to define the cipher slice modes.
 *
 * @Note
 *      Only some algorithms are valid in some of the modes. If you dont know
 *      what you are doing then refer back to the EAS
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_CIPHER_ECB_MODE=0,                /*!< ECB mode */
   ICP_QAT_HW_CIPHER_CBC_MODE=1,                /*!< CBC more */
   ICP_QAT_HW_CIPHER_CTR_MODE=2,                /*!< CTR mode */
   ICP_QAT_HW_CIPHER_F8_MODE=3,                 /*!< F8 mode */
   ICP_QAT_HW_CIPHER_MODE_DELIMITER=4           /**< Delimiter type */
} icp_qat_hw_cipher_mode_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Cipher Configuration Struct
 *
 * @description
 *      Configuration data used for setting up the QAT Cipher Slice
 *
 *****************************************************************************/

typedef struct icp_qat_hw_cipher_config_s
{
   uint32_t val;
   /**< Cipher slice configuration */

   uint32_t reserved;
   /**< Reserved */
} icp_qat_hw_cipher_config_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the cipher direction
 * @description
 *      Enumeration which is used to define the cipher direction to apply
 *
 *****************************************************************************/

typedef enum
{
   /*!< Flag to indicate that encryption is required */
    ICP_QAT_HW_CIPHER_ENCRYPT=0,
   /*!< Flag to indicate that decryption is required */
    ICP_QAT_HW_CIPHER_DECRYPT=1,

} icp_qat_hw_cipher_dir_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the cipher key conversion modes
 * @description
 *      Enumeration which is used to define if cipher key conversion is needed
 *
 *****************************************************************************/

typedef enum
{
    /*!< Flag to indicate that no key convert is required */
   ICP_QAT_HW_CIPHER_NO_CONVERT=0,
    /*!< Flag to indicate that key conversion is required */
   ICP_QAT_HW_CIPHER_KEY_CONVERT=1,
} icp_qat_hw_cipher_convert_t;

/* Private defines */

/* Note: Bit positions have been arranged for little endian ordering */

#define QAT_CIPHER_MODE_BITPOS          4
/**< @ingroup icp_qat_hw_defs
 * Define for the cipher mode bit position */

#define QAT_CIPHER_MODE_MASK            0xF
/**< @ingroup icp_qat_hw_defs
 * Define for the cipher mode mask (four bits) */

#define QAT_CIPHER_ALGO_BITPOS           0
/**< @ingroup icp_qat_hw_defs
 * Define for the cipher algo bit position */

#define QAT_CIPHER_ALGO_MASK            0xF
/**< @ingroup icp_qat_hw_defs
 * Define for the cipher algo mask (four bits) */

#define QAT_CIPHER_CONVERT_BITPOS        9
/**< @ingroup icp_qat_hw_defs
 * Define the cipher convert key bit position */

#define QAT_CIPHER_CONVERT_MASK         0x1
/**< @ingroup icp_qat_hw_defs
 * Define for the cipher convert key mask (one bit)*/

#define QAT_CIPHER_DIR_BITPOS            8
/**< @ingroup icp_qat_hw_defs
 * Define for the cipher direction bit position */

#define QAT_CIPHER_DIR_MASK             0x1
/**< @ingroup icp_qat_hw_defs
 * Define for the cipher direction mask (one bit) */

#define QAT_CIPHER_MODE_F8_KEY_SZ_MULT    2
/**< @ingroup icp_qat_hw_defs
 * Define for the cipher mode F8 key size */

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Build the cipher configuration field
 *
 * @param mode      Cipher Mode to use
 * @param algo      Cipher Algorithm to use
 * @param convert   Specify if the key is to be converted
 * @param dir       Specify the cipher direction either encrypt or decrypt
 *
 *****************************************************************************/
#define ICP_QAT_HW_CIPHER_CONFIG_BUILD(mode,algo,convert,dir)               \
         ((((mode) & QAT_CIPHER_MODE_MASK) << QAT_CIPHER_MODE_BITPOS )    | \
          (((algo) & QAT_CIPHER_ALGO_MASK) << QAT_CIPHER_ALGO_BITPOS )    | \
          (((convert) & QAT_CIPHER_CONVERT_MASK) <<                         \
                                            QAT_CIPHER_CONVERT_BITPOS )   | \
          (((dir) & QAT_CIPHER_DIR_MASK) << QAT_CIPHER_DIR_BITPOS ) )

#define ICP_QAT_HW_DES_BLK_SZ              8
/**< @ingroup icp_qat_hw_defs
 * Define the block size for DES.
 * This used as either the size of the IV or CTR input value */
#define ICP_QAT_HW_3DES_BLK_SZ             8
/**< @ingroup icp_qat_hw_defs
 * Define the processing block size for 3DES */
#define ICP_QAT_HW_NULL_BLK_SZ             8
/**< @ingroup icp_qat_hw_defs
 * Define the processing block size for NULL */
#define ICP_QAT_HW_AES_BLK_SZ              16
/**< @ingroup icp_qat_hw_defs
 * Define the processing block size for AES 128, 192 and 256 */
#define ICP_QAT_HW_KASUMI_BLK_SZ           8
/**< @ingroup icp_qat_hw_defs
 * Define the processing block size for KASUMI */
#define ICP_QAT_HW_SNOW_3G_BLK_SZ          8
/**< @ingroup icp_qat_hw_defs
 * Define the processing block size for SNOW_3G */
#define ICP_QAT_HW_DES_KEY_SZ              8
/**< @ingroup icp_qat_hw_defs
 * Define the key size for DES */
#define ICP_QAT_HW_3DES_KEY_SZ             24
/**< @ingroup icp_qat_hw_defs
 * Define the key size for 3DES */
#define ICP_QAT_HW_AES_128_KEY_SZ          16
/**< @ingroup icp_qat_hw_defs
 * Define the key size for AES128 */
#define ICP_QAT_HW_AES_192_KEY_SZ          24
/**< @ingroup icp_qat_hw_defs
 * Define the key size for AES192 */
#define ICP_QAT_HW_AES_256_KEY_SZ          32
/**< @ingroup icp_qat_hw_defs
 * Define the key size for AES256 */
#define ICP_QAT_HW_AES_128_F8_KEY_SZ       (ICP_QAT_HW_AES_128_KEY_SZ * \
                                        QAT_CIPHER_MODE_F8_KEY_SZ_MULT)
/**< @ingroup icp_qat_hw_defs
 * Define the key size for AES128 F8 */
#define ICP_QAT_HW_AES_192_F8_KEY_SZ       (ICP_QAT_HW_AES_192_KEY_SZ * \
                                        QAT_CIPHER_MODE_F8_KEY_SZ_MULT)
/**< @ingroup icp_qat_hw_defs
 * Define the key size for AES192 F8 */
#define ICP_QAT_HW_AES_256_F8_KEY_SZ       (ICP_QAT_HW_AES_256_KEY_SZ * \
                                        QAT_CIPHER_MODE_F8_KEY_SZ_MULT)
/**< @ingroup icp_qat_hw_defs
 * Define the key size for AES256 F8 */
#define ICP_QAT_HW_KASUMI_KEY_SZ           16
/**< @ingroup icp_qat_hw_defs
 * Define the key size for Kasumi */
#define ICP_QAT_HW_KASUMI_F8_KEY_SZ    (ICP_QAT_HW_KASUMI_KEY_SZ * \
                                        QAT_CIPHER_MODE_F8_KEY_SZ_MULT)
/**< @ingroup icp_qat_hw_defs
 * Define the key size for Kasumi F8 */
#define ICP_QAT_HW_ARC4_KEY_SZ             256
/**< @ingroup icp_qat_hw_defs
 * Define the key size for ARC4 */
#define ICP_QAT_HW_SNOW_3G_UEA2_KEY_SZ     16
/**< @ingroup icp_cpm_hw_defs
 * Define the key size for SNOW_3G_UEA2 */
#define ICP_QAT_HW_SNOW_3G_UEA2_IV_SZ      16
/**< @ingroup icp_cpm_hw_defs
 * Define the iv size for SNOW_3G_UEA2 */
#define ICP_QAT_HW_MODE_F8_NUM_REG_TO_CLEAR  2
/**< @ingroup icp_cpm_hw_defs
 * Number of the HW register to clear in F8 mode */


/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of AES-256 F8 cipher algorithm processing struct
 * @description
 *      This structs described the parameters to pass to the slice for
 *      configuring it for AES-256 F8 processing
 *
 *****************************************************************************/
typedef struct icp_qat_hw_cipher_aes256_f8_s
{
   icp_qat_hw_cipher_config_t cipher_config;
   /**< Cipher configuration word for the slice set to
    * AES-256 and the F8 mode */

   uint8_t key[ICP_QAT_HW_AES_256_F8_KEY_SZ];
   /**< Cipher key */

} icp_qat_hw_cipher_aes256_f8_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Supported hardware cipher algorithms
 * @description
 *      Common grouping of the cipher algorithm types supported by the QAT.
 *      This is the largest possible cipher setup block size
 *
 *****************************************************************************/
typedef union icp_qat_hw_cipher_algo_blk_u
{

   icp_qat_hw_cipher_aes256_f8_t aes256_f8;
   /**< AES-256 F8 Cipher */

} icp_qat_hw_cipher_algo_blk_t;


/* ========================================================================= */
/*                                                  TRNG SLICE */
/* ========================================================================= */

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported TRNG configuration modes
 * @description
 *      Enumeration used to define the TRNG modes. Used by clients when
 *      configuring the TRNG for use
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_TRNG_DBL=0,                /*!< TRNG Disabled mode */
   ICP_QAT_HW_TRNG_NHT=1,                /*!< TRNG Normal Health Test mode */
   ICP_QAT_HW_TRNG_KAT=4,                /*!< TRNG Known Answer Test mode */
   ICP_QAT_HW_TRNG_DELIMITER=8           /**< Delimiter type */
} icp_qat_hw_trng_cfg_mode_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported TRNG KAT (known answer test) modes
 * @description
 *      Enumeration which is used to define the TRNG KAT modes. Used by clients
 *      when configuring the TRNG for testing
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_TRNG_NEG_0=0,             /*!< TRNG Neg Zero Test */
   ICP_QAT_HW_TRNG_NEG_1=1,             /*!< TRNG Neg One Test */
   ICP_QAT_HW_TRNG_POS=2,               /*!< TRNG POS Test */
   ICP_QAT_HW_TRNG_POS_VNC=3,           /*!< TRNG POS VNC Test */
   ICP_QAT_HW_TRNG_KAT_DELIMITER=4      /**< Delimiter type */
} icp_qat_hw_trng_kat_mode_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      TRNG mode configuration structure.
 *
 * @description
 *      Definition of the format of the TRNG slice configuration. Used
 *      internally by the QAT FW for configuration of the KAT unit or the
 *      TRNG depending on the slice command i.e. either a set_slice_config or
 *      slice_wr_KAT_type
 *
 *****************************************************************************/

typedef struct icp_qat_hw_trng_config_s
{
   uint32_t val;
  /**< Configuration used for setting up the TRNG slice */

   uint32_t reserved;
   /**< Reserved */
} icp_qat_hw_trng_config_t;

/* Private Defines*/

/* Note: Bit positions have been arranged for little endian ordering */


#define QAT_TRNG_CONFIG_MODE_MASK              0x7
/**< @ingroup icp_qat_hw_defs
 * Mask for the TRNG configuration mode. (Three bits) */

#define QAT_TRNG_CONFIG_MODE_BITPOS            5
/**< @ingroup icp_qat_hw_defs
 * TRNG configuration mode bit positions start */

#define QAT_TRNG_KAT_MODE_MASK                 0x3
/**< @ingroup icp_qat_hw_defs
 * Mask of two bits for the TRNG known answer test mode */

#define QAT_TRNG_KAT_MODE_BITPOS               6
/**< @ingroup icp_qat_hw_defs
 * TRNG known answer test mode bit positions start */

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Build the configuration byte for the TRNG slice based on the mode
 *
 * @param mode   Configuration mode parameter
 *
 *****************************************************************************/
#define ICP_QAT_HW_TRNG_CONFIG_MODE_BUILD(mode)   \
      (((mode) & QAT_TRNG_CONFIG_MODE_MASK) << QAT_TRNG_CONFIG_MODE_BITPOS)

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Build the configuration byte for the TRNG KAT based on the mode
 *
 * @param mode   Configuration mode parameter
 *
 *****************************************************************************/
#define ICP_QAT_HW_TRNG_KAT_MODE_BUILD(mode)   \
      ((((mode) & QAT_TRNG_KAT_MODE_MASK) << QAT_TRNG_KAT_MODE_BITPOS))

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      TRNG test status structure.
 *
 * @description
 *      Definition of the format of the TRNG slice test status structure. Used
 *      internally by the QAT FW.
 *
 *****************************************************************************/

typedef struct icp_qat_hw_trng_test_status_s
{

    uint32_t status;
   /**< Status used for setting up the TRNG slice */

    uint32_t fail_count;
   /**< Comparator fail count */
} icp_qat_hw_trng_test_status_t;

#define ICP_QAT_HW_TRNG_TEST_NO_FAILURES                  1
/**< @ingroup icp_qat_hw_defs
 * Flag to indicate that there were no Test Failures */

#define ICP_QAT_HW_TRNG_TEST_FAILURES_FOUND               0
/**< @ingroup icp_qat_hw_defs
 * Flag to indicate that there were Test Failures */

#define ICP_QAT_HW_TRNG_TEST_STATUS_VALID                  1
/**< @ingroup icp_qat_hw_defs
 * Flag to indicate that there is no valid Test output */

#define ICP_QAT_HW_TRNG_TEST_STATUS_INVALID                0
/**< @ingroup icp_qat_hw_defs
 * Flag to indicate that the Test output is still invalid */

/* Private defines */
#define QAT_TRNG_TEST_FAILURE_FLAG_MASK                    0x1
/**< @ingroup icp_qat_hw_defs
 * Mask of one bit used to determine the TRNG Test pass/fail */

#define QAT_TRNG_TEST_FAILURE_FLAG_BITPOS                  4
/**< @ingroup icp_qat_hw_defs
 * Flag position to indicate that the TRNG Test status is pass of fail */

#define QAT_TRNG_TEST_STATUS_MASK                          0x1
/**< @ingroup icp_qat_hw_defs
 * Mask of one bit used to determine the TRNG Test staus */

#define QAT_TRNG_TEST_STATUS_BITPOS                        1
/**< @ingroup icp_qat_hw_defs
 * Flag position to indicate the TRNG Test status */

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Extract the fail bit for the TRNG slice
 *
 * @param status   TRNG status value
 *
 *****************************************************************************/

#define ICP_QAT_HW_TRNG_FAIL_FLAG_GET(status)         \
   (((status) >> QAT_TRNG_TEST_FAILURE_FLAG_BITPOS) &    \
                                              QAT_TRNG_TEST_FAILURE_FLAG_MASK)

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Extract the status valid bit for the TRNG slice
 *
 * @param status   TRNG status value
 *
 *****************************************************************************/
#define ICP_QAT_HW_TRNG_STATUS_VALID_GET(status)   \
   (((status) >> QAT_TRNG_TEST_STATUS_BITPOS) & QAT_TRNG_TEST_STATUS_MASK)

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      TRNG entropy counters
 *
 * @description
 *      Definition of the format of the TRNG entropy counters. Used internally
 *      by the QAT FW.
 *
 *****************************************************************************/

typedef struct icp_qat_hw_trng_entropy_counts_s
{
   uint64_t raw_ones_count;
   /**< Count of raw ones of entropy */

   uint64_t raw_zeros_count;
   /**< Count of raw zeros of entropy */

   uint64_t cond_ones_count;
   /**< Count of conditioned ones entropy */

   uint64_t cond_zeros_count;
   /**< Count of conditioned zeros entropy */
} icp_qat_hw_trng_entropy_counts_t;

/* Private defines */
#define QAT_HW_TRNG_ENTROPY_STS_RSVD_SZ      4
/**< @ingroup icp_qat_hw_defs
 * TRNG entropy status reserved size in bytes */

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      TRNG entropy available status.
 *
 * @description
 *      Definition of the format of the TRNG slice entropy status available.
 *      struct. Used internally by the QAT FW.
 *
 *****************************************************************************/
typedef struct icp_qat_hw_trng_entropy_status_s
{
   uint32_t status;
   /**< Entropy status in the TRNG */

    uint8_t reserved[QAT_HW_TRNG_ENTROPY_STS_RSVD_SZ];
   /**< Reserved */
} icp_qat_hw_trng_entropy_status_t;

#define ICP_QAT_HW_TRNG_ENTROPY_AVAIL                  1
/**< @ingroup icp_qat_hw_defs
 * Flag indicating that entropy data is available in the QAT TRNG slice */

#define ICP_QAT_HW_TRNG_ENTROPY_NOT_AVAIL              0
/**< @ingroup icp_qat_hw_defs
 * Flag indicating that no entropy data is available in the QAT TRNG slice */

/* Private defines */
#define QAT_TRNG_ENTROPY_STATUS_MASK                   1
/**< @ingroup icp_qat_hw_defs
 * Mask of one bit used to determine the TRNG Entropy status*/

#define QAT_TRNG_ENTROPY_STATUS_BITPOS                 0
/**< @ingroup icp_qat_hw_defs
 * Starting bit position for TRNG Entropy status. */

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Extract the entropy available status bit
 *
 * @param status   TRNG status value
 *
 *****************************************************************************/
#define ICP_QAT_HW_TRNG_ENTROPY_STATUS_GET(status)   \
   (((status) >> QAT_TRNG_ENTROPY_STATUS_BITPOS) & \
                                                QAT_TRNG_ENTROPY_STATUS_MASK)

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Entropy seed data
 *
 * @description
 *      This type is used for the definition of the entropy generated by a read
 *      of the TRNG slice
 *
 *****************************************************************************/
typedef uint64_t icp_qat_hw_trng_entropy;

/* ========================================================================= */
/*                                                STORAGE SLICE */
/* ========================================================================= */

/* To be defined */

/* ========================================================================= */
/*                                            COMPRESSION SLICE */
/* ========================================================================= */

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported compression directions
 * @description
 *      Enumeration used to define the compression directions
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_COMPRESSION_DIR_COMPRESS=0,                /*!< Compression */
   ICP_QAT_HW_COMPRESSION_DIR_DECOMPRESS=1,              /*!< Decompression */
   ICP_QAT_HW_COMPRESSION_DIR_DELIMITER=2                /**< Delimiter type */
} icp_qat_hw_compression_direction_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported delayed match modes
 * @description
 *      Enumeration used to define whether delayed match is enabled
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_COMPRESSION_DELAYED_MATCH_DISABLED=0,
   /*!< Delayed match disabled */

   ICP_QAT_HW_COMPRESSION_DELAYED_MATCH_ENABLED=1,
   /*!< Delayed match enabled */

   ICP_QAT_HW_COMPRESSION_DELAYED_MATCH_DELIMITER=2
   /**< Delimiter type */

} icp_qat_hw_compression_delayed_match_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported compression algorithms
 * @description
 *      Enumeration used to define the compression algorithms
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_COMPRESSION_ALGO_DEFLATE=0,            /*!< Deflate compression */
   ICP_QAT_HW_COMPRESSION_ALGO_LZS=1,                /*!< LZS compression */
   ICP_QAT_HW_COMPRESSION_ALGO_DELIMITER=2           /**< Delimiter type */
} icp_qat_hw_compression_algo_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported compression depths
 * @description
 *      Enumeration used to define the compression slice depths.
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_COMPRESSION_DEPTH_1=0,
   /*!< Search depth 1 (Fastest least exhaustive) */

   ICP_QAT_HW_COMPRESSION_DEPTH_4=1,
   /*!< Search depth 4 */

   ICP_QAT_HW_COMPRESSION_DEPTH_8=2,
   /*!< Search depth 8 */

   ICP_QAT_HW_COMPRESSION_DEPTH_16=3,
   /*!< Search depth 16 (Slowest, most exhaustive) */

   ICP_QAT_HW_COMPRESSION_DEPTH_DELIMITER=4
   /**< Delimiter type */

} icp_qat_hw_compression_depth_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported file types
 * @description
 *      Enumeration used to define the compression file types.
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_COMPRESSION_FILE_TYPE_0=0,
   /*!< Use Static Trees */
   
   ICP_QAT_HW_COMPRESSION_FILE_TYPE_1=1,
   /*!< Use Semi-Dynamic Trees at offset 0 */
   
   ICP_QAT_HW_COMPRESSION_FILE_TYPE_2=2,
   /*!< Use Semi-Dynamic Trees at offset 320 */
   
   ICP_QAT_HW_COMPRESSION_FILE_TYPE_3=3,
   /*!< Use Semi-Dynamic Trees at offset 640 */
   
   ICP_QAT_HW_COMPRESSION_FILE_TYPE_4=4,
   /*!< Use Semi-Dynamic Trees at offset 960 */
   
   ICP_QAT_HW_COMPRESSION_FILE_TYPE_DELIMITER=5
   /**< Delimiter type */

} icp_qat_hw_compression_file_type_t;



/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Compression Configuration Struct
 *
 * @description
 *      Configuration data used for setting up the QAT Compression Slice
 *
 *****************************************************************************/

typedef struct icp_qat_hw_compression_config_s
{
   uint32_t val;
   /**< Compression slice configuration */

   uint32_t reserved;
   /**< Reserved */
} icp_qat_hw_compression_config_t;


/* Private defines */
#define QAT_COMPRESSION_DIR_BITPOS               4
/**< @ingroup icp_qat_hw_defs
 * Define for the compression direction bit position */

#define QAT_COMPRESSION_DIR_MASK                 0x7
/**< @ingroup icp_qat_hw_defs
 * Define for the compression direction mask (three bits) */
 
 #define QAT_COMPRESSION_DELAYED_MATCH_BITPOS    16
/**< @ingroup icp_qat_hw_defs
 * Define for the compression delayed match bit position */

#define QAT_COMPRESSION_DELAYED_MATCH_MASK       0x1
/**< @ingroup icp_qat_hw_defs
 * Define for the delayed match mask (one bit) */
 
 #define QAT_COMPRESSION_ALGO_BITPOS             31
/**< @ingroup icp_qat_hw_defs
 * Define for the compression algorithm bit position */

#define QAT_COMPRESSION_ALGO_MASK                0x1
/**< @ingroup icp_qat_hw_defs
 * Define for the compression algorithm mask (one bit) */
 
 #define QAT_COMPRESSION_DEPTH_BITPOS            28
/**< @ingroup icp_qat_hw_defs
 * Define for the compression depth bit position */

#define QAT_COMPRESSION_DEPTH_MASK               0x7
/**< @ingroup icp_qat_hw_defs
 * Define for the compression depth mask (three bits) */
 
 #define QAT_COMPRESSION_FILE_TYPE_BITPOS        24
/**< @ingroup icp_qat_hw_defs
 * Define for the compression file type bit position */

#define QAT_COMPRESSION_FILE_TYPE_MASK            0xF
/**< @ingroup icp_qat_hw_defs
 * Define for the compression file type mask (four bits) */
 
 

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Build the compression slice configuration field
 *
 * @param dir      Compression Direction to use, compress or decompress
 * @param delayed  Specify if delayed match should be enabled
 * @param algo     Compression algorithm to use
 * @param depth    Compression search depth to use
 * @param filetype Compression file type to use, static or semi dynamic trees
 *
 *****************************************************************************/
#define ICP_QAT_HW_COMPRESSION_CONFIG_BUILD(dir,delayed,algo,depth,filetype) \
         ((((dir) & QAT_COMPRESSION_DIR_MASK) <<                             \
                                QAT_COMPRESSION_DIR_BITPOS )  |              \
         (((delayed) & QAT_COMPRESSION_DELAYED_MATCH_MASK) <<                \
                                QAT_COMPRESSION_DELAYED_MATCH_BITPOS )  |    \
         (((algo) & QAT_COMPRESSION_ALGO_MASK) <<                            \
                                QAT_COMPRESSION_ALGO_BITPOS )  |             \
         (((depth) & QAT_COMPRESSION_DEPTH_MASK) <<                          \
                                QAT_COMPRESSION_DEPTH_BITPOS )  |            \
         (((filetype) & QAT_COMPRESSION_FILE_TYPE_MASK) <<                     \
                                QAT_COMPRESSION_FILE_TYPE_BITPOS ) ) 
                                

/* ========================================================================= */
/*                                            TRANSLATOR SLICE */
/* ========================================================================= */

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported lfct1 enabled values
 * @description
 *      Enumeration used to define lfct1 counter is enabled or not
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_TRANSLATOR_LFCT1_DISABLED=0,                /*!< LFCT1 Disabled */
   ICP_QAT_HW_TRANSLATOR_LFCT1_ENABLED=1,                 /*!< LFCT1 Enabled */
   ICP_QAT_HW_TRANSLATOR_LFCT1_DELIMITER=2                /**< Delimiter type */
} icp_qat_hw_translator_lfct1_enabled_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported lfct0 enabled values
 * @description
 *      Enumeration used to define lfct0 counter is enabled or not
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_TRANSLATOR_LFCT0_DISABLED=0,                /*!< LFCT0 Disabled */
   ICP_QAT_HW_TRANSLATOR_LFCT0_ENABLED=1,                 /*!< LFCT0 Enabled */
   ICP_QAT_HW_TRANSLATOR_LFCT0_DELIMITER=2                /**< Delimiter type */
} icp_qat_hw_translator_lfct0_enabled_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported lfct0 reset values
 * @description
 *      Enumeration used to define lfct0 reset is enabled or not
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_TRANSLATOR_LFCT0_RESET_ON=0,           /*!< LFCT0 Reset Enabled */
   ICP_QAT_HW_TRANSLATOR_LFCT0_RESET_OFF=1,          /*!< LFCT0 Reset Disabled */
   ICP_QAT_HW_TRANSLATOR_LFCT0_RESET_DELIMITER=2     /**< Delimiter type */
} icp_qat_hw_translator_lfct0_reset_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Definition of the supported lfct selection values
 * @description
 *      Enumeration used to define which lfct counters are selected
 *
 *****************************************************************************/

typedef enum
{
   ICP_QAT_HW_TRANSLATOR_SELECT_LFCT0=0,               /*!< Select LFCT0 */
   ICP_QAT_HW_TRANSLATOR_SELECT_LFCT1=1,               /*!< Select LFCT1 */
   ICP_QAT_HW_TRANSLATOR_SELECT_DELIMITER=2            /**< Delimiter type */
} icp_qat_hw_translator_lfct_select_t;


/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Translator Configuration Struct
 *
 * @description
 *      Configuration data used for setting up the QAT Translator Slice
 *
 *****************************************************************************/

typedef struct icp_qat_hw_translator_config_s
{
   uint32_t val;
   /**< Translator slice configuration */

   uint32_t reserved;
   /**< Reserved */
} icp_qat_hw_translator_config_t;


/* Private defines */
#define QAT_TRANSLATOR_LFCT1_ENABLED_BITPOS           21
/**< @ingroup icp_qat_hw_defs
 * Define for the translator lfct1 enabled bit position */

#define QAT_TRANSLATOR_LFCT1_ENABLED_MASK             0x1
/**< @ingroup icp_qat_hw_defs
 * Define for the translator lfct1 enabled mask (one bit) */
 
#define QAT_TRANSLATOR_LFCT0_ENABLED_BITPOS           20
/**< @ingroup icp_qat_hw_defs
 * Define for the translator lfct0 enabled bit position */

#define QAT_TRANSLATOR_LFCT0_ENABLED_MASK             0x1
/**< @ingroup icp_qat_hw_defs
 * Define for the translator lfct0 enabled mask (one bit) */
 
 #define QAT_TRANSLATOR_LFCT0_RESET_BITPOS            28
/**< @ingroup icp_qat_hw_defs
 * Define for the translator lfct0 reset bit position */

#define QAT_TRANSLATOR_LFCT0_RESET_MASK               0x1
/**< @ingroup icp_qat_hw_defs
 * Define for the translator lfct0 reset mask (one bit) */
 
 #define QAT_TRANSLATOR_LFCT_SELECT_BITPOS            27
/**< @ingroup icp_qat_hw_defs
 * Define for the translator lfct select bit position */

#define QAT_TRANSLATOR_LFCT_SELECT_MASK               0x1
/**< @ingroup icp_qat_hw_defs
 * Define for the translator lfct select mask (one bit) */
 
/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Build the translator slice configuration field
 *
 * @param lfct1_enable  Specify whether lfct1 counters are enabled
 * @param lfct0_enable  Specify whether lfct0 counters are enabled
 * @param lfct0_reset   Specify whether to reset counters after unload
 * @param lfct_select   Specify which counter to select for commands
 *
 *****************************************************************************/
#define ICP_QAT_HW_TRANSLATOR_CONFIG_BUILD(lfct1_enable,lfct0_enable,        \
                                                    lfct0_reset,lfct_select) \
         ((((lfct1_enable) & QAT_TRANSLATOR_LFCT1_ENABLED_MASK) <<           \
                                QAT_TRANSLATOR_LFCT1_ENABLED_BITPOS )  |     \
         (((lfct0_enable) & QAT_TRANSLATOR_LFCT0_ENABLED_MASK) <<            \
                                QAT_TRANSLATOR_LFCT0_ENABLED_BITPOS )  |     \
         (((lfct0_reset) & QAT_TRANSLATOR_LFCT0_RESET_MASK) <<               \
                                QAT_TRANSLATOR_LFCT0_RESET_BITPOS )    |     \
         (((lfct_select) & QAT_TRANSLATOR_LFCT_SELECT_MASK) <<               \
                                QAT_TRANSLATOR_LFCT_SELECT_BITPOS ) )


/* ========================================================================= */
/*                                            RegEx SLICE */
/* ========================================================================= */

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      Skinny Virtual Address
 *
 * @description
 *      The virtual address in Skinny Memory. The structure is not intended to 
 *      be used on its own.
 *
 *****************************************************************************/
typedef struct icp_qat_hw_regex_skinny_addr_s
{
   uint32_t addr;
   /**< 32 bit address */
   
   uint32_t reserved;
   /**< Reserved */
   
} icp_qat_hw_regex_skinny_addr_t;

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      RegEx CSR register
 *
 * @description
 *      64 bit CSR register in the RegEx slice.
 *
 *****************************************************************************/
typedef union icp_qat_hw_regex_csr_s
{
   uint64_t reg_64;
   /**< 64 bit value */
   
   struct 
   {
       uint32_t reg_32;
       /**< 32 bit value */
       
       uint32_t reserved;
       /**< Reserved */
   } s1;
   /**< Reserved */
   
} icp_qat_hw_regex_csr_t;


#define QAT_REGEX_HISTORY_LENGTH_MASK              0x1FFF
 /**< @ingroup icp_qat_hw_defs
 * Define for the RegEx history length mask (13 bits) */

/**
 ******************************************************************************
 * @ingroup icp_qat_hw_defs
 *
 * @description
 *      Extract the RegEx history length
 *
 * @param reg     Register to extract length from
 *
 *****************************************************************************/
 
#define ICP_QAT_HW_REGEX_HISTORY_LEN_GET(reg)         \
   ((reg) & QAT_REGEX_HISTORY_LENGTH_MASK)
                                              

/**
 *****************************************************************************
 * @ingroup icp_qat_hw_defs
 *      RegEx DB CSRs struct
 *
 * @description
 *     Data Base CSRs used for setting up the QAT RegEx Slice.
 *     This definition is a reference only as DB CSRs are build by the compiler.
 *     IA driver should be only interested in reading history length field.
 *
 *****************************************************************************/
typedef struct icp_qat_hw_regex_db_csrs_s
{
   icp_qat_hw_regex_skinny_addr_t slow_skinny_job_base_addr;
   /**< Base address of skinny machine transitions with the
    *     fast/slow selector set to 0. */
   
   icp_qat_hw_regex_skinny_addr_t fast_skinny_job_base_addr;
   /**< Base address of skinny machine transitions with the
    *     fast/slow selector set to 1. */
    
   icp_qat_hw_regex_skinny_addr_t one_byte_repeated_skinny_job_base_addr;
   /**< 1B repeated skinny Job Base Address*/
   
   icp_qat_hw_regex_skinny_addr_t one_byte_skinny_job_base_addr;
   /**< 1B skinny Job Base Address*/
   
   icp_qat_hw_regex_csr_t history_length;
   /**< History length. When Clear indicates there is no history
    *     in the input data buffer. When a non-zero value is programmed
    *     it indicates that the number of bytes in the input data buffer
    *     are part of this  history/replay for the fragment. */
   
   icp_qat_hw_regex_csr_t max_returned_result_len;
   /**< The length of the maximum problem data sequence that can be 
    *   returned as a match. */
   
   icp_qat_hw_regex_csr_t max_backward_scan_len;
   /**< The length of the maximum problem data sequence that can be scanned 
     *  backward from the start of the pattern, at the time the job first 
     *  entered the skinny machine from the bushy and hash machines.*/
     
   icp_qat_hw_regex_csr_t max_backward_scan_from_end_len;
   /**< The length of the maximum problem data sequence that can be scanned 
    *  backward from the end of the pattern, at any given point in the pattern
    */
   
   icp_qat_hw_regex_csr_t overflow_replay_limit;
   /**< The number of bytes that must be added to the input data buffer before 
    *   a job that has entered the overflow array is reinserted into the skinny
    *   job queue.*/
   
   icp_qat_hw_regex_csr_t slice_control_1;
   /**< Controls several miscellaneous functions in the Slice. */
   
   icp_qat_hw_regex_csr_t partial_match_ctrl;
   /**< Controls partial match functions in the Slice. */
   
   icp_qat_hw_regex_csr_t pdb_id;
   /**< The ID of the pattern data base */
   
} icp_qat_hw_regex_db_csrs_t;

              
#endif /* __ICP_QAT_HW_H__ */

