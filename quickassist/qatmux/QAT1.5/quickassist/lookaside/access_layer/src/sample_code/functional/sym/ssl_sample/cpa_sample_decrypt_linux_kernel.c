/***************************************************************************
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
 ***************************************************************************/
#include "cpa.h"
#include <linux/crypto.h>

/* *************************************************************
 *
 * On core crypto for SSL decrypt
 *
 * ************************************************************* */

CpaStatus
sampleCodeAesCbcDecrypt(Cpa8U * pKey, Cpa32U keyLen, Cpa8U *pIv, Cpa8U * pIn, Cpa8U *pOut)
{
   struct crypto_cipher *cipher_tfm = NULL;
   int i=0;

   cipher_tfm = crypto_alloc_cipher("aes", 0, 0);
   if(IS_ERR(cipher_tfm))
   {
      printk("crypto_alloc_cipher aes failed\n");
      return CPA_STATUS_FAIL;
   }

   crypto_cipher_setkey(cipher_tfm, pKey, keyLen);
   crypto_cipher_decrypt_one(cipher_tfm, pOut, pIn);

   /* Xor with IV */
   for(i=0; i<16; i++)
   {
      pOut[i] = pOut[i] ^ pIv[i]; 
   }

   crypto_free_cipher(cipher_tfm);
   return CPA_STATUS_SUCCESS;
}

