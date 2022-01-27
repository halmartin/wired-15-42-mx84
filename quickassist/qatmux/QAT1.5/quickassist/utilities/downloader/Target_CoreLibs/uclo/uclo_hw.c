/**
 **************************************************************************
 * @file uclo_hw.c
 *
 * @description
 *      This file provides Ucode Object File Loader facilities
 *
 * @par 
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
 **************************************************************************/ 

#include "halAeApi.h"
#include "uclo_helper.h"

int UcLo_isProfileSupported(void);
int UcLo_isChipNameMatch(char *name);

void UcLo_preIntrCallback(void);
void UcLo_postIntrCallback(void);
void UcLo_kickStartArbiter(icp_firml_handle_t *handle, unsigned int hwAeNum);


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *      check if profile is supported
 *
 * @param 
 * @param 
 *
 * @retval  FALSE
 * 
 * 
 *****************************************************************************/
int UcLo_isProfileSupported(void)
{
    return FALSE;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *      check if the input parameter matches chip name
 *
 * @param name - IN
 * @param 
 *
 * @retval  TRUE 
 * 
 * 
 *****************************************************************************/
int UcLo_isChipNameMatch(char *name)
{
    return TRUE;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *      pre-processing for interrupt callback
 *
 * @param 
 * @param 
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
void UcLo_preIntrCallback(void)
{
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *      post-processing for interrupt callback
 *
 * @param 
 * @param 
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
void UcLo_postIntrCallback(void)
{
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *      Kick-start the context arbiter in order to get the next context
 *               to swap in properly after hitting a soft breakpoint.
 *
 * @param hwAeNum - IN
 * @param 
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
void UcLo_kickStartArbiter(icp_firml_handle_t *handle, unsigned int hwAeNum)
{
    return;
}

