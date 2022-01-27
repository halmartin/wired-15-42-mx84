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
 ***************************************************************************
 * @file lac_dh_interface_check.c
 *
 * @ingroup Lac_Dh
 *
 * This file checks at compile time that the IA/FW interface is as expected
 * For example, in lac_dh_data_path.c we use the fact that the index of g in the
 * structure icp_qat_fw_mmp_dh_768_input_t is equal to the index of g in the
 * structure icp_qat_fw_mmp_dh_768_input_t. If this assumption becomes invalid
 * (FW interface changes to break the assumption or the compiler moves things
 * around) this file will fail to compile.
 *
 * Note for strutures with only 1 member no check is required.
 *
 ***************************************************************************/


#include "cpa.h"

#include "icp_qat_fw_pke.h"
#include "icp_qat_fw_mmp.h"
#include "lac_common.h"

#define COMPILE_TIME_ASSERT(pred) \
        switch(0){case 0: case pred:;}


void LacDh_CompileTimeAssertions(void)
{
    /* *************************************************************
     * DH Checks
     * ************************************************************* */

    /* Check that icp_qat_fw_mmp_dh_768_input_t,
       icp_qat_fw_mmp_dh_1024_input_t, icp_qat_fw_mmp_dh_1536_input_t,
       icp_qat_fw_mmp_dh_2048_input_t, icp_qat_fw_mmp_dh_3072_input_t,
       icp_qat_fw_mmp_dh_4096_input_t structures are equivalent */
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,g)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_1024_input_t,g));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_1024_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_1024_input_t,m));

    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,g)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_1536_input_t,g));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_1536_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_1536_input_t,m));

    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,g)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_2048_input_t,g));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_2048_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_2048_input_t,m));

    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,g)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_3072_input_t,g));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_3072_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_3072_input_t,m));

    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,g)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_4096_input_t,g));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_4096_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_4096_input_t,m));


    /* *************************************************************
     * DH G2 Checks
     * ************************************************************* */
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_1024_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_1024_input_t,m));

    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_1536_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_1536_input_t,m));

    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_2048_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_2048_input_t,m));

    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_3072_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_3072_input_t,m));

    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,e)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_4096_input_t,e));
    COMPILE_TIME_ASSERT(LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_768_input_t,m)
                  == LAC_IDX_OF(icp_qat_fw_mmp_dh_g2_4096_input_t,m));


}
