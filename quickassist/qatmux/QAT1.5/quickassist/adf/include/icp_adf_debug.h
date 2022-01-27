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

/******************************************************************************
 * @file icp_adf_debug.h
 *
 * @description
 *      This header file that contains the prototypes and definitions required
 *      for ADF debug feature.
 *
*****************************************************************************/
#ifndef ICP_ADF_DEBUG_H
#define ICP_ADF_DEBUG_H

/*
 * debug_dir_info_t
 * Struct which is used to hold information about a debug directory
 * under the proc filesystem.
 * Client should only set name and parent fields.
 */
typedef struct debug_dir_info_s
{
    char *name;
    struct debug_dir_info_s *parent;
    /* The below fields are used internally by the driver */
    struct debug_dir_info_s *dirChildListHead;
    struct debug_dir_info_s *dirChildListTail;
    struct debug_dir_info_s *pNext;
    struct debug_dir_info_s *pPrev;
    struct debug_file_info_s *fileListHead;
    struct debug_file_info_s *fileListTail;
    void *proc_entry;
} debug_dir_info_t;

/*
 * Read handle type for sequential proc file
 * Function can be called more than once. It will be called until the
 * return value is not 0. offset should be used to mark the starting
 * point for next step. In one go function can print up to 4kB (size).
 * Function should return 0 (zero) if all info is printed or
 * offset from where to start in next step.
 */
typedef int (*file_read_seq)(void*private_data, char* buff,
                                                int size, int offset);

/*
 * debug_file_info_t
 * Struct which is used to hold information about a debug file
 * under the proc filesystem.
 * Client should only set name, type, private_data, parent fields
 * and seq_read pointers depending on type used.
 */
typedef struct debug_file_info_s
{
    char *name;
    struct debug_dir_info_s *parent;
    file_read_seq seq_read;
    void *private_data;
    /* The below fields are used internally by the driver */
    struct debug_file_info_s *pNext;
    struct debug_file_info_s *pPrev;
    void *page;
    Cpa32U offset;
    void *proc_entry;
} debug_file_info_t;

/*
 * icp_adf_debugAddDir
 *
 * Description:
 *  Function used by subsystem to register a new
 *  directory under the proc filesystem
 *
 * Returns:
 *   CPA_STATUS_SUCCESS    on success
 *   CPA_STATUS_FAIL       on failure
 */
CpaStatus
icp_adf_debugAddDir(icp_accel_dev_t *accel_dev,
		                 debug_dir_info_t* dir_info);

/*
 * icp_adf_debugRemoveDir
 *
 * Description:
 *  Function used by subsystem to remove an existing
 *  directory for which debug output may be stored
 *  in the proc filesystem.
 *
*/
void
icp_adf_debugRemoveDir(debug_dir_info_t* dir_info);

/*
 * icp_adf_debugAddFile
 *
 * Description:
 *  Function used by subsystem to add a new file under
 *  the proc file system in which debug output may be written
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus
icp_adf_debugAddFile(icp_accel_dev_t *accel_dev,
		     debug_file_info_t* file_info);

/*
 * icp_adf_debugRemoveFile
 *
 * Description:
 *  Function used by subsystem to remove an existing file under
 *  the proc filesystem in which debug output may be written
 *
 */
void
icp_adf_debugRemoveFile(debug_file_info_t* file_info);

#endif /* ICP_ADF_DEBUG_H */

