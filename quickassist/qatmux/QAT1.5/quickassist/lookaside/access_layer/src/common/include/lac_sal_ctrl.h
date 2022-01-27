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
 * @file lac_sal_ctrl.h
 *
 * @ingroup SalCtrl
 *
 * Functions to register and deregister qat and service controllers with ADF.
 *
 ***************************************************************************/

#ifndef LAC_SAL_CTRL_H
#define LAC_SAL_CTRL_H


/************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function registers an internal event handler function and
 *    name with ADF for the QAT controller.
 *    It calls the ADF function to register the handler.
 *
 * @context
 *      This function is called from the kernel space module init function.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 *************************************************************************/

CpaStatus
SalCtrl_AdfQatRegister(void);

/**********************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function unregisters the QAT component from ADF.
 *
 * @context
 *      This function is called from the kernel space module exit function.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 *********************************************************************/

CpaStatus
SalCtrl_AdfQatUnregister(void);


/**********************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function registers an internal event handler function and
 *    name with ADF for the SAL controller.
 *    It calls the ADF function to register the handler.
 *
 * @context
 *      This function is called from the kernel module init function
 *      and the icp_sal_userStart() function in user space.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 ***********************************************************************/

CpaStatus SalCtrl_AdfServicesRegister(void);

/******************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function unregisters the service component from ADF.
 *
 * @context
 *      This function is called from the kernel module exit function
 *      and icp_sal_userStop() function in user space.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 *******************************************************************/

CpaStatus SalCtrl_AdfServicesUnregister(void);

/*******************************************************************
 * @ingroup SalCtrl
 * @description
 *    This function is used to check whether the service component
 *    has been successfully started.
 *
 * @context
 *      This function is called from the icp_sal_userStart() function.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 ******************************************************************/

CpaStatus SalCtrl_AdfServicesStartedCheck(void);

/*******************************************************************
 * @ingroup SalCtrl
 * @description
 *    This function is used to check whether the user's parameter
 *    for concurrent request is valid. 
 *
 * @context
 *      This function is called when crypto or compression is init
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      Yes 
 * @threadSafe
 *      Yes
 *
 ******************************************************************/
CpaStatus validateConcurrRequest(Cpa32U numConcurrRequests);

#endif
