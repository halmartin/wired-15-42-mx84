/**
 **************************************************************************
 * @file halAeApi.h
 *
 * @description
 *      This is the header file for Hardware Abstraction Layer
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

/*
 ****************************************************************************
 * Doxygen group definitions
 ****************************************************************************/

/**
 *****************************************************************************
 * @file halAeApi.h
 * 
 * @defgroup icp_hal Hardware Abstraction Layer
 *
 * @description
 *      This header file that contains the prototypes and definitions required
 *      for Hardware Abstraction Layer
 *
 *****************************************************************************/
 
#ifndef __HAL_AEAPI_H
#define __HAL_AEAPI_H

#include "core_io.h"
#include "icp_firml_handle.h"

#define MAX_EXEC_INST           100
#define HALAE_INVALID_XADDR 0xffffffffffffffffULL

#include "hal_ae.h"
#include "ae_constants.h"

enum{
   HALAE_SUCCESS=0,         /**< the operation was successful */
   HALAE_FAIL=0x8100,       /**< the operation failed */
   HALAE_BADARG,            /**< bad function argument */
   HALAE_DUPBKP,            /**< duplicate break point */
   HALAE_NOUSTORE,          /**< not ustore available */
   HALAE_BADADDR,           /**< bad address */
   HALAE_BADLIB,            /**< bad debug library -- wasn't initialized */
   HALAE_DISABLED,          /**< acceleration engine or interrupt disabled */
   HALAE_ENABLED,           /**< acceleration engine enabled */
   HALAE_RESET,             /**< acceleration engine is in reset */
   HALAE_TIMEOUT,           /**< the operation execced th etime limit */
   HALAE_ISSET,             /**< condition/evaluation is set/true */
   HALAE_NOTSET,            /**< condition/evaluation is not set/false */
   HALAE_AEACTIVE,          /**< ae is running */
   HALAE_MEMALLOC,          /**< memory allocation error */
   HALAE_NEIGHAEACTIVE      /**< neighbour ae is running */
};

enum{
    HALAE_UNINIT,           /**< HAL library uninitialized */ 
    HALAE_CLR_RST,          /**< HAL library clear reset status */  
    HALAE_RST               /**< HAL library reset status */  
};

/* Define Interrupt types and type masks */
#define HALAE_INTR_ATTN_BKPT   0    /**< AE attention interrupt */                            
#define HALAE_INTR_ATTN_BKPT_MASK    (1 << HALAE_INTR_ATTN_BKPT)   /**< AE attention interrupt mask */

/* Define New Page callback mechanism */
typedef enum {               /**< Argument0        Argument1       */
    START_OF_PAGE_CHANGE,    /**< N/A              N/A             */
    NEW_PAGE_LOADED,         /**< new page num     old page num    */
    END_OF_PAGE_CHANGE,      /**< N/A              N/A             */
    WAITING_FOR_PAGE_CHANGE  /**< ctx              Virt Addr or -1 */
} Hal_PageChangeReason_T;
typedef void (*HalAePageChangeCallback_T)(icp_firml_handle_t *dbgHandle,
                                          Hal_PageChangeReason_T reason,
                                          unsigned int           hwAeNum,
                                          unsigned int           arg0,
                                          unsigned int           arg1,
                                          void*                  user_data);

typedef enum {              /**< Argument0   Argument1            */
    UPDATE_AE_ENABLES,      /**< AeMask      ptr to AeEnablesMask */
    PAUSING_AES             /**< AeMask                           */
} Hal_UcloCallReason_T;
typedef void (*HalAe_UcloCall_T)(Hal_UcloCallReason_T reason,
                                 unsigned int         arg0,
                                 unsigned int*        arg1,
                                 void*                user_data);

typedef struct batch_init_s{
    unsigned int    hwAe;
    unsigned int    addr;
    unsigned int    *value;
    unsigned int    size;
    struct batch_init_s *next;
}batch_init_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Initializes the acceleration engines and takes them out of reset. 
 *      acceleration engines with their corresponding bit set in aeMask are initialized
 *      to the following state:
 *      1. Context mode is set to eight
 *      2. Program counters are set to zero
 *      3. Next context to run is set to zero
 *      4. All contexts are disabled
 *      5. cc_enable is set to 0x2000
 *      6. Wakeup events are set to one
 *      7. The signal event is set to zero
 *      All other acceleration engines remain untouched thought they are taken out of 
 *      reset. This function should be called prior to calling most of the HAL
 *      functions.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param aeMask - IN The bits correspond to the accelaration engine number and address
 *                    of acceleration engines to be initialized. For example, the first
 *                    accelaration engine in the second cluster corresponds to bit 16.
 *                    write the value uWord at the address uAddr.
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_Init(icp_firml_handle_t *handle, 
           unsigned int aeMask);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Restores the system resources allocated by the halAe_Init() function. 
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC void 
halAe_DelLib(icp_firml_handle_t *handle);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determines if the specified accelaration engine is valid, whether it is 
 *      initialized, or whether it is in or out of reset.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 *
 * @retval HALAE_UNINIT A bad debug library is associated with the specified
 *                      accelaration engine the library is not initialized
 * @retval HALAE_CLR_RST The accelaration engine is not in a reset state
 * @retval HALAE_RST The accelaration engine is in reset state
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetAeState(icp_firml_handle_t *handle,
                 unsigned char ae);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set a mask of the contexts that are loaded.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxMask - IN The context or contexts to set to alive
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_SetLiveCtx(icp_firml_handle_t *handle, 
                 unsigned char ae, 
                 unsigned int ctxMask);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get a mask of the contexts that are loaded.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxMask - OUT A pointer referencec to the alive context or contexts
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetLiveCtx(icp_firml_handle_t *handle, 
                 unsigned char ae, 
                 unsigned int *ctxMask);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Defines a region of microstore that is unused. 
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param begFreeAddr - IN Specifies the microstore address where the free 
                        region begins
 * @param size - IN Indicates the number of free microstore words
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_SetUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int begFreeAddr, 
                       unsigned int size);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Returns the starting address and size of the specified accelaration engine 
 *      microstore free region
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param begFreeAddr - OUT A pointer to the location of the beginning of 
 *                          the free region for the specified accelaration engine 
 *                          microstore.                        
 * @param size - OUT A pointer to the location of the size of the free region
 *                   for the specified accelaration engine microstore
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int *begFreeAddr, 
                       unsigned int *size);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determines if the specified accelaration engine is enabled to run and if any
 *      of its contexts are either running or waiting to run
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_DISABLED accelaration engine or interrupt disabled 
 * @retval HALAE_BADLIB A bad debug library is associated with the specified
                        accelaration engine the library is not initialized
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_IsAeEnabled(icp_firml_handle_t *handle, 
                  unsigned char ae);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the wake-event to voluntary and starts the accelaration engine context 
 *      specified by the ae and startCtx parameters from that accelaration engine’s 
 *      current program counter. The ctxEnMask specifies the contexts to be 
 *      enabled. If one of these acceleration engines is in reset it is taken out 
 *      of reset.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxEnMask - IN Specifies the contexts to be enabled
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB A bad debug library is associated with the specified
                        accelaration engine the library is not initialized
 * @retval HALAE_BADARG Bad function argument
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_Start(icp_firml_handle_t *handle, 
            unsigned char ae, 
            unsigned int ctxEnMask);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Stops the accelaration engine contexts that have a corresponding bit 
 *      set in the ctxMask parameter at the next context arbitration 
 *      instruction. The context may not stop because it never executes a 
 *      context arbitration instruction. A value of HALAE_RESET is returned 
 *      if the accelaration engine is in reset.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxEnMask - IN Specifies the contexts to stop
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_Stop(icp_firml_handle_t *handle, 
           unsigned char ae, 
		   unsigned int ctxMask);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Resets the specified acceleration engines with a corresponding bit set in 
 *      aeMask. If clrReg is set then the acceleration engines are initialized to 
 *      the states described in halAe_Init().
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param aeMask - IN Specifies the acceleration engines of interest
 * @param clrReg - IN If this parameter is set, then following register
 *                    initialization is performed:
 *                    1. CTX_ENABLES are set to zero
 *                    2. CTX_ARB_CNTL is set to zero
 *                    3. CC_ENABLE is set to x2000
 *                    4. All context program counters are set to zero
 *                    5. WAKEUP_EVENTS are set to one
 *                    6. SIG_EVENTS are set to zero
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC void 
halAe_Reset(icp_firml_handle_t *handle, 
            unsigned int aeMask, 
			int clrReg);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Takes the specified acceleration engines out of the reset state.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param aeMask - IN Specifies the acceleration engines of interest
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_ClrReset(icp_firml_handle_t *handle, 
               unsigned int aeMask);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the accelaration engine context arbitration control register and returns
 *      the value in the ctxArbCtl argument.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxArbCtl - OUT A pointer to the location of the specified context 
 *                        arbitration control register
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 *
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetCtxArb(icp_firml_handle_t *handle, 
                unsigned char ae, 
			    unsigned int *ctxArbCtl);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword ctxArbCtl value to the accelaration engine context 
 *      arbitration control register for the specified accelaration engine.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxArbCtl - IN The new value for the context arbitration 
 *                       control register
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutCtxArb(icp_firml_handle_t *handle, 
                unsigned char ae, 
			    unsigned int ctxArbCtl);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the accelaration engine control status register indicated by csr and 
 *      returns the value in the value parameter. The csr value must be a valid
 *      accelaration engine CSR offset.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param csr - IN A valid accelaration engine CSR offset
 * @param value - OUT A pointer to the location of the requested accelaration engine 
 *                    control status register
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetAeCsr(icp_firml_handle_t *handle, 
               unsigned char ae, 
               unsigned int csr, 
               unsigned int *value);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the longword value to the accelaration engine CSR indicated by csr 
 *      parameter. The csr value must be a valid AE CSR offset.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param csr - IN A valid accelaration engine CSR offset
 * @param value - IN The new value for the specified accelaration engine control status
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutAeCsr(icp_firml_handle_t *handle, 
               unsigned char ae, 
               unsigned int csr, 
               unsigned int value);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the local-memory mode to relative or global.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param lmType - IN Specifies the local memory bank. 
                   This value is ICP_LMEM0 or ICP_LMEM1
 * @param mode - IN Specifies the local memory mode. This value is one of:
 *               1. zero - the memory mode is relative
 *               2. one - the memory mode is global
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutAeLmMode(icp_firml_handle_t *handle, 
                  unsigned char ae, 
                  icp_RegType_T lmType, 
                  unsigned char mode);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Gets the accelaration engine context mode to either four or eight
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param ae - IN Specifies the accelaration engine of interest
 *
 * @retval ctx mode FOUR_CTX or EIGHT_CTX
 * 
 * 
 *****************************************************************************/
HAL_DECLSPEC int 
halAe_GetAeCtxMode(icp_firml_handle_t *handle, 
                   unsigned char ae);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the accelaration engine context mode to either four or eight
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param mode - IN Specifies the context mode. This value must be one of {4, 8}
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutAeCtxMode(icp_firml_handle_t *handle, 
                   unsigned char ae,
			       unsigned char mode);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the accelaration engine next-neighbor mode to either write to itself or to
 *      its neighbor.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param mode - IN The next-neighbor mode to set. This value is one of:
 *                  1. zero - the next-neighbor mode is write to self
 *                  2. one - the next-neighbor mode is write to neighbor
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutAeNnMode(icp_firml_handle_t *handle, 
                  unsigned char ae, 
			      unsigned char mode);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set the shared ustore mode on or off for the specified AE and its 
 *      neighbor's.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param mode - IN Specifies the mode to set
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutAeSharedCsMode(icp_firml_handle_t *handle, 
                        unsigned char ae, 
			            unsigned char mode);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determines if the specified accelaration engine is valid, whether it is 
 *      initialized, or whether it is in or out of reset.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param mode - OUT A pointer to shared control store mode
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetAeSharedCsMode(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned char *mode);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the neighbor of a shareable-ustore pair.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param aeNeigh - OUT A pointer to neighboring accelaration engine
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int
halAe_GetSharedUstoreNeigh(icp_firml_handle_t *handle, 
                           unsigned char ae, 
                           unsigned char *aeNeigh);


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the state of the accelaration engine context wakeup-event signals.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the accelaration engine context of interest
 * @param events - OUT A pointer to the location of the wakeup-event signals 
 *                     for the specified acceleration engines and context
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetCtxWakeupEvents(icp_firml_handle_t *handle, 
                         unsigned char ae, 
                         unsigned char ctx, 
                         unsigned int *events);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the longword events to the accelaration engine context wakeup-events 
 *      CSR. Only the context with a corresponding bit set in ctxMask are 
 *      written; unspecified contexts remains unchanged.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the accelaration engine context of interest
 * @param events - OUT The longword specifying the new wakeup-event signals
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutCtxWakeupEvents(icp_firml_handle_t *handle, 
                         unsigned char ae, 
                         unsigned int ctxMask, 
                         unsigned int events);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the signal events for the specified accelaration engine context
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the accelaration engine context of interest.
 * @param events - IN A pointer to the location of the current signal events 
 *                    for the specified accelaration engine context
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetCtxSigEvents(icp_firml_handle_t *handle, 
                      unsigned char ae, 
                      unsigned char ctx, 
                      unsigned int *events);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the longword value of the event parameter to the signal events 
 *      of the specified accelaration engine contexts.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxMask - IN Specifies the accelaration engine context or contexts
 * @param events - IN The longword specifying the new signal events
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutCtxSigEvents(icp_firml_handle_t *handle, 
                      unsigned char ae, 
                      unsigned int ctxMask, 
                      unsigned int events);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the context-status CSR of the contexts specified by the ctxMask 
 *      parameter.The accelaration engine must be stopped before calling this function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine to write to
 * @param ctxMask - IN Specifies the context or contexts to write to.
 * @param ctxStatus - IN Specifies the longword value to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutCtxStatus(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned int ctxMask, 
                   unsigned int ctxStatus);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the context-status CSR for the context specified by the ctx 
 *      parameter.The accelaration engine must be stopped before calling this function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine to write to
 * @param ctxMask - IN Specifies the context or contexts to write to.
 * @param ctxStatus - OUT A pointer to the location at which to return the 
 *                        longword value read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetCtxStatus(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned char ctx, 
                   unsigned int *ctxStatus);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a 32-bit value to the accelaration-engine context(s) indirect CSR. 
 *      It's unsafe to call this function while the AE is enabled.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxMask - IN Specifies the contexts CSR to be written
 * @param aeCsr - IN The indirect CSR to be written. Must be one of the
 *                following:
 *                1. CTX_FUTURE_COUNT_INDIRECT
 *                2. CTX_WAKEUP_EVENTS_INDIRECT
 *                3. CTX_STS_INDIRECT
 *                4. CTX_SIG_EVENTS_INDIRECT
 *                5. LM_ADDR_0_INDIRECT
 *                6. LM_ADDR_1_INDIRECT
 * @param csrVal - IN Specifies the longword value to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutCtxIndrCsr(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned int ctxMask, 
                    unsigned int aeCsr, 
                    unsigned int csrVal);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a 32-bit value to the accelaration-engine context(s) indirect CSR. 
 *      It's unsafe to call this function while the AE is enabled.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxMask - IN Specifies the contexts CSR to be written
 * @param aeCsr - IN The indirect CSR to be written. Must be one of the
 *                following:
 *                1. CTX_FUTURE_COUNT_INDIRECT
 *                2. CTX_WAKEUP_EVENTS_INDIRECT
 *                3. CTX_STS_INDIRECT
 *                4. CTX_SIG_EVENTS_INDIRECT
 *                5. LM_ADDR_0_INDIRECT
 *                6. LM_ADDR_1_INDIRECT
 * @param csrVal - IN Specifies the longword value to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetCtxIndrCsr(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned char ctx, 
                    unsigned int aeCsr, 
                    unsigned int *csrVal);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the CPU Type and product maj/min revisions, and return
 *      the CPU type, and the combined maj and min revision.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param prodType - IN A pointer to the location of product type
 * @param prodRev - IN A pointer to the location of product revision
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetProdInfo(icp_firml_handle_t *handle, 
                  unsigned int *prodType, 
                  unsigned int *prodRev);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the indicated accelaration engine context program counters to the value 
 *      specified by the upc parameter.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctxMask - IN Specifies the accelaration engine context or contexts
 * @param upc - IN The new program counter value
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutPC(icp_firml_handle_t *handle, 
            unsigned char ae, 
            unsigned int ctxMask, 
            unsigned int upc);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Returns the program counter for the specified accelaration engine context
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the accelaration engine context of interest
 * @param upc - OUT A pointer to the location of the requested program 
 *                  counter value
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetPC(icp_firml_handle_t *handle, 
            unsigned char ae, 
            unsigned char ctx, 
            unsigned int *upc);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a number of microword to the specified accelaration engine. The 
 *      accelaration engine must be disabled and out of reset before writing to its 
 *      microstore.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param uWordAddr - IN The microstore address specifying the start of the 
 *                       write operation
 * @param numWords - IN The number of microwords to write
 * @param uWord - IN A pointer to the location of the microword to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int
halAe_PutUwords(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int uWordAddr, 
                unsigned int numWords, 
                uword_T *uWord);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads and returns the number of microword from the specified 
 *      accelaration engine microstore. acceleration engines must be disabled before reading 
 *      from the microstore.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param uWordAddr - IN The microstore address specifying the start of the 
 *                       write operation
 * @param numWords - IN The number of microwords to write
 * @param uWord - OUT A pointer to the location of the requested microwords
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetUwords(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int uWordAddr, 
                unsigned int numWords, 
                uword_T *uWord);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      writes a number of longwords to the microstore. The accelaration engine must 
 *      be inactive, possibly under reset, before writing to the microstore.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param uWordAddr - IN The microstore address specifying the start of the 
 *                       write operation
 * @param numWords - IN The number of microwords to write
 * @param data - IN A pointer to the location of the longwords to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutUmem(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned int uWordAddr, 
              unsigned int numWords, 
              unsigned int *data);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      reads a number of longwords to the microstore memory specified by the 
 *      uAddr word-address. The accelaration engine must be inactive before reading 
 *      from microstore.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param uWordAddr - IN The microstore address specifying the start of the 
 *                       write operation
 * @param numWords - IN The number of microwords to write
 * @param data - OUT A pointer to the location of the requested longwords
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetUmem(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned int uWordAddr, 
              unsigned int numWords, 
              unsigned int *data);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword from the relative accelaration engine general purpose register
 *      specified by the ae, ctx, regType, and regNum parameters. The 
 *      accelaration engine must be disabled before calling this function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the context
 * @param regType - IN Specifies the type of register to read from. This must 
 *                  be one of {ICP_GPA_REL,ICP_GPB_REL, ICP_DR_RD_REL, 
 *                  ICP_SR_RD_REL, ICP_DR_WR_REL,
 *                  ICP_SR_WR_REL, ICP_NEIGH_REL}.
 * @param regNum - IN Specifies the register number to read from. The register 
 *                 number is relative to the context. Therefore, this number 
 *                 is in the range of 0 through max - 1, where max is the 
 *                 maximum number of context-relative registers for the type 
 *                 specified by regType argument and the context mode
 *
 * @param regData - OUT A pointer to the location of the data to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetRelDataReg(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned char ctx, 
                    icp_RegType_T regType,
                    unsigned short regNum, 
                    unsigned int *regData);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword to the relative accelaration engine general purpose register
 *      specified by the ae, ctx, regType, and regNum parameters. The 
 *      accelaration engine must be disabled before calling this function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the context
 * @param regType - IN Specifies the type of register to read from. This must 
 *                  be one of {ICP_GPA_REL,ICP_GPB_REL, ICP_DR_RD_REL, 
 *                  ICP_SR_RD_REL, ICP_DR_WR_REL,
 *                  ICP_SR_WR_REL, ICP_NEIGH_REL}.
 * @param regNum - IN Specifies the register number to read from. The register 
 *                 number is relative to the context. Therefore, this number 
 *                 is in the range of 0 through max - 1, where max is the 
 *                 maximum number of context-relative registers for the type 
 *                 specified by regType argument and the context mode
 * @param regData - IN The longword to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutRelDataReg(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned char ctx, 
                    icp_RegType_T regType,
                    unsigned short regNum, 
                    unsigned int regData);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword from the accelaration engine absolute general purpose register
 *      specified by the ae, ctx, regType, and regNum parameters. The 
 *      accelaration engine must be disabled before calling this function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param regType - IN Specifies the type of register to read from. This must 
 *                  be one of {ICP_GPA_ABS, ICP_GPB_ABS, ICP_DR_RD_ABS, 
 *                  ICP_SR_RD_ABS, ICP_DR_WR_ABS,
 *                  ICP_SR_WR_ABS, ICP_NEIGH_ABS}.
 * @param absRegNum - IN Specifies the register number to read from
 * @param regData - OUT A pointer to the location of the data to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetAbsDataReg(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    icp_RegType_T regType,
                    unsigned short absRegNum, 
                    unsigned int *regData);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword to the accelaration engine absolute general purpose register
 *      specified by the ae, regType, and regNum parameters. The 
 *      accelaration engine must be disabled before calling this function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param regType - IN Specifies the type of register to read from. This must 
 *                  be one of {ICP_GPA_ABS, ICP_GPB_ABS, ICP_DR_RD_ABS, 
 *                  ICP_SR_RD_ABS, ICP_DR_WR_ABS,
 *                  ICP_SR_WR_ABS, ICP_NEIGH_ABS}.
 * @param absRegNum - IN Specifies the register number to write
 * @param regData - IN The longword to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutAbsDataReg(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    icp_RegType_T regType,
                    unsigned short absRegNum, 
                    unsigned int regData);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read a generic data register of any type.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the context
 * @param regType - IN Specifies the register type
 * @param regAddr - IN Specifies the register address offset
 * @param regData - IN A pointer to the location of the data to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetDataReg(icp_firml_handle_t *handle, 
                 unsigned char hwAeNum, 
                 unsigned char ctx,
                 icp_RegType_T regType,
                 unsigned short regAddr, 
                 unsigned int *regData);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword value to the specified accelaration engine’s local memory 
 *      location specified by the lmaddr word address.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param lmAddr - IN Local memory word address
 * @param value - IN The longword to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int
halAe_PutLM(icp_firml_handle_t *handle, 
            unsigned char ae, 
            unsigned short lmAddr, 
            unsigned int value);

/**
 *****************************************************************************
 * @ingroup icp_hal
 *
 * @description
 *     Calculate the number of instruction to init Local memory
 *
 * @param LMByteSize - IN
 *
 * @retval the number of the instruction number
 *
 *****************************************************************************/
int
halAe_GetInstrucNum(int LMByteSize);

/**
 *****************************************************************************
 * @ingroup icp_hal
 *
 * @description
 *      Batch writes to the specified accelaration engine~Rs local memory
 *      location specified by a link of the lmaddr word address.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param handle - IN AC firmware loader handle
 * @param ae - IN Specifies the accelaration engine of interest
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument
 *
 *
 *****************************************************************************/

HAL_DECLSPEC int
halAe_BatchPutLM(icp_firml_handle_t *handle,
            unsigned char ae,
            batch_init_t *lm_init_header);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword value from the specified accelaration engine local memory 
 *      location specified by the lmAddr word address.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param lmAddr - IN Local memory word address
 * @param value - IN A pointer to the location of location of the longword 
 *                read from the specified register
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetLM(icp_firml_handle_t *handle, 
            unsigned char ae, 
            unsigned short lmAddr, 
            unsigned int *value);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the longword data to the specified accelaration engine CAM entry and 
 *      marks it as the most recently used. The entry and state must be in 
 *      the range of zero through fifteen.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param entry - IN A pointer to the location of the CAM entry ID
 * @param data - IN The CAM data to write
 * @param state - IN The value of the state for the specified CAM entry
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutCAM(icp_firml_handle_t *handle, 
             unsigned char ae, 
             unsigned char entry, 
             unsigned int data,
             unsigned char state);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the entire contents of the accelaration engine CAM, that is, sixteen CAM 
 *      entries and stores the tag value and status in the appropriate parameters.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param tags - OUT An array of 16 unsigned integers to receive the CAM 
 *                   tag-data of each CAM entry
 * @param state - OUT An array of 16 chars to receive the CAM state-bits of 
 *                    each CAM entry
 * @param lru - OUT A pointer to the location of the array describing the 
 *                  order of use of the CAM entries with values of zero through 
 *                  fifteen. The entry with a zero value is the Least Recently Used. 
 *                  The entry with the value of fifteen is the Most Recently Used.
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetCAMState(icp_firml_handle_t *handle, 
                  unsigned char ae, 
                  unsigned int *tags,
                  unsigned char *state, 
                  unsigned char *lru);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determines if the accelaration engine is in the state specified by inpState 
 *      and returns a value indicating whether it is set, not set, or an error 
 *      occurred. There are 16 values for inpState, and their meaning is 
 *      specific to the chip implementation. 
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param inpState - IN A chip-specific value denoting a accelaration engine state
 * 
 * @retval HALAE_ISSET The value is set
 * @retval HALAE_NOTSET The value is not set
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_IsInpStateSet(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned char inpState);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Checks that the value specified by the ae parameter is a valid 
 *      accelaration engine number. The library must be initialized before calling 
 *      this function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_VerifyAe(icp_firml_handle_t *handle, 
               unsigned char ae);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Checks that the value specified by the ae parameter is a valid 
 *      accelaration engine number. The library must be initialized before calling 
 *      this function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param aeMask - IN Specifies the acceleration engines of interest
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
HAL_DECLSPEC int 
halAe_VerifyAeMask(icp_firml_handle_t *handle, 
                   unsigned int aeMask);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Stops the timestamp clock and zeroes the timestamps of all specified 
 *      acceleration engines then restarts the timestamp clock.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param aeMask - IN Specifies one or more acceleration engines
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_ResetTimestamp(icp_firml_handle_t *handle, 
                     unsigned int aeMask);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Waits until one or more of the interrupts specified by type_mask 
 *      occurs, or returns immediately if there are any outstanding interrupts 
 *      of the specified type. Interrupts must be enabled by halAe_IntrEnable() 
 *      or else any attempt to poll returns HALAE_DISABLED.
 *        
 *      Upon return, the mask parameter contains a bit mask of the acceleration engines 
 *      or threads which have an interrupt of the type or types specified in 
 *      the poll request. The masks may be zero in the case of a 
 *      halAe_IntrDisable() request. The interrupt is cleared before the call 
 *      returns.

 *      In the event that multiple threads were blocked on the same interrupt, 
 *      when that interrupt occurs, only one poll call completes. There is no 
 *      guarantee which of the calls completes.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param type_mask - IN Specifies the mask type must be the 
 *                       value HALAE_INTR_ATTN_BKPT_MASK
 * @param masks - OUT On input specifies the location where the mask is to be
                     returned. On output the returned mask
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_DISABLED accelaration engine or interrupt disabled 
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_IntrPoll(icp_firml_handle_t *handle,
			   unsigned int type_mask,      
               Hal_IntrMasks_T  *masks);    

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Enables breakpoint, and enables interrupt 
 *      processing by the HAL. If the HAL interrupt processing is not enabled, 
 *      then the application is free to manage the particular interrupts in 
 *      an OS dependent manner.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param type_mask - IN Specifies the mask type must be the 
 *                       value HALAE_INTR_ATTN_BKPT_MASK
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval system error number
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_IntrEnable(icp_firml_handle_t *handle, 
                 unsigned int type_mask);          

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Disables breakpoint interrupt.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param type_mask - IN Specifies the mask type must be the 
 *                       value HALAE_INTR_ATTN_BKPT_MASK
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval system error number
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_IntrDisable(icp_firml_handle_t *handle, 
                  unsigned int type_mask);         

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      This function is similar to halAe_IntrPoll() except that it does not 
 *      block waiting for the specified interrupts. Instead, it returns 
 *      immediately and invokes the specified callback function for subsequent
 *      occurrences of the interrupt or interrupts. The callback data are 
 *      passed to the callback function on invocation. A handle associated 
 *      with  this call is returned and this handle can be used to terminate
 *      the request and disregard the specified interrupts.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param type_mask - IN Specifies the mask type must be the 
 *                       value HALAE_INTR_ATTN_BKPT_MASK
 * @param callback_func - IN A pointer to the location of a callback function.
 * @param callback_data - IN A pointer to the location of application-specific 
 *                           data. This pointer is passed to the callback 
 *                           function when it is invoked
 * @param thd_priority - IN The thread priority
 * @param handle - IN A handle associated with and returned by this call. 
 *                    This handle can be used to terminate the request and 
 *                    disregard the specified interrupts
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_SpawnIntrCallbackThd(unsigned int type_mask,
                           HalAeIntrCallback_T callback_func,
                           void*               callback_data,
                           int                 thd_priority,
                           void*              *int_handle);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      This function is similar to halAe_IntrPoll() except that it does not 
 *      block waiting for the specified interrupts. Instead, it returns 
 *      immediately and invokes the specified callback function for subsequent
 *      occurrences of the interrupt or interrupts. The callback data are 
 *      passed to the callback function on invocation. A handle associated 
 *      with  this call is returned and this handle can be used to terminate
 *      the request and disregard the specified interrupts.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param type_mask - IN Specifies the mask type must be the 
 *                       value HALAE_INTR_ATTN_BKPT_MASK
 * @param callback_func - IN A pointer to the location of a callback function
 * @param callback_data - IN A pointer to the location of application-specific 
 *                           data. This pointer is passed to the callback 
 *                           function when it is invoked
 * @param thd_priority - IN The thread priority
 * @param callback_priority - IN The callback priority
 * @param handle - IN A handle associated with and returned by this call. 
 *                    This handle can be used to terminate the request and 
 *                    disregard the specified interrupts
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/

/* halAe_SpawnIntrCallbackThdEx() is used when chaining interrupts.
   Higher priority callbacks are made first */
HAL_DECLSPEC int 
halAe_SpawnIntrCallbackThdEx(unsigned int type_mask,
                             HalAeIntrCallback_T callback_func,
                             void*               callback_data,
                             int                 thd_priority,
                             unsigned int        callback_priority,
                             void*               *int_handle);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Cancels a request initiated by the halAe_SpawnIntrCallbackThd() 
 *      function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param int_handle - IN The handle returned by a call to 
 *                    halAe_SpawnIntrCallbackThd(). This handle
 *                    specifies which request to terminate.
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_TerminateCallbackThd(icp_firml_handle_t *handle, 
                           void* int_handle);


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Register new page callback function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param callback_func - IN A pointer to the location of a callback function
 * @param user_data - IN A pointer to user input callback function data
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

/* The New Page Callback is defined globally and not on a per-chip basis
   (when running under simulation) */
HAL_DECLSPEC void
halAe_DefineNewPageCallback(HalAePageChangeCallback_T callback_func,
                            void*                     user_data);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Calls new page callback function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param reason - IN Specifies the reason why callback function is invoked. 
 *                    Must be one of the following values
 *                    {START_OF_PAGE_CHANGE, NEW_PAGE_LOADED,
 *                     END_OF_PAGE_CHANGE, WAITING_FOR_PAGE_CHANGE} 
 *                      
 * @param hwAeNum - IN Specifies the accelaration engine of interest
 * @param new_page_num - IN Specifies the new page number
 * @param old_page_num - IN Specifies the old page number
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC void 
halAe_CallNewPageCallback(icp_firml_handle_t *handle,
                          Hal_PageChangeReason_T reason,
                          unsigned int           hwAeNum,
                          unsigned int           new_page_num,
                          unsigned int           old_page_num);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Defines UCLO callback function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param callback_func - IN A pointer to the location of a callback function
 * @param user_data - IN A pointer to user input callback function data
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

/* The UCLO callbacks can be defined multiple times. Each one is called in an
   arbitrary order */
HAL_DECLSPEC void
halAe_DefineUcloCallback(HalAe_UcloCall_T callback_func,
                         void*            user_data);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Deletes UCLO callback function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param callback_func - IN A pointer to the location of a callback function
 * @param user_data - IN A pointer to user input callback function data
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC void
halAe_DeleteUcloCallback(HalAe_UcloCall_T callback_func,
                         void*            user_data);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Calls UCLO callback function.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param reason - IN Specifies the reason why callback function is invoked. 
 *                    Must be one of the following values
 *                    {UPDATE_AE_ENABLES, PAUSING_AES,
 *                     END_OF_PAGE_CHANGE, WAITING_FOR_PAGE_CHANGE} 
 * @param arg0 - IN Callback function argument 
 * @param arg1 - IN Callback function argument 
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC void 
halAe_CallUcloCallback(Hal_UcloCallReason_T reason,
                       unsigned int         arg0,
                       unsigned int*        arg1);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Convert from a virtual to a physical uword address.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param hwAeNum - IN Specifies the accelaration engine of interest
 * @param v_addr - IN Specifies the uword virtual address
 * @param p_addr - OUT A pointer to uwrod physical address
 * @param loaded - OUT A pointer to indicate wether or not the address is 
 *                     currently in ustore
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADADDR Bad address
 * 
 * 
 *****************************************************************************/

/* In halAe_MapVirtToPhysUaddr(), the value returned in loaded is a boolean
   flag, either zero or non-zero */
HAL_DECLSPEC int 
halAe_MapVirtToPhysUaddr(icp_firml_handle_t *handle, 
                         unsigned char hwAeNum,
                         unsigned int  v_addr,
                         unsigned int *p_addr,
                         unsigned int *loaded);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Convert from a physical to a virtual uword address.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param hwAeNum - IN Specifies the accelaration engine of interest
 * @param p_addr - IN Specifies the uword physical address
 * @param v_addr - OUT A pointer to uwrod virtual address
 * @param loaded - OUT A pointer to indicate wether or not the address is 
 *                     currently in ustore
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADADDR Bad address
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_MapPhysToVirtUaddr(icp_firml_handle_t *handle, 
                         unsigned char hwAeNum,
                         unsigned int  p_addr,
                         unsigned int *v_addr);

/* halAe_SetNumPages() and halAe_SetPageData() are designed to be used
   internally by the loader */
int 
halAe_SetNumPages(icp_firml_handle_t *handle, 
                  unsigned char hwAeNum, 
			      unsigned int numPages);
int 
halAe_SetPageData(icp_firml_handle_t *handle, 
                  unsigned char hwAeNum, 
                  unsigned int page,
                  unsigned int vaddr, 
                  unsigned int paddr, 
                  unsigned int size);

HAL_DECLSPEC void
halAe_DisplayQatSlices(unsigned int *pQatDevEnable);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Convert from a virtual to a physical uword address.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param hwAeNum - IN Specifies the accelaration engine of interest
 * @param v_addr - IN Specifies the uword virtual address
 * @param p_addr - OUT A pointer to uwrod physical address
 * @param loaded - OUT A pointer to indicate wether or not the address is 
 *                     currently in ustore
 * @param page_num - OUT A pointer to indicate page number associated with 
 *                       the physical address
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADADDR Bad address
 * 
 * 
 *****************************************************************************/

int halAe_MapVirtToPhysUaddrEx(icp_firml_handle_t *handle, 
                               unsigned char hwAeNum,
                               unsigned int  v_addr,
                               unsigned int *p_addr,
                               unsigned int *loaded,
                               unsigned int *page_num);

/* internal use only */
HAL_DECLSPEC int 
halAe_SetReloadUstore(icp_firml_handle_t *handle, 
                      unsigned char ae, 
                      unsigned int reloadSize, 
                      int sharedMode, 
                      unsigned int ustoreDramBuf);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a number of long-words to micro-store memory specified such that
 *      the even address goes to the even numbered AE, and the odd address 
 *      goes to the odd numbered AE.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param uAddr - IN Specifies the microstore address to start of the 
 *                   write operation
 * @param numWords - IN Specifies The number of microwords to write
 * @param uWord - IN A pointer to the location of the microwords to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset 
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutCoalesceUwords(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned int uAddr,
                        unsigned int numWords, 
                        uword_T *uWord);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read a number of long-words from micro-store memory such that the 
 *      ven address is taken from even numbered AE, and the odd address is 
 *      taken from the odd numbered AE.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param uAddr - IN Specifies the microstore address to start of the 
 *                   write operation
 * @param numWords - IN Specifies The number of microwords to write
 * @param uWord - OUT A pointer to the location of the microwords to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset 
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetCoalesceUwords(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned int uAddr,
                        unsigned int numWords, 
                        uword_T *uWord);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword value from specified Gige control and status register.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param mac - IN Specifies the MAC number
 * @param csr - IN Specified Gige CSR offset
 * @param value - OUT A pointer to the location of the data to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetGigeCsr(icp_firml_handle_t *handle, 
                 unsigned char ae,
                 unsigned int mac, 
                 unsigned int csr, 
                 unsigned int *value);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword value to specified Gige control and status register.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param mac - IN Specifies the MAC number
 * @param csr - IN Specified Gige CSR offset
 * @param value - IN A data to to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_PutGigeCsr(icp_firml_handle_t *handle, 
                 unsigned char ae,
                 unsigned int mac, 
                 unsigned int csr, 
                 unsigned int value);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a long-word value from the SSU Share Ram location. It's unsafe to 
 *      call this function while the AE is enabled.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param addr - IN Specified shared ram address index
 * @param value - OUT A point to the location of data to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetSharedRam(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned char qat,
                   unsigned int addr, 
                   unsigned int *value);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a long-word value to the SSU Share Ram location. It's unsafe to 
 *      call this function while the AE is enabled.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN Specifies the accelaration engine of interest
 * @param addr - IN Specified shared ram address index
 * @param value - IN A data to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int
halAe_PutSharedRam(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned char qat,
                   unsigned int addr, 
                   unsigned int value);
                   
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word value to the Ram location specified
 *     by the 64-bit addr. It's unsafe to call this function while the AE 
 *     is enabled.
 *
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN
 * @param addr - IN
 * @param value - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int halAe_WriteRam(icp_firml_handle_t *handle,
                          unsigned char ae, 
                          uint64 addr, 
                          unsigned int value);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a long-word value from the Ram location 
 *     specified by the 64-bit addr. It's unsafe to call this function while 
 *     the AE is enabled.
 *
 * @param handle - IN AC firmware loader handle 
 * @param ae - IN
 * @param addr - IN
 * @param value - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/                          
int halAe_ReadRam(icp_firml_handle_t *handle,
                              unsigned char ae, 
                              uint64 addr, 
                              unsigned int *value);
                   
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get dram1 physical base address.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param baseAddr - OUT A point to the location of base address to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_UNINIT A bad debug library is associated with the specified
                        accelaration engine the library is not initialized
 * @retval HALAE_CLR_RST The accelaration engine is not in a reset state
 * @retval HALAE_RST The accelaration engine is in reset state
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int 
halAe_GetDram1BaseAddr(icp_firml_handle_t *handle, 
                       uint64 *baseAddr);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get dram0 physical base address.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN AC firmware loader handle 
 * @param baseAddr - OUT A point to the location of base address to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_UNINIT A bad debug library is associated with the specified
                        accelaration engine the library is not initialized
 * @retval HALAE_CLR_RST The accelaration engine is not in a reset state
 * @retval HALAE_RST The accelaration engine is in reset state
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC int
halAe_GetDram0BaseAddr(icp_firml_handle_t *handle, 
                        uint64 *baseAddr);

/* internal used */
HAL_DECLSPEC int 
halAe_SetMemoryStartOffset(icp_firml_handle_t *handle, 
                           unsigned int scratchOffset, 
                           unsigned int sramOffset, 
                           unsigned int dram0Offset, 
                           unsigned int dram1Offset);

/* internal used */
HAL_DECLSPEC int 
halAe_GetMemoryStartOffset(icp_firml_handle_t *handle, 
                           unsigned int *scratchOffset, 
                           unsigned int *sramOffset, 
                           unsigned int *dram0Offset, 
                           unsigned int *dram1Offset);
/* internal used */
HAL_DECLSPEC int
halAe_GetPciCsr(icp_firml_handle_t *handle,
                unsigned int offset,
                unsigned int numBytes,
                unsigned int *data);
/* internal used */
HAL_DECLSPEC int
halAe_PutPciCsr(icp_firml_handle_t *handle,
                unsigned int offset,
                unsigned int numBytes,
                unsigned int data);

/* internal used */
HAL_DECLSPEC int 
halAe_GetExecutableAe(icp_firml_handle_t *handle,
                      unsigned char *ae);

/* internal used */
HAL_DECLSPEC int 
halAe_WaitNumCycles(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned int cycles);

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      get AC firmware loader handle from hal debug kernel module
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      No
 * 
 * @param instanceId - IN Accel Complex Instance ID
 * @param handle - OUT AC firmware loader handle 
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/
HAL_DECLSPEC int 
halAe_GetHandle(unsigned int instanceId, 
               icp_firml_handle_t **handle);


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the error description string for provided error code.
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param errCode - IN Specifies the error code
 *
 * @retval error description string
 * 
 * 
 *****************************************************************************/

HAL_DECLSPEC char 
*halAe_GetErrorStr(int errCode);

#ifdef __cplusplus
}
#endif

#endif          /* __HAL_AEAPI_H */
