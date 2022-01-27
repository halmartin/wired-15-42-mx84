/**
 **************************************************************************
 * @file halAeDrv.c
 *
 * @description
 *      This file provides implementation of driver framework
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
 
#include "halAe_platform.h"
#include "core_io.h"
#include "icp_firml_interface.h"
#include "halAeDrv.h"
#include "halAe.h"
#include "halMemScrub.h"

/* AE interrupt types */
#define HALAE_INTR_NUM_TYPES (HALAE_INTR_ATTN_BKPT+1)


void 
intrInit(icp_firml_handle_t *handle);
void 
intrCleanup(icp_firml_handle_t *handle);
int
halAe_checkSINT(icp_firml_handle_t *handle);
int 
halAe_setUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int freeAddr, 
                       unsigned int freeSize);
int 
halAe_getUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int *freeAddr, 
                       unsigned int *freeSize);

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
*     Get a REQUEST structure for the purpose of blocking on an 
 *    ISR event. Use a Structure on the g_available queue if available
 *    else allocate using malloc
 *
 *    Initialize a semaphore for a wait state  
 * 
 * @param type_mask - IN
 * @param masks - OUT
 *
 * @retval
 * 
 * 
 *****************************************************************************/
static REQUEST_T *
getRequest(icp_firml_handle_t *handle,
            unsigned int type_mask, 
            Hal_IntrMasks_T *masks)
{
    REQUEST_T *request;

    if (handle->halHandle->g_available) 
    {
        request = handle->halHandle->g_available;
        handle->halHandle->g_available = request->next;
    } 
    else 
    {
        if(!(request = (REQUEST_T*) osalMemAlloc(sizeof(REQUEST_T)))) 
        {
            return (NULL);
        }

        osalMemSet(request, 0, sizeof(REQUEST_T));

        if(osalSemaphoreInit(&request->sem, 1)) 
        {
            osalMemFree(request);
            return (NULL);
        }
        osalSemaphoreWaitInterruptible(&request->sem, OSAL_WAIT_FOREVER);
    }

    request->type_mask = type_mask;
    request->masks = masks;
    request->status = HALAE_SUCCESS;
    return (request);
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
*    Free a REQUEST structure to the g_available queue 
 * 
 * @param request - IN
 *
 * @retval
 * 
 * 
 *****************************************************************************/
static void 
freeRequest(icp_firml_handle_t *handle, 
             REQUEST_T* request)
{
    request->next = handle->halHandle->g_available;
    handle->halHandle->g_available = request;
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *    Free REQUEST structures on the REQUEST queue, defined by list
 * 
 * @param list - IN
 *
 * @retval
 * 
 * 
 *****************************************************************************/
static void 
cleanupRequests(REQUEST_T* *list)
{
    REQUEST_T *request, *next;

    for (request = *list; request; request = next) 
    {
        next = request->next;
        osalSemaphoreDestroy(&request->sem);
        osalMemFree(request);
    } /* end for request */
    *list = NULL;
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *    Initialize GLOBAL queues and status
 * 
 *
 * @retval
 * 
 * 
 *****************************************************************************/
void 
intrInit(icp_firml_handle_t *handle)
{
    /* Summary of INTR_TYPEs with ATTN flags set */ 
    handle->halHandle->g_summary_mask = 0;
    /*  INTR_TYPEs enabled */
    handle->halHandle->g_enabled_mask = 0;
    osalMemSet(&handle->halHandle->g_intr_masks, 
               0, sizeof(handle->halHandle->g_intr_masks));
    /* Outstanding REQUEST queue */    
    handle->halHandle->g_requests = NULL;
    /* Free REQUEST structure queue */    
    handle->halHandle->g_available = NULL;
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
*     Free REQUEST structures on the global REQUEST queues
 *
 *    Initialize a semaphore for a wait state  
 * 
 * @param type_mask - IN
 * @param masks - OUT
 *
 * @retval
 * 
 * 
 *****************************************************************************/
void 
intrCleanup(icp_firml_handle_t *handle)
{
    cleanupRequests(&handle->halHandle->g_available);
    cleanupRequests(&handle->halHandle->g_requests);
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
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
int 
halAe_IntrPoll(icp_firml_handle_t *handle,
               unsigned int type_mask, 
               Hal_IntrMasks_T  *masks)
{
    unsigned long lock_level;
    unsigned int type_summary_mask;
    REQUEST_T *request;
    int status;

    if ((type_mask == 0) || (type_mask & ~((1 << HALAE_INTR_NUM_TYPES)-1))) 
    {
        ERRINFO(("type_mask=0x%x\n ", type_mask)); 
        return (HALAE_FAIL);
    }
    SPIN_LOCK_IRQSAVE(handle->halHandle->bk_lock, lock_level);
    {
        /* This test needs to be inside protection to make sure
           that the interrupt is not disabled after we check it and
           before we int Lock */
        if (0 == (type_mask & handle->halHandle->g_enabled_mask)) 
        {
            SPIN_UNLOCK_IRQRESTORE(handle->halHandle->bk_lock, lock_level);

            /* this is really saying that the interrupt is disabled */
            return (HALAE_DISABLED);    
        }

        type_summary_mask = (type_mask & handle->halHandle->g_summary_mask);
        if (type_summary_mask) 
        {
            osalMemSet(masks, 0, sizeof(Hal_IntrMasks_T));
            if (type_summary_mask & HALAE_INTR_ATTN_BKPT_MASK) 
            {
                masks->attn_bkpt_mask = 
                        handle->halHandle->g_intr_masks.attn_bkpt_mask;
                handle->halHandle->g_intr_masks.attn_bkpt_mask = 0;
            }
            handle->halHandle->g_summary_mask &= ~type_mask;
            SPIN_UNLOCK_IRQRESTORE(handle->halHandle->bk_lock, lock_level);
            return (HALAE_SUCCESS);
        } 
    }
    SPIN_UNLOCK_IRQRESTORE(handle->halHandle->bk_lock, lock_level);
   
    /* getRequest can put the current process to sleep.
     * So, put it out of spin lock to make sure it is reentrant.
     */
        if(!(request = getRequest(handle, type_mask, masks)))
        {
           ERRINFO(("IntrPoll getRequest return NULL\n")); 
           return (HALAE_FAIL);
        }

    SPIN_LOCK_IRQSAVE(handle->halHandle->bk_lock, lock_level);
        request->next = handle->halHandle->g_requests;
        handle->halHandle->g_requests = request;
    SPIN_UNLOCK_IRQRESTORE(handle->halHandle->bk_lock, lock_level);
        
    osalSemaphoreWaitInterruptible(&request->sem, OSAL_WAIT_FOREVER);
    status = request->status;

    DBGINFO(("IntrPoll API UnBlocked with status = %x\n", status));
    SPIN_LOCK_IRQSAVE(handle->halHandle->bk_lock, lock_level);
    freeRequest(handle, request);
    SPIN_UNLOCK_IRQRESTORE(handle->halHandle->bk_lock, lock_level);

    return (status);
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *    Process IRQ_AE_ATTN interrupts(called by ISR)
 *    Interrupts are cleared and attn flags are passed to caller via
 *    p_attn_bkpt_mask
 * 
 * @param p_attn_bkpt_mask - IN
 *
 * @retval
 * 
 * 
 *****************************************************************************/
static int 
handleAttnIntrIcp(icp_firml_handle_t *handle, 
                     unsigned int *p_attn_bkpt_mask)
{
    int ii, ae;
    unsigned int aeAttnRawStat, aeAttnMask, ctxEn;
    unsigned int attn_bkpt_mask;
    unsigned int bpt_hit = 0;
    unsigned long flags;

    aeAttnRawStat = READ_LWORD(handle->Hal_cap_global_ctl_csr_virtAddr + 
                            CAP_RAW_ATTN_STATUS);
    
    aeAttnMask = READ_LWORD(handle->Hal_cap_global_ctl_csr_virtAddr + 
                            CAP_ATTN_MASK);

    /* see if nothing needs to be done */
    if ((aeAttnRawStat & aeAttnMask) == 0) 
    {
        return 1;
    }
    attn_bkpt_mask = 0;
    SPIN_LOCK_IRQSAVE(handle->halHandle->bpt_lock, flags);
    
    /* see which AE needs attention */
    for (ii=0; ii<MAX_AE; ii++) 
    {
        if (!(handle->sysMemInfo.aeMask & (1 << ii))) 
        {
            continue;
        }
        if (!(aeAttnMask & (1 << ii))) 
        {
            continue;
        }
        ae = AE_NUM(ii);
        ctxEn = GET_AE_CSR(handle, ae, CTX_ENABLES);

        if (ctxEn & CE_BREAKPOINT_BIT) 
        {
            attn_bkpt_mask |= (1 << ae);
            bpt_hit = 1;
        }
        else 
        {
            continue;
        }

        /* prevent clearing the W1C bits for error: 
         * ECC error bit, and Parity error bit */
        ctxEn &= IGNORE_W1C_ERR_MASK;            

        /* clear breakpoint interrupt, interrupt bits are W1C */
        SET_AE_CSR(handle, ae, CTX_ENABLES, ctxEn);        
    } 
    SPIN_UNLOCK_IRQRESTORE(handle->halHandle->bpt_lock, flags);

    *p_attn_bkpt_mask = attn_bkpt_mask;
    if (bpt_hit) 
    {
        return 0;
    }
    return 1;
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *    Wakeup all REQUEST structures whose REQUEST->type_mask is satisfied
 *    by the the INTR types reflected in the global g_summary_mask. The
 *    actual attention flags corresponding the the satisfied requests are
 *    are copied to the REQUEST->masks[] words. The satisfied INTR types
 *    in g_summary_mask are cleared.
 *
 *    The threads associated with the satifisfied REQUEST structures are
 *    unblocked via REQUEST->sem.
 *    The following must be called in an ISR or when interrupts are disabled
 * 
 *
 * @retval
 * 
 * 
 *****************************************************************************/
static void 
processRequests(icp_firml_handle_t *handle)
{
    REQUEST_T *request, *prev_req, *next_req;
    unsigned int mask;
    unsigned long lock_level;
    
    SPIN_LOCK_IRQSAVE(handle->halHandle->tasklet_lock, lock_level);
    
    for (prev_req = NULL, request = handle->halHandle->g_requests; request; 
         prev_req = request, request = next_req) 
    {
        next_req = request->next;
        if(0 == 
           (mask = (request->type_mask & handle->halHandle->g_summary_mask))) 
        {
            continue;
        }
        /* Remove request from list */
        if (prev_req == NULL) 
        {
            handle->halHandle->g_requests = next_req;
        }    
        else
        {
            prev_req->next = next_req;
        }

        osalMemSet(request->masks, 0, sizeof(Hal_IntrMasks_T));

        if (mask & HALAE_INTR_ATTN_BKPT_MASK) 
        {
            request->masks->attn_bkpt_mask = 
                    handle->halHandle->g_intr_masks.attn_bkpt_mask;
            handle->halHandle->g_intr_masks.attn_bkpt_mask = 0;
        }
        osalSemaphorePostWakeup(&request->sem);
    
        /* On loop, don't change prev_req */
        request = prev_req;

        handle->halHandle->g_summary_mask &= ~(mask);
        if (handle->halHandle->g_summary_mask == 0) 
        {
            break;
        }    
    } /* end for request */
    
    SPIN_UNLOCK_IRQRESTORE(handle->halHandle->tasklet_lock, lock_level);
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *    Wakeup all REQUEST structures whose REQUEST->type_mask is satisfied
 *    by the the INTR types NOT reflected in the global g_enabled_mask. 
 *
 *    The threads associated with the satifisfied REQUEST structures are
 *    unblocked via REQUEST->sem.
 *    The following must be called in an ISR or when interrupts are disabled
 * 
 *
 * @retval
 * 
 * 
 *****************************************************************************/
static void 
processDisabledRequests(icp_firml_handle_t *handle)
{
    REQUEST_T *request, *prev_req, *next_req;
    unsigned long lock_level;
    
    SPIN_LOCK_IRQSAVE(handle->halHandle->tasklet_lock, lock_level);

    for (prev_req = NULL, request = handle->halHandle->g_requests; request; 
         prev_req = request, request = next_req) 
    {
        next_req = request->next;

        /* don't remove requests only if all requested interrupts are still
           enabled */
        if (!(request->type_mask & ~handle->halHandle->g_enabled_mask)) 
        {
            continue;
        }
        /* Remove request from list */
        if (prev_req == NULL) 
        {
            handle->halHandle->g_requests = next_req;
        }    
        else
        {
            prev_req->next = next_req;
        }
        request->status = HALAE_DISABLED;

        DBGINFO(("Waking Disabled IntrPoll API\n"));
        osalSemaphorePostWakeup(&request->sem);

        /* On loop, don't change prev_req */
        request = prev_req;

    } /* end for request */
    
    SPIN_UNLOCK_IRQRESTORE(handle->halHandle->tasklet_lock, lock_level);
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *    ISR for IRQ_AE_ATTN. Calls handle_intr_icp to clear and
 *    return flags.
 *    Updates g_intr_masks and g_summary_mask as per returned ATTN flags
 *
 * @param irq - IN
 * @param dev_id - IN
 * @param regs - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
int 
icp_FirmLoader_ISR(void *handle)
{
    unsigned int attn_bkpt_mask;
    icp_firml_handle_t *myhandle = (icp_firml_handle_t *)handle;
    int rc = 0;
    
    if(myhandle == NULL)
    {
        return (ICP_FIRMLOADER_FAIL);
    }
    if(halAe_checkSINT(myhandle) != 0)
    {
        return (ICP_FIRMLOADER_FAIL);
    }    

    attn_bkpt_mask = 0;
    /* handle attn interrupt */
    rc = handleAttnIntrIcp(myhandle, &attn_bkpt_mask);
    if(rc)
    {
        return (ICP_FIRMLOADER_FAIL);
    }
   
    myhandle->halHandle->g_intr_masks.attn_bkpt_mask |= attn_bkpt_mask;

    if (0 != myhandle->halHandle->g_intr_masks.attn_bkpt_mask) 
    {
        myhandle->halHandle->g_summary_mask |= HALAE_INTR_ATTN_BKPT_MASK;
    }    
     
    processRequests(myhandle);            

    return (ICP_FIRMLOADER_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *      Enables breakpoint and enables interrupt 
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
 * @param type_mask - IN Specifies the mask type must be the 
 *                       value HALAE_INTR_ATTN_BKPT_MASK 
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval system error number
 * 
 * 
 *****************************************************************************/
int 
halAe_IntrEnable(icp_firml_handle_t *handle, 
                 unsigned int   type_mask)
{
    unsigned long lock_level;
    unsigned long addr;
    
    /* all AE interrupts share a interrupt number, so call request_irq just once
       and ISR dispath interrupts
    */
    SPIN_LOCK_IRQSAVE(handle->halHandle->bk_lock, lock_level);
    {
       /* Don't enable interrupts that are already enabled */
       type_mask &= ~handle->halHandle->g_enabled_mask;

       handle->halHandle->g_enabled_mask |= type_mask;

       /* Clear summary mask */
       handle->halHandle->g_summary_mask &= ~type_mask;

       if(type_mask & HALAE_INTR_ATTN_BKPT_MASK) 
       {
           handle->halHandle->g_intr_masks.attn_bkpt_mask = 0;

           addr = 
               (unsigned long)(handle->Hal_cap_global_ctl_csr_virtAddr
                               + CAP_ATTN_MASK_SET);
           WRITE_LWORD(addr, (AE_ATTN_ALLBITS & handle->sysMemInfo.aeMask));
       }
    }
    SPIN_UNLOCK_IRQRESTORE(handle->halHandle->bk_lock, lock_level);

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *      Enables breakpoint and enables interrupt 
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
 * @param type_mask - IN Specifies the mask type must be the 
 *                       value HALAE_INTR_ATTN_BKPT_MASK
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval system error number
 * 
 * 
 *****************************************************************************/
int 
halAe_IntrDisable(icp_firml_handle_t *handle, 
                  unsigned int type_mask)
{
    unsigned long lock_level;
    unsigned long addr;

    SPIN_LOCK_IRQSAVE(handle->halHandle->bk_lock, lock_level);
    {
        /* Don't Disable interrupts that are not enabled */
        type_mask &= handle->halHandle->g_enabled_mask;

        handle->halHandle->g_enabled_mask &= ~type_mask;
        handle->halHandle->g_summary_mask &= ~type_mask;

        if (type_mask & HALAE_INTR_ATTN_BKPT_MASK) 
        {
            handle->halHandle->g_intr_masks.attn_bkpt_mask = 0;

            addr = (unsigned long)(handle->Hal_cap_global_ctl_csr_virtAddr + 
                                  CAP_ATTN_MASK_CLR);
            WRITE_LWORD(addr, (AE_ATTN_ALLBITS & handle->sysMemInfo.aeMask));
        }

        processDisabledRequests(handle);
    }
    SPIN_UNLOCK_IRQRESTORE(handle->halHandle->bk_lock, lock_level);

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *   set memory start offset
 * 
 * @param scratchOffset - IN
 * @param sramOffset - IN
 * @param ncdramOffset - IN
 * @param cdramOffset - IN
 *
 * @retval HALAE_SUCCESS or error number
 * 
 * 
 *****************************************************************************/
int 
halAe_SetMemoryStartOffset(icp_firml_handle_t *handle, 
                           unsigned int scratchOffset, 
                           unsigned int sramOffset, 
                           unsigned int dram0Offset, 
                           unsigned int dram1Offset)
{
   handle->sysMemInfo.scratchDesc.startOffset = scratchOffset;
   handle->sysMemInfo.sramDesc.startOffset = sramOffset;
   handle->sysMemInfo.dramDesc[0].startOffset = dram0Offset;
   handle->sysMemInfo.dramDesc[1].startOffset = dram1Offset;
    
   return (HALAE_SUCCESS);    
}         

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
 * 
 * @description
 *   get memory start offset
 * 
 * @param scratchOffset - OUT
 * @param sramOffset - OUT
 * @param ncdramOffset - OUT
 * @param cdramOffset - OUT
 *
 * @retval HALAE_SUCCESS or error number
 * 
 * 
 *****************************************************************************/
int 
halAe_GetMemoryStartOffset(icp_firml_handle_t *handle, 
                           unsigned int *scratchOffset, 
                           unsigned int *sramOffset, 
                           unsigned int *dram0Offset, 
                           unsigned int *dram1Offset)
{
   *scratchOffset = handle->sysMemInfo.scratchDesc.startOffset;
   *sramOffset = handle->sysMemInfo.sramDesc.startOffset;
   *dram0Offset = (unsigned int)handle->sysMemInfo.dramDesc[0].startOffset;
   *dram1Offset = (unsigned int)handle->sysMemInfo.dramDesc[1].startOffset; 

   return (HALAE_SUCCESS);       
}    

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param begFreeAddr - IN Specifies the microstore address where the free 
                        region begins
 * @param size - IN Indicates the number of free microstore words
 *
 * @retval HALAE_SUCCESS or error number
 * 
 * 
 *****************************************************************************/
int 
halAe_setUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int freeAddr, 
                       unsigned int freeSize)
{
    AE(handle, ae).freeAddr = freeAddr;
    AE(handle, ae).freeSize = freeSize;

    return HALAE_SUCCESS;    
}

/**
 *****************************************************************************
 * @ingroup icp_ae_loader_kernel
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param begFreeAddr - OUT A pointer to the location of the beginning of 
 *                          the free region for the specified accelaration engine 
 *                          microstore.                        
 * @param size - OUT A pointer to the location of the size of the free region
 *                   for the specified accelaration engine microstore
 *
 * @retval HALAE_SUCCESS or error number
 * 
 * 
 *****************************************************************************/
int 
halAe_getUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int *freeAddr, 
                       unsigned int *freeSize)
{
    *freeAddr = AE(handle, ae).freeAddr;
    *freeSize = AE(handle, ae).freeSize;
    return HALAE_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup icp_hal_debug
 * 
 * @description
 *     read PCI configuraion header CSR
 *
 * @param handle - IN
 * @param offset - IN
 * @param data - OUT
 *
 * @retval HALAE_SUCCESS, HALAE_FAIL, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
int 
halAe_getPciCsr(icp_firml_handle_t *handle,
                unsigned int offset,
                unsigned int numBytes,
                unsigned int *data)
{
    struct pci_dev *pdev = NULL;
    int status = HALAE_SUCCESS;
    unsigned int value_dword;
    unsigned short value_word;
    unsigned char  value_byte;

    pdev = (struct pci_dev *)handle->sysMemInfo.dev;

    if(pdev)
    {
        switch(numBytes)
        {
            case 0x4:
               pci_read_config_dword(pdev,(offset & 0xFFFFFFFC),&value_dword);
               *data = value_dword;
               break;
            case 0x2:
               pci_read_config_word(pdev,(offset & 0xFFFFFFFE),&value_word);
               *data = ((unsigned int)value_word) & 0xFFFF;
               break;
            case 1:
               pci_read_config_byte(pdev,offset,&value_byte);
               *data = ((unsigned int)value_byte) & 0xFF;
            break;
          default:
               status = HALAE_BADARG;
        }
    }

    return(status);
}

/**
 *****************************************************************************
 * @ingroup icp_hal_debug
 * 
 * @description
 *     write PCI configuraion header CSR
 *
 * @param handle - IN
 * @param offset - IN
 * @param data - IN
 *
 * @retval HALAE_SUCCESS, HALAE_FAIL, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
int halAe_putPciCsr(icp_firml_handle_t *handle,
                    unsigned int offset,
                    unsigned int numBytes,
                    unsigned int data)
{
    struct pci_dev *pdev = NULL;
    int status = HALAE_SUCCESS;
    unsigned short value_word;
    unsigned char  value_byte;    

    pdev = (struct pci_dev *)handle->sysMemInfo.dev;

    if(pdev)
    {
        switch(numBytes)
        {
            case 0x4:
               pci_write_config_dword(pdev,(offset & 0xFFFFFFFC),data);
               break;
            case 0x2:
               value_word = (unsigned short)(data & 0xFFFF);
               pci_write_config_word(pdev,(offset & 0xFFFFFFFE),value_word);
               break;
            case 0x1:
               value_byte = data & 0xFF;
               pci_write_config_byte(pdev,offset,value_byte); 
               break;
            default:
               status = HALAE_BADARG;
        }
    }

    return(status);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      
 * @param
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
int
halAe_ContinuousDramAlloc(icp_firml_handle_t *Handle,
                          icp_firml_dram_desc_t *pDram_desc,
                          unsigned int size)
{
    void *ptr = NULL;

    pDram_desc->dramBaseAddr_v = (uint64)((long)dma_alloc_coherent(
         &(((struct pci_dev *)Handle->sysMemInfo.dev)->dev),
         size,
         (dma_addr_t *)&ptr,
         GFP_KERNEL));

    if (ptr == NULL)
    {
        return HALAE_FAIL;
    }
    
    pDram_desc->dramBaseAddr   = (uint64)((long)ptr);
    pDram_desc->dramBusAddr   = pDram_desc->dramBaseAddr ;
    pDram_desc->dramSize = size;
    pDram_desc->startOffset = 0;
    
    return HALAE_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      
 * @param
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
void
halAe_ContinuousDramFree(icp_firml_handle_t *Handle,
                         icp_firml_dram_desc_t *pDram_desc)
{
    dma_free_coherent(&(((struct pci_dev *)Handle->sysMemInfo.dev)->dev), 
                        (size_t)(pDram_desc->dramSize),
                        (void *)((long)pDram_desc->dramBaseAddr_v),
                        (long)pDram_desc->dramBusAddr);
    osalMemSet (pDram_desc, sizeof(icp_firml_dram_desc_t), 0);
}


