/****************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
 *
 ***************************************************************************/

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description
*     This file contains inline functions used to measure cost of offload
*
*****************************************************************************/
#ifndef __QAT_PERF_CYCLES_H_
#define __QAT_PERF_CYCLES_H_

#include "icp_sal_poll.h"

/* Global state of initalization */
static Cpa8U coo_initialized = CPA_FALSE;

/* This is used to define a pointer to a function */
typedef CpaStatus (*coo_poll_func)(CpaInstanceHandle, Cpa32U);

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function is used to free all memory
*                                  allocated by coo_init and reset request and
*                                  poll counters
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
****************************************************************************/
static inline void coo_deinit(perf_data_t *perf_data)
{
    if (CPA_CC_REQ_POLL_STAMP == iaCycleCount_g)
    {
        qaeMemFree((void **)(&perf_data->req_cycles));
        perf_data->req_cycles = NULL;
        qaeMemFree((void **)(&perf_data->poll_cycles));
        perf_data->poll_cycles = NULL;
        qaeMemFree((void **)(&perf_data->cost_cycles));
        perf_data->cost_cycles = NULL;
        perf_data->req_count = 0;
        perf_data->poll_count = 0;
        coo_initialized = CPA_FALSE;
    }
    return;
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function is used to allocate memory for coo
*                                  values
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
* @param[in]   size                number of opearations used to allocate
*                                  memory for coo values
*
* @retval CPA_STATUS_SUCCESS       Coo is initialized and all memory was
*                                  allocated
*
* @retval CPA_STATUS_FAIL          Coo is not initialized and memory is freed
*                                  before return of function
*
****************************************************************************/
static inline CpaStatus coo_init(perf_data_t *perf_data, Cpa64U size)
{
    if (CPA_CC_REQ_POLL_STAMP == iaCycleCount_g)
    {
        if (NULL == perf_data->req_cycles && NULL == perf_data->poll_cycles &&
            NULL == perf_data->cost_cycles)
        {
            perf_data->req_cycles = qaeMemAlloc(sizeof(perf_cycles_t) * size);
            perf_data->req_count = 0;
            perf_data->poll_cycles = qaeMemAlloc(sizeof(perf_cycles_t) * size);
            perf_data->poll_count = 0;
            perf_data->cost_cycles =
                qaeMemAlloc(sizeof(perf_cycles_t) * size * 2);
            if (NULL == perf_data->req_cycles ||
                NULL == perf_data->poll_cycles ||
                NULL == perf_data->cost_cycles)
            {
                coo_deinit(perf_data);
                /* report allocation failure */
                return CPA_STATUS_FAIL;
            }
            else
            {
                coo_initialized = CPA_TRUE;
            }
        }
    }
    return CPA_STATUS_SUCCESS;
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function is used to retrive timestamp
*
* @retval      perf_cycles_t       time stamp value
*
****************************************************************************/
static inline perf_cycles_t coo_timestamp(void)
{
#ifdef __x86_64__
    uint32_t cycles_high;
    uint32_t cycles_low;

    asm volatile(
        "rdtscp\n\t"
        "mov %%edx, %0\n\t"
        "mov %%eax, %1\n\t"
        : "=r"(cycles_high), "=r"(cycles_low)::"%rax", "%rbx", "%rcx", "%rdx");

    return (((perf_cycles_t)cycles_high << 32) | cycles_low);
#else
    Cpa64U ts = 0;
    asm volatile("rdtsc" : "=A"(ts));
    return ((perf_cycles_t)ts);
#endif
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function is used to retrive time stamp
*                                  before sending request
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
****************************************************************************/
static inline void coo_req_start(perf_data_t *perf_data)
{
    if (CPA_CC_REQ_POLL_STAMP == iaCycleCount_g && coo_initialized)
    {
        /* Initialize local pointer */
        perf_cycles_t *req_cycles =
            &perf_data->req_cycles[perf_data->req_count];

        /* Start coo measure */
        *req_cycles = coo_timestamp(); /* 1. First timestamp - coo measure */
    }
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function takes time stamp after sending
*                                  request. Difference of start and stop
*                                  (request time) is stored in perf_data. If
*                                  request status is not equal to
*                                  CPA_STATUS_SUCCESS coo value will be
*                                  overwritten by next successful request
*                                  coo value.
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
* @param[in]   status              request status
*
****************************************************************************/
static inline void coo_req_stop(perf_data_t *perf_data, CpaStatus status)
{
    if (CPA_CC_REQ_POLL_STAMP == iaCycleCount_g && coo_initialized)
    {
        perf_cycles_t timestamp_req =
            coo_timestamp(); /* 3. Second timestamp - coo measure */
        /* End coo measure */

        /* Initialize local pointers */
        perf_cycles_t *req_cycles =
            &perf_data->req_cycles[perf_data->req_count];
        perf_cycles_t *cost_cycles =
            &perf_data->cost_cycles[perf_data->cost_count];

        /* Save coo value in perf_data_t */
        *req_cycles = timestamp_req - *req_cycles;

        if (CPA_STATUS_SUCCESS == status)
        {
            /* Start coo cost measure */
            *cost_cycles =
                coo_timestamp(); /* 1. First timestamp - cost measure */
            /* End of function coo_req_start() */
            /* 2. Measured function call */
            /* Start of function coo_req_stop() */
            if (CPA_CC_REQ_POLL_STAMP == iaCycleCount_g && coo_initialized)
            {
                perf_cycles_t timestamp_cost =
                    coo_timestamp(); /* 3. Second timestamp - cost measure */
                /* End coo cost measure */

                /* Save coo cost value in perf_data_t */
                *cost_cycles = timestamp_cost - *cost_cycles;

                /* Increase counters */
                perf_data->req_count++;
                perf_data->cost_count++;
            }
        }
    }
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function performs polling on instance
*                                  and measures coo of polling call
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
* @param[in]   func                pointer to function used to perform polling
*
* @param[in]   instance            instance handle used in polling function
*
* @param[in]   status              pointer to status where polling function
*                                  will return value
*
****************************************************************************/
static inline void coo_poll(perf_data_t *perf_data,
                            coo_poll_func func,
                            CpaInstanceHandle instance,
                            CpaStatus *status)
{
    if (CPA_CC_REQ_POLL_STAMP == iaCycleCount_g && coo_initialized)
    {
        /* Initialize local variables */
        perf_cycles_t *poll_cycles =
            &perf_data->poll_cycles[perf_data->poll_count];
        perf_cycles_t *cost_cycles =
            &perf_data->cost_cycles[perf_data->cost_count];
        perf_cycles_t timestamp = 0;

        /* Start coo measure */
        timestamp = coo_timestamp(); /* 1. First timestamp - coo measure */
        *status = func(instance, 0); /* 2. Measured function call */
        *poll_cycles =
            coo_timestamp() - timestamp; /* 3. Second timestamp - coo measure */
        /* End coo measure */

        if (CPA_STATUS_SUCCESS == *status)
        {
            /* Start coo cost measure */
            timestamp = coo_timestamp(); /* 1. First timestamp - cost measure */
            /* 2. Measured function call */
            *cost_cycles = coo_timestamp() -
                           timestamp; /* 3. Second timestamp - cost measure */
            /* End coo cost measure */

            /* Start coo cost measure */
            perf_data->poll_count++;
            perf_data->cost_count++;
        }
    }
    else
        *status = func(instance, 0);
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function id used to call coo_poll using
*                                  polling function in traditional mode for
*                                  crypto instances
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
* @param[in]   instance            instance handle used in polling function
*
* @param[in]   status              pointer to status where polling function
*                                  will return value
*
****************************************************************************/
static inline void coo_poll_trad_cy(perf_data_t *perf_data,
                                    CpaInstanceHandle instance,
                                    CpaStatus *status)
{
    coo_poll(perf_data, icp_sal_CyPollInstance, instance, status);
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function id used to call coo_poll using
*                                  polling function in traditional mode for
*                                  compression instances
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
* @param[in]   instance            instance handle used in polling function
*
* @param[in]   status              pointer to status where polling function
*                                  will return value
*
****************************************************************************/
static inline void coo_poll_trad_dc(perf_data_t *perf_data,
                                    CpaInstanceHandle instance,
                                    CpaStatus *status)
{
    coo_poll(perf_data, icp_sal_DcPollInstance, instance, status);
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function id used to call coo_poll using
*                                  polling function in data plane mode for
*                                  crypto instances
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
* @param[in]   instance            instance handle used in polling function
*
* @param[in]   status              pointer to status where polling function
*                                  will return value
*
****************************************************************************/
static inline void coo_poll_dp_cy(perf_data_t *perf_data,
                                  CpaInstanceHandle instance,
                                  CpaStatus *status)
{
    coo_poll(perf_data, icp_sal_CyPollDpInstance, instance, status);
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function id used to call coo_poll using
*                                  polling function in data plane mode for
*                                  compression instances
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
* @param[in]   instance            instance handle used in polling function
*
* @param[in]   status              pointer to status where polling function
*                                  will return value
*
****************************************************************************/
static inline void coo_poll_dp_dc(perf_data_t *perf_data,
                                  CpaInstanceHandle instance,
                                  CpaStatus *status)
{
    coo_poll(perf_data, icp_sal_DcPollDpInstance, instance, status);
}

/**
*****************************************************************************
* @file qat_perf_cycles.h
*
* @ingroup sample_code
*
* @description                     Function performs average cost of offload
*                                  calculation using coo data stored in
*                                  perf_data
*
* @param[in]   perf_data           pointer to structure of performance data
*                                  used to store cyclecount values
*
* @retval CPA_STATUS_SUCCESS       Cost of offload is calculated
* @retval CPA_STATUS_FAIL          Cost of offload is not calculated
*
****************************************************************************/
static inline CpaStatus coo_average(perf_data_t *perf_data)
{
    if (CPA_CC_REQ_POLL_STAMP == iaCycleCount_g && coo_initialized)
    {
        perf_cycles_t *req_cycles = perf_data->req_cycles;
        perf_cycles_t *poll_cycles = perf_data->poll_cycles;
        perf_cycles_t *cost_cycles = perf_data->cost_cycles;
        perf_cycles_t avr_req = 0;
        perf_cycles_t avr_poll = 0;
        perf_cycles_t avr_cost = 0;
        Cpa64U req_count = perf_data->req_count;
        Cpa64U poll_count = perf_data->poll_count;
        Cpa64U cost_count = req_count + poll_count;
        Cpa64U index = 0;

        if (req_count > 0)
        {
            for (index = 0; index < req_count; index++)
                avr_req += req_cycles[index];
            do_div(avr_req, req_count);
        }
        if (poll_count > 0)
        {
            for (index = 0; index < poll_count; index++)
                avr_poll += poll_cycles[index];
            do_div(avr_poll, req_count);
        }
        if (cost_count > 0)
        {
            for (index = 0; index < cost_count; index++)
                avr_cost += cost_cycles[index];
            do_div(avr_cost, cost_count);
        }
        if (avr_req > 0 && avr_cost > 0)
            avr_req -= avr_cost;
        if (avr_poll > 0 && avr_cost > 0)
            avr_poll -= avr_cost;
        perf_data->offloadCycles = avr_req + avr_poll;
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_FAIL;
}
#endif /* __QAT_PERF_CYCLES_H_ */
