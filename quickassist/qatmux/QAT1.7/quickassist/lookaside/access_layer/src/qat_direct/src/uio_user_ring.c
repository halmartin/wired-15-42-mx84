/***************************************************************************
 *
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
 *
 ***************************************************************************/
#include <string.h>
#include <errno.h>
#include "uio_user_ring.h"
#include "uio_user_cfg.h"
#include <qae_mem.h>
#include <adf_platform_common.h>
#include <adf_platform_acceldev_common.h>
#include <icp_platform.h>
#include "adf_transport_ctrl.h"

static uint32_t validateRingSize(uint32_t num_msgs_on_ring,
                                 uint32_t msg_size_in_bytes,
                                 uint32_t *modulo_value)
{
    const uint32_t size_in_bytes = num_msgs_on_ring * msg_size_in_bytes;
    const uint32_t max_ring_bytes = ICP_ET_SIZE_TO_BYTES(ICP_RINGSIZE_MEG_4);
    uint32_t size_config = 0;
    uint32_t kbytes = 0;
    uint32_t k_size = 0;

    if (0 >= num_msgs_on_ring || 0 >= msg_size_in_bytes ||
        0 != modulo(msg_size_in_bytes, 4))
    {
        ADF_ERROR("Invalid Input: Num messages on ring=%d, "
                  "Msg size(bytes)=%d. Using 16K\n",
                  num_msgs_on_ring,
                  msg_size_in_bytes);
        /* use default ring size */
        size_config = ICP_RINGSIZE_KILO_16;
        *modulo_value = MODULO_SHIFT_FOR_16K;
    }
    else if (max_ring_bytes >= size_in_bytes)
    {
        /*  under range will auto give minimum size
            or valid range calculate the highest bit set
            first right shift to smallest ring size */
        kbytes = ((size_in_bytes - 1) >> MODULO_SHIFT_FOR_1K);
        while (kbytes > 0)
        {
            kbytes = kbytes >> 1;
            k_size++;
        }
        size_config = ICP_RINGSIZE_KILO_1 + k_size;
        *modulo_value = MODULO_SHIFT_FOR_1K + k_size;
    }
    else
    {
        /* larger than supported maximum */
        /* set for maximum size */
        size_config = ICP_RINGSIZE_MEG_4;
        *modulo_value = MODULO_SHIFT_FOR_4M;
    }

    return size_config;
}

static int adf_reserve_ring(adf_dev_bank_handle_t *bank, uint32_t ring_number)
{
    int status = -1;

    if (NULL == bank->user_bank_lock)
    {
        ADF_ERROR("user_bank_lock is not initialized\n");
        return status;
    }

    status = ICP_MUTEX_LOCK(bank->user_bank_lock);
    if (status)
    {
        ADF_ERROR("Failed to lock bank with error %d\n", status);
        return status;
    }

    if (!(bank->ring_mask & (1 << ring_number)))
    {
        bank->ring_mask |= (1 << ring_number);
    }
    else
        status = -EBUSY;

    ICP_MUTEX_UNLOCK(bank->user_bank_lock);

    return status;
}

static void adf_unreserve_ring(adf_dev_bank_handle_t *bank,
                               uint32_t ring_number)
{
    int status;

    status = ICP_MUTEX_LOCK(bank->user_bank_lock);
    if (status)
    {
        ADF_ERROR("Failed to lock bank with error %d\n", status);
        return;
    }
    bank->ring_mask &= ~(1 << ring_number);
    ICP_MUTEX_UNLOCK(bank->user_bank_lock);
}


int32_t adf_user_put_msg(adf_dev_ring_handle_t *ring,
                         uint32_t *inBuf,
                         uint64_t *seq_num)
{
    int status;
    uint32_t *targetAddr;
    uint8_t *csr_base_addr;
    int64_t flight;
    ICP_CHECK_FOR_NULL_PARAM(ring);
    ICP_CHECK_FOR_NULL_PARAM(inBuf);
    ICP_CHECK_FOR_NULL_PARAM(ring->accel_dev);

    csr_base_addr = ((uint8_t *)ring->csr_addr);
    status = ICP_MUTEX_LOCK(ring->user_lock);
    if (status)
    {
        ADF_ERROR("Failed to lock bank with error %d\n", status);
        return CPA_STATUS_FAIL;
    }

    /* Check if there is enough space in the ring */
    flight = __sync_add_and_fetch(ring->in_flight, 1);
    if (flight > ring->max_requests_inflight)
    {
        __sync_sub_and_fetch(ring->in_flight, 1);
        status = CPA_STATUS_RETRY;
        goto adf_user_put_msg_exit;
    }

    targetAddr = (uint32_t *)(((UARCH_INT)ring->ring_virt_addr) + ring->tail);
    if (ring->message_size == ADF_MSG_SIZE_64_BYTES)
    {
        adf_memcpy64(targetAddr, inBuf);
    }
    else if (ring->message_size == ADF_MSG_SIZE_128_BYTES)
    {
        adf_memcpy128(targetAddr, inBuf);
    }
    else
    {
        status = CPA_STATUS_FAIL;
        goto adf_user_put_msg_exit;
    }

    /* Update shadow copy values */
    ring->tail = modulo((ring->tail + ring->message_size), ring->modulo);
    /* and the config space of the device */
    WRITE_CSR_RING_TAIL(ring->bank_offset, ring->ring_num, ring->tail);

    ring->csrTailOffset = ring->tail;

    if (NULL != seq_num)
        *seq_num = ring->send_seq;

    ring->send_seq++;

adf_user_put_msg_exit:
    ICP_MUTEX_UNLOCK(ring->user_lock);
    return status;
}

/*
 * Notifies the transport handle in question.
 */
int32_t adf_user_notify_msgs(adf_dev_ring_handle_t *ring)
{
    uint32_t *msg;
    uint32_t msg_counter = 0;
    uint8_t *csr_base_addr;

    ICP_CHECK_FOR_NULL_PARAM(ring);

    csr_base_addr = ((uint8_t *)ring->csr_addr);
    msg = (uint32_t *)(((UARCH_INT)ring->ring_virt_addr) + ring->head);

    /* If there are valid messages then process them */
    while (*msg != EMPTY_RING_SIG_WORD)
    {
        /* Invoke the callback for the message */
        ring->callback((uint32_t *)msg);

        /* Mark the message as processed */
        *msg = EMPTY_RING_SIG_WORD;

        /* Advance the head offset and handle wraparound */
        ring->head = modulo((ring->head + ring->message_size), ring->modulo);
        msg_counter++;

        /* Point to where the next message should be */
        msg = (uint32_t *)(((UARCH_INT)ring->ring_virt_addr) + ring->head);
    }

    /* Update the head CSR if any messages were processed */
    if (msg_counter > 0)
    {
        __sync_sub_and_fetch(ring->in_flight, msg_counter);
        /* Coalesce head writes to reduce impact of MMIO write */
        if (msg_counter > ring->coal_write_count)
        {
            ring->coal_write_count = ring->min_resps_per_head_write;
            WRITE_CSR_RING_HEAD(ring->bank_offset, ring->ring_num, ring->head);
        }
        else
        {
            /* Not enough responses have been processed to warrant the cost
             * of a head write. Updating the count for the next time. */
            ring->coal_write_count -= msg_counter;
        }
    }

    return 0;
}

/*
 * Notify function used for polling. Messages are read until the ring is
 * empty or the response quota has been fulfilled.
 * If the response quota is zero, messages are read until the ring is drained.
 */
int32_t adf_user_notify_msgs_poll(adf_dev_ring_handle_t *ring)
{
    volatile uint32_t *msg = NULL;
    uint32_t msg_counter = 0, response_quota;
    uint8_t *csr_base_addr = NULL;

    csr_base_addr = ((uint8_t *)ring->csr_addr);
    response_quota = (ring->ringResponseQuota != 0) ? ring->ringResponseQuota
                                                    : ICP_NO_RESPONSE_QUOTA;
    /* point to where the next message should be */
    msg = (uint32_t *)(((UARCH_INT)ring->ring_virt_addr) + ring->head);

    /* If there are valid messages then process them */
    while ((*msg != EMPTY_RING_SIG_WORD) && (msg_counter < response_quota))
    {
        /* Invoke the callback for the message */
        ring->callback((uint32_t *)msg);

        /* Mark the message as processed */
        *msg = EMPTY_RING_SIG_WORD;

        /* Advance the head offset and handle wraparound */
        ring->head = modulo((ring->head + ring->message_size), ring->modulo);
        msg_counter++;
        /* Point to where the next message should be */
        msg = (uint32_t *)(((UARCH_INT)ring->ring_virt_addr) + ring->head);
    }

    /* Update the head CSR if any messages were processed */
    if (msg_counter > 0)
    {
        /* May need to do this earlier to prevent perf impact in multi-threaded
         * scenarios */
        __sync_sub_and_fetch(ring->in_flight, msg_counter);

        /* Coalesce head writes to reduce impact of MMIO write, except if
         * interrupt method is enabled cause otherwise it would keep triggering
         * new interrupts over and over again */
        if (msg_counter > ring->coal_write_count ||
            ICP_RESP_TYPE_IRQ == ring->resp)
        {
            ring->coal_write_count = ring->min_resps_per_head_write;
            WRITE_CSR_RING_HEAD(ring->bank_offset, ring->ring_num, ring->head);
        }
        else
        {
            /* Not enough responses have been processed to warrant the cost
             * of a head write. Updating the count for the next time. */
            ring->coal_write_count -= msg_counter;
        }
    }
    else
    {
        return CPA_STATUS_RETRY;
    }

    return CPA_STATUS_SUCCESS;
}

int32_t adf_init_ring(adf_dev_ring_handle_t *ring,
                      adf_dev_bank_handle_t *bank,
                      uint32_t ring_num,
                      uint32_t *csr_base_addr,
                      uint32_t num_msgs,
                      uint32_t msg_size,
                      int nodeid)
{
    uint32_t modulo = 0;
    uint32_t ring_size_cfg = validateRingSize(num_msgs, msg_size, &modulo);
    uint32_t ring_size_bytes = ICP_ET_SIZE_TO_BYTES(ring_size_cfg);
    uint64_t ring_base_cfg, ring_config;
    uint8_t nearly_full_wm = ICP_RING_NEAR_WATERMARK_512;
    uint8_t nearly_empty_wm = ICP_RING_NEAR_WATERMARK_0;
    uint32_t max_space = ring_size_bytes;

    /* Exclusive access to one ring */
    if (adf_reserve_ring(bank, ring_num))
    {
        ADF_ERROR(
            "Ring [%u:%u] existed already\n", bank->bank_number, ring_num);
        return -EBUSY;
    }

    ring->head = 0;
    ring->tail = 0;
    ring->send_seq = 0;
    ring->bank_data = bank;
    /* Now the bank offset is 0 because we get the band's offset */
    ring->bank_offset = 0;
    ring->ring_num = ring_num;
    ring->csr_addr = csr_base_addr;
    ring->message_size = msg_size;
    ring->modulo = modulo;
    ring->ring_size = ring_size_bytes;
    ring->ring_virt_addr =
        qaeMemAllocNUMA(ring_size_bytes, nodeid, ring_size_bytes);
    ring->ring_phys_base_addr = qaeVirtToPhysNUMA(ring->ring_virt_addr);
    if ((NULL == ring->ring_virt_addr) || (0 == ring->ring_phys_base_addr))
    {
        ADF_ERROR("unable to get ringbuf(v:%p,p:%p) for rings in bank(%u)\n",
                  ring->ring_virt_addr,
                  ring->ring_phys_base_addr,
                  ring->ring_num);
        adf_unreserve_ring(bank, ring_num);
        if (ring->ring_phys_base_addr)
            qaeMemFreeNUMA(&ring->ring_virt_addr);
        return -ENOMEM;
    }
    memset(ring->ring_virt_addr, EMPTY_RING_SIG_BYTE, ring_size_bytes);

    ring->min_resps_per_head_write =
        ((max_space / msg_size) >> 1 > MIN_RESPONSES_PER_HEAD_WRITE)
            ? MIN_RESPONSES_PER_HEAD_WRITE
            : (max_space / msg_size) >> 1;
    ring->max_requests_inflight = num_msgs - 1;

    ring_base_cfg =
        BUILD_RING_BASE_ADDR(ring->ring_phys_base_addr, ring_size_cfg);
    WRITE_CSR_RING_BASE(ring->bank_offset, ring->ring_num, ring_base_cfg);

    if (bank->tx_rings_mask & (1 << ring_num))
    {
        ring_config = BUILD_RING_CONFIG(ring_size_cfg);
    }
    else
    {
        ring_config = BUILD_RESP_RING_CONFIG(
            ring_size_cfg, nearly_full_wm, nearly_empty_wm);
    }
    WRITE_CSR_RING_CONFIG(ring->bank_offset, ring->ring_num, ring_config);

    icp_adf_enable_ring(
        ring->accel_dev->accelId, ring->bank_num, ring->ring_num);

    return 0;
}

int32_t adf_ring_freebuf(adf_dev_ring_handle_t *ring)
{
    if (ring->ring_virt_addr)
    {
        /* This function would set it to NULL */
        qaeMemFreeNUMA(&ring->ring_virt_addr);
        ring->ring_virt_addr = NULL;
    }

    return 0;
}

void adf_cleanup_ring(adf_dev_ring_handle_t *ring)
{
    uint32_t *csr_base_addr = ring->csr_addr;

    icp_adf_disable_ring(
        ring->accel_dev->accelId, ring->bank_num, ring->ring_num);

    /* Clear CSR configuration */
    WRITE_CSR_RING_CONFIG(ring->bank_offset, ring->ring_num, 0);
    WRITE_CSR_RING_BASE(ring->bank_offset, ring->ring_num, 0);

    adf_unreserve_ring(ring->bank_data, ring->ring_num);

    if (ring->ring_virt_addr)
    {
        memset(ring->ring_virt_addr, EMPTY_RING_SIG_BYTE, ring->ring_size);
        qaeMemFreeNUMA(&ring->ring_virt_addr);
    }
}
