/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2015 Intel Corporation.
  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  qat-linux@intel.com

  BSD LICENSE
  Copyright(c) 2015 Intel Corporation.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef ADF_PF2VF_MSG_H
#define ADF_PF2VF_MSG_H

/*
 * PF<->VF Messaging
 * The PF has an array of 32-bit PF2VF registers, one for each VF.  The
 * PF can access all these registers; each VF can access only the one
 * register associated with that particular VF.
 *
 * The register functionally is split into two parts:
 * The bottom half is for PF->VF messages. In particular when the first
 * bit of this register (bit 0) gets set an interrupt will be triggered
 * in the respective VF.
 * The top half is for VF->PF messages. In particular when the first bit
 * of this half of register (bit 16) gets set an interrupt will be triggered
 * in the PF.
 *
 * The remaining bits within this register are available to encode messages.
 * and implement a collision control mechanism to prevent concurrent use of
 * the PF2VF register by both the PF and VF.
 *
 *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 * +-----------------------------------------------+
 *  \___________________________/ \_________/ ^   ^
 *                ^                    ^      |   |
 *                |                    |      |   VF2PF Int
 *                |                    |      Message Origin
 *                |                    Message Type
 *                Message-specific Data/Reserved
 *
 *  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *  _______________________________________________
 * |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 * +-----------------------------------------------+
 *  \___________________________/ \_________/ ^   ^
 *                ^                    ^      |   |
 *                |                    |      |   PF2VF Int
 *                |                    |      Message Origin
 *                |                    Message Type
 *                Message-specific Data/Reserved
 *
 * Message Origin (Should always be 1)
 * A legacy out-of-tree QAT driver allowed for a set of messages not supported
 * by this driver; these had a Msg Origin of 0 and are ignored by this driver.
 *
 * When a PF or VF attempts to send a message in the lower or upper 16 bits,
 * respectively, the other 16 bits are written to first with a defined
 * IN_USE_BY pattern as part of a collision control scheme (see adf_iov_putmsg).
 */

/* VF/PF compatibility version. */
/* ADF_PFVF_COMPATIBILITY_EXT_CAP: Support for extended capabilities */
#define ADF_PFVF_COMPATIBILITY_CAPABILITIES	2
/* ADF_PFVF_COMPATIBILITY_FAST_ACK: In-use pattern cleared by receiver */
#define ADF_PFVF_COMPATIBILITY_FAST_ACK		3
#define ADF_PFVF_COMPATIBILITY_VERSION		3	/* PF<->VF compat */

/* PF->VF messages */
#define ADF_PF2VF_INT				BIT(0)
#define ADF_PF2VF_MSGORIGIN_SYSTEM		BIT(1)
#define ADF_PF2VF_MSGTYPE_MASK			0x0000003C
#define ADF_PF2VF_MSGTYPE_SHIFT			2
#define ADF_PF2VF_MSGTYPE_RESTARTING		0x01
#define ADF_PF2VF_MSGTYPE_VERSION_RESP		0x02
#define ADF_PF2VF_MSGTYPE_BLOCK_RESP		0x03
#define ADF_PF2VF_MSGTYPE_FATAL_ERROR		0x04
#define ADF_PF2VF_IN_USE_BY_PF			0x6AC20000
#define ADF_PF2VF_IN_USE_BY_PF_MASK		0xFFFE0000

/* PF->VF Fatal Error */
#define ADF_PF2VF_FATAL_ERROR_SHIFT		6
#define ADF_PF2VF_FATAL_ERROR_MASK		0x0000FFC0
#define ADF_PF2VF_FATAL_ERROR_UNCORRECTABLE	0x00
#define ADF_PF2VF_FATAL_ERROR_HEARTBEAT		0x01

/* PF->VF Version Response */
#define ADF_PF2VF_VERSION_RESP_VERS_MASK	0x00003FC0
#define ADF_PF2VF_VERSION_RESP_VERS_SHIFT	6
#define ADF_PF2VF_VERSION_RESP_RESULT_MASK	0x0000C000
#define ADF_PF2VF_VERSION_RESP_RESULT_SHIFT	14
#define ADF_PF2VF_MINORVERSION_SHIFT		6
#define ADF_PF2VF_MAJORVERSION_SHIFT		10
#define ADF_PF2VF_VF_COMPATIBLE			1
#define ADF_PF2VF_VF_INCOMPATIBLE		2
#define ADF_PF2VF_VF_COMPAT_UNKNOWN		3

/* PF->VF Block Request Type */
#define ADF_VF2PF_MIN_SMALL_MESSAGE_TYPE	0
#define ADF_VF2PF_MAX_SMALL_MESSAGE_TYPE \
		(ADF_VF2PF_MIN_SMALL_MESSAGE_TYPE + 15)
#define ADF_VF2PF_MIN_MEDIUM_MESSAGE_TYPE \
		(ADF_VF2PF_MAX_SMALL_MESSAGE_TYPE + 1)
#define ADF_VF2PF_MAX_MEDIUM_MESSAGE_TYPE \
		(ADF_VF2PF_MIN_MEDIUM_MESSAGE_TYPE + 7)
#define ADF_VF2PF_MIN_LARGE_MESSAGE_TYPE \
		(ADF_VF2PF_MAX_MEDIUM_MESSAGE_TYPE + 1)
#define ADF_VF2PF_MAX_LARGE_MESSAGE_TYPE \
		(ADF_VF2PF_MIN_LARGE_MESSAGE_TYPE + 3)
#define ADF_VF2PF_SMALL_PAYLOAD_SIZE		30
#define ADF_VF2PF_MEDIUM_PAYLOAD_SIZE		62
#define ADF_VF2PF_LARGE_PAYLOAD_SIZE		126

#define ADF_VF2PF_BLOCK_REQ_TYPE_SHIFT		22
#define ADF_VF2PF_LARGE_BLOCK_BYTE_NUM_SHIFT	24
#define ADF_VF2PF_MEDIUM_BLOCK_BYTE_NUM_SHIFT	25
#define ADF_VF2PF_SMALL_BLOCK_BYTE_NUM_SHIFT	26
#define ADF_VF2PF_BLOCK_REQ_CRC_SHIFT		31
#define ADF_VF2PF_LARGE_BLOCK_BYTE_NUM_MASK	0x7F000000
#define ADF_VF2PF_MEDIUM_BLOCK_BYTE_NUM_MASK	0x7E000000
#define ADF_VF2PF_SMALL_BLOCK_BYTE_NUM_MASK	0x7C000000
#define ADF_VF2PF_LARGE_BLOCK_REQ_TYPE_MASK	0xC00000
#define ADF_VF2PF_MEDIUM_BLOCK_REQ_TYPE_MASK	0x1C00000
#define ADF_VF2PF_SMALL_BLOCK_REQ_TYPE_MASK	0x3C00000

/* PF->VF Block Response Type */
#define ADF_PF2VF_BLOCK_RESP_TYPE_DATA		0x0
#define ADF_PF2VF_BLOCK_RESP_TYPE_CRC		0x1
#define ADF_PF2VF_BLOCK_RESP_TYPE_ERROR		0x2
#define ADF_PF2VF_BLOCK_RESP_TYPE_SHIFT		6
#define ADF_PF2VF_BLOCK_RESP_DATA_SHIFT		8
#define ADF_PF2VF_BLOCK_RESP_TYPE_MASK		0x000000C0
#define ADF_PF2VF_BLOCK_RESP_DATA_MASK		0x0000FF00

/* PF-VF block message header bytes */
#define ADF_VF2PF_BLOCK_VERSION_BYTE		0
#define ADF_VF2PF_BLOCK_LEN_BYTE                1
#define ADF_VF2PF_BLOCK_DATA	                2

/* PF->VF Block Error Code */
#define ADF_PF2VF_INVALID_BLOCK_TYPE		0x0
#define ADF_PF2VF_INVALID_BYTE_NUM_REQ		0x1
#define ADF_PF2VF_PAYLOAD_TRUNCATED		0x2
#define ADF_PF2VF_UNSPECIFIED_ERROR		0x3

/* VF->PF messages */
#define ADF_VF2PF_IN_USE_BY_VF			0x00006AC2
#define ADF_VF2PF_IN_USE_BY_VF_MASK		0x0000FFFE
#define ADF_VF2PF_INT				BIT(16)
#define ADF_VF2PF_MSGORIGIN_SYSTEM		BIT(17)
#define ADF_VF2PF_MSGTYPE_MASK			0x003C0000
#define ADF_VF2PF_MSGTYPE_SHIFT			18
#define ADF_VF2PF_MSGTYPE_INIT			0x3
#define ADF_VF2PF_MSGTYPE_SHUTDOWN		0x4
#define ADF_VF2PF_MSGTYPE_VERSION_REQ		0x5
#define ADF_VF2PF_MSGTYPE_COMPAT_VER_REQ	0x6
#define ADF_VF2PF_MSGTYPE_GET_LARGE_BLOCK_REQ	0x7
#define ADF_VF2PF_MSGTYPE_GET_MEDIUM_BLOCK_REQ	0x8
#define ADF_VF2PF_MSGTYPE_GET_SMALL_BLOCK_REQ	0x9

/* Block message types
 *  0..15 - 32 byte message
 * 16..23 - 64 byte message
 * 24..27 - 128 byte message
 * 2 - Get Capability Request message
 */
#define ADF_VF2PF_BLOCK_MSG_CAP_SUMMARY 2

/* VF->PF Compatible Version Request */
#define ADF_VF2PF_COMPAT_VER_REQ_SHIFT		22

/* How long to wait for far side to acknowledge receipt */
#define ADF_IOV_MSG_ACK_DELAY_US		5
#define ADF_IOV_MSG_ACK_MAX_DELAY_US	(2 * 1000 * 1000)
/* If CSR is busy, how long to delay before retrying */
#define ADF_IOV_MSG_RETRY_DELAY			5
#define ADF_IOV_MSG_MAX_RETRIES			3
/* How long to wait for a response from the other side */
#define ADF_IOV_MSG_RESP_TIMEOUT		100
/* How often to retry when there is no response */
#define ADF_IOV_MSG_RESP_RETRIES		5

#define ADF_IOV_RATELIMIT_INTERVAL		8
#define ADF_IOV_RATELIMIT_BURST			130

/* CRC Calculation */
#define ADF_CRC8_INIT_VALUE 0xFF
/* PF VF message byte shift */
#define ADF_PFVF_DATA_SHIFT                     8
#define ADF_PFVF_DATA_MASK                      0xFF
#endif /* ADF_IOV_MSG_H */
