/**
 * @file OsalZlib.c (linux kernel space)
 *
 * @brief Implementation for calls to Zlib.
 *
 *
 * @par
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
 *  version: QAT1.5.L.1.11.0-36
 */

#include "Osal.h"
#include "OsalOsTypes.h"
#include "cpa.h"
#include "zlib.h"

OSAL_PUBLIC OSAL_INLINE OSAL_STATUS
osalAdler32(UINT32 *checksum, UINT8 *buffer, UINT32 length)
{
    UINT32 tempChecksum = *checksum;
    *checksum = adler32(tempChecksum, buffer, length);
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalZlibInflate(void *srcBufferList,
                void *destBufferList, UINT32 *produced,
                UINT32 checksumType, UINT32 *checksum,
                osalz_stream *zstream)
{
    INT32 ret = 0;
    UINT32 tmpChksum = 0;
    UINT32 srcBuffIdx = 0;
    UINT32 destBuffIdx = 0;
    UINT32 checksumDataLen = 0;
    UINT32 checksumProduced = 0;
    UINT32 bufferLen = 0;
    UINT32 i = 0;
    z_streamp stream = Z_NULL;
    CpaBufferList* const srcBuffList = (CpaBufferList *)srcBufferList;
    CpaBufferList* const destBuffList = (CpaBufferList *)destBufferList;
    /* Port to zlib z_stream type */
    stream = (z_streamp)zstream;


    stream->zalloc = (alloc_func)0;
    stream->zfree = (free_func)0;
    stream->opaque = (voidpf)0;
    stream->next_in = Z_NULL;
    stream->next_out = Z_NULL;
    stream->avail_in = 0;
    stream->total_out = 0;
    stream->adler = 0;

    /* Initialise Zlib for raw inflate.*/
    ret = inflateInit2(stream, -DEFLATE_DEF_WINBITS);
    if (ret != Z_OK)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalZlibInflate(): Failed to initialise zlib\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        *checksum = 0;
        return OSAL_FAIL;
    }

    ret = inflateReset(stream);
    if(ret != Z_OK)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalZlibInflate(): Failed to reset zlib\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        inflateEnd(stream);
        *checksum = 0;
        return OSAL_FAIL;
    }

    stream->next_in = (UINT8 *)srcBuffList->pBuffers[srcBuffIdx].pData;
    stream->avail_in = srcBuffList->pBuffers[srcBuffIdx].dataLenInBytes;
    stream->next_out  = (UINT8 *)destBuffList->pBuffers[destBuffIdx].pData;
    stream->avail_out = destBuffList->pBuffers[destBuffIdx].dataLenInBytes;

    do {
       ret = inflate(stream, Z_FINISH);

       /* Inspect error */
       switch (ret)
       {
           case Z_BUF_ERROR:
               /* Either the input is processed but end of
                * compressed data stream hasn't been reached
                * or output buffer not big enough.
                */
               if (0 == stream->avail_in)
               {
                   /* The input buffer has been processed. */
                   srcBuffIdx++;
                   if (srcBuffIdx >= srcBuffList->numBuffers)
                   {   /* All the buffers have been processed and
                        * Z_STREAM_END has not been foudn, therefore
                        * clean up and return an error.
                        */
                       osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                          "osalZlibInflate(): Invalid input buffer.\n",
                          0, 0, 0, 0, 0, 0, 0, 0);
                       inflateEnd(stream);
                       *checksum = 0;
                       return OSAL_FAIL;
                   }
                   stream->next_in = (UINT8 *)srcBuffList->pBuffers[srcBuffIdx].pData;
                   stream->avail_in = srcBuffList->pBuffers[srcBuffIdx].dataLenInBytes;
               }
               if (0 == stream->avail_out)
               {
                   /* The output buffer is not big enough. */
                   destBuffIdx++;
                   if (destBuffIdx >= destBuffList->numBuffers)
                   {
                       /* We've reached the last buffer,
                        * therfore cleanup and return an
                        * overflow error.
                        */
                       osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                           "osalZlibInflate(): Invalid output buffer.\n",
                           0, 0, 0, 0, 0, 0, 0, 0);
                       inflateEnd(stream);
                       *checksum = 0;
                       return OSAL_DC_OVERFLOW;
                   }
                   stream->next_out  = (UINT8 *)destBuffList->pBuffers[destBuffIdx].pData;
                   stream->avail_out = destBuffList->pBuffers[destBuffIdx].dataLenInBytes;
               }
               break;

           case Z_NEED_DICT:
           case Z_DATA_ERROR:
           case Z_MEM_ERROR:
               osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                  "osalZlibInflate(): Failed to run inflate\n",
                  0, 0, 0, 0, 0, 0, 0, 0);
               inflateEnd(stream);
               *checksum = 0;
               return OSAL_FAIL;
       }
    }while (ret != Z_STREAM_END);

    *produced = stream->total_out;

    /* Initialise checksum seed. */
    if (OSAL_DC_CRC32 == checksumType)
    {
        tmpChksum = 0;
    }
    else if (OSAL_DC_ADLER32 == checksumType)
    {
       tmpChksum = 1;
    }

    /* Compute checksum */
    checksumProduced = 0;
    for (i=0; i<destBuffList->numBuffers; i++)
    {
       if (checksumProduced < stream->total_out)
       {
           bufferLen = destBuffList->pBuffers[i].dataLenInBytes;
           if ((checksumProduced + bufferLen) < stream->total_out)
           {
               checksumDataLen = bufferLen;
           }
           else
           {
               checksumDataLen = (stream->total_out - checksumProduced);
           }

           if (OSAL_DC_CRC32 == checksumType)
           {
               tmpChksum = crc32(tmpChksum,
                                 destBuffList->pBuffers[i].pData,
                                 checksumDataLen);
           }
           else if (OSAL_DC_ADLER32 == checksumType)
           {
               tmpChksum = adler32(tmpChksum,
                                 destBuffList->pBuffers[i].pData,
                                 checksumDataLen);
           }
           checksumProduced += checksumDataLen;
       }
    }

    /* Return checksum */
    *checksum = tmpChksum;

    /* Exit Zlib */
    inflateEnd(stream);
    return OSAL_SUCCESS;
}

