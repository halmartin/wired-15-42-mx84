/**
 * @file OsalZlib.c (linux kernel space)
 *
 * @brief Implementation for calls to Zlib.
 *
 *
 * @par
 * GPL LICENSE SUMMARY
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
 *  version: QAT1.5.L.1.11.0-36
 */

#include "Osal.h"
#include "OsalOsTypes.h"
#include "cpa.h"
#include "zlib.h"

/* OSAL_MEM_ASSERT defined in OsalOsTypes.h */
#ifdef OSAL_MEM_ASSERT
#undef OSAL_MEM_ASSERT
#endif

/* Override original to provide error return value and message */
#define OSAL_MEM_ASSERT(mem_ref) \
do { \
    if (NULL == mem_ref) { \
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT, \
                 "osalZlibInflate(): " #mem_ref " is NULL\n", \
                 0, 0, 0, 0, 0, 0, 0, 0); \
        return OSAL_FAIL; \
    } \
} while (0)

OSAL_PUBLIC OSAL_INLINE OSAL_STATUS
osalAdler32(UINT32 *checksum, UINT8 *buffer, UINT32 length)
{
    UINT32 tempChecksum = *checksum;
    OSAL_MEM_ASSERT (checksum);
    OSAL_MEM_ASSERT (buffer);
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

    OSAL_MEM_ASSERT (srcBufferList);
    OSAL_MEM_ASSERT (destBufferList);
    OSAL_MEM_ASSERT (produced);
    OSAL_MEM_ASSERT (checksum);
    OSAL_MEM_ASSERT (zstream);
    
    /* Port to zlib z_streamp type */
    stream = (z_streamp)zstream;

    /* Initialise checksum seed. */
    if (OSAL_DC_CRC32 == checksumType)
    {
        tmpChksum = 0;
    } 
    else if (OSAL_DC_ADLER32 == checksumType)
    {
        tmpChksum = 1;
    } 
    else 
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT, \
                 "osalZlibInflate(): Invalid checksum type requested\n", \
                 0, 0, 0, 0, 0, 0, 0, 0); \
        return OSAL_FAIL;
    }
    
    /* Allocate memory for Zlib workspace. */
    stream->workspace = kmalloc(zlib_inflate_workspacesize(), GFP_ATOMIC);
    if(!stream->workspace)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalZlibInflate(): Error while allocating z_stream"
                 " workspace\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        *checksum = 0;
        return OSAL_FAIL;
    }

    /* Must be equal to 15 if deflateInit2() was not used.
     * -15 means looking for raw inflate.
     */
    ret = inflateInit2(stream, -DEFLATE_DEF_WINBITS);
    if (Z_OK != ret)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalZlibInflate(): Failed to initialise zlib\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        kfree(stream->workspace);
        stream->workspace = NULL;
        *checksum = 0;
        return OSAL_FAIL;
    }

    stream->next_in = (UINT8 *)srcBuffList->pBuffers[srcBuffIdx].pData;
    stream->avail_in = srcBuffList->pBuffers[srcBuffIdx].dataLenInBytes;
    stream->next_out  = (UINT8 *)destBuffList->pBuffers[destBuffIdx].pData;
    stream->avail_out = destBuffList->pBuffers[destBuffIdx].dataLenInBytes;
    do {
        if ( NULL == stream->next_out )
               ret = Z_STREAM_ERROR;
        else
               ret = inflate(stream, Z_FINISH);

       /* Inspect error */
       switch (ret)
       {
           case Z_OK:
           case Z_STREAM_END:
           {    
               break;
           } /* Good result, leave switch */

           case Z_BUF_ERROR:
           {
               /* The output buffer is not big enough
                * or the input buffer is fully processed.
                * Otherwise there is an unrecoverable error 
                * (falls through to default clause).
                * All versions of zlib implementations
                * set avail_out to 0 if output buffer 
                * is not big enough. However avail_in may or
                * may not be 0 when avail_out is 0 */
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
                       kfree(stream->workspace);
                       stream->workspace = NULL;
                       *checksum = 0;
                       return OSAL_DC_OVERFLOW;
                   }
                   stream->next_out  = 
                           (UINT8 *)destBuffList->pBuffers[destBuffIdx].pData;
                   stream->avail_out = 
                           destBuffList->pBuffers[destBuffIdx].dataLenInBytes;
                   break;
               }
               
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
                       kfree(stream->workspace);
                       stream->workspace = NULL;
                       *checksum = 0;
                       return OSAL_FAIL;
                   }
                   stream->next_in = 
                           (UINT8 *)srcBuffList->pBuffers[srcBuffIdx].pData;
                   stream->avail_in = 
                           srcBuffList->pBuffers[srcBuffIdx].dataLenInBytes;
                   break;
               }
           } /* Z_BUF_ERROR falls through on unrecoverable error */
           case Z_NEED_DICT:
           case Z_DATA_ERROR:
           case Z_MEM_ERROR:
           case Z_STREAM_ERROR:
           default:
           {
               osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                        "osalZlibInflate(): Failed to run zlib_inflate %d\n",
                        ret, 0, 0, 0, 0, 0, 0, 0);
               inflateEnd(stream);
               kfree(stream->workspace);
               stream->workspace = NULL;
               *checksum = 0;
               return OSAL_FAIL;
           } /* default Unrecoverable errors */
       }
    }while (Z_STREAM_END != ret);

    *produced = stream->total_out;

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
               tmpChksum = crc32(tmpChksum ^ OSAL_XOR_VALUE, 
                                 destBuffList->pBuffers[i].pData,
                                 checksumDataLen) ^ OSAL_XOR_VALUE;
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

    /* Exit Zlib and cleanup */
    inflateEnd(stream);
    if (NULL != stream->workspace)
    {
        kfree(stream->workspace);
        stream->workspace = NULL;
    }
    return OSAL_SUCCESS;
}
