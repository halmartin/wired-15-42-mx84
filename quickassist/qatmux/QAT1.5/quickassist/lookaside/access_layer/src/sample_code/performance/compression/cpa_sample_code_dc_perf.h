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
 *****************************************************************************
 * @file cpa_sample_code_dc_perf.h
 *
 * @defgroup compressionThreads
 *
 * @ingroup compressionThreads
 *
 * @description
 * Contains function prototypes and #defines used throughout code
 * and macros
 *
 ***************************************************************************/

#ifndef CPA_SAMPLE_CODE_DC_PERF_H_
#define CPA_SAMPLE_CODE_DC_PERF_H_

#include "cpa.h"
#include "cpa_dc.h"
#include "cpa_sample_code_framework.h"

/*
 *******************************************************************************
 * General performance code settings
 *******************************************************************************
 */

#define MIN_DC_LOOPS            (1)
#define DEFAULT_DC_LOOPS        (100)

/* Dynamic number of buffers to be created while initializing the Compression
 * session
 */
#define TEMP_NUM_BUFFS (5)
#define MIN_BUFFER_SIZE (15)
/* Extra buffer */
#define EXTRA_BUFFER (2)
#define MIN_DST_BUFFER_SIZE (8192)
#define DEFAULT_COMPRESSION_LOOPS (100)
#define DEFAULT_COMPRESSION_WINDOW_SIZE (7)
#define INITIAL_RESPONSE_COUNT (-1)
#define SCALING_FACTOR_100 (100)
#define SCALING_FACTOR_1000 (1000)
#define BASE_10 (10)
#define DYNAMIC_BUFFER_AREA (0x20000)
#define SINGLE_REQUEST      (1)
#define SINGLE_LOOP         (1)
#define ONE_BUFFER_DC   (1)
#ifdef LATENCY_CODE
/* 1 MByte all zeros corpus. Enables calibrating latencies among different buffer sizes */
#define ZEROS_CORPUS_LENGTH (64*1024*16)
#endif

/* ReMapping Compression Level based on API version */
#if ( CPA_DC_API_VERSION_NUM_MAJOR == 1 && CPA_DC_API_VERSION_NUM_MINOR < 6 )
#define SAMPLE_CODE_CPA_DC_L1 (CPA_DC_L1)
#define SAMPLE_CODE_CPA_DC_L2 (CPA_DC_L2)
#define SAMPLE_CODE_CPA_DC_L3 (CPA_DC_L3)
#define SAMPLE_CODE_CPA_DC_L4 (CPA_DC_L4)
#define SAMPLE_CODE_CPA_DC_L5 (CPA_DC_L5)
#define SAMPLE_CODE_CPA_DC_L6 (CPA_DC_L6)
#define SAMPLE_CODE_CPA_DC_L7 (CPA_DC_L7)
#define SAMPLE_CODE_CPA_DC_L8 (CPA_DC_L8)
#define SAMPLE_CODE_CPA_DC_L9 (CPA_DC_L9)
#else
#define SAMPLE_CODE_CPA_DC_L1 (CPA_DC_L1)
#define SAMPLE_CODE_CPA_DC_L2 (CPA_DC_L1)
#define SAMPLE_CODE_CPA_DC_L3 (CPA_DC_L2)
#define SAMPLE_CODE_CPA_DC_L4 (CPA_DC_L2)
#define SAMPLE_CODE_CPA_DC_L5 (CPA_DC_L3)
#define SAMPLE_CODE_CPA_DC_L6 (CPA_DC_L3)
#define SAMPLE_CODE_CPA_DC_L7 (CPA_DC_L4)
#define SAMPLE_CODE_CPA_DC_L8 (CPA_DC_L4)
#define SAMPLE_CODE_CPA_DC_L9 (CPA_DC_L4)
#endif

/* the following are defined in the framework, these are used for setup only
 * and are not to be used in functions not thread safe
 */

extern Cpa8U thread_setup_g[MAX_THREAD_VARIATION][MAX_SETUP_STRUCT_SIZE_IN_BYTES];
extern int testTypeCount_g;
extern thread_creation_data_t testSetupData_g[];

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  Corpus Setup Data.
 *   @description
 *       This ENUM  contains data relating to corpus type.
 *       The client needs pass corpus type to setup
 *
 * ****************************************************************************/

typedef enum _corpusType
{
    /* Canterbury Corpus */
    CANTERBURY_CORPUS = 0,
    /* Calgary Corpus*/
    CALGARY_CORPUS,
    SIGN_OF_LIFE_CORPUS,
#ifdef LATENCY_CODE
    /* All zeros corpus. Enables calibrating latencies among different buffer sizes */
    ,ZEROS_CORPUS
#endif
} corpus_type_t;


/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  enum for synchronous Corpus Setup Data.
 *   @description
 *       This ENUM will be used to specify if the compression operation
 *       is synchronous or Asynchronous
 *       The client needs pass provide this information in the setup
 *
 * ****************************************************************************/

typedef enum _syncFlag
{
    /*Synchronous flag*/
    CPA_SAMPLE_SYNCHRONOUS = 0,
    CPA_SAMPLE_ASYNCHRONOUS
} synchronous_flag_t;


/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  corpus file data.
 *   @description
 *       This structure contains data relating to setup corpus file.
 *       This structure is updated for each file in the corpus by performance
 *       API.
 *
 * ****************************************************************************
 * */
typedef struct corpus_file_s
{
    /* Corpus data in char format */
    Cpa8U* corpusBinaryData;
    /* Corpus data length */
    Cpa32U corpusBinaryDataLen;
} corpus_file_t;


/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  enum for Data Plane Request Type
 *   @description
 *       This ENUM will be used to specify whether the compression operations
 *       are enqueued by the driver a single request at a time or in batches of
 *       multiple requests.
 *       The client needs pass provide this information in the setup
 *
 * ****************************************************************************/
typedef enum _dpRequestType
{
    /*Synchronous flag*/
    DC_DP_BATCHING = 0,
    DC_DP_ENQUEUEING
} dp_request_type_t;

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  compression setup Data.
 *   @description
 *    This structure contains data relating to setup compression performance
 *    tests.The client needs to fill this structure before calling performance
 *    API
 *
 * ****************************************************************************/

typedef struct compression_test_params_s
{
     /* Session Direction */
     CpaDcSessionDir dcSessDir;
    /*compression instance handle of service that has already been started*/
    CpaInstanceHandle dcInstanceHandle;
    /*pointer to pre-allocated memory for thread to store performance data*/
    perf_data_t* performanceStats;
    /* Performance setup data for initializing sessions */
    CpaDcSessionSetupData setupData;
    /* Corpus Type */
    corpus_type_t corpus;
    /*Buffer Size */
    Cpa32U bufferSize;
    /* Synchronous Flag */
    synchronous_flag_t    syncFlag;
    /* Number of Loops */
    Cpa32U numLoops;
    /* Request type (Batch or Enqueue) */
    dp_request_type_t dpTestType;
    /* Number of requests to submit before processing */
    Cpa32U numRequests;
    /* Array of buffers required, indexed by corpus file number */
    Cpa32U *numberOfBuffers;
    /* Unique thread ID based on the order in which the thread was created */
    Cpa32U threadID;
    /* identifies if Data Plane API is used */
    CpaBoolean isDpApi;
    Cpa32U fileSize[20];
    //session per file array of FileSize provides the fileSize for each session
    Cpa32U sessions;
    Cpa32U inputListSize;
    Cpa32U outputListSize;
    Cpa32U *numberOfOutputLists;
    Cpa32U flatBuffSize;

} compression_test_params_t;

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  corpus file data.
 *   @description
 *       This structure contains data relating to setup corpus.
 *       This structure is updated for whole  corpus by performance
 *       API.
 *
 * ****************************************************************************
 * */
typedef struct corpus_data_s
{
    /* Array of files on Corpus */
    corpus_file_t* fileArray;
    /* Number of Files in Corpus */
    Cpa32U numFilesInCorpus;
} corpus_data_t;

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *   Callback tag data structure
 *   @description
 *       This structure contains results, bufflist and the performance
 *       data structures . The results structure is used to update the
 *       compressed data length in the bufflist structure and perforamance
 *       data structure is used to update the performance results.
 *
 * ****************************************************************************
 * */
typedef struct dc_callbacktag_s
{
    /* pointer to the DC results structure */
    CpaDcRqResults* dcResult;
    /* pointer to the performance data structure */
    perf_data_t* perfData;
    /* pointer to the BufferList structure */
    CpaBufferList *pBuffList;
} dc_callbacktag_t;

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  create Buffers
 *
 *  @description
 *      this API Flat buffers Create Buffers
 *  @threadSafe
 *      No
 *
 *  @param[out]   None
 *
 *
 *  @param[in]  buffSize size of the buffer
 *  @param[in]  numBuffs Number of Buffers to create
 *  @param[in]  pBuffListArray pointer to the array of the buffer list
 *  @param[in]  nodeId node affinity
 *
 ******************************************************************************/

CpaStatus createBuffers(Cpa32U buffSize, Cpa32U numBuffs,
        CpaBufferList **pBuffListArray, Cpa32U nodeId);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcPerform
 *
 *  @description
 *      this API creates the buffer List, populate the bufflist with Corpus data
 *      and does compression or decompression based on the session direction
 *  @threadSafe
 *      Yes
 *
 *  @param[out]   status
 *
 *  @param[in]  setup pointer to test setup structure
 *
 ******************************************************************************/

CpaStatus dcPerform( compression_test_params_t* setup);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  setupDcTest
 *
 *  @description
 *      this API is the main API called by the framework, this is configures
 *      data structure before starting the performance threads
 *  @threadSafe
 *      No
 *
 *  @param[out]   None
 *
 *  @param[in]  algorithm Algorithm used for compression/decompression
 *  @param[in]  direction session direction
 *  @param[in]  compLevel compression Level
 *  @param[in]  HuffmanType HuffMantype Dynamic/static
 *  @param[in]  fileType type of the file to be compressed/decompressed
 *  @param[in]  state stateful operation or stateless operation
 *  @param[in]  windowSize window size to be used for compression/decompression
 *  @param[in]  testBuffersize size of the flat Buffer to use
 *  @parma[in]  corpusType type of corpus calgary/cantrbury corpus
 *  @param[in]  syncFlag synchronous/Asynchronous operation
 *  @param[in]  numloops Number of loops to compress or decompress
 ******************************************************************************/
CpaStatus setupDcTest(CpaDcCompType algorithm,
                   CpaDcSessionDir direction ,
                   CpaDcCompLvl compLevel,
                   CpaDcHuffType huffmanType,
                   CpaDcFileType fileType,
                   CpaDcSessionState state,
                   Cpa32U windowSize,
                   Cpa32U testBufferSize,
                   corpus_type_t corpusType,
                   synchronous_flag_t syncFlag,
                   Cpa32U numLoops);


/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  setupDcTest
 *
 *  @description
 *      this API is the main API called by the framework, this is configures
 *      data structure before starting the performance threads
 *  @threadSafe
 *      No
 *
 *  @param[out]   None
 *
 *  @param[in]  algorithm Algorithm used for compression/decompression
 *  @param[in]  direction session direction
 *  @param[in]  compLevel compression Level
 *  @param[in]  HuffmanType HuffMantype Dynamic/static
 *  @param[in]  fileType type of the file to be compressed/decompressed
 *  @param[in]  state stateful operation or stateless operation
 *  @param[in]  windowSize window size to be used for compression/decompression
 *  @param[in]  testBuffersize size of the flat Buffer to use
 *  @parma[in]  corpusType type of corpus calgary/cantrbury corpus
 *  @param[in]  syncFlag synchronous/Asynchronous operation
 *  @param[in]  numloops Number of loops to compress or decompress
 ******************************************************************************/
CpaStatus setupDcStatefulTest(CpaDcCompType algorithm,
        CpaDcSessionDir direction ,
        CpaDcCompLvl compLevel,
        CpaDcHuffType huffmanType,
        Cpa32U testBufferSize,
        corpus_type_t corpusType,
        synchronous_flag_t syncFlag,
        Cpa32U numLoops);


/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  populateCantrBryCorpus
 *
 *  @description
 *      this API populates the canterbury corpus to the corpus data structure
 *  @threadSafe
 *      No
 *
 *  @param[out]  status
 *
 *  @param[in]  buffSize size of the flat buffer
 *
 ******************************************************************************/
CpaStatus populateCantrBryCorpus(Cpa32U buffSize);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  PopulateCorpus
 *
 *  @description
 *      this API populates the calgary corpus to the corpus data structure
 *  @threadSafe
 *      No
 *
 *  @param[out]  status
 *
 *  @param[in]  buffSize size of the flat buffer
 *
 ******************************************************************************/
CpaStatus populateCalgaryCorpus(Cpa32U buffSize);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  getCorpusFile
 *
 *  @description
 *      this API copies the corpus file in user space to a char buffer
 *      in the kernel space using request_firmware API. this API expects all
 *      corpus file in /lib/firmware directory.
 *  @threadSafe
 *      No
 *
 *  @param[in]  ppSrcBuff pointer to char buffer to copy corpus file
 *  @param[in]  filename  corpus file name to be copied
 *  @param[out]  size      size of the file
 *
 ******************************************************************************/
CpaStatus getCorpusFile(Cpa8U** ppSrcBuff, char *filename, Cpa32U *size);
CpaStatus getCompressedFile(Cpa8U** ppSrcBuff, char *filename, Cpa32U *size);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  freeBuffers
 *
 *  @description
 *      this API free all the Flat buffers from Bufflist
 *  @threadSafe
 *      No
 *
 *  @param[in]  pBuffListArray pointer to bufflist
 *  @param[in]  numberOfFiles  number of files in Buffer List
 *  @param[in] pointer to compression_test_params_t structure.
 *
 ******************************************************************************/
void freeBuffers(CpaBufferList ***pBuffListArray,
                 Cpa32U numberOfFiles, compression_test_params_t* setup);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  compareBuffers
 *
 *  @description
 *      this API compares the buffers before compression and after decompression
 *  @threadSafe
 *      No
 *
 *  @param[in]  ppSrc pointer to source buffer list
 *  @param[in]  ppDst pointer to destination buffer list
 *  @param[in] pointer to compression_test_params_t structure.
 *
 ******************************************************************************/
CpaStatus compareBuffers(CpaBufferList ***ppSrc,
                           CpaBufferList ***ppDst,
                           compression_test_params_t* setup);


/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcPerformCallback
 *
 *  @description
 *      this API is compression call back, called by compress API after
 *      the compression is performed
 *  @threadSafe
 *      No
 *
 *  @param[in]  pcallbackTag call back Tag
 *  @param[in]  status status of the operation performed
 *
 ******************************************************************************/
void dcPerformCallback(
        void       *pCallbackTag,
        CpaStatus  status);
/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  deCompressCallback
 *
 *  @description
 *      this API  is decompression call back, called by de-compress API after
 *      the decompression is performed
 *  @threadSafe
 *      No
 *
 *  @param[in]  pCallbackTag call back Tag
 *  @param[in]  status status of the operation performed
 *
 ******************************************************************************/
void deCompressCallback(
        void       *pCallbackTag,
        CpaStatus  status);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  free results Structures
 *
 *  @description
 *      this API frees all the results structures
 *
 *  @threadSafe
 *      No
 *
 *  @param[in] ppDcResult array of cpaDcRqResults structure.
 *  @param[in] numFiles number of files in corpus.
 *  @param[in] pointer to compression_test_params_t structure.
 *
 ******************************************************************************/
void freeResults(CpaDcRqResults ***ppDcResult, Cpa32U numFiles,
                 compression_test_params_t* setup);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  free callback  Structures
 *
 *  @description
 *      this API frees all the callback structures
 *
 *  @threadSafe
 *      No
 *
 *  @param[in] ppCallbackTag  array of dc_callbacktag_t structure.
 *  @param[in] numFiles number of files in corpus.
 *  @param[in] pointer to compression_test_params_t structure.
 *
 ******************************************************************************/
void freeCbTags(dc_callbacktag_t ***ppCallbackTag, Cpa32U numFiles,
        compression_test_params_t* setup);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  Perform Compress
 *
 *  @description
 *      this API measures the performance of Compression.
 *
 *  @threadSafe
 *      Yes
 *
 *  @param[in] setup setup pointer to test setup structure.
 *  @param[in] srcBuffListArray pointer to Source array of bufflists
 *  @param[in] dstBuffListArray pointer to destination array of bufflists
 *  @param[in] cmpResult  pointer to Results structure
 *  @param[in] dcCbFn  pointer to Callback Function
 *
 ******************************************************************************/
CpaStatus performCompress(compression_test_params_t* setup,
                          CpaBufferList ***srcBuffListArray,
                          CpaBufferList ***dstBuffListArray,
                          CpaDcRqResults ***cmpResult,
                          CpaDcCallbackFn dcCbFn);
/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  Perform DeCompress
 *
 *  @description
 *      this API measures the performance of DeCompression.
 *
 *  @threadSafe
 *      Yes
 *
 *  @param[in] setup setup pointer to test setup structure.
 *  @param[in] srcBuffListArray pointer to Source array of bufflists
 *  @param[in] dstBuffListArray pointer to destination array of bufflists
 *  @param[in] cmpBuffListArray pointr to array of bufflists for
 *             comparision of source and the result of the decompression
 *  @param[in] dcmpResult  pointer to Results structure
 *  @param[in] dcCbFn  pointer to Callback Function
 *
 ******************************************************************************/
CpaStatus performDeCompress(compression_test_params_t* setup,
                          CpaBufferList ***srcBuffListArray,
                          CpaBufferList ***dstBuffListArray,
                          CpaBufferList ***cmpBuffListArray,
                          CpaDcRqResults ***cmpResult,
                          CpaDcRqResults ***dcmpResult,
                          CpaDcCallbackFn dcCbFn);
/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  Perform DeCompress
 *
 *  @description
 *      this API Compress the corpus, the output of this API will be used in the
 *      deCompression performance
 *
 *  @threadSafe
 *      Yes
 *
 *  @param[in] setup setup pointer to test setup structure.
 *  @param[in] srcBuffListArray pointer to Source array of bufflists
 *  @param[in] dstBuffListArray pointer to destination array of bufflists
 *  @param[in] cmpResult  pointer to Results structure
 *  @param[in] callbacktag  pointer to Callback Function
 *
 ******************************************************************************/

CpaStatus compressCorpus(compression_test_params_t* setup,
                          CpaBufferList ***srcBuffListArray,
                          CpaBufferList ***dstBuffListArray,
                          CpaDcRqResults ***cmpResult,
                          dc_callbacktag_t ***callbacktag);
/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcSampleCreateContextBuffer
 *
 *  @description
 *      this API Flat buffers Create Context Buffers
 *  @threadSafe
 *      No
 *
 *  @param[out]   None
 *
 *
 *  @param[in]  buffSize size of the buffer
 *  @param[in]  metaSize meta size of the buffer
 *  @param[in]  pBuffListArray pointer to the array of the buffer list
 *  @param[in]  nodeId node affinity
 *
 ******************************************************************************/

CpaStatus dcSampleCreateContextBuffer(Cpa32U buffSize,
                Cpa32U metaSize,CpaBufferList **pBuffListArray, Cpa32U nodeId);

/**
 * *****************************************************************************
 *  @ingroup compressionThreads
 *  dcSampleFreeContextBuffer
 *
 *  @description
 *      this API free all the Context Flat buffers from Bufflist
 *  @threadSafe
 *      No
 *
 *  @param[in]  pBuffListArray pointer to bufflist
 *
 ******************************************************************************/
void dcSampleFreeContextBuffer(CpaBufferList *pBuffListArray);

#endif /* CPA_SAMPLE_CODE_DC_PERF_H_ */
