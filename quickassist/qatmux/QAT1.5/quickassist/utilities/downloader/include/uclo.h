/**
 **************************************************************************
 * @file uclo.h
 *
 * @description
 *      This is the header file for uCode Loader Library
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
 * @file uclo.h
 * 
 * @defgroup Uclo Microcode Loader Library
 *
 * @description
 *      This header file that contains the prototypes and definitions required
 *      for microcode loader
 *
 *****************************************************************************/

#ifndef __UCLO_H__
#define __UCLO_H__

#include "icptype.h"
#include "core_platform.h"
#include "uof.h"
#include "dbgAeInfo.h"
#include "uclo_status.h"
#include "icp_firml_handle.h"

#define UCLO_BADFUNCID  -1          /**< bad function ID */

/**
 * @description ustore memory segment
*/
typedef struct uclo_varumemseg_s{
    unsigned int umemBaseAddr;      /**< ustore memory segment base address */
    unsigned int umemSize;          /**< ustore memory segment byte size */
}uclo_varumemseg_t;

/**
 * @description data memory segment
*/
typedef struct uclo_varmemseg_s{
    uint64 sramBaseAddr;            /**< SRAM memory segment base address */
    unsigned int sramSize;          /**< SRAM memory segment byte size */
    unsigned int sramAlignment;     /**< SRAM memory segment byte alignment */
    uint64 sdramBaseAddr;           /**< DRAM0 memory segment base address */
    unsigned int sdramSize;         /**< DRAM0 memory segment byte size */
    unsigned int sdramAlignment;    /**< DRAM0 memory segment byte alignment */
    uint64 sdram1BaseAddr;          /**< DRAM1 memory segment base address */
    unsigned int sdram1Size;        /**< DRAM1 memory segment byte size */
    unsigned int sdram1Alignment;   /**< DRAM1 memory segment byte alignment */
    uint64 scratchBaseAddr;         /**< SCRATCH memory segment base address */
    unsigned int scratchSize;       /**< SCRATCH memory segment byte size */
    unsigned int scratchAlignment;  /**< SCRATCH memory segment byte alignment */
    uclo_varumemseg_t aeUmemSeg[MAX_AE]; /**< ustore being used in the uof */
                                   /**< ustore values are specified in terms of
                                      uwords, not bytes or 32-bit words.
                                      Array is indexed using physical AE addrs */
}uclo_varmemseg_t;

/**
 * @description system memory information
*/
typedef struct uclo_symbolinfo_s{
    uof_MemRegion memType;     /**< memory type: sram/dram/lmem/umem/scratch */
    unsigned int baseAddr;     /**< word aligned byte address of the symbol */ 
    unsigned int allocSize;    /**< total bytes allocated */
}uclo_symbolinfo_t;

/**
 * @description image assigned information
*/
typedef struct uclo_imageassign_s{
    unsigned int assignedCtxMask; /**< image assigned context mask */
    char        *imageName;       /**< image name */
}uclo_imageassign_t;

/**
 * @description import variable information
*/
typedef struct uclo_importvalue_s{
   uint64    value;             /**< value of import variable */
   unsigned int    valueAttrs;  /**< import variable attribute 
                                     bit<0> (Scope: 0=global), 
                                     bit<1> (init: 0=no, 1=yes) */
}uclo_importvalue_t;

/**
 * @description uof version information
*/
typedef struct uof_ver_s{
    unsigned int fileId;   /**< File Id and endian indicator. */
    unsigned int majVer;   /**< UOF Major Version */
    unsigned int minVer;   /**< UOF Minor Version */
    unsigned int acType;   /**< chip family type */
    unsigned int minAcVer; /**< The minimum chip stepping 
                                that the UOF will run on, 
                                e.g. A0 stepping or B0 stepping */
    unsigned int maxAcVer; /**< The maximum chip stepping 
                                that the UOF will run on, 
                                e.g. A0 stepping or B0 stepping */
}uof_ver_t;

#ifdef __cplusplus
extern "C"{
#endif

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      This function initializes the uclo library and performs any AccelEngine
 *      driver initialization. It should be called prior to calling any of the 
 *      other functions in the uclo library
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/

EXPORT_API void UcLo_InitLib(icp_firml_handle_t *handle);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Initializes the Loader library. AccelEngines whose corresponding bit 
 *      is set in aeMask are set to the powerup default state. The library 
 *      assumes that those AccelEngines that are not specified are in a
 *      reset state
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param aeMask - IN An integer mask specifying the AccelEngines to initialize
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/
 
EXPORT_API void UcLo_InitLibUeng(icp_firml_handle_t *handle,
                                 unsigned int aeMask);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Writes a 32-bit unsigned value to the specified AccelEngine(s) and 
 *      micro address
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param aeMask - IN An integer mask specifying the AccelEngines to which to
 *                    write the value uWord at the address uAddr
 * @param phyUaddr - IN An integer value indicating the microstore physical  
 *                      address to write the value to
 * @param uWord - IN The value to be written to one or more AccelEngines 
 *                   initialize
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_WriteUword(icp_firml_handle_t *handle, 
                               unsigned int aeMask, 
                               unsigned int phyUaddr, 
                               uword_T uWord);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Reads a word from the specified uEngine
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * @param phyUaddr - IN An integer value indicating the microstore physical  
 *                      address to write the value to
 * @param uWord - OUT The value read from the specified AccelEngines at address 
 *                    uAddr 
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_ReadUword(icp_firml_handle_t *handle, 
                              unsigned char ae, 
                              unsigned int phyUaddr, 
                              uword_T *uWord);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Compares the UOF image that was either loaded or mapped to the content 
 *      of the assigned AccelEngine(s)
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ucodeImageName - IN Pointer to character string containing name of 
 *                             the AccelEngine image
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_VerifyUimage(icp_firml_handle_t *handle, 
                                 char *ucodeImageName);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Compares the content of the specified AccelEngine to its assigned UOF 
 *      image that was either mapped or loaded
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_VerifyUengine(icp_firml_handle_t *handle, 
                                  unsigned char ae);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Writes the specified image in the UOF object to the assigned 
 *      AccelEngines 
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_UNINITVAR A variable was not initialized
 * 
 * 
*****************************************************************************/

EXPORT_API int UcLo_WriteUimage(icp_firml_handle_t *handle, 
                                char *ucodeImageName);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Writes all the microcode images to one or more appropriate 
 *      AccelEngine(s)
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_WriteUimageAll(icp_firml_handle_t *handle);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      authenticate MMP image and move it into correct sram address
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_AuthenticateMMP (icp_firml_handle_t *handle, 
                                     char *filePtr, 
                                     uint64 fileSize);


/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieve the locations, sizes, and alignment of the C compiler variables
 *      memory segments 
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param varMemSeg - OUT A pointer to variable segment structure
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_GetVarMemSegs(icp_firml_handle_t *handle, 
                                  uclo_varmemseg_t *varMemSeg);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Removes all references to the UOF image -- if one was either loaded 
 *      or mapped. The memory region of mapped UOF, by calling 
 *      UcLo_MapImageAddr, is not deallocated by this function and it is 
 *      responsible of the caller to explicitly delete it
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_DeleObj(icp_firml_handle_t *handle);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Maps the memory location pointed to by the addrPtr parameter to the 
 *      microcode Object File (UOF). The library verifies that the mapped 
 *      object fits within the size specified by the memSize parameter and 
 *      that the object checksum is valid.
 *      If the readOnly parameter is not equal to zero, then the library copies
 *      any region of memory that it needs to modify. Also, the memory region, 
 *      (addrPtr through addrPtr + memSize + 1), should not be modified by the
 *      caller while the image is mapped to the region -- unless through the 
 *      use of uclo library functions.
 *      The user should call UcLo_DeleObj() to remove reference to the object 
 *      and to free the resources that was allocated by the library.
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param addrPtr - IN Pointer to the memory location where the Ucode Object 
 *                     File image resides
 * @param memSize - IN An integer indicating the size of the memory region 
 *                     pointed to by the addrPtr parameter  
 * @param readOnly - IN Indicates whether the memory space being pointed to by
 *                       addrPtr is read-only initialize
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_MapObjAddr(icp_firml_handle_t *handle, 
                               void *addrPtr, 
                               int memSize, 
                               int readOnly);

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Open the MOF/UOF file and apply a buffer to store it
 *     It's similar to UcLo_CopyObjFile which can only be called at user space
 *     It's caller's responsibility to release mofPtr at the end
 *
 * @param mofName - IN
 * @param mofPtr  - OUT
 * @param mofSize - OUT
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG
 * 
 * 
 *****************************************************************************/
#ifndef __KERNEL__

EXPORT_API int UcLo_CopyMof (char* mofName, 
                             char** mofPtr, 
                             uint64* mofSize);

#endif /* #ifndef (__KERNEL__) */

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parses MOF/UOF file format, returns required UOF objects
 *     While the expected uofName is NULL, this function would parse the MOF 
 *     object and initialize mof_handle, but no UOF object would be returned.
 *
 * @param handle - IN
 * @param mofPtr - IN  Start address of MOF/UOF buffer
 * @param mofSize - IN MOF/UOF length
 * @param uofName - IN The expected UOF file name
 * @param uofPtr - OUT Start address of expected UOF
 * @param uofSize - OUT UOF length
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
EXPORT_API int UcLo_MapMofAddr (icp_firml_handle_t *handle, 
                                void *mofPtr, 
                                uint64 mofSize, 
                                char* uofName, 
                                char** uofPtr, 
                                unsigned int* uofSize);


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Release MOF handle created during UcLo_MapMofAddr
 *
 * @param handle - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG
 * 
 * 
 *****************************************************************************/
EXPORT_API int UcLo_DelMof (icp_firml_handle_t *handle);


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Parses SUOF file format
 *
 * @param handle - IN
 * @param suofPtr -  IN  Start address of SUOF buffer
 * @param suofSize - IN SUOF length
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG, UCLO_BADOBJ
 * 
 * 
 *****************************************************************************/
EXPORT_API int UcLo_MapSuofAddr (icp_firml_handle_t *handle,
                                 void *suofPtr, 
                                 uint64 suofSize);

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Release SUOF handle created during UcLo_MapSuofAddr
 *
 * @param handle - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG
 * 
 * 
 *****************************************************************************/
EXPORT_API int UcLo_DelSuof (icp_firml_handle_t *handle);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Loads the UOF objects from the file specified by fileName to a buffer. 
 *      If UCLO_SUCCESS is returned, then a buffer is allocated and its pointer
 *      and size are returned in objBuf and chunkSize respectively. It is the 
 *      responsibility of the caller to delete the buffer. 
 *      This API does not support kernel mode as it involves in file operation.
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
 * @param fileName - IN Character string pointer to the name of the image file 
 *                      that was created by linker.
 * @param objBuf - OUT A char pointer to accept the pointer to the object
 * @param chunkSize - OUT The size of the buffer pointed to by objBuf
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FILEFAIL File operation failed due to file related error
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/

#ifndef __KERNEL__

EXPORT_API int UcLo_CopyObjFile(char *fileName, 
                                char **objBuf, 
                                unsigned int *chunkSize);

#endif /* #ifndef (__KERNEL__) */

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Loads an object file that was created by the microcode linker to 
 *      processor memory. The library allocates and manages the resources to 
 *      accommodate the object. The user should call UcLo_DeleObj() to remove 
 *      reference to the object and to free the resources that was allocated by 
 *      the library.
 *      This API does not support kernel mode as it involves in file operation.
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
 * @param handle - OUT AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param fileName - IN Character string pointer to the name of the image file 
 *                      that was created by linker
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FILEFAIL File operation failed due to file related error
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_MEMFAIL Failed allocate memory
 * @retval UCLO_BADOBJ Error in object format or checksum
 * @retval UCLO_UOFVERINCOMPAT The UOF is incompatible with this version of 
 *                             the Loader library
 * @retval UCLO_UOFINCOMPAT The UOF is incompatible with this version of the 
                            chip
 * 
 * 
 *****************************************************************************/

#ifndef __KERNEL__

EXPORT_API int UcLo_LoadObjFile(icp_firml_handle_t *handle, 
                                char *fileName);

#endif /* #ifndef (__KERNEL__) */

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Creates an association between an application value and a microcode 
 *      symbol. It initializes all occurrences of the specified symbol in the 
 *      UOF image to the 32-bit value, or portion of the 32-bit value, as 
 *      defined by the ucode assembler (uca). 
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ucodeImageName - IN Pointer to a character string containing the name
 *                            of the AccelEngine image
 * @param ucodeSymName - IN String pointer to the name of the microcode symbol 
 *                          to bind 
 * @param value - IN An unsigned 32-bit value of which to initialize the 
 *                   microcode symbols 
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_SYMNOTFND Symbol not found
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_BindSymbol(icp_firml_handle_t *handle, 
                               char *ucodeImageName, 
                               char *ucodeSymName, 
                               uint64 value);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Binds a value to the microcode symbol in the UOF image. Because 
 *      multiple AccelEngines may be assigned to the same UOF image, the 
 *      result of this function applies to the specified AccelEngine
 *      and to all assigned AccelEngines
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * @param ucodeSymName - IN String pointer to the name of the microcode symbol 
 *                          to bind 
 * @param value - IN An unsigned 32-bit value of which to initialize the 
 *                   microcode symbols 
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_SYMNOTFND Symbol not found
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_AeBindSymbol(icp_firml_handle_t *handle, 
                                 unsigned char ae, 
                                 char *ucodeSymName, 
                                 uint64 value);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      First, the function verifies that the file is an UOF. If it is, then 
 *      the UOF format version of the file is returned in the minVer and majVer
 *      arguments.
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
 * @param fileHdr - IN A handle to the file 
 * @param minVer - IN The UOF format minor version
 * @param majVer - IN The UOF format major version
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_UOFINCOMPAT The UOF is incompatible with this version of 
 *                          the Loader library
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_verifyFile(char *fileHdr, 
                               short *minVer, 
                               short *majVer);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Returns the specified object's CRC checksum
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param checksum - OUT The specified object's checksum
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_GetChecksum(icp_firml_handle_t *handle, 
                                unsigned int *checksum);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Return the image-name by name of the specified AE. The return
 *      value to the image-name must not be modified micro address
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 *
 * @retval String pointer to the name of the image
 * 
 * 
 *****************************************************************************/

EXPORT_API char *UcLo_GetAeImageName(icp_firml_handle_t *handle, 
                                     unsigned char ae);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Returns the mask specifying the AccelEngines that are assigned to an 
 *      image in the UOF
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 *
 * @retval Mask of the assigned AEs
 * 
 * 
 *****************************************************************************/

EXPORT_API unsigned int UcLo_GetAssignedAEs(icp_firml_handle_t *handle);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Returns the mask specifying the contexts that are assigned to the 
 *      AccelEngine in the UOF 
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 *
 * @retval Mask of the assigned contexts
 * 
 * 
 *****************************************************************************/

EXPORT_API unsigned int UcLo_GetAssignedCtxs(icp_firml_handle_t *handle, 
                                             unsigned char ae);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieves the udbug information from debug section of an UOF. If the 
 *      UOF is linked with debug information, then this function allocates 
 *      memory, extracts the debug information, and returns a pointer in the 
 *      image parameter. It is the caller's responsibility to delete the debug 
 *      information. 
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param image - OUT An pointer reference to the debug information
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_GetDebugInfo(icp_firml_handle_t *handle, 
                                 dbgAeInfo_Image_T *image);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Reads the specified file and extracts information to be used to 
 *      initialize import-variables. The file format is the following 
 *      space-separated values:
 *      <chipName> <imageName> <symbolName> <value>
 *      ImageName and chipName can be specified as a wildcard (*) to match any 
 *      chip or image. If imagename is *, then all instances of the variable 
 *      found in any of the images are initialized to the specified value.
 *      This API does not support kernel mode as it involves in file operation.
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
 * @param filename - IN Character string pointer to the name of the Ivd file 
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/

#ifndef __KERNEL__

EXPORT_API int UcLo_LoadIvdFile(char *filename);

#endif /* #ifndef (__KERNEL__) */

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Extracts the information from string buffer to be used to initialize 
 *      import-variables.
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
 * @param ivdBuf - IN Character string pointer to the buffer
 * @param ivdBufLen - IN Buffer string length
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_LoadIvdBuf(char *ivdBuf, 
                               unsigned int ivdBufLen);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Removes all the information and resources that were created as a 
 *      result of calls to UcLo_LoadIvdFile, or UcLo_LoadIvdBuf
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
 * @param - none
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

EXPORT_API void UcLo_DelIvd(void);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieves the storage information of a local symbol
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * @param symName - IN String pointer to the name of the microcode local symbol 
 *                     to bind 
 * @param symInfo - OUT A pointer reference to the symbol information
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_GetLocalSymbolInfo(icp_firml_handle_t *handle, 
                                       unsigned int ae, 
                                       char *symName, 
                                       uclo_symbolinfo_t *symInfo);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieves the storage information of a global symbol
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param symName - IN String pointer to the name of the microcode global symbol 
 *                     to bind 
 * @param symInfo - OUT A pointer reference to the symbol information
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_GetGlobalSymbolInfo(icp_firml_handle_t *handle, 
                                        char *symName, 
                                        uclo_symbolinfo_t *symInfo);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieve the application meta-data asociated with the AE or any
 *      AE if hwAe is UCLO_BADAE.
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 *
 * @retval Pointer to character string containing meta-data
 * 
 * 
 *****************************************************************************/

EXPORT_API char *UcLo_GetAppMetaData(icp_firml_handle_t *handle, 
                                     unsigned int ae);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieves the physical ustore address for the specified virtual 
 *      ustore address
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param hwAe - IN Specifies a AccelEngine
 * @param virtUaddr - IN Virtual ustore address 
 * @param pageNum - IN Page number associated with the physical address
 * @param phyUaddr - OUT Physical ustore address
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_ADDRNOTFOUND Address not found
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_GetPhyUaddr(icp_firml_handle_t *handle, 
                                unsigned int hwAe, 
                                unsigned int virtUaddr,
					            unsigned int*pageNum, 
					            unsigned int *phyUaddr);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Locate an import variable and returns its value and an attribute 
 *      indicating whether or not the variable was initialized, and whether 
 *      its scope is global or local
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param hwAe - IN An integer mask specifying the AccelEngines to which to
 *                    write the value uWord at the address uAddr.
 * @param varName - IN Name of the import-variable 
 * @param importVal - OUT Pointer to structure to receive the import-variable's
 *                    value and attributes
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_GetImportVal(icp_firml_handle_t *handle, 
                                 unsigned int hwAe, 
                                 char *varName, 
                                 uclo_importvalue_t *importVal);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Set the SCRATCH/SRAM/NCDRAM/CDRAM allocated base address for AccelEngine
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
 * @param scratchOffset - IN Scratch memory address offset     
 * @param sramOffset - IN SRAM memory address offset
 * @param dram0Offset - IN DRAM0 memory address offset
 * @param dram1Offset - IN DRAM1 memory address offset
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/

EXPORT_API int UcLo_SetMemoryStartOffset(icp_firml_handle_t *handle,
                                         unsigned int scratchOffset, 
                                         unsigned int sramOffset, 
                                         unsigned int dram0Offset, 
                                         unsigned int dram1Offset);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      retrieve UOF version info
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
 * @param objBuf - IN a Pointer to the memory location 
 *                    where the Ucode Object image resides
 * @param uofVer - OUT UOF version info
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/
EXPORT_API int UcLo_GetUofVer(void * objBuf, 
                              unsigned int objBufSize,
                              uof_ver_t * uofVer);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      retrieve MMP version info
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
 * @param handle - IN AC firmware loader handle, 
 *                 which contains a void pointer reference to 
 *                 the loaded and mapped object
 * @param objBuf - IN a Pointer to the memory location 
 *                    where the Ucode Object image resides
 * @param mofVer - OUT UOF version info
 
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/
EXPORT_API int UcLo_GetMmpVer(icp_firml_handle_t *handle,
                              void * objBuf, unsigned int* mmpVer);

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Get the error string for provided error code
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
 * @param errCode - IN Loader library error code
 *
 * @retval Pointer to character string containing error description
 * 
 * 
 *****************************************************************************/

EXPORT_API char *UcLo_GetErrorStr(int errCode);

#ifdef __cplusplus
}
#endif

#endif          /* __UCLO_H__ */
