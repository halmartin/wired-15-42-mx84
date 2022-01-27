/**
 **************************************************************************
 * @file uclo_overlay.h
 *
 * @description
 *      This is the header file for Ucode Object File Loader Library: overlay support
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

#ifndef __UCLO_OVERLAY_H
#define __UCLO_OVERLAY_H

#include "core_platform.h"
#include "icp_firml_handle.h"
#include "uof.h"

#define MAX_CONTEXTS  8
#define UCLO_PAGEFLAGS_WAITING 0x0001

struct UcLo_Region_S;
struct UcLo_OverlayData_S;

/**
 * @description This page structure has a unique instantiation for each AE, whereas the
 *              associated uof_encapPage structure has only one instantiation per image
*/
typedef struct uclo_page_s {
    struct uclo_page_s     *next;       /* Used to link pages into a list */
    struct uof_encapPage_S *encapPage;  /* Associatd uof_encapPage structure */
    struct uclo_region_s   *region;     /* Region containing this page */
    unsigned int            flags;      /* UCLO_PAGEFLAGS, defined below */
    unsigned int            halPageNum; /* Cross ref to hal page num */
} uclo_page_t;

/**
 * @description This structure defines a simple linked-list of pages
*/
typedef struct uclo_pagelist_s {
    uclo_page_t *head;  /* pages are removed from this end */
    uclo_page_t *tail;  /* pages are added at this end */
} uclo_pagelist_t;

/**
 * @description This structure represents one region in a single AE
*/
typedef struct uclo_region_s { 
    uclo_page_t    *loaded;         /* Page currently loaded in region */
    uclo_pagelist_t waitingPageIn;  /* Pages waiting to be paged in */
} uclo_region_t;

/**
 * @description This structure represents the group of contexts -- slices --that are loaded in a single AE
*/
typedef struct uclo_aeslice_s {
    unsigned int  assignedCtxMask;            /* bitMask of the assigned contexts */
    uclo_region_t *regions;                   /* Array of region objects */
    uclo_page_t   *pages;                     /* Array of page objects */
    uclo_page_t   *currentPage[MAX_CONTEXTS]; /* Current page for each ctx */
    unsigned int   newUaddr[MAX_CONTEXTS];    /* Addr to be set on page-in */
    struct uof_encapAe_S *encapImage;         /* Pointer to UCLO image */
} uclo_aeslice_t;

/**
 * @description this structure is used to cross reference between halAe and uclo page number
*/
typedef struct uclo_pagexref_s {
    unsigned int aeSliceIndex;                /* Index into the AE slice */
    unsigned int pageNum;                     /* AE slice page numnber */
} uclo_pagexref_t;

/**
 * @description UcLo_OverlayAeData_T contains data specific to an AE
*/
typedef struct uclo_aedata_s {
    unsigned int    numPageXref;             /* The number of xfer structures */
    uclo_pagexref_t *pageXref;               /* Array of hal page number cross reference */
    unsigned int    numSlices;               /* Number of slices for this AE */
    uclo_aeslice_t  aeSlice[MAX_CONTEXTS];   /* Array of AE slices */
    unsigned int    relocUstoreDram;         /* The reloadable ustore-dram address */
    unsigned int    effectUstoreSize;        /* Effective AE ustore size */
    unsigned int    shareableUstore;
} uclo_aedata_t;

typedef enum {
    UCLO_PO_LOADED = 0,
    UCLO_PO_WAITING = 1
} uclo_pageout_t;

struct Hal_IntrMasks_S;
struct uclo_objHandle_S;
struct uof_encapAe_S;

int UcLo_ClearAeData(uclo_aedata_t *aeData);

int UcLo_AssignHalPages(icp_firml_handle_t *handle,
                        uclo_aedata_t *aeData, 
			            unsigned int hwAeNum);

int UcLo_InitAeData(struct uclo_objHandle_S *objHandle, 
			unsigned int swAe, 
			unsigned int imageNum);

int UcLo_FreeAeData(uclo_aedata_t *aeData);

uclo_pageout_t UcLo_PageOut(icp_firml_handle_t *handle,
                            uclo_aedata_t *aeData,
                            unsigned int  ctx,
                            unsigned int  newPageNum, /* New page that CTX is switching to */
                            unsigned int  hwAeNum);

int UcLo_doPageIn(icp_firml_handle_t *handle,
                  unsigned int             swAe,
                  uclo_page_t             *page,
                  uclo_page_t             *oldPage,
                  int                     *startingPageChange);

int UcLo_PageInRegion(icp_firml_handle_t *handle,
                  unsigned int             swAe,
                  uclo_region_t           *region,
                  int                     *startingPageChange);

void UcLo_IntrCallback(struct Hal_IntrMasks_S *masks, 
			           void* data);

#endif /* ifdef __UCLO_OVERLAY_H */
