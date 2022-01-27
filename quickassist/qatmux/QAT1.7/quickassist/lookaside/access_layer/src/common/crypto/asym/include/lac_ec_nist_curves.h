/******************************************************************************
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
 *****************************************************************************/

/**
 *****************************************************************************
 * @file lac_ec_nist_curves.h
 *
 * @ingroup Lac_Ec
 *
 * Elliptic Curves definitions for accelerated L256 and 571 GF2 PKE service and
 * for 521 GFP PKE service
 *
 *****************************************************************************/

#ifndef LAC_EC_NIST_CURVES_H
#define LAC_EC_NIST_CURVES_H

/*********** NIST PRIME 521 CURVE ****************/
#define NIST_GFP_Q_521_BIT_POS 520
#define NIST_GFP_A_521_BIT_POS 520
#define NIST_GFP_B_521_BIT_POS 518
#define NIST_GFP_H_521_BIT_POS 0
#define NIST_GFP_R_521_BIT_POS 520

STATIC const Cpa8U nist_p521_q[] = {
    0x1,  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

STATIC const Cpa8U nist_p521_a[] = {
    0x1,  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};

STATIC const Cpa8U nist_p521_b[] = {
    0x51, 0x95, 0x3e, 0xb9, 0x61, 0x8e, 0x1c, 0x9a, 0x1f, 0x92, 0x9a,
    0x21, 0xa0, 0xb6, 0x85, 0x40, 0xee, 0xa2, 0xda, 0x72, 0x5b, 0x99,
    0xb3, 0x15, 0xf3, 0xb8, 0xb4, 0x89, 0x91, 0x8e, 0xf1, 0x9,  0xe1,
    0x56, 0x19, 0x39, 0x51, 0xec, 0x7e, 0x93, 0x7b, 0x16, 0x52, 0xc0,
    0xbd, 0x3b, 0xb1, 0xbf, 0x7,  0x35, 0x73, 0xdf, 0x88, 0x3d, 0x2c,
    0x34, 0xf1, 0xef, 0x45, 0x1f, 0xd4, 0x6b, 0x50, 0x3f, 0x0};

STATIC const Cpa8U nist_p521_r[] = {
    0x1,  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xfa, 0x51, 0x86, 0x87, 0x83, 0xbf, 0x2f, 0x96, 0x6b, 0x7f, 0xcc,
    0x1,  0x48, 0xf7, 0x9,  0xa5, 0xd0, 0x3b, 0xb5, 0xc9, 0xb8, 0x89,
    0x9c, 0x47, 0xae, 0xbb, 0x6f, 0xb7, 0x1e, 0x91, 0x38, 0x64, 0x9};

/*********** NIST 163 KOBLITZ  AND BINARY CURVES ****************/
#define NIST_GF2_Q_163_BIT_POS 163
#define NIST_GF2_A_163_BIT_POS 0
#define NIST_GF2_H_163_BIT_POS 1
#define NIST_GF2_R_163_BIT_POS 162

STATIC const Cpa8U nist_gf2_163_q[] = {0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                       0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                       0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc9};

STATIC const Cpa8U nist_gf2_163_a[] = {0x1};

STATIC const Cpa8U nist_gf2_163_h[] = {0x2};

/*********** NIST 163 KOBLITZ CURVE ****************/

#define NIST_GF2_B_K163_BIT_POS 0

STATIC const Cpa8U nist_koblitz_gf2_163_b[] = {0x1};

STATIC const Cpa8U nist_koblitz_gf2_163_r[] = {
    0x4, 0x0, 0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0, 0x2,
    0x1, 0x8, 0xa2, 0xe0, 0xcc, 0xd, 0x99, 0xf8, 0xa5, 0xef};

/*********** NIST 163 BINARY CURVE ****************/

#define NIST_GF2_B_B163_BIT_POS 161

STATIC const Cpa8U nist_binary_gf2_163_b[] = {
    0x2,  0xa,  0x60, 0x19, 0x7,  0xb8, 0xc9, 0x53, 0xca, 0x14, 0x81,
    0xeb, 0x10, 0x51, 0x2f, 0x78, 0x74, 0x4a, 0x32, 0x5,  0xfd};

STATIC const Cpa8U nist_binary_gf2_163_r[] = {
    0x4,  0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,  0x0, 0x2,
    0x92, 0xfe, 0x77, 0xe7, 0xc, 0x12, 0xa4, 0x23, 0x4c, 0x33};

/*********** NIST 233 KOBLITZ AND BINARY CURVES ****************/
#define NIST_GF2_Q_233_BIT_POS 233
#define NIST_GF2_A_233_BIT_POS 0

STATIC const Cpa8U nist_gf2_233_q[] = {
    0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1};

/*********** NIST 233 KOBLITZ CURVE ****************/

#define NIST_GF2_H_K233_BIT_POS 2
#define NIST_GF2_B_K233_BIT_POS 0
#define NIST_GF2_R_K233_BIT_POS 231

STATIC const Cpa8U nist_koblitz_gf2_233_h[] = {0x4};

STATIC const Cpa8U nist_koblitz_gf2_233_a[] = {0x0};

STATIC const Cpa8U nist_koblitz_gf2_233_b[] = {0x1};

STATIC const Cpa8U nist_koblitz_gf2_233_r[] = {
    0x80, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x6,  0x9d, 0x5b, 0xb9, 0x15, 0xbc,
    0xd4, 0x6e, 0xfb, 0x1a, 0xd5, 0xf1, 0x73, 0xab, 0xdf};

/*********** NIST 233 BINARY CURVE ****************/

#define NIST_GF2_H_B233_BIT_POS 1
#define NIST_GF2_B_B233_BIT_POS 230
#define NIST_GF2_R_B233_BIT_POS 232

STATIC const Cpa8U nist_binary_gf2_233_h[] = {0x2};

STATIC const Cpa8U nist_binary_gf2_233_a[] = {0x1};

STATIC const Cpa8U nist_binary_gf2_233_b[] = {
    0x66, 0x64, 0x7e, 0xde, 0x6c, 0x33, 0x2c, 0x7f, 0x8c, 0x9,
    0x23, 0xbb, 0x58, 0x21, 0x3b, 0x33, 0x3b, 0x20, 0xe9, 0xce,
    0x42, 0x81, 0xfe, 0x11, 0x5f, 0x7d, 0x8f, 0x90, 0xad};

STATIC const Cpa8U nist_binary_gf2_233_r[] = {
    0x1,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x0,  0x0, 0x0,  0x13, 0xe9, 0x74, 0xe7, 0x2f,
    0x8a, 0x69, 0x22, 0x3, 0x1d, 0x26, 0x3,  0xcf, 0xe0, 0xd7};

/*********** NIST 571 KOBLITZ  AND BINARY CURVES ****************/
#define NIST_GF2_Q_571_BIT_POS 571
#define NIST_GF2_A_571_BIT_POS 0

STATIC const Cpa8U nist_gf2_571_q[] = {
    0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x25};

/*********** NIST 571 KOBLITZ CURVE ****************/

#define NIST_GF2_B_K571_BIT_POS 0
#define NIST_GF2_H_K571_BIT_POS 2
#define NIST_GF2_R_K571_BIT_POS 569

STATIC const Cpa8U nist_koblitz_gf2_571_h[] = {0x4};

STATIC const Cpa8U nist_koblitz_gf2_571_r[] = {
    0x2,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
    0x13, 0x18, 0x50, 0xe1, 0xf1, 0x9a, 0x63, 0xe4, 0xb3, 0x91, 0xa8, 0xdb,
    0x91, 0x7f, 0x41, 0x38, 0xb6, 0x30, 0xd8, 0x4b, 0xe5, 0xd6, 0x39, 0x38,
    0x1e, 0x91, 0xde, 0xb4, 0x5c, 0xfe, 0x77, 0x8f, 0x63, 0x7c, 0x10, 0x1};

/*********** NIST 571 BINARY CURVE ****************/
#define NIST_GF2_B_B571_BIT_POS 569
#define NIST_GF2_H_B571_BIT_POS 1
#define NIST_GF2_R_B571_BIT_POS 569

STATIC const Cpa8U nist_binary_gf2_571_b[] = {
    0x2,  0xf4, 0xe,  0x7e, 0x22, 0x21, 0xf2, 0x95, 0xde, 0x29, 0x71, 0x17,
    0xb7, 0xf3, 0xd6, 0x2f, 0x5c, 0x6a, 0x97, 0xff, 0xcb, 0x8c, 0xef, 0xf1,
    0xcd, 0x6b, 0xa8, 0xce, 0x4a, 0x9a, 0x18, 0xad, 0x84, 0xff, 0xab, 0xbd,
    0x8e, 0xfa, 0x59, 0x33, 0x2b, 0xe7, 0xad, 0x67, 0x56, 0xa6, 0x6e, 0x29,
    0x4a, 0xfd, 0x18, 0x5a, 0x78, 0xff, 0x12, 0xaa, 0x52, 0xe,  0x4d, 0xe7,
    0x39, 0xba, 0xca, 0xc,  0x7f, 0xfe, 0xff, 0x7f, 0x29, 0x55, 0x72, 0x7a};

STATIC const Cpa8U nist_binary_gf2_571_h[] = {0x2};

STATIC const Cpa8U nist_binary_gf2_571_r[] = {
    0x3,  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xe6, 0x61, 0xce, 0x18, 0xff, 0x55, 0x98, 0x73, 0x8,  0x5,  0x9b, 0x18,
    0x68, 0x23, 0x85, 0x1e, 0xc7, 0xdd, 0x9c, 0xa1, 0x16, 0x1d, 0xe9, 0x3d,
    0x51, 0x74, 0xd6, 0x6e, 0x83, 0x82, 0xe9, 0xbb, 0x2f, 0xe8, 0x4e, 0x47};

#endif /* LAC_EC_NIST_CURVES_H */
