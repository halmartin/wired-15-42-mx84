/*
 * Copyright 1995-2016 The OpenSSL Project Authors. All Rights Reserved.
 *
 * Licensed under the OpenSSL license (the "License").  You may not use
 * this file except in compliance with the License.  You can obtain a copy
 * in the file LICENSE in the source distribution or at
 * https://www.openssl.org/source/license.html
 */

/**
 * @file stack.h
 *
 * @note
 *    This file is taken from the OpenSSL project to reuse
 *    the functionality and modified in order to prevent conflicts with
 *    public symbols in another existing OpenSSL library:
 *    - renames the public functions to have ossl_ prefix
 *    - makes non-public functions static
 *    - modifies path to the header files
 *
 * @par
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
 */

#ifndef HEADER_STACK_H
# define HEADER_STACK_H

#ifdef  __cplusplus
extern "C" {
#endif

typedef struct stack_st OPENSSL_STACK; /* Use STACK_OF(...) instead */

typedef int (*OPENSSL_sk_compfunc)(const void *, const void *);
typedef void (*OPENSSL_sk_freefunc)(void *);
typedef void *(*OPENSSL_sk_copyfunc)(const void *);

int OPENSSL_sk_num(const OPENSSL_STACK *);
void *OPENSSL_sk_value(const OPENSSL_STACK *, int);

void *OPENSSL_sk_set(OPENSSL_STACK *, int, void *);

OPENSSL_STACK *OPENSSL_sk_new(OPENSSL_sk_compfunc cmp);
OPENSSL_STACK *OPENSSL_sk_new_null(void);
void OPENSSL_sk_free(OPENSSL_STACK *);
void OPENSSL_sk_pop_free(OPENSSL_STACK *st, void (*func) (void *));
OPENSSL_STACK *OPENSSL_sk_deep_copy(OPENSSL_STACK *, OPENSSL_sk_copyfunc c, OPENSSL_sk_freefunc f);
int OPENSSL_sk_insert(OPENSSL_STACK *sk, void *data, int where);
void *OPENSSL_sk_delete(OPENSSL_STACK *st, int loc);
void *OPENSSL_sk_delete_ptr(OPENSSL_STACK *st, const void *p);
int OPENSSL_sk_find(OPENSSL_STACK *st, const void *data);
int OPENSSL_sk_find_ex(OPENSSL_STACK *st, const void *data);
int OPENSSL_sk_push(OPENSSL_STACK *st, void *data);
int OPENSSL_sk_unshift(OPENSSL_STACK *st, void *data);
void *OPENSSL_sk_shift(OPENSSL_STACK *st);
void *OPENSSL_sk_pop(OPENSSL_STACK *st);
void OPENSSL_sk_zero(OPENSSL_STACK *st);
OPENSSL_sk_compfunc OPENSSL_sk_set_cmp_func(OPENSSL_STACK *sk, OPENSSL_sk_compfunc cmp);
OPENSSL_STACK *OPENSSL_sk_dup(OPENSSL_STACK *st);
void OPENSSL_sk_sort(OPENSSL_STACK *st);
int OPENSSL_sk_is_sorted(const OPENSSL_STACK *st);

# if OPENSSL_API_COMPAT < 0x10100000L
#  define _STACK OPENSSL_STACK
#  define sk_num OPENSSL_sk_num
#  define sk_value OPENSSL_sk_value
#  define sk_set OPENSSL_sk_set
#  define sk_new OPENSSL_sk_new
#  define sk_new_null OPENSSL_sk_new_null
#  define sk_free OPENSSL_sk_free
#  define sk_pop_free OPENSSL_sk_pop_free
#  define sk_deep_copy OPENSSL_sk_deep_copy
#  define sk_insert OPENSSL_sk_insert
#  define sk_delete OPENSSL_sk_delete
#  define sk_delete_ptr OPENSSL_sk_delete_ptr
#  define sk_find OPENSSL_sk_find
#  define sk_find_ex OPENSSL_sk_find_ex
#  define sk_push OPENSSL_sk_push
#  define sk_unshift OPENSSL_sk_unshift
#  define sk_shift OPENSSL_sk_shift
#  define sk_pop OPENSSL_sk_pop
#  define sk_zero OPENSSL_sk_zero
#  define sk_set_cmp_func OPENSSL_sk_set_cmp_func
#  define sk_dup OPENSSL_sk_dup
#  define sk_sort OPENSSL_sk_sort
#  define sk_is_sorted OPENSSL_sk_is_sorted
# endif

#ifdef  __cplusplus
}
#endif

#endif
