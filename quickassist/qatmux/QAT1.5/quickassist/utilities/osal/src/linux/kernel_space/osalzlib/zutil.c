/* zutil.c -- target dependent utility functions for the compression library
 * Copyright (C) 1995-2005, 2010, 2011, 2012 Jean-loup Gailly.
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/**
 * @file zutil.c
 *
 * @note This file is  based on zlib 1.2.8. The modifications were, only zmemcpy is in use, rest of the code has been removed.
 * The original code may be found at one of the listed mirror in www.zlib.net.
 *
 * @par
 * <COPYRIGHT_TAG>
 *
 */

/* @(#) $Id$ */


#include "zutil.h"


#ifndef HAVE_MEMCPY

void ZLIB_INTERNAL zmemcpy(dest, source, len)
    Bytef* dest;
    const Bytef* source;
    uInt  len;
{
    if (len == 0) return;
    do {
        *dest++ = *source++; /* ??? to be unrolled */
    } while (--len != 0);
}

#endif /* HAVE_MEMCPY */
