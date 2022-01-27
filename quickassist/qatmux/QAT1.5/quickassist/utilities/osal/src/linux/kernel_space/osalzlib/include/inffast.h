/* inffast.h -- header to use inffast.c
 * Copyright (C) 1995-2003, 2010 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/**
 * @file inffast.h
 *
 * @note This file is based on zlib 1.2.8.
 * The original code may be found at one of the mirror list in www.zlib.net.
 *
 * @par
 * <COPYRIGHT_TAG>
 *
 */
/* WARNING: this file should *not* be used by applications. It is
   part of the implementation of the compression library and is
   subject to change. Applications should only use zlib.h.
 */

void ZLIB_INTERNAL inflate_fast OF((z_streamp strm, unsigned start));
