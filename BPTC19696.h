/*
 *   Copyright (C) 2015,2023 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *   ---
 *   Converted from C++ to C.
 *     - The `CBPTC19696` class becomes a plain struct holding the same two
 *       heap-allocated buffers.
 *     - The constructor/destructor become CBPTC19696_init()/CBPTC19696_free().
 *       C has no RAII, so callers must call CBPTC19696_init() before use and
 *       CBPTC19696_free() when done, in place of `CBPTC19696 x;` / going out
 *       of scope.
 *     - The private helper methods (decodeExtractBinary, encodeInterleave,
 *       etc.) are declared `static` in BPTC19696.c instead - C has no
 *       class-level access control, so file-scope `static` is the
 *       equivalent way to keep them internal to the translation unit.
 */

#if !defined(BPTC19696_H)
#define	BPTC19696_H

#include "Defines.h"
#include <stdbool.h>


typedef struct {
	bool* m_rawData;
	bool* m_deInterData;
} CBPTC19696;

/* Allocates the two internal 196-bool work buffers. Must be called before
   any other CBPTC19696_* function on this instance. */
void CBPTC19696_init(CBPTC19696* self);

/* Frees the buffers allocated by CBPTC19696_init(). */
void CBPTC19696_free(CBPTC19696* self);

void CBPTC19696_decode(CBPTC19696* self, const unsigned char* in, unsigned char* out);

void CBPTC19696_encode(CBPTC19696* self, const unsigned char* in, unsigned char* out);

#endif

