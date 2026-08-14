/*
 *	Copyright (C) 2009,2014,2015,2021,2022,2023 by Jonathan Naylor, G4KLX
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; version 2 of the License.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	---
 *	Converted from C++ to C.
 *	  - `CUtils::foo(...)` -> `CUtils_foo(...)`.
 *	  - std::string parameters/returns become plain `const char*` in /
 *	    `char*` + buffer-size out-parameters, since C has no std::string.
 *	  - The two dump() overloads that took an implicit level (2U) become
 *	    explicit wrappers: CUtils_dump / CUtils_dumpBits.
 *	  - bitsToByteBE/LE took `unsigned char&` (an output reference); C has
 *	    no references, so these take `unsigned char*` instead. Every call
 *	    site was updated to pass `&byte` accordingly.
 */

#ifndef	Utils_H
#define	Utils_H

#include <stdbool.h>
#include <stddef.h>

void CUtils_dump(const char* title, const unsigned char* data, unsigned int length);
void CUtils_dumpLevel(int level, const char* title, const unsigned char* data, unsigned int length);

void CUtils_dumpBits(const char* title, const bool* bits, unsigned int length);
void CUtils_dumpBitsLevel(int level, const char* title, const bool* bits, unsigned int length);

void CUtils_byteToBitsBE(unsigned char byte, bool* bits);
void CUtils_byteToBitsLE(unsigned char byte, bool* bits);

void CUtils_bitsToByteBE(const bool* bits, unsigned char* byte);
void CUtils_bitsToByteLE(const bool* bits, unsigned char* byte);

unsigned int CUtils_countBits(unsigned int v);

void CUtils_removeChar(unsigned char* haystack, char needle);

/* Writes a NUL-terminated ISO-8601-ish UTC timestamp into buffer.
   bufferSize must be at least 32 bytes. */
void CUtils_createTimestamp(char* buffer, size_t bufferSize);

#endif
