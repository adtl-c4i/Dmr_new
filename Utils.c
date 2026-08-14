/*
 *	Copyright (C) 2009,2014,2015,2016,2021,2022,2023,2025 Jonathan Naylor, G4KLX
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
 *	Converted from C++ to C (see Utils.h for the summary of changes).
 *	dump()'s std::string output buffer becomes a fixed-size char array built
 *	with strcat, which is fine here since each dumped line is bounded
 *	(16 hex pairs + 16 ASCII chars + formatting, well under the buffer size).
 */

#include "Utils.h"
//#include "Log.h"

#include <stdio.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>

#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#endif

void CUtils_dump(const char* title, const unsigned char* data, unsigned int length)
{
	assert(data != NULL);

	CUtils_dumpLevel(2, title, data, length);
}

void CUtils_dumpLevel(int level, const char* title, const unsigned char* data, unsigned int length)
{
	assert(data != NULL);

//	Log((unsigned int)level, "%s", title);

	unsigned int offset = 0U;

	while (length > 0U) {
		char output[128U];
		output[0] = '\0';

		unsigned int bytes = (length > 16U) ? 16U : length;

		for (unsigned int i = 0U; i < bytes; i++) {
			char temp[10U];
			sprintf(temp, "%02X ", data[offset + i]);
			strcat(output, temp);
		}

		for (unsigned int i = bytes; i < 16U; i++)
			strcat(output, "   ");

		strcat(output, "   *");

		unsigned int pos = strlen(output);
		for (unsigned int i = 0U; i < bytes; i++) {
			unsigned char c = data[offset + i];

			output[pos++] = isprint(c) ? (char)c : '.';
		}
		output[pos] = '\0';

		strcat(output, "*");

	//	Log((unsigned int)level, "%04X:  %s", offset, output);

		offset += 16U;

		if (length >= 16U)
			length -= 16U;
		else
			length = 0U;
	}
}

void CUtils_dumpBits(const char* title, const bool* bits, unsigned int length)
{
	assert(bits != NULL);

	CUtils_dumpBitsLevel(2, title, bits, length);
}

void CUtils_dumpBitsLevel(int level, const char* title, const bool* bits, unsigned int length)
{
	assert(bits != NULL);

	unsigned char bytes[100U];
	unsigned int nBytes = 0U;
	for (unsigned int n = 0U; n < length; n += 8U, nBytes++)
		CUtils_bitsToByteBE(bits + n, bytes + nBytes);

	CUtils_dumpLevel(level, title, bytes, nBytes);
}

void CUtils_byteToBitsBE(unsigned char byte, bool* bits)
{
	assert(bits != NULL);

	bits[0U] = (byte & 0x80U) == 0x80U;
	bits[1U] = (byte & 0x40U) == 0x40U;
	bits[2U] = (byte & 0x20U) == 0x20U;
	bits[3U] = (byte & 0x10U) == 0x10U;
	bits[4U] = (byte & 0x08U) == 0x08U;
	bits[5U] = (byte & 0x04U) == 0x04U;
	bits[6U] = (byte & 0x02U) == 0x02U;
	bits[7U] = (byte & 0x01U) == 0x01U;
}

void CUtils_byteToBitsLE(unsigned char byte, bool* bits)
{
	assert(bits != NULL);

	bits[0U] = (byte & 0x01U) == 0x01U;
	bits[1U] = (byte & 0x02U) == 0x02U;
	bits[2U] = (byte & 0x04U) == 0x04U;
	bits[3U] = (byte & 0x08U) == 0x08U;
	bits[4U] = (byte & 0x10U) == 0x10U;
	bits[5U] = (byte & 0x20U) == 0x20U;
	bits[6U] = (byte & 0x40U) == 0x40U;
	bits[7U] = (byte & 0x80U) == 0x80U;
}

void CUtils_bitsToByteBE(const bool* bits, unsigned char* byte)
{
	assert(bits != NULL);
	assert(byte != NULL);

	*byte  = bits[0U] ? 0x80U : 0x00U;
	*byte |= bits[1U] ? 0x40U : 0x00U;
	*byte |= bits[2U] ? 0x20U : 0x00U;
	*byte |= bits[3U] ? 0x10U : 0x00U;
	*byte |= bits[4U] ? 0x08U : 0x00U;
	*byte |= bits[5U] ? 0x04U : 0x00U;
	*byte |= bits[6U] ? 0x02U : 0x00U;
	*byte |= bits[7U] ? 0x01U : 0x00U;
}

void CUtils_bitsToByteLE(const bool* bits, unsigned char* byte)
{
	assert(bits != NULL);
	assert(byte != NULL);

	*byte  = bits[0U] ? 0x01U : 0x00U;
	*byte |= bits[1U] ? 0x02U : 0x00U;
	*byte |= bits[2U] ? 0x04U : 0x00U;
	*byte |= bits[3U] ? 0x08U : 0x00U;
	*byte |= bits[4U] ? 0x10U : 0x00U;
	*byte |= bits[5U] ? 0x20U : 0x00U;
	*byte |= bits[6U] ? 0x40U : 0x00U;
	*byte |= bits[7U] ? 0x80U : 0x00U;
}

unsigned int CUtils_countBits(unsigned int v)
{
	unsigned int count = 0U;

	while (v != 0U) {
		v &= v - 1U;
		count++;
	}

	return count;
}

void CUtils_removeChar(unsigned char* haystack, char needle)
{
	unsigned int i = 0;
	unsigned int j = 0;

	while (haystack[i] != '\0') {
		if (haystack[i] != (unsigned char)needle)
			haystack[j++] = haystack[i];
		i++;
	}

	haystack[j] = '\0';
}

void CUtils_createTimestamp(char* buffer, size_t bufferSize)
{
	(void)bufferSize;

#if defined(_WIN32) || defined(_WIN64)
	SYSTEMTIME st;
	GetSystemTime(&st);

	sprintf(buffer, "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
#else
	struct timeval now;
	gettimeofday(&now, NULL);

	struct tm* tm = gmtime(&now.tv_sec);

	sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d.%03lldZ", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, (long long)(now.tv_usec / 1000LL));
#endif
}
