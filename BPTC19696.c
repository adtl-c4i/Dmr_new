/*
 *	 Copyright (C) 2012 by Ian Wraith
 *   Copyright (C) 2015,2023,2025 by Jonathan Naylor G4KLX
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
 *   Converted from C++ to C (see BPTC19696.h for the summary of changes).
 *   `new bool[196]` / `delete[]` become `malloc`/`free`; `CBPTC19696::foo(...)`
 *   methods become `static` functions taking `CBPTC19696* self` as their
 *   first argument in place of the implicit `this`. `CUtils::bitsToByteBE`,
 *   whose C++ signature returned through a `unsigned char&`, is called with
 *   `&data[i]` here since the C version takes a pointer. Logic is otherwise
 *   untouched line-for-line.
 */

#include "BPTC19696.h"
#include "Hamming.h"
#include "Utils.h"

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>

static void decodeExtractBinary(CBPTC19696* self, const unsigned char* in);
static void decodeErrorCheck(CBPTC19696* self);
static void decodeDeInterleave(CBPTC19696* self);
static void decodeExtractData(const CBPTC19696* self, unsigned char* data);

static void encodeExtractData(const CBPTC19696* self, const unsigned char* in);
static void encodeInterleave(CBPTC19696* self);
static void encodeErrorCheck(CBPTC19696* self);
static void encodeExtractBinary(CBPTC19696* self, unsigned char* data);

void CBPTC19696_init(CBPTC19696* self)
{
	assert(self != NULL);

	self->m_rawData     = (bool*)malloc(196U * sizeof(bool));
	self->m_deInterData = (bool*)malloc(196U * sizeof(bool));
}

void CBPTC19696_free(CBPTC19696* self)
{
	assert(self != NULL);

	free(self->m_rawData);
	free(self->m_deInterData);
	self->m_rawData     = NULL;
	self->m_deInterData = NULL;
}

/* The main decode function */
void CBPTC19696_decode(CBPTC19696* self, const unsigned char* in, unsigned char* out)
{
	assert(self != NULL);
	assert(in != NULL);
	assert(out != NULL);

	/*  Get the raw binary */
	decodeExtractBinary(self, in);

	/* Deinterleave */
	decodeDeInterleave(self);

	/* Error check */
	decodeErrorCheck(self);

	/* Extract Data */
	decodeExtractData(self, out);
}

/* The main encode function */
void CBPTC19696_encode(CBPTC19696* self, const unsigned char* in, unsigned char* out)
{
	assert(self != NULL);
	assert(in != NULL);
	assert(out != NULL);

	/* Extract Data */
	encodeExtractData(self, in);

	/* Error check */
	encodeErrorCheck(self);

	/* Deinterleave */
	encodeInterleave(self);

	/*  Get the raw binary */
	encodeExtractBinary(self, out);
}

static void decodeExtractBinary(CBPTC19696* self, const unsigned char* in)
{
	/* First block */
	CUtils_byteToBitsBE(in[0U],  self->m_rawData + 0U);
	CUtils_byteToBitsBE(in[1U],  self->m_rawData + 8U);
	CUtils_byteToBitsBE(in[2U],  self->m_rawData + 16U);
	CUtils_byteToBitsBE(in[3U],  self->m_rawData + 24U);
	CUtils_byteToBitsBE(in[4U],  self->m_rawData + 32U);
	CUtils_byteToBitsBE(in[5U],  self->m_rawData + 40U);
	CUtils_byteToBitsBE(in[6U],  self->m_rawData + 48U);
	CUtils_byteToBitsBE(in[7U],  self->m_rawData + 56U);
	CUtils_byteToBitsBE(in[8U],  self->m_rawData + 64U);
	CUtils_byteToBitsBE(in[9U],  self->m_rawData + 72U);
	CUtils_byteToBitsBE(in[10U], self->m_rawData + 80U);
	CUtils_byteToBitsBE(in[11U], self->m_rawData + 88U);
	CUtils_byteToBitsBE(in[12U], self->m_rawData + 96U);

	/* Handle the two bits */
	bool bits[8U];
	CUtils_byteToBitsBE(in[20U], bits);
	self->m_rawData[98U] = bits[6U];
	self->m_rawData[99U] = bits[7U];

	/* Second block */
	CUtils_byteToBitsBE(in[21U], self->m_rawData + 100U);
	CUtils_byteToBitsBE(in[22U], self->m_rawData + 108U);
	CUtils_byteToBitsBE(in[23U], self->m_rawData + 116U);
	CUtils_byteToBitsBE(in[24U], self->m_rawData + 124U);
	CUtils_byteToBitsBE(in[25U], self->m_rawData + 132U);
	CUtils_byteToBitsBE(in[26U], self->m_rawData + 140U);
	CUtils_byteToBitsBE(in[27U], self->m_rawData + 148U);
	CUtils_byteToBitsBE(in[28U], self->m_rawData + 156U);
	CUtils_byteToBitsBE(in[29U], self->m_rawData + 164U);
	CUtils_byteToBitsBE(in[30U], self->m_rawData + 172U);
	CUtils_byteToBitsBE(in[31U], self->m_rawData + 180U);
	CUtils_byteToBitsBE(in[32U], self->m_rawData + 188U);
}

/* Deinterleave the raw data */
static void decodeDeInterleave(CBPTC19696* self)
{
	for (unsigned int i = 0U; i < 196U; i++)
		self->m_deInterData[i] = false;

	/* The first bit is R(3) which is not used so can be ignored */
	for (unsigned int a = 0U; a < 196U; a++)	{
		/* Calculate the interleave sequence */
		unsigned int interleaveSequence = (a * 181U) % 196U;
		/* Shuffle the data */
		self->m_deInterData[a] = self->m_rawData[interleaveSequence];
	}
}

/* Check each row with a Hamming (15,11,3) code and each column with a Hamming (13,9,3) code */
static void decodeErrorCheck(CBPTC19696* self)
{
	bool fixing;
	unsigned int count = 0U;
	do {
		fixing = false;

		/* Run through each of the 15 columns */
		bool col[13U];
		for (unsigned int c = 0U; c < 15U; c++) {
			unsigned int pos = c + 1U;
			for (unsigned int a = 0U; a < 13U; a++) {
				col[a] = self->m_deInterData[pos];
				pos = pos + 15U;
			}

			if (CHamming_decode1393(col)) {
				unsigned int pos2 = c + 1U;
				for (unsigned int a = 0U; a < 13U; a++) {
					self->m_deInterData[pos2] = col[a];
					pos2 = pos2 + 15U;
				}

				fixing = true;
			}
		}

		/* Run through each of the 9 rows containing data */
		for (unsigned int r = 0U; r < 9U; r++) {
			unsigned int pos = (r * 15U) + 1U;
			if (CHamming_decode15113_2(self->m_deInterData + pos))
				fixing = true;
		}

		count++;
	} while (fixing && count < 5U);
}

/* Extract the 96 bits of payload */
static void decodeExtractData(const CBPTC19696* self, unsigned char* data)
{
	bool bData[96U];
	unsigned int pos = 0U;
	for (unsigned int a = 4U; a <= 11U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	for (unsigned int a = 16U; a <= 26U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	for (unsigned int a = 31U; a <= 41U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	for (unsigned int a = 46U; a <= 56U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	for (unsigned int a = 61U; a <= 71U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	for (unsigned int a = 76U; a <= 86U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	for (unsigned int a = 91U; a <= 101U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	for (unsigned int a = 106U; a <= 116U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	for (unsigned int a = 121U; a <= 131U; a++, pos++)
		bData[pos] = self->m_deInterData[a];

	CUtils_bitsToByteBE(bData + 0U,  &data[0U]);
	CUtils_bitsToByteBE(bData + 8U,  &data[1U]);
	CUtils_bitsToByteBE(bData + 16U, &data[2U]);
	CUtils_bitsToByteBE(bData + 24U, &data[3U]);
	CUtils_bitsToByteBE(bData + 32U, &data[4U]);
	CUtils_bitsToByteBE(bData + 40U, &data[5U]);
	CUtils_bitsToByteBE(bData + 48U, &data[6U]);
	CUtils_bitsToByteBE(bData + 56U, &data[7U]);
	CUtils_bitsToByteBE(bData + 64U, &data[8U]);
	CUtils_bitsToByteBE(bData + 72U, &data[9U]);
	CUtils_bitsToByteBE(bData + 80U, &data[10U]);
	CUtils_bitsToByteBE(bData + 88U, &data[11U]);
}

/* Extract the 96 bits of payload */
static void encodeExtractData(const CBPTC19696* self, const unsigned char* in)
{
	bool bData[96U];
	CUtils_byteToBitsBE(in[0U],  bData + 0U);
	CUtils_byteToBitsBE(in[1U],  bData + 8U);
	CUtils_byteToBitsBE(in[2U],  bData + 16U);
	CUtils_byteToBitsBE(in[3U],  bData + 24U);
	CUtils_byteToBitsBE(in[4U],  bData + 32U);
	CUtils_byteToBitsBE(in[5U],  bData + 40U);
	CUtils_byteToBitsBE(in[6U],  bData + 48U);
	CUtils_byteToBitsBE(in[7U],  bData + 56U);
	CUtils_byteToBitsBE(in[8U],  bData + 64U);
	CUtils_byteToBitsBE(in[9U],  bData + 72U);
	CUtils_byteToBitsBE(in[10U], bData + 80U);
	CUtils_byteToBitsBE(in[11U], bData + 88U);

	for (unsigned int i = 0U; i < 196U; i++)
		self->m_deInterData[i] = false;

	unsigned int pos = 0U;
	for (unsigned int a = 4U; a <= 11U; a++, pos++)
		self->m_deInterData[a] = bData[pos];

	for (unsigned int a = 16U; a <= 26U; a++, pos++)
		self->m_deInterData[a] = bData[pos];

	for (unsigned int a = 31U; a <= 41U; a++, pos++)
		self->m_deInterData[a] = bData[pos];

	for (unsigned int a = 46U; a <= 56U; a++, pos++)
		self->m_deInterData[a] = bData[pos];

	for (unsigned int a = 61U; a <= 71U; a++, pos++)
		self->m_deInterData[a] = bData[pos];

	for (unsigned int a = 76U; a <= 86U; a++, pos++)
		self->m_deInterData[a] = bData[pos];

	for (unsigned int a = 91U; a <= 101U; a++, pos++)
		self->m_deInterData[a] = bData[pos];

	for (unsigned int a = 106U; a <= 116U; a++, pos++)
		self->m_deInterData[a] = bData[pos];

	for (unsigned int a = 121U; a <= 131U; a++, pos++)
		self->m_deInterData[a] = bData[pos];
}

/* Check each row with a Hamming (15,11,3) code and each column with a Hamming (13,9,3) code */
static void encodeErrorCheck(CBPTC19696* self)
{
	/* Run through each of the 9 rows containing data */
	for (unsigned int r = 0U; r < 9U; r++) {
		unsigned int pos = (r * 15U) + 1U;
		CHamming_encode15113_2(self->m_deInterData + pos);
	}

	/* Run through each of the 15 columns */
	bool col[13U];
	for (unsigned int c = 0U; c < 15U; c++) {
		unsigned int pos = c + 1U;
		for (unsigned int a = 0U; a < 13U; a++) {
			col[a] = self->m_deInterData[pos];
			pos = pos + 15U;
		}

		CHamming_encode1393(col);

		pos = c + 1U;
		for (unsigned int a = 0U; a < 13U; a++) {
			self->m_deInterData[pos] = col[a];
			pos = pos + 15U;
		}
	}
}

/* Interleave the raw data */
static void encodeInterleave(CBPTC19696* self)
{
	for (unsigned int i = 0U; i < 196U; i++)
		self->m_rawData[i] = false;

	/* The first bit is R(3) which is not used so can be ignored */
	for (unsigned int a = 0U; a < 196U; a++)	{
		/* Calculate the interleave sequence */
		unsigned int interleaveSequence = (a * 181U) % 196U;
		/* Unshuffle the data */
		self->m_rawData[interleaveSequence] = self->m_deInterData[a];
	}
}

static void encodeExtractBinary(CBPTC19696* self, unsigned char* data)
{
	/* First block */
	CUtils_bitsToByteBE(self->m_rawData + 0U,  &data[0U]);
	CUtils_bitsToByteBE(self->m_rawData + 8U,  &data[1U]);
	CUtils_bitsToByteBE(self->m_rawData + 16U, &data[2U]);
	CUtils_bitsToByteBE(self->m_rawData + 24U, &data[3U]);
	CUtils_bitsToByteBE(self->m_rawData + 32U, &data[4U]);
	CUtils_bitsToByteBE(self->m_rawData + 40U, &data[5U]);
	CUtils_bitsToByteBE(self->m_rawData + 48U, &data[6U]);
	CUtils_bitsToByteBE(self->m_rawData + 56U, &data[7U]);
	CUtils_bitsToByteBE(self->m_rawData + 64U, &data[8U]);
	CUtils_bitsToByteBE(self->m_rawData + 72U, &data[9U]);
	CUtils_bitsToByteBE(self->m_rawData + 80U, &data[10U]);
	CUtils_bitsToByteBE(self->m_rawData + 88U, &data[11U]);

	/* Handle the two bits */
	unsigned char byte;
	CUtils_bitsToByteBE(self->m_rawData + 96U, &byte);
	data[12U] = (data[12U] & 0x3FU) | ((byte >> 0) & 0xC0U);
	data[20U] = (data[20U] & 0xFCU) | ((byte >> 4) & 0x03U);

	/* Second block */
	CUtils_bitsToByteBE(self->m_rawData + 100U, &data[21U]);
	CUtils_bitsToByteBE(self->m_rawData + 108U, &data[22U]);
	CUtils_bitsToByteBE(self->m_rawData + 116U, &data[23U]);
	CUtils_bitsToByteBE(self->m_rawData + 124U, &data[24U]);
	CUtils_bitsToByteBE(self->m_rawData + 132U, &data[25U]);
	CUtils_bitsToByteBE(self->m_rawData + 140U, &data[26U]);
	CUtils_bitsToByteBE(self->m_rawData + 148U, &data[27U]);
	CUtils_bitsToByteBE(self->m_rawData + 156U, &data[28U]);
	CUtils_bitsToByteBE(self->m_rawData + 164U, &data[29U]);
	CUtils_bitsToByteBE(self->m_rawData + 172U, &data[30U]);
	CUtils_bitsToByteBE(self->m_rawData + 180U, &data[31U]);
	CUtils_bitsToByteBE(self->m_rawData + 188U, &data[32U]);
}

