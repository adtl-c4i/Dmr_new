



/**
 * @file llc_crc.c
 * @brief CRC-CCITT implementation
 *
 * ETSI TS 102 361-1, Annex B.3.8
 *
 * Generator : G(x) = x^16 + x^12 + x^5 + 1  (0x1021)
 * Initial   : 0x0000
 * Final XOR : 0xFFFF
 * Bit order : MSB first, no reflection
 *
 * The standard specifies:
 *   FH(x) = (x^16 * M(x) mod GH(x)) + IH(x)   [mod 2]
 * where IH(x) = x^15 + x^14 + … + x + 1  (= 0xFFFF).
 * Equivalent to: compute CRC with init=0x0000, then XOR result with 0xFFFF.
 */

#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "dmr_llc.h"

/* ETSI TS 102 361-1 Annex B.3.10, formula B.23: G9(x) = x^9 + x^6 + x^4 + x^3 + 1.
 * Stored as a 9-bit constant with the leading x^9 term implicit (standard
 * LFSR convention — it's the overflow bit that triggers the XOR, not a
 * stored bit): bits at positions 6,4,3,0 = 0x059. The previous value here
 * (0x015B) had spurious x^8 and x^1 terms not in the spec polynomial at all. */
#define DMR_CRC9_POLY 0x059
/* Pre-computed CRC-CCITT (0x1021) table for fast byte-at-a-time processing */
static const uint16_t crc_table[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
    0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
    0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
    0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
    0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
};

// Standard CRC-16-CCITT calculation (ETSI Annex B.3.8)
uint16_t llc_crc_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x0000u;
    for (size_t i = 0u; i < len; i++) {
        crc = (uint16_t)((crc << 8) ^ crc_table[(crc >> 8) ^ data[i]]);
    }
    return crc ^ 0xFFFFu;   /* Final ones-complement per ETSI */
}

// Append CRC with DMR Masking (e.g., mask = 0xCCCC for Data Header)
void llc_crc_append(uint8_t *data, size_t payload_len, uint16_t mask)
{
    
    
    
	uint16_t crc16 = 0U;

	for (unsigned i = 0U; i < (payload_len); i++) {
		uint8_t lowByte  = (uint8_t)(crc16 & 0xFFU);
		uint8_t highByte = (uint8_t)((crc16 >> 8) & 0xFFU);
		crc16 = ((uint16_t)lowByte << 8) ^ crc_table[highByte ^ data[i]];
	}

	crc16 = (uint16_t)~crc16;



    uint16_t crc = (crc16 & 0xFFFFU) ^ (mask& 0xFFFFU);

	data[payload_len] = (uint8_t)((crc >> 8) & 0xFFU); /* high byte, sent/stored first (MSB first) */
	data[payload_len+1] = (uint8_t)(crc & 0xFFU);         /* low byte, sent/stored second */


}

/* Verify CRC with DMR Masking (e.g., mask = 0xCCCC for Data Header) */
bool llc_crc_verify(const uint8_t *data, size_t total_len, uint16_t mask)
{
    if (total_len < 2u) {
        return false;
    }

    size_t payload_len = total_len - 2u;

    /* Calculate expected CRC with mask */
    uint16_t expected = llc_crc_ccitt(data, payload_len) ^ mask;

    /* Extract received Big-Endian 16-bit CRC from frame */
    uint16_t received = (uint16_t)(((uint16_t)data[payload_len] << 8U) |
                                   ((uint16_t)data[payload_len + 1U] & 0xFFu));

    return (expected == received);
}




/**
 * @brief Calculates 9-bit CRC for DMR payload bits.
 * ETSI TS 102 361-1 Annex B.3.10, formula B.25:
 *   F9(x) = (x^9 * M(x) mod G9(x)) + I9(x)
 * where I9(x) = x^8+x^7+...+x+1 = 0x1FF (all 9 bits set) — a final XOR,
 * exactly analogous to llc_crc_ccitt()'s final XOR with 0xFFFF above.
 */
uint16_t dmr_crc9_calc(const uint8_t *buffer, size_t bit_len) {
    uint16_t crc = 0;

    for (size_t i = 0; i < bit_len; i++) {
        uint8_t bit = (buffer[i / 8] >> (7 - (i % 8))) & 0x01;
        uint16_t msb = (crc >> 8) & 0x01;
        crc = ((crc << 1) | bit) & 0x01FF;

        if (msb) {
            crc ^= DMR_CRC9_POLY;
        }
    }

    return (crc & 0x01FF) ^ 0x01FFu;   /* + I9(x) */
}

/**
 * @brief Appends a 9-bit CRC to the end of a payload byte stream.
 * 
 * @param buffer Buffer holding the data payload. Must have capacity for (payload_bit_len + 9) bits.
 * @param payload_bit_len Number of payload bits.
 */
void dmr_crc9_append(uint8_t *buffer, size_t payload_bit_len) {
    // 1. Calculate CRC over the payload bits
    uint16_t crc = dmr_crc9_calc(buffer, payload_bit_len);

    // 2. Append the 9 bits (MSB to LSB) directly after the payload bits
    for (int i = 0; i < 9; i++) {
        size_t bit_pos = payload_bit_len + i;
        uint8_t crc_bit = (crc >> (8 - i)) & 0x01;  // Bit 8 down to Bit 0

        if (crc_bit) {
            buffer[bit_pos / 8] |= (1 << (7 - (bit_pos % 8)));   // Set bit
        } else {
            buffer[bit_pos / 8] &= ~(1 << (7 - (bit_pos % 8)));  // Clear bit
        }
    }
}

/**
 * @brief Verifies the 9-bit CRC of a received block (Payload + 9-bit CRC).
 * 
 * @param buffer Buffer containing payload + 9-bit CRC.
 * @param total_bit_len Total length of the message in bits (Payload bits + 9).
 * @return true if valid, false if corrupt.
 */
bool dmr_crc9_verify(const uint8_t *buffer, size_t total_bit_len) {
    if (total_bit_len <= 9) return false;

    size_t payload_bit_len = total_bit_len - 9;

    // 1. Calculate CRC on payload portion
    uint16_t calc_crc = dmr_crc9_calc(buffer, payload_bit_len);

    // 2. Extract original 9-bit CRC from the stream
    uint16_t recv_crc = 0;
    for (int i = 0; i < 9; i++) {
        size_t bit_pos = payload_bit_len + i;
        uint8_t bit = (buffer[bit_pos / 8] >> (7 - (bit_pos % 8))) & 0x01;
        recv_crc = (recv_crc << 1) | bit;
    }

    // 3. Compare calculated CRC against received CRC
    return (calc_crc == recv_crc);
}





