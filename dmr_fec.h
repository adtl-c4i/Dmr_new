/**

/**

/**
 * @file dmr_fec.h
 * @brief MOD-02 — Burst Processor & FEC Engine — Public Interface
 *
 * ETSI TS 102 361-1, Clauses 5–9 (Data Link Layer)
 *
 * FEC algorithms implemented
 * ==========================
 *   BPTC(196,96)      — All data/control burst payloads      Cl. 7.2
 *   Golay(20,8)       — SLOT_TYPE field (12 FEC bits)        Cl. 8.1 / Annex B.3.1
 *   Golay(24,12)      — CACH field (12 FEC bits)             Cl. 8.2 / Annex B.3.1
 *   QR(16,7)          — EMB field parity (9 bits)            Cl. 8.3 / Annex B.3.2
 *   RS(12,9) GF(2^8)  — Full LC FEC (3 bytes)               Cl. 8.4 / Annex B.3.6
 *   Hamming(7,4,3)    — CACH TACT bits                       Annex B.3.5
 *   Hamming(16,11,4)  — BPTC row parity                      Annex B.3.4
 *   CRC-CCITT (16-bit)— Data headers, CSBKs                  Cl. 9.1.7 / Annex B.3.8
 *
 * Integration with LLC (MOD-04)
 * ==============================
 * TX path:  CCL → LLC builds 12-byte PDU → bptc_encode() → llc_burst_pack() → MAC
 * RX path:  MAC → bptc_decode() → llc_burst_unpack() → llc_rx_dispatch() → CCL
 *
 * The functions fec_tx_process() and fec_rx_process() are the main integration
 * points — drop them into the burst pipeline between LLC and MAC.
 *
 * Standards
 * =========
 *   ETSI TS 102 361-1 V2.6.1 (2023-05), Clauses 5–9, Annexes B, E
 */

#ifndef DMR_FEC_H
#define DMR_FEC_H

#define _POSIX_C_SOURCE 200809L

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "dmr_pdu.h"
#include "dmr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * FEC result codes (extend dmr_err_t for FEC-specific conditions)
 * ========================================================================= */
#define DMR_FEC_OK              0    /* No errors                               */
#define DMR_FEC_CORRECTED       1    /* Errors corrected successfully            */
#define DMR_FEC_UNCORRECTABLE   2    /* Too many errors — data unreliable        */

typedef int dmr_fec_result_t;

/* =========================================================================
 * Section 1 — BPTC(196,96)
 * ETSI TS 102 361-1 Clause 7.2 / Annex B.1
 *
 * A BPTC(196,96) block (ETSI TS 102 361-1 Annex B.1.1, Figure B.1):
 *   - 96 information bits I(95)..I(0)
 *   - 3 reserved bits R(0)..R(2) set to zero
 *   - 9x15 matrix: 9 rows x (11 info + 4 parity) bits per row, each row a
 *     Hamming(15,11,3) codeword (implemented via hamming_16_11_encode()'s
 *     11-bit generator table, truncated to its 4 low parity bits)
 *   - 4x15 column parity: each of the 15 columns is its own
 *     Hamming(13,9,3) codeword (9 data rows + 4 parity rows, Table B.14) —
 *     not simple even parity. This is what makes it a genuine Block
 *     *Product* Code: the row code alone has no spare syndrome space to
 *     ever detect a row with 2+ errors, but the column code does (2 of
 *     its 15 possible syndromes are impossible for any single-bit error).
 *   - 1 extra reserved bit R(3) = 0
 *   Total coded: 196 bits
 *
 * After FEC, the 196 bits are interleaved using Table B.2:
 *   Interleave Index = Index × 181 mod 196
 * and placed into INFO_1 (bits 195..99) and INFO_2 (bits 98..2) of the burst,
 * with one bit TX(195) in position 0 of INFO_1 (always R(3)=0).
 *
 * In llc_burst_pack the 96 raw info bits are written to raw[0..11].
 * bptc_encode() reads those 96 bits, builds the 196-bit coded+interleaved
 * block, and writes it into the full INFO_1+INFO_2 fields of raw[].
 *
 * bptc_decode() reads INFO_1+INFO_2 from raw[], deinterleaves, applies
 * row Hamming correction, checks column parity, and writes the 96 clean
 * info bits back to raw[0..11].
 * ========================================================================= */

/**
 * @brief Encode 96 info bits (from raw[0..11]) into full BPTC(196,96)
 *        coded+interleaved layout in INFO_1 and INFO_2 of the burst.
 *
 * Reads raw[0..11] as I(95)..I(0).
 * Writes all 196 interleaved bits directly into the burst's INFO_1/INFO_2
 * byte ranges, skipping the 20 bits reserved for SLOT_TYPE.
 *
 * The SYNC and SLOT_TYPE fields in raw[] are preserved.
 *
 * @param raw   33-byte burst raw[] buffer (modified in-place)
 */
void bptc_encode(uint8_t *raw);

/**
 * @brief Decode a BPTC(196,96) block from INFO_1+INFO_2, correct errors,
 *        and extract the 96 clean info bits back to raw[0..11].
 *
 * Reads the 196 coded bits directly from the burst's INFO_1/INFO_2 byte
 * ranges, skipping the 20 bits reserved for SLOT_TYPE.
 * Applies deinterleaving (Table B.2 inverse), Hamming row correction, column
 * parity check, and writes the 96 recovered info bits to raw[0..11].
 *
 * @param raw       33-byte burst buffer (modified in-place)
 * @return          DMR_FEC_OK, DMR_FEC_CORRECTED, or DMR_FEC_UNCORRECTABLE
 */
dmr_fec_result_t bptc_decode(uint8_t *raw);

/* =========================================================================
 * Section 2 — Golay(20,8) — SLOT_TYPE FEC
 * ETSI TS 102 361-1 Annex B.3.1, Table B.11
 *
 * Input:  8 data bits {CC[3:0], DT[3:0]}
 * Output: 12 parity bits → stored as Golay field in SLOT_TYPE
 * Minimum distance: d=4 (corrects all 1-bit errors, detects all 2-bit)
 * ========================================================================= */

/**
 * @brief Compute 12-bit Golay(20,8) parity for an 8-bit SLOT_TYPE data word.
 *
 * @param data8  8-bit word: {CC[3:0], DT[3:0]}
 * @return       12-bit Golay parity (bits 11:0)
 */
uint16_t golay_20_8_encode(uint8_t data8);

/**
 * @brief Decode and error-correct a 20-bit Golay(20,8) codeword.
 *
 * @param codeword  20-bit received word: {data8[7:0], parity[11:0]}
 * @param data_out  Output: corrected 8-bit data word
 * @return          DMR_FEC_OK, DMR_FEC_CORRECTED, or DMR_FEC_UNCORRECTABLE
 */
dmr_fec_result_t golay_20_8_decode(uint32_t codeword, uint8_t *data_out);

/* =========================================================================
 * Section 3 — Golay(24,12) / Golay(18,6) — CACH EMB FEC
 * ETSI TS 102 361-1 Annex B.3.1
 *
 * The (24,12,8) extended Golay code encodes 12 data bits to 24.
 * The shortened (18,6,8) variant uses only 6 data bits.
 * Both share the same generator polynomial:
 *   g(x) = x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1
 * ========================================================================= */

/**
 * @brief Encode 12 data bits with Golay(24,12) FEC.
 *
 * @param data12  12-bit data (bits 11:0)
 * @return        24-bit codeword {data12[11:0], parity[11:0]}
 */
uint32_t golay_24_12_encode(uint16_t data12);

/**
 * @brief Decode and correct a 24-bit Golay(24,12) codeword.
 *
 * Minimum distance 8: corrects all 3-bit errors.
 *
 * @param codeword   24-bit received codeword
 * @param data_out   Output: corrected 12-bit data
 * @return           DMR_FEC_OK, DMR_FEC_CORRECTED, or DMR_FEC_UNCORRECTABLE
 */
dmr_fec_result_t golay_24_12_decode(uint32_t codeword, uint16_t *data_out);

/* =========================================================================
 * Section 4 — QR(16,7,6) — EMB field parity
 * ETSI TS 102 361-1 Annex B.3.2, Table B.12
 *
 * Shortened quadratic residue code, formed from the primitive (17,9,5) code.
 * G(x) = x^8 + x^5 + x^4 + x^3 + 1
 * Used to protect the 7-bit EMB data (CC+PI+LCSS) with 9 parity bits.
 * ========================================================================= */

/**
 * @brief Encode 7 EMB data bits with QR(16,7) FEC.
 *
 * @param data7   7-bit EMB data {CC[3:0], PI, LCSS[1:0]}
 * @return        16-bit QR codeword {data7[6:0], parity[8:0]}
 */
uint16_t qr_16_7_encode(uint8_t data7);

/**
 * @brief Decode and correct a 16-bit QR(16,7) codeword.
 *
 * @param codeword   16-bit received QR codeword
 * @param data_out   Output: corrected 7-bit data
 * @return           DMR_FEC_OK, DMR_FEC_CORRECTED, or DMR_FEC_UNCORRECTABLE
 */
dmr_fec_result_t qr_16_7_decode(uint16_t codeword, uint8_t *data_out);

/* =========================================================================
 * Section 5 — RS(12,9) GF(2^8) — Full LC FEC
 * ETSI TS 102 361-1 Annex B.3.6, Tables B.18-B.20
 *
 * Shortened Reed-Solomon code over GF(2^8):
 *   Generator polynomial: G(x) = (x+α)(x+α²)(x+α³)
 *   g(x) = x³ + 0x0E·x² + 0x38·x + 0x40
 *   Field polynomial: α^8 + α^4 + α^3 + α^2 + 1 = 0x11D
 *   (12,9,4) code: 9 data symbols, 3 parity symbols, minimum distance 4
 *
 * Applied to the 12-byte Full LC PDU:
 *   - Bytes 0-8:  9 data symbols (FLCO, FID, SVC, DST, SRC)
 *   - Bytes 9-11: 3 RS parity symbols
 * ========================================================================= */

/**
 * @brief Compute 3 RS(12,9) parity bytes for a 9-byte Full LC message.
 *
 * @param data9    9 data bytes (Full LC bytes 0-8)
 * @param parity3  Output: 3 RS parity bytes (to go into Full LC bytes 9-11)
 */
void rs_12_9_encode(const uint8_t data9[9], uint8_t parity3[3]);

/**
 * @brief Decode and correct up to 1 symbol error in a 12-byte RS(12,9) codeword.
 *
 * @param codeword12  12-byte RS codeword (9 data + 3 parity)
 * @param data9_out   Output: corrected 9 data bytes (may equal codeword12)
 * @return            DMR_FEC_OK, DMR_FEC_CORRECTED, or DMR_FEC_UNCORRECTABLE
 */
dmr_fec_result_t rs_12_9_decode(const uint8_t codeword12[12],
                                  uint8_t       data9_out[9]);

/* =========================================================================
 * Section 6 — Hamming(7,4,3) — CACH TACT bits
 * ETSI TS 102 361-1 Annex B.3.5, Table B.17
 *
 * Primitive Hamming code: G(x) = x³ + x + 1
 * Encodes 4 data bits to 7-bit codeword.
 * Corrects all 1-bit errors.
 * ========================================================================= */

/**
 * @brief Encode 4-bit data with Hamming(7,4) FEC.
 *
 * @param data4  4-bit data (bits 3:0)
 * @return       7-bit codeword {data4[3:0], parity[2:0]}
 */
uint8_t hamming_7_4_encode(uint8_t data4);

/**
 * @brief Decode and correct a 7-bit Hamming(7,4) codeword.
 *
 * @param codeword7  7-bit received codeword
 * @param data_out   Output: corrected 4-bit data
 * @return           DMR_FEC_OK, DMR_FEC_CORRECTED, or DMR_FEC_UNCORRECTABLE
 */
dmr_fec_result_t hamming_7_4_decode(uint8_t codeword7, uint8_t *data_out);

/* =========================================================================
 * Section 7 — Hamming(16,11,4) — BPTC row parity
 * ETSI TS 102 361-1 Annex B.3.4, Table B.16
 *
 * Extended Hamming code from (15,11,3) primitive.
 * G(x) = x^4 + x + 1 = 0x13
 * Encodes 11 data bits to 16-bit codeword (with overall parity extension).
 * Corrects all 1-bit errors, detects all 2-bit errors.
 * ========================================================================= */

/**
 * @brief Encode 11-bit data with Hamming(16,11) FEC.
 *
 * @param data11  11-bit data (bits 10:0)
 * @return        16-bit codeword {data11[10:0], parity[4:0]} — upper 5 bits parity
 */
uint16_t hamming_16_11_encode(uint16_t data11);

/**
 * @brief Decode and correct a 16-bit Hamming(16,11) codeword (BPTC row).
 *
 * @param codeword16  16-bit received codeword
 * @param data_out    Output: corrected 11-bit data (bits 10:0 of result)
 * @return            DMR_FEC_OK, DMR_FEC_CORRECTED, or DMR_FEC_UNCORRECTABLE
 */
dmr_fec_result_t hamming_16_11_decode(uint16_t codeword16, uint16_t *data_out);

/* =========================================================================
 * Section 8 — Pipeline integration
 *
 * These are the primary integration points for the burst processor.
 * Call fec_tx_process() after LLC builds the burst (to apply BPTC + RS FEC).
 * Call fec_rx_process() before LLC parses the burst (to decode + correct).
 * ========================================================================= */

/**
 * @brief TX pipeline: apply all FEC to a burst built by LLC.
 *
 * Operations performed:
 *   1. For Voice LC Header / Terminator bursts:
 *      - Compute RS(12,9) parity for Full LC bytes (writes bytes 9-11)
 *   2. For all data bursts (CSBK, data header, data block, idle, LC hdr, term):
 *      - BPTC(196,96) encode: raw[0..11] → 196-bit coded+interleaved layout
 *        written to INFO_1 + INFO_2
 *   3. For voice bursts (B-F) with EMB:
 *      - Compute QR(16,7) parity for EMB ctrl word (CC+PI+LCSS)
 *        and write into the EMB field
 *
 * @param burst   Burst to process in-place (raw[] modified)
 */
void fec_tx_process(dmr_burst_t *burst);

/**
 * @brief RX pipeline: decode and correct all FEC in a received burst.
 *
 * Operations performed:
 *   1. For data bursts:
 *      - Verify Golay(20,8) SLOT_TYPE FEC; attempt correction
 *      - BPTC(196,96) decode: INFO_1+INFO_2 → raw[0..11]
 *   2. For Voice LC Header / Terminator bursts (after BPTC):
 *      - RS(12,9) decode: verify/correct Full LC parity
 *   3. For voice bursts B-F:
 *      - QR(16,7) decode: verify/correct EMB ctrl word
 *
 * @param burst   Burst to decode in-place (raw[] modified)
 * @return        DMR_FEC_OK, DMR_FEC_CORRECTED, or DMR_FEC_UNCORRECTABLE
 */
dmr_fec_result_t fec_rx_process(dmr_burst_t *burst);

/* =========================================================================
 * Section 9 — Diagnostic helpers
 * ========================================================================= */

/**
 * @brief Verify BPTC(196,96) integrity of a burst without modifying it.
 * @return true if all Hamming rows and column parities check out
 */
bool bptc_check(const uint8_t *raw);

/**
 * @brief Return the Hamming(16,11) syndrome for a BPTC row.
 *        Syndrome = 0 means no error.
 */
uint8_t hamming_16_11_syndrome(uint16_t codeword16);

/* =========================================================================
 * Section 10 — BPTC(128) Embedded LC assembly/disassembly
 * ETSI TS 102 361-1 Annex B.2.1 (Figures B.2, B.3), Annex B.3.11
 *
 * Reassembles the 72-bit Full LC carried across 4 voice bursts (B,C,D,E).
 * Distinct from BPTC(196,96) in Section 1 (general data bursts) — shares
 * only the Hamming(16,11,4) row code from Section 7.
 *
 * The physical EMB LC field is 32 bits per burst, not 24 — fragments must
 * be collected as 4-byte/uint32_t values, not 3-byte ones.
 * ========================================================================= */

/**
 * @brief Encode a 9-byte (72-bit) LC into 4 BPTC(128) burst fragments.
 * @param lc9      9 raw LC bytes (Full LC bytes 0-8)
 * @param frags4   Output: 4 fragments (bursts B, C, D, E in order), each
 *                 4 bytes MSB-first (frags4[k][0]=bits 31..24 ... [3]=bits
 *                 7..0) — matches dmr_burst_set_emb()'s lc_frag[4] convention
 */
void emblc_bptc_encode(const uint8_t lc9[9], uint8_t frags4[4][4]);

/**
 * @brief Decode 4 received BPTC(128) fragments back into a 9-byte LC,
 *        correcting up to one bit error per matrix row.
 * @param frags4         4 received fragments (B, C, D, E in order), each
 *                        4 bytes MSB-first, as produced by dmr_burst_get_emb_lc()
 * @param lc9_out        Output: 9 recovered LC bytes. Zeroed (not left with
 *                        partially-decoded garbage) if this returns false.
 * @param corrected_rows Output (optional): number of rows Hamming corrected
 *                        (only meaningful if this returns true — set to the
 *                        count of rows successfully corrected before an
 *                        UNCORRECTABLE row was hit, if that's why it failed)
 * @return false if EITHER any row has 2+ bit errors (Hamming(16,11,4)
 *         reports DMR_FEC_UNCORRECTABLE — a certain signal) OR the
 *         checksum doesn't match (catches most, not all, other corruption
 *         — a 5-bit checksum only rejects ~31/32 of random corruption by
 *         chance). true only if both checks pass, meaning lc9_out is
 *         trustworthy.
 */
bool emblc_bptc_decode(const uint8_t frags4[4][4], uint8_t lc9_out[9], int *corrected_rows);

#ifdef __cplusplus
}
#endif

#endif /* DMR_FEC_H */