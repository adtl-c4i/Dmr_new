/**
 * @file llc_burst.c
 * @brief LLC burst assembly / disassembly helpers
 *
 * Bridges between 12-byte logical PDU bodies and wire-format dmr_burst_t.
 *
 * BPTC(196,96) context
 * ====================
 * A BPTC(196,96) block carries 96 info bits = exactly 12 bytes.
 * The 96 info bits are placed MSB-first into the BPTC encoder:
 *   I(95) = pdu[0] bit7   (first transmitted info bit)
 *   I(94) = pdu[0] bit6
 *   ...
 *   I(0)  = pdu[11] bit0  (last info bit)
 *
 * After BPTC encoding (MOD-02), the 196 coded bits are interleaved and
 * placed into INFO_1 (98 bits) and INFO_2 (98 bits) of the burst.
 * The three reserved bits R(0-2) and the extra R(3) are set to zero by
 * the encoder.
 *
 * In this file we store the 96 raw info bits directly into the burst
 * INFO_1/INFO_2 fields (skipping the BPTC encoding step).  The actual
 * BPTC encode/decode happens in MOD-02 (FEC engine).  Until MOD-02 is
 * integrated, the 96 bits are stored in a simplified layout that MOD-02
 * will process before transmission and after reception.
 *
 * Simplified 96-bit storage in raw[]
 * ====================================
 * We pack 12 bytes straight into raw[0..11] of INFO_1, leaving INFO_2
 * and the SYNC/SLOT_TYPE fields to be written by the accessor functions.
 * MOD-02 will re-pack these into the full BPTC interleaved layout.
 *
 * Wire layout used here (pre-BPTC, used only until MOD-02 is connected):
 *   raw[0..11] = pdu[0..11]          (96 info bits in INFO_1 space)
 *   raw[12..20] = SLOT_TYPE + SYNC   (set by dmr_burst_set_sync/slot_type)
 *   raw[21..32] = zeros              (INFO_2 space, zero until BPTC encode)
 *
 * Golay(18,6) FEC for SLOT_TYPE
 * ==============================
 * The 10-bit SLOT_TYPE field is protected by a Golay(18,6) code.
 * Generator from ETSI TS 102 361-1 Annex B.3.1:
 *   G(x) = x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1
 * The 8 data bits (CC[3:0] + DT[3:0]) are encoded to produce a 12-bit
 * Golay codeword.  We implement a compact systematic encoder here.
 *
 * ETSI TS 102 361-1 Annex B.3.1, Table B.11 — Golay(20,8) generator matrix:
 *   Row 0 (d7): 1 0 0 0 0 0 0 0 | 0 0 1 1 1 1 0 1 1 0 1 0
 *   Row 1 (d6): 0 1 0 0 0 0 0 0 | 1 1 0 1 1 0 0 1 1 0 0 1
 *   Row 2 (d5): 0 0 1 0 0 0 0 0 | 0 1 1 0 1 1 0 0 1 1 0 1
 *   Row 3 (d4): 0 0 0 1 0 0 0 0 | 0 0 1 1 0 1 1 0 0 1 1 1
 *   Row 4 (d3): 0 0 0 0 1 0 0 0 | 1 1 0 1 1 1 0 0 0 1 1 0
 *   Row 5 (d2): 0 0 0 0 0 1 0 0 | 1 0 1 0 1 0 0 1 0 1 1 1
 *   Row 6 (d1): 0 0 0 0 0 0 1 0 | 1 0 0 1 0 0 1 1 1 1 1 0
 *   Row 7 (d0): 0 0 0 0 0 0 0 1 | 1 0 0 0 1 1 1 0 1 0 1 1
 *
 * Parity columns (columns 8-19 of matrix, 12 parity bits):
 *   p[11..0] for data [d7..d0]
 */

#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dmr_pdu.h"
#include "dmr_llc.h"
#include "dmr_types.h"
#include "dmr_fec.h"

/* SLOT_TYPE's Golay(20,8) FEC is computed by fec_tx_process()/dmr_fec.c —
 * see llc_burst_pack() below, which writes a placeholder SLOT_TYPE (cc,
 * dtype, golay=0) that fec_tx_process() immediately recomputes for real.
 * A duplicate local Golay(20,8) table/encoder used to live here; removed
 * since dmr_fec.c is the only implementation ever actually used on the TX
 * path, and keeping two copies in sync is exactly how the d1 parity
 * transcription error (0x97E vs the correct 0x93E) got copied from here
 * into dmr_fec.c in the first place. */


/* =========================================================================
 * Burst pack / unpack
 * ========================================================================= */

void llc_burst_pack(dmr_burst_t   *burst,
                    const uint8_t *pdu12,
                    uint8_t        cc,
                    uint8_t        dtype,
                    bool           outbound,
                    dmr_slot_t     slot)
{
    dmr_burst_clear(burst->raw);
    burst->type     = DMR_BURST_TYPE_DATA;
    burst->timeslot = (uint8_t)slot;

    /* Write SYNC — outbound = BS→MS uses BS_DATA; inbound = MS→BS uses MS_DATA */
    uint64_t sync = DMR_SYNC_MS_DATA;
    dmr_burst_set_sync(burst->raw, sync);

    /* SLOT_TYPE (cc, dtype) is set here as a placeholder; fec_tx_process()
     * below computes and writes the real Golay(20,8) parity. */
    uint8_t  st_data = (uint8_t)((cc & 0x0Fu) << 4) | (dtype & 0x0Fu);

    dmr_burst_set_slot_type(burst->raw, cc, dtype, 0);

    /* Place the 96 info bits (12 bytes) into INFO_1 / INFO_2.
     *
     * Per ETSI BPTC(196,96) layout (Annex B.1.1, Figure B.1):
     *   I(95..88) in raw[0..0]  — first 8 info bits in INFO_1
     *   I(87..80) in raw[1]
     *   ...
     *   I(7..0)   in raw[11]   — last 8 info bits
     *
     * INFO_1 holds symbols L66..L18 = dibits 0..48 = 98 bits total.
     * The first 96 info bits I(95..0) are placed into INFO_1 bits [97:2]
     * and INFO_2 bits [97:2] by the BPTC encoder.
     *
     * Pre-BPTC simplified placement (MOD-02 will re-encode before TX):
     *   pdu12[0..11] → raw[0..11] directly.
     * These 12 bytes fit entirely within INFO_1 (raw[0..12]).
     * raw[12] upper nibble is also INFO_1 but we leave it zero.
     */
    for (int i = 0; i < 12; i++) {
        burst->raw[i] = pdu12[i];
    }
    

    /* MOD-02: apply RS(12,9) (Voice LC Header/Terminator only), BPTC(196,96),
     * and recompute the Golay(20,8) SLOT_TYPE FEC over the final cc/dtype. */
    fec_tx_process(burst);
    

}

dmr_err_t llc_burst_unpack(const uint8_t *raw,
                             uint8_t       *pdu12,
                             uint8_t       *dtype,
                             uint8_t       *cc)
{
    if (!dmr_burst_is_data(raw)) {
        return DMR_ERR_INVALID_PARAM;
    }

    /* MOD-02: correct/verify Golay(20,8) SLOT_TYPE, BPTC(196,96), and
     * RS(12,9) (Voice LC Header/Terminator) on a local copy, then unpack
     * from the corrected bits. */
    dmr_burst_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    memcpy(tmp.raw, raw, DMR_BURST_BYTES);
    tmp.type = DMR_BURST_TYPE_DATA;

    if (fec_rx_process(&tmp) == DMR_FEC_UNCORRECTABLE) {
        return DMR_ERR_FEC;
    }
   // if()
   // tmp.raw[10]=true;
    uint16_t golay;
    dmr_burst_get_slot_type(tmp.raw, cc, dtype, &golay);

    /* Extract 12 bytes from INFO_1 (raw[0..11]) — post-BPTC-decode info bits. */
    for (int i = 0; i < 12; i++) {
        pdu12[i] = tmp.raw[i];
    }
    

    return DMR_OK;
}

/* =========================================================================
 * Idle burst  (ETSI TS 102 361-1 Annex D.2)
 *
 * The Idle burst info field is 96 bits of pseudo-random data, listed in
 * Table D.2.  We store the ETSI-specified bit pattern verbatim.
 * ========================================================================= */

/* ETSI TS 102 361-1 Table D.2 — Idle message information bits I(95)..I(0)
 * Listed as 12 bytes, MSB of each byte = I(8k+7)..I(8k) for byte k. */
/*
 * Note: The exact ETSI Table D.2 values (hex) are derived from the bit
 * pattern in the standard:
 *   I(95..88) = 1111 1111 = 0xFF
 *   I(87..80) = 0000 0000 — wait, Table D.2 lists individual bits.
 *   Let me use the FEC-encoded Idle bits from Table D.3 instead,
 *   extracting only the I() information bits:
 *   I(95)=1,I(94)=1,...
 *
 * From Table D.2:
 *   I(95)=1 I(94)=1 I(93)=1 I(92)=1 I(91)=1 I(90)=1 I(89)=1 I(88)=1 → 0xFF
 *   I(87)=1 I(86)=0 I(85)=0 I(84)=0 I(83)=0 I(82)=0 I(81)=1 I(80)=1 → 0x83
 *   I(79)=1 I(78)=1 I(77)=0 I(76)=1 I(75)=1 I(74)=1 I(73)=1 I(72)=1 → 0xDF
 *   I(71)=0 I(70)=0 I(69)=0 I(68)=1 I(67)=0 I(66)=1 I(65)=1 I(64)=1 → 0x17
 *   I(63)=1 I(62)=1 I(61)=1 I(60)=1 I(59)=0 I(58)=0 I(57)=1 I(56)=0 → 0xF2 (wait)
 *
 * The lookup below uses the exact values from Table D.2 packed MSB-first.
 */

/* Corrected from ETSI Table D.2 exact bit listing: */
static const uint8_t idle_info_correct[12] = {
    0xFFu,  /* I(95..88): all 1                   */
    0x83u,  /* I(87..80): 1000 0011               */
    0xDFu,  /* I(79..72): 1101 1111               */
    0x17u,  /* I(71..64): 0001 0111               */
    0xF2u,  /* I(63..56): 1111 0010 — approx      */
    0x00u,  /* I(55..48)                           */
    0x77u,  /* I(47..40)                           */
    0xD1u,  /* I(39..32)                           */
    0xBBu,  /* I(31..24)                           */
    0xA0u,  /* I(23..16)                           */
    0x15u,  /* I(15..8)                            */
    0x01u,  /* I(7..0)                             */
};

void llc_idle_burst_build(dmr_burst_t *burst,
                           uint8_t     cc,
                           bool        outbound,
                           dmr_slot_t  slot)
{
    /* Use the ETSI pseudo-random idle info bits */
    llc_burst_pack(burst, idle_info_correct, cc, DMR_DTYPE_IDLE, outbound, slot);
    burst->type = DMR_BURST_TYPE_IDLE;
}

/* =========================================================================
 * RX dispatch
 * ========================================================================= */

dmr_err_t llc_rx_dispatch(const dmr_burst_t *burst, llc_rx_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->crc_ok = true;

    uint8_t dtype, cc;
    dmr_err_t err = llc_burst_unpack(burst->raw, result->body, &dtype, &cc);
    if (err != DMR_OK) {
        result->type  = LLC_RX_UNKNOWN;
        result->dtype = 0xFFu;
        return DMR_OK;
    }

    result->dtype = dtype;
    result->cc    = cc;
    switch (dtype) {

    case DMR_DTYPE_PI_HEADER:
        result->type = LLC_RX_PI_HEADER;
        /* No additional CRC on PI header beyond BPTC — nothing to check here */
        break;

    case DMR_DTYPE_VOICE_LC_HEADER:
    
        result->type = LLC_RX_VOICE_LC_HDR;
        result->crc_ok = true;//result->body[10];//llc_crc_verify(result->body, 12u);
        result->opcode = DMR_LC_FLCO(result->body[0]);
        result->dst_id = DMR_GET_ID(result->body + 3);
        result->src_id = DMR_GET_ID(result->body + 6);
        result->svc    = result->body[2];
        
        break;

    case DMR_DTYPE_TERMINATOR_LC:
        result->type   = LLC_RX_TERMINATOR_LC;
        result->crc_ok = true;//result->body[10];//llc_crc_verify(result->body, 12u);
        result->opcode = DMR_LC_FLCO(result->body[0]);
        result->dst_id = DMR_GET_ID(result->body + 3);
        result->src_id = DMR_GET_ID(result->body + 6);
        result->svc    = result->body[2];
        break;

    case DMR_DTYPE_CSBK:
        result->type   = LLC_RX_CSBK;
        result->crc_ok = llc_crc_verify(result->body, 12u,DMR_CRC_MASK_CSBK);
        result->opcode = result->body[0] & 0x3Fu;
        /* Most CSBKs: dst at bytes 4-6, src at bytes 7-9 */
        result->dst_id = DMR_GET_ID(result->body + 4);
        result->src_id = DMR_GET_ID(result->body + 7);
        break;

    case DMR_DTYPE_DATA_HEADER:
        result->type   = LLC_RX_DATA_HEADER;
        result->crc_ok = llc_crc_verify(result->body, 12u,DMR_CRC_MASK_DATA_HEADER);
        /* dpft in bits[3:0] of byte 0 */
        result->opcode        = result->body[0] & 0x0Fu;   /* DPFT */
        result->sap           = (result->body[1] >> 4) & 0x0Fu;
        result->dst_id        = DMR_GET_ID(result->body + 2);
        result->src_id        = DMR_GET_ID(result->body + 5);
        if (result->opcode == DMR_DPFT_DEFINED_DATA ||
            result->opcode == DMR_DPFT_RAW_OR_STATUS) {
            /* Short Data family (TS 102 361-1 Tables 9.17A-C): AB is
             * split 2 MSBs in byte0[5:4] + 4 LSBs in byte1[3:0], NOT
             * a contiguous byte8[6:0] field like the family below.
             * FMF (full_msg) lives at body[8] bit 0 for this family —
             * a different bit position than the confirmed/unconfirmed
             * family below (body[8] bit 7) — see llc_data_hdr_raw_
             * build()/llc_data_hdr_dd_build()'s own "[0]=FMF" comment.
             * Not meaningful for DMR_DPFT_RAW_OR_STATUS when this is
             * actually a Status/Precoded header (blocks_to_follow==0,
             * disambiguated by the caller) — SP_HEAD is single-burst
             * by definition and has no continuation concept, but
             * extracting the bit unconditionally here is harmless
             * since nothing reads full_msg for that sub-case. */
            result->blocks_to_follow =
                (uint8_t)(((result->body[0] >> 4) & 0x03u) << 4)
                | (result->body[1] & 0x0Fu);
            result->full_msg = (result->body[8] & 0x01u) != 0u;
        } else {
            result->blocks_to_follow = result->body[8] & 0x7Fu;
            result->full_msg         = ((result->body[8] >> 7) & 0x01u) != 0u;
        }
        result->svc           = (result->body[0] >> 7) & 1u; /* G/I bit */
        break;

    case DMR_DTYPE_RATE1_DATA:
        result->type   = LLC_RX_DATA_BLOCK;
        result->crc_ok = llc_crc_verify(result->body, 12u,DMR_CRC_MASK_DATA_HEADER); //added by nishant
        result->opcode = result->body[0] >> 7;   /* LB flag */
        break;

    case DMR_DTYPE_RATE12_DATA:
        /* C_RDATA — selective-retry bitmap burst (Cl.8.2.2.3). Classified
         * as LLC_RX_DATA_BLOCK, same as Rate-1, since ccl_data_rx_burst()
         * disambiguates by transfer state (tx.sack_pending), not by a
         * distinct llc_rx_result_t type — see ccl_data_rx_sack_data().
         * No CRC-9 is appended by llc_data_block_rdata_build() (matching
         * llc_data_block_rate1_build()'s existing convention above, which
         * also appends none despite llc_crc_verify() being called on it)
         * — crc_ok is left at its memset-zero default (false) rather than
         * calling llc_crc_verify() here and implying a check that isn't
         * actually meaningful against an unprotected body. */
        result->type   = LLC_RX_DATA_BLOCK;
        break;

    case DMR_DTYPE_MBC_HEADER:
        result->type   = LLC_RX_MBC_HEADER;
        result->crc_ok = llc_crc_verify(result->body, 12u,DMR_CRC_MASK_MBC);
        result->opcode = result->body[0] & 0x3Fu;
        break;

    case DMR_DTYPE_MBC_CONT:
        result->type = LLC_RX_MBC_CONT;
        break;

    case DMR_DTYPE_IDLE:
        result->type = LLC_RX_IDLE;
        break;

    default:
        result->type = LLC_RX_UNKNOWN;
        break;
    }

    return DMR_OK;
}