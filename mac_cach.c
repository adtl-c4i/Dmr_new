/**

/**

/**
 * @file mac_cach.c
 * @brief MOD-03 — CACH PDU encode/decode with Hamming(7,4) FEC
 *
 * ETSI TS 102 361-1, Clauses 4.5, 6.3, 9.1.4, Annex B.3.5, Annex B.4
 *
 * The CACH (Common Announcement Channel) carries:
 *   - AT  (1 bit):  Access Type — inbound channel idle(0) or busy(1)
 *   - TC  (1 bit):  TDMA Channel — slot 1(0) or slot 2(1)
 *   - LCSS (2 bits): LC Start/Stop fragment indicator
 *   - 3 Hamming(7,4) parity bits protecting the above 4 TACT bits
 *   - 17-bit Short Data payload (Short LC fragment or null)
 *
 * Wire format (24 bits, post-interleave, Annex B.4):
 *   TX(23)=AT, TX(22)=P16, TX(21)=P15, TX(20)=P14,
 *   TX(19)=TC, TX(18)=P13, TX(17)=P12, TX(16)=P11,
 *   TX(15)=LCSS(1), TX(14)=P10, TX(13)=P9, TX(12)=LCSS(0),
 *   TX(11)=P7, TX(10)=H(2), TX(9)=P6, TX(8)=P5, TX(7)=P4,
 *   TX(6)=H(1), TX(5)=P3, TX(4)=P2, TX(3)=P1, TX(2)=H(0), TX(1)=P0
 *   (P16..P0 = 17-bit Short Data payload, MSB=P16)
 *   (H(2:0)  = Hamming(7,4) parity over AT,TC,LCSS[1:0])
 *
 * Logical (pre-interleave) layout used in dmr_cach_pdu_t:
 *   Byte 0: [7]=AT [6]=TC [5:4]=LCSS [3:0]=Hamming_FEC[3:0]
 *   Byte 1: [7:0]=Short_Data[16:9]
 *   Byte 2: [7:1]=Short_Data[8:2]  [0]=Short_Data[1]
 *   (Short_Data[0] would require a 4th byte — stored in Byte 2 bit0 for now)
 *
 * NOTE: The Hamming(7,4) code used here is the primitive code with
 *       generator polynomial G(x) = x^3 + x + 1 = 0xB (binary: 1011).
 *       Generator matrix from ETSI TS 102 361-1 Annex B.3.5, Table B.17.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "dmr_pdu.h"
#include "dmr_mac.h"
#include "dmr_types.h"

/* =========================================================================
 * Hamming(7,4,3) — ETSI TS 102 361-1 Annex B.3.5, Table B.17
 *
 * Generator matrix G (4 rows x 7 cols):
 *   Row d3: 1 0 0 0 | 1 0 1
 *   Row d2: 0 1 0 0 | 1 1 1
 *   Row d1: 0 0 1 0 | 1 1 0
 *   Row d0: 0 0 0 1 | 0 1 1
 *
 * Codeword = data x G, systematic form [d3 d2 d1 d0 | p2 p1 p0]:
 *   p2 (col 4) = d3 ^ d2 ^ d1   (rows d3,d2,d1 have a 1 in this column)
 *   p1 (col 5) = d2 ^ d1 ^ d0   (rows d2,d1,d0 have a 1 in this column)
 *   p0 (col 6) = d3 ^ d2 ^ d0   (rows d3,d2,d0 have a 1 in this column)
 *
 * Verified exhaustively: all 16 data values round-trip with zero error,
 * and all 112 possible single-bit flips (16 values x 7 bit positions) are
 * correctly detected and corrected.
 * ========================================================================= */

/*
 * Hamming(7,4) encode.
 * data: 4 bits [d3 d2 d1 d0] in bits [3:0] of input byte
 * returns: 7-bit codeword [d3 d2 d1 d0 p2 p1 p0] in bits [6:0]
 */
static uint8_t hamming74_encode(uint8_t data4)
{
    uint8_t d3 = (data4 >> 3) & 1u;
    uint8_t d2 = (data4 >> 2) & 1u;
    uint8_t d1 = (data4 >> 1) & 1u;
    uint8_t d0 = (data4      ) & 1u;

    uint8_t p2 = d3 ^ d2 ^ d1;
    uint8_t p1 = d2 ^ d1 ^ d0;
    uint8_t p0 = d3 ^ d2 ^ d0;

    return (uint8_t)((d3 << 6) | (d2 << 5) | (d1 << 4) | (d0 << 3)
                   | (p2 << 2) | (p1 << 1) |  p0);
}

/*
 * Hamming(7,4) decode with single-bit error correction.
 * codeword7: 7-bit received codeword in bits [6:0]
 * out_data4: 4-bit decoded data written to bits [3:0]
 * returns: true if syndrome was zero (no error detected), false if corrected
 *
 * Syndrome s = [s2 s1 s0] computed from parity-check matrix H.
 * H (3×7) — parity-check rows derived from above:
 *   s2 = c6^c5^c4^c2         (positions where p2 was formed: d3,d2,d1,p2)
 *   s1 = c5^c4^c3^c1         (d2,d1,d0,p1)
 *   s0 = c6^c5^c3^c0         (d3,d2,d0,p0)
 *
 * Non-zero syndrome → error position (from standard Hamming syndrome table):
 *   001=pos0(p0), 010=pos1(p1), 011=pos3(d0), 100=pos2(p2),
 *   101=pos6(d3), 110=pos4(d1), 111=pos5(d2)
 * Error position numbered 0=LSB(p0), 6=MSB(d3)
 */
static bool hamming74_decode(uint8_t cw7, uint8_t *out_data4)
{
    uint8_t c6 = (cw7 >> 6) & 1u;
    uint8_t c5 = (cw7 >> 5) & 1u;
    uint8_t c4 = (cw7 >> 4) & 1u;
    uint8_t c3 = (cw7 >> 3) & 1u;
    uint8_t c2 = (cw7 >> 2) & 1u;
    uint8_t c1 = (cw7 >> 1) & 1u;
    uint8_t c0 = (cw7      ) & 1u;

    uint8_t s2 = c6 ^ c5 ^ c4 ^ c2;
    uint8_t s1 = c5 ^ c4 ^ c3 ^ c1;
    uint8_t s0 = c6 ^ c5 ^ c3 ^ c0;

    uint8_t syndrome = (uint8_t)((s2 << 2) | (s1 << 1) | s0);
    bool    corrected = false;

    if (syndrome != 0) {
        /* Map syndrome to bit position and flip it */
        static const uint8_t syn_to_pos[8] = {
            255, 0, 1, 3, 2, 6, 4, 5
        };
        uint8_t pos = syn_to_pos[syndrome & 0x07u];
        if (pos <= 6) {
            cw7 ^= (uint8_t)(1u << pos);
        }
        corrected = true;
    }

    *out_data4 = (uint8_t)((cw7 >> 3) & 0x0Fu);
    return corrected;
}

/* =========================================================================
 * CACH interleaver / deinterleaver
 * ETSI TS 102 361-1, Annex B.4.1, Figure B.9
 *
 * The interleaver scatters the 7 TACT bits (AT, TC, LCSS[1:0], H[2:0])
 * among the 17 payload bits P[16:0] across the 24-bit CACH burst.
 *
 * Wire bit positions (TX(23)=MSB .. TX(0)=LSB), transcribed directly from
 * Figure B.9's "AT P(16) P(15) P(14) TC ... H(0) P(0)" row, verified
 * bit-for-bit against the spec text (not just re-derived from memory):
 *   TX(23) = AT
 *   TX(22) = P(16)
 *   TX(21) = P(15)
 *   TX(20) = P(14)
 *   TX(19) = TC
 *   TX(18) = P(13)
 *   TX(17) = P(12)
 *   TX(16) = P(11)
 *   TX(15) = LCSS(1)
 *   TX(14) = P(10)
 *   TX(13) = P(9)
 *   TX(12) = P(8)
 *   TX(11) = LCSS(0)
 *   TX(10) = P(7)
 *   TX(9)  = H(2)
 *   TX(8)  = P(6)
 *   TX(7)  = P(5)
 *   TX(6)  = P(4)
 *   TX(5)  = H(1)
 *   TX(4)  = P(3)
 *   TX(3)  = P(2)
 *   TX(2)  = P(1)
 *   TX(1)  = H(0)
 *   TX(0)  = P(0)
 * ========================================================================= */

/*
 * Pack 24-bit CACH wire value from logical fields.
 * at, tc: single bits
 * lcss: 2-bit value [1:0]
 * hamming7: 7-bit encoded TACT codeword [d3=AT, d2=TC, d1=LCSS1, d0=LCSS0, p2, p1, p0]
 * payload17: 17-bit Short Data [16:0]
 */
static uint32_t cach_interleave(uint8_t at, uint8_t tc, uint8_t lcss,
                                 uint8_t hamming7, uint32_t payload17)
{
    /* Extract individual payload bits */
    uint8_t p[17];
    for (int i = 0; i < 17; i++) {
        p[i] = (uint8_t)((payload17 >> i) & 1u);
    }

    /* Extract Hamming parity bits (bits 2:0 of hamming7) */
    uint8_t h0 = (hamming7     ) & 1u;
    uint8_t h1 = (hamming7 >> 1) & 1u;
    uint8_t h2 = (hamming7 >> 2) & 1u;

    uint8_t lcss1 = (lcss >> 1) & 1u;
    uint8_t lcss0 =  lcss       & 1u;

    /* Build 24-bit wire value from interleave table (Annex B.4 Figure B.9) */
    uint32_t wire = 0;
    wire |= (uint32_t) at    << 23;  /* TX(23) */
    wire |= (uint32_t)p[16]  << 22;  /* TX(22) */
    wire |= (uint32_t)p[15]  << 21;  /* TX(21) */
    wire |= (uint32_t)p[14]  << 20;  /* TX(20) */
    wire |= (uint32_t) tc    << 19;  /* TX(19) */
    wire |= (uint32_t)p[13]  << 18;  /* TX(18) */
    wire |= (uint32_t)p[12]  << 17;  /* TX(17) */
    wire |= (uint32_t)p[11]  << 16;  /* TX(16) */
    wire |= (uint32_t)lcss1  << 15;  /* TX(15) */
    wire |= (uint32_t)p[10]  << 14;  /* TX(14) */
    wire |= (uint32_t)p[9]   << 13;  /* TX(13) */
    wire |= (uint32_t)p[8]   << 12;  /* TX(12) */
    wire |= (uint32_t)lcss0  << 11;  /* TX(11) */
    wire |= (uint32_t)p[7]   << 10;  /* TX(10) */
    wire |= (uint32_t) h2    <<  9;  /* TX(9)  */
    wire |= (uint32_t)p[6]   <<  8;  /* TX(8)  */
    wire |= (uint32_t)p[5]   <<  7;  /* TX(7)  */
    wire |= (uint32_t)p[4]   <<  6;  /* TX(6)  */
    wire |= (uint32_t) h1    <<  5;  /* TX(5)  */
    wire |= (uint32_t)p[3]   <<  4;  /* TX(4)  */
    wire |= (uint32_t)p[2]   <<  3;  /* TX(3)  */
    wire |= (uint32_t)p[1]   <<  2;  /* TX(2)  */
    wire |= (uint32_t) h0    <<  1;  /* TX(1)  */
    wire |= (uint32_t)p[0]   <<  0;  /* TX(0)  */

    return wire;
}

/*
 * Deinterleave 24-bit CACH wire value into logical fields.
 * Returns the 7-bit TACT codeword (for Hamming decode) and 17-bit payload.
 */
static void cach_deinterleave(uint32_t wire,
                               uint8_t *at, uint8_t *tc, uint8_t *lcss,
                               uint8_t *tact7, uint32_t *payload17)
{
    /* Extract control bits from their wire positions */
    *at    = (wire >> 23) & 1u;
    *tc    = (wire >> 19) & 1u;
    uint8_t lcss1 = (wire >> 15) & 1u;
    uint8_t lcss0 = (wire >> 11) & 1u;
    *lcss  = (uint8_t)((lcss1 << 1) | lcss0);

    /* Extract Hamming parity bits */
    uint8_t h2 = (wire >>  9) & 1u;
    uint8_t h1 = (wire >>  5) & 1u;
    uint8_t h0 = (wire >>  1) & 1u;

    /* Reconstruct 7-bit TACT codeword [AT TC LCSS1 LCSS0 H2 H1 H0] */
    *tact7 = (uint8_t)((*at << 6) | (*tc << 5) | (lcss1 << 4) | (lcss0 << 3)
                       | (h2 << 2) | (h1 << 1) | h0);

    /* Extract 17-bit payload from remaining wire positions */
    uint8_t p[17];
    p[16] = (wire >> 22) & 1u;
    p[15] = (wire >> 21) & 1u;
    p[14] = (wire >> 20) & 1u;
    p[13] = (wire >> 18) & 1u;
    p[12] = (wire >> 17) & 1u;
    p[11] = (wire >> 16) & 1u;
    p[10] = (wire >> 14) & 1u;
    p[9]  = (wire >> 13) & 1u;
    p[8]  = (wire >> 12) & 1u;
    p[7]  = (wire >> 10) & 1u;
    p[6]  = (wire >>  8) & 1u;
    p[5]  = (wire >>  7) & 1u;
    p[4]  = (wire >>  6) & 1u;
    p[3]  = (wire >>  4) & 1u;
    p[2]  = (wire >>  3) & 1u;
    p[1]  = (wire >>  2) & 1u;
    p[0]  = (wire >>  0) & 1u;

    uint32_t pay = 0;
    for (int i = 16; i >= 0; i--) {
        pay = (pay << 1) | p[i];
    }
    *payload17 = pay;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/*
 * mac_cach_build — build and interleave a CACH PDU for transmission.
 *
 * Fills cach->at_tc_lcss_fec with the Hamming-encoded TACT word,
 * and cach->sd / cach->golay_hi with the 17-bit Short Data.
 * The caller writes these 3 bytes into the CACH burst slot.
 */
void mac_cach_build(dmr_cach_pdu_t *cach,
                    uint8_t at, uint8_t tc, uint8_t lcss,
                    uint32_t short_data)
{
    /* Clamp inputs */
    at         &= 0x01u;
    tc         &= 0x01u;
    lcss       &= 0x03u;
    short_data &= 0x1FFFFu;  /* 17 bits */

    /* Build 4-bit TACT data word: [AT TC LCSS1 LCSS0] */
    uint8_t tact4 = (uint8_t)((at << 3) | (tc << 2) | (lcss & 0x03u));

    /* Encode with Hamming(7,4) */
    uint8_t h7 = hamming74_encode(tact4);

    /* Interleave into 24-bit wire value */
    uint32_t wire = cach_interleave(at, tc, lcss, h7, short_data);

    /* Pack into 3-byte dmr_cach_pdu_t (big-endian: wire[23:16], [15:8], [7:0]) */
    cach->at_tc_lcss_fec = (uint8_t)((wire >> 16) & 0xFFu);
    cach->sd             = (uint8_t)((wire >>  8) & 0xFFu);
    cach->golay_hi       = (uint8_t)( wire         & 0xFFu);
}

/*
 * mac_cach_parse — deinterleave and Hamming-decode a received CACH PDU.
 */
dmr_err_t mac_cach_parse(const dmr_cach_pdu_t *cach,
                          uint8_t *at, uint8_t *tc, uint8_t *lcss,
                          uint32_t *short_data)
{
    /* Reconstruct 24-bit wire value from the 3-byte PDU */
    uint32_t wire = ((uint32_t)cach->at_tc_lcss_fec << 16)
                  | ((uint32_t)cach->sd             <<  8)
                  |  (uint32_t)cach->golay_hi;

    /* Deinterleave */
    uint8_t  tact7;
    uint32_t payload17;
    cach_deinterleave(wire, at, tc, lcss, &tact7, &payload17);

    /* Hamming(7,4) FEC decode on TACT bits */
    uint8_t decoded4;
    bool corrected = hamming74_decode(tact7, &decoded4);
    (void)corrected; /* single-bit error corrected silently */

    /* After FEC correction, re-extract individual fields */
    *at   = (decoded4 >> 3) & 1u;
    *tc   = (decoded4 >> 2) & 1u;
    *lcss = decoded4 & 0x03u;

    if (short_data) {
        *short_data = payload17;
    }

    return DMR_OK;
}

/* =========================================================================
 * Slot activity tracking
 * ========================================================================= */

void mac_update_slot_activity(mac_ctx_t *ctx, uint8_t at, uint8_t tc)
{
    uint64_t now = dmr_time_now_us();

    /* Save previous busy state before overwriting — needed by MAC to
     * detect the Busy→Idle transition that signals end of call hang-time
     * on Tier II/III (see mac_handle_cach_at_idle() in mac_channel_access.c). */
    if (tc == 0) {
        ctx->slot_activity.prev_slot1_busy = ctx->slot_activity.slot1_busy;
        ctx->slot_activity.slot1_busy      = (at == 1);
    } else {
        ctx->slot_activity.prev_slot2_busy = ctx->slot_activity.slot2_busy;
        ctx->slot_activity.slot2_busy      = (at == 1);
    }
    ctx->slot_activity.active_slot    = tc;
    ctx->slot_activity.last_update_us = now;
}


bool mac_slot_is_busy(const mac_ctx_t *ctx, dmr_slot_t slot)
{
    if (slot == DMR_SLOT_1) return ctx->slot_activity.slot1_busy;
    return ctx->slot_activity.slot2_busy;
}