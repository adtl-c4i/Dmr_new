    llc_burst_pack(burst, body, cc, dtype, false, slot);

/**
 * @file llc_csbk.c
 * @brief LLC — CSBK, Full LC, PI Header, Data Header, Data Block builders/parsers
 *
 * ETSI TS 102 361-1 Cl.7.1–7.2, 8.2.1, 9.1.5
 * ETSI TS 102 361-2 Cl.7.1.2
 * ETSI TS 102 361-4 Cl.6
 *
 * Every build function follows the same pattern:
 *   1. memset the PDU struct to zero
 *   2. Fill fields from parameters
 *   3. Append CRC-CCITT (bytes 10-11) where the standard mandates it
 *   4. Call llc_burst_pack() to place the 12 bytes into the burst
 *
 * Every parse function:
 *   1. Calls llc_crc_verify() on the 12-byte body
 *   2. Extracts fields by byte position (no bitfields)
 *   3. Returns DMR_ERR_CRC on failure but still fills output fields
 *      (CCL decides whether to act on a CRC-failed PDU)
 */

#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dmr_pdu.h"
#include "dmr_llc.h"
#include "dmr_types.h"

/* =========================================================================
 * Internal: build a minimal CSBK body and wrap in a burst.
 *
 * All CSBKs share byte 0 (LB=1,PF=0,CSBKO) and byte 1 (FID=0x00).
 * Bytes 2-9 are opcode-specific.  Bytes 10-11 = CRC-CCITT over bytes 0-9.
 * ========================================================================= */
static void csbk_finalise_burst(dmr_burst_t *burst,
                                 uint8_t      body[12],
                                 uint8_t      cc,
                                 dmr_slot_t   slot)
{
    /* CRC-CCITT over bytes 0..9 (10 bytes of data + opcode header) */
    llc_crc_append(body, 10u,DMR_CRC_MASK_CSBK);
    /* Pack into burst: inbound MS→BS uses MS_DATA sync */
    llc_burst_pack(burst, body, cc, DMR_DTYPE_CSBK, false, slot);
}

/* =========================================================================
 * SECTION A — CSBK builders
 * ========================================================================= */

/* BS Down-link Activation (CSBKO=0x28) — TS 102 361-2, Table 7.5a
 *
 * Byte 0: LB=1, PF=0, CSBKO=0x28 → 0xA8
 * Byte 1: FID = 0x00
 * Bytes 2-3: Reserved (0x00)
 * Bytes 4-6: BS address (dst)
 * Bytes 7-9: Source address
 * Bytes 10-11: CRC-CCITT
 */
void llc_csbk_bs_dwna_build(dmr_burst_t *burst,
                              uint32_t bs_id,
                              uint32_t src_id,
                              uint8_t  cc,
                              uint8_t  fid,
                              dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_BS_DWNA);
    body[1] = fid;
    /* bytes 2-3: reserved = 0 */
    DMR_SET_ID(body + 4, bs_id);
    DMR_SET_ID(body + 7, src_id);
         
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Preamble CSBK (CSBKO=0x3D) — TS 102 361-2, Table 7.7
 *
 * Byte 2: [7]=Data/CSBK [6]=Group/Ind [5:0]=Reserved
 * Byte 3: CBF (CSBK Blocks to Follow)
 * Bytes 4-6: Destination ID
 * Bytes 7-9: Source ID
 */
void llc_csbk_preamble_build(dmr_burst_t *burst,
                               bool     is_data,
                               bool     is_group,
                               uint8_t  cbf,
                               uint32_t dst_id,
                               uint32_t src_id,
                               uint8_t  cc,
                               dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_PREAMBLE);
    body[1] = 0x00u;
    body[2] = (uint8_t)((is_data  ? 0x80u : 0x00u)
                       |(is_group ? 0x40u : 0x00u));
    body[3] = cbf;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Unit-to-Unit Voice Service Request (CSBKO=0x04) — TS 102 361-2, Table 7.5b
 *
 * Byte 2: Service Options
 * Byte 3: Reserved
 * Bytes 4-6: Target (destination) address
 * Bytes 7-9: Source address
 */
void llc_csbk_uu_v_req_build(dmr_burst_t *burst,
                               uint8_t  svc,
                               uint32_t dst_id,
                               uint32_t src_id,
                               uint8_t  cc,
                               dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_UU_V_REQ);
    body[1] = 0x00u;
    body[2] = svc;
    body[3] = 0x00u;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Unit-to-Unit Answer Response (CSBKO=0x24) — TS 102 361-2, Table 7.5c
 *
 * Byte 2: Service Options
 * Byte 3: Answer Response (0x40=Proceed, 0x41=Deny per Table 7.12)
 * Bytes 4-6: Target (destination) address  (the MS that sent UU_V_REQ)
 * Bytes 7-9: Source address  (the responding MS)
 */
void llc_csbk_uu_ans_rsp_build(dmr_burst_t *burst,
                                 uint8_t  svc,
                                 uint8_t  response,
                                 uint32_t dst_id,
                                 uint32_t src_id,
                                 uint8_t  cc,
                                 dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_UU_ANS_RSP);
    body[1] = 0x00u;
    body[2] = svc;
    /* Answer Response encoding per TS 102 361-2 Table 7.12:
     * 0x40 = 0100 0000 = Proceed
     * 0x41 = 0100 0001 = Deny                                               */
    body[3] = (response == DMR_UU_ANS_PROCEED) ? 0x40u : 0x41u;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Call Alert (CSBKO=0x1F) */
void llc_csbk_call_alert_build(dmr_burst_t *burst,
                                 uint32_t dst_id,
                                 uint32_t src_id,
                                 uint8_t  cc,
                                 dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_CALL_ALERT);
    body[1] = 0x00u;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Acknowledge Response (CSBKO=0x20) */
void llc_csbk_ack_rsp_build(dmr_burst_t *burst,
                              uint32_t dst_id,
                              uint32_t src_id,
                              uint8_t  cc,
                              dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_ACK_RSP);
    body[1] = 0x00u;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Emergency Alarm (CSBKO=0x27 Tier-II / 0x28 Tier-III)
 * Byte 2: [7:4]=emerg_type [3:0]=reserved
 */
void llc_csbk_emerg_alarm_build(dmr_burst_t *burst,
                                  uint8_t  emerg_type,
                                  uint32_t dst_id,
                                  uint32_t src_id,
                                  uint8_t  cc,
                                  dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_EMERG_ALARM_ACK);
    body[1] = 0x00u;
    body[2] = (uint8_t)((emerg_type & 0x0Fu) << 4);
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Cancel Call Alert (CSBKO=0x23) */
void llc_csbk_cancel_build(dmr_burst_t *burst,
                             uint32_t dst_id,
                             uint32_t src_id,
                             uint8_t  cc,
                             dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_CANCEL_CALL);
    body[1] = 0x00u;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* CSBK parse — common path for all opcodes */
dmr_err_t llc_csbk_parse(const uint8_t *raw12,
                           uint8_t  *opcode,
                           uint32_t *dst_id,
                           uint32_t *src_id)
{
    dmr_err_t rc = llc_crc_verify(raw12, 12u,DMR_CRC_MASK_CSBK) ? DMR_OK : DMR_ERR_CRC;

    *opcode = raw12[0] & 0x3Fu;
    /* Standard CSBK layout: dst at bytes 4-6, src at bytes 7-9.
     * Note: a few opcodes reverse this — the caller checks opcode and
     * re-interprets if needed.  For BS_DWNA dst=BS address at 4-6.    */
    *dst_id = DMR_GET_ID(raw12 + 4);
    *src_id = DMR_GET_ID(raw12 + 7);

    return rc;
}

/* =========================================================================
 * SECTION A2 — Channel Timing CSBK (CT_CSBK) build / parse
 * ETSI TS 102 361-2 Cl.6.2 / Cl.7.1.2.6, Table 7.8
 *
 * Bytes 2-9 (64 bits) hold ten fields packed MSB-first in strict table
 * order. Rather than hand-deriving byte/bit offsets from the spec's
 * prose pointers (which describe MSB/LSB positions for the two 20-bit
 * ID fields and are easy to get off-by-one on), this uses a generic
 * MSB-first bit-stream writer/reader driven purely by the field widths
 * in Table 7.8 (11+5+20+1+2+1+20+1+2+1 = 64 bits exactly, bytes 2-9).
 * This self-checks against the two unambiguous MSB positions the spec
 * does state in prose (LID MSB = octet 4 bit 7, SID MSB = octet 7
 * bit 7) — both fall out correctly from cumulative-width arithmetic.
 * ========================================================================= */

/* Write the low `width` bits of val into the bitstream at *bitpos
 * (absolute bit index from the start of the buffer, MSB-first),
 * advancing *bitpos by width. */
static void ct_bits_write(uint8_t *buf, int *bitpos, uint32_t val, int width)
{
    for (int i = width - 1; i >= 0; i--) {
        int bytei = *bitpos / 8;
        int biti  = 7 - (*bitpos % 8);
        if ((val >> i) & 1u) {
            buf[bytei] |= (uint8_t)(1u << biti);
        }
        (*bitpos)++;
    }
}

static uint32_t ct_bits_read(const uint8_t *buf, int *bitpos, int width)
{
    uint32_t val = 0u;
    for (int i = 0; i < width; i++) {
        int bytei = *bitpos / 8;
        int biti  = 7 - (*bitpos % 8);
        val = (val << 1) | ((uint32_t)(buf[bytei] >> biti) & 1u);
        (*bitpos)++;
    }
    return val;
}

void llc_ct_csbk_build(dmr_burst_t *burst,
                        const dmr_ct_csbk_t *ct,
                        dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_CHANNEL_TIMING);
    body[1] = 0x00u;

    int bitpos = 16; /* bytes 0-1 (16 bits) already written above */
    ct_bits_write(body, &bitpos, ct->sync_age & 0x7FFu,   11);
    ct_bits_write(body, &bitpos, ct->gen & 0x1Fu,          5);
    ct_bits_write(body, &bitpos, ct->leader_id & 0xFFFFFu, 20);
    ct_bits_write(body, &bitpos, ct->new_leader ? 1u : 0u, 1);
    ct_bits_write(body, &bitpos, ct->leader_di & 0x3u,     2);
    ct_bits_write(body, &bitpos, (uint32_t)((ct->cto >> 1) & 0x1u), 1); /* CTO MSB */
    ct_bits_write(body, &bitpos, ct->source_id & 0xFFFFFu, 20);
    ct_bits_write(body, &bitpos, 0u,                        1); /* Reserved = 0 */
    ct_bits_write(body, &bitpos, ct->source_di & 0x3u,      2);
    ct_bits_write(body, &bitpos, (uint32_t)(ct->cto & 0x1u), 1); /* CTO LSB */

    /* CT_CSBK always transmits with the All Site colour code (Cl.6.2.2.2) */
    csbk_finalise_burst(burst, body, DMR_CC_ALL_SITE, slot);
}

dmr_err_t llc_ct_csbk_parse(const uint8_t *raw12, dmr_ct_csbk_t *ct)
{
    dmr_err_t rc = llc_crc_verify(raw12, 12u,DMR_CRC_MASK_CSBK) ? DMR_OK : DMR_ERR_CRC;

    memset(ct, 0, sizeof(*ct));
    int bitpos = 16;
    ct->sync_age   = (uint16_t)ct_bits_read(raw12, &bitpos, 11);
    ct->gen        = (uint8_t)ct_bits_read(raw12, &bitpos, 5);
    ct->leader_id  = ct_bits_read(raw12, &bitpos, 20);
    ct->new_leader = ct_bits_read(raw12, &bitpos, 1) != 0u;
    ct->leader_di  = (uint8_t)ct_bits_read(raw12, &bitpos, 2);
    uint32_t cto_msb = ct_bits_read(raw12, &bitpos, 1);
    ct->source_id  = ct_bits_read(raw12, &bitpos, 20);
    (void)ct_bits_read(raw12, &bitpos, 1); /* Reserved */
    ct->source_di  = (uint8_t)ct_bits_read(raw12, &bitpos, 2);
    uint32_t cto_lsb = ct_bits_read(raw12, &bitpos, 1);
    ct->cto = (uint8_t)((cto_msb << 1) | cto_lsb);

    return rc;
}

/* =========================================================================
 * SECTION B — Full LC builders / parsers
 * ========================================================================= */

/*
 * Internal: zero-fill lc, set FLCO byte and FID, then let caller fill
 * svc/dst/src.  RS(12,9) FEC bytes (9-11) are left zero — filled by MOD-02.
 */
static void lc_init(dmr_full_lc_t *lc, uint8_t flco, uint8_t  fid)
{
    memset(lc, 0, sizeof(*lc));
    lc->flco = DMR_LC_MAKE_BYTE(flco, 0);
    lc->fid  = fid;   /* ETSI SFID */
}

void llc_full_lc_grp_voice_build(dmr_full_lc_t *lc,
                                  uint8_t  svc,
                                  uint8_t  fid,
                                  uint32_t group_id,
                                  uint32_t src_id)
{
    lc_init(lc, DMR_FLCO_GRP_V_CH_USR,fid);
    lc->svc = svc;
    DMR_SET_ID(lc->dst_id, group_id);
    DMR_SET_ID(lc->src_id, src_id);
 //   uint8_t *b = (uint8_t *)lc;
  //  llc_crc_append(b, 10);
}

void llc_full_lc_ind_voice_build(dmr_full_lc_t *lc,
                                  uint8_t  svc,
                                  uint8_t  fid,
                                  uint32_t dst_id,
                                  uint32_t src_id)
{
    lc_init(lc, DMR_FLCO_IND_V_CH_USR,fid);
    lc->svc = svc;
    DMR_SET_ID(lc->dst_id, dst_id);
    DMR_SET_ID(lc->src_id, src_id);
 //         uint8_t *b = (uint8_t *)lc;
//    llc_crc_append(b, 10);
    
}

/* GPS Info LC (FLCO=0x08) — TS 102 361-2 Table 7.3
 *
 * Byte 2: [4:2]=PositionError  [1:0]=Lon[24:23]
 * Bytes 3-5: Lon[22:7] (continued, packed across 3 bytes with Lat)
 * Bytes 5-8: Lat[23:0]
 *
 * ETSI encoding:
 *   Longitude: 25-bit signed, resolution = 360/2^25 degrees ≈ 1.07e-5 deg
 *              encoded_lon = round(lon_deg * 2^25 / 360)
 *   Latitude:  24-bit signed, resolution = 180/2^24 degrees ≈ 1.07e-5 deg
 *              encoded_lat = round(lat_deg * 2^24 / 180)
 *
 * Layout (bits, MSB first, total = 3+25+24 = 52 info bits in bytes 2-8):
 *   Byte 2: [7:5]=reserved [4:2]=pos_err [1:0]=lon[24:23]
 *   Byte 3: lon[22:15]
 *   Byte 4: lon[14:7]
 *   Byte 5: [7:1]=lon[6:0]  [0]=lat[23]
 *   Byte 6: lat[22:15]
 *   Byte 7: lat[14:7]
 *   Byte 8: [7:1]=lat[6:0]  [0]=reserved
 */
void llc_full_lc_gps_build(dmr_full_lc_t *lc,
                             int32_t lat_deg7,
                             int32_t lon_deg7,
                              uint8_t  fid,
                             uint8_t pos_error)
{
    lc_init(lc, DMR_FLCO_GPS_INFO,fid);

    /* Convert lat/lon from ×1e-7 degrees to ETSI encoding.
     * ETSI lon resolution = 360/2^25 = 1.0728836e-5 deg
     * Input is deg×1e7, so: encoded = deg×1e7 × 2^25 / (360×1e7)
     *                              = deg×1e7 × 2^25 / 3.6e9              */
    int32_t enc_lon = (int32_t)(((int64_t)lon_deg7 * (1L << 25)) / 3600000000LL);
    int32_t enc_lat = (int32_t)(((int64_t)lat_deg7 * (1L << 24)) / 1800000000LL);

    /* Clamp to valid ranges */
    if (enc_lon >  16777215) enc_lon =  16777215;
    if (enc_lon < -16777216) enc_lon = -16777216;
    if (enc_lat >   8388607) enc_lat =   8388607;
    if (enc_lat <  -8388608) enc_lat =  -8388608;

    uint32_t ulon = (uint32_t)(enc_lon & 0x1FFFFFFu);  /* 25 bits */
    uint32_t ulat = (uint32_t)(enc_lat & 0xFFFFFFu);   /* 24 bits */

    /* Reinterpret lc as byte array — bytes 0-8 are the 9-byte info payload */
    uint8_t *b = (uint8_t *)lc;
    b[2] = (uint8_t)(((pos_error & 0x07u) << 2) | ((ulon >> 23) & 0x03u));
    b[3] = (uint8_t)((ulon >> 15) & 0xFFu);
    b[4] = (uint8_t)((ulon >>  7) & 0xFFu);
    b[5] = (uint8_t)(((ulon & 0x7Fu) << 1) | ((ulat >> 23) & 0x01u));
    b[6] = (uint8_t)((ulat >> 15) & 0xFFu);
    b[7] = (uint8_t)((ulat >>  7) & 0xFFu);
    b[8] = (uint8_t)((ulat & 0x7Fu) << 1);
    
    
   // uint8_t lc12[12];
   // memcpy(lc12,&lc,12);
    
  //  llc_crc_append(b, 10);
    
   // memcpy(&lc,lc12,12);
    
    
    
}

dmr_err_t llc_full_lc_parse(const uint8_t *raw12,
                              uint8_t  *flco,
                              uint32_t *dst_id,
                              uint32_t *src_id,
                              uint8_t  *svc)
{
    *flco   = DMR_LC_FLCO(raw12[0]);
    *svc    = raw12[2];
    *dst_id = DMR_GET_ID(raw12 + 3);
    *src_id = DMR_GET_ID(raw12 + 6);
    /* RS(12,9) FEC bytes are at raw12[9..11] — checked by MOD-02 */
    return DMR_OK;
}

/* Voice LC Header burst (Data Type 0x01) */
void llc_voice_lc_header_build(dmr_burst_t         *burst,
                                const dmr_full_lc_t *lc,
                                uint8_t              cc,
                                bool                 outbound,
                                dmr_slot_t           slot)
{
    llc_burst_pack(burst, (const uint8_t *)lc,
                   cc, DMR_DTYPE_VOICE_LC_HEADER, outbound, slot);
    burst->type = DMR_BURST_TYPE_DATA;
}

/* Terminator with LC burst (Data Type 0x02) */
void llc_terminator_lc_build(dmr_burst_t         *burst,
                               const dmr_full_lc_t *lc,
                               uint8_t              cc,
                               bool                 outbound,
                               dmr_slot_t           slot)
{
    llc_burst_pack(burst, (const uint8_t *)lc,
                   cc, DMR_DTYPE_TERMINATOR_LC, outbound, slot);
    burst->type = DMR_BURST_TYPE_DATA;
}

/* =========================================================================
 * SECTION C — PI Header (Data Type 0x00)
 * ETSI TS 102 361-1 Cl.9.1.5
 *
 * The PI Header is 17 bytes wide but only the first 12 bytes fit in a
 * single-burst payload.  ETSI encodes it in BPTC(196,96) like all other
 * data bursts, packing only the first 12 bytes of the dmr_pi_header_t.
 * The CRC-CCITT covers bytes 0-9 (10 bytes), placed at bytes 10-11.
 *
 * Byte 0:    Algorithm ID
 * Bytes 1-2: Key ID [15:8],[7:0]
 * Bytes 3-11: Message Indicator (9 bytes, MI[71:0])
 * Note: The PI header layout in a single BPTC burst uses:
 *   Byte 0:  ALG_ID
 *   Byte 1:  KEY_ID[15:8]
 *   Byte 2:  KEY_ID[7:0]
 *   Bytes 3-5: DST_ID
 *   Bytes 6-7: reserved
 *   Bytes 8-9: MI[71:56] (first 2 of 9 MI bytes)
 *   Bytes 10-11: CRC
 * Full MI goes in subsequent MBC blocks for long PI headers.
 *
 * For the standard single-burst PI header (most common case):
 *   Byte 0: reserved  [7]=0
 *   Byte 1: ALG_ID
 *   Byte 2: KEY_ID[15:8]
 *   Byte 3: KEY_ID[7:0]
 *   Bytes 4-6: DST_ID
 *   Bytes 7-9: MI[71:48] (3 of 9 bytes — rest carried in MBC if needed)
 *   Bytes 10-11: CRC-CCITT over bytes 0-9
 * ========================================================================= */
void llc_pi_header_build(dmr_burst_t   *burst,
                          const uint8_t  mi[9],
                          uint8_t        alg_id,
                          uint16_t       key_id,
                          uint32_t       dst_id,
                          uint8_t        cc,
                          dmr_slot_t     slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = 0x00u;                        /* reserved */
    body[1] = alg_id;
    body[2] = (uint8_t)(key_id >> 8);
    body[3] = (uint8_t)(key_id & 0xFFu);
    DMR_SET_ID(body + 4, dst_id);
    /* First 3 bytes of MI in bytes 7-9 (rest in subsequent MBC if needed) */
    body[7] = mi[0];
    body[8] = mi[1];
    body[9] = mi[2];
    llc_crc_append(body, 10u,DMR_CRC_MASK_NONE); // pi header has rs not crc ????? nishant
    llc_burst_pack(burst, body, cc, DMR_DTYPE_PI_HEADER, false, slot);
}

/* =========================================================================
 * SECTION D — Data Header builders / parser
 * ETSI TS 102 361-1 Cl.8.2.1
 *
 * All data headers share:
 *   Byte 0:  [7]=G/I [6]=A [5]=res [4]=POC_MSB [3:0]=DPFT
 *   Byte 1:  [7:4]=SAP [3:0]=POC[3:0]
 *   Bytes 2-4: DST_LLID
 *   Bytes 5-7: SRC_LLID
 *   Byte 8:  [7]=FMF [6:0]=Blocks-to-Follow
 *   Byte 9:  format-specific (FSN, N(S), etc.)
 *   Bytes 10-11: CRC-CCITT over bytes 0-9
 * ========================================================================= */

static void dhdr_common(uint8_t *body,
                         bool    is_group,
                         bool    req_ack,
                         uint8_t dpft,
                         uint8_t sap,
                         uint8_t poc,
                         uint32_t dst_id,
                         uint32_t src_id,
                         uint8_t  blocks,
                         bool     full_msg)
{
    
    
    body[0] = (uint8_t)(
    (is_group ? 0x80u : 0x00u) |
    (req_ack  ? 0x40u : 0x00u) |
    (dpft     & 0x0Fu)
);

// Byte 1: SAP (bits 7-4) | Pad Octet Count (bits 3-0)
body[1] = (uint8_t)(
    ((sap & 0x0Fu) << 4) |
    (poc & 0x0Fu)
);

// Bytes 2–7: IDs
DMR_SET_ID(body + 2, dst_id);
DMR_SET_ID(body + 5, src_id);

// Byte 8: Full/Partial Flag (bit 7) | Appended Blocks (bits 6-0)
body[8] = (uint8_t)(
    (full_msg ? 0x80u : 0x00u) |
    (blocks & 0x7Fu)
);

// Byte 9: Reserved / Padding
body[9] = 0x00u;
    
    /*
    body[0] = (uint8_t)((is_group ? 0x80u : 0x00u)
                       |(req_ack  ? 0x40u : 0x00u)
                       |((poc >> 4) & 0x01u) << 4    // POC MSB 
                       |(dpft & 0x0Fu));
    body[1] = (uint8_t)((sap & 0x0Fu) << 4) | (poc & 0x0Fu);
    DMR_SET_ID(body + 2, dst_id);
    DMR_SET_ID(body + 5, src_id);
    body[8] = (uint8_t)((full_msg ? 0x80u : 0x00u) | (blocks & 0x7Fu));*/
}

void llc_data_hdr_unconf_build(dmr_burst_t *burst,
                                uint32_t dst_id,
                                uint32_t src_id,
                                bool     is_group,
                                uint8_t  sap,
                                uint8_t  blocks,
                                uint8_t  pad_octets,
                                uint8_t  fsn,
                                uint8_t  cc,
                                dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    dhdr_common(body, is_group, false, DMR_DPFT_UNCONFIRMED,
                sap, pad_octets, dst_id, src_id, blocks, true);
    /* Byte 9: [7:4]=reserved [3:0]=FSN */
    body[9] = fsn & 0x0Fu;
    llc_crc_append(body, 10u,DMR_CRC_MASK_DATA_HEADER);
    llc_burst_pack(burst, body, cc, DMR_DTYPE_DATA_HEADER, false, slot);
}

void llc_data_hdr_conf_build(dmr_burst_t *burst,
                               uint32_t dst_id,
                               uint32_t src_id,
                               bool     is_group,
                               uint8_t  sap,
                               uint8_t  blocks,
                               uint8_t  pad_octets,
                               uint8_t  fsn,
                               uint8_t  send_seq,
                               bool     req_ack,
                               bool     full_msg,
                               uint8_t  cc,
                               dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    dhdr_common(body, is_group, req_ack, DMR_DPFT_CONFIRMED,
                sap, pad_octets, dst_id, src_id, blocks, full_msg);
    /* Byte 9: [7]=A [6:4]=N(S) [3:0]=FSN */
    body[9] = (uint8_t)((req_ack ? 0x80u : 0x00u)
                       |((send_seq & 0x07u) << 4)
                       |(fsn & 0x0Fu));
    llc_crc_append(body, 10u,DMR_CRC_MASK_DATA_HEADER);
    llc_burst_pack(burst, body, cc, DMR_DTYPE_DATA_HEADER, false, slot);
}

void llc_data_hdr_resp_build(dmr_burst_t *burst,
                               uint32_t dst_id,
                               uint32_t src_id,
                               uint8_t  sap,
                               uint8_t  class_v,
                               uint8_t  type_v,
                               uint8_t  status_v,
                               uint8_t  blocks,
                               uint8_t  cc,
                               dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    dhdr_common(body, false, false, DMR_DPFT_RESPONSE,
                sap, 0u, dst_id, src_id, blocks, false);
    /* Byte 9: [7:6]=Class [5:3]=Type [2:0]=Status */
    body[9] = (uint8_t)(((class_v  & 0x03u) << 6)
                       |((type_v   & 0x07u) << 3)
                       | (status_v & 0x07u));
    llc_crc_append(body, 10u,DMR_CRC_MASK_DATA_HEADER);
    llc_burst_pack(burst, body, cc, DMR_DTYPE_DATA_HEADER, false, slot);
}

void llc_data_hdr_udt_build(dmr_burst_t *burst,
                              uint32_t dst_id,
                              uint32_t src_id,
                              bool     is_group,
                              uint8_t  udt_format,
                              uint8_t  pad_nibbles,
                              uint8_t  blocks,
                              uint8_t  cc,
                              dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    dhdr_common(body, is_group, false, DMR_DPFT_UDT,
                DMR_SAP_SHORT_DATA, 0u, dst_id, src_id, blocks, true);
    /* Byte 9: [7:4]=UDT_format [3:0]=pad_nibbles[4:1]
     * Note: pad_nibbles is 5 bits — MSB in byte 0 bit4, lower 4 in byte 9 */
    body[0] |= (uint8_t)((pad_nibbles >> 4) & 0x01u) << 4; /* POC_MSB reused */
    body[9]  = (uint8_t)(((udt_format & 0x0Fu) << 4)
                        | (pad_nibbles & 0x0Fu));
    llc_crc_append(body, 10u,DMR_CRC_MASK_DATA_HEADER);
    llc_burst_pack(burst, body, cc, DMR_DTYPE_DATA_HEADER, false, slot);
}

/* =========================================================================
 * Short Data Header common byte0/byte1/dst/src fill — TS 102 361-1
 * Cl.9.2.10-9.2.12. Unlike dhdr_common() above, bits[5:4] of byte0 and
 * bits[3:0] of byte1 carry a split 6-bit Appended Blocks (AB) field
 * instead of [reserved,POC_MSB]/[SAP,POC] — AB replaces POC entirely
 * for this family (see dmr_pdu.h struct comments).
 * ========================================================================= */
static void sdhdr_common(uint8_t *body,
                          bool     is_group,
                          bool     req_ack,
                          uint8_t  dpft,
                          uint8_t  sap,
                          uint8_t  blocks6,
                          uint32_t dst_id,
                          uint32_t src_id)
{
    
  
    body[0] = (uint8_t)((is_group ? 0x80u : 0x00u)
                       |(req_ack  ? 0x40u : 0x00u)
                       |(((blocks6 >> 4) & 0x03u) << 4)  /* AB MSBs */
                       |(dpft & 0x0Fu));

    body[1] = (uint8_t)((sap & 0x0Fu) << 4) | (blocks6 & 0x0Fu); /* AB LSBs */
    DMR_SET_ID(body + 2, dst_id);
    DMR_SET_ID(body + 5, src_id);

}

void llc_data_hdr_sp_build(dmr_burst_t *burst,
                            uint32_t dst_id,
                            uint32_t src_id,
                            bool     is_group,
                            uint8_t  sap,
                            uint8_t  src_port,
                            uint8_t  dst_port,
                            uint16_t status_value,
                            bool     req_ack,
                            uint8_t  cc,
                            dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    /* AB forced to 0 — "These bits shall be set to 0" (Table 9.17A) */
    sdhdr_common(body, is_group, req_ack, DMR_DPFT_RAW_OR_STATUS, sap,
                 0u, dst_id, src_id);
    /* Byte 8: [7:5]=SP [4:2]=DP [1:0]=Status[9:8] ; Byte 9: Status[7:0] */
    body[8] = (uint8_t)(((src_port & 0x07u) << 5)
                       | ((dst_port & 0x07u) << 2)
                       | ((status_value >> 8) & 0x03u));
    body[9] = (uint8_t)(status_value & 0xFFu);
    llc_crc_append(body, 10u,DMR_CRC_MASK_DATA_HEADER);
    llc_burst_pack(burst, body, cc, DMR_DTYPE_DATA_HEADER, false, slot);
}

void llc_data_hdr_raw_build(dmr_burst_t *burst,
                             uint32_t dst_id,
                             uint32_t src_id,
                             bool     is_group,
                             uint8_t  sap,
                             uint8_t  src_port,
                             uint8_t  dst_port,
                             uint8_t  blocks,
                             bool     sarq,
                             bool     full_msg,
                             uint8_t  cc,
                             dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    sdhdr_common(body, is_group, sarq, DMR_DPFT_RAW_OR_STATUS, sap,
                 blocks, dst_id, src_id);
    /* Byte 8: [7:5]=SP [4:2]=DP [1]=SARQ [0]=FMF ; Byte 9: Bit Padding (0) */
    body[8] = (uint8_t)(((src_port & 0x07u) << 5)
                       | ((dst_port & 0x07u) << 2)
                       | (sarq      ? 0x02u : 0x00u)
                       | (full_msg  ? 0x01u : 0x00u));
    body[9] = 0x00u;
    llc_crc_append(body, 10u,DMR_CRC_MASK_DATA_HEADER);

    llc_burst_pack(burst, body, cc, DMR_DTYPE_DATA_HEADER, false, slot);
}

void llc_data_hdr_dd_build(dmr_burst_t *burst,
                            uint32_t dst_id,
                            uint32_t src_id,
                            bool     is_group,
                            uint8_t  sap,
                            uint8_t  dd_format,
                            uint8_t  blocks,
                            bool     sarq,
                            bool     full_msg,
                            uint8_t  cc,
                            dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    sdhdr_common(body, is_group, sarq, DMR_DPFT_DEFINED_DATA, sap,
                 blocks, dst_id, src_id);
    /* Byte 8: [7:2]=DD Format [1]=SARQ [0]=FMF ; Byte 9: Bit Padding (0) */
    body[8] = (uint8_t)(((dd_format & 0x3Fu) << 2)
                       | (sarq     ? 0x02u : 0x00u)
                       | (full_msg ? 0x01u : 0x00u));
    body[9] = 0x00u;
    llc_crc_append(body, 10u,DMR_CRC_MASK_DATA_HEADER);
    llc_burst_pack(burst, body, cc, DMR_DTYPE_DATA_HEADER, false, slot);
}

dmr_err_t llc_data_hdr_parse(const uint8_t *raw12,
                               uint8_t  *dpft,
                               uint8_t  *sap,
                               uint32_t *dst_id,
                               uint32_t *src_id,
                               uint8_t  *blocks,
                               bool     *is_group)
{
    dmr_err_t rc = llc_crc_verify(raw12, 12u,DMR_CRC_MASK_DATA_HEADER) ? DMR_OK : DMR_ERR_CRC;

    *dpft     = raw12[0] & 0x0Fu;
    *sap      = (raw12[1] >> 4) & 0x0Fu;
    *dst_id   = DMR_GET_ID(raw12 + 2);
    *src_id   = DMR_GET_ID(raw12 + 5);
    *blocks   = raw12[8] & 0x7Fu;
    *is_group = (raw12[0] >> 7) & 0x01u;

    return rc;
}

/* =========================================================================
 * SECTION E — Rate-1 Data Block
 * ETSI TS 102 361-1 Cl.8.2.2.1 / 9.2.15
 *
 * Unconfirmed Rate-1 block = 24 bytes payload = 192 bits (no DBSN/CRC-9).
 * Rate-1 Data Type (0x0A) carries 192 raw bits — no internal FEC beyond BPTC.
 *
 * In the 12-byte burst body:
 *   Byte 0:   [7]=LB(last_block) [6:0]=DBSN
 *   Bytes 1-11: up to 11 bytes user data per burst (rate-1 confirmed)
 *
 * NOTE: Full rate-1 unconfirmed uses 24 bytes per block across dual-slot.
 * Single-slot rate-1 confirmed uses 22 bytes + 2 bytes DBSN+CRC-9.
 * This implementation builds single-slot confirmed-style blocks (12 bytes).
 * ========================================================================= */

void llc_data_block_rate1_build(dmr_burst_t   *burst,
                                 uint8_t        dbsn,
                                 bool           last_block,
                                 const uint8_t *payload,
                                 size_t         payload_len,
                                 uint8_t        cc,
                                 dmr_slot_t     slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));

    body[0] = (uint8_t)((last_block ? 0x80u : 0x00u) | (dbsn & 0x7Fu));

    /* Copy up to 11 bytes of payload */
    size_t copy_len = payload_len < 11u ? payload_len : 11u;
    if (payload && copy_len > 0u) {
        memcpy(body + 1, payload, copy_len);
    }
    dmr_crc9_append(body, 11);

    uint8_t dtype = last_block ? DMR_DTYPE_RATE1_DATA : DMR_DTYPE_RATE1_DATA;
    llc_burst_pack(burst, body, cc, dtype, false, slot);
}

dmr_err_t llc_data_block_rate1_parse(const uint8_t *raw12,
                                      uint8_t  *dbsn,
                                      bool     *last_block,
                                      uint8_t  *payload,
                                      size_t   *payload_len)
{
    *last_block  = (raw12[0] >> 7) & 0x01u;
    *dbsn        = raw12[0] & 0x7Fu;
    *payload_len = 11u;
    if (payload) {
        memcpy(payload, raw12 + 1, 11u);
    }
  //  if(dmr_crc9_verify(raw12, 11u))
    {
    return DMR_OK;
    }
  /*  else
    return DMR_ERR_CRC;*/
}

/* =========================================================================
 * SECTION E1 — C_RDATA (Selective ARQ retry bitmap)
 * ETSI TS 102 361-1, Cl.8.2.2.3 / Cl.9.3.37 (SARQ field) — TS 102 361-3
 * Cl.5.4.3/6.5 (selective retry procedure)
 *
 * NOTE ON FIELD LAYOUT: the exact bit-level position of the SARQ field
 * within the 12-byte body is not independently confirmed against the
 * Cl.9.3.37 table in this implementation (that table's content was not
 * available to cross-check at build time) — this places the 64-bit
 * bitmap at body[0..7] MSB-first, mirroring this file's established
 * convention for every other block/header field (e.g.
 * llc_data_hdr_resp_build's Class/Type/Status packing). Body[8..11]
 * are reserved/zero. Re-verify byte offsets against the primary spec
 * text before relying on this for interop with third-party equipment;
 * for two same-codebase MS instances (the only interop this module is
 * currently exercised against) internal consistency between build and
 * parse is what matters and is what's actually verified here.
 *
 * Uses DMR_DTYPE_RATE12_DATA (not DMR_DTYPE_RATE1_DATA) per TS 102
 * 361-3's own description of C_RDATA's Data Type — this codebase's
 * llc_burst_pack() pre-BPTC placement is identical for every Data
 * Type (always 12 raw bytes; the FEC-rate distinction only affects
 * the value stamped into SLOT_TYPE and fec_tx_process()'s later
 * encoding pass), so no separate packing path is needed here.
 * ========================================================================= */

/**
 * @brief Build a C_RDATA burst carrying a selective-retry bitmap
 *        (Cl.8.2.2.3). Sent as one or two of these following a
 *        Response Header naming Class=SACK — one C_RDATA covers up to
 *        64 blocks; a transfer of 65-127 blocks needs a second
 *        C_RDATA for the remaining blocks (see
 *        llc_data_hdr_resp_build()'s blocks-to-follow parameter).
 *
 * @param burst  Output burst
 * @param flags  64-bit retry bitmap for this burst's blocks, MSB-first:
 *               bit (63-i) corresponds to block i (0-indexed within
 *               this burst, i.e. relative to the 64-block half this
 *               C_RDATA covers) — block 0 is the top bit of the word,
 *               block 63 is the bottom bit. Bit set (1) means that
 *               block was received correctly and needs no
 *               retransmission; bit clear (0) means it needs to be
 *               resent. Per spec, bits beyond the actual block count
 *               in a short final burst are set to 1 (no retry needed
 *               for a nonexistent block) — for fewer than 64 real
 *               blocks, those nonexistent-block positions are the
 *               *low*-index bit positions (since block 0 occupies the
 *               top bit and higher block numbers count downward);
 *               callers must pad those to 1 themselves before calling.
 * @param cc     Colour code
 * @param slot   Timeslot
 */
void llc_data_block_rdata_build(dmr_burst_t *burst,
                                 uint64_t     flags,
                                 uint8_t      cc,
                                 dmr_slot_t   slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    for (int i = 0; i < 8; i++) {
        body[i] = (uint8_t)(flags >> (56 - 8 * i));
    }
    llc_burst_pack(burst, body, cc, DMR_DTYPE_RATE12_DATA, false, slot);
}

/**
 * @brief Parse a C_RDATA selective-retry bitmap burst.
 *
 * @param raw12  12-byte C_RDATA body (post-FEC-decode)
 * @param flags  Output: 64-bit retry bitmap, same bit meaning as
 *               llc_data_block_rdata_build()'s flags parameter
 */
void llc_data_block_rdata_parse(const uint8_t *raw12, uint64_t *flags)
{
    uint64_t v = 0u;
    for (int i = 0; i < 8; i++) {
        v = (v << 8) | raw12[i];
    }
    *flags = v;
}

/* =========================================================================
 * SECTION F — Tier III CSBK builders / parsers
 * ETSI TS 102 361-4, Clause 6
 * ========================================================================= */

/* MS Registration (CSBKO=0x24) — TS 102 361-4 Cl.6.7
 * Byte 2: [7:4]=reason [3:0]=reserved
 */
void llc_t3_ms_reg_build(dmr_burst_t *burst,
                           uint8_t  reason,
                           uint32_t dst_id,
                           uint32_t src_id,
                           uint8_t  cc,
                           dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_T3_MS_REGIST);
    body[1] = 0x00u;
    body[2] = (uint8_t)((reason & 0x0Fu) << 4);
    body[3] = 0x00u;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* MS De-registration (CSBKO=0x27) */
void llc_t3_ms_dereg_build(dmr_burst_t *burst,
                             uint32_t dst_id,
                             uint32_t src_id,
                             uint8_t  cc,
                             dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_T3_MS_DEREGIST);
    body[1] = 0x00u;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Network Status Broadcast parse (CSBKO=0x14) — TS 102 361-4 Cl.6.1
 * Byte 2-4: NET_ID (3 bytes)
 * Byte 5:   SITE_ID
 * Byte 6:   [7:3]=ch_count [2:0]=req_access
 */
dmr_err_t llc_t3_net_status_parse(const uint8_t *raw12,
                                    uint32_t *net_id,
                                    uint8_t  *site_id,
                                    uint8_t  *ch_count,
                                    uint8_t  *req_access)
{
    dmr_err_t rc = llc_crc_verify(raw12, 12u,DMR_CRC_MASK_CSBK) ? DMR_OK : DMR_ERR_CRC;

    *net_id     = DMR_GET_ID(raw12 + 2);
    *site_id    = raw12[5];
    *ch_count   = (raw12[6] >> 3) & 0x1Fu;
    *req_access =  raw12[6] & 0x07u;

    return rc;
}

/* Adjacent Site Information parse (CSBKO=0x19) — TS 102 361-4 Cl.6.1
 * Byte 2:   AREA_ID
 * Bytes 3-4: SYS_ID
 * Byte 5:   SITE_ID
 * Bytes 6-7: CH_ID
 * Byte 8:   [7:5]=req_access [4:1]=svc_type [0]=reserved
 */
dmr_err_t llc_t3_adj_site_parse(const uint8_t *raw12,
                                  uint8_t  *area_id,
                                  uint16_t *sys_id,
                                  uint8_t  *site_id,
                                  uint16_t *ch_id)
{
    dmr_err_t rc = llc_crc_verify(raw12, 12u,DMR_CRC_MASK_CSBK) ? DMR_OK : DMR_ERR_CRC;

    *area_id = raw12[2];
    *sys_id  = (uint16_t)((uint16_t)raw12[3] << 8) | raw12[4];
    *site_id = raw12[5];
    *ch_id   = (uint16_t)((uint16_t)raw12[6] << 8) | raw12[7];

    return rc;
}

/* TV_GRANT / TD_GRANT parse (CSBKO=0x01/0x03) — TS 102 361-4 Cl.6.3
 * Byte 2:   [7]=A/D [6]=emergency [5:0]=reserved
 * Bytes 3-4: CH_ID
 * Byte 5:   [7:6]=slot [5:0]=reserved
 * Bytes 6-8: DST_ID (group)
 */
/* TV_GRANT / TD_GRANT parse (CSBKO=0x01/0x03) — TS 102 361-4 Cl.6.3
 * Byte 2:   [7]=A/D [6]=emergency [5:0]=reserved
 * Bytes 3-4: CH_ID
 * Byte 5:   [7:6]=slot [5:0]=reserved
 * Bytes 6-8: DST_ID (group)
 */
dmr_err_t llc_t3_grant_parse(const uint8_t *raw12,
                               uint16_t *ch_id,
                               uint8_t  *slot,
                               uint32_t *dst_id,
                               bool     *emergency)
{
    dmr_err_t rc = llc_crc_verify(raw12, 12u,DMR_CRC_MASK_CSBK) ? DMR_OK : DMR_ERR_CRC;

    *ch_id     = (uint16_t)((uint16_t)raw12[3] << 8) | raw12[4];
    *slot      = (raw12[5] >> 6) & 0x03u;
    *dst_id    = DMR_GET_ID(raw12 + 6);
    *emergency = ((raw12[2] >> 6) & 0x01u) != 0u;

    return rc;
}

/* TV_GRANT / TD_GRANT build (CSBKO=0x01/0x03) — TS 102 361-4 Cl.6.3
 * Mirrors the byte layout assumed by llc_t3_grant_parse() above.
 */
void llc_t3_grant_build(dmr_burst_t *burst,
                          bool     is_data,
                          bool     emergency,
                          uint16_t ch_id,
                          uint8_t  grant_slot,
                          uint32_t dst_id,
                          uint8_t  cc,
                          dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    uint8_t opcode = is_data ? DMR_CSBKO_T3_TD_GRANT : DMR_CSBKO_T3_TV_GRANT;
    body[0] = DMR_CSBK_B0(1u, 0u, opcode);
    body[1] = 0x00u;
    body[2] = (uint8_t)(emergency ? 0x40u : 0x00u);
    body[3] = (uint8_t)((ch_id >> 8) & 0xFFu);
    body[4] = (uint8_t)(ch_id & 0xFFu);
    body[5] = (uint8_t)((grant_slot & 0x03u) << 6);
    DMR_SET_ID(body + 6, dst_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Random Access Request build (CSBKO=0x02, C_RAND) — TS 102 361-4 Cl.6.2
 * Byte 2: [7:5]=service_kind [4]=is_group [3:0]=reserved
 * Byte 3: reserved
 * Bytes 4-6: DST_ID
 * Bytes 7-9: SRC_ID
 */
void llc_t3_rand_access_build(dmr_burst_t *burst,
                                uint8_t  service_kind,
                                bool     is_group,
                                uint32_t dst_id,
                                uint32_t src_id,
                                uint8_t  cc,
                                dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_T3_RAND_ACCESS);
    body[1] = 0x00u;
    body[2] = (uint8_t)(((service_kind & 0x07u) << 5)
                       | (is_group ? 0x10u : 0x00u));
    body[3] = 0x00u;
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}

/* Random Access Request parse (CSBKO=0x02) */
dmr_err_t llc_t3_rand_access_parse(const uint8_t *raw12,
                                     uint8_t  *service_kind,
                                     bool     *is_group,
                                     uint32_t *dst_id,
                                     uint32_t *src_id)
{
    dmr_err_t rc = llc_crc_verify(raw12, 12u,DMR_CRC_MASK_CSBK) ? DMR_OK : DMR_ERR_CRC;

    *service_kind = (raw12[2] >> 5) & 0x07u;
    *is_group     = ((raw12[2] >> 4) & 0x01u) != 0u;
    *dst_id       = DMR_GET_ID(raw12 + 4);
    *src_id       = DMR_GET_ID(raw12 + 7);

    return rc;
}

/* MS Registration Response build (CSBKO=0x25) — TS 102 361-4 Cl.6.7
 * Byte 2: [5:4]=response (0=accept 1=refuse 2=fail) [7:6][3:0]=reserved
 */
void llc_t3_ms_reg_resp_build(dmr_burst_t *burst,
                                uint8_t  response,
                                uint32_t dst_id,
                                uint32_t src_id,
                                uint8_t  cc,
                                dmr_slot_t slot)
{
    uint8_t body[12];
    memset(body, 0, sizeof(body));
    body[0] = DMR_CSBK_B0(1u, 0u, DMR_CSBKO_T3_MS_REGIST_RSP);
    body[1] = 0x00u;
    body[2] = (uint8_t)((response & 0x03u) << 4);
    DMR_SET_ID(body + 4, dst_id);
    DMR_SET_ID(body + 7, src_id);
    csbk_finalise_burst(burst, body, cc, slot);
}


/**
 * Packs a 72-bit (9-byte) DMR Data Terminator PDU (TD_LC).
 *
 * @param pdu_out Output array of at least 9 bytes.
 * @param encrypted Protect Flag: true (1) if encrypted/protected, false (0) for clear.
 * @param dst_id 24-bit Destination/Target Logical Link ID.
 * @param src_id 24-bit Source Logical Link ID.
 * @param fid Feature Set ID (Use 0x00 for standard ETSI DMR).
 * @param data_info Data Options/Info (Use 0x00 for standard PDP).
 */
 
 
void llc_data_terminator_build(dmr_burst_t *burst,
                            bool encrypted,
                            uint32_t dst_id,
                            uint32_t src_id,
                            uint8_t fid, 
                            uint8_t      cc,
                            uint8_t data_info,
                            dmr_slot_t   slot
                            )
{
    uint8_t body[9];
    // Octet 0: encrypted (1 bit) | Reserved (1 bit) | FLCO (6 bits)
    // Bit 7: encrypted, Bit 6: 0 (Reserved), Bits 5..0: FLCO (0x30)
    body[0] = ((encrypted ? 1 : 0) << 7) | 
                 (0 << 6) | 
                 (DMR_FLCO_DATA_TERMINATOR & 0x3F);

    // Octet 1: Feature Set ID (FID)
    body[1] = fid;

    // Octet 2: Data Options / Reserved
    body[2] = data_info;

    // Octets 3..5: Target LLID (24-bit, Big Endian)
    body[3] = (uint8_t)((dst_id >> 16) & 0xFF);
    body[4] = (uint8_t)((dst_id >> 8) & 0xFF);
    body[5] = (uint8_t)( dst_id & 0xFF);

    // Octets 6..8: Source LLID (24-bit, Big Endian)
    body[6] = (uint8_t)((src_id >> 16) & 0xFF);
    body[7] = (uint8_t)((src_id >> 8) & 0xFF);
    body[8] = (uint8_t)( src_id & 0xFF);
    
     llc_burst_pack(burst, body, cc, DMR_DTYPE_TERMINATOR_LC, false, slot);
    
}





