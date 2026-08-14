void llc_crc_append(uint8_t *data, size_t len,uint16_t mask);

void llc_crc_append(uint8_t *data, size_t len,uint16_t mask);

/**
 * @file dmr_llc.h
 * @brief MOD-04 — Logical Link Control (LLC) — Public Interface
 *
 * ETSI TS 102 361-1, Clauses 6–9 / TS 102 361-2, Clause 7
 *
 * Responsibilities
 * ================
 * LLC sits between MAC (burst containers) and CCL (call state machines).
 * It handles every PDU type that travels across the DMR air interface:
 *
 *   TX path  (CCL → LLC → MAC):
 *     1. CCL supplies logical parameters (addresses, opcodes, data)
 *     2. LLC builds the typed PDU struct (dmr_csbk_t, dmr_full_lc_t, …)
 *     3. LLC appends / verifies CRC-CCITT where required
 *     4. LLC packs the 96 info-bits into dmr_burst_t.raw[] via the
 *        burst-assembly helpers in dmr_pdu.h
 *     5. Sets correct SYNC pattern + SLOT_TYPE (cc, dtype, Golay FEC)
 *     6. Delivers the ready-to-transmit dmr_burst_t to MAC
 *
 *   RX path  (MAC → LLC → CCL):
 *     1. MAC delivers a dmr_burst_t (BPTC already decoded by MOD-02)
 *     2. LLC reads dtype from SLOT_TYPE, verifies CRC, dispatches to
 *        the correct parse function
 *     3. Returns typed result to CCL via llc_rx_result_t
 *
 * CRC-CCITT
 * =========
 * Polynomial : G(x) = x^16 + x^12 + x^5 + 1  (0x1021)
 * Initial value : 0x0000
 * Input/output reflected : NO  (MSB-first, straight)
 * Final XOR with 0xFFFF (per ETSI TS 102 361-1, Annex B.3.8)
 *
 * ETSI References
 * ===============
 * TS 102 361-1  Cl.7.2      CSBK message structure
 * TS 102 361-1  Cl.7.1      Full LC / Short LC
 * TS 102 361-1  Cl.8.2.1    Data header block structure
 * TS 102 361-1  Annex B.3.8 CRC-CCITT
 * TS 102 361-2  Cl.7.1      Layer 3 PDUs (LC, CSBK opcodes)
 * TS 102 361-3  Cl.5-6      Packet data headers and blocks
 */

#ifndef DMR_LLC_H
#define DMR_LLC_H

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
 * Section 1 — CRC-CCITT
 * ETSI TS 102 361-1, Annex B.3.8
 * ========================================================================= */

/**
 * @brief Compute CRC-CCITT over a byte buffer.
 *
 * Initial remainder = 0x0000, generator = 0x1021, final XOR = 0xFFFF.
 * MSB first, no reflection.
 *
 * @param data  Input bytes
 * @param len   Number of bytes
 * @return      16-bit CRC (already XOR'd with 0xFFFF per ETSI)
 */
uint16_t llc_crc_ccitt(const uint8_t *data, size_t len);

/**
 * @brief Append CRC-CCITT as two big-endian bytes at data[len..len+1].
 *        Buffer must have at least len+2 bytes allocated.
 */
void llc_crc_append(uint8_t *data, size_t len,uint16_t mask);

/**
 * @brief Verify CRC-CCITT: last two bytes of buf[0..len-1] are the CRC
 *        over buf[0..len-3].
 * @return true if CRC is correct
 */
bool llc_crc_verify(const uint8_t *data, size_t len,uint16_t mask);



/**
 * @brief Append CRC-CCITT as two big-endian bytes at data[len..len+1].
 *        Buffer must have at least len+2 bytes allocated.
 */

 void dmr_crc9_append(uint8_t *buffer, size_t payload_bit_len);
 
 
 /**
 * @brief Verify CRC-9: 
 * @return true if CRC is correct
 */
bool dmr_crc9_verify(const uint8_t *buffer, size_t total_bit_len);

/* =========================================================================
 * Section 2 — Burst assembly / disassembly
 *
 * These wrap dmr_burst_set_sync + dmr_burst_set_slot_type + memcpy of the
 * 96 BPTC info-bits, producing a wire-ready dmr_burst_t.
 *
 * NOTE: BPTC(196,96) FEC encoding is the responsibility of MOD-02.
 *       LLC writes the 96 raw info-bits; MOD-02 encodes before TX.
 *       On RX, MOD-02 decodes first; LLC receives the 96 clean bits.
 *       These helpers operate on the 12-byte (96-bit) logical PDU bodies
 *       which exactly match the 96 info-bits of BPTC(196,96).
 * ========================================================================= */

/**
 * @brief Pack a 12-byte PDU body into a data burst raw[] at INFO_1+INFO_2.
 *
 * Writes:
 *   - SYNC  : DMR_SYNC_MS_DATA (inbound) or DMR_SYNC_BS_DATA (outbound)
 *   - SLOT_TYPE : cc, dtype, Golay(18,6) FEC over (cc<<4|dtype)
 *   - INFO_1 + INFO_2 : the 96 bits of pdu12[0..11]
 *
 * The 96 bits of a 12-byte PDU fill INFO_1 (98 bits) and INFO_2 (98 bits)
 * as follows per ETSI TS 102 361-1 Table B.3:
 *   info-bits I(95)..I(0) → placed into INFO_1/INFO_2 by BPTC encoder.
 *   Before BPTC encoding, the 96 info bytes are simply placed as:
 *     I(95)=pdu12[0] bit7, I(94)=pdu12[0] bit6, ... I(0)=pdu12[11] bit0
 *
 * @param burst    Output: burst container to fill
 * @param pdu12    12-byte PDU body (96 bits of BPTC info payload)
 * @param cc       Colour code (0-15)
 * @param dtype    Data Type (DMR_DTYPE_*)
 * @param outbound true=BS→MS (use BS_DATA sync), false=MS→BS (MS_DATA sync)
 * @param slot     Timeslot (DMR_SLOT_1 or DMR_SLOT_2)
 */
void llc_burst_pack(dmr_burst_t   *burst,
                    const uint8_t *pdu12,
                    uint8_t        cc,
                    uint8_t        dtype,
                    bool           outbound,
                    dmr_slot_t     slot);

/**
 * @brief Extract the 12-byte PDU body from a decoded data burst raw[].
 *
 * Inverse of llc_burst_pack (operates on BPTC-decoded bits, i.e. after
 * MOD-02 processing).  Simply reads INFO_1 and INFO_2 back into pdu12[].
 *
 * @param raw      33-byte burst raw[] (post-BPTC-decode)
 * @param pdu12    Output: 12-byte PDU body
 * @param dtype    Output: Data Type from SLOT_TYPE
 * @param cc       Output: Colour Code from SLOT_TYPE
 * @return DMR_OK or DMR_ERR_INVALID_PARAM if not a data burst
 */
dmr_err_t llc_burst_unpack(const uint8_t *raw,
                            uint8_t       *pdu12,
                            uint8_t       *dtype,
                            uint8_t       *cc);

/* =========================================================================
 * Section 3 — Full LC build / parse
 * ETSI TS 102 361-1, Cl.7.1 / TS 102 361-2, Cl.7.1.1
 * ========================================================================= */

/**
 * @brief Build a Group Voice Channel User Full LC (FLCO=0x00).
 *        TS 102 361-2 Table 7.1
 *
 * @param lc       Output: 12-byte Full LC PDU (bytes 0-8 info + 3 RS FEC)
 *                 RS(12,9) FEC bytes are left zero — filled by MOD-02.
 * @param svc      Service Options byte (use DMR_SVC_* macros)
 * @param group_id 24-bit destination group ID
 * @param src_id   24-bit source radio ID
 */
void llc_full_lc_grp_voice_build(dmr_full_lc_t *lc,
                                  uint8_t  svc,
                                  uint8_t  fid,
                                  uint32_t group_id,
                                  uint32_t src_id);

/**
 * @brief Build an Individual Voice Channel User Full LC (FLCO=0x03).
 *        TS 102 361-2 Table 7.2
 */
void llc_full_lc_ind_voice_build(dmr_full_lc_t *lc,
                                  uint8_t  svc,
                                  uint8_t  fid,
                                  uint32_t dst_id,
                                  uint32_t src_id);

/**
 * @brief Build a GPS Info Full LC (FLCO=0x08).
 *        TS 102 361-2 Table 7.3
 *
 * @param lc          Output Full LC
 * @param lat_deg7    Latitude  × 1e7 (signed, WGS84 degrees × 10^7)
 * @param lon_deg7    Longitude × 1e7 (signed)
 * @param pos_error   Position error class 0-7 (DMR_POS_ERR_*)
 */
void llc_full_lc_gps_build(dmr_full_lc_t *lc,
                             int32_t lat_deg7,
                             int32_t lon_deg7,
                             uint8_t  fid,
                             uint8_t pos_error);

/**
 * @brief Parse any Full LC PDU — determine FLCO and fill a union.
 *
 * @param raw12    12-byte Full LC PDU body (bytes 0-11)
 * @param flco     Output: FLCO opcode (DMR_FLCO_*)
 * @param dst_id   Output: destination ID (parsed from position 3-5)
 * @param src_id   Output: source ID (parsed from position 6-8)
 * @param svc      Output: service options byte (byte 2, where present)
 * @return DMR_OK always (unknown FLCOs return their raw opcode in *flco)
 */
dmr_err_t llc_full_lc_parse(const uint8_t *raw12,
                              uint8_t  *flco,
                              uint32_t *dst_id,
                              uint32_t *src_id,
                              uint8_t  *svc);

/**
 * @brief Build a Voice LC Header burst (Data Type 0x01).
 *        Wraps llc_full_lc_grp_voice_build + llc_burst_pack.
 */
void llc_voice_lc_header_build(dmr_burst_t   *burst,
                                const dmr_full_lc_t *lc,
                                uint8_t  cc,
                                bool     outbound,
                                dmr_slot_t slot);

/**
 * @brief Build a Terminator with LC burst (Data Type 0x02).
 */
void llc_terminator_lc_build(dmr_burst_t         *burst,
                               const dmr_full_lc_t *lc,
                               uint8_t  cc,
                               bool     outbound,
                               dmr_slot_t slot);

/* =========================================================================
 * Section 4 — CSBK build / parse
 * ETSI TS 102 361-1, Cl.7.2 / TS 102 361-2, Cl.7.1.2
 * ========================================================================= */

/**
 * @brief Parse the opcode from any CSBK raw PDU body (12 bytes).
 *
 * @param raw12  12-byte CSBK PDU
 * @return CSBK opcode (bits [5:0] of byte 0)
 */
static inline uint8_t llc_csbk_opcode(const uint8_t *raw12) {
    return raw12[0] & 0x3Fu;
}

/**
 * @brief Build a BS Down-link Activation CSBK burst (CSBKO=0x28).
 *        TS 102 361-2 Table 7.5a
 */
void llc_csbk_bs_dwna_build(dmr_burst_t *burst,
                              uint32_t bs_id,
                              uint32_t src_id,
                              uint8_t  cc,
                               uint8_t  fid,
                              dmr_slot_t slot);

/**
 * @brief Build a Preamble CSBK burst (CSBKO=0x3D).
 *        TS 102 361-2 Table 7.7
 *
 * @param is_data   true=data content follows, false=CSBK content follows
 * @param is_group  true=group address, false=individual
 * @param cbf       CSBK Blocks to Follow (count NOT including this preamble)
 */
void llc_csbk_preamble_build(dmr_burst_t *burst,
                               bool     is_data,
                               bool     is_group,
                               uint8_t  cbf,
                               uint32_t dst_id,
                               uint32_t src_id,
                               uint8_t  cc,
                               dmr_slot_t slot);

/**
 * @brief Build a Unit-to-Unit Voice Service Request CSBK (CSBKO=0x04).
 *        TS 102 361-2 Table 7.5b  (OACSU presence check)
 */
void llc_csbk_uu_v_req_build(dmr_burst_t *burst,
                               uint8_t  svc,
                               uint32_t dst_id,
                               uint32_t src_id,
                               uint8_t  cc,
                               dmr_slot_t slot);

/**
 * @brief Build a Unit-to-Unit Answer Response CSBK (CSBKO=0x24).
 *        TS 102 361-2 Table 7.5c
 *
 * @param response  DMR_UU_ANS_PROCEED or DMR_UU_ANS_DENY
 */
void llc_csbk_uu_ans_rsp_build(dmr_burst_t *burst,
                                 uint8_t  svc,
                                 uint8_t  response,
                                 uint32_t dst_id,
                                 uint32_t src_id,
                                 uint8_t  cc,
                                 dmr_slot_t slot);

/**
 * @brief Build a Call Alert CSBK (CSBKO=0x1F).
 */
void llc_csbk_call_alert_build(dmr_burst_t *burst,
                                 uint32_t dst_id,
                                 uint32_t src_id,
                                 uint8_t  cc,
                                 dmr_slot_t slot);

/**
 * @brief Build an Acknowledge Response CSBK (CSBKO=0x20).
 */
void llc_csbk_ack_rsp_build(dmr_burst_t *burst,
                              uint32_t dst_id,
                              uint32_t src_id,
                              uint8_t  cc,
                              dmr_slot_t slot);

/**
 * @brief Build an Emergency Alarm CSBK (CSBKO=0x27/0x28).
 *        TS 102 361-2 Cl.7.2 / TS 102 361-4 Cl.6.9
 */
void llc_csbk_emerg_alarm_build(dmr_burst_t *burst,
                                  uint8_t  emerg_type,
                                  uint32_t dst_id,
                                  uint32_t src_id,
                                  uint8_t  cc,
                                  dmr_slot_t slot);

/**
 * @brief Build a Cancel Call Alert CSBK (CSBKO=0x23).
 */
void llc_csbk_cancel_build(dmr_burst_t *burst,
                             uint32_t dst_id,
                             uint32_t src_id,
                             uint8_t  cc,
                             dmr_slot_t slot);

/**
 * @brief Parse a received CSBK burst into its typed representation.
 *
 * @param raw12    12-byte CSBK body (post-BPTC decode, post-CRC strip)
 * @param opcode   Output: CSBK opcode
 * @param dst_id   Output: destination ID
 * @param src_id   Output: source ID
 * @return DMR_OK, DMR_ERR_CRC if CRC fails
 */
dmr_err_t llc_csbk_parse(const uint8_t *raw12,
                           uint8_t  *opcode,
                           uint32_t *dst_id,
                           uint32_t *src_id);

/* =========================================================================
 * Section 4A — Channel Timing CSBK (CT_CSBK) build / parse
 * ETSI TS 102 361-2, Cl.6.2 (TDMA direct mode wide area timing) / Cl.7.1.2.6
 *
 * Distinct field layout from the generic dst/src CSBKs above — CT_CSBK
 * carries no addresses at all, only wide-area-timing state. Octets 0-1
 * follow the standard CSBK header (LB=1, PF=0, CSBKO=0x07, FID=0x00).
 * Octets 2-9 (64 bits) are packed MSB-first in strict field order per
 * Table 7.8: SA(11) Gen(5) LID(20) NL(1) LDI(2) CTO_msb(1) SID(20)
 * Reserved(1) SDI(2) CTO_lsb(1). CT_CSBK always transmits with the
 * "All Site" colour code (0xF, Cl.6.2.2.2) — the builder sets this
 * internally, callers do not pass a colour code.
 * ========================================================================= */

/** Channel Timing Opcode (CTO) values — Cl.7.2.11, Table 7.20 */
#define DMR_CTO_UNALIGNED_REQ    0x0u  /* 00 — unaligned request               */
#define DMR_CTO_UNALIGNED_TERM   0x1u  /* 01 — unaligned terminator            */
#define DMR_CTO_ALIGNED_STATUS   0x2u  /* 10 — aligned status (req/term/resp)  */
#define DMR_CTO_ALIGNED_PUSH     0x3u  /* 11 — aligned push (beacon/prop/corr) */

/** Wide-area timing "All Site" colour code — Cl.6.2.2.2 */
#define DMR_CC_ALL_SITE          0x0Fu

typedef struct {
    uint16_t sync_age;     /**< SA,  11 bits (0-2047), SAIncr=500ms units      */
    uint8_t  gen;          /**< Gen,  5 bits (0-31), hops from leader           */
    uint32_t leader_id;    /**< LID, 20 bits                                    */
    bool     new_leader;   /**< NL,   1 bit                                     */
    uint8_t  leader_di;    /**< LDI,  2 bits — leader's preference/eligibility  */
    uint8_t  cto;          /**< CTO,  2 bits — DMR_CTO_* combined MSB+LSB       */
    uint32_t source_id;    /**< SID, 20 bits                                    */
    uint8_t  source_di;    /**< SDI,  2 bits                                    */
} dmr_ct_csbk_t;

/**
 * @brief Build a Channel Timing CSBK (CT_CSBK) burst, CSBKO=0x07.
 *        Always uses the All Site colour code (0xF) per Cl.6.2.2.2.
 */
void llc_ct_csbk_build(dmr_burst_t *burst,
                        const dmr_ct_csbk_t *ct,
                        dmr_slot_t slot);

/**
 * @brief Parse a received CT_CSBK burst body into its typed fields.
 * @return DMR_OK, or DMR_ERR_CRC if the CRC-CCITT check fails (fields
 *         are still populated — caller decides whether to act on it).
 */
dmr_err_t llc_ct_csbk_parse(const uint8_t *raw12, dmr_ct_csbk_t *ct);

/* =========================================================================
 * Section 5 — PI Header build
 * ETSI TS 102 361-1, Cl.9.1.5 / Data Type 0x00
 * ========================================================================= */

/**
 * @brief Build a Privacy Indicator (PI) Header burst.
 *
 * @param burst    Output burst
 * @param mi       9-byte Message Indicator (72-bit cryptographic seed)
 * @param alg_id   Algorithm ID (DMR_ALG_*)
 * @param key_id   16-bit Key ID
 * @param dst_id   24-bit destination ID
 * @param cc       Colour code
 * @param slot     Timeslot
 */
void llc_pi_header_build(dmr_burst_t   *burst,
                          const uint8_t  mi[9],
                          uint8_t        alg_id,
                          uint16_t       key_id,
                          uint32_t       dst_id,
                          uint8_t        cc,
                          dmr_slot_t     slot);

/* =========================================================================
 * Section 6 — Data Header build / parse
 * ETSI TS 102 361-1, Cl.8.2.1 / Data Type 0x06
 * ========================================================================= */

/**
 * @brief Build an Unconfirmed Data Header burst (DPFT=0x02).
 *        TS 102 361-1 Figure 8.3
 *
 * @param burst      Output burst
 * @param dst_id     24-bit destination LLID
 * @param src_id     24-bit source LLID
 * @param is_group   true=group address
 * @param sap        Service Access Point (DMR_SAP_*)
 * @param blocks     Blocks to follow (not counting header)
 * @param pad_octets Pad octet count (0-31)
 * @param fsn        Fragment Sequence Number (4 bits)
 * @param cc         Colour code
 * @param slot       Timeslot
 */
void llc_data_hdr_unconf_build(dmr_burst_t *burst,
                                uint32_t dst_id,
                                uint32_t src_id,
                                bool     is_group,
                                uint8_t  sap,
                                uint8_t  blocks,
                                uint8_t  pad_octets,
                                uint8_t  fsn,
                                uint8_t  cc,
                                dmr_slot_t slot);

/**
 * @brief Build a Confirmed Data Header burst (DPFT=0x03).
 *        TS 102 361-1 Figure 8.4
 *
 * @param send_seq   N(S) send sequence number (0-7)
 * @param req_ack    true=response requested (A-bit)
 * @param full_msg   Full Message Flag (F, Cl.9.3.20): true=this header
 *                   announces a complete message (the normal case for a
 *                   first transmission); false=this header announces a
 *                   partial retransmission of specific block(s) only —
 *                   TS 102 361-3 Cl.5.4.3/6.5 selective-retry procedure:
 *                   "the Full Message Flag... shall be set to 0(2) to
 *                   indicate it is transmitting a partial message".
 */
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
                               dmr_slot_t slot);

/**
 * @brief Build a Response Data Header burst (DPFT=0x01).
 *        TS 102 361-1 Figure 8.5 / Table 8.3
 *
 * @param sap       SAP Identifier — TS 102 361-3 Cl.5.4.1.3/6.4: "shall be
 *                  the same value as contained in" the message being
 *                  responded to (DMR_SAP_IP_PACKET or DMR_SAP_SHORT_DATA)
 * @param class_v   Response class (0=ACK, 1=NACK, 2=SACK)
 * @param type_v    Response type (see Table 8.3)
 * @param status_v  Status / sequence number
 * @param blocks    Blocks to follow (0 for simple ACK/NACK)
 */
void llc_data_hdr_resp_build(dmr_burst_t *burst,
                               uint32_t dst_id,
                               uint32_t src_id,
                               uint8_t  sap,
                               uint8_t  class_v,
                               uint8_t  type_v,
                               uint8_t  status_v,
                               uint8_t  blocks,
                               uint8_t  cc,
                               dmr_slot_t slot);

/**
 * @brief Build a UDT Short Data Header burst (DPFT=0x00, SAP=0x00).
 *        TS 102 361-1 Figure 8.10  (SDS up to 20 bytes)
 *
 * @param udt_format UDT Format nibble (0=binary, 3=7-bit ASCII, …)
 * @param pad_nibbles Number of pad nibbles (0-31)
 */
void llc_data_hdr_udt_build(dmr_burst_t *burst,
                              uint32_t dst_id,
                              uint32_t src_id,
                              bool     is_group,
                              uint8_t  udt_format,
                              uint8_t  pad_nibbles,
                              uint8_t  blocks,
                              uint8_t  cc,
                              dmr_slot_t slot);

/* =========================================================================
 * Section 6A — Short Data Header build (DD_HEAD/R_HEAD/SP_HEAD)
 * TS 102 361-1 Cl.9.2.10-9.2.12, TS 102 361-3 Cl.6
 *
 * All three share dst/src LLID at the same byte offsets as Section 6's
 * family but replace POC with a split 6-bit Appended Blocks (AB) field
 * — see the dmr_data_hdr_{sp,raw,dd}_t struct comments in dmr_pdu.h.
 * RX-side: llc_rx_dispatch() already extracts dpft/sap/dst_id/src_id/
 * blocks_to_follow/is_group correctly for these (dmr_pdu.h's
 * DMR_DPFT_DEFINED_DATA/DMR_DPFT_RAW_OR_STATUS branch in llc_burst.c);
 * callers read the type-specific tail fields (ports, DD format,
 * Status/Precoded value, SARQ/FMF) directly from the decoded body,
 * matching the existing convention for the Response header's Class/
 * Type/Status tail (see dmr_ccl_data.c's use of result->body[9]).
 * ========================================================================= */

/**
 * @brief Build a Status/Precoded Short Data Header (SP_HEAD, DPFT=0x0E,
 *        AB forced to 0 per spec — no data blocks follow).
 *
 * @param src_port,dst_port  3-bit port numbers (0-7)
 * @param status_value       10-bit Status/Precoded code
 * @param req_ack            true=confirmed (Response Requested)
 */
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
                            dmr_slot_t slot);

/**
 * @brief Build a Raw Short Data Header (R_HEAD, DPFT=0x0E).
 *
 * @param blocks     Appended Blocks — data blocks to follow (0-63)
 * @param sarq       true=confirmed (stop-and-wait; no N(S), see FMF)
 * @param full_msg   Full Message Flag — false on retransmit of a
 *                   partial message, matching C_HEAD's FMF convention
 */
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
                             dmr_slot_t slot);

/**
 * @brief Build a Defined Data Short Data Header (DD_HEAD, DPFT=0x0D).
 *
 * @param dd_format  6-bit predefined-format code
 */
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
                            dmr_slot_t slot);

/**
 * @brief Parse a Data Header burst into its fields.
 *
 * @param raw12      12-byte data header body
 * @param dpft       Output: Data Packet Format Type (DMR_DPFT_*)
 * @param sap        Output: SAP identifier
 * @param dst_id     Output: destination LLID
 * @param src_id     Output: source LLID
 * @param blocks     Output: blocks to follow
 * @param is_group   Output: group flag
 * @return DMR_OK, DMR_ERR_CRC
 */
dmr_err_t llc_data_hdr_parse(const uint8_t *raw12,
                               uint8_t  *dpft,
                               uint8_t  *sap,
                               uint32_t *dst_id,
                               uint32_t *src_id,
                               uint8_t  *blocks,
                               bool     *is_group);

/* =========================================================================
 * Section 7 — Rate-1 Data Block build / parse
 * ETSI TS 102 361-1, Cl.8.2.2.1 / Data Type 0x0A
 * ========================================================================= */

/**
 * @brief Build a Rate-1 Unconfirmed Data Block burst.
 *
 * @param burst      Output burst
 * @param dbsn       Data Block Serial Number (0-127)
 * @param last_block true=last block in transfer (Data Type = Rate-1 Last)
 * @param payload    Up to 24 bytes of user data (or 20 bytes if last+CRC)
 * @param payload_len Bytes in payload (max 24 unconfirmed, 22 confirmed)
 * @param cc         Colour code
 * @param slot       Timeslot
 */
void llc_data_block_rate1_build(dmr_burst_t   *burst,
                                 uint8_t        dbsn,
                                 bool           last_block,
                                 const uint8_t *payload,
                                 size_t         payload_len,
                                 uint8_t        cc,
                                 dmr_slot_t     slot);

/**
 * @brief Parse a Rate-1 Data Block.
 *
 * @param raw12      12-byte data block body
 * @param dbsn       Output: Data Block Serial Number
 * @param last_block Output: true if last-block flag is set
 * @param payload    Output: up to 11 bytes of user data
 * @param payload_len Output: number of user data bytes
 */
dmr_err_t llc_data_block_rate1_parse(const uint8_t *raw12,
                                      uint8_t  *dbsn,
                                      bool     *last_block,
                                      uint8_t  *payload,
                                      size_t   *payload_len);

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
                                 dmr_slot_t   slot);

/**
 * @brief Parse a C_RDATA selective-retry bitmap burst.
 *
 * @param raw12  12-byte C_RDATA body (post-FEC-decode)
 * @param flags  Output: 64-bit retry bitmap, same bit meaning as
 *               llc_data_block_rdata_build()'s flags parameter
 */
void llc_data_block_rdata_parse(const uint8_t *raw12, uint64_t *flags);

/* =========================================================================
 * Section 8 — Idle burst build
 * ETSI TS 102 361-1, Cl.7.3 / Data Type 0x09
 * ========================================================================= */

/**
 * @brief Build an Idle burst (fills with ETSI Annex D.2 pseudo-random bits).
 */
void llc_idle_burst_build(dmr_burst_t *burst,
                           uint8_t     cc,
                           bool        outbound,
                           dmr_slot_t  slot);
                           
                           
                           
                           
/**
 * Packs a 72-bit (9-byte) DMR Data Terminator PDU (TD_LC).
 *
 * @param pdu_out Output array of at least 9 bytes.
 * @param pf Protect Flag: true (1) if encrypted/protected, false (0) for clear.
 * @param target_llid 24-bit Destination/Target Logical Link ID.
 * @param source_llid 24-bit Source Logical Link ID.
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
                                  );


/* =========================================================================
 * Section 9 — RX dispatch result
 * ========================================================================= */

/**
 * @brief Decoded LLC result delivered to CCL after parsing an RX burst.
 */
typedef enum {
    LLC_RX_VOICE_LC_HDR    = 0,  /* Voice LC Header (Data Type 0x01)              */
    LLC_RX_TERMINATOR_LC   = 1,  /* Terminator with LC (Data Type 0x02)           */
    LLC_RX_CSBK            = 2,  /* Control Signalling Block (Data Type 0x03)     */
    LLC_RX_DATA_HEADER     = 3,  /* Data Header (Data Type 0x06)                  */
    LLC_RX_DATA_BLOCK      = 4,  /* Rate-1 data continuation                      */
    LLC_RX_PI_HEADER       = 5,  /* PI (Privacy Indicator) Header (Data Type 0x00)*/
    LLC_RX_IDLE            = 6,  /* Idle burst (Data Type 0x09)                   */
    LLC_RX_MBC_HEADER      = 7,  /* MBC header (Data Type 0x04)                   */
    LLC_RX_MBC_CONT        = 8,  /* MBC continuation (Data Type 0x05)             */
    LLC_RX_UNKNOWN         = 9,  /* Unrecognised Data Type                        */
} llc_rx_type_t;

typedef struct {
    llc_rx_type_t type;
    uint8_t       cc;         /* Colour code from SLOT_TYPE                        */
    uint8_t       dtype;      /* Raw Data Type nibble                              */
    uint8_t       body[12];   /* Raw 12-byte PDU body (post-BPTC, post-CRC-strip) */

    /* Parsed fields (valid depending on type) */
    uint8_t  opcode;          /* CSBK opcode or FLCO                               */
    uint32_t dst_id;          /* Destination ID                                    */
    uint32_t src_id;          /* Source ID                                         */
    uint8_t  svc;             /* Service Options                                   */
    uint8_t  sap;             /* SAP (data headers)                                */
    uint8_t  blocks_to_follow;
    bool     full_msg;        /* F-bit (Cl.9.3.20): true=complete message,
                                  false=partial/selective retransmission.
                                  Only meaningful for the DPFT_CONFIRMED/
                                  DPFT_UNCONFIRMED header family — see
                                  llc_rx_dispatch()'s LLC_RX_DATA_HEADER
                                  case; unset (memset default false) for
                                  every other type.                       */
    bool     crc_ok;          /* false = CRC failed (PDU still delivered)          */
} llc_rx_result_t;

/**
 * @brief Dispatch a received data burst to the correct LLC parser.
 *
 * Called by CCL or the burst processor after BPTC decoding.
 * Determines the Data Type, verifies CRC, fills llc_rx_result_t.
 *
 * @param burst   RX burst (BPTC already decoded, info-bits in raw[])
 * @param result  Output: parsed LLC result
 * @return DMR_OK always (crc_ok field indicates CRC result)
 */
dmr_err_t llc_rx_dispatch(const dmr_burst_t *burst, llc_rx_result_t *result);

/* =========================================================================
 * Section 10 — Tier III CSBK builders
 * ETSI TS 102 361-4, Clause 6
 * ========================================================================= */

/**
 * @brief Build a Tier III MS Registration CSBK (CSBKO=0x24).
 *        TS 102 361-4 Cl.6.7
 *
 * @param reason  Registration reason (0=normal, 1=migration, …)
 */
void llc_t3_ms_reg_build(dmr_burst_t *burst,
                           uint8_t  reason,
                           uint32_t dst_id,
                           uint32_t src_id,
                           uint8_t  cc,
                           dmr_slot_t slot);

/**
 * @brief Build a Tier III MS De-registration CSBK (CSBKO=0x27).
 */
void llc_t3_ms_dereg_build(dmr_burst_t *burst,
                             uint32_t dst_id,
                             uint32_t src_id,
                             uint8_t  cc,
                             dmr_slot_t slot);

/**
 * @brief Parse a Tier III Network Status Broadcast (CSBKO=0x14).
 *        TS 102 361-4 Cl.6.1
 *
 * @param raw12       12-byte CSBK body
 * @param net_id      Output: 24-bit Network ID (3 bytes)
 * @param site_id     Output: Site ID
 * @param ch_count    Output: number of traffic channels
 * @param req_access  Output: required access type
 */
dmr_err_t llc_t3_net_status_parse(const uint8_t *raw12,
                                    uint32_t *net_id,
                                    uint8_t  *site_id,
                                    uint8_t  *ch_count,
                                    uint8_t  *req_access);

/**
 * @brief Parse a Tier III Adjacent Site Info CSBK (CSBKO=0x19).
 *        TS 102 361-4 Cl.6.1
 *
 * @param raw12    12-byte CSBK body
 * @param area_id  Output: Area ID
 * @param sys_id   Output: System ID
 * @param site_id  Output: Site ID
 * @param ch_id    Output: Channel ID (traffic channel)
 */
dmr_err_t llc_t3_adj_site_parse(const uint8_t *raw12,
                                  uint8_t  *area_id,
                                  uint16_t *sys_id,
                                  uint8_t  *site_id,
                                  uint16_t *ch_id);

/**
 * @brief Parse a Tier III TV_GRANT (CSBKO=0x01) or TD_GRANT (0x03).
 *        TS 102 361-4 Cl.6.3
 *
 * @param raw12      12-byte CSBK body
 * @param ch_id      Output: assigned channel ID
 * @param slot       Output: assigned slot (1 or 2)
 * @param dst_id     Output: destination group/individual ID
 * @param emergency  Output: emergency grant flag
 */
dmr_err_t llc_t3_grant_parse(const uint8_t *raw12,
                               uint16_t *ch_id,
                               uint8_t  *slot,
                               uint32_t *dst_id,
                               bool     *emergency);

/**
 * @brief Build a Tier III TV_GRANT (CSBKO=0x01) or TD_GRANT (0x03) CSBK.
 *        TS 102 361-4 Cl.6.3 — TSCC→MS outbound, grants a traffic channel.
 *
 * @param is_data        false=TV_GRANT (voice), true=TD_GRANT (data)
 * @param emergency      Emergency grant flag
 * @param ch_id          Assigned channel ID
 * @param grant_slot     Assigned slot (1 or 2)
 * @param dst_id          Destination group/individual ID (the granted call)
 */
void llc_t3_grant_build(dmr_burst_t *burst,
                          bool     is_data,
                          bool     emergency,
                          uint16_t ch_id,
                          uint8_t  grant_slot,
                          uint32_t dst_id,
                          uint8_t  cc,
                          dmr_slot_t slot);

/**
 * @brief Build a Tier III Random Access Request CSBK (CSBKO=0x02, C_RAND).
 *        TS 102 361-4 Cl.6.2 — MS→TSCC inbound, requests a traffic channel
 *        grant for a voice or data service.
 *
 * Byte 2:   [7:5]=service_kind [4]=is_group [3:0]=reserved
 * Byte 3:   reserved (random-access slot sub-division — not modelled here)
 * Bytes 4-6: DST_ID (target group or individual)
 * Bytes 7-9: SRC_ID (requesting MS)
 *
 * @param service_kind  DMR_T3_SVC_VOICE or DMR_T3_SVC_DATA
 * @param is_group       true for group call/data, false for individual
 */
void llc_t3_rand_access_build(dmr_burst_t *burst,
                                uint8_t  service_kind,
                                bool     is_group,
                                uint32_t dst_id,
                                uint32_t src_id,
                                uint8_t  cc,
                                dmr_slot_t slot);

/**
 * @brief Parse a Tier III Random Access Request CSBK (CSBKO=0x02).
 */
dmr_err_t llc_t3_rand_access_parse(const uint8_t *raw12,
                                     uint8_t  *service_kind,
                                     bool     *is_group,
                                     uint32_t *dst_id,
                                     uint32_t *src_id);

/**
 * @brief Build a Tier III MS Registration Response CSBK (CSBKO=0x25).
 *        TS 102 361-4 Cl.6.7 — TSCC→MS, accepts/rejects a registration.
 *
 * Byte 2: [7:6]=reserved [5:4]=response [3:0]=reserved
 *         response: 0=accept, 1=refuse (channel info elsewhere), 2=fail
 */
void llc_t3_ms_reg_resp_build(dmr_burst_t *burst,
                                uint8_t  response,
                                uint32_t dst_id,
                                uint32_t src_id,
                                uint8_t  cc,
                                dmr_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* DMR_LLC_H */