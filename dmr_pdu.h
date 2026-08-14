#define DMR_CRC_MASK_DATA_HEADER   0xCCCCu  /* Data Header (DPF) */

#define DMR_CRC_MASK_MBC           0x3333u  /* Multi-Block Control */

/**
 * @file dmr_pdu.h
 * @brief ETSI TS 102 361-1/2/3/4 — DMR Air Interface PDU & Message Structures
 *
 * All on-air PDU structures use __attribute__((packed)) and explicit uint8_t
 * arrays — NO bitfields — eliminating all compiler-inserted padding.
 *
 * CRITICAL BURST BIT-LAYOUT (ETSI TS 102 361-1, Clause 4.2 + Annex E)
 * ======================================================================
 * Every burst is 264 bits = 132 dibits, stored in raw[0..32] (33 bytes).
 * Transmission order: L66 Bit1 → L66 Bit0 → L65 Bit1 → ... → R66 Bit0.
 *
 *   raw[0] b7 = dibit L66 Bit1  (FIRST bit transmitted)
 *   raw[0] b6 = dibit L66 Bit0
 *   raw[0] b5 = dibit L65 Bit1
 *   ...
 *   Symbol index n (0=L66, 65=L1, 66=R1, 131=R66):
 *     raw[n/4] bit (7 - 2*(n%4))   = Bit1
 *     raw[n/4] bit (7 - 2*(n%4)-1) = Bit0
 *
 * FIELD POSITIONS IN raw[] (derived from Annex E Tables E.1, E.5):
 *
 *   DATA BURST (Table E.1):
 *     INFO_1  [98 bits ] : raw[0..11] full + raw[12][7:2]      (symbols L66..L18)
 *     ST_HI   [10 bits ] : raw[12][1:0] + raw[13][7:4]         (symbols L17..L13)
 *       CC    [4 bits  ] : raw[12][1:0] gives CC[3:2],         (L17,L16 partial)
 *                          raw[12][5:4] = CC[3:2], [3:2]=CC[1:0]
 *       DT    [4 bits  ] : raw[12][1:0] = DT[3:2], raw[13][7:6]=DT[1:0]
 *       Golay_hi[2 bits]: raw[13][5:4]
 *     SYNC    [48 bits ] : raw[13][3:0] + raw[14..18] + raw[19][7:4]
 *     ST_LO   [10 bits ] : raw[19][3:0] + raw[20][7:2]         (symbols R13..R17)
 *     INFO_2  [98 bits ] : raw[20][1:0] + raw[21..32]          (symbols R18..R66)
 *
 *   Exact SLOT_TYPE symbol map (L17=sym49 .. L13=sym53):
 *     sym49 raw[12]b5,b4 : CC(3),CC(2)
 *     sym50 raw[12]b3,b2 : CC(1),CC(0)
 *     sym51 raw[12]b1,b0 : DT(3),DT(2)
 *     sym52 raw[13]b7,b6 : DT(1),DT(0)
 *     sym53 raw[13]b5,b4 : Golay(11),Golay(10)
 *
 *   Exact SYNC symbol map (L12=sym54 .. R12=sym77):
 *     sym54 raw[13]b3,b2 : Sync(47),Sync(46)
 *     sym55 raw[13]b1,b0 : Sync(45),Sync(44)
 *     sym56 raw[14]b7,b6 : Sync(43),Sync(42)
 *     sym57 raw[14]b5,b4 : Sync(41),Sync(40)
 *     sym58 raw[14]b3,b2 : Sync(39),Sync(38)
 *     sym59 raw[14]b1,b0 : Sync(37),Sync(36)
 *     sym60 raw[15]b7,b6 : Sync(35),Sync(34)
 *     sym61 raw[15]b5,b4 : Sync(33),Sync(32)
 *     sym62 raw[15]b3,b2 : Sync(31),Sync(30)
 *     sym63 raw[15]b1,b0 : Sync(29),Sync(28)
 *     sym64 raw[16]b7,b6 : Sync(27),Sync(26)
 *     sym65 raw[16]b5,b4 : Sync(25),Sync(24)
 *     sym66 raw[16]b3,b2 : Sync(23),Sync(22)
 *     sym67 raw[16]b1,b0 : Sync(21),Sync(20)
 *     sym68 raw[17]b7,b6 : Sync(19),Sync(18)
 *     sym69 raw[17]b5,b4 : Sync(17),Sync(16)
 *     sym70 raw[17]b3,b2 : Sync(15),Sync(14)
 *     sym71 raw[17]b1,b0 : Sync(13),Sync(12)
 *     sym72 raw[18]b7,b6 : Sync(11),Sync(10)
 *     sym73 raw[18]b5,b4 : Sync(9), Sync(8)
 *     sym74 raw[18]b3,b2 : Sync(7), Sync(6)
 *     sym75 raw[18]b1,b0 : Sync(5), Sync(4)
 *     sym76 raw[19]b7,b6 : Sync(3), Sync(2)
 *     sym77 raw[19]b5,b4 : Sync(1), Sync(0)
 *
 *   VOICE BURST (burst A, Table E.5):
 *     INFO_1 [108 bits]: raw[0..12] full + raw[13][7:4]   (VS(215)..VS(108))
 *     SYNC   [48  bits]: raw[13][3:0] + raw[14..18] + raw[19][7:4]
 *     INFO_2 [108 bits]: raw[19][3:0] + raw[20..32]       (VS(107)..VS(0))
 *
 *   VOICE BURST (bursts B-F, EMB, Tables E.6-E.9):
 *     INFO_1 [108 bits]: raw[0..12] full + raw[13][7:4]   (VS(215)..VS(108))
 *     EMB_ctrl [8 bits]: raw[13][3:0] + raw[14][7:4]
 *       CC[3:2] = raw[13][3:2]
 *       CC[1:0] = raw[13][1:0]
 *       PI      = raw[14][7]
 *       LCSS[1] = raw[14][6]
 *       LCSS[0] = raw[14][5]
 *       QR[8]   = raw[14][4]
 *     EMB_LC  [32 bits]: raw[14][3:0] + raw[15..18] + raw[19][7:4]
 *       (embedded LC fragment or RC payload + QR[7:0])
 *     INFO_2 [108 bits]: raw[19][3:0] + raw[20..32]       (VS(107)..VS(0))
 *
 *   RC BURST (Table E.12):
 *     EMB_ctrl [8 bits]: raw[0][7:4] + raw[0][3:0] — same EMB layout at L24..L21
 *     RC_sig  [32 bits]: BPTC-encoded RC signalling (RC info + FEC)
 *     RC_SYNC [48 bits]: raw[13][3:0] + raw[14..18] + raw[19][7:4]  (R_Sync pattern)
 *     RC_sig2 [32 bits]: post-sync RC continuation
 *     EMB_ctrl2[8 bits]: mirrored at R24..R21
 *   NOTE: RC burst is 96 bits (48 symbols L24..R24 only)
 *
 * Standards:
 *   ETSI TS 102 361-1 V2.6.1 (2023-05) — Air Interface Protocol
 *   ETSI TS 102 361-2 V2.5.1 (2023-05) — Voice & Generic Services
 *   ETSI TS 102 361-3 V1.3.1 (2017-10) — Data Protocol
 *   ETSI TS 102 361-4 V1.12.1(2023-07) — Trunking Protocol
 */

#ifndef DMR_PDU_H
#define DMR_PDU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* =========================================================================
 * Pack attribute — required for all on-air PDU structures
 * ========================================================================= */
#define DMR_PACKED  __attribute__((packed))

/* Compile-time size assertion helper */
#define DMR_STATIC_ASSERT(cond, name) \
    typedef char _dmr_static_assert_##name[(cond) ? 1 : -1]

/* =========================================================================
 * SECTION 1 — PHYSICAL LAYER CONSTANTS
 * ETSI TS 102 361-1, Clauses 4.2, 10.2
 * ========================================================================= */

/* Burst sizes */
#define DMR_BURST_BITS          264     /* Traffic burst: 264 bits                */
#define DMR_BURST_BYTES          33     /* 264 bits packed into 33 bytes          */
#define DMR_CACH_BITS            24     /* CACH burst: 24 bits                    */
#define DMR_CACH_BYTES            3
#define DMR_RC_BITS              96     /* Standalone RC burst: 96 bits           */
#define DMR_RC_BYTES             12

/* Symbol / timing */
#define DMR_SYMBOL_RATE        4800     /* Baud (symbols/sec)                     */
#define DMR_BIT_RATE           9600     /* Gross bit rate (2 bits/symbol)         */
#define DMR_TIMESLOT_MS          30     /* One TDMA slot = 30 ms                  */
#define DMR_FRAME_MS             60     /* One TDMA frame (2 slots) = 60 ms       */
#define DMR_SUPERFRAME_MS       360     /* Voice superframe (6 bursts) = 360 ms   */
#define DMR_CHANNEL_KHZ        12500    /* RF channel spacing (12.5 kHz)          */

/*
 * SYNC pattern constants — ETSI TS 102 361-1, Table 9.10
 * 48-bit patterns stored as uint64_t (upper 16 bits always 0).
 * Bit 47 is transmitted FIRST (maps to raw[13] bit 3 in the burst).
 */
#define DMR_SYNC_BS_VOICE        UINT64_C(0x755FD7DF75F7)
#define DMR_SYNC_BS_DATA         UINT64_C(0xDFF57D75DF5D)
#define DMR_SYNC_MS_VOICE        UINT64_C(0x7F7D5DD57DFD)
#define DMR_SYNC_MS_DATA         UINT64_C(0xD5D7F77FD757)
#define DMR_SYNC_DIRECT_VOICE    UINT64_C(0x5D577F7757FF)
#define DMR_SYNC_DIRECT_DATA     UINT64_C(0xF7FDD5DDFD55)
#define DMR_SYNC_RC              UINT64_C(0x77D55F7DFD77)
#define DMR_SYNC_IDLE            UINT64_C(0xD7FDD57DDFFD)


/* ETSI TS 102 361-1 16-Bit CRC Masks */
#define DMR_CRC_MASK_DATA_HEADER   0xCCCCu  /* Data Header (DPF) */
#define DMR_CRC_MASK_VOICE_HEADER  0x9999u  /* Voice Header / Full LC */
#define DMR_CRC_MASK_CSBK          0xA5A5u  /* Control Slot Block */
#define DMR_CRC_MASK_MBC           0x3333u  /* Multi-Block Control */
#define DMR_CRC_MASK_NONE          0x0000u  /* No Mask */
/*
 * Data Types (4-bit field within SLOT_TYPE)
 * ETSI TS 102 361-1, Table 9.34
 */
#define DMR_DTYPE_PI_HEADER         0x0u
#define DMR_DTYPE_VOICE_LC_HEADER   0x1u
#define DMR_DTYPE_TERMINATOR_LC     0x2u
#define DMR_DTYPE_CSBK              0x3u
#define DMR_DTYPE_MBC_HEADER        0x4u
#define DMR_DTYPE_MBC_CONT          0x5u
#define DMR_DTYPE_DATA_HEADER       0x6u
#define DMR_DTYPE_RATE12_DATA       0x7u
#define DMR_DTYPE_RATE34_DATA       0x8u
#define DMR_DTYPE_IDLE              0x9u
#define DMR_DTYPE_RATE1_DATA        0xAu
#define DMR_DTYPE_UNIFIED_SINGLE    0xBu

/* Full Link Control Opcodes (6-bit FLCO field)
 * ETSI TS 102 361-2, Annex B.1 */
#define DMR_FLCO_GRP_V_CH_USR       0x00u   /* Group Voice Channel User          */
#define DMR_FLCO_IND_V_CH_USR       0x03u   /* Individual Voice Channel User     */
#define DMR_FLCO_GPS_INFO            0x08u   /* GPS / Location Info               */
#define DMR_FLCO_EMERG_VOICE_USR    0x10u   /* Emergency Voice (proprietary ext) */
#define DMR_FLCO_DATA_TERMINATOR    0x30u   /* DMR FLCO for Data Terminator LC (TD_LC) is hardcoded to 0x30 (110000 in binary)*/

/* CSBK Opcodes (6-bit CSBKO field)
 * ETSI TS 102 361-2, Annex B.2 / TS 102 361-4, Table 6.8 */
#define DMR_CSBKO_UU_V_REQ           0x04u
#define DMR_CSBKO_UU_ANS_RSP         0x24u
#define DMR_CSBKO_CALL_ALERT         0x1Fu
#define DMR_CSBKO_ACK_RSP            0x20u
#define DMR_CSBKO_CANCEL_CALL        0x23u
#define DMR_CSBKO_EMERG_ALARM_ACK    0x27u
#define DMR_CSBKO_BS_DWNA            0x28u
#define DMR_CSBKO_CHANNEL_TIMING     0x07u   /* CT_CSBK, TS 102 361-2 Annex B.2 (was 0x22 — corrected) */
#define DMR_CSBKO_PREAMBLE           0x3Du
/* Tier III (TS 102 361-4) */
#define DMR_CSBKO_T3_TV_GRANT        0x01u
#define DMR_CSBKO_T3_TD_GRANT        0x03u
#define DMR_CSBKO_T3_NET_STATUS      0x14u
#define DMR_CSBKO_T3_ADJ_SITE        0x19u
#define DMR_CSBKO_T3_MS_REGIST       0x24u
#define DMR_CSBKO_T3_MS_REGIST_RSP   0x25u
#define DMR_CSBKO_T3_MS_DEREGIST     0x27u
#define DMR_CSBKO_T3_EMERG_ALARM     0x28u
#define DMR_CSBKO_T3_RAND_ACCESS     0x02u   /**< Random Access Request (C_RAND), MS→TSCC, Cl.6.2 */

/* Tier III Random Access — requested service kind (body[2] bits[7:5]) */
#define DMR_T3_SVC_VOICE             0x00u
#define DMR_T3_SVC_DATA              0x01u
#define DMR_T3_SVC_SUPPLEMENTARY     0x02u
#define DMR_T3_SVC_REGISTRATION      0x03u

/* Encryption Algorithm IDs — ETSI TS 102 361-1, Clause 9.1.5 */
#define DMR_ALG_NONE                 0x00u
#define DMR_ALG_DES_OFB              0x01u
#define DMR_ALG_DES_XL               0x09u
#define DMR_ALG_AES128_OFB           0x21u
#define DMR_ALG_AES256_OFB           0x22u
#define DMR_ALG_ARC4                 0x25u

/* SAP Identifiers — ETSI TS 102 361-1, Table 9.60 */
#define DMR_SAP_UDT                  0x00u  /* Unified Data Transport            */
#define DMR_SAP_TCP_IP_HDRCOMP       0x02u  /* TCP/IP header compression         */
#define DMR_SAP_UDP_IP_HDRCOMP       0x03u  /* UDP/IP header compression         */
#define DMR_SAP_IP_PACKET            0x04u  /* IP based Packet data              */
#define DMR_SAP_ARP                  0x05u  /* Address Resolution Protocol (ARP) */
#define DMR_SAP_PROPRIETARY          0x09u  /* Proprietary Packet data           */
#define DMR_SAP_SHORT_DATA           0x0Au  /* Short Data                        */
/* All values per TS 102 361-1 Table 9.31 (Cl.9.3.18) — verified directly
 * against the canonical table, not assumed; the previous values here
 * (SHORT_DATA=0x00, UDP_IP_HDRCOMP=0x02, ARP=0x03) were wrong and
 * predate this session. */

/* Data Packet Format Types (DPFT) — ETSI TS 102 361-1, Table 9.49 */
#define DMR_DPFT_UDT                 0x00u
#define DMR_DPFT_RESPONSE            0x01u
#define DMR_DPFT_UNCONFIRMED         0x02u
#define DMR_DPFT_CONFIRMED           0x03u
#define DMR_DPFT_PROPRIETARY         0x09u
#define DMR_DPFT_DEFINED_DATA        0x0Du  /* DD_HEAD — TS 102 361-1 Table 9.17C, TS 102 361-3 Cl.6.1 */
#define DMR_DPFT_RAW_OR_STATUS       0x0Eu  /* R_HEAD/SP_HEAD — share this DPFT; SP_HEAD always has
                                             * AB==0 (Table 9.17A "shall be set to 0"), R_HEAD
                                             * normally has AB!=0 (it carries data blocks) — the two
                                             * are distinguished by AB, not by DPFT. TS 102 361-1
                                             * Tables 9.17A/9.17B, TS 102 361-3 Cl.6.2/6.3. */

/* LCSS values — ETSI TS 102 361-1, Table 9.20 */
#define DMR_LCSS_SINGLE              0x00u   /* Single fragment (complete LC)     */
#define DMR_LCSS_FIRST               0x01u   /* First fragment of multi-frag LC   */
#define DMR_LCSS_LAST                0x02u   /* Last fragment                     */
#define DMR_LCSS_CONT                0x03u   /* Continuation fragment             */

/* UU_ANS_RSP response codes */
#define DMR_UU_ANS_PROCEED           0x00u
#define DMR_UU_ANS_DENY              0x01u
#define DMR_UU_ANS_WAIT              0x02u
#define DMR_UU_ANS_QUEUED            0x03u

/* AMBE+2 vocoder frame */
#define DMR_AMBE_FRAME_BYTES         9u
#define DMR_AMBE_FRAME_BITS          72u
#define DMR_SUPERFRAME_BURSTS         6u

/* =========================================================================
 * SECTION 2 — BURST FIELD BIT POSITIONS (raw[] index)
 *
 * These constants document exactly which raw[] bytes carry each field.
 * The SYNC field straddles raw[13..19] — it is NOT byte-aligned.
 * ========================================================================= */

/*
 * SYNC field within raw[0..32] (applies to both voice and data bursts):
 *
 *   raw[13] bits [3:0]  → Sync[47:44]  (4 bits, lower nibble of byte 13)
 *   raw[14] bits [7:0]  → Sync[43:36]  (full byte 14)
 *   raw[15] bits [7:0]  → Sync[35:28]  (full byte 15)
 *   raw[16] bits [7:0]  → Sync[27:20]  (full byte 16)
 *   raw[17] bits [7:0]  → Sync[19:12]  (full byte 17)
 *   raw[18] bits [7:0]  → Sync[11:4]   (full byte 18)
 *   raw[19] bits [7:4]  → Sync[3:0]    (4 bits, upper nibble of byte 19)
 *
 * Sync[47] is the MSB (transmitted FIRST within the sync field).
 */
#define DMR_SYNC_BYTE_LO    13u   /* raw[13] lower nibble holds Sync[47:44]        */
#define DMR_SYNC_BYTE_MID0  14u   /* raw[14] full byte holds Sync[43:36]           */
#define DMR_SYNC_BYTE_MID1  15u
#define DMR_SYNC_BYTE_MID2  16u
#define DMR_SYNC_BYTE_MID3  17u
#define DMR_SYNC_BYTE_MID4  18u
#define DMR_SYNC_BYTE_HI    19u   /* raw[19] upper nibble holds Sync[3:0]          */

/*
 * SLOT_TYPE field within raw[] (data bursts only):
 *
 *   High half (symbols L17..L13 = dibits 49..53):
 *     raw[12] bits [5:4] → CC[3:2]
 *     raw[12] bits [3:2] → CC[1:0]
 *     raw[12] bits [1:0] → DT[3:2]
 *     raw[13] bits [7:6] → DT[1:0]
 *     raw[13] bits [5:4] → Golay[11:10]
 *
 *   Low half (symbols R13..R17 = dibits 78..82):
 *     raw[19] bits [3:2] → Golay[9:8]
 *     raw[19] bits [1:0] → Golay[7:6]
 *     raw[20] bits [7:6] → Golay[5:4]
 *     raw[20] bits [5:4] → Golay[3:2]
 *     raw[20] bits [3:2] → Golay[1:0]
 */
#define DMR_ST_BYTE_CC_DT   12u   /* raw[12]: CC[3:0] in [5:2], DT[3:2] in [1:0]  */
#define DMR_ST_BYTE_DT_GL   13u   /* raw[13]: DT[1:0] in [7:6], Golay_hi in [5:4] */
#define DMR_ST_BYTE_GL_LO   19u   /* raw[19]: Golay[9:6] in [3:0]                 */
#define DMR_ST_BYTE_GL_END  20u   /* raw[20]: Golay[5:0] in [7:2]                 */

/*
 * VOICE INFO field boundaries:
 *   INFO_1: raw[0..12] (all 8 bits) + raw[13] bits [7:4]  = 104+4 = 108 bits
 *   INFO_2: raw[19] bits [3:0] + raw[20..32] (all 8 bits) = 4+104 = 108 bits
 *
 * EMB ctrl field (bursts B-F):
 *   raw[13] bits [3:0] = CC[3:0]
 *   raw[14] bits [7:4] = PI, LCSS[1:0], QR[8]
 *
 * EMB LC payload (embedded LC/RC):
 *   raw[14] bits [3:0] + raw[15..18] + raw[19] bits [7:4]
 *   = 4 + 32 + 4 = 40 bits  (32 bits LC fragment + 8 bits QR remainder)
 */
#define DMR_INFO1_FULL_BYTES  13u /* raw[0..12]: 104 bits of INFO_1               */
#define DMR_INFO1_PARTIAL     13u /* raw[13] upper nibble: 4 more INFO_1 bits      */
#define DMR_INFO2_PARTIAL     19u /* raw[19] lower nibble: 4 first INFO_2 bits     */
#define DMR_INFO2_FULL_START  20u /* raw[20..32]: 104 bits of INFO_2               */


/* =========================================================================
 * SECTION 3 — PHYSICAL LAYER PDU STRUCTURES
 * ========================================================================= */

/**
 * @brief SYNC PDU — 48 bits (6 bytes)
 *        ETSI TS 102 361-1, Table 9.10
 *        Bit 47 (Sync[47]) is transmitted first.
 *        NOTE: In raw[], this is NOT byte-aligned — use dmr_burst_set_sync() /
 *              dmr_burst_get_sync() to read/write this field correctly.
 */
typedef struct DMR_PACKED {
    uint8_t sync[6];    /* sync[0] bit7 = Sync[47] (first transmitted)            */
} dmr_sync_pdu_t;
DMR_STATIC_ASSERT(sizeof(dmr_sync_pdu_t) == 6, sync_pdu);


/**
 * @brief SLOT_TYPE PDU — 20 bits packed into 3 bytes
 *        ETSI TS 102 361-1, Table 9.34
 *
 * Logical layout (before interleaving into burst):
 *   Byte 0: [7:4]=CC  [3:0]=DataType
 *   Byte 1: [7:0]=Golay(18,6) FEC bits [11:4]
 *   Byte 2: [7:4]=Golay FEC bits [3:0]   [3:0]=0 (unused)
 *
 * NOTE: In raw[] these 20 bits are split across raw[12][5:0] + raw[13][7:4]
 *       (hi half) and raw[19][3:0] + raw[20][7:2] (lo half).
 *       Use dmr_burst_get_slot_type() / dmr_burst_set_slot_type() which
 *       perform correct non-byte-aligned extraction/insertion.
 */
typedef struct DMR_PACKED {
    uint8_t cc_dtype;       /* [7:4]=CC  [3:0]=DataType                            */
    uint8_t golay_hi;       /* Golay(18,6) FEC bits [11:4]                         */
    uint8_t golay_lo_pad;   /* [7:4]=Golay FEC [3:0]  [3:0]=padding/0             */
} dmr_slot_type_t;
DMR_STATIC_ASSERT(sizeof(dmr_slot_type_t) == 3, slot_type);

/* SLOT_TYPE logical field accessors */
#define DMR_ST_GET_CC(st)     (((st)->cc_dtype >> 4) & 0x0Fu)
#define DMR_ST_GET_DTYPE(st)  (((st)->cc_dtype      ) & 0x0Fu)
#define DMR_ST_SET_CC(st,v)   ((st)->cc_dtype = (uint8_t)(((st)->cc_dtype & 0x0Fu) | (((v)&0x0Fu)<<4)))
#define DMR_ST_SET_DTYPE(st,v)((st)->cc_dtype = (uint8_t)(((st)->cc_dtype & 0xF0u) | ((v)&0x0Fu)))


/**
 * @brief EMB (Embedded Signalling) PDU — 16 bits (2 bytes)
 *        ETSI TS 102 361-1, Table 9.18
 *
 * Logical layout:
 *   Byte 0: [7:4]=CC  [3]=PI  [2:1]=LCSS  [0]=QR[8]
 *   Byte 1: [7:0]=QR[7:0]
 *
 * NOTE: In raw[] (voice bursts B-F), this is NOT byte-aligned.
 *       The CC field occupies raw[13][3:0] and PI+LCSS+QR[8] occupy raw[14][7:4].
 *       Use dmr_burst_get_emb() / dmr_burst_set_emb() for correct access.
 */
typedef struct DMR_PACKED {
    uint8_t ctrl;       /* [7:4]=CC [3]=PI [2:1]=LCSS [0]=QR_FEC[8]              */
    uint8_t qr_lo;      /* QR_FEC[7:0]                                             */
} dmr_emb_pdu_t;
DMR_STATIC_ASSERT(sizeof(dmr_emb_pdu_t) == 2, emb_pdu);

/* EMB logical field accessors (operate on dmr_emb_pdu_t) */
#define DMR_EMB_GET_CC(e)    (((e)->ctrl >> 4) & 0x0Fu)
#define DMR_EMB_GET_PI(e)    (((e)->ctrl >> 3) & 0x01u)
#define DMR_EMB_GET_LCSS(e)  (((e)->ctrl >> 1) & 0x03u)
#define DMR_EMB_GET_QR8(e)   (((e)->ctrl      ) & 0x01u)
#define DMR_EMB_SET_CC(e,v)  ((e)->ctrl = (uint8_t)(((e)->ctrl & 0x0Fu) | (((v)&0x0Fu)<<4)))
#define DMR_EMB_SET_PI(e,v)  ((e)->ctrl = (uint8_t)(((e)->ctrl & ~0x08u)| (((v)&1u)<<3)))
#define DMR_EMB_SET_LCSS(e,v)((e)->ctrl = (uint8_t)(((e)->ctrl & ~0x06u)| (((v)&3u)<<1)))
#define DMR_EMB_SET_QR8(e,v) ((e)->ctrl = (uint8_t)(((e)->ctrl & ~0x01u)| ((v)&1u)))


/**
 * @brief CACH PDU — 24 bits (3 bytes)
 *        ETSI TS 102 361-1, Table 9.5 / Annex B.4
 *
 * Logical (pre-interleave) layout:
 *   Byte 0: [7]=AT  [6]=TC  [5:4]=LCSS  [3:0]=Golay(7,4) FEC [3:0]
 *   Byte 1: [7:0]=Short Data payload (8 bits)
 *   Byte 2: [7:3]=Short Data [4:0] (if extended) else 0
 *
 * After the CACH interleaver (Annex B.4) the bits are rearranged:
 *   TX(23)=AT, TX(22)=TC, TX(19)=LCSS(1), TX(12)=LCSS(0), TX(9)=H(2), ...
 * The structure below represents the LOGICAL (pre-interleave) form.
 * Use dmr_cach_interleave() / dmr_cach_deinterleave() for wire format.
 */
typedef struct DMR_PACKED {
    uint8_t at_tc_lcss_fec; /* [7]=AT [6]=TC [5:4]=LCSS [3:0]=Golay_FEC[3:0]     */
    uint8_t sd;              /* Short Data payload                                  */
    uint8_t golay_hi;        /* Golay(7,4) FEC [6:4] in bits [6:4], rest=0         */
} dmr_cach_pdu_t;
DMR_STATIC_ASSERT(sizeof(dmr_cach_pdu_t) == 3, cach_pdu);

/* CACH logical field accessors */
#define DMR_CACH_GET_AT(c)   (((c)->at_tc_lcss_fec >> 7) & 0x01u)
#define DMR_CACH_GET_TC(c)   (((c)->at_tc_lcss_fec >> 6) & 0x01u)
#define DMR_CACH_GET_LCSS(c) (((c)->at_tc_lcss_fec >> 4) & 0x03u)
#define DMR_CACH_SET_AT(c,v)   ((c)->at_tc_lcss_fec = \
    (uint8_t)(((c)->at_tc_lcss_fec & 0x7Fu) | (((v)&1u)<<7)))
#define DMR_CACH_SET_TC(c,v)   ((c)->at_tc_lcss_fec = \
    (uint8_t)(((c)->at_tc_lcss_fec & 0xBFu) | (((v)&1u)<<6)))
#define DMR_CACH_SET_LCSS(c,v) ((c)->at_tc_lcss_fec = \
    (uint8_t)(((c)->at_tc_lcss_fec & 0xCFu) | (((v)&3u)<<4)))


/**
 * @brief RC (Reverse Channel) PDU — 96 bits (12 bytes)
 *        ETSI TS 102 361-1, Table 9.32 / Annex B.2.2.2 / Clause 6.4
 *
 * A standalone RC burst is only 96 bits (symbols L24..R24).
 * Layout (logical, before Single-Burst RC BPTC FEC + interleaving):
 *   Byte 0:  EMB ctrl [7:0] (CC, PI, LCSS, QR8)    — at symbols L24..L21
 *   Bytes 1-5: RC signal + BPTC FEC (40 bits)
 *   Bytes 6-11: RC_SYNC pattern (48 bits)           — at symbols L12..R12
 *   ... second half mirrors first half
 *
 * NOTE: The on-air RC SYNC pattern (R_Sync) occupies the same relative
 *       position as the SYNC field in traffic bursts.
 *       In the 12-byte raw_rc[0..11] array the R_SYNC is at raw_rc[3][3:0]
 *       through raw_rc[9][7:4] — same nibble-straddle layout as traffic bursts.
 */
typedef struct DMR_PACKED {
    uint8_t emb_pre[2];    /* EMB ctrl bytes (CC+PI+LCSS+QR) at symbols L24..L21  */
    uint8_t rc_sig_pre[4]; /* RC signalling first half (pre-sync)                  */
    uint8_t rc_sync[6];    /* R_Sync 48-bit pattern                               */
    /* NOTE: total = 12 bytes but burst is only 96 bits = 12 bytes, correct      */
} dmr_rc_pdu_t;
DMR_STATIC_ASSERT(sizeof(dmr_rc_pdu_t) == 12, rc_pdu);


/* =========================================================================
 * SECTION 4 — BURST RAW CONTAINERS
 * ========================================================================= */

/**
 * @brief Traffic burst raw container — 264 bits (33 bytes)
 *        Applies to: voice bursts, data bursts, CSBK bursts, idle bursts.
 *
 * Use the field-access functions below to read/write each logical field.
 * Do NOT access raw[] bytes directly without accounting for the non-aligned
 * SYNC and SLOT_TYPE fields.
 */
typedef struct DMR_PACKED {
    uint8_t raw[DMR_BURST_BYTES];
} dmr_data_burst_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_burst_t) == 33, data_burst);


/**
 * @brief Burst type tag (runtime use only, not transmitted)
 */
typedef enum {
    DMR_BURST_TYPE_VOICE  = 0,
    DMR_BURST_TYPE_DATA   = 1,
    DMR_BURST_TYPE_CACH   = 2,
    DMR_BURST_TYPE_RC     = 3,
    DMR_BURST_TYPE_IDLE   = 4,
    DMR_BURST_TYPE_UNKNOWN= 5,
    /**
     * Synthetic control burst injected by MAC into mq_rx_voice to convey
     * meta-events that have no wire representation — missed voice burst
     * positions, call-end reasons, CACH-signalled channel idle, etc.
     * raw[] is zeroed. Read synth_event and synth_pos; ignore raw[].
     */
    DMR_BURST_TYPE_SYNTHETIC_EVT = 6,
} dmr_burst_type_e;

/**
 * @brief Synthetic event codes carried in dmr_burst_t.synth_event when
 *        dmr_burst_t.type == DMR_BURST_TYPE_SYNTHETIC_EVT.
 *
 * MAC posts these through mq_rx_voice (not a separate queue) so CCL
 * Voice's existing worker-loop epoll needs only a single new branch on
 * burst.type before dispatching to the appropriate handler.
 */
typedef enum {
    MAC_SYNTH_EVT_NONE                  = 0,

    /**
     * A voice superframe burst B-F was not received within its 80 ms
     * window. synth_pos holds the missed burst position (1=B .. 5=F).
     * CCL Voice should insert a PLC / comfort-noise frame for that slot.
     */
    MAC_SYNTH_EVT_VOICE_BURST_LOST      = 1,

    /**
     * Terminator with LC received — explicit, clean call end.
     * The Terminator burst was already delivered as a real
     * LLC_RX_TERMINATOR_LC burst; this event signals MAC-level cleanup
     * is complete and CCL Voice should tear down call state.
     */
    MAC_SYNTH_EVT_CALL_ENDED_TERMINATOR = 2,

    /**
     * CACH AT bit transitioned Busy→Idle while a voice call was active.
     * Only fired on Tier II / Tier III — never for Tier I DMO (no CACH).
     * Means the BS hang-time has expired; the inbound channel is free.
     * CCL Voice should treat this as a normal call end.
     */
    MAC_SYNTH_EVT_CALL_ENDED_CACH_IDLE  = 3,

    /**
     * The 400 ms superframe watchdog expired with no new burst A and no
     * Terminator. The peer likely dropped without completing its
     * transmission. CCL Voice should tear down call state and mark the
     * end as abnormal (no LC teardown information available).
     */
    MAC_SYNTH_EVT_CALL_ENDED_TIMEOUT    = 4,

    /**
     * The hangover window (one superframe = 400 ms) after
     * CALL_ENDED_TIMEOUT expired without a new burst A resuming the
     * call. Definitive end — CCL Voice must complete any cleanup it
     * deferred while waiting for possible resumption.
     */
    MAC_SYNTH_EVT_CALL_GONE             = 5,
} mac_synth_event_t;

/**
 * @brief Generic burst container with runtime metadata — NOT transmitted.
 *        Explicit padding ensures natural alignment without compiler surprises.
 *
 * When type == DMR_BURST_TYPE_SYNTHETIC_EVT:
 *   synth_event  identifies the meta-event (mac_synth_event_t above)
 *   synth_pos    carries auxiliary data — e.g. the missed burst position
 *                (0=A .. 5=F) for MAC_SYNTH_EVT_VOICE_BURST_LOST
 *   raw[]        is zeroed and MUST NOT be read
 *
 * synth_event and synth_pos occupy what was previously _pad[3]; they are
 * zero for all non-SYNTHETIC_EVT burst types so existing code that ignores
 * them is unaffected. sizeof(dmr_burst_t) remains exactly 44 bytes.
 */
typedef struct {
    dmr_burst_type_e type;   /* 4 bytes (enum = int)                                  */
    uint8_t  timeslot;       /* TDMA slot: 1 or 2                                     */
    uint8_t  synth_event;    /* mac_synth_event_t — valid iff type==SYNTHETIC_EVT     */
    uint8_t  synth_pos;      /* Auxiliary payload for the synthetic event (burst pos) */
    uint8_t  is_direct_mode; /* 1=DMO (no base station)                 (1 byte)     */
    uint8_t  raw[33];        /* 264-bit burst content (zeroed for SYNTHETIC_EVT)      */
    uint8_t  _pad2[3];       /* pad → total = 4+1+1+1+1+33+3 = 44 bytes              */
} dmr_burst_t;
DMR_STATIC_ASSERT(sizeof(dmr_burst_t) == 44, burst_t);



/**
 * @brief Voice superframe — 6 consecutive bursts (A..F), 360 ms
 *        ETSI TS 102 361-1, Clause 4.3 / 5.1.2.1
 */
typedef struct {
    bool missed_frame;
    uint8_t index; // array index to store next audio packet
    uint8_t data[DMR_AMBE_FRAME_BYTES*3*DMR_SUPERFRAME_BURSTS]; //store superframe audio data
} dmr_voice_superframe_t;
DMR_STATIC_ASSERT(sizeof(dmr_voice_superframe_t) == 164, dmr_voice_superframe_t);

/**
 * @brief AMBE+2 voice frame — 72 bits (9 bytes)
 *        ETSI TS 102 361-1, Clause 4.3 / MOD-09
 */
typedef struct DMR_PACKED {
    uint8_t data[DMR_AMBE_FRAME_BYTES];
} dmr_ambe_frame_t;
DMR_STATIC_ASSERT(sizeof(dmr_ambe_frame_t) == 9, ambe_frame);







/* =========================================================================
 * SECTION 5 — SERVICE OPTIONS (uint8_t with bit-level macros)
 *             ETSI TS 102 361-2, Table 7.11
 *
 * Bit layout (MSB first):
 *   [7]=Emergency  [6]=Privacy  [5]=Reserved  [4]=Reserved
 *   [3]=Broadcast  [2]=OVCM    [1:0]=Priority (0-3, 3=highest)
 * ========================================================================= */
typedef uint8_t dmr_svc_opts_t;

#define DMR_SVC_GET_EMERGENCY(s)  (((s) >> 7) & 0x01u)
#define DMR_SVC_GET_PRIVACY(s)    (((s) >> 6) & 0x01u)
#define DMR_SVC_GET_BROADCAST(s)  (((s) >> 3) & 0x01u)
#define DMR_SVC_GET_OVCM(s)       (((s) >> 2) & 0x01u)
#define DMR_SVC_GET_PRIORITY(s)   (((s)      ) & 0x03u)
#define DMR_SVC_SET_EMERGENCY(s,v)((s) = (uint8_t)(((s) & 0x7Fu) | (((v)&1u)<<7)))
#define DMR_SVC_SET_PRIVACY(s,v)  ((s) = (uint8_t)(((s) & 0xBFu) | (((v)&1u)<<6)))
#define DMR_SVC_SET_BROADCAST(s,v)((s) = (uint8_t)(((s) & 0xF7u) | (((v)&1u)<<3)))
#define DMR_SVC_SET_OVCM(s,v)     ((s) = (uint8_t)(((s) & 0xFBu) | (((v)&1u)<<2)))
#define DMR_SVC_SET_PRIORITY(s,v) ((s) = (uint8_t)(((s) & 0xFCu) | ((v)&0x03u)))

/* Legacy wrapper for API compatibility */
typedef struct DMR_PACKED { uint8_t raw; } dmr_service_options_t;
DMR_STATIC_ASSERT(sizeof(dmr_service_options_t) == 1, svc_opts);


/* =========================================================================
 * SECTION 6 — FULL LINK CONTROL PDU — 12 bytes
 *             ETSI TS 102 361-1, Clause 9.1.6
 *
 * Byte 0:    [7:2]=FLCO  [1]=FID_present  [0]=Reserved/0
 * Byte 1:    FID (Feature ID, 0x00=ETSI Standard)
 * Byte 2:    Service Options (dmr_svc_opts_t)
 * Bytes 3-5: Destination ID (24 bits, big-endian MSB first)
 * Bytes 6-8: Source ID (24 bits, big-endian MSB first)
 * Bytes 9-11: RS(12,9) FEC parity bytes (filled by FEC encoder)
 * ========================================================================= */
typedef struct DMR_PACKED {
    uint8_t  flco;       /* [7:2]=opcode [1]=FID_present [0]=reserved             */
    uint8_t  fid;        /* Feature ID (0x00=ETSI SFID)                            */
    uint8_t  svc;        /* Service Options — use DMR_SVC_* macros                 */
    uint8_t  dst_id[3];  /* Destination ID (MSB first)                             */
    uint8_t  src_id[3];  /* Source ID (MSB first)                                  */
    uint8_t  rs_fec[3];  /* RS(12,9) parity — computed by FEC encoder              */
} dmr_full_lc_t;
DMR_STATIC_ASSERT(sizeof(dmr_full_lc_t) == 12, full_lc);

/* Typedef aliases per ETSI terminology */
typedef dmr_full_lc_t  dmr_voice_lc_header_t;
typedef dmr_full_lc_t  dmr_terminator_lc_t;


/* =========================================================================
 * SECTION 7 — CSBK PDUs — all exactly 12 bytes
 *             ETSI TS 102 361-1, Clause 9.1.7 / TS 102 361-2, Clause 7.1.2
 *
 * Every CSBK (single block) is 96 bits:
 *   Byte  0: [7]=LB  [6]=PF  [5:0]=CSBKO
 *   Byte  1: FID
 *   Bytes 2-9: opcode-specific data (64 bits)
 *   Bytes 10-11: CRC-CCITT
 *
 * Multi-Block Control (MBC) uses the same first-byte format with LB=0.
 * ========================================================================= */

/** First-byte builder macro for all CSBK/MBC structures */
#define DMR_CSBK_B0(lb, pf, op) \
    (uint8_t)(((lb)&1u)<<7 | ((pf)&1u)<<6 | ((op)&0x3Fu))

/** First-byte field accessors */
#define DMR_CSBK_GET_LB(c)     (((c)->lb_pf_csbko >> 7) & 0x01u)
#define DMR_CSBK_GET_PF(c)     (((c)->lb_pf_csbko >> 6) & 0x01u)
#define DMR_CSBK_GET_OPCODE(c) (((c)->lb_pf_csbko      ) & 0x3Fu)


/**
 * @brief Base CSBK PDU — 12 bytes
 *        Generic form; cast to specific opcode structs as needed.
 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko; /* [7]=LB [6]=PF [5:0]=CSBKO                             */
    uint8_t fid;         /* Feature ID                                              */
    uint8_t data[8];     /* 64 bits opcode-specific payload                         */
    uint8_t crc_hi;      /* CRC-CCITT[15:8]                                         */
    uint8_t crc_lo;      /* CRC-CCITT[7:0]                                          */
} dmr_csbk_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_t) == 12, csbk_base);


/** BS Down-link Activate (CSBKO=0x28) — ETSI TS 102 361-2, Table 7.5a */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;    /* DMR_CSBK_B0(1,0,0x28)                               */
    uint8_t fid;
    uint8_t reserved[2];
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_bs_dwna_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_bs_dwna_t) == 12, csbk_bsdwna);


/** Preamble (CSBKO=0x3D) — ETSI TS 102 361-2, Table 7.7 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;    /* DMR_CSBK_B0(1,0,0x3D)                               */
    uint8_t fid;
    uint8_t type_blks;      /* [7]=data(1)/csbk(0) [6]=grp/ind [5:0]=reserved      */
    uint8_t cbf;            /* CSBK Blocks to Follow                                */
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_preamble_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_preamble_t) == 12, csbk_preamble);


/** Unit-to-Unit Voice Service Request (CSBKO=0x04) — TS 102 361-2, Table 7.5b */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;    /* DMR_CSBK_B0(1,0,0x04)                               */
    uint8_t fid;
    uint8_t svc;            /* Service Options                                      */
    uint8_t reserved;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_uu_v_req_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_uu_v_req_t) == 12, csbk_uuvreq);


/**
 * Unit-to-Unit Answer Response (CSBKO=0x24) — TS 102 361-2, Table 7.5c
 * Byte 2: [7:4]=answer_response  [3:0]=reserved
 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;    /* DMR_CSBK_B0(1,0,0x24)                               */
    uint8_t fid;
    uint8_t ans_svc;        /* [7:4]=answer_response [3:0]=reserved                 */
    uint8_t reserved;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_uu_ans_rsp_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_uu_ans_rsp_t) == 12, csbk_uuans);

#define DMR_UU_ANS_GET_RESPONSE(c) (((c)->ans_svc >> 4) & 0x0Fu)
#define DMR_UU_ANS_SET_RESPONSE(c,v) \
    ((c)->ans_svc = (uint8_t)(((c)->ans_svc & 0x0Fu) | (((v)&0x0Fu)<<4)))


/** Call Alert (CSBKO=0x1F) — TS 102 361-2, Clause 7.1.2 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t reserved[2];
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_call_alert_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_call_alert_t) == 12, csbk_calert);


/** Acknowledge Response (CSBKO=0x20) */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t port_cmd;       /* [7:4]=src_port [3:0]=response_cmd                   */
    uint8_t additional;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_ack_rsp_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_ack_rsp_t) == 12, csbk_ack);


/** Cancel Call Alert (CSBKO=0x23) */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t reserved[2];
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_cancel_call_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_cancel_call_t) == 12, csbk_cancel);


/** Emergency Alarm Acknowledgement (CSBKO=0x27) */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t reserved[2];
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_emerg_alarm_ack_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_emerg_alarm_ack_t) == 12, csbk_emack);


/** Channel Timing (CSBKO=0x22) — DMO TDMA timing, TS 102 361-2, Cl.6.2.2.3 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t ts_info;
    uint8_t reserved;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_csbk_channel_timing_t;
DMR_STATIC_ASSERT(sizeof(dmr_csbk_channel_timing_t) == 12, csbk_ct);


/* =========================================================================
 * SECTION 8 — LINK CONTROL MESSAGE PDUs (9-byte info payload, packed)
 *             ETSI TS 102 361-2, Clause 7.1.1
 * ========================================================================= */

/** Group Voice Channel User LC (FLCO=0x00) — TS 102 361-2, Table 7.1 */
typedef struct DMR_PACKED {
    uint8_t flco;        /* [7:2]=0x00 [1]=FID_present [0]=reserved                */
    uint8_t fid;
    uint8_t svc;         /* Service Options                                          */
    uint8_t group_id[3]; /* Destination Group ID                                    */
    uint8_t src_id[3];   /* Source Radio ID                                         */
} dmr_lc_grp_v_ch_usr_t;
DMR_STATIC_ASSERT(sizeof(dmr_lc_grp_v_ch_usr_t) == 9, lc_grpv);


/** Individual Voice Channel User LC (FLCO=0x03) — TS 102 361-2, Table 7.2 */
typedef struct DMR_PACKED {
    uint8_t flco;
    uint8_t fid;
    uint8_t svc;
    uint8_t dst_id[3];
    uint8_t src_id[3];
} dmr_lc_ind_v_ch_usr_t;
DMR_STATIC_ASSERT(sizeof(dmr_lc_ind_v_ch_usr_t) == 9, lc_indv);


/**
 * GPS Info LC (FLCO=0x08) — TS 102 361-2, Table 7.3
 *
 * Byte 2: [7:5]=reserved [4:2]=PositionError [1:0]=Longitude[24:23]
 * Bytes 3-5: Longitude[22:0] continued  (25 bits total, big-endian split)
 * Bytes 5-7: Latitude[23:0]              (24 bits total)
 * Bytes 7-8: padding/reserved
 *
 * Use gps_to_lc_format() / gps_from_lc_format() helper functions.
 */
typedef struct DMR_PACKED {
    uint8_t flco;        /* 0x08                                                    */
    uint8_t fid;
    uint8_t err_lon2;    /* [4:2]=PosErr [1:0]=Lon[24:23]                          */
    uint8_t lon_hi;      /* Lon[22:15]                                              */
    uint8_t lon_lo;      /* Lon[14:7]                                               */
    uint8_t lon_lat;     /* [7:1]=Lon[6:0] [0]=Lat[23]                             */
    uint8_t lat_hi;      /* Lat[22:15]                                              */
    uint8_t lat_lo;      /* Lat[14:7]                                               */
    uint8_t lat_end;     /* [7:1]=Lat[6:0] [0]=reserved                            */
} dmr_lc_gps_info_t;
DMR_STATIC_ASSERT(sizeof(dmr_lc_gps_info_t) == 9, lc_gps);


/** Emergency Voice Channel User LC (FLCO=0x10, proprietary extension) */
typedef struct DMR_PACKED {
    uint8_t flco;
    uint8_t fid;
    uint8_t svc;
    uint8_t emerg_type_res; /* [7:4]=emerg_type [3:0]=reserved                     */
    uint8_t dst_id[3];
    uint8_t src_id[2];      /* Only 2 bytes to keep total at 9 bytes                */
} dmr_lc_emerg_voice_usr_t;
DMR_STATIC_ASSERT(sizeof(dmr_lc_emerg_voice_usr_t) == 9, lc_emerg);


/* =========================================================================
 * SECTION 9 — PI (PRIVACY INDICATOR) HEADER PDU — 17 bytes
 *             ETSI TS 102 361-1, Clause 9.1.5  Data Type = 0x00
 * ========================================================================= */
typedef struct DMR_PACKED {
    uint8_t mi[9];       /* Message Indicator (72-bit PRNG seed)                    */
    uint8_t alg_id;      /* Algorithm ID (DMR_ALG_*)                                */
    uint8_t key_id[2];   /* Key ID [15:8], [7:0]                                    */
    uint8_t dst_id[3];   /* Destination ID                                          */
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_pi_header_t;
DMR_STATIC_ASSERT(sizeof(dmr_pi_header_t) == 17, pi_hdr);


/* =========================================================================
 * SECTION 10 — DATA HEADER PDUs — each exactly 12 bytes
 *              ETSI TS 102 361-1, Clause 8.2.1 / 9.2
 *              Data Type = 0x06, encoded in BPTC(196,96) on-air
 * ========================================================================= */

/**
 * Unconfirmed Data Header (DPFT=0x02) — TS 102 361-1, Figure 8.3
 *
 * Byte 0:  [7]=G/I [6]=A(=0) [5]=res [4]=POC_MSB [3:0]=DPFT(=0x02)
 * Byte 1:  [7:4]=SAP [3:0]=POC[3:0]
 * Bytes 2-4: Destination LLID
 * Bytes 5-7: Source LLID
 * Byte 8:  [7]=FMF [6:0]=Blocks-to-Follow
 * Byte 9:  [7:4]=reserved [3:0]=FSN
 * Bytes 10-11: CRC-CCITT
 */
typedef struct DMR_PACKED {
    uint8_t flags;       /* [7]=G/I [6]=A [5]=res [4]=POC_MSB [3:0]=DPFT          */
    uint8_t sap_poc;     /* [7:4]=SAP [3:0]=POC[3:0]                               */
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t fmf_blks;   /* [7]=FMF [6:0]=Blocks-to-Follow                          */
    uint8_t res_fsn;     /* [7:4]=reserved [3:0]=FSN                                */
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_data_hdr_unconfirmed_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_hdr_unconfirmed_t) == 12, dhdr_unconf);


/**
 * Confirmed Data Header (DPFT=0x03) — TS 102 361-1, Figure 8.4
 *
 * Byte 9: [7]=A-bit [6:4]=N(S) [3:0]=FSN
 */
typedef struct DMR_PACKED {
    uint8_t flags;
    uint8_t sap_poc;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t fmf_blks;
    uint8_t a_ns_fsn;    /* [7]=A [6:4]=N(S) [3:0]=FSN                              */
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_data_hdr_confirmed_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_hdr_confirmed_t) == 12, dhdr_conf);


/** Response Data Header (DPFT=0x01) — TS 102 361-1, Figure 8.5 */
typedef struct DMR_PACKED {
    uint8_t flags;
    uint8_t sap_res;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t fmf_blks;
    uint8_t class_type_status; /* [7:6]=Class [5:3]=Type [2:0]=Status              */
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_data_hdr_response_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_hdr_response_t) == 12, dhdr_resp);


/** UDT Short Data Header (DPFT=0x00) — TS 102 361-1, Figure 8.10 */
typedef struct DMR_PACKED {
    uint8_t flags;
    uint8_t sap_poc;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t udt_fmt_pad; /* [7:4]=UDT format [3:0]=pad nibble count                */
    uint8_t bit_padding;
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_data_hdr_udt_short_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_hdr_udt_short_t) == 12, dhdr_udt);


/* =========================================================================
 * SECTION 10A — SHORT DATA HEADERS — TS 102 361-1 Cl.9.2.10-9.2.12,
 *               TS 102 361-3 Cl.6 (Short data bearer service)
 *
 * Structurally related to but NOT byte-compatible with the Section 10
 * family above: byte0 bits[5:4] and byte1 bits[3:0] carry a SPLIT
 * 6-bit Appended Blocks (AB) field here (2 MSBs + 4 LSBs) instead of
 * [reserved,POC_MSB] / [SAP,POC] — AB replaces POC entirely for this
 * family. dst/src LLID stay at the same byte offsets (2-4/5-7) as the
 * Section 10 family; G/I (byte0 bit7), A (byte0 bit6), and SAP
 * (byte1 bits[7:4]) also keep the same positions.
 * ========================================================================= */

/** Status/Precoded Short Data Header (SP_HEAD, DPFT=0x0E, AB=0) —
 *  TS 102 361-1 Table 9.17A. No data blocks follow — the Status/
 *  Precoded value is carried entirely within this 12-byte header. */
typedef struct DMR_PACKED {
    uint8_t flags;       /* [7]=G/I [6]=A [5:4]=AB_msb(=0) [3:0]=DPFT(=0x0E)   */
    uint8_t sap_ab;      /* [7:4]=SAP [3:0]=AB_lsb(=0)                         */
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t ports_stat_hi; /* [7:5]=Source Port [4:2]=Dest Port [1:0]=Status[9:8] */
    uint8_t stat_lo;       /* Status/Precoded[7:0]                              */
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_data_hdr_sp_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_hdr_sp_t) == 12, dhdr_sp);

/** Raw Short Data Header (R_HEAD, DPFT=0x0E, AB usually !=0) —
 *  TS 102 361-1 Table 9.17B. Followed by AB data blocks; port-based
 *  (not SAP-registration-based) application addressing. */
typedef struct DMR_PACKED {
    uint8_t flags;       /* [7]=G/I [6]=A [5:4]=AB_msb [3:0]=DPFT(=0x0E)       */
    uint8_t sap_ab;      /* [7:4]=SAP [3:0]=AB_lsb                             */
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t ports_sarq_fmf; /* [7:5]=Source Port [4:2]=Dest Port [1]=SARQ [0]=FMF */
    uint8_t bit_padding;    /* Reserved, transmitted as 0                       */
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_data_hdr_raw_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_hdr_raw_t) == 12, dhdr_raw);

/** Defined Data Short Data Header (DD_HEAD, DPFT=0x0D) —
 *  TS 102 361-1 Table 9.17C. Followed by AB data blocks; no ports —
 *  the DD Format code identifies the payload's predefined structure. */
typedef struct DMR_PACKED {
    uint8_t flags;       /* [7]=G/I [6]=A [5:4]=AB_msb [3:0]=DPFT(=0x0D)       */
    uint8_t sap_ab;      /* [7:4]=SAP [3:0]=AB_lsb                             */
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t dd_sarq_fmf; /* [7:2]=Defined Data (DD) Format [1]=SARQ [0]=FMF    */
    uint8_t bit_padding; /* Reserved, transmitted as 0                          */
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_data_hdr_dd_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_hdr_dd_t) == 12, dhdr_dd);


/** Proprietary Data Header — TS 102 361-1, Figure 8.6 */
typedef struct DMR_PACKED {
    uint8_t sap_dpft;    /* [7:4]=SAP [3:0]=DPFT                                    */
    uint8_t mfid;
    uint8_t payload[8];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_data_hdr_proprietary_t;
DMR_STATIC_ASSERT(sizeof(dmr_data_hdr_proprietary_t) == 12, dhdr_prop);


/* =========================================================================
 * SECTION 11 — DATA BLOCKS
 * ========================================================================= */

/** Rate-1 Data Block — 12 bytes (96 bits info) — TS 102 361-1, Clause 9.2.15 */
typedef struct DMR_PACKED {
    uint8_t lb_dbsn;    /* [7]=Last_Block [6:0]=DBSN                               */
    uint8_t data[11];
} dmr_rate1_data_block_t;
DMR_STATIC_ASSERT(sizeof(dmr_rate1_data_block_t) == 12, r1blk);


/** Rate-1/2 Data Block — 6 bytes — TS 102 361-1, Clause 9.2.7 */
typedef struct DMR_PACKED {
    uint8_t lb_dbsn;
    uint8_t data[5];
} dmr_rate12_data_block_t;
DMR_STATIC_ASSERT(sizeof(dmr_rate12_data_block_t) == 6, r12blk);


/** Rate-3/4 Data Block — 3 bytes — TS 102 361-1, Clause 9.2.2 */
typedef struct DMR_PACKED {
    uint8_t lb_dbsn;
    uint8_t data_hi;    /* data bits [9:2]                                          */
    uint8_t data_lo;    /* [7:6]=data[1:0] [5:0]=padding                           */
} dmr_rate34_data_block_t;
DMR_STATIC_ASSERT(sizeof(dmr_rate34_data_block_t) == 3, r34blk);


/* =========================================================================
 * SECTION 12 — UDP/IP COMPRESSED HEADER (SAP=0x02)
 *              ETSI TS 102 361-3, Clause 6
 * ========================================================================= */
typedef struct DMR_PACKED {
    uint8_t version_res; /* [7:4]=IP version (4) [3:0]=reserved                    */
    uint8_t ip_src[4];   /* Source IPv4                                             */
    uint8_t ip_dst[4];   /* Destination IPv4                                        */
    uint8_t udp_src[2];  /* UDP source port [15:8],[7:0]                            */
    uint8_t udp_dst[2];  /* UDP destination port [15:8],[7:0]                       */
} dmr_udpip_compressed_hdr_t;
DMR_STATIC_ASSERT(sizeof(dmr_udpip_compressed_hdr_t) == 13, udpip_hdr);


/* =========================================================================
 * SECTION 13 — GPS / LRRP REPORT (runtime use, TS 102 361-2 Cl.7.1.8)
 *              Transmitted via confirmed data call — not a wire-format struct.
 * ========================================================================= */
typedef struct {
    int32_t  latitude;       /* Latitude  × 1e-7 degrees (WGS84)    4 bytes        */
    int32_t  longitude;      /* Longitude × 1e-7 degrees (WGS84)    4 bytes        */
    uint32_t timestamp;      /* UTC Unix epoch                       4 bytes        */
    int16_t  altitude;       /* Altitude in 0.5 m steps             2 bytes        */
    uint8_t  src_id[3];      /* Source Radio ID                      3 bytes        */
    uint8_t  report_type;    /* 0=Immediate 1=Triggered 2=Poll      1 byte         */
    uint8_t  reason;         /* Reason code                          1 byte         */
    uint8_t  pos_error;      /* Position error class (0-7)           1 byte         */
    uint8_t  speed;          /* Speed in km/h                        1 byte         */
    uint8_t  heading;        /* Heading in 2° steps                  1 byte         */
    uint8_t  alt_valid;      /* 1=altitude valid                     1 byte         */
    uint8_t  speed_valid;    /* 1=speed valid                        1 byte         */
    uint8_t  _pad[1];        /* explicit pad → 4-byte alignment     1 byte         */
} dmr_gps_report_t;
/* 4+4+4+2+3+1+1+1+1+1+1+1+1 = 25 → pad 1 → 26? recount:
   int32 lat=4, int32 lon=4, uint32 ts=4, int16 alt=2, uint8[3]=3,
   uint8 rtype=1, uint8 reason=1, uint8 poserr=1, uint8 speed=1,
   uint8 heading=1, uint8 alt_valid=1, uint8 speed_valid=1, uint8 pad=1
   = 4+4+4+2+3+1+1+1+1+1+1+1+1 = 25 bytes + 1 pad = 26 bytes,
   but int32 needs 4-byte align: largest member=int32(4), size must be multiple of 4
   26 → pad to 28 bytes */
DMR_STATIC_ASSERT(sizeof(dmr_gps_report_t) == 28, gps_report);


/* =========================================================================
 * SECTION 14 — TIER III TRUNKING CSBK PDUs (all 12 bytes)
 *              ETSI TS 102 361-4, Clause 6
 * ========================================================================= */

/** Traffic Channel Voice Grant (CSBKO=0x01) — TS 102 361-4, Cl.6.3 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t ad_emerg_res;  /* [7]=A/D [6]=emergency [5:0]=reserved                 */
    uint8_t ch_id[2];      /* Assigned channel ID [15:8],[7:0]                      */
    uint8_t slot_res;      /* [7:6]=slot [5:0]=reserved                             */
    uint8_t dst_id[3];
    uint8_t src_id_hi;     /* Source ID [23:16] — lower 16 bits in crc_hi/lo?
                              NOTE: TV_GRANT only has 1 byte of src → use dst       */
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_t3_csbk_tv_grant_t;
DMR_STATIC_ASSERT(sizeof(dmr_t3_csbk_tv_grant_t) == 12, t3tvg);


/** MS Registration (CSBKO=0x24) — TS 102 361-4, Cl.6.7 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t reason_res;    /* [7:4]=reason [3:0]=reserved                           */
    uint8_t reserved;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_t3_csbk_ms_reg_t;
DMR_STATIC_ASSERT(sizeof(dmr_t3_csbk_ms_reg_t) == 12, t3reg);


/** MS Registration Response (CSBKO=0x25) — TS 102 361-4, Cl.6.7 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t res_attach;    /* [7:4]=response_code [3]=attach [2:0]=reserved        */
    uint8_t reg_grp[3];    /* Assigned registration group                           */
    uint8_t dst_id[3];
    uint8_t reserved;
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_t3_csbk_ms_reg_rsp_t;
DMR_STATIC_ASSERT(sizeof(dmr_t3_csbk_ms_reg_rsp_t) == 12, t3regrsp);


/** Network Status Broadcast (CSBKO=0x14) — TS 102 361-4, Cl.6.1 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t net_id[3];
    uint8_t site_id;
    uint8_t ch_count_access; /* [7:3]=ch_count [2:0]=req_access                    */
    uint8_t svc_res;          /* [7:4]=svc_type [3:0]=reserved                     */
    uint8_t reserved[2];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_t3_csbk_net_status_t;
DMR_STATIC_ASSERT(sizeof(dmr_t3_csbk_net_status_t) == 12, t3net);


/** Adjacent Site Information (CSBKO=0x19) — TS 102 361-4, Cl.6.1 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t area_id;
    uint8_t sys_id[2];
    uint8_t site_id;
    uint8_t ch_id[2];
    uint8_t access_svc;  /* [7:5]=req_access [4:1]=svc_type [0]=reserved           */
    uint8_t reserved;
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_t3_csbk_adj_site_t;
DMR_STATIC_ASSERT(sizeof(dmr_t3_csbk_adj_site_t) == 12, t3adj);


/** Emergency Alarm (CSBKO=0x28) — TS 102 361-4, Cl.6.9 */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;
    uint8_t fid;
    uint8_t emerg_type_res; /* [7:4]=emerg_type [3:0]=reserved                     */
    uint8_t reserved;
    uint8_t dst_id[3];
    uint8_t src_id[3];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_t3_csbk_emerg_alarm_t;
DMR_STATIC_ASSERT(sizeof(dmr_t3_csbk_emerg_alarm_t) == 12, t3em);


/* =========================================================================
 * SECTION 15 — MULTI-BLOCK CONTROL (MBC) PDUs (each 12 bytes)
 *              ETSI TS 102 361-1, Clause 7.4
 * ========================================================================= */

/** MBC Header — Data Type=0x04, LB=0 for header blocks */
typedef struct DMR_PACKED {
    uint8_t lb_pf_csbko;    /* LB=0 for header, PF=0, CSBKO=opcode                 */
    uint8_t fid;
    uint8_t reserved;
    uint8_t data[7];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_mbc_header_t;
DMR_STATIC_ASSERT(sizeof(dmr_mbc_header_t) == 12, mbc_hdr);


/** MBC Continuation / Last Block — Data Type=0x05 */
typedef struct DMR_PACKED {
    uint8_t lb_res;         /* [7]=LB [6:0]=reserved                                */
    uint8_t data[9];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_mbc_cont_t;
DMR_STATIC_ASSERT(sizeof(dmr_mbc_cont_t) == 12, mbc_cont);


/** Idle PDU — Data Type=0x09 */
typedef struct DMR_PACKED {
    uint8_t data[10];
    uint8_t crc_hi;
    uint8_t crc_lo;
} dmr_idle_pdu_t;
DMR_STATIC_ASSERT(sizeof(dmr_idle_pdu_t) == 12, idle_pdu);


/* =========================================================================
 * SECTION 16 — CHANNEL & FRAME DESCRIPTORS (runtime only)
 * ========================================================================= */

/**
 * @brief Logical channel descriptor — runtime metadata, not transmitted.
 *        Explicit padding for natural alignment.
 */
typedef struct {
    uint32_t frequency_hz;  /* RF carrier frequency in Hz              (4 bytes)   */
    uint8_t  timeslot;      /* 1 or 2                                  (1 byte)    */
    uint8_t  colour_code;   /* Colour code 0-15                        (1 byte)    */
    uint8_t  is_control_ch; /* 1=TSCC control channel                  (1 byte)    */
    uint8_t  is_voice_ch;   /* 1=voice traffic                         (1 byte)    */
    uint8_t  is_direct_mode;/* 1=DMO (no base station)                 (1 byte)    */
    uint8_t  _pad[3];       /* explicit pad → 12 bytes total           (3 bytes)   */
} dmr_logical_channel_t;
DMR_STATIC_ASSERT(sizeof(dmr_logical_channel_t) == 12, logical_ch);


/**
 * @brief TDMA frame container — runtime, not transmitted.
 *        slot1(44) + slot2(44) + cach(3) + _pad(5) + frame_no(4) + ts(8) = 108
 */
typedef struct {
    dmr_burst_t    slot1;          /* Timeslot 1 burst              (44 bytes)     */
    dmr_burst_t    slot2;          /* Timeslot 2 burst              (44 bytes)     */
    dmr_cach_pdu_t cach;           /* CACH between bursts           (3 bytes)      */
    uint8_t        _pad[5];        /* explicit pad → align uint32_t (5 bytes)      */
    uint32_t       frame_number;   /* Frame sequence counter        (4 bytes)      */
    uint64_t       timestamp_us;   /* CLOCK_MONOTONIC timestamp     (8 bytes)      */
} dmr_tdma_frame_t;
/* 44+44+3+5+4+8 = 108 bytes */
DMR_STATIC_ASSERT(sizeof(dmr_tdma_frame_t) == 112, tdma_frame);


/* =========================================================================
 * SECTION 17 — UTILITY MACROS & INLINE FUNCTIONS
 * ========================================================================= */

/** Pack/unpack 24-bit Radio ID — always big-endian (MSB first) */
#define DMR_GET_ID(arr) \
    ((uint32_t)(((uint32_t)(arr)[0] << 16) | \
                ((uint32_t)(arr)[1] <<  8) | \
                 (uint32_t)(arr)[2]))

#define DMR_SET_ID(arr, id) do { \
    (arr)[0] = (uint8_t)(((uint32_t)(id) >> 16) & 0xFFu); \
    (arr)[1] = (uint8_t)(((uint32_t)(id) >>  8) & 0xFFu); \
    (arr)[2] = (uint8_t)( (uint32_t)(id)         & 0xFFu); \
} while (0)

/** Full LC FLCO byte helpers */
#define DMR_LC_FLCO(b)           (((b) >> 2) & 0x3Fu)
#define DMR_LC_FID_PRESENT(b)    (((b) >> 1) & 0x01u)
#define DMR_LC_MAKE_BYTE(op, fp) (uint8_t)(((op)&0x3Fu)<<2 | ((fp)?0x02u:0x00u))

/** Zero-init a CSBK and set first byte */
static inline void dmr_csbk_init(dmr_csbk_t *c, uint8_t opcode)
{
    memset(c, 0, sizeof(*c));
    c->lb_pf_csbko = DMR_CSBK_B0(1u, 0u, opcode);
}

/** Zero-init a Full LC and set opcode + IDs */
static inline void dmr_full_lc_init(dmr_full_lc_t *lc,
                                     uint8_t opcode,
                                     uint32_t dst_id,
                                     uint32_t src_id)
{
    memset(lc, 0, sizeof(*lc));
    lc->flco = DMR_LC_MAKE_BYTE(opcode, 0);
    DMR_SET_ID(lc->dst_id, dst_id);
    DMR_SET_ID(lc->src_id, src_id);
}


/* =========================================================================
 * SECTION 18 — NON-BYTE-ALIGNED BURST FIELD ACCESS FUNCTIONS
 *
 * THE SYNC FIELD IS NOT BYTE-ALIGNED IN raw[].
 * It straddles raw[13][3:0] through raw[19][7:4].
 *
 * These functions are the ONLY correct way to read/write sync, slot_type,
 * and emb fields in a burst raw[] array. Never access bytes 13-20 directly
 * for these fields without using these helpers.
 * ========================================================================= */

/**
 * @brief Write a 48-bit SYNC pattern into burst raw[] at the correct
 *        non-byte-aligned position.
 *
 * SYNC occupies:
 *   raw[13][3:0]  ← sync[47:44]  (4 bits, lower nibble)
 *   raw[14][7:0]  ← sync[43:36]  (full byte)
 *   raw[15][7:0]  ← sync[35:28]
 *   raw[16][7:0]  ← sync[27:20]
 *   raw[17][7:0]  ← sync[19:12]
 *   raw[18][7:0]  ← sync[11:4]
 *   raw[19][7:4]  ← sync[3:0]    (4 bits, upper nibble)
 *
 * The upper nibble of raw[13] and lower nibble of raw[19] are
 * PRESERVED (they carry SLOT_TYPE or voice INFO bits).
 *
 * @param raw    Pointer to 33-byte burst buffer
 * @param sync   48-bit sync pattern (bits 47..0, MSB=first transmitted)
 */
static inline void dmr_burst_set_sync(uint8_t *raw, uint64_t sync)
{
    /* raw[13]: preserve upper nibble [7:4], replace lower nibble [3:0]
     * with sync[47:44]                                                      */
    raw[13] = (uint8_t)((raw[13] & 0xF0u) | ((sync >> 44) & 0x0Fu));

    /* raw[14..18]: full bytes — sync[43:36], [35:28], [27:20], [19:12], [11:4] */
    raw[14] = (uint8_t)((sync >> 36) & 0xFFu);
    raw[15] = (uint8_t)((sync >> 28) & 0xFFu);
    raw[16] = (uint8_t)((sync >> 20) & 0xFFu);
    raw[17] = (uint8_t)((sync >> 12) & 0xFFu);
    raw[18] = (uint8_t)((sync >>  4) & 0xFFu);

    /* raw[19]: preserve lower nibble [3:0], replace upper nibble [7:4]
     * with sync[3:0]                                                         */
    raw[19] = (uint8_t)((raw[19] & 0x0Fu) | ((sync & 0x0Fu) << 4));
}

/**
 * @brief Read the 48-bit SYNC pattern from burst raw[] at the correct
 *        non-byte-aligned position.
 *
 * @param raw  Pointer to 33-byte burst buffer
 * @return     48-bit sync value (bits 47..0)
 */
static inline uint64_t dmr_burst_get_sync(const uint8_t *raw)
{
    uint64_t sync = 0;

    /* sync[47:44] from raw[13][3:0] */
    sync  = (uint64_t)(raw[13] & 0x0Fu) << 44;

    /* sync[43:36] from raw[14] */
    sync |= (uint64_t)raw[14] << 36;
    sync |= (uint64_t)raw[15] << 28;
    sync |= (uint64_t)raw[16] << 20;
    sync |= (uint64_t)raw[17] << 12;
    sync |= (uint64_t)raw[18] <<  4;

    /* sync[3:0] from raw[19][7:4] */
    sync |= (uint64_t)(raw[19] >> 4) & 0x0Fu;

    return sync;
}

/**
 * @brief Check whether the burst raw[] contains a specific SYNC pattern.
 *        Performs non-byte-aligned comparison across raw[13..19].
 *
 * @param raw  Pointer to 33-byte burst buffer
 * @param sync Expected 48-bit sync pattern constant (DMR_SYNC_*)
 * @return     true if the sync field matches exactly
 */
static inline bool dmr_burst_check_sync(const uint8_t *raw, uint64_t sync)
{
    /* Check raw[13] lower nibble */
    if ((raw[13] & 0x0Fu) != (uint8_t)((sync >> 44) & 0x0Fu)) return false;

    /* Check raw[14..18] — full bytes */
    if (raw[14] != (uint8_t)((sync >> 36) & 0xFFu)) return false;
    if (raw[15] != (uint8_t)((sync >> 28) & 0xFFu)) return false;
    if (raw[16] != (uint8_t)((sync >> 20) & 0xFFu)) return false;
    if (raw[17] != (uint8_t)((sync >> 12) & 0xFFu)) return false;
    if (raw[18] != (uint8_t)((sync >>  4) & 0xFFu)) return false;

    /* Check raw[19] upper nibble */
    if ((raw[19] >> 4) != (uint8_t)(sync & 0x0Fu)) return false;

    return true;
}

/**
 * @brief Determine if a burst is a voice burst by checking for any voice
 *        sync pattern (BS, MS, or direct mode voice).
 */
static inline bool dmr_burst_is_voice(const uint8_t *raw)
{
    return dmr_burst_check_sync(raw, DMR_SYNC_BS_VOICE)    ||
           dmr_burst_check_sync(raw, DMR_SYNC_MS_VOICE)    ||
           dmr_burst_check_sync(raw, DMR_SYNC_DIRECT_VOICE);
}

/**
 * @brief Determine if a burst is a data/control burst.
 */
static inline bool dmr_burst_is_data(const uint8_t *raw)
{
    return dmr_burst_check_sync(raw, DMR_SYNC_BS_DATA)    ||
           dmr_burst_check_sync(raw, DMR_SYNC_MS_DATA)    ||
           dmr_burst_check_sync(raw, DMR_SYNC_DIRECT_DATA);
}

/**
 * @brief Identify the sync pattern type in a burst.
 * @return Pointer to matched DMR_SYNC_* constant name string, or "UNKNOWN"
 */
static inline const char *dmr_burst_sync_name(const uint8_t *raw)
{
    uint64_t s = dmr_burst_get_sync(raw);
    if (s == DMR_SYNC_BS_VOICE)     return "BS_VOICE";
    if (s == DMR_SYNC_BS_DATA)      return "BS_DATA";
    if (s == DMR_SYNC_MS_VOICE)     return "MS_VOICE";
    if (s == DMR_SYNC_MS_DATA)      return "MS_DATA";
    if (s == DMR_SYNC_DIRECT_VOICE) return "DIRECT_VOICE";
    if (s == DMR_SYNC_DIRECT_DATA)  return "DIRECT_DATA";
    if (s == DMR_SYNC_RC)           return "RC";
    if (s == DMR_SYNC_IDLE)         return "IDLE";
    return "UNKNOWN";
}


/**
 * @brief Write SLOT_TYPE into a data burst raw[] at the correct
 *        non-byte-aligned positions (symbols L17..L13 and R13..R17).
 *
 * SLOT_TYPE high half (symbols L17..L13 = dibits 49..53):
 *   raw[12][5:4] ← CC[3:2]
 *   raw[12][3:2] ← CC[1:0]
 *   raw[12][1:0] ← DT[3:2]
 *   raw[13][7:6] ← DT[1:0]
 *   raw[13][5:4] ← Golay[11:10]
 *
 * SLOT_TYPE low half (symbols R13..R17 = dibits 78..82):
 *   raw[19][3:2] ← Golay[9:8]
 *   raw[19][1:0] ← Golay[7:6]
 *   raw[20][7:6] ← Golay[5:4]
 *   raw[20][5:4] ← Golay[3:2]
 *   raw[20][3:2] ← Golay[1:0]
 *
 * raw[12][7:6] (INFO_1 last bits) and raw[13][3:0] (SYNC first 4 bits)
 * and raw[19][7:4] (SYNC last 4 bits) and raw[20][1:0] (INFO_2 first bits)
 * are PRESERVED.
 *
 * @param raw     33-byte burst buffer
 * @param cc      Colour Code (0-15)
 * @param dtype   Data Type (DMR_DTYPE_*)
 * @param golay   12-bit Golay(18,6) FEC over CC+DT (bits 11..0)
 */
static inline void dmr_burst_set_slot_type(uint8_t *raw,
                                            uint8_t  cc,
                                            uint8_t  dtype,
                                            uint16_t golay)
{
    /* High half — preserve raw[12][7:6] (INFO_1 bits), raw[13][3:0] (SYNC) */
    raw[12] = (uint8_t)((raw[12] & 0xC0u)         /* preserve [7:6]          */
            | ((cc    & 0x0Cu) << 2)               /* CC[3:2] → raw[12][5:4] */
            | ((cc    & 0x03u) << 2)               /* CC[1:0] → raw[12][3:2] */
            | ((dtype & 0x0Cu) >> 2));             /* DT[3:2] → raw[12][1:0] */
    /* Fix CC packing: CC is 4 bits, so: */
    raw[12] = (uint8_t)((raw[12] & 0xC0u)
            | (((cc   >> 2) & 0x03u) << 4)         /* CC[3:2] → bits [5:4]   */
            | (((cc        ) & 0x03u) << 2)         /* CC[1:0] → bits [3:2]   */
            | (((dtype >> 2) & 0x03u)));            /* DT[3:2] → bits [1:0]   */

    raw[13] = (uint8_t)((raw[13] & 0x0Fu)          /* preserve [3:0] (SYNC)  */
            | (((dtype     ) & 0x03u) << 6)         /* DT[1:0] → bits [7:6]   */
            | (((golay >> 10) & 0x03u) << 4));      /* Golay[11:10]→bits[5:4] */

    /* Low half — preserve raw[19][7:4] (SYNC) and raw[20][1:0] (INFO_2) */
    raw[19] = (uint8_t)((raw[19] & 0xF0u)          /* preserve [7:4] (SYNC)  */
            | (((golay >> 8) & 0x03u) << 2)         /* Golay[9:8] → bits[3:2] */
            | (((golay >> 6) & 0x03u)));            /* Golay[7:6] → bits[1:0] */

    raw[20] = (uint8_t)((raw[20] & 0x03u)          /* preserve [1:0] (INFO_2)*/
            | (((golay >> 4) & 0x03u) << 6)         /* Golay[5:4] → bits[7:6] */
            | (((golay >> 2) & 0x03u) << 4)         /* Golay[3:2] → bits[5:4] */
            | (((golay     ) & 0x03u) << 2));        /* Golay[1:0] → bits[3:2] */
}

/**
 * @brief Read SLOT_TYPE fields from a data burst raw[].
 *
 * @param raw    33-byte burst buffer
 * @param cc     Output: Colour Code (0-15)
 * @param dtype  Output: Data Type (DMR_DTYPE_*)
 * @param golay  Output: 12-bit Golay FEC value
 */
static inline void dmr_burst_get_slot_type(const uint8_t *raw,
                                            uint8_t  *cc,
                                            uint8_t  *dtype,
                                            uint16_t *golay)
{
    /* High half — from raw[12][5:0] and raw[13][7:4] */
    *cc    = (uint8_t)(((raw[12] >> 4) & 0x03u) << 2  /* CC[3:2] from [5:4]    */
                     | ((raw[12] >> 2) & 0x03u));      /* CC[1:0] from [3:2]    */
    *dtype = (uint8_t)(((raw[12]     ) & 0x03u) << 2  /* DT[3:2] from [1:0]    */
                     | ((raw[13] >> 6) & 0x03u));      /* DT[1:0] from [7:6]    */

    /* Golay[11:10] from raw[13][5:4] */
    uint16_t g = (uint16_t)((raw[13] >> 4) & 0x03u) << 10;

    /* Golay[9:6] from raw[19][3:0] */
    g |= (uint16_t)((raw[19] >> 2) & 0x03u) << 8;     /* Golay[9:8]            */
    g |= (uint16_t)((raw[19]     ) & 0x03u) << 6;     /* Golay[7:6]            */

    /* Golay[5:0] from raw[20][7:2] */
    g |= (uint16_t)((raw[20] >> 6) & 0x03u) << 4;     /* Golay[5:4]            */
    g |= (uint16_t)((raw[20] >> 4) & 0x03u) << 2;     /* Golay[3:2]            */
    g |= (uint16_t)((raw[20] >> 2) & 0x03u);          /* Golay[1:0]            */
    *golay = g;
}

/**
 * @brief Write the EMB field into a voice burst raw[] at the correct
 *        non-byte-aligned position (bursts B-F only, symbols L12..R12).
 *
 * EMB control word (8 bits) layout:
 *   raw[13][3:2] ← CC[3:2]
 *   raw[13][1:0] ← CC[1:0]
 *   raw[14][7]   ← PI
 *   raw[14][6]   ← LCSS[1]
 *   raw[14][5]   ← LCSS[0]
 *   raw[14][4]   ← QR[8]
 *
 * EMB LC / QR payload (40 bits):
 *   raw[14][3:0] + raw[15..18][7:0] + raw[19][7:4]
 *   = 4 + 32 + 4 = 40 bits
 *
 * raw[13][7:4] (INFO_1 last 4 bits) and raw[19][3:0] (INFO_2 first 4 bits)
 * are PRESERVED.
 *
 * @param raw        33-byte burst buffer
 * @param cc         Colour Code (0-15)
 * @param pi         Privacy Indicator (0 or 1)
 * @param lcss       LC Start/Stop (DMR_LCSS_*)
 * @param qr16       16-bit QR(16,7) codeword (bits [15:0], bit 8 goes to EMB ctrl)
 * @param lc_frag    Pointer to 4 bytes: 32-bit embedded LC fragment (bit 31 = MSB)
 *                   Pass NULL for null LC (burst F) — fills with zeros.
 */
static inline void dmr_burst_set_emb(uint8_t       *raw,
                                      uint8_t        cc,
                                      uint8_t        pi,
                                      uint8_t        lcss,
                                      uint16_t       qr16,
                                       const uint8_t* lc_frag)
{
    /* raw[13]: preserve [7:4] (INFO_1), write CC[3:0] into [3:0] */
    raw[13] = (uint8_t)((raw[13] & 0xF0u) | (cc & 0x0Fu));

    /* raw[14]: PI(bit7), LCSS[1](bit6), LCSS[0](bit5), QR[8](bit4),
     *          lc_frag bits [31:28] in [3:0]                                */

    /* Fix LCSS placement: LCSS[1] → bit6, LCSS[0] → bit5 */
    raw[14] = (uint8_t)(((pi         & 0x01u) << 7)
                       |(((lcss >> 1) & 0x01u) << 6)  /* LCSS[1] → bit6     */
                       |(( lcss       & 0x01u) << 5)  /* LCSS[0] → bit5     */
                       |(((qr16 >> 8) & 0x01u) << 4)  /* QR[8]   → bit4     */
                       |(lc_frag ? ((lc_frag[0] >> 4) & 0x0Fu) : 0u));


#if 1
    if (lc_frag) {
        /* raw[15]: lc_frag[27:20] */
        raw[15] = (uint8_t)(((lc_frag[0] & 0x0Fu) << 4) | ((lc_frag[1] >> 4) & 0x0Fu));
        /* raw[16]: lc_frag[19:12] */
        raw[16] = (uint8_t)(((lc_frag[1] & 0x0Fu) << 4) | ((lc_frag[2] >> 4) & 0x0Fu));
        /* raw[17]: lc_frag[11:4] */
        raw[17] = (uint8_t)(((lc_frag[2] & 0x0Fu) << 4) | ((lc_frag[3] >> 4) & 0x0Fu));
        /* raw[18]: lc_frag[3:0] | QR[7:4] */
        raw[18] = (uint8_t)(((lc_frag[3] & 0x0Fu) << 4) | ((qr16 >> 4) & 0x0Fu));
    } else {
       
        raw[15] = 0x00u;
        raw[16] = 0x00u;
        raw[17] = 0x00u;
        raw[18] = (uint8_t)((qr16 >> 4) & 0x0Fu);
    }
#endif
    /* raw[19]: QR[3:0] into [7:4], preserve [3:0] (INFO_2) */
    raw[19] = (uint8_t)((raw[19] & 0x0Fu) | ((qr16 & 0x0Fu) << 4));
}

/**
 * @brief Read the EMB control field from a voice burst raw[].
 *
 * @param raw    33-byte burst buffer
 * @param cc     Output: Colour Code
 * @param pi     Output: Privacy Indicator
 * @param lcss   Output: LC Start/Stop
 * @param qr16   Output: QR[8:0] (9 bits, stored in uint16_t lower 9 bits)
 */
static inline void dmr_burst_get_emb_ctrl(const uint8_t *raw,
                                           uint8_t  *cc,
                                           uint8_t  *pi,
                                           uint8_t  *lcss,
                                           uint16_t *qr9)
{
    *cc   = raw[13] & 0x0Fu;                              /* raw[13][3:0]      */
    *pi   = (raw[14] >> 7) & 0x01u;                       /* raw[14][7]        */
    *lcss = (uint8_t)(((raw[14] >> 6) & 0x01u) << 1      /* raw[14][6]=LCSS[1]*/
                     | ((raw[14] >> 5) & 0x01u));         /* raw[14][5]=LCSS[0]*/
    *qr9 = (uint16_t)(((raw[14] >> 4) & 0x01u) << 8     /* QR[8] from [4]   */
                      | (raw[18] & 0x0Fu) << 4            /* QR[7:4] from raw18*/
                      | ((raw[19] >> 4)  & 0x0Fu));        /* QR[3:0] from raw19*/

    /* Note: EMB QR(16,7) FEC correction is performed in fec_rx_process()
     * (MOD-02) before llc_rx_dispatch() is called — callers of this
     * accessor receive already-corrected CC/PI/LCSS/QR9 values. */
                      
}

/**
 * @brief Read the 32-bit embedded LC fragment from a voice burst raw[].
 *
 * Extracts the 32 bits of embedded LC payload (or RC signalling) from the
 * EMB field in voice bursts B-F.
 *
 * @param raw      33-byte burst buffer
 * @param lc_frag  Output: 4-byte buffer receiving the 32-bit fragment
 *                 (byte 0 contains the MSB — first transmitted fragment bit)
 */
static inline void dmr_burst_get_emb_lc(const uint8_t *raw,
                                          uint8_t       *lc_frag)
{
    /* 32-bit fragment spans raw[14][3:0] + raw[15..18][7:0] + raw[19][7:4]
     * = 4 + 32 + 4 = 40 bits total, but only the 32 LC bits (not the 8 QR bits)
     * are in [14][3:0]+[15]+[16]+[17]+[18][7:4] = 4+8+8+8+4 = 32 bits         */
    lc_frag[0] = (uint8_t)(((raw[14] & 0x0Fu) << 4) | ((raw[15] >> 4) & 0x0Fu));
    lc_frag[1] = (uint8_t)(((raw[15] & 0x0Fu) << 4) | ((raw[16] >> 4) & 0x0Fu));
    lc_frag[2] = (uint8_t)(((raw[16] & 0x0Fu) << 4) | ((raw[17] >> 4) & 0x0Fu));
    lc_frag[3] = (uint8_t)(((raw[17] & 0x0Fu) << 4) | ((raw[18] >> 4) & 0x0Fu));
}

/**
 * @brief Read INFO_1 field from a voice burst into a caller buffer.
 *        INFO_1 = VS(215)..VS(108) = 108 bits
 *
 * Extracts bits from raw[0..12] (full, 104 bits) and raw[13][7:4] (4 bits).
 * Output is stored MSB-first in out_14bytes[0..13], where:
 *   out_14bytes[0] bit7 = VS(215) (first transmitted)
 *   out_14bytes[13] bits [7:4] = VS(108..105), bits[3:0] = 0 (unused)
 *
 * @param raw          33-byte burst buffer
 * @param out_14bytes  14-byte output buffer (108 bits, upper-aligned)
 */
static inline void dmr_burst_get_info(const uint8_t *raw,
                                        uint8_t       *out27)//out_14bytes)
{
    #if 0
    /* Copy raw[0..12] directly (104 bits) */
    memcpy(out_14bytes, raw, 13);
    /* Get upper nibble of raw[13] into lower byte, zero lower nibble */
    out_14bytes[13] = raw[13] & 0xF0u;
    #endif
    
    
    
    
    
    
    memset(out27, 0, 27);
    int k = 0;

    /* INFO_1: raw[0..12] (104 bits) + raw[13][7:4] (4 bits) = 108 bits */
    for (int i = 0; i < 13; i++) {
        for (int b = 7; b >= 0; b--) {
            if ((raw[i] >> b) & 1u) out27[k >> 3] |= (uint8_t)(1u << (7 - (k & 7)));
            k++;
        }
    }
    for (int b = 7; b >= 4; b--) {
        if ((raw[13] >> b) & 1u) out27[k >> 3] |= (uint8_t)(1u << (7 - (k & 7)));
        k++;
    }

    /* INFO_2: raw[19][3:0] (4 bits) + raw[20..32] (104 bits) = 108 bits */
    for (int b = 3; b >= 0; b--) {
        if ((raw[19] >> b) & 1u) out27[k >> 3] |= (uint8_t)(1u << (7 - (k & 7)));
        k++;
    }
    for (int i = 20; i <= 32; i++) {
        for (int b = 7; b >= 0; b--) {
            if ((raw[i] >> b) & 1u) out27[k >> 3] |= (uint8_t)(1u << (7 - (k & 7)));
            k++;
        }
    }
    /* k == 216 */

    
    
    
    
    
    
    
    
}








/**
 * @brief Read INFO_2 field from a voice burst into a caller buffer.
 *        INFO_2 = VS(107)..VS(0) = 108 bits
 *
 * Extracts lower nibble of raw[19] (4 bits) and raw[20..32] (104 bits).
 * Output stored in out_14bytes[0..13] upper-aligned:
 *   out_14bytes[0] bit7 = VS(107)
 *   out_14bytes[13] bits [3:0] = VS(3..0), bits [7:4] = VS(7..4)
 *
 * @param raw          33-byte burst buffer
 * @param out_14bytes  14-byte output buffer
 */
static inline void dmr_burst_get_info2(const uint8_t *raw,
                                        uint8_t       *out_14bytes)
{
    /* Shift 108 bits: raw[19][3:0] become high nibble of byte0,
     * then raw[20..32] fill the rest                                         */
  /*  out_14bytes[0] = (uint8_t)((raw[19] & 0x0Fu) << 4) | (raw[20] >> 4);
    for (int i = 1; i < 13; i++) {
        out_14bytes[i] = (uint8_t)((raw[19 + i] & 0x0Fu) << 4)
                       | (raw[20 + i] >> 4);
    }
    out_14bytes[13] = (uint8_t)((raw[32] & 0x0Fu) << 4);*/
            /* printf("\n info1");
                             
                             for(int i=0;i<14;i++)
                              printf("%d,",info1[i]);
                               printf("\n in");*/
   //  out_14bytes[0] = (uint8_t)(raw[18] & 0x0Fu);
      out_14bytes[0] = (uint8_t)((out_14bytes[0])|(raw[19] & 0x0Fu));
     // printf("%d,",info1[i]);
    for (int i = 1; i < 14; i++) {
        out_14bytes[i] = (uint8_t)raw[19+i];
    }
  //  out_14bytes[13] = (uint8_t)((raw[32] & 0x0Fu) << 4);
}


/**
 * @brief Write INFO_1 field from caller buffer into a voice burst raw[].
 *
 * @param raw          33-byte burst buffer
 * @param in_14bytes   14-byte source buffer (108 bits, upper-aligned in byte 13)
 */
static inline void dmr_burst_set_info(uint8_t       *raw,
                                        const uint8_t *in27)
{
    #if 0
    memcpy(raw, in_14bytes, 13);
    /* Preserve lower nibble of raw[13] (SYNC/EMB), write upper nibble */
    raw[13] = (uint8_t)((raw[13]) | (in_14bytes[13] & 0xF0u));
    #endif
    
    
    int k = 0;

    /* INFO_1: raw[0..12] (104 bits) + raw[13][7:4] (4 bits) */
    for (int i = 0; i < 13; i++) {
        uint8_t byte = 0u;
        for (int b = 7; b >= 0; b--) {
            (void)b;
            byte = (uint8_t)((byte << 1) |
                   ((in27[k >> 3] >> (7 - (k & 7))) & 1u));
            k++;
        }
        raw[i] = byte;
    }
    {
        uint8_t hi = 0u;
        for (int b = 0; b < 4; b++) {
            (void)b;
            hi = (uint8_t)((hi << 1) |
                 ((in27[k >> 3] >> (7 - (k & 7))) & 1u));
            k++;
        }
        raw[13] = (uint8_t)((raw[13] & 0x0Fu) | (hi << 4));
    }

    /* INFO_2: raw[19][3:0] (4 bits) + raw[20..32] (104 bits) */
    {
        uint8_t lo = 0u;
        for (int b = 0; b < 4; b++) {
            (void)b;
            lo = (uint8_t)((lo << 1) |
                 ((in27[k >> 3] >> (7 - (k & 7))) & 1u));
            k++;
        }
        raw[19] = (uint8_t)((raw[19] & 0xF0u) | lo);
    }
    for (int i = 20; i <= 32; i++) {
        uint8_t byte = 0u;
        for (int b = 7; b >= 0; b--) {
            (void)b;
            byte = (uint8_t)((byte << 1) |
                   ((in27[k >> 3] >> (7 - (k & 7))) & 1u));
            k++;
        }
        raw[i] = byte;
    }
    /* k == 216 */

}
/**
 * @brief Write INFO_2 field into a voice burst raw[].
 *
 * @param raw          33-byte burst buffer
 * @param in_14bytes   14-byte source buffer (108 bits, upper-aligned)
 */
static inline void dmr_burst_set_info2(uint8_t       *raw,
                                        const uint8_t *in_14bytes)
{
    /* Preserve upper nibble of raw[19] (SYNC/EMB last 4 bits) */
    
    memcpy(&raw[20], &in_14bytes[1], 13);
     raw[19]=raw[19] |(in_14bytes[0]&0x0Fu);
    
  /*  raw[19] = (uint8_t)((raw[19] & 0xF0u) | ((in_14bytes[0] >> 4) & 0x0Fu));
    for (int i = 0; i < 12; i++) {
        raw[20 + i] = (uint8_t)((in_14bytes[i] & 0x0Fu) << 4)
                    | (in_14bytes[i + 1] >> 4);
    }
    raw[32] = (uint8_t)((in_14bytes[13] >> 4) & 0x0Fu);*/
}

/**
 * @brief Clear (zero) a full burst raw[] and set all non-field bytes to 0.
 *        Use this before calling set_sync, set_slot_type, set_info1/2.
 */
static inline void dmr_burst_clear(uint8_t *raw)
{
    memset(raw, 0, DMR_BURST_BYTES);
}

/**
 * @brief Extract the 216-bit (27-byte) voice payload from a voice burst.
 *
 * Voice bursts (A-F) carry 216 bits of AMBE+2 data in two non-contiguous
 * INFO_1 (108 bits: raw[0..12] + raw[13][7:4]) and INFO_2 (108 bits:
 * raw[19][3:0] + raw[20..32]) fields, with the 18-bit EMB field (or SYNC
 * on burst A) occupying raw[13..18] in between (not part of the payload).
 *
 * out27[0] bit7 = VS(215) (first transmitted voice bit).
 *
 * @param raw    33-byte burst raw[] buffer
 * @param out27  Output: 27-byte buffer receiving the 216 voice payload bits
 */
static inline void dmr_burst_get_voice_info(const uint8_t *raw,
                                              uint8_t out27[27])
{
    /* INFO_1: 108 bits, bit-serial from raw[0..13] */
    uint32_t bit_idx = 0u;
    for (uint32_t i = 0u; i < 108u; i++) {
        uint32_t byte_pos = i / 8u;
        uint32_t bit_pos  = 7u - (i % 8u);
        uint32_t out_byte = bit_idx / 8u;
        uint32_t out_bit  = 7u - (bit_idx % 8u);
        uint8_t  val      = (raw[byte_pos] >> bit_pos) & 0x01u;
        if (val) out27[out_byte] |=  (uint8_t)(1u << out_bit);
        else     out27[out_byte] &= (uint8_t)~(1u << out_bit);
        bit_idx++;
    }
    /* Skip the 18-bit EMB/SYNC field at raw[13..18] (bits 108..125) */
    /* INFO_2: 108 bits starting at raw[19] bit3 (vs bit 0=126 of voice) */
    uint32_t raw_bit = 108u + 18u; /* = 126 */
    for (uint32_t i = 0u; i < 108u; i++) {
        uint32_t byte_pos = raw_bit / 8u;
        uint32_t bit_pos  = 7u - (raw_bit % 8u);
        uint32_t out_byte = bit_idx / 8u;
        uint32_t out_bit  = 7u - (bit_idx % 8u);
        uint8_t  val      = (raw[byte_pos] >> bit_pos) & 0x01u;
        if (val) out27[out_byte] |=  (uint8_t)(1u << out_bit);
        else     out27[out_byte] &= (uint8_t)~(1u << out_bit);
        bit_idx++;
        raw_bit++;
    }
}

/**
 * @brief Write the 216-bit (27-byte) voice payload into a voice burst.
 *
 * Inverse of dmr_burst_get_voice_info(). The SYNC/EMB field at
 * raw[13..18] is left untouched; call dmr_burst_set_sync() or
 * dmr_burst_set_emb() separately.
 *
 * @param raw  33-byte burst raw[] buffer (modified in-place)
 * @param in27 27-byte source of the 216 voice payload bits
 */
static inline void dmr_burst_set_voice_info(uint8_t       *raw,
                                              const uint8_t  in27[27])
{
    uint32_t bit_idx = 0u;
    for (uint32_t i = 0u; i < 108u; i++) {
        uint32_t byte_pos = i / 8u;
        uint32_t bit_pos  = 7u - (i % 8u);
        uint32_t in_byte  = bit_idx / 8u;
        uint32_t in_bit   = 7u - (bit_idx % 8u);
        uint8_t  val      = (in27[in_byte] >> in_bit) & 0x01u;
        if (val) raw[byte_pos] |=  (uint8_t)(1u << bit_pos);
        else     raw[byte_pos] &= (uint8_t)~(1u << bit_pos);
        bit_idx++;
    }
    uint32_t raw_bit = 126u;
    for (uint32_t i = 0u; i < 108u; i++) {
        uint32_t byte_pos = raw_bit / 8u;
        uint32_t bit_pos  = 7u - (raw_bit % 8u);
        uint32_t in_byte  = bit_idx / 8u;
        uint32_t in_bit   = 7u - (bit_idx % 8u);
        uint8_t  val      = (in27[in_byte] >> in_bit) & 0x01u;
        if (val) raw[byte_pos] |=  (uint8_t)(1u << bit_pos);
        else     raw[byte_pos] &= (uint8_t)~(1u << bit_pos);
        bit_idx++;
        raw_bit++;
    }
}

/**
 * @brief Determine the Data Type of a data burst by reading the SLOT_TYPE
 *        field with correct non-byte-aligned extraction.
 *
 * @param raw  33-byte burst buffer
 * @return     Data Type nibble (DMR_DTYPE_*), or 0xFF if not a data burst
 */
static inline uint8_t dmr_burst_get_dtype(const uint8_t *raw)
{
    uint8_t cc, dtype;
    uint16_t golay;
    if (dmr_burst_is_data(raw)) {
        dmr_burst_get_slot_type(raw, &cc, &dtype, &golay);
        return dtype;
    }
    return 0xFFu; /* not a data burst */
}

/**
 * @brief Get the Colour Code from a data burst SLOT_TYPE field.
 */
static inline uint8_t dmr_burst_get_cc(const uint8_t *raw)
{
    uint8_t cc, dtype;
    uint16_t golay;
    dmr_burst_get_slot_type(raw, &cc, &dtype, &golay);
    return cc;
}

#ifdef __cplusplus
}
#endif

#endif /* DMR_PDU_H */