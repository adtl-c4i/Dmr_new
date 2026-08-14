/**

/**

/**
 * @file dmr_ccl_data.h
 * @brief MOD-06 — Call Control Layer (CCL) Data Services — Public Interface
 *
 * Implements the MS-side CCL state machine for packet data services as
 * defined in ETSI TS 102 361-1, Clause 8.2 (Data Header / Data Block PDUs)
 * and TS 102 361-2, Clause 7 (Packet Data Call procedures).
 *
 * Supports:
 *   - Unconfirmed packet data delivery (DPFT=0x02), fragmented into
 *     Rate-1 Data Blocks (11 bytes payload each, max 127 blocks)
 *   - Confirmed packet data delivery (DPFT=0x03) with stop-and-wait ARQ:
 *     Response Header (ACK/NACK, DPFT=0x01) and bounded retransmission
 *   - Preamble CSBK (CSBKO=0x3D) announcement before multi-block transfers
 *   - RX reassembly of inbound data headers + Rate-1 blocks into a
 *     contiguous datagram, delivered to the application via callback
 *   - Automatic ACK generation for inbound confirmed transfers
 *
 * Deferred to future work (not implemented in this module):
 *   - SAP 0x02 UDP/IP header compression
 *   - UDT Short Data / SDS (DPFT=0x00)
 *   - Multi-PDU send-sequence windows (N(S)/N(R) sliding window) — this
 *     module uses a single outstanding transfer with stop-and-wait ARQ
 *
 * Linux POSIX:
 *   - Each CCL Data instance runs as a pthread (mirrors MOD-05 CCL Voice)
 *   - MAC interaction via POSIX message queues (mqueue.h)
 *   - Retry/response timeout via timerfd_create
 *
 * NOTE on mqueue topology: this module attaches to the REAL, shared MAC
 * queues for its slot (DMR_MQ_MAC_TX_REQ/CONF/RX_BURST_S1 or _S2, from
 * dmr_mac.h) — the same queues CCL Voice and Tier III Trunking use on
 * that slot. MAC is the sole creator of those queues (see the ownership
 * contract in dmr_mac.h); this module only ever opens them, with a
 * bounded retry if MAC hasn't created them yet. mq_evt is the one
 * queue this module creates/owns itself, scoped per slot. MAC fans RX
 * bursts out to whichever CCL instance the Data Type addresses (Voice
 * LC / Terminator / EMB → CCL Voice; Data Header / Rate-1 Data Block /
 * Response Header → CCL Data); each module also exposes a public
 * ccl_data_rx_burst()-style injection API for direct testing.
 *
 * ETSI References:
 *   TS 102 361-1 Cl. 8.2   — Data Header / Data Block PDU structures
 *   TS 102 361-1 Cl. 9.2   — PDU field encodings
 *   TS 102 361-2 Cl. 7.2.1 — Packet data call establishment (Preamble)
 *   TS 102 361-1 Table 8.3 — Response PDU Class/Type/Status
 */

#ifndef DMR_CCL_DATA_H
#define DMR_CCL_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>

#include "dmr_pdu.h"
#include "dmr_types.h"
#include "dmr_mac.h"
#include "dmr_dmo.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Sizing constants
 * ========================================================================= */
#define CCL_DATA_BLOCK_PAYLOAD   11u    /**< User bytes per Rate-1 data block */
#define CCL_DATA_MAX_PAYLOAD     1024u  /**< Max bytes per data transfer      */
#define CCL_DATA_MAX_BLOCKS      \
    ((CCL_DATA_MAX_PAYLOAD + CCL_DATA_BLOCK_PAYLOAD - 1u) / CCL_DATA_BLOCK_PAYLOAD)
    /* 94 blocks — within the 7-bit (max 127) Blocks-to-Follow field        */

/* =========================================================================
 * CCL Data State Machine States
 * ========================================================================= */
typedef enum {
    CCL_DATA_STATE_IDLE          = 0,  /**< No transfer in progress              */
    CCL_DATA_STATE_TX_BURST_PENDING = 1,/**< One burst submitted, awaiting conf   */
    CCL_DATA_STATE_TX_WAIT_ACK   = 2,  /**< Confirmed TX — awaiting Response Hdr */
    CCL_DATA_STATE_RX_REASSEMBLE = 3,  /**< Inbound header seen, collecting blks */
    CCL_DATA_STATE_TX_WAIT_SACK_DATA = 4,/**< SACK Response Hdr seen — collecting
                                            the C_RDATA bitmap burst(s) that
                                            follow it (Cl.8.2.2.3) before a
                                            selective retry can be built     */
} ccl_data_state_t;

/* TX_PENDING kept as alias for compatibility with existing tests that
 * check the state value directly. */
#define CCL_DATA_STATE_TX_PENDING CCL_DATA_STATE_TX_BURST_PENDING

static const char * const CCL_DATA_STATE_NAMES[] = {
    "IDLE", "TX_BURST_PENDING", "TX_WAIT_ACK", "RX_REASSEMBLE", "TX_WAIT_SACK_DATA"
};

/* =========================================================================
 * CCL Data Events — posted to / consumed by the worker thread
 * ========================================================================= */
typedef enum {
    CCL_DATA_EVT_TX_CONF      = 0,  /**< MAC confirms a submitted TX burst    */
    CCL_DATA_EVT_TX_ABORTED   = 1,  /**< MAC could not transmit a burst       */
    CCL_DATA_EVT_BURST_RECEIVED = 2,/**< MAC delivered a decoded RX burst     */
    CCL_DATA_EVT_TIMER_RESPONSE = 3,/**< T_Response expired (ARQ retry/fail)  */
    CCL_DATA_EVT_SHUTDOWN     = 4,  /**< Graceful shutdown requested          */
} ccl_data_event_type_t;

typedef struct {
    ccl_data_event_type_t type;
    uint64_t              timestamp_us;
    union {
        dmr_burst_t       burst;     /**< For CCL_DATA_EVT_BURST_RECEIVED    */
        dmr_mac_tx_conf_t tx_conf;   /**< For CCL_DATA_EVT_TX_CONF/ABORTED   */
    } u;
} ccl_data_event_t;

/* POSIX mqueue names — CCL Data instance, per slot.
 *
 * mq_evt is private to this module — CCL Data creates/owns it. The
 * MAC-facing queues (mq_mac_tx/conf/rx) are NOT defined here — this
 * module uses the real, shared MAC queue names from dmr_mac.h
 * (DMR_MQ_MAC_TX_REQ_S1/S2 etc.), the same ones CCL Voice and Tier III
 * Trunking attach to on the same slot. See the ownership contract in
 * dmr_mac.h: MAC is the sole creator of those queues; this module must
 * never pass O_CREAT for them. */
#define DMR_MQ_CCL_DATA_EVT_S1      "/dmr_ccl_data_evt_s1"
#define DMR_MQ_CCL_DATA_EVT_S2      "/dmr_ccl_data_evt_s2"

/* =========================================================================
 * Timers — ETSI TS 102 361-2 Clause 7 (T_Response, ARQ retry)
 * ========================================================================= */
#define CCL_DATA_T_RESPONSE_MS   2000u  /**< Wait for Response Hdr after TX  */
#define CCL_DATA_MAX_RETRIES     3u     /**< Max retransmissions (confirmed) */

/**
 * Inbound-reassembly stall watchdog — NOT an ETSI-specified timer (the
 * spec bounds the *sender's* retry behaviour via T_Response/max retries,
 * but does not itself specify how long a receiver should wait for a
 * stalled reassembly before abandoning it). Exists so
 * CCL_DATA_STATE_RX_REASSEMBLE cannot be occupied forever if a peer's
 * blocks stop arriving and the peer never gets to (or never manages to)
 * tell us it has given up.
 *
 * Re-armed on every block arrival (genuine progress OR a duplicate
 * retransmission — either way, evidence the peer is still trying),
 * not fixed to the total transfer duration, so a slow-but-progressing
 * transfer near the deadline is not penalised for its length. Sized to
 * outlast a full sender-side retry envelope: the sender's own
 * T_Response wait recurs up to (CCL_DATA_MAX_RETRIES + 1) times before
 * it gives up (2000ms x 4 = 8000ms in the worst case with default
 * timing), so this is set comfortably above that with headroom rather
 * than tied 1:1 to CCL_DATA_T_RESPONSE_MS, which times a different
 * thing (the TX side's own single wait, not the RX side's tolerance
 * for the sender's entire retry cycle).
 */
#define CCL_DATA_T_RX_STALL_MS   10000u

/* Response Header Class/Type/Status — TS 102 361-1 Table 8.3. Public
 * so applications and tests can interpret/construct response bursts;
 * the implementation (dmr_ccl_data.c) sends exactly one of these per
 * confirmed transfer, after the whole transfer concludes — never per
 * intermediate block. */
#define CCL_DATA_RESP_CLASS_ACK              0u  /* 00 */
#define CCL_DATA_RESP_CLASS_NACK             1u  /* 01 */
#define CCL_DATA_RESP_CLASS_SACK             2u  /* 10 — selective retry; see ccl_data_send_sack() */

#define CCL_DATA_RESP_TYPE_ACK               0u  /* 000: ACK                              */
#define CCL_DATA_RESP_TYPE_NACK_ILLEGAL      0u  /* 000: Illegal format                   */
#define CCL_DATA_RESP_TYPE_NACK_CRC          1u  /* 001: Packet CRC failed                */
#define CCL_DATA_RESP_TYPE_NACK_MEMORY_FULL  2u  /* 010: Recipient memory full            */
#define CCL_DATA_RESP_TYPE_NACK_OUT_OF_SEQ_FSN 3u/* 011: Received FSN out of sequence     */
#define CCL_DATA_RESP_TYPE_NACK_UNDELIVERABLE 4u /* 100: Undeliverable                    */
#define CCL_DATA_RESP_TYPE_NACK_OUT_OF_SEQ_PKT 5u/* 101: Packet out of sequence (N(S))    */
#define CCL_DATA_RESP_TYPE_NACK_DISALLOWED   6u  /* 110: Invalid user disallowed          */

/* =========================================================================
 * Outbound transfer context
 *
 * TX dispatch is paced: one burst is submitted to MAC at a time. After
 * MAC confirms it (TX_CONF on mq_mac_conf), the worker thread submits the
 * next burst via ccl_data_tx_next_burst(). This prevents the MAC TX queue
 * from overflowing and ensures each burst is actually transmitted before
 * the next is queued — not all bursts dumped in a tight loop that
 * overwrites queue capacity before MAC can drain them.
 *
 * Dispatch sequence: PREAMBLE(0) → HEADER(1) → BLOCK_0(2) → … → BLOCK_N
 * tracked by tx_phase and tx_block_idx.
 * ========================================================================= */
/**
 * Which header family a TX transfer uses — selects which builder
 * ccl_data_tx_next_burst() calls in CCL_DATA_TX_PHASE_HEADER, and (for
 * STATUS_PRECODED) forces n_blocks=0 so the header is the only burst.
 * TS 102 361-3 Cl.6: Short Data reuses the IP bearer service's DLL
 * transport (Data Block/Last-Data-Block PDUs, ARQ, C_RHEAD response)
 * verbatim — only the Data Header PDU and SAP differ.
 */
typedef enum {
    CCL_DATA_SD_KIND_IP               = 0, /**< U_HEAD/C_HEAD — existing IP bearer transfer */
    CCL_DATA_SD_KIND_RAW              = 1, /**< R_HEAD  — Raw short data, port-addressed     */
    CCL_DATA_SD_KIND_DEFINED          = 2, /**< DD_HEAD — Defined short data, format code    */
    CCL_DATA_SD_KIND_STATUS_PRECODED  = 3, /**< SP_HEAD — header-only, no data blocks        */
} ccl_data_sd_kind_t;

typedef enum {
    CCL_DATA_TX_PHASE_PREAMBLE = 0,  /**< Next burst to send is the Preamble CSBK */
    CCL_DATA_TX_PHASE_HEADER   = 1,  /**< Next burst to send is the Data Header    */
    CCL_DATA_TX_PHASE_BLOCKS   = 2,  /**< Next burst to send is a Rate-1 block     */
    CCL_DATA_TX_PHASE_DONE     = 3,  /**< All bursts submitted                     */
} ccl_data_tx_phase_t;

typedef struct {
    bool      active;
    bool      confirmed;
    bool      is_group;
    uint32_t  dst_id;
    uint8_t   sap;
    uint8_t   fsn;            /**< Fragment Sequence Number (4 bits)        */
    uint8_t   data[CCL_DATA_MAX_PAYLOAD];
    size_t    data_len;
    uint8_t   n_blocks;
    uint8_t   pad_octets;
    uint8_t   retry_count;
    bool              encrypted;       /**< Privacy indicator from LC                 */


    /* Short Data (TS 102 361-3 Cl.6) — which header family this
     * transfer uses, and the fields specific to that family. Unused
     * (left zero) when sd_kind == CCL_DATA_SD_KIND_IP. */
    ccl_data_sd_kind_t sd_kind;
    uint8_t   src_port;      /**< RAW/STATUS_PRECODED — 3-bit source port  */
    uint8_t   dst_port;      /**< RAW/STATUS_PRECODED — 3-bit dest port    */
    uint8_t   dd_format;     /**< DEFINED — 6-bit predefined format code   */
    uint16_t  status_value;  /**< STATUS_PRECODED — 10-bit status code     */

    /* Paced dispatch state — updated after each TX_CONF */
    ccl_data_tx_phase_t tx_phase;    /**< Which burst type to send next        */
    uint8_t   tx_block_idx;  /**< Index of next Rate-1 block to send (0-based)*/
    uint32_t  pending_req_id;/**< req_id of the one outstanding burst          */

    uint32_t  last_req_id;   /**< req_id of final block — gates ACK wait      */
    bool      last_conf_seen;/**< true once final block TX is confirmed        */

    /* Selective retry (SACK, TS 102 361-1 Cl.8.2.2.3 / TS 102 361-3
     * Cl.5.4.3-6.5) — populated once ccl_data_rx_response() sees a
     * Class=SACK reply to our outstanding confirmed TX. sack_pending
     * gates ccl_data_rx_burst()'s LLC_RX_DATA_BLOCK routing: while
     * true, an arriving Rate-1-shaped burst is the peer's C_RDATA
     * bitmap, not inbound reassembly payload, and is handled by
     * ccl_data_rx_sack_data() instead of ccl_data_rx_block(). */
    bool      sack_pending;
    uint8_t   sack_bursts_expected; /**< 1 or 2, from the Response Hdr's
                                          blocks-to-follow (Cl.8.2.2.3)     */
    uint8_t   sack_bursts_received;
    uint64_t  sack_retry_flags[2];  /**< Accumulated bitmap(s), MSB-first:
                                          bit (63-i) of [half] set = block
                                          (half*64+i) was received OK, no
                                          retry needed (see
                                          ccl_data_send_sack()/
                                          ccl_data_rx_sack_data())        */
    uint8_t   sack_retry_list[CCL_DATA_MAX_BLOCKS]; /**< DBSNs still needing
                                          retransmission, built once both
                                          expected bitmap bursts have
                                          arrived; walked by tx_block_idx
                                          during the retry itself         */
    uint8_t   sack_retry_count;     /**< Valid entries in sack_retry_list  */
    bool      sack_retry_active;    /**< true only while actively
                                          transmitting the selective retry
                                          built from sack_retry_list — distinct
                                          from sack_pending (which means
                                          "waiting to receive the bitmap");
                                          gates ccl_data_tx_next_burst()'s
                                          HEADER (full_msg=false) and BLOCKS
                                          (index via sack_retry_list, not i
                                          directly) phases                */
} ccl_data_tx_ctx_t;

/* =========================================================================
 * Inbound reassembly context
 * ========================================================================= */
typedef struct {
    bool      active;
    bool      confirmed;
    bool      is_group;
    uint8_t   sap;
    uint32_t  dst_id;
    uint32_t  src_id;
    uint8_t   total_blocks;
    uint8_t   blocks_received;
    size_t    total_len;
    uint8_t   buffer[CCL_DATA_MAX_PAYLOAD];
    bool              encrypted;       /**< Privacy indicator from LC                 */

    /* Short Data (TS 102 361-3 Cl.6) — which header family this
     * reassembly uses, and its type-specific fields (unused/0 when
     * sd_kind == CCL_DATA_SD_KIND_IP). Selects which callback fires
     * on completion — on_data_received() for IP, on_short_data_received()
     * otherwise (see ccl_data_rx_block()'s completion branch). */
    ccl_data_sd_kind_t sd_kind;
    uint8_t   src_port;
    uint8_t   dst_port;
    uint8_t   dd_format;
    uint16_t  status_value;

    /* Failure tracking for the single end-of-transfer Response Header
     * (TS 102 361-1 Table 8.3) — set the first time a problem is
     * detected; later blocks no longer change it, since only one
     * response is ever sent per transfer. seen[] guards against a
     * duplicate/retransmitted block being miscounted as out-of-sequence
     * or double-counted toward blocks_received. */
    bool      failed;
    uint8_t   fail_type;    /**< CCL_DATA_RESP_TYPE_NACK_* once failed=true */
    uint8_t   fail_status;
    bool      seen[CCL_DATA_MAX_BLOCKS];

    /* Outgoing SACK response — paced dispatch, mirroring ctx->tx's
     * PREAMBLE/HEADER/BLOCKS phase machine (ccl_data_tx_next_burst()/
     * ccl_data_handle_tx_conf()) rather than firing every burst in one
     * call the way ccl_data_send_sack() originally did. sack_tx_active
     * gates a dedicated confirmation check (separate from
     * ctx->tx.pending_req_id, which belongs to the unrelated outbound-
     * transfer sequence) so a SACK response we're sending as the
     * receiver never gets confused with a transfer we're sending as
     * the sender — both can be genuinely concurrent on the same ctx. */
    bool      sack_tx_active;
    uint8_t   sack_tx_phase;      /**< 0=Response Hdr, 1=C_RDATA burst 1,
                                        2=C_RDATA burst 2 (only if
                                        sack_tx_bursts_expected==2)      */
    uint8_t   sack_tx_bursts_expected; /**< 1 or 2 C_RDATA bursts total,
                                        same blocks_to_follow value the
                                        Response Hdr itself carries      */
    uint32_t  sack_tx_pending_req_id;
    uint32_t  sack_tx_peer_id;
    uint8_t   sack_tx_sap;
    /* seen[]/total_blocks above are read directly when building each
     * C_RDATA burst's bitmap — no separate copy needed, since rx stays
     * alive (not reset) for the whole SACK-send sequence, same as it
     * already does while waiting for the retry that follows. */
} ccl_data_rx_ctx_t;

/* =========================================================================
 * Statistics
 * ========================================================================= */
typedef struct {
    uint64_t tx_transfers;
    uint64_t tx_bytes;
    uint64_t tx_blocks;
    uint64_t tx_retries;
    uint64_t tx_failures;
    uint64_t rx_transfers;
    uint64_t rx_bytes;
    uint64_t rx_blocks;
    uint64_t rx_acks_sent;
    uint64_t rx_nacks_sent;
} ccl_data_stats_t;

/* =========================================================================
 * CCL Data Instance — one per TDMA timeslot
 * ========================================================================= */
typedef struct ccl_data_ctx {
    /* Identity */
    dmr_slot_t          slot;
    uint32_t            my_radio_id;
    uint8_t             colour_code;
    uint8_t             fid;
    /* State machine */
    ccl_data_state_t    state;
    pthread_mutex_t     state_mutex;

    /* Transfer contexts */
    ccl_data_tx_ctx_t   tx;
    ccl_data_rx_ctx_t   rx;

    uint32_t            tx_req_id_next;

    /* Configurable response timeout (ms) — overridable for fast tests */
    uint32_t            t_response_ms;

    /* Configurable RX-reassembly stall timeout (ms) — overridable for
     * fast tests, same pattern as t_response_ms above. */
    uint32_t            t_rx_stall_ms;

    /* POSIX message queues */
    mqd_t               mq_evt;       /**< Inbound worker events            */
    mqd_t               mq_mac_tx;    /**< TX burst requests → MAC          */
    mqd_t               mq_mac_conf;  /**< TX confirmations ← MAC           */
    mqd_t               mq_mac_rx;    /**< RX bursts ← MAC (decoded)        */

    /* Retry / response timer */
    dmr_phy_timer_oneshot_t                 tmr_response;

    /* RX-reassembly stall watchdog — see CCL_DATA_T_RX_STALL_MS's doc
     * comment for why this exists and how it's re-armed. */
    dmr_phy_timer_oneshot_t                 tmr_rx_stall;


    /* Worker thread */
    pthread_t           thread;
    volatile bool       running;

    /* Callbacks (optional) */
    void (*on_data_received)(struct ccl_data_ctx *ctx,
                              uint32_t src_id, uint32_t dst_id,
                              bool is_group, uint8_t sap,
                              const uint8_t *data, size_t len);
    /**
     * @brief Short Data (TS 102 361-3 Cl.6) reception — Raw, Defined,
     *        or Status/Precoded. kind distinguishes which; fields not
     *        applicable to the received kind are 0 (e.g. data/len are
     *        0/NULL for STATUS_PRECODED, whose payload is status_value
     *        alone; dd_format is 0 for RAW; ports are 0 for DEFINED).
     */
    void (*on_short_data_received)(struct ccl_data_ctx *ctx,
                                    ccl_data_sd_kind_t kind,
                                    uint32_t src_id, uint32_t dst_id,
                                    bool is_group,
                                    uint8_t src_port, uint8_t dst_port,
                                    uint8_t dd_format, uint16_t status_value,
                                    const uint8_t *data, size_t len);
    void (*on_tx_complete)(struct ccl_data_ctx *ctx,
                            bool success, uint8_t retries);

    /* Statistics */
    ccl_data_stats_t    stats;

    /* Optional MOD-15 (DCDM) back-reference — non-NULL only when this
     * MS is DMR_TIER_2_CONVENTIONAL with dcdm_enabled=true (wired by
     * dmr_ms.c after dmr_dmo_init() succeeds). Used to send the
     * appropriate CT_CSBK_Term after a data TX completes, per
     * Cl.6.2.2.3.3 — except a confirmed-data Response Data Header
     * (this MS responding as the data TARGET), which is explicitly
     * excluded; only the source side's TD_LC gets a Term. */
    dmr_dmo_ctx_t      *dcdm;

    /* MAC back-reference, wired by dmr_ms.c — used alongside dcdm to
     * query mac_slot_is_busy() on the *other* slot for the DCDM
     * Transmit procedure's channel_activity argument (Cl.6.2.3.12). */
    mac_ctx_t          *mac;
} ccl_data_ctx_t;

/* =========================================================================
 * CCL Data API — public functions
 * ========================================================================= */

/**
 * @brief Initialise a CCL Data instance (opens mqueues/timerfd, zeroes state).
 */
dmr_err_t ccl_data_init(ccl_data_ctx_t *ctx,
                          dmr_slot_t      slot,
                          uint32_t        my_radio_id,
                          uint8_t         colour_code);

/**
 * @brief Start the worker thread.
 */
dmr_err_t ccl_data_start(ccl_data_ctx_t *ctx);

/**
 * @brief Request graceful shutdown of the worker thread and join it.
 */
dmr_err_t ccl_data_stop(ccl_data_ctx_t *ctx);

/**
 * @brief Close mqueues/timerfd and zero the context. Call after ccl_data_stop().
 */
void ccl_data_destroy(ccl_data_ctx_t *ctx);

/**
 * @brief Submit an unconfirmed data transfer (DPFT=0x02).
 *
 * Builds a Preamble CSBK + Unconfirmed Data Header + N Rate-1 Data Blocks
 * and submits them to the MAC TX request queue. Returns DMR_OK once all
 * bursts are enqueued (does not block for over-the-air completion).
 *
 * @param data,len  User payload, max CCL_DATA_MAX_PAYLOAD bytes
 * @return DMR_OK, DMR_ERR_INVALID (len==0 or too large), DMR_ERR_BUSY
 *         (a transfer is already in progress)
 */
dmr_err_t ccl_data_tx_unconfirmed(ccl_data_ctx_t *ctx,
                                    uint32_t dst_id, bool is_group,
                                    uint8_t sap,
                                    const uint8_t *data, size_t len);

/**
 * @brief Submit a confirmed data transfer (DPFT=0x03) with stop-and-wait ARQ.
 *
 * Like ccl_data_tx_unconfirmed(), but the A-bit (request-ack) is set. After
 * the final block is confirmed transmitted, T_Response starts; on a
 * Response Header ACK from the peer, on_tx_complete(ctx,true,retries) is
 * invoked. On NACK or T_Response timeout the whole header+blocks sequence
 * is retransmitted up to CCL_DATA_MAX_RETRIES times, after which
 * on_tx_complete(ctx,false,retries) is invoked.
 */
dmr_err_t ccl_data_tx_confirmed(ccl_data_ctx_t *ctx,
                                  uint32_t dst_id, bool is_group,
                                  uint8_t sap,
                                  const uint8_t *data, size_t len);

/* =========================================================================
 * Short Data (TS 102 361-3 Cl.6) — same DLL transport/ARQ/response as
 * the IP bearer functions above (paced dispatch, retry, C_RHEAD), only
 * the Data Header PDU and SAP differ. Confirmed/unconfirmed selection
 * is by req_ack, matching every Short Data header's A-bit convention
 * (Cl.6.4: A=0 unconfirmed, A=1 confirmed — stop-and-wait only, no
 * sliding window for any Short Data type).
 * ========================================================================= */

/**
 * @brief Submit a Raw Data transfer (R_HEAD, DPFT=0x0E) — application-
 *        defined payload, addressed by 3-bit source/destination ports
 *        rather than SAP (TS 102 361-1 Table 9.17B).
 */
dmr_err_t ccl_data_tx_raw(ccl_data_ctx_t *ctx,
                           uint32_t dst_id, bool is_group,
                           uint8_t src_port, uint8_t dst_port,
                           bool req_ack,
                           const uint8_t *data, size_t len);

/**
 * @brief Submit a Defined Data transfer (DD_HEAD, DPFT=0x0D) — payload
 *        structure identified by a 6-bit predefined format code rather
 *        than ports (TS 102 361-1 Table 9.17C).
 */
dmr_err_t ccl_data_tx_defined(ccl_data_ctx_t *ctx,
                               uint32_t dst_id, bool is_group,
                               uint8_t dd_format,
                               bool req_ack,
                               const uint8_t *data, size_t len);

/**
 * @brief Submit a Status/Precoded message (SP_HEAD, DPFT=0x0E, AB=0) —
 *        the entire message is this one 12-byte header; there are no
 *        data blocks (TS 102 361-1 Table 9.17A, TS 102 361-3 Cl.6.3).
 *
 * @param status_value  10-bit status/precoded code (0-1023)
 */
dmr_err_t ccl_data_tx_status_precoded(ccl_data_ctx_t *ctx,
                                       uint32_t dst_id, bool is_group,
                                       uint8_t src_port, uint8_t dst_port,
                                       uint16_t status_value,
                                       bool req_ack);

/**
 * @brief Inject a decoded RX burst into the CCL Data state machine.
 *
 * May be called directly (e.g. by a test harness, or a MAC fan-out layer)
 * or internally by the worker thread when reading mq_mac_rx. Dispatches
 * via llc_rx_dispatch(); handles Data Header (starts/identifies a
 * reassembly), Rate-1 Data Block (accumulates payload, delivers via
 * on_data_received() when complete, sends ACK if confirmed), and
 * Response Header (ACK/NACK for our own outstanding confirmed TX).
 *
 * @return DMR_OK always (errors are reflected in stats/callbacks)
 */
dmr_err_t ccl_data_rx_burst(ccl_data_ctx_t *ctx, const dmr_burst_t *burst);

/**
 * @brief Copy out current statistics (thread-safe).
 */
void ccl_data_get_stats(ccl_data_ctx_t *ctx, ccl_data_stats_t *out);

/**
 * @brief Copy out current state (thread-safe).
 */
ccl_data_state_t ccl_data_get_state(ccl_data_ctx_t *ctx);

/**
 * @brief Package-private: build and submit the full PDU sequence for ctx->tx.
 *        Exposed for test harness use only; not part of the public API.
 */
dmr_err_t ccl_data_submit_transfer(ccl_data_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DMR_CCL_DATA_H */