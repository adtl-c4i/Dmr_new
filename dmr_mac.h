/**
 * @file dmr_mac.h
 * @brief MOD-03 — Medium Access Control (MAC) — Public Interface
 *
 * ETSI TS 102 361-1, Clauses 5.2–5.4
 *
 * Responsibilities:
 *   - Listen-Before-Transmit (LBT) polite protocol
 *   - T_IdleSrch / T_Holdoff / T_DataTxLmt timers
 *   - CACH PDU encode/decode (AT, TC, LCSS, Short Data)
 *   - TDMA slot scheduling (30 ms windows)
 *   - Reverse Channel (RC) burst transmission
 *   - Channel access state machine per slot
 *
 * Thread model:
 *   Each slot runs an independent mac_ctx_t instance in its own pthread.
 *   CCL submits TX requests via POSIX mqueue (non-blocking).
 *   MAC delivers decoded RX bursts to CCL via POSIX mqueue.
 *   PHY layer delivers raw bursts to MAC via the burst_rx queue.
 *
 * Queue topology:
 *
 *   [PHY RX] --q_phy_rx_s{1,2}--> [MAC task_mac_slot{1,2}]
 *                                       |            |
 *                      q_mac_rx_burst_s{1,2}    q_mac_tx_req_s{1,2}
 *                                       |            |
 *                                   [CCL task]   [CCL task]
 *                           q_mac_tx_conf_s{1,2}<---/
 *
 * ETSI References:
 *   TS 102 361-1 Cl. 5.2  — LBT / channel access
 *   TS 102 361-1 Cl. 5.4  — CACH structure
 *   TS 102 361-1 Cl. 9.1.3 — SLOT TYPE
 *   TS 102 361-1 Cl. 9.1.4 — TACT / CACH PDU
 */

#ifndef DMR_MAC_H
#define DMR_MAC_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>

#include "dmr_pdu.h"
#include "dmr_types.h"
#include "dmr_tier.h"
#include "dmr_phy.h"
#ifdef __cplusplus
extern "C" {
#endif
//#define TEST_CODE
/* =========================================================================
 * MAC Timer values — ETSI TS 102 361-1, Annex F
 * ========================================================================= */
#define MAC_T_IDLE_SRCH_MS      30u     /* T_IdleSrch  — 1 burst (30 ms)            */
#define MAC_T_HOLDOFF_MAX_MS   120u     /* T_Holdoff   — random 0..4 bursts (120 ms) */
#define MAC_T_DATA_TX_LMT_MS   120u     /* T_DataTxLmt — 4 bursts max wait (120 ms)  */
#define MAC_T_GUARD_US        1000u     /* T_Guard     — RX→TX guard (1.0 ms)        */
#define MAC_T_RAMP_UP_US       500u     /* T_RampUp    — PA ramp-up (0.5 ms)         */
#define MAC_T_RAMP_DOWN_US     500u     /* T_RampDown  — PA ramp-down (0.5 ms)       */
#define MAC_TIMESLOT_MS         30u     /* TDMA timeslot duration                    */
#define MAC_MAX_HOLDOFF_BURSTS   4u     /* Maximum random holdoff burst count        */

/* =========================================================================
 * SOURCE OF TX REQ
 * ========================================================================= */
#define  CCL_TX_ORIGIN_VOICE        0x01
#define CCL_TX_ORIGIN_DATA        0x02
#define T3_TX_ORIGIN_TRUNK        0x03
#define DCDM_TX_ORIGIN_DCDM       0x04

//#define PHY_SIM

void stub_mac_send_tx_burst(dmr_burst_t *out_burst);


/* =========================================================================
 * Channel Access Priority
 * ========================================================================= */
typedef enum {
    DMR_MAC_PRIORITY_LOW    = 0,   /* Non-time-critical data, deferred TX           */
    DMR_MAC_PRIORITY_NORMAL = 1,   /* Normal voice / CSBK — polite LBT             */
    DMR_MAC_PRIORITY_HIGH   = 2,   /* High-priority / in-call re-transmission       */
    DMR_MAC_PRIORITY_EMERG  = 3,   /* Emergency — bypasses holdoff                  */
} dmr_mac_priority_t;

/* =========================================================================
 * MAC Channel Access State Machine
 * ETSI TS 102 361-1, Clause 5.2
 * ========================================================================= */
typedef enum {
    MAC_STATE_IDLE_MONITOR  = 0,   /* Monitoring channel; RSSI below threshold      */
    MAC_STATE_QUALIFY_IDLE  = 1,   /* Running T_IdleSrch — verifying channel idle   */
    MAC_STATE_HOLDOFF       = 2,   /* Random T_Holdoff backoff after busy detect    */
    MAC_STATE_TX_PENDING    = 3,   /* Channel idle; waiting for TDMA slot window    */
    MAC_STATE_TRANSMITTING  = 4,   /* Burst actively being transmitted              */
    MAC_STATE_TX_ABORT      = 5,   /* T_DataTxLmt expired — abort TX attempt        */
} mac_ch_access_state_t;

static const char * const MAC_STATE_NAMES[] = {
    "IDLE_MONITOR", "QUALIFY_IDLE", "HOLDOFF",
    "TX_PENDING", "TRANSMITTING", "TX_ABORT"
};

/* =========================================================================
 * TX Request — submitted by CCL to MAC (via mqueue)
 * ========================================================================= */
typedef struct {
    dmr_burst_t        burst;          /* Complete 264-bit burst to transmit         */
    dmr_slot_t         slot;           /* DMR_SLOT_1 or DMR_SLOT_2                   */
    dmr_mac_priority_t priority;       /* Access priority                            */
    uint64_t           deadline_us;    /* Latest acceptable TX time (0=no deadline)  */
    uint32_t           req_id;         /* Caller-assigned ID for confirmation match  */
    uint8_t           originated_from;         /* source of packet generated(ccl_data, ccl_voice or t3_trunk)  */
    bool               impolite;       /* true = skip LBT (in-call retransmission)   */
} dmr_mac_tx_req_t;

/* =========================================================================
 * TX Confirmation — MAC → CCL result notification (via mqueue)
 * ========================================================================= */
typedef enum {
    DMR_MAC_TX_OK        = 0,  /* Burst was transmitted on schedule                 */
    DMR_MAC_TX_ABORTED   = 1,  /* T_DataTxLmt expired before TX opportunity         */
    DMR_MAC_TX_CANCELLED = 2,  /* Cancelled via mac_tx_cancel()                     */
    DMR_MAC_TX_DEADLINE  = 3,  /* Deadline passed before TX opportunity             */
} dmr_mac_tx_result_t;

typedef struct {
    uint32_t            req_id;        /* Matches dmr_mac_tx_req_t.req_id            */
    dmr_mac_tx_result_t result;
    uint64_t            actual_tx_us;  /* CLOCK_MONOTONIC timestamp of actual TX     */
} dmr_mac_tx_conf_t;

/* =========================================================================
 * CACH slot activity map — runtime tracking, not transmitted
 * ========================================================================= */
typedef struct {
    bool     slot1_busy;     /* AT bit for slot 1 from last CACH                    */
    bool     slot2_busy;     /* AT bit for slot 2 from last CACH                    */
    bool     prev_slot1_busy;/* AT bit for slot 1 from the CACH before last         */
    bool     prev_slot2_busy;/* AT bit for slot 2 from the CACH before last         */
    uint8_t  active_slot;    /* TC bit: which slot the CACH described               */
    uint64_t last_update_us; /* Timestamp of last CACH reception                    */
} mac_slot_activity_t;

/* =========================================================================
 * POSIX message queue names
 *
 * OWNERSHIP CONTRACT (critical — read before adding a new queue or a new
 * consumer of an existing one):
 *
 *   MAC is the SOLE creator (O_CREAT) of every queue in this section.
 *   mac_init() must run to completion (and therefore must be called)
 *   before any CCL module (CCL Voice, CCL Data, Tier III Trunking, ...)
 *   calls its own *_init(). All CCL-side modules open these queues
 *   WITHOUT O_CREAT, using only the minimal directional flag they need
 *   (O_RDONLY or O_WRONLY — never O_RDWR on a queue whose direction is
 *   fixed by the comment below).
 *
 *   Rationale: POSIX mq_open() ignores the `attr` argument unless the
 *   call is the one that actually creates the queue. If two different
 *   translation units each call mq_open(name, O_CREAT, ..., &attr) with
 *   different mq_maxmsg/mq_msgsize values for the SAME name, whichever
 *   call runs first silently wins and the second caller's attr is
 *   discarded with no error — the queue still opens successfully, but
 *   any size mismatch (e.g. one side built with a larger dmr_burst_t
 *   after a struct change) is now invisible until a write is rejected
 *   or read data is misinterpreted. Single ownership removes the
 *   ambiguity: there is exactly one mq_attr per queue, defined once
 *   below, used once at creation time in mac_init().
 *
 *   If a CCL-side module calls mq_open() before MAC has created the
 *   queue, the open fails with ENOENT. CCL init functions retry with a
 *   short bounded backoff (see CCL_MQ_OPEN_RETRY_* in each CCL module)
 *   rather than failing immediately, to tolerate process/thread startup
 *   ordering races without requiring a hard barrier between MAC and CCL
 *   startup.
 * ========================================================================= */
#define DMR_MQ_PHY_RX_S1        "/dmr_phy_rx_s1"       /* PHY  → MAC slot1         */
#define DMR_MQ_PHY_RX_S2        "/dmr_phy_rx_s2"       /* PHY  → MAC slot2         */
#define DMR_MQ_MAC_TX_REQ_S1    "/dmr_mac_tx_req_s1"   /* CCL  → MAC slot1 TX req  */
#define DMR_MQ_MAC_TX_REQ_S2    "/dmr_mac_tx_req_s2"   /* CCL  → MAC slot2 TX req  */
/* RX burst delivery — ONE QUEUE PER DESTINATION MODULE, not one shared
 * queue. POSIX message queues are single-consumer: mq_receive() hands
 * a message to exactly one reader, so if CCL Voice, CCL Data, and
 * Tier III Trunking all called mq_receive() on the same queue name,
 * whichever of the three happened to win a given message would
 * silently "steal" it from the others — there is no broadcast/fan-out
 * with a shared mqueue. MAC therefore classifies every decoded RX
 * burst by Data Type (and, for CSBK, by Data Type + CSBK Opcode + the
 * tier MAC was configured for — see mac_classify_rx_burst() in
 * mac_channel_access.c) and posts it to exactly one of the three
 * queues below. Each CCL-side module opens ONLY its own queue.
 *
 * A burst MAC cannot classify (unrecognised Data Type, or a CSBK
 * opcode not in the classification table for this tier) is dropped
 * with a warning log rather than guessed at — silently misrouting a
 * burst to the wrong module is worse than dropping it. */
 #define DMR_MQ_MAC_TX_CONF_VOICE_S1   "/dmr_mac_tx_conf_voice_s1"  /* CCL -> MAC slot1 TX voice conf */
#define DMR_MQ_MAC_TX_CONF_VOICE_S2   "/dmr_mac_tx_conf_voice_s2"  //* CCL -> MAC slot2 TX voice conf */
#define DMR_MQ_MAC_TX_CONF_DATA_S1   "/dmr_mac_tx_conf_data_s1"  /* CCL -> MAC slot1 TX data conf */
#define DMR_MQ_MAC_TX_CONF_DATA_S2   "/dmr_mac_tx_conf_data_s2"  /* CCL -> MAC slot1 TX data conf */
#define DMR_MQ_MAC_TX_CONF_TRUNK_S1   "/dmr_mac_tx_conf_trunk_s1"  /* CCL -> MAC slot1 TX trunk conf */
#define DMR_MQ_MAC_TX_CONF_TRUNK_S2   "/dmr_mac_tx_conf_trunk_s2"  /* CCL -> MAC slot1 TX trunk conf */
#define DMR_MQ_MAC_TX_CONF_DCDM_S1   "/dmr_mac_tx_conf_dcdm_s1"  /* DCDM -> MAC slot1 TX dcdm conf */
#define DMR_MQ_MAC_TX_CONF_DCDM_S2   "/dmr_mac_tx_conf_dcdm_s2"  /* DCDM -> MAC slot2 TX dcdm conf */
#define DMR_MQ_MAC_RX_VOICE_S1  "/dmr_mac_rx_voice_s1" /* MAC → CCL Voice slot1   */
#define DMR_MQ_MAC_RX_VOICE_S2  "/dmr_mac_rx_voice_s2" /* MAC → CCL Voice slot2   */
#define DMR_MQ_MAC_RX_DATA_S1   "/dmr_mac_rx_data_s1"  /* MAC → CCL Data slot1    */
#define DMR_MQ_MAC_RX_DATA_S2   "/dmr_mac_rx_data_s2"  /* MAC → CCL Data slot2    */
#define DMR_MQ_MAC_RX_TRUNK_S1  "/dmr_mac_rx_trunk_s1" /* MAC → Trunking slot1    */
#define DMR_MQ_MAC_RX_TRUNK_S2  "/dmr_mac_rx_trunk_s2" /* MAC → Trunking slot2    */
#define DMR_MQ_MAC_RX_DCDM_S1   "/dmr_mac_rx_dcdm_s1"  /* MAC → DCDM slot1        */
#define DMR_MQ_MAC_RX_DCDM_S2   "/dmr_mac_rx_dcdm_s2"  /* MAC → DCDM slot2        */
#define DMR_MQ_PHY_TX_S1        "/dmr_phy_tx_s1"       /* MAC  → PHY slot1 TX burst*/
#define DMR_MQ_PHY_TX_S2        "/dmr_phy_tx_s2"       /* MAC  → PHY slot2 TX burst*/
#define DMR_MQ_PHY_TX_CONF_S1        "/dmr_phy_tx_conf_s1"       /* PHY  → MAC slot1 TX burst confirm*/
#define DMR_MQ_PHY_TX_CONF_S2        "/dmr_phy_tx_conf_s2"       /* PHY  → MAC slot2 TX burst confirm*/


/* Canonical depth for every queue MAC creates. This is the ONLY place
 * mq_maxmsg is decided for the MAC↔CCL/PHY queues — do not redefine a
 * competing *_MQ_MAX_MSGS constant in any CCL module for these names. */
#define DMR_MQ_MAX_MSGS          8     /* Max messages in any single queue           */

/* Bounded retry/backoff for CCL-side modules opening a MAC-owned queue
 * before MAC has necessarily finished mac_init(). Total worst-case wait
 * is RETRY_COUNT * RETRY_DELAY_MS = 500ms, which comfortably covers
 * normal process/thread startup scheduling without masking a genuine
 * "MAC was never started" misconfiguration (which still fails after
 * the retries are exhausted). */
#define DMR_MQ_OPEN_RETRY_COUNT     50
#define DMR_MQ_OPEN_RETRY_DELAY_MS  10


/* =========================================================================
 * Voice RX superframe tracking — per-slot state held inside mac_ctx_t.
 *
 * MAC needs to track which bursts it has seen from the current voice
 * superframe so it can:
 *   (a) route B-F voice bursts (EMB replaces SYNC — no SYNC pattern to
 *       match) to mq_rx_voice by temporal inference rather than content,
 *   (b) detect missed B-F burst positions and inject synthetic
 *       MAC_SYNTH_EVT_VOICE_BURST_LOST events so CCL Voice can insert PLC,
 *   (c) detect a missing Terminator LC (call end without explicit close)
 *       via the superframe watchdog timer, and
 *   (d) on Tier II/III, treat a CACH AT Busy→Idle transition as an
 *       additional call-end signal (Tier I: no CACH, never used).
 *
 * Timer values — grounded in ETSI TS 102 361-1 Clause 5.1.2:
 *   Superframe = 6 bursts × 60 ms = 360 ms.
 *   Burst guard (B-F window) = 60 ms + 20 ms jitter = 80 ms.
 *   Superframe watchdog = 360 ms + 40 ms jitter = 400 ms.
 *   Hangover window = one superframe = 400 ms.
 *   Max consecutive missed B-F bursts before HANGOVER = 2 (= 120 ms gap).
 * ========================================================================= */
#define MAC_VOICE_BURST_GUARD_MS        80u   /* B-F burst window (60 ms + jitter)   */ /*this depends on tier.if tier 1 then packet comes without any slot boundary.*/
#define MAC_VOICE_SUPERFRAME_GUARD_MS  400u   /* Superframe watchdog (360 ms + guard)*/
#define MAC_VOICE_HANGOVER_MS          400u   /* Hangover window = one superframe     */
#define MAC_VOICE_MAX_MISSED_BURSTS      2u   /* Consecutive misses → HANGOVER        */

typedef enum {
    MAC_VOICE_RX_IDLE     = 0,  /* No voice call being tracked                */
    MAC_VOICE_RX_LATE_ENTRY    = 1,  /* No voice call being tracked                */
    MAC_VOICE_RX_ACTIVE   = 2,  /* In a voice superframe (burst A received)   */
    MAC_VOICE_RX_HANGOVER = 3,  /* Waiting for resumption or definitive end   */
} mac_voice_rx_state_t;

typedef struct {
    mac_voice_rx_state_t state;
    uint8_t  expected_pos;      /* Next expected burst position: 0=A..5=F     */
    uint8_t  missed_count;      /* Consecutive missed B-F bursts this frame   */
    uint64_t last_burst_us;     /* Timestamp of last voice burst received     */
} mac_voice_rx_ctx_t;

/* =========================================================================
 * MAC context — one instance per TDMA slot
 * ========================================================================= */
typedef struct {
    /* Configuration (set at init, read-only thereafter) */
    dmr_slot_t          slot;
    uint8_t             colour_code;
    uint8_t             my_id[3];          /* 24-bit Radio ID                       */
    dmr_tier_t          tier;              /* Selects the RX classification table   */

    /* Channel access state machine */
    mac_ch_access_state_t ch_state;
    pthread_mutex_t     state_mutex;

    /* Pending TX request (one at a time; queued on mq_tx_req) */
    dmr_mac_tx_req_t    pending_req;
    bool                has_pending;
    uint32_t            holdoff_count;     /* Retry count for current request        */
    unsigned int        rng_state;         /* Per-context seed for rand_r() (holdoff) */
    /* CACH state */
    mac_slot_activity_t slot_activity;
    dmr_cach_pdu_t      last_cach_rx;     /* Last decoded CACH                      */
    dmr_cach_pdu_t      cach_tx;          /* CACH to append on next TX burst        */


   /* Voice RX superframe tracking (see mac_voice_rx_ctx_t above) */
    mac_voice_rx_ctx_t  voice_rx;

    /* POSIX message queues */
    mqd_t               mq_phy_rx;        /* Receive raw bursts from PHY            */
    mqd_t               mq_tx_req;        /* Receive TX requests from CCL           */
    mqd_t               mq_tx_voice_conf;       /* Send TX confirmations to CCL voice          */
    mqd_t               mq_tx_data_conf;       /* Send TX confirmations to CCL  data         */
    mqd_t               mq_tx_trunk_conf;       /* Send TX confirmations to CCL  trunking         */
    mqd_t               mq_tx_dcdm_conf;       /* Send TX confirmations to DCDM (MOD-15)         */


    
    mqd_t               mq_rx_voice;      /* Send voice-classified bursts to CCL Voice */
    mqd_t               mq_rx_data;       /* Send data-classified bursts to CCL Data   */
    mqd_t               mq_rx_trunk;      /* Send trunk-classified bursts to Trunking  */
    mqd_t               mq_rx_dcdm;       /* Send CT_CSBK bursts to DCDM (MOD-15)      */
    mqd_t               mq_phy_tx;        /* Send TX bursts to PHY                  */
    mqd_t               mq_phy_tx_conf;        /* Send TX confirm from PHY to mac                  */

    


    /* Linux timerfd descriptors */
    dmr_phy_timer_oneshot_t                 tmr_idle_srch;    /* T_IdleSrch timer                       */
    dmr_phy_timer_oneshot_t                 tmr_holdoff;      /* T_Holdoff timer                        */
    dmr_phy_timer_oneshot_t                 tmr_tx_lmt;       /* T_DataTxLmt timer                      */
    dmr_phy_timer_oneshot_t                 tmr_voice_burst;  /* 80 ms B-F burst window watchdog        */
    dmr_phy_timer_oneshot_t                 tmr_voice_sf;     /* 400 ms superframe watchdog             */

    /* epoll fd for event loop */
    int                 epoll_fd;

    /* Statistics */
    uint64_t            tx_burst_count;
    uint64_t            rx_burst_count;
    uint64_t            lbt_holdoff_count;
    uint64_t            tx_abort_count;

    /* Worker thread */
    pthread_t           thread;
    volatile bool       running;
} mac_ctx_t;

/* =========================================================================
 * MAC Public API
 * ========================================================================= */

/**
 * @brief Initialise a MAC context for the given TDMA slot.
 *
 * Opens all POSIX message queues, creates timerfd descriptors, initialises
 * mutexes, and sets up the epoll fd. Does NOT start the worker thread.
 *
 * @param ctx          Caller-allocated mac_ctx_t
 * @param slot         DMR_SLOT_1 or DMR_SLOT_2
 * @param colour_code  Active colour code (0-15)
 * @param radio_id     24-bit radio identifier
 * @param tier         Which tier this MS is configured for — selects the
 *                      RX burst classification table (see
 *                      mac_classify_rx_burst() in mac_channel_access.c).
 *                      A CSBK opcode meaningful in one tier but not
 *                      composed in this tier (e.g. a Tier III grant
 *                      opcode arriving on a Tier II MS) is dropped
 *                      rather than misrouted to an unrelated module.
 * @return DMR_OK on success, negative error code on failure
 */
dmr_err_t mac_init(mac_ctx_t *ctx,
                   dmr_slot_t slot,
                   uint8_t    colour_code,
                   uint32_t   radio_id,
                   dmr_tier_t tier);

/**
 * @brief Start the MAC worker thread for this slot.
 */
dmr_err_t mac_start(mac_ctx_t *ctx);

/**
 * @brief Stop the MAC worker thread gracefully and join it.
 */
dmr_err_t mac_stop(mac_ctx_t *ctx);

/**
 * @brief Release all resources held by the MAC context.
 * Must be called after mac_stop().
 */
void mac_destroy(mac_ctx_t *ctx);

/**
 * @brief Submit a burst for channel-accessed transmission (non-blocking).
 *
 * Places req on the MAC TX request queue. MAC runs LBT and transmits
 * the burst in the next available slot window.
 *
 * @param ctx  MAC context for the target slot
 * @param req  TX request descriptor
 * @return DMR_OK or DMR_ERR_QUEUE_FULL
 */
dmr_err_t mac_tx_request(mac_ctx_t *ctx, const dmr_mac_tx_req_t *req);

/**
 * @brief Submit a TX request directly via mqueue fd (CCL convenience wrapper).
 *
 * Identical to mac_tx_request() but takes a pre-opened mqueue descriptor
 * directly — used by CCL which holds its own mq handle.
 */
dmr_err_t mac_tx_enqueue(mqd_t mq_tx, const dmr_mac_tx_req_t *req);

/**
 * @brief Wait for TX confirmation on the given mqueue (blocking).
 *
 * @param mq_conf    Pre-opened TX confirmation queue
 * @param conf       Output: filled with confirmation details
 * @param timeout_ms Milliseconds to wait; 0=poll, <0=infinite
 * @return DMR_OK, DMR_ERR_TIMEOUT, or DMR_ERR_QUEUE_EMPTY
 */
dmr_err_t mac_tx_wait_conf(mqd_t mq_conf, dmr_mac_tx_conf_t *conf,
                            int timeout_ms);

/**
 * @brief Cancel a pending TX request by req_id.
 *
 * If the request is still in the MAC TX queue (not yet in LBT), it is
 * removed and a DMR_MAC_TX_CANCELLED confirmation is posted.
 *
 * @return DMR_OK if found and cancelled, DMR_ERR_INVALID_PARAM if not found
 */
dmr_err_t mac_tx_cancel(mac_ctx_t *ctx, uint32_t req_id);

/**
 * @brief Receive the next decoded burst from MAC (blocking).
 *
 * @param mq_rx      Pre-opened RX burst queue
 * @param burst      Output: next burst from MAC
 * @param timeout_ms Milliseconds to wait; <0=infinite
 * @return DMR_OK, DMR_ERR_TIMEOUT
 */
dmr_err_t mac_rx_burst_get(mqd_t mq_rx, dmr_burst_t *burst,
                            int timeout_ms);

/**
 * @brief Build a CACH PDU with AT/TC/LCSS fields and Hamming(7,4) FEC.
 *
 * @param cach        Output CACH PDU (3 bytes, pre-interleave logical form)
 * @param at          Access Type bit (0=idle, 1=busy)
 * @param tc          TDMA Channel bit (0=slot1, 1=slot2)
 * @param lcss        LC Start/Stop (DMR_LCSS_*)
 * @param short_data  17-bit CACH payload (Short LC fragment or 0)
 */
void mac_cach_build(dmr_cach_pdu_t *cach,
                    uint8_t at, uint8_t tc, uint8_t lcss,
                    uint32_t short_data);

/**
 * @brief Decode a received CACH PDU and extract AT/TC/LCSS fields.
 *
 * Applies Hamming(7,4) FEC correction on the TACT bits.
 *
 * @param cach     Input CACH PDU (3 bytes, post-deinterleave)
 * @param at       Output: Access Type bit
 * @param tc       Output: TDMA Channel bit
 * @param lcss     Output: LCSS value
 * @param short_data Output: 17-bit Short Data payload
 * @return DMR_OK, DMR_ERR_FEC if uncorrectable
 */
dmr_err_t mac_cach_parse(const dmr_cach_pdu_t *cach,
                          uint8_t *at, uint8_t *tc, uint8_t *lcss,
                          uint32_t *short_data);

/**
 * @brief Update internal slot activity map from a decoded CACH.
 */
void mac_update_slot_activity(mac_ctx_t *ctx,
                               uint8_t at, uint8_t tc);

/**
 * @brief React to a CACH AT Busy→Idle transition for our slot.
 *        Called internally by mac_update_slot_activity(); also declared
 *        here so mac_cach.c can call it without a circular include.
 *        No-op on Tier I DMO (no CACH) or when no voice call is active.
 */
void mac_handle_cach_at_idle(mac_ctx_t *ctx, uint8_t tc);

/**
 * @brief Inject a verified over-the-air raw frame into the MAC receiver stream pipeline.
 * Use this to route base traffic arrays directly into active receiver loops.
 *
 * @param slot Intended target slot identification.
 * @param burst Reference addressing the payload array information matrix.
 * @return DMR_OK or DMR_ERR_QUEUE_FULL.
 */
dmr_err_t dmr_mac_inject_rx_burst(mac_ctx_t *ctx, const dmr_burst_t *burst);

/**
 * @brief Query whether the given slot is currently marked busy by CACH.
 */
bool mac_slot_is_busy(const mac_ctx_t *ctx, dmr_slot_t slot);

/**
 * @brief Transmit a Reverse Channel (RC) burst.
 *
 * RC bursts are 96 bits (12 bytes). The burst is transmitted impolitely
 * in the alternate slot to the active voice call.
 *
 * @param ctx        MAC context
 * @param rc_payload 4-bit RC command payload (DMR_RC_CMD_*)
 * @return DMR_OK or DMR_ERR_BUSY
 */
dmr_err_t mac_rc_burst_tx(mac_ctx_t *ctx, uint8_t rc_payload);

/**
 * @brief Query the current channel access state (thread-safe).
 */
mac_ch_access_state_t mac_get_ch_state(mac_ctx_t *ctx);

/**
 * @brief Get a statistics snapshot (thread-safe copy).
 */
void mac_get_stats(mac_ctx_t *ctx,
                   uint64_t *tx_bursts, uint64_t *rx_bursts,
                   uint64_t *holdoffs,  uint64_t *aborts);

/* =========================================================================
 * Internal helpers — implemented in mac_channel_access.c / mac_cach.c
 * ========================================================================= */

/** Start T_IdleSrch and transition to QUALIFY_IDLE */
void mac_lbt_start(mac_ctx_t *ctx);

/** Called when RSSI / sync detect indicates channel busy */
void mac_lbt_channel_busy(mac_ctx_t *ctx);

/** Called when T_Holdoff expires — retry LBT */
void mac_holdoff_expired(mac_ctx_t *ctx);

/** Called when T_IdleSrch expires cleanly — channel qualified idle */
void mac_idle_srch_expired(mac_ctx_t *ctx);

/** Called when T_DataTxLmt expires — abort the pending TX request */
void mac_tx_lmt_expired(mac_ctx_t *ctx);

/** Schedule a burst into the next TDMA slot window */
void mac_tx_slot_schedule(mac_ctx_t *ctx, const dmr_mac_tx_req_t *req);

/** Internal: post TX confirmation to the CCL mqueue */
void mac_post_tx_conf(mac_ctx_t *ctx, uint32_t req_id,
                      dmr_mac_tx_result_t result, uint64_t tx_time,uint8_t type);

/** The MAC worker thread entry point */
void *mac_thread(void *arg);
/* =========================================================================
 * Reverse Channel command values — TS 102 361-2, Table 7.27
 * ========================================================================= */
#define DMR_RC_CMD_CEASE_TX_CMD  0x04u   /* Cease Transmission Command (emergency)  */
#define DMR_RC_CMD_CEASE_TX_REQ  0x05u   /* Cease Transmission Request (voice int.) */

#ifdef __cplusplus
}
#endif

#endif /* DMR_MAC_H */