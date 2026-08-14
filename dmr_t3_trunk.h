/**
 * @file dmr_t3_trunk.h
 * @brief MOD-07 — Tier III Trunking: Random Access & Channel Grant Flow
 *
 * Implements the MS-side procedure for requesting and receiving a traffic
 * channel grant from a Trunking System Control Channel (TSCC), per
 * ETSI TS 102 361-4, Clause 6.2 (Random Access Procedures) and Clause 6.3
 * (Channel Grants).
 *
 * SCOPE OF THIS MODULE (by design, narrowed for this implementation pass):
 *   - Random Access Request (C_RAND, CSBKO=0x02) — MS→TSCC
 *   - TV_GRANT / TD_GRANT reception (CSBKO=0x01/0x03) — TSCC→MS
 *   - Random access backoff/retry (T_Holdoff-style random backoff) and
 *     T_GrantWait timeout handling
 *   - Channel switch notification via an abstract callback — the actual
 *     RF re-tune is delegated to a HAL/Physical-Layer stub
 *     (on_channel_switch), consistent with MOD-01 (Physical Layer) not yet
 *     being implemented
 *
 * EXPLICITLY OUT OF SCOPE for this pass (future work):
 *   - TSCC monitoring / site search / Network Status Broadcast parsing
 *   - MS Registration / De-registration procedures (CSBKO=0x24/0x25/0x27 —
 *     LLC builders already exist in dmr_llc.h from a prior session, but
 *     the trunking state machine here assumes the MS is ALREADY
 *     registered with the current TSCC)
 *   - Adjacent site migration
 *   - TSCC-side (BS) logic — this module is MS-only
 *
 * Architecture (mirrors MOD-05/MOD-06):
 *   - One instance per MS per TSCC slot (typically the random access /
 *     control channel slot; channel grants are then handed off to CCL
 *     Voice/CCL Data for the actual call, which run on whichever
 *     slot/channel the grant specifies)
 *   - pthread worker + POSIX mqueues + timerfd timers + epoll
 *   - Attaches to the REAL, shared MAC queues for its tscc_slot
 *     (DMR_MQ_MAC_TX_REQ/CONF/RX_BURST_S1 or _S2, from dmr_mac.h) — the
 *     same queues CCL Voice and CCL Data use on that slot. MAC is the
 *     sole creator of those queues (see ownership contract in
 *     dmr_mac.h); this module only ever opens them, with a bounded
 *     retry if MAC hasn't created them yet. mq_evt is the one queue
 *     this module creates/owns itself, scoped per slot.
 *   - Public API: request a channel (voice or data, group or individual),
 *     receive a callback with the granted ch_id/slot or a failure reason
 *
 * ETSI References:
 *   TS 102 361-4 Cl. 6.2   — Random Access Procedures, T_Holdoff backoff
 *   TS 102 361-4 Cl. 6.2.1.1.6 — Random Backoff algorithm
 *   TS 102 361-4 Cl. 6.3   — TV_GRANT / TD_GRANT CSBK PDUs
 */

#ifndef DMR_T3_TRUNK_H
#define DMR_T3_TRUNK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>
#include <mqueue.h>

#include "dmr_pdu.h"
#include "dmr_types.h"
#include "dmr_mac.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Random Access — sizing & timing constants
 * ETSI TS 102 361-4 Cl. 6.2.1.1.6 (Random Backoff), Cl. 6.2.1.1.5 (response delay)
 * ========================================================================= */
#define T3_RAND_MAX_ATTEMPTS      4u      /**< Max Random Access attempts before giving up */
#define T3_T_GRANT_WAIT_MS        1500u   /**< Wait for TV/TD_GRANT after C_RAND (ms)       */
#define T3_T_HOLDOFF_MIN_MS       60u     /**< Random backoff lower bound (ms)              */
#define T3_T_HOLDOFF_MAX_MS       360u    /**< Random backoff upper bound (ms)              */

/* =========================================================================
 * Trunking request state machine
 * ========================================================================= */
typedef enum {
    T3_TRUNK_STATE_IDLE          = 0,  /**< No outstanding channel request          */
    T3_TRUNK_STATE_HOLDOFF       = 1,  /**< Random backoff before (re)transmitting  */
    T3_TRUNK_STATE_RAND_PENDING  = 2,  /**< C_RAND submitted, MAC TX not yet confirmed */
    T3_TRUNK_STATE_GRANT_WAIT    = 3,  /**< C_RAND transmitted, awaiting TV/TD_GRANT */
} t3_trunk_state_t;

static const char * const T3_TRUNK_STATE_NAMES[] = {
    "IDLE", "HOLDOFF", "RAND_PENDING", "GRANT_WAIT"
};

/* =========================================================================
 * Outcome reasons reported to the application via on_grant_result
 * ========================================================================= */
typedef enum {
    T3_GRANT_OK             = 0,  /**< Channel granted — ch_id/slot valid       */
    T3_GRANT_TIMEOUT        = 1,  /**< T_GrantWait expired, retries exhausted   */
    T3_GRANT_TX_FAILED      = 2,  /**< MAC could not transmit C_RAND            */
    T3_GRANT_ABORTED        = 3,  /**< Cancelled via t3_trunk_cancel()          */
} t3_grant_outcome_t;

/* =========================================================================
 * Events — posted to / consumed by the worker thread
 * ========================================================================= */
typedef enum {
    T3_TRUNK_EVT_TX_CONF       = 0,  /**< MAC confirms the C_RAND burst          */
    T3_TRUNK_EVT_BURST_RECEIVED = 1, /**< MAC delivered a decoded RX burst       */
    T3_TRUNK_EVT_TIMER_HOLDOFF = 2,  /**< Random backoff expired — retransmit    */
    T3_TRUNK_EVT_TIMER_GRANTWAIT = 3,/**< T_GrantWait expired — retry or fail    */
    T3_TRUNK_EVT_SHUTDOWN      = 4,  /**< Graceful shutdown requested            */
} t3_trunk_event_type_t;

typedef struct {
    t3_trunk_event_type_t type;
    uint64_t              timestamp_us;
    union {
        dmr_burst_t       burst;     /**< For T3_TRUNK_EVT_BURST_RECEIVED       */
        dmr_mac_tx_conf_t tx_conf;   /**< For T3_TRUNK_EVT_TX_CONF               */
    } u;
} t3_trunk_event_t;

/* POSIX mqueue names — one Trunking instance per MS, per TSCC slot.
 *
 * mq_evt is private to this module — Trunking creates/owns it (mirrors
 * CCL Voice's mq_evt / CCL Data's mq_evt). The MAC-facing queues
 * (mq_mac_tx/conf/rx) are NOT defined here — this module uses the real,
 * shared MAC queue names from dmr_mac.h (DMR_MQ_MAC_TX_REQ_S1/S2 etc.),
 * the same ones CCL Voice and CCL Data attach to on the same slot. See
 * the ownership contract in dmr_mac.h: MAC is the sole creator of those
 * queues; this module must never pass O_CREAT for them. */
#define DMR_MQ_T3_TRUNK_EVT_S1       "/dmr_t3_trunk_evt_s1"
#define DMR_MQ_T3_TRUNK_EVT_S2       "/dmr_t3_trunk_evt_s2"

/* =========================================================================
 * Outstanding channel request context
 * ========================================================================= */
typedef struct {
    bool      active;
    uint8_t   service_kind;   /**< DMR_T3_SVC_VOICE or DMR_T3_SVC_DATA       */
    bool      is_group;
    uint32_t  dst_id;
    uint8_t   attempt;        /**< 0-based attempt counter                    */
    uint32_t  rand_req_id;    /**< req_id of the submitted C_RAND burst       */
} t3_trunk_req_ctx_t;

/* =========================================================================
 * Statistics
 * ========================================================================= */
typedef struct {
    uint64_t rand_access_sent;
    uint64_t rand_access_retries;
    uint64_t grants_received;
    uint64_t grant_timeouts;
    uint64_t grant_tx_failures;
} t3_trunk_stats_t;

/* =========================================================================
 * Trunking Instance — one per MS
 * ========================================================================= */
typedef struct t3_trunk_ctx {
    /* Identity */
    dmr_slot_t          tscc_slot;     /**< Slot the TSCC is monitored on      */
    uint32_t            my_radio_id;
    uint8_t             colour_code;

    /* State machine */
    t3_trunk_state_t    state;
    pthread_mutex_t     state_mutex;

    /* Outstanding request */
    t3_trunk_req_ctx_t  req;

    uint32_t            tx_req_id_next;

    /* Configurable timings (ms) — overridable for fast tests */
    uint32_t            t_grant_wait_ms;
    uint32_t            t_holdoff_min_ms;
    uint32_t            t_holdoff_max_ms;

    /* PRNG state for random backoff (xorshift32 — deterministic, seedable) */
    uint32_t            rand_state;

    /* POSIX message queues */
    mqd_t               mq_evt;
    mqd_t               mq_mac_tx;
    mqd_t               mq_mac_conf;
    mqd_t               mq_mac_rx;

    /* Timers */
    dmr_phy_timer_oneshot_t                 tmr_holdoff;
    dmr_phy_timer_oneshot_t                 tmr_grantwait;


    /* Worker thread */
    pthread_t           thread;
    volatile bool       running;

    /* Callbacks (optional) */
    void (*on_grant_result)(struct t3_trunk_ctx *ctx,
                             t3_grant_outcome_t outcome,
                             uint16_t ch_id, uint8_t slot,
                             bool emergency, uint8_t attempts);
    /**
     * @brief Channel-switch hook — invoked once a grant is accepted, before
     *        on_grant_result fires, so the caller can re-tune a (future)
     *        Physical Layer / HAL before handing off to CCL Voice/Data.
     *        May be NULL; this module performs no RF action itself.
     */
    void (*on_channel_switch)(struct t3_trunk_ctx *ctx,
                               uint16_t ch_id, uint8_t slot);

    /* Statistics */
    t3_trunk_stats_t    stats;
} t3_trunk_ctx_t;

/* =========================================================================
 * Trunking API — public functions
 * ========================================================================= */

/**
 * @brief Initialise a Tier III Trunking instance (opens mqueues/timerfds,
 *        zeroes state, seeds the backoff PRNG from the MS radio ID).
 */
dmr_err_t t3_trunk_init(t3_trunk_ctx_t *ctx,
                          dmr_slot_t      tscc_slot,
                          uint32_t        my_radio_id,
                          uint8_t         colour_code);

/**
 * @brief Start the worker thread.
 */
dmr_err_t t3_trunk_start(t3_trunk_ctx_t *ctx);

/**
 * @brief Request graceful shutdown of the worker thread and join it.
 */
dmr_err_t t3_trunk_stop(t3_trunk_ctx_t *ctx);

/**
 * @brief Close mqueues/timerfds and zero the context. Call after t3_trunk_stop().
 */
void t3_trunk_destroy(t3_trunk_ctx_t *ctx);

/**
 * @brief Request a traffic channel grant for a voice or data call.
 *
 * Submits a Random Access Request (C_RAND) CSBK immediately (no initial
 * backoff), then waits T_GrantWait for a TV_GRANT/TD_GRANT addressed to
 * dst_id. On timeout, retries up to T3_RAND_MAX_ATTEMPTS times with a
 * random backoff between each attempt (ETSI Cl. 6.2.1.1.6). The outcome
 * (success with ch_id/slot, or a failure reason) is reported via
 * on_grant_result(); this function itself returns once the request has
 * been accepted into the state machine (does not block for the grant).
 *
 * @param service_kind  DMR_T3_SVC_VOICE or DMR_T3_SVC_DATA
 * @param is_group       true for group call/data, false for individual
 * @param dst_id          Target group or individual ID for the call
 * @return DMR_OK, DMR_ERR_BUSY (a request is already in progress)
 */
dmr_err_t t3_trunk_request_channel(t3_trunk_ctx_t *ctx,
                                     uint8_t service_kind,
                                     bool    is_group,
                                     uint32_t dst_id);

/**
 * @brief Cancel an outstanding channel request.
 *
 * If a request is in progress, disarms timers and invokes
 * on_grant_result(ctx, T3_GRANT_ABORTED, 0, 0, false, attempts).
 */
dmr_err_t t3_trunk_cancel(t3_trunk_ctx_t *ctx);

/**
 * @brief Inject a decoded RX burst into the Trunking state machine.
 *
 * May be called directly (e.g. by a test harness) or internally by the
 * worker thread when reading mq_mac_rx. Looks for TV_GRANT/TD_GRANT CSBKs
 * addressed to our outstanding request's dst_id while in GRANT_WAIT state;
 * all other burst types are ignored by this module (TSCC monitoring /
 * other CSBKs are out of scope — see module header).
 *
 * @return DMR_OK always (errors are reflected in stats/callbacks)
 */
dmr_err_t t3_trunk_rx_burst(t3_trunk_ctx_t *ctx, const dmr_burst_t *burst);

/**
 * @brief Copy out current statistics (thread-safe).
 */
void t3_trunk_get_stats(t3_trunk_ctx_t *ctx, t3_trunk_stats_t *out);

/**
 * @brief Copy out current state (thread-safe).
 */
t3_trunk_state_t t3_trunk_get_state(t3_trunk_ctx_t *ctx);

/**
 * @brief Package-private: submit the C_RAND burst for ctx->req and arm
 *        T_GrantWait. Exposed for test harness use only; not part of the
 *        public API.
 */
dmr_err_t t3_trunk_submit_rand_access(t3_trunk_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DMR_T3_TRUNK_H */
