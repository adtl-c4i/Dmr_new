/**
 * @file dmr_dmo.h
 * @brief MOD-15 — Dual Capacity Direct Mode (DCDM) Channel Timing
 *        Leader Election
 *
 * ETSI TS 102 361-2 V2.5.1, Clause 6.2 (TDMA direct mode wide area
 * timing) and Clause 7.1.2.6 / 7.2.8-7.2.14 (CT_CSBK PDU + info
 * elements). Scoped to DMR_TIER_2_CONVENTIONAL with dcdm_enabled=true
 * — see dmr_tier.h for why this is a Tier II facility, not Tier I.
 *
 * Facility summary (Cl.6.2.1)
 * ============================
 * With independent TDMA transmissions sharing a channel, all MS units
 * must transmit with the same channel slot timing to avoid inter-slot
 * interference. One MS — the Channel Timing Leader — is elected to set
 * that timing; every other MS (follower) propagates timing information
 * so it reaches the whole wide-area system. Four high-level states:
 *
 *   LEADER_AND_TIMING_UNKNOWN  Power-up/channel-change default. Knows
 *                              neither channel slot timing nor a leader.
 *   LEADER_UNKNOWN             Once knew channel timing (SyncAge
 *                              expired); no longer knows the leader.
 *   LEADER_AND_TIMING_KNOWN    Knows both channel timing and the leader.
 *   LEADER                     This MS IS the leader; sets the timing.
 *
 * Timing information travels in a Channel Timing CSBK (CT_CSBK,
 * CSBKO=0x07), one of 5 sub-types distinguished by NL/CTO:
 *   CT_CSBK_Beacon  periodic, from the leader (CTO=11, transmitted by LEADER)
 *   CT_CSBK_Prop    non-leader propagation/diffusion (CTO=11)
 *   CT_CSBK_Term    transmitted after every voice/data/CSBK TX (any CTO)
 *   CT_CSBK_Req     request for current leader/timing info (CTO=00)
 *   CT_CSBK_Resp    response to a request, or an unsolicited correction
 *                   (CTO=10)
 *
 * Sub-procedures (each mirrors one SDL clause 1:1 in dmr_dmo.c):
 *   CCE  CT_CSBK Evaluation      Cl.6.2.3.7  — every RX_CT_CSBK funnels here
 *   IC   Identifier Conflict     Cl.6.2.3.6  — leader sees its own WATID reused
 *   SC   Send Correction         Cl.6.2.3.8
 *   AL   Accept Leader           Cl.6.2.3.9
 *   ANL  Appoint New Leader      Cl.6.2.3.10
 *   TP   Timing Push             Cl.6.2.3.11
 *
 * Architecture (mirrors MOD-07 / t3_trunk_ctx_t)
 * ================================================
 *   - One instance per MS, on the tier2.fixed_slot the app configured
 *     DCDM for. pthread worker + POSIX mqueues + timerfd timers + epoll.
 *   - Attaches to the REAL, shared MAC queues for that slot
 *     (DMR_MQ_MAC_TX_REQ_S{1,2}, DMR_MQ_MAC_TX_CONF_DCDM_S{1,2},
 *     DMR_MQ_MAC_RX_DCDM_S{1,2}) — MAC is sole creator (see ownership
 *     contract in dmr_mac.h); this module only ever opens them, with a
 *     bounded retry. mq_evt is the one queue this module creates/owns
 *     itself.
 *   - CT_CSBK build/parse lives in llc_ct_csbk_build()/llc_ct_csbk_parse()
 *     (dmr_llc.h / llc_csbk.c) — this module only handles the state
 *     machine and timer logic, not bit-level PDU packing.
 */

#ifndef DMR_DMO_H
#define DMR_DMO_H

#define _POSIX_C_SOURCE 200809L

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>
#include <mqueue.h>

#include "dmr_types.h"
#include "dmr_tier.h"
#include "dmr_pdu.h"
#include "dmr_mac.h"
#include "dmr_llc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Timers and constants — ETSI TS 102 361-2 Annex A.1/A.2
 * ========================================================================= */
#define DMO_NO_LEADER_TIMER_MS        270000u  /* 4.5 min                    */
#define DMO_SYNC_AGE_MS               600000u  /* 10 min                    */
#define DMO_SYNC_AGE_WARNING_MS       540000u  /* 9 min = 2*BeaconInterval  */
#define DMO_BEACON_INTERVAL_MS        270000u  /* 4.5 min                   */
#define DMO_SA_INCR_MS                   500u  /* SyncAge increment          */
#define DMO_BEACON_DURATION_MIN_MS       600u  /* CT_CSBK_Beacon/Prop min    */
#define DMO_CT_DURATION_MIN_MS           180u  /* CT_CSBK_Req/Resp min       */

/* CT_RHOT — random holdoff before a CT_CSBK TX, Annex A.1 */
#define DMO_CT_RHOT_UNKNOWN_MIN_MS         0u
#define DMO_CT_RHOT_UNKNOWN_MAX_MS      10 //3240u  /* uniform 0-3.24s, 60ms step */
#define DMO_CT_RHOT_KNOWN_MIN_MS         10 //2160u  /* uniform 2.16-3.24s after   */
#define DMO_CT_RHOT_KNOWN_MAX_MS        15 //3240u  /*   a CT_CSBK TX, 60ms step  */
#define DMO_CT_RHOT_STEP_MS              1// 60u
#define DMO_CT_RHOT_DECREMENT_MS         2 //120u  /* per cancelled scheduled TX */

/* MS may retry sending a CT_CSBK for up to this long before giving up
 * (Cl.6.2.3.2 etc.: "may attempt to send for up to 2 minutes") */
#define DMO_TX_RETRY_LIMIT_MS         120000u

/* =========================================================================
 * Wide Area Timing IDentifier (WATID) — Cl.7.2.10
 *
 * WATID = {DI (2 bits, leader preference/eligibility), ID (20 bits)}.
 * Packed as a single comparable uint32_t with DI in the high bits so
 * ">"/"<"/"=="  comparisons in the SDL ("LWATID > MS_WATID" etc.) work
 * as plain integer comparisons — DI dominates, ID breaks ties within
 * the same DI, matching the spec's intent that a higher-preference DI
 * always outranks a lower one regardless of ID.
 * ========================================================================= */
#define DMO_WATID(di, id)   ((((uint32_t)(di) & 0x3u) << 20) | ((uint32_t)(id) & 0xFFFFFu))
#define DMO_WATID_DI(watid) (((uint32_t)(watid) >> 20) & 0x3u)
#define DMO_WATID_ID(watid) ((uint32_t)(watid) & 0xFFFFFu)

/* Dynamic Identifier (DI) values — Cl.7.2.9, Table 7.18 */
#define DMO_DI_UNKNOWN_OR_INELIGIBLE  0x0u
#define DMO_DI_LOW                    0x1u
#define DMO_DI_MEDIUM                 0x2u
#define DMO_DI_HIGH                   0x3u

/* Response-wait / self-promotion bootstrap — NOT ETSI-specified.
 * Addresses a genuine gap: without this, an MS whose NoLeaderTimer-
 * triggered CT_CSBK_Req goes unanswered would send exactly one Req
 * and then sit silent forever, never self-promoting to LEADER — DCDM
 * could never bootstrap on a channel with no existing leader. Values
 * are our own reasonable defaults, not spec-mandated; tune as needed.
 * Retries reuse the existing CT_RHOT ("unknown" range) backoff for
 * collision avoidance rather than a separate holdoff mechanism. */
#define DMO_RESPONSE_WAIT_MS        1500u  /* wait for a reply after the Req actually transmits */
#define DMO_NOLEADER_MAX_RETRIES       3u  /* attempts before self-promoting to LEADER           */

/* =========================================================================
 * Pending CT_CSBK kind — Cl.6.2.3.2/3/4/8/10/11's "while waiting to
 * transmit X, a received CT_CSBK either keeps X alive or cancels it"
 * rules each have a different condition, so the kind selects which one
 * applies in dmo_check_pending_tx_interception(). DMO_PENDING_PROP
 * additionally gates the Cl.6.2.2.3.2 CT_RHOT sliding-window decrement
 * (Prop only). DMO_PENDING_OTHER covers CT_CSBK_Beacon and the ad-hoc
 * Req that CCE bullet 3 schedules in response to an incoming CT_CSBK —
 * neither has a stated interception rule, so a received CT_CSBK during
 * either is evaluated normally (CCE/LDR) without touching the pending
 * TX at all.
 * ========================================================================= */
typedef enum {
    DMO_PENDING_NONE = 0,             /* nothing scheduled                  */
    DMO_PENDING_NOLEADER_REQ,         /* Cl.6.2.3.2/6.2.3.3                 */
    DMO_PENDING_SYNCAGEWARNING_REQ,   /* Cl.6.2.3.4                        */
    DMO_PENDING_SC,                   /* Cl.6.2.3.8                        */
    DMO_PENDING_ANL,                  /* Cl.6.2.3.10                       */
    DMO_PENDING_PROP,                 /* Cl.6.2.3.11 (TP's Prop)           */
    DMO_PENDING_OTHER,                /* Beacon / CCE-triggered ad-hoc Req */
} dmo_pending_kind_t;

/* =========================================================================
 * State machine — Cl.6.2.1
 *
 * DCDM operates on a single simplex frequency sliced into two TDMA
 * slots; conventional repeater operation uses a different (duplex)
 * frequency arrangement entirely. The two can't coexist on one
 * channel, so which mode an MS runs is a static, config-time choice
 * (dmr_tier2_config_t.dcdm_enabled) — not something to probe for or
 * switch between at runtime. This module is only ever composed at all
 * when dcdm_enabled is true; there is no "wait and see" state.
 * ========================================================================= */
typedef enum {
    DMO_STATE_LEADER_AND_TIMING_UNKNOWN = 0,
    DMO_STATE_LEADER_UNKNOWN            = 1,
    DMO_STATE_LEADER_AND_TIMING_KNOWN   = 2,
    DMO_STATE_LEADER                    = 3,
} dmo_state_t;

static const char * const DMO_STATE_NAMES[] = {
    "LEADER_AND_TIMING_UNKNOWN", "LEADER_UNKNOWN",
    "LEADER_AND_TIMING_KNOWN", "LEADER"
};

/* =========================================================================
 * Events — posted to / consumed by the worker thread
 * ========================================================================= */
typedef enum {
    DMO_EVT_SHUTDOWN            = 0,
    /* Application-facing: "I am about to transmit voice/data/a CSBK
     * other than CT_CSBK — give me the CTO to close it with and send
     * my own CT_CSBK_Term" (Cl.6.2.3.12 Transmit procedure). */
    DMO_EVT_TX_REQUEST          = 1,
} dmo_event_type_t;

typedef struct {
    dmo_event_type_t type;
    uint64_t         timestamp_us;
} dmo_event_t;

/* mq_evt is private to this module — created/owned here, one per slot
 * (mirrors CCL Voice/Data/Trunking's own mq_evt pattern). */
#define DMR_MQ_DMO_EVT_S1   "/dmr_dmo_evt_s1"
#define DMR_MQ_DMO_EVT_S2   "/dmr_dmo_evt_s2"

/* =========================================================================
 * Statistics
 * ========================================================================= */
typedef struct {
    uint64_t ct_csbk_tx;
    uint64_t ct_csbk_rx;
    uint64_t beacons_sent;
    uint64_t leader_elections;   /* times this MS became LEADER (any path) */
    uint64_t self_promotions;    /* of which: via unanswered-retry bootstrap*/
    uint64_t leaders_accepted;   /* times this MS accepted another leader  */
    uint64_t id_conflicts;
    uint64_t no_leader_timeouts;
    uint64_t sync_age_timeouts;
} dmo_stats_t;

/* =========================================================================
 * DCDM Channel Timing instance — one per MS
 * ========================================================================= */
typedef struct dmr_dmo_ctx {
    /* Configuration */
    dmr_slot_t          slot;          /* tier2.fixed_slot                  */
    uint32_t            my_radio_id;
    uint8_t             colour_code;   /* this MS's own CC (non-CT_CSBK use)*/
    bool                dcdm_enabled;

    /* State machine */
    dmo_state_t         state;
    pthread_mutex_t     state_mutex;

    /* Wide Area Timing identity — Cl.6.2.3.1 (power up/channel change) */
    uint8_t             ms_di;         /* MS_DI — provisionable, Cl.6.2.3.1 */
    uint32_t            ms_id;         /* MS_ID — MS Identifier             */
    uint32_t            ms_watid;      /* DMO_WATID(ms_di, ms_id)           */
    uint8_t             ms_gen;        /* MS_Gen — hops from leader (5 bit) */
    uint16_t            ms_sa;         /* MS_SA  — Sync Age (11 bit)        */
    uint32_t            ms_lwatid;     /* MS_LWATID — leader's WATID, 0=none*/

    /* PRNG state for CT_RHOT / MS_ID regeneration (xorshift32) */
    uint32_t            rand_state;

    /* CT_RHOT sliding-window decrement tracking (Cl.6.2.2.3.2) */
    /* CT_RHOT sliding-window decrement tracking (Cl.6.2.2.3.2) — this
     * specific mechanism is scoped to CT_CSBK_Prop only (not Req/Resp/
     * Beacon), so it gets its own persistent range separate from
     * ct_rhot_min_ms/max_ms below (which every OTHER CT_CSBK type
     * resets explicitly to a full range each time it schedules a TX). */
    uint32_t            prop_ct_rhot_min_ms;
    uint32_t            prop_ct_rhot_max_ms;

    uint32_t            ct_rhot_min_ms;
    uint32_t            ct_rhot_max_ms;

    /* Configurable timer durations (ms) — default to the DMO_*_MS spec
     * values in dmr_dmo_init(), overridable afterward for fast tests
     * (mirrors t3_trunk_ctx_t's t_grant_wait_ms/t_holdoff_min_ms/max_ms
     * pattern). Most consequential: no_leader_timer_ms defaults to 4.5
     * minutes real-world, which is impractical for test loops. */
    uint32_t            no_leader_timer_ms;
    uint32_t            response_wait_ms;
    uint32_t            sync_age_ms;
    uint32_t            sync_age_warning_ms;
    uint32_t            beacon_interval_ms;

    /* Retry-for-up-to-2-minutes bookkeeping for the CT_CSBK currently
     * being (re)transmitted after CT_RHOT/channel-busy */
    uint64_t            tx_retry_start_us;
    bool                tx_retry_active;
    /* Which pending-transmit procedure to resume when the channel
     * frees up / CT_RHOT re-expires (mirrors the SDL's per-procedure
     * "TX_CT_CSBK" / "Channel_Busy" branches, Cl.6.2.3.2-6.2.3.11) */
    dmr_ct_csbk_t       pending_ct_csbk;
    bool                pending_tx_active;
    /* req_id actually submitted to MAC for pending_ct_csbk — set only
     * when dmo_submit_pending_ct_csbk() hands the burst off, NOT when
     * the TX is merely scheduled (dmo_schedule_ct_csbk_tx() arms
     * CT_RHOT but hasn't submitted anything yet). Matches the
     * pending_req_id/tx->pending_req_id/rand_req_id convention already
     * used by ccl_voice_ctx_t/ccl_data_ctx_t/t3_trunk_ctx_t: a TX
     * confirmation is only ever acted on if conf->req_id matches this —
     * a stale confirmation for an already-cancelled/superseded request
     * (submitted before being locally cancelled by an incoming RX
     * event, whose MAC-side confirmation is still in flight when a new
     * request gets scheduled) must never be misattributed to whatever
     * happens to be pending now. */
    uint32_t            pending_req_id;
    /* Which CT_CSBK type is pending — selects the interception rule in
     * dmo_check_pending_tx_interception() and (for DMO_PENDING_PROP)
     * gates the Cl.6.2.2.3.2 CT_RHOT sliding-window decrement. */
    dmo_pending_kind_t  pending_kind;

    /* Retry count for the NoLeaderTimer-Req bootstrap sequence — reset
     * to 0 each time dmo_handle_no_leader_timer() starts a fresh
     * attempt; incremented by dmo_handle_response_wait_timer() on each
     * unanswered retry; see DMO_NOLEADER_MAX_RETRIES. */
    uint8_t             noleader_retry_count;

    /* POSIX message queues */
    mqd_t               mq_evt;
    mqd_t               mq_mac_tx;
    mqd_t               mq_mac_conf;
    mqd_t               mq_mac_rx;

    /* Timers — Annex A.1/A.2 */
    dmr_phy_timer_oneshot_t tmr_ct_rhot;
    /* Response-wait / self-promotion bootstrap (not ETSI-specified —
     * see DMO_RESPONSE_WAIT_MS). Armed once a NoLeaderTimer-triggered
     * CT_CSBK_Req actually transmits; if it fires with nothing heard
     * back, dmo_handle_response_wait_timer() retries or self-promotes. */
    dmr_phy_timer_oneshot_t tmr_response_wait;
    dmr_phy_timer_oneshot_t tmr_no_leader;
    dmr_phy_timer_oneshot_t tmr_sync_age;
    dmr_phy_timer_oneshot_t tmr_sync_age_warning;
    dmr_phy_timer_oneshot_t tmr_beacon_interval;

    /* Worker thread */
    pthread_t           thread;
    volatile bool       running;
    uint32_t            tx_req_id_next;

    /* Optional callback: fired whenever channel slot timing is
     * (re)accepted from a received CT_CSBK, so an application/PHY
     * layer could re-phase its own slot boundary tracking. May be NULL. */
    void (*on_slot_timing_update)(struct dmr_dmo_ctx *ctx, dmr_slot_t rx_slot);

    /* Statistics */
    dmo_stats_t         stats;

    bool                initialised;
} dmr_dmo_ctx_t;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Initialise a DCDM Channel Timing instance (opens mqueues/
 *        timerfds, zeroes state, seeds MS_WATID from radio_id/cfg).
 *        Only meaningful when cfg->dcdm_enabled is true; dmr_ms.c only
 *        calls this for DMR_TIER_2_CONVENTIONAL with dcdm_enabled=true.
 */
dmr_err_t dmr_dmo_init(dmr_dmo_ctx_t *ctx,
                       const dmr_tier2_config_t *cfg,
                       uint32_t my_radio_id,
                       uint8_t colour_code);

/**
 * @brief Start the worker thread. Enters LEADER_AND_TIMING_UNKNOWN and
 *        arms NoLeaderTimer (Cl.6.2.3.1, power up/channel change SDL).
 */
dmr_err_t dmr_dmo_start(dmr_dmo_ctx_t *ctx);

/**
 * @brief Request graceful shutdown of the worker thread and join it.
 */
dmr_err_t dmr_dmo_stop(dmr_dmo_ctx_t *ctx);

/**
 * @brief Close mqueues/timerfds and zero the context. Call after
 *        dmr_dmo_stop(). Safe to call on a zeroed/never-initialised ctx.
 */
void dmr_dmo_destroy(dmr_dmo_ctx_t *ctx);

/**
 * @brief Notify DCDM that the application/CCL is about to transmit
 *        voice, data, or a CSBK other than CT_CSBK, so DCDM can send
 *        the appropriate CT_CSBK_Term immediately afterward with the
 *        correct CTO (Cl.6.2.3.12 Transmit procedure).
 *
 * @param channel_activity  true if there is other TDMA direct mode
 *                          activity on the frequency right now (drives
 *                          CTO=01 aligned-terminator vs CTO=10 aligned
 *                          status per the Transmit procedure SDL)
 */
dmr_err_t dmr_dmo_notify_tx(dmr_dmo_ctx_t *ctx, bool channel_activity);

/**
 * @brief Inject a decoded RX burst into the state machine. May be
 *        called directly (test harness) or internally by the worker
 *        thread reading mq_mac_rx. Non-CT_CSBK bursts are ignored.
 */
dmr_err_t dmr_dmo_rx_burst(dmr_dmo_ctx_t *ctx, const dmr_burst_t *burst);

/**
 * @brief Copy out current state (thread-safe).
 */
dmo_state_t dmr_dmo_get_state(dmr_dmo_ctx_t *ctx);

/**
 * @brief Copy out current statistics (thread-safe).
 */
void dmr_dmo_get_stats(dmr_dmo_ctx_t *ctx, dmo_stats_t *out);

/* Note: the old dmr_dmo_init_stub()/dmr_dmo_destroy_stub() placeholder
 * shims have been removed now that dmr_ms.c calls dmr_dmo_init()/
 * dmr_dmo_start()/dmr_dmo_stop()/dmr_dmo_destroy() directly — the stub
 * signature couldn't carry the mac_ctx_t back-reference this module
 * now needs for repeater detection, so keeping it around would only
 * invite a call site to compile against a broken shim. */

#ifdef __cplusplus
}
#endif

#endif /* DMR_DMO_H */