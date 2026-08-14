/**
 * @file dmr_dmo.c
 * @brief MOD-15 — Dual Capacity Direct Mode (DCDM) Channel Timing
 *        Leader Election
 *
 * See dmr_dmo.h for module overview, scope, and the sub-procedure map.
 * Every static function below implements exactly one ETSI TS 102 361-2
 * clause; the clause number is given in its comment. Implementation
 * follows the NORMATIVE PROSE bullets under each clause (marked "The
 * SDL in figure X defines the following requirements:") rather than
 * hand-parsing the SDL diagrams themselves — the spec explicitly says
 * the diagrams are "informative... serve as a guide" while "the text
 * preceding each diagram includes normative points" (Cl.6.2.3.0).
 *
 * Channel-busy / polite access: MAC (MOD-03) already owns Listen-
 * Before-Transmit and polite channel access (mac_tx_enqueue() with
 * impolite=false). Rather than re-implementing the SDL's inline
 * "Channel_Busy?" checks, this module submits every CT_CSBK politely
 * and reacts to the resulting TX confirmation: DMR_MAC_TX_OK completes
 * the attempt, anything else re-arms CT_RHOT and retries (bounded by
 * DMO_TX_RETRY_LIMIT_MS, "the MS may attempt to send for up to 2
 * minutes before cancelling" — stated identically in every SC/ANL/TP/
 * Req/Resp/Beacon clause).
 */

#define _POSIX_C_SOURCE 200809L

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/epoll.h>

#include "dmr_dmo.h"

/* =========================================================================
 * Local constants
 * ========================================================================= */
#define DMO_MQ_EVT_MAX_MSGS     10
#define DMO_EPOLL_MAX_EVENTS     8

/* =========================================================================
 * Small helpers
 * ========================================================================= */
static void dmo_arm_timer(dmr_phy_timer_oneshot_t *t, uint32_t ms)
{
    dmr_phy_timer_oneshot_arm_ms(t, ms);
}
static void dmo_disarm_timer(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_disarm(t);
}
static void dmo_drain_timer(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_drain(t);
}

static const char * const DMO_PENDING_KIND_NAMES[] = {
    "NONE", "NOLEADER_REQ", "SYNCAGEWARNING_REQ", "SC", "ANL", "PROP", "OTHER"
};

/* One-line, consistently-formatted dump of a CT_CSBK's fields, used for
 * both RX and TX debug logging so the two are easy to compare by eye. */
static void dmo_log_ct_csbk(dmr_dmo_ctx_t *ctx, const char *dir, const dmr_ct_csbk_t *ct)
{
    DMR_LOGI("[DCDM S%d] %s CT_CSBK: CTO=%u NL=%u Gen=%u SA=%u "
             "LID=0x%05X LDI=%u SID=0x%05X SDI=%u",
             ctx->slot, dir, ct->cto, ct->new_leader ? 1u : 0u,
             ct->gen, ct->sync_age, ct->leader_id, ct->leader_di,
             ct->source_id, ct->source_di);
}

/* xorshift32 — small, fast, deterministic PRNG for CT_RHOT backoff and
 * MS_ID regeneration. Not cryptographic; matches MOD-07's rationale. */
static uint32_t dmo_rand_next(dmr_dmo_ctx_t *ctx)
{
    uint32_t x = ctx->rand_state;
    if (x == 0u) x = 0xC0FFEEu;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    ctx->rand_state = x;
    return x;
}

/* Random backoff in [min_ms, max_ms], snapped to DMO_CT_RHOT_STEP_MS
 * increments per Annex A.1 ("with an increment of 60 ms"). */
static uint32_t dmo_rand_ct_rhot_ms(dmr_dmo_ctx_t *ctx)
{
    uint32_t span_steps = (ctx->ct_rhot_max_ms - ctx->ct_rhot_min_ms)
                          / DMO_CT_RHOT_STEP_MS + 1u;
    uint32_t step = dmo_rand_next(ctx) % span_steps;
    return ctx->ct_rhot_min_ms + step * DMO_CT_RHOT_STEP_MS;
}

/* Regenerate MS_ID (20-bit, non-zero) and recompute MS_WATID from it —
 * used by CCE bullet 2 (WATID collision) and IC (leader ID conflict). */
static void dmo_generate_new_ms_id(dmr_dmo_ctx_t *ctx)
{
    uint32_t new_id;
    do {
        new_id = dmo_rand_next(ctx) & 0xFFFFFu;
    } while (new_id == 0u);
    ctx->ms_id    = new_id;
    ctx->ms_watid = DMO_WATID(ctx->ms_di, ctx->ms_id);
}

/* Forward declaration — defined later, called from
 * dmo_handle_response_wait_timer() before its own definition. */
static void dmo_self_promote_to_leader(dmr_dmo_ctx_t *ctx);

/* Cl.6.2.3.x "Slot_Timing:=RX_Slot_Timing" — accept the channel slot
 * timing carried by a received CT_CSBK. Actual RF/PHY re-phasing is a
 * future MOD-01 concern (that module is currently timer-skeleton only
 * — see dmr_phy.h); this hook lets an application/HAL react today. */
static void dmo_accept_slot_timing(dmr_dmo_ctx_t *ctx, dmr_slot_t rx_slot)
{
    if (ctx->on_slot_timing_update != NULL) {
        ctx->on_slot_timing_update(ctx, rx_slot);
    }
}

/* =========================================================================
 * mq_evt (owned) and MAC queue (shared, opened only) helpers — mirrors
 * MOD-07's t3_trunk_mq_create_own()/t3_trunk_open_mac_queues() exactly.
 * ========================================================================= */
static mqd_t dmo_mq_create_own(const char *name, int max_msgs, size_t msg_size)
{
    struct mq_attr attr;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = max_msgs;
    attr.mq_msgsize = (long)msg_size;
    attr.mq_curmsgs = 0;

    mqd_t mq = mq_open(name, O_CREAT | O_NONBLOCK | O_RDWR, 0600, &attr);
    if (mq == (mqd_t)-1) {
        DMR_LOGE("[DCDM] mq_open(%s) failed: %s", name, strerror(errno));
    }
    return mq;
}

static dmr_err_t dmo_open_mac_queues(const char *tx_name,
                                      const char *conf_name,
                                      const char *rx_name,
                                      mqd_t *out_tx, mqd_t *out_conf, mqd_t *out_rx)
{
    int attempts = 0;

    do {
        mqd_t tx   = mq_open(tx_name,   O_NONBLOCK | O_WRONLY);
        mqd_t conf = mq_open(conf_name, O_NONBLOCK | O_RDONLY);
        mqd_t rx   = mq_open(rx_name,   O_NONBLOCK | O_RDONLY);

        if (tx != (mqd_t)-1 && conf != (mqd_t)-1 && rx != (mqd_t)-1) {
            *out_tx = tx; *out_conf = conf; *out_rx = rx;
            return DMR_OK;
        }
        if (tx   != (mqd_t)-1) mq_close(tx);
        if (conf != (mqd_t)-1) mq_close(conf);
        if (rx   != (mqd_t)-1) mq_close(rx);

        attempts++;
        if (attempts < DMR_MQ_OPEN_RETRY_COUNT) {
            struct timespec ts = { 0, (long)DMR_MQ_OPEN_RETRY_DELAY_MS * 1000000L };
            nanosleep(&ts, NULL);
        }
    } while (attempts < DMR_MQ_OPEN_RETRY_COUNT);

    DMR_LOGE("[DCDM] mq_open(%s,%s,%s) failed after %d retries — was "
             "mac_init() called before dmr_dmo_init()?",
             tx_name, conf_name, rx_name, attempts);
    *out_tx = (mqd_t)-1; *out_conf = (mqd_t)-1; *out_rx = (mqd_t)-1;
    return DMR_ERR_QUEUE_FULL;
}

/* =========================================================================
 * TX scheduling — arm CT_RHOT, remember what to send when it fires.
 * Shared by every procedure that ends in "Set(CT_RHOT) / TX_CT_CSBK".
 * ========================================================================= */
static void dmo_schedule_ct_csbk_tx(dmr_dmo_ctx_t *ctx, const dmr_ct_csbk_t *ct,
                                     dmo_pending_kind_t kind)
{
    ctx->pending_ct_csbk   = *ct;
    ctx->pending_tx_active = true;
    ctx->pending_kind      = kind;
    ctx->tx_retry_active   = true;
    ctx->tx_retry_start_us = dmr_time_now_us();

    uint32_t backoff = dmo_rand_ct_rhot_ms(ctx);
    dmo_arm_timer(&ctx->tmr_ct_rhot, backoff);
}

/* Submit the pending CT_CSBK to MAC right now (called on CT_RHOT expiry,
 * or immediately for CT_CSBK_Term which the spec says has no holdoff). */
static void dmo_submit_pending_ct_csbk(dmr_dmo_ctx_t *ctx)
{
    dmr_mac_tx_req_t req;
    memset(&req, 0, sizeof(req));
    req.slot            = ctx->slot;
    req.priority        = DMR_MAC_PRIORITY_NORMAL;
    req.deadline_us     = 0u;
    req.impolite         = false; /* Polite to All, Cl.5.2.2.1 of TS 102 361-1 */
    req.originated_from = DCDM_TX_ORIGIN_DCDM;
    req.req_id          = ctx->tx_req_id_next++;
    ctx->pending_req_id = req.req_id; /* what dmo_handle_tx_conf() must match */

    llc_ct_csbk_build(&req.burst, &ctx->pending_ct_csbk, ctx->slot);

    dmo_log_ct_csbk(ctx, "TX ", &ctx->pending_ct_csbk);
    DMR_LOGI("[DCDM S%d] submitting %s (req_id=%u) to MAC",
             ctx->slot, DMO_PENDING_KIND_NAMES[ctx->pending_kind], req.req_id);

    if (mac_tx_enqueue(ctx->mq_mac_tx, &req) == DMR_OK) {
        ctx->stats.ct_csbk_tx++;
    } else {
        DMR_LOGW("[DCDM S%d] mac_tx_enqueue failed for req_id=%u", ctx->slot, req.req_id);
    }
}

/* CT_RHOT expiry: try to transmit now (MAC applies LBT itself). */
static void dmo_handle_ct_rhot_expiry(dmr_dmo_ctx_t *ctx)
{
    if (!ctx->pending_tx_active) return;
    dmo_submit_pending_ct_csbk(ctx);
}

/* TX confirmation from MAC for a previously-submitted CT_CSBK. */
static void dmo_handle_tx_conf(dmr_dmo_ctx_t *ctx, const dmr_mac_tx_conf_t *conf)
{
    /* Only react to the one outstanding request we're actually waiting
     * on (mirrors ccl_data.c's tx->pending_req_id / t3_trunk.c's
     * rand_req_id checks). Without this, a confirmation for a request
     * that was already submitted to MAC but then superseded locally
     * (cancelled by dmo_dispatch_rx()'s interception logic, which
     * schedules a brand-new pending request in the same evaluation)
     * would be misattributed to the NEW request — silently marking it
     * "confirmed" without it ever actually having been transmitted. */
    if (!ctx->pending_tx_active || conf->req_id != ctx->pending_req_id) {
        DMR_LOGI("[DCDM S%d] stale/mismatched TX conf (req_id=%u, "
                 "pending_req_id=%u, pending_active=%d) — ignoring",
                 ctx->slot, conf->req_id, ctx->pending_req_id, ctx->pending_tx_active);
        return;
    }

    if (conf->result == DMR_MAC_TX_OK) {
        DMR_LOGI("[DCDM S%d] TX confirmed OK for %s",
                 ctx->slot, DMO_PENDING_KIND_NAMES[ctx->pending_kind]);
        if (ctx->pending_kind == DMO_PENDING_PROP) {
            /* "until a scheduled CT_CSBK_Prop is transmitted... the
             * CT_RHOT range is set back to 2.16-3.24s" (Cl.6.2.2.3.2) */
            ctx->prop_ct_rhot_min_ms = DMO_CT_RHOT_KNOWN_MIN_MS;
            ctx->prop_ct_rhot_max_ms = DMO_CT_RHOT_KNOWN_MAX_MS;
        }
        bool was_noleader_req = (ctx->pending_kind == DMO_PENDING_NOLEADER_REQ);
        ctx->pending_tx_active = false;
        ctx->tx_retry_active   = false;
        ctx->pending_kind      = DMO_PENDING_NONE;
        if (was_noleader_req) {
            /* Not ETSI-specified — our own retry/self-promotion
             * bootstrap; see DMO_RESPONSE_WAIT_MS. Wait for a reply;
             * dmo_dispatch_rx() disarms this the moment anything at
             * all is received, successful or not. */
            dmo_arm_timer(&ctx->tmr_response_wait, ctx->response_wait_ms);
        }
        return;
    }

    /* Not OK (aborted/cancelled/deadline — MAC's LBT effectively played
     * the SDL's "Channel_Busy = yes" role here). Retry via CT_RHOT,
     * bounded to ~2 minutes total per every clause's retry note. */
    uint64_t elapsed_ms = (dmr_time_now_us() - ctx->tx_retry_start_us) / 1000u;
    DMR_LOGI("[DCDM S%d] TX NOT OK (result=%d) for %s, elapsed=%lums",
             ctx->slot, (int)conf->result, DMO_PENDING_KIND_NAMES[ctx->pending_kind],
             (unsigned long)elapsed_ms);
    if (elapsed_ms >= DMO_TX_RETRY_LIMIT_MS) {
        ctx->pending_tx_active = false;
        ctx->tx_retry_active   = false;
        ctx->pending_kind      = DMO_PENDING_NONE;
        DMR_LOGW("[DCDM S%d] CT_CSBK retry limit (2 min) reached — giving up",
                 ctx->slot);
        return;
    }
    /* Sliding-window CT_RHOT decrement on a cancelled/failed attempt
     * (Cl.6.2.2.3.2) — scoped specifically to CT_CSBK_Prop, not Req/
     * Resp/Beacon: narrow the persistent Prop range by 120ms each time,
     * down to 0, then reset back to the full 2.16-3.24s window. */
    if (ctx->pending_kind == DMO_PENDING_PROP) {
        if (ctx->prop_ct_rhot_min_ms >= DMO_CT_RHOT_DECREMENT_MS) {
            ctx->prop_ct_rhot_min_ms -= DMO_CT_RHOT_DECREMENT_MS;
            ctx->prop_ct_rhot_max_ms -= DMO_CT_RHOT_DECREMENT_MS;
        } else {
            ctx->prop_ct_rhot_min_ms = DMO_CT_RHOT_KNOWN_MIN_MS;
            ctx->prop_ct_rhot_max_ms = DMO_CT_RHOT_KNOWN_MAX_MS;
        }
        ctx->ct_rhot_min_ms = ctx->prop_ct_rhot_min_ms;
        ctx->ct_rhot_max_ms = ctx->prop_ct_rhot_max_ms;
    }
    uint32_t backoff = dmo_rand_ct_rhot_ms(ctx);
    DMR_LOGI("[DCDM S%d] retrying %s after %ums CT_RHOT backoff",
             ctx->slot, DMO_PENDING_KIND_NAMES[ctx->pending_kind], backoff);
    dmo_arm_timer(&ctx->tmr_ct_rhot, backoff);
}

/* =========================================================================
 * Sub-procedures — Cl.6.2.3.6, 6.2.3.8-6.2.3.11
 * ========================================================================= */

/* Send Correction — Cl.6.2.3.8, Fig 6.9 field box */
static void dmo_sc(dmr_dmo_ctx_t *ctx)
{
    dmr_ct_csbk_t resp;
    memset(&resp, 0, sizeof(resp));
    resp.new_leader = false;
    resp.cto        = DMR_CTO_ALIGNED_STATUS; /* 10 */

    if (ctx->state == DMO_STATE_LEADER) {
        resp.gen       = 0u;
        resp.sync_age  = ctx->ms_sa;
        resp.leader_id = DMO_WATID_ID(ctx->ms_watid);
        resp.leader_di = (uint8_t)DMO_WATID_DI(ctx->ms_watid);
        resp.source_id = ctx->ms_id;
        resp.source_di = ctx->ms_di;
    } else {
        resp.gen       = ctx->ms_gen;
        resp.sync_age  = ctx->ms_sa;
        resp.leader_id = DMO_WATID_ID(ctx->ms_lwatid);
        resp.leader_di = (uint8_t)DMO_WATID_DI(ctx->ms_lwatid);
        resp.source_id = ctx->ms_id;
        resp.source_di = 0u;
    }
    dmo_schedule_ct_csbk_tx(ctx, &resp, DMO_PENDING_SC);
}

/* Appoint New Leader — Cl.6.2.3.10, Fig 6.11 field box */
static void dmo_anl(dmr_dmo_ctx_t *ctx, uint32_t new_leader_watid)
{
    ctx->ms_gen    = 1u;
    ctx->ms_lwatid = new_leader_watid;
    ctx->state     = DMO_STATE_LEADER_AND_TIMING_KNOWN;

    dmr_ct_csbk_t req;
    memset(&req, 0, sizeof(req));
    req.gen        = ctx->ms_gen;
    req.sync_age   = ctx->ms_sa;
    req.leader_id  = DMO_WATID_ID(ctx->ms_lwatid);
    req.leader_di  = (uint8_t)DMO_WATID_DI(ctx->ms_lwatid);
    req.source_id  = ctx->ms_id;
    req.source_di  = 0u;
    req.new_leader = true;               /* NL=1 — appointing */
    req.cto        = DMR_CTO_ALIGNED_STATUS; /* 10 */
    dmo_schedule_ct_csbk_tx(ctx, &req, DMO_PENDING_ANL);
}

/* Accept Leader — Cl.6.2.3.9, Fig 6.10 */
static void dmo_al(dmr_dmo_ctx_t *ctx, const dmr_ct_csbk_t *rx, dmr_slot_t rx_slot)
{
    if (rx->cto == DMR_CTO_UNALIGNED_REQ || rx->cto == DMR_CTO_UNALIGNED_TERM) {
        DMR_LOGI("[DCDM S%d] AL: CTO=%u (unaligned) -> NOT accepting leader",
                 ctx->slot, rx->cto);
        return; /* CTO 00/01 — do not accept the leader */
    }
    /* CTO 10/11 — accept channel slot timing and timing parameters */
    DMR_LOGI("[DCDM S%d] AL: accepting leader LID=0x%05X LDI=%u Gen->%u SA->%u",
             ctx->slot, rx->leader_id, rx->leader_di, rx->gen + 1u, rx->sync_age);
    ctx->ms_gen    = (uint8_t)(rx->gen + 1u);
    ctx->ms_sa     = rx->sync_age;
    ctx->ms_lwatid = DMO_WATID(rx->leader_di, rx->leader_id);

    /* "Set both SyncAgeWarning and SyncAge timers initialized with the
     * received SA value" — SA already represents elapsed time since
     * the leader's last beacon, so the remaining budget on each timer
     * is its full duration minus that elapsed amount. */
    uint32_t elapsed_ms = (uint32_t)rx->sync_age * DMO_SA_INCR_MS;
    uint32_t sa_remain_ms = (elapsed_ms < ctx->sync_age_ms)
                            ? (ctx->sync_age_ms - elapsed_ms) : 1u;
    uint32_t warn_remain_ms = (elapsed_ms < ctx->sync_age_warning_ms)
                              ? (ctx->sync_age_warning_ms - elapsed_ms) : 1u;
    dmo_arm_timer(&ctx->tmr_sync_age, sa_remain_ms);
    dmo_arm_timer(&ctx->tmr_sync_age_warning, warn_remain_ms);

    dmo_accept_slot_timing(ctx, rx_slot);
    ctx->state = DMO_STATE_LEADER_AND_TIMING_KNOWN;
    ctx->stats.leaders_accepted++;
}

/* Timing Push — Cl.6.2.3.11, Fig 6.12 field box */
static void dmo_tp(dmr_dmo_ctx_t *ctx, const dmr_ct_csbk_t *rx, dmr_slot_t rx_slot)
{
    ctx->ms_gen    = (uint8_t)(rx->gen + 1u);
    ctx->ms_sa     = rx->sync_age;
    ctx->ms_lwatid = DMO_WATID(rx->leader_di, rx->leader_id);
    dmo_accept_slot_timing(ctx, rx_slot);

    dmr_ct_csbk_t prop;
    memset(&prop, 0, sizeof(prop));
    prop.gen        = ctx->ms_gen;
    prop.sync_age   = ctx->ms_sa;
    prop.leader_id  = DMO_WATID_ID(ctx->ms_lwatid);
    prop.leader_di  = (uint8_t)DMO_WATID_DI(ctx->ms_lwatid);
    prop.source_id  = ctx->ms_id;
    prop.source_di  = 0u;
    prop.new_leader = false;
    prop.cto        = DMR_CTO_ALIGNED_PUSH; /* 11 */
    ctx->state = DMO_STATE_LEADER_AND_TIMING_KNOWN;

    /* Cl.6.2.2.3.2 sliding-window CT_RHOT is scoped to Prop specifically
     * — load this Prop's persistent range (narrowed over successive
     * cancelled attempts, reset to full on success) rather than the
     * generic known-range used by Req/Resp/Beacon. */
    ctx->ct_rhot_min_ms = ctx->prop_ct_rhot_min_ms;
    ctx->ct_rhot_max_ms = ctx->prop_ct_rhot_max_ms;
    dmo_schedule_ct_csbk_tx(ctx, &prop, DMO_PENDING_PROP);
}

/* Leader Identifier Conflict — Cl.6.2.3.6, Fig 6.7 */
static void dmo_ic(dmr_dmo_ctx_t *ctx, const dmr_ct_csbk_t *rx, dmr_slot_t rx_slot)
{
    int32_t diff = (int32_t)rx->sync_age - (int32_t)ctx->ms_sa;

    if (diff >= -10 && diff <= 10) {
        DMR_LOGI("[DCDM S%d] IC: SA within tolerance (diff=%d) -> still leader, "
                 "no action", ctx->slot, diff);
        return; /* within +-10 SAIncr of MS_SA — still the leader, no action */
    }

    if (ctx->ms_sa != 0u) {
        int32_t rem  = (int32_t)rx->sync_age % (int32_t)ctx->ms_sa;
        int32_t rem2 = rem - (int32_t)ctx->ms_sa;
        if ((rem >= -10 && rem <= 10) || (rem2 >= -10 && rem2 <= 10)) {
            /* Multiple of MS_SA within +-10 SAIncr — still the leader,
             * but send out a correction. */
            DMR_LOGI("[DCDM S%d] IC: SA is a multiple of ours -> still leader, "
                     "sending SC", ctx->slot);
            dmo_sc(ctx);
            return;
        }
    }

    /* Genuine conflict: another MS using the same LWATID. Regenerate
     * MS_ID and resolve by comparing the new ID against the received
     * Leader Identifier. */
    DMR_LOGI("[DCDM S%d] IC: genuine WATID conflict -> regenerating MS_ID "
             "(old ms_id=0x%05X)", ctx->slot, ctx->ms_id);
    dmo_generate_new_ms_id(ctx);
    if (ctx->ms_id > rx->leader_id) {
        DMR_LOGI("[DCDM S%d] IC: new ms_id=0x%05X > rx leader_id=0x%05X "
                 "-> staying leader", ctx->slot, ctx->ms_id, rx->leader_id);
        return; /* stay leader */
    }
    DMR_LOGI("[DCDM S%d] IC: new ms_id=0x%05X <= rx leader_id=0x%05X "
             "-> stepping down via AL", ctx->slot, ctx->ms_id, rx->leader_id);
    dmo_al(ctx, rx, rx_slot); /* accept the other MS as leader */
}

/* =========================================================================
 * CT_CSBK Evaluation (CCE) — Cl.6.2.3.7, Fig 6.8. Non-leader states only.
 * ========================================================================= */
static void dmo_cce(dmr_dmo_ctx_t *ctx, const dmr_ct_csbk_t *rx, dmr_slot_t rx_slot)
{
    uint32_t rx_lwatid = DMO_WATID(rx->leader_di, rx->leader_id);
    uint32_t rx_swatid = DMO_WATID(rx->source_di, rx->source_id);

    /* Bullet 1 — appointed as the new leader */
    if (rx_lwatid == ctx->ms_watid && rx->new_leader) {
        DMR_LOGI("[DCDM S%d] CCE bullet1: appointed as LEADER -> transitioning, "
                 "sending beacon", ctx->slot);
        ctx->ms_gen = 0u;
        ctx->ms_sa  = rx->sync_age;
        dmo_arm_timer(&ctx->tmr_beacon_interval, ctx->beacon_interval_ms);
        ctx->state = DMO_STATE_LEADER;
        ctx->stats.leader_elections++;
        /* "transitions to the Leader state to immediately transmit a
         * beacon" — build and schedule it right away (still politely,
         * via CT_RHOT, matching every other CT_CSBK TX in this module). */
        {
            dmr_ct_csbk_t beacon;
            memset(&beacon, 0, sizeof(beacon));
            beacon.gen        = 0u;
            beacon.sync_age   = ctx->ms_sa;
            beacon.leader_id  = DMO_WATID_ID(ctx->ms_watid);
            beacon.leader_di  = (uint8_t)DMO_WATID_DI(ctx->ms_watid);
            beacon.source_id  = ctx->ms_id;
            beacon.source_di  = ctx->ms_di;
            beacon.new_leader = false;
            beacon.cto        = DMR_CTO_ALIGNED_PUSH;
            ctx->stats.beacons_sent++;
            dmo_schedule_ct_csbk_tx(ctx, &beacon, DMO_PENDING_OTHER);
        }
        return;
    }

    /* Bullet 2 — WATID collision with a different MS: regenerate MS_ID,
     * then continue evaluating with the (possibly now different) state
     * below rather than stopping — the diagram flows onward. */
    if (rx_lwatid == ctx->ms_watid &&
        (!rx->new_leader || rx->source_id == ctx->ms_id)) {
        DMR_LOGI("[DCDM S%d] CCE bullet2: WATID collision -> regenerating MS_ID "
                 "(old ms_id=0x%05X)", ctx->slot, ctx->ms_id);
        dmo_generate_new_ms_id(ctx);
    }

    /* Bullet 3 — this MS has no leader yet. Compares full WATID
     * (DI+ID), not DI alone: a bare DI comparison can never resolve
     * when both radios share the same DI (the common case — most
     * fresh radios default to the same preference level), causing an
     * infinite Req/Req livelock where neither side ever defers. WATID
     * comparison matches every other bullet in this clause (5, 6, 7,
     * ANL's own cancellation check, LDR's SC/ANL/AL) — DI still
     * dominates, ID only breaks a tie within the same DI. */
    if (ctx->ms_lwatid == 0u) {
        dmo_accept_slot_timing(ctx, rx_slot);
        if (rx_lwatid == 0u && rx_swatid > ctx->ms_watid) {
            DMR_LOGI("[DCDM S%d] CCE bullet3: no leader, sender WATID higher "
                     "-> ANL (deferring to sender)", ctx->slot);
            dmo_anl(ctx, rx_swatid);
        } else if (rx_lwatid == 0u && rx_swatid <= ctx->ms_watid) {
            DMR_LOGI("[DCDM S%d] CCE bullet3: no leader, sender WATID not "
                     "higher -> sending own CT_CSBK_Req", ctx->slot);
            /* "attempt to send a CT_CSBK_Req" */
            dmr_ct_csbk_t req;
            memset(&req, 0, sizeof(req));
            req.gen = 0u; req.sync_age = 0u; req.leader_id = 0u;
            req.leader_di = 0u; req.new_leader = false;
            req.source_id = ctx->ms_id; req.source_di = ctx->ms_di;
            req.cto = DMR_CTO_UNALIGNED_REQ;
            dmo_schedule_ct_csbk_tx(ctx, &req, DMO_PENDING_OTHER);
        } else /* rx_lwatid != 0 */ {
            DMR_LOGI("[DCDM S%d] CCE bullet3: no leader, sender named a leader "
                     "-> AL (accept leader)", ctx->slot);
            dmo_al(ctx, rx, rx_slot);
        }
        return;
    }

    /* Bullets 6/7 — received leader is BETTER (higher LWATID) than ours.
     * Checked before bullet 4/5 below: otherwise a blanket "CTO=01 ->
     * handle and stop" check would make bullet 7 (CTO=01 + better
     * leader -> AL) unreachable, since CTO=01 is exactly bullet 7's
     * trigger too. */
    if (rx_lwatid > ctx->ms_lwatid) {
        if (rx->cto == DMR_CTO_ALIGNED_PUSH) {         /* Bullet 6: accept + TP */
            DMR_LOGI("[DCDM S%d] CCE bullet6: better leader, CTO=push -> "
                     "accept timing + TP", ctx->slot);
            dmo_accept_slot_timing(ctx, rx_slot);
            dmo_tp(ctx, rx, rx_slot);
        } else if (rx->cto == DMR_CTO_UNALIGNED_TERM) { /* Bullet 7: accept + AL */
            DMR_LOGI("[DCDM S%d] CCE bullet7: better leader, CTO=unaligned_term "
                     "-> accept timing + AL", ctx->slot);
            dmo_accept_slot_timing(ctx, rx_slot);
            dmo_al(ctx, rx, rx_slot);
        } else {
            DMR_LOGI("[DCDM S%d] CCE: better leader, CTO=%u -> no rule, no action",
                     ctx->slot, rx->cto);
        }
        /* CTO 00/10 with a better leader: no rule stated — no action. */
        return;
    }

    /* Bullet 5 — received leader is stale/lower, or same leader but a
     * more recent SA. This also fully covers what a narrower "CTO=01 ->
     * SC only" rule would give for the lwatid<ms_lwatid case, just with
     * the correct SC-vs-ANL choice via SDI instead of always SC. */
    if (rx_lwatid < ctx->ms_lwatid ||
        (rx_lwatid == ctx->ms_lwatid && rx->sync_age > ctx->ms_sa)) {
        uint32_t ms_ldi = DMO_WATID_DI(ctx->ms_lwatid);
        if (rx->source_di <= ms_ldi) {
            DMR_LOGI("[DCDM S%d] CCE bullet5: stale/lower leader, sender DI<=ours "
                     "-> SC (send correction)", ctx->slot);
            dmo_sc(ctx);
        } else {
            DMR_LOGI("[DCDM S%d] CCE bullet5: stale/lower leader, sender DI>ours "
                     "-> ANL (appoint sender)", ctx->slot);
            dmo_anl(ctx, rx_swatid);
        }
        return;
    }

    /* Bullet 4's residual case — this MS has a leader, an unaligned
     * terminator (CTO=01) was received, but it's neither strictly lower
     * LWATID nor a fresher SA at equal LWATID (both already handled by
     * bullet 5 above). Per spec: "shall not accept the timing or update
     * the leader information" — no action for this remaining slice. */

    /* Bullets 8/9 — same leader+SA, lower Gen: accept, bump our Gen;
     * bullet 9 additionally sends a Timing Push when CTO is aligned push */
    if (rx_lwatid == ctx->ms_lwatid && rx->sync_age == ctx->ms_sa &&
        rx->gen < ctx->ms_gen) {
        dmo_accept_slot_timing(ctx, rx_slot);
        if (rx->cto == DMR_CTO_ALIGNED_PUSH) {
            DMR_LOGI("[DCDM S%d] CCE bullet9: same leader+SA, lower Gen, "
                     "CTO=push -> accept + TP", ctx->slot);
            dmo_tp(ctx, rx, rx_slot); /* also sets MS_Gen = rx.gen+1 */
        } else {
            DMR_LOGI("[DCDM S%d] CCE bullet8: same leader+SA, lower Gen "
                     "-> accept, bump Gen to %u", ctx->slot, rx->gen + 1u);
            ctx->ms_gen = (uint8_t)(rx->gen + 1u);
        }
        return;
    }

    DMR_LOGI("[DCDM S%d] CCE: no rule matched -> no action", ctx->slot);
}

/* Leader's own evaluation — Cl.6.2.3.5 prose bullets, Fig 6.6. Only
 * reached in DMO_STATE_LEADER (referred to as "LDR" elsewhere). */
static void dmo_ldr(dmr_dmo_ctx_t *ctx, const dmr_ct_csbk_t *rx, dmr_slot_t rx_slot)
{
    uint32_t rx_lwatid = DMO_WATID(rx->leader_di, rx->leader_id);
    uint32_t rx_swatid = DMO_WATID(rx->source_di, rx->source_id);

    if ((rx_lwatid != 0u && rx_lwatid < ctx->ms_watid) ||
        (rx_lwatid == 0u && rx->source_di <= ctx->ms_di)) {
        DMR_LOGI("[DCDM S%d] LDR: lower/no leader named -> SC (send correction) %d %d %d",
                 ctx->slot,rx->source_di,rx_lwatid,ctx->ms_watid );
        dmo_sc(ctx);
        return;
    }
    if ((rx_lwatid != 0u && rx_lwatid > ctx->ms_watid && rx->source_di > rx->leader_di) ||
        (rx_lwatid == 0u && rx->source_di > ctx->ms_di)) {
        DMR_LOGI("[DCDM S%d] LDR: higher-preference sender -> ANL (appoint sender)",
                 ctx->slot);
        dmo_anl(ctx, rx_swatid);
        return;
    }
    if (rx_lwatid == ctx->ms_watid && rx->sync_age != ctx->ms_sa) {
        DMR_LOGI("[DCDM S%d] LDR: someone else claims our own WATID with "
                 "mismatched SA -> IC (identifier conflict)", ctx->slot);
        dmo_ic(ctx, rx, rx_slot);
        return;
    }
    if (rx_lwatid > ctx->ms_watid && rx->source_di <= ctx->ms_di) {
        DMR_LOGI("[DCDM S%d] LDR: better leader named, sender DI<=ours "
                 "-> AL (accept leader, stepping down)", ctx->slot);
        dmo_al(ctx, rx, rx_slot);
        return;
    }
    DMR_LOGI("[DCDM S%d] LDR: no rule matched -> no action (still LEADER)", ctx->slot);
}

/* Result of checking a received CT_CSBK against whatever CT_CSBK this
 * MS currently has pending — Cl.6.2.3.2/3/4/8/10/11. */
typedef enum {
    DMO_INTERCEPT_NONE = 0,  /* no rule for this pending kind — evaluate normally, don't touch it */
    DMO_INTERCEPT_CONTINUE,  /* keep the pending TX alive; do not evaluate this CT_CSBK further   */
    DMO_INTERCEPT_CANCEL,    /* cancel the pending TX, then evaluate this CT_CSBK normally        */
} dmo_intercept_result_t;

static dmo_intercept_result_t dmo_check_pending_tx_interception(dmr_dmo_ctx_t *ctx,
                                                                  const dmr_ct_csbk_t *rx,
                                                                  dmr_slot_t rx_slot)
{
    if (!ctx->pending_tx_active) return DMO_INTERCEPT_NONE;

    uint32_t rx_lwatid = DMO_WATID(rx->leader_di, rx->leader_id);
    uint32_t rx_swatid = DMO_WATID(rx->source_di, rx->source_id);

    switch (ctx->pending_kind) {

    case DMO_PENDING_NOLEADER_REQ:
        /* Cl.6.2.3.2/6.2.3.3: a lower-DI leader is accepted for timing
         * purposes only, without cancelling our own pending Req. */
        if (rx_lwatid != 0u && rx->leader_di < ctx->ms_di) {
            dmo_accept_slot_timing(ctx, rx_slot);
            return DMO_INTERCEPT_CONTINUE;
        }
        return DMO_INTERCEPT_CANCEL;

    case DMO_PENDING_SYNCAGEWARNING_REQ:
        /* Cl.6.2.3.4 */
        if (rx->cto == DMR_CTO_UNALIGNED_REQ || rx->cto == DMR_CTO_UNALIGNED_TERM ||
            rx_lwatid < ctx->ms_lwatid ||
            (rx_lwatid == ctx->ms_lwatid && rx->sync_age > ctx->ms_sa)) {
            return DMO_INTERCEPT_CONTINUE;
        }
        return DMO_INTERCEPT_CANCEL;

    case DMO_PENDING_SC: {
        /* Cl.6.2.3.8. "Receiver's MS_LWATID" here means our own WATID
         * when we're LEADER (mirrors dmo_ldr()'s use of ms_watid). */
        uint32_t eff_lwatid = (ctx->state == DMO_STATE_LEADER) ? ctx->ms_watid : ctx->ms_lwatid;
        if (rx->cto == DMR_CTO_UNALIGNED_REQ || rx->cto == DMR_CTO_UNALIGNED_TERM ||
            rx_lwatid < eff_lwatid) {
            return DMO_INTERCEPT_CONTINUE;
        }
        return DMO_INTERCEPT_CANCEL;
    }

    case DMO_PENDING_ANL:
        /* Cl.6.2.3.10 — ms_lwatid already holds the newly-appointed
         * leader's WATID at this point (dmo_anl() sets it up front). */
        if (rx_lwatid > ctx->ms_lwatid || rx_swatid > ctx->ms_lwatid) {
            return DMO_INTERCEPT_CANCEL;
        }
        return DMO_INTERCEPT_CONTINUE;

    case DMO_PENDING_PROP:
        /* Cl.6.2.3.11 */
        if (rx_lwatid > ctx->ms_lwatid || rx_swatid > ctx->ms_lwatid) {
            return DMO_INTERCEPT_CANCEL;
        }
        if (rx_lwatid == ctx->ms_lwatid && rx->cto == DMR_CTO_ALIGNED_PUSH &&
            rx->sync_age == ctx->ms_sa) {
            return DMO_INTERCEPT_CANCEL;
        }
        return DMO_INTERCEPT_CONTINUE;

    case DMO_PENDING_OTHER:
    case DMO_PENDING_NONE:
    default:
        return DMO_INTERCEPT_NONE;
    }
}

/* Top-level RX dispatch — routes to CCE (non-leader) or LDR (leader),
 * per Fig 6.3-6.6's uniform "RX_CT_CSBK -> CCE" (or "-> LDR" for LEADER). */
static void dmo_dispatch_rx(dmr_dmo_ctx_t *ctx, const dmr_ct_csbk_t *rx, dmr_slot_t rx_slot)
{
    ctx->stats.ct_csbk_rx++;
    dmo_log_ct_csbk(ctx, "RX ", rx);

    /* Not ETSI-specified — part of our own retry/self-promotion
     * bootstrap (see DMO_RESPONSE_WAIT_MS). Any received CT_CSBK is
     * evidence the channel isn't silent, so it cancels the wait
     * regardless of what it turns out to be or how it's evaluated
     * below. No-op if the timer isn't currently armed. */
    dmo_disarm_timer(&ctx->tmr_response_wait);

    DMR_LOGI("[DCDM S%d] state=%s pending_kind=%s pending_active=%d",
             ctx->slot, DMO_STATE_NAMES[ctx->state],
             DMO_PENDING_KIND_NAMES[ctx->pending_kind], ctx->pending_tx_active);

    dmo_intercept_result_t ir = dmo_check_pending_tx_interception(ctx, rx, rx_slot);
    if (ir == DMO_INTERCEPT_CONTINUE) {
        DMR_LOGI("[DCDM S%d] decision: pending %s TX interception -> CONTINUE "
                 "(keeping pending TX alive, ignoring this CT_CSBK)",
                 ctx->slot, DMO_PENDING_KIND_NAMES[ctx->pending_kind]);
        return; /* absorbed by the pending-TX rule; no further evaluation */
    }
    if (ir == DMO_INTERCEPT_CANCEL) {
        DMR_LOGI("[DCDM S%d] decision: pending %s TX interception -> CANCEL "
                 "(dropping pending TX, evaluating this CT_CSBK normally)",
                 ctx->slot, DMO_PENDING_KIND_NAMES[ctx->pending_kind]);
        ctx->pending_tx_active = false;
        ctx->tx_retry_active   = false;
        ctx->pending_kind      = DMO_PENDING_NONE;
        dmo_disarm_timer(&ctx->tmr_ct_rhot);
        /* fall through to ordinary CCE/LDR evaluation below */
    }

    if (ctx->state == DMO_STATE_LEADER) {
        DMR_LOGI("[DCDM S%d] decision: routing to LDR (this MS is LEADER)", ctx->slot);
        dmo_ldr(ctx, rx, rx_slot);
    } else {
        DMR_LOGI("[DCDM S%d] decision: routing to CCE (state=%s)",
                 ctx->slot, DMO_STATE_NAMES[ctx->state]);
        dmo_cce(ctx, rx, rx_slot);
    }
}

/* =========================================================================
 * Timer expiry handlers — Cl.6.2.3.1-6.2.3.5
 * ========================================================================= */

/* NoLeaderTimer — Fig 6.3 (Leader_and_Timing_Unknown) and Fig 6.4
 * (Leader_Unknown) share an identical requirement. */
static void dmo_send_noleader_req(dmr_dmo_ctx_t *ctx)
{
    dmr_ct_csbk_t req;
    memset(&req, 0, sizeof(req));
    req.gen = 0u; req.sync_age = 0u; req.leader_id = 0u; req.leader_di = 0u;
    req.new_leader = false;
    req.source_id  = ctx->ms_id;
    req.source_di  = ctx->ms_di; /* "shall use its MS_DI bits for SDI" */
    req.cto        = DMR_CTO_UNALIGNED_REQ;

    /* Unknown-range CT_RHOT for this attempt */
    ctx->ct_rhot_min_ms = DMO_CT_RHOT_UNKNOWN_MIN_MS;
    ctx->ct_rhot_max_ms = DMO_CT_RHOT_UNKNOWN_MAX_MS;
    dmo_schedule_ct_csbk_tx(ctx, &req, DMO_PENDING_NOLEADER_REQ);
}

static void dmo_handle_no_leader_timer(dmr_dmo_ctx_t *ctx)
{
    DMR_LOGI("[DCDM S%d] NoLeaderTimer expired (state=%s) -> sending CT_CSBK_Req",
             ctx->slot, DMO_STATE_NAMES[ctx->state]);
    ctx->stats.no_leader_timeouts++;
    ctx->noleader_retry_count = 0u; /* fresh bootstrap sequence */
    dmo_send_noleader_req(ctx);
}

/* Response-wait expired with nothing heard back after a NoLeaderTimer-
 * triggered CT_CSBK_Req actually transmitted (Cl.6.2.3.2/3 only cover
 * sending the Req and reacting to a reply — this retry-then-self-
 * promote loop is our own addition; see DMO_RESPONSE_WAIT_MS). */
static void dmo_handle_response_wait_timer(dmr_dmo_ctx_t *ctx)
{
    ctx->noleader_retry_count++;

    if (ctx->noleader_retry_count >= DMO_NOLEADER_MAX_RETRIES) {
        dmo_self_promote_to_leader(ctx);
        return;
    }

    DMR_LOGI("[DCDM S%d] No response to CT_CSBK_Req (attempt %u/%u) — retrying",
             ctx->slot, ctx->noleader_retry_count, DMO_NOLEADER_MAX_RETRIES);
    dmo_send_noleader_req(ctx);
}

/* Retry budget exhausted with zero responses — conclude the channel is
 * unmanaged and become the Channel Timing Leader ourselves, so any
 * radio arriving later has something to synchronize to. Mirrors CCE
 * bullet 1's externally-appointed leader transition (Cl.6.2.3.7). */
static void dmo_self_promote_to_leader(dmr_dmo_ctx_t *ctx)
{
    DMR_LOGI("[DCDM S%d] No response after %u attempts — self-promoting to "
             "LEADER", ctx->slot, DMO_NOLEADER_MAX_RETRIES);

    ctx->ms_gen    = 0u;
    ctx->ms_sa     = 0u;
    ctx->ms_lwatid = ctx->ms_watid; /* we are now our own recognized leader */
    dmo_arm_timer(&ctx->tmr_beacon_interval, ctx->beacon_interval_ms);
    ctx->state = DMO_STATE_LEADER;
    ctx->stats.leader_elections++;
    ctx->stats.self_promotions++;

    dmr_ct_csbk_t beacon;
    memset(&beacon, 0, sizeof(beacon));
    beacon.gen        = 0u;
    beacon.sync_age   = ctx->ms_sa;
    beacon.leader_id  = DMO_WATID_ID(ctx->ms_watid);
    beacon.leader_di  = (uint8_t)DMO_WATID_DI(ctx->ms_watid);
    beacon.source_id  = ctx->ms_id;
    beacon.source_di  = ctx->ms_di;
    beacon.new_leader = false;
    beacon.cto        = DMR_CTO_ALIGNED_PUSH;
    ctx->stats.beacons_sent++;
    dmo_schedule_ct_csbk_tx(ctx, &beacon, DMO_PENDING_OTHER);
}

/* SyncAge expiry — Fig 6.5. Re-initialise timing, drop to Leader_Unknown. */
static void dmo_handle_sync_age_timer(dmr_dmo_ctx_t *ctx)
{
    DMR_LOGI("[DCDM S%d] SyncAge expired -> losing leader, dropping to "
             "LEADER_UNKNOWN", ctx->slot);
    ctx->stats.sync_age_timeouts++;
    ctx->ms_gen    = 0u;
    ctx->ms_sa     = 0u;
    ctx->ms_lwatid = 0u;
    ctx->state = DMO_STATE_LEADER_UNKNOWN;
    dmo_arm_timer(&ctx->tmr_no_leader, ctx->no_leader_timer_ms);
}

/* SyncAgeWarning expiry — Fig 6.5. Request a refresh from the leader. */
static void dmo_handle_sync_age_warning_timer(dmr_dmo_ctx_t *ctx)
{
    DMR_LOGI("[DCDM S%d] SyncAgeWarning expired -> requesting refresh from "
             "leader", ctx->slot);
    dmr_ct_csbk_t req;
    memset(&req, 0, sizeof(req));
    req.gen        = ctx->ms_gen;
    req.sync_age   = ctx->ms_sa;
    req.leader_id  = DMO_WATID_ID(ctx->ms_lwatid);
    req.leader_di  = (uint8_t)DMO_WATID_DI(ctx->ms_lwatid);
    req.new_leader = false;
    req.source_id  = ctx->ms_id;
    req.source_di  = 0u;
    req.cto        = DMR_CTO_ALIGNED_STATUS; /* 10 — aligned, we know timing */

    ctx->ct_rhot_min_ms = DMO_CT_RHOT_KNOWN_MIN_MS;
    ctx->ct_rhot_max_ms = DMO_CT_RHOT_KNOWN_MAX_MS;
    dmo_schedule_ct_csbk_tx(ctx, &req, DMO_PENDING_SYNCAGEWARNING_REQ);
}

/* BeaconInterval expiry — Fig 6.6. Only meaningful in LEADER state. */
static void dmo_handle_beacon_interval_timer(dmr_dmo_ctx_t *ctx)
{
    DMR_LOGI("[DCDM S%d] BeaconInterval expired -> sending periodic beacon",
             ctx->slot);
    dmr_ct_csbk_t beacon;
    memset(&beacon, 0, sizeof(beacon));
    beacon.gen        = 0u;
    beacon.sync_age   = ctx->ms_sa;
    beacon.leader_id  = DMO_WATID_ID(ctx->ms_watid);
    beacon.leader_di  = (uint8_t)DMO_WATID_DI(ctx->ms_watid);
    beacon.new_leader = false;
    beacon.source_id  = ctx->ms_id;
    beacon.source_di  = ctx->ms_di;
    beacon.cto        = DMR_CTO_ALIGNED_PUSH;

    ctx->ct_rhot_min_ms = DMO_CT_RHOT_KNOWN_MIN_MS;
    ctx->ct_rhot_max_ms = DMO_CT_RHOT_KNOWN_MAX_MS;
    ctx->stats.beacons_sent++;
    dmo_schedule_ct_csbk_tx(ctx, &beacon, DMO_PENDING_OTHER);

    dmo_arm_timer(&ctx->tmr_beacon_interval, ctx->beacon_interval_ms);
}

/* =========================================================================
 * Transmit procedure — Cl.6.2.3.12, Fig 6.13. Sets CTO on the
 * CT_CSBK_Term sent right after any non-CT_CSBK transmission. No
 * CT_RHOT holdoff — spec says Term is sent "immediately following".
 * ========================================================================= */
dmr_err_t dmr_dmo_notify_tx(dmr_dmo_ctx_t *ctx, bool channel_activity)
{
    if (ctx == NULL || !ctx->initialised) return DMR_ERR_INVALID_PARAM;

    DMR_LOGI("[DCDM S%d] notify_tx: app transmitted, channel_activity=%d "
             "-> sending CT_CSBK_Term", ctx->slot, channel_activity);
    dmr_ct_csbk_t term;
    memset(&term, 0, sizeof(term));
    term.gen        = ctx->ms_gen;
    term.sync_age   = ctx->ms_sa;
    term.leader_id  = DMO_WATID_ID(ctx->ms_lwatid);
    term.leader_di  = (uint8_t)DMO_WATID_DI(ctx->ms_lwatid);
    term.new_leader = false;
    term.source_id  = ctx->ms_id;
    term.source_di  = 0u; /* "shall set SDI bits to 00" — uniform across states */

    if (ctx->ms_lwatid == 0u) {
        term.cto = DMR_CTO_UNALIGNED_REQ;   /* 00 — no leader */
    } else if (channel_activity) {
        term.cto = DMR_CTO_UNALIGNED_TERM;  /* 01 — leader known, channel busy */
    } else {
        term.cto = DMR_CTO_ALIGNED_STATUS;  /* 10 — leader known, channel clear */
    }

    dmr_mac_tx_req_t req;
    memset(&req, 0, sizeof(req));
    req.slot            = ctx->slot;
    req.priority        = DMR_MAC_PRIORITY_NORMAL;
    req.impolite         = false;
    req.originated_from = DCDM_TX_ORIGIN_DCDM;
    req.req_id          = ctx->tx_req_id_next++;
    llc_ct_csbk_build(&req.burst, &term, ctx->slot);

    if (mac_tx_enqueue(ctx->mq_mac_tx, &req) == DMR_OK) {
        ctx->stats.ct_csbk_tx++;
    }
    return DMR_OK;
}

/* =========================================================================
 * Public RX entry point
 * ========================================================================= */
dmr_err_t dmr_dmo_rx_burst(dmr_dmo_ctx_t *ctx, const dmr_burst_t *burst)
{
    if (ctx == NULL || !ctx->initialised || burst == NULL) {
        return DMR_ERR_INVALID_PARAM;
    }

    /* burst->raw is still raw wire format (BPTC-encoded/interleaved) —
     * llc_csbk_opcode()/llc_ct_csbk_parse() both expect the decoded
     * 12-byte logical PDU body. llc_burst_unpack() runs FEC/BPTC decode
     * (via fec_rx_process on an internal copy) and extracts that body. */
    uint8_t pdu12[12];
    uint8_t dtype, cc;
    if (llc_burst_unpack(burst->raw, pdu12, &dtype, &cc) != DMR_OK) {
        return DMR_OK; /* not a data burst, or FEC uncorrectable — not for us */
    }
    if (dtype != DMR_DTYPE_CSBK) {
        return DMR_OK; /* not a CSBK at all */
    }

    uint8_t opcode = llc_csbk_opcode(pdu12);
    if (opcode != DMR_CSBKO_CHANNEL_TIMING) {
        return DMR_OK; /* not for us */
    }

    dmr_ct_csbk_t ct;
    if (llc_ct_csbk_parse(pdu12, &ct) == DMR_ERR_CRC) {
        return DMR_OK; /* CRC failed — cannot trust contents; drop */
    }

    pthread_mutex_lock(&ctx->state_mutex);
    dmo_dispatch_rx(ctx, &ct, (dmr_slot_t)burst->timeslot);
    pthread_mutex_unlock(&ctx->state_mutex);
    return DMR_OK;
}

/* =========================================================================
 * Accessors
 * ========================================================================= */
dmo_state_t dmr_dmo_get_state(dmr_dmo_ctx_t *ctx)
{
    if (ctx == NULL) return DMO_STATE_LEADER_AND_TIMING_UNKNOWN;
    pthread_mutex_lock(&ctx->state_mutex);
    dmo_state_t s = ctx->state;
    pthread_mutex_unlock(&ctx->state_mutex);
    return s;
}

void dmr_dmo_get_stats(dmr_dmo_ctx_t *ctx, dmo_stats_t *out)
{
    if (ctx == NULL || out == NULL) return;
    pthread_mutex_lock(&ctx->state_mutex);
    *out = ctx->stats;
    pthread_mutex_unlock(&ctx->state_mutex);
}

/* =========================================================================
 * Worker thread
 * ========================================================================= */
static void *dmo_thread(void *arg)
{
    dmr_dmo_ctx_t *ctx = (dmr_dmo_ctx_t *)arg;
    struct epoll_event events[DMO_EPOLL_MAX_EVENTS];

    DMR_LOGI("[DCDM S%d] Worker thread started (radio_id=0x%06X ms_di=%u ms_id=0x%05X)",
             ctx->slot, ctx->my_radio_id, ctx->ms_di, ctx->ms_id);

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        DMR_LOGE("[DCDM S%d] epoll_create1 failed: %s", ctx->slot, strerror(errno));
        return NULL;
    }

    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_ct_rhot);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_ct_rhot), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_no_leader);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_no_leader), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_sync_age);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_sync_age), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_sync_age_warning);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_sync_age_warning), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_beacon_interval);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_beacon_interval), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_response_wait);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_response_wait), &ev);
    ev.data.fd = (int)ctx->mq_evt;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_evt, &ev);
    ev.data.fd = (int)ctx->mq_mac_conf;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_mac_conf, &ev);
    ev.data.fd = (int)ctx->mq_mac_rx;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_mac_rx, &ev);

    while (ctx->running) {
        int nev = epoll_wait(epfd, events, DMO_EPOLL_MAX_EVENTS, 200);
        if (nev < 0) {
            if (errno == EINTR) continue;
            DMR_LOGE("[DCDM S%d] epoll_wait: %s", ctx->slot, strerror(errno));
            break;
        }

        for (int i = 0; i < nev; i++) {
            int fd = events[i].data.fd;

            pthread_mutex_lock(&ctx->state_mutex);

            if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_ct_rhot)) {
                dmo_drain_timer(&ctx->tmr_ct_rhot);
                dmo_handle_ct_rhot_expiry(ctx);
            } else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_no_leader)) {
                dmo_drain_timer(&ctx->tmr_no_leader);
                if (ctx->state == DMO_STATE_LEADER_AND_TIMING_UNKNOWN ||
                    ctx->state == DMO_STATE_LEADER_UNKNOWN) {
                    dmo_handle_no_leader_timer(ctx);
                }
            } else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_sync_age)) {
                dmo_drain_timer(&ctx->tmr_sync_age);
                if (ctx->state == DMO_STATE_LEADER_AND_TIMING_KNOWN) {
                    dmo_handle_sync_age_timer(ctx);
                }
            } else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_sync_age_warning)) {
                dmo_drain_timer(&ctx->tmr_sync_age_warning);
                if (ctx->state == DMO_STATE_LEADER_AND_TIMING_KNOWN) {
                    dmo_handle_sync_age_warning_timer(ctx);
                }
            } else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_beacon_interval)) {
                dmo_drain_timer(&ctx->tmr_beacon_interval);
                if (ctx->state == DMO_STATE_LEADER) {
                    dmo_handle_beacon_interval_timer(ctx);
                }
            } else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_response_wait)) {
                dmo_drain_timer(&ctx->tmr_response_wait);
                if (ctx->state == DMO_STATE_LEADER_AND_TIMING_UNKNOWN ||
                    ctx->state == DMO_STATE_LEADER_UNKNOWN) {
                    dmo_handle_response_wait_timer(ctx);
                }
            } else if (fd == (int)ctx->mq_evt) {
                dmo_event_t e;
                while (mq_receive(ctx->mq_evt, (char *)&e, sizeof(e), NULL) >= 0) {
                    if (e.type == DMO_EVT_SHUTDOWN) {
                        ctx->running = false;
                    }
                }
            } else if (fd == (int)ctx->mq_mac_conf) {
                dmr_mac_tx_conf_t conf;
                while (mq_receive(ctx->mq_mac_conf, (char *)&conf, sizeof(conf), NULL) >= 0) {
                    dmo_handle_tx_conf(ctx, &conf);
                }
            } else if (fd == (int)ctx->mq_mac_rx) {
                dmr_burst_t burst;
                while (mq_receive(ctx->mq_mac_rx, (char *)&burst, sizeof(burst), NULL) >= 0) {
                    /* burst.raw is raw wire format — decode it first
                     * (see the matching comment in dmr_dmo_rx_burst()). */
                    uint8_t pdu12[12];
                    uint8_t dtype, cc;
                    if (llc_burst_unpack(burst.raw, pdu12, &dtype, &cc) != DMR_OK) {
                        continue;
                    }
                    if (dtype != DMR_DTYPE_CSBK) {
                        continue;
                    }
                    uint8_t opcode = llc_csbk_opcode(pdu12);
                    if (opcode == DMR_CSBKO_CHANNEL_TIMING) {
                        dmr_ct_csbk_t ct;
                        if (llc_ct_csbk_parse(pdu12, &ct) != DMR_ERR_CRC) {
                            dmo_dispatch_rx(ctx, &ct, (dmr_slot_t)burst.timeslot);
                        }
                    }
                }
            }

            pthread_mutex_unlock(&ctx->state_mutex);
        }
    }

    close(epfd);
    DMR_LOGI("[DCDM S%d] Worker thread exiting", ctx->slot);
    return NULL;
}

/* =========================================================================
 * Lifecycle
 * ========================================================================= */
dmr_err_t dmr_dmo_init(dmr_dmo_ctx_t *ctx,
                       const dmr_tier2_config_t *cfg,
                       uint32_t my_radio_id,
                       uint8_t colour_code)
{
    if (ctx == NULL || cfg == NULL) return DMR_ERR_INVALID_PARAM;

    memset(ctx, 0, sizeof(*ctx));
    ctx->slot         = cfg->fixed_slot;
    ctx->my_radio_id  = my_radio_id;
    ctx->colour_code  = colour_code;
    ctx->dcdm_enabled = cfg->dcdm_enabled;

    if (pthread_mutex_init(&ctx->state_mutex, NULL) != 0) {
        return DMR_ERR_INVALID_PARAM;
    }

    /* Cl.6.2.3.1 Power up and channel change SDL */
    ctx->rand_state = my_radio_id ? my_radio_id : 0xC0FFEEu;
    ctx->ms_di    = (cfg->leader_di <= 3u) ? cfg->leader_di : DMO_DI_MEDIUM;
    ctx->ms_id    = (my_radio_id & 0xFFFFFu) ? (my_radio_id & 0xFFFFFu) : 1u;
    ctx->ms_watid = DMO_WATID(ctx->ms_di, ctx->ms_id);
    ctx->ms_gen    = 0u;
    ctx->ms_sa     = 0u;
    ctx->ms_lwatid = 0u;
    ctx->state     = DMO_STATE_LEADER_AND_TIMING_UNKNOWN;

    ctx->ct_rhot_min_ms = DMO_CT_RHOT_UNKNOWN_MIN_MS;
    ctx->ct_rhot_max_ms = DMO_CT_RHOT_UNKNOWN_MAX_MS;
    ctx->prop_ct_rhot_min_ms = DMO_CT_RHOT_KNOWN_MIN_MS;
    ctx->prop_ct_rhot_max_ms = DMO_CT_RHOT_KNOWN_MAX_MS;
    ctx->no_leader_timer_ms    = DMO_NO_LEADER_TIMER_MS;
    ctx->response_wait_ms      = DMO_RESPONSE_WAIT_MS;
    ctx->sync_age_ms           = DMO_SYNC_AGE_MS;
    ctx->sync_age_warning_ms   = DMO_SYNC_AGE_WARNING_MS;
    ctx->beacon_interval_ms    = DMO_BEACON_INTERVAL_MS;
    ctx->tx_req_id_next = 1u;

    const char *mq_evt_name  = (ctx->slot == DMR_SLOT_1)
                               ? DMR_MQ_DMO_EVT_S1 : DMR_MQ_DMO_EVT_S2;
    const char *mq_tx_name   = (ctx->slot == DMR_SLOT_1)
                               ? DMR_MQ_MAC_TX_REQ_S1 : DMR_MQ_MAC_TX_REQ_S2;
    const char *mq_conf_name = (ctx->slot == DMR_SLOT_1)
                               ? DMR_MQ_MAC_TX_CONF_DCDM_S1 : DMR_MQ_MAC_TX_CONF_DCDM_S2;
    const char *mq_rx_name   = (ctx->slot == DMR_SLOT_1)
                               ? DMR_MQ_MAC_RX_DCDM_S1 : DMR_MQ_MAC_RX_DCDM_S2;

    ctx->mq_evt = dmo_mq_create_own(mq_evt_name, DMO_MQ_EVT_MAX_MSGS, sizeof(dmo_event_t));

    dmr_err_t mac_q_err = dmo_open_mac_queues(mq_tx_name, mq_conf_name, mq_rx_name,
                                               &ctx->mq_mac_tx, &ctx->mq_mac_conf,
                                               &ctx->mq_mac_rx);
    if (ctx->mq_evt == (mqd_t)-1 || mac_q_err != DMR_OK) {
        dmr_dmo_destroy(ctx);
        return DMR_ERR_QUEUE_FULL;
    }

    if (dmr_phy_timer_oneshot_init(&ctx->tmr_ct_rhot)          != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_no_leader)        != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_sync_age)         != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_sync_age_warning) != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_beacon_interval)  != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_response_wait)  != DMR_OK) {
        DMR_LOGE("[DCDM] timerfd_create failed: %s", strerror(errno));
        dmr_dmo_destroy(ctx);
        return DMR_ERR_NO_MEM;
    }

    ctx->initialised = true;
    return DMR_OK;
}

dmr_err_t dmr_dmo_start(dmr_dmo_ctx_t *ctx)
{
    if (ctx == NULL || !ctx->initialised) return DMR_ERR_INVALID_PARAM;
    if (ctx->running) return DMR_OK;

    /* Cl.6.2.3.1: "Set(NoLeaderTimer)" then enter Leader_and_Timing_Unknown.
     * DCDM-vs-conventional is a static config choice (dcdm_enabled) —
     * this module only ever runs when that's already true, so there's
     * no probing step here. */
    dmo_arm_timer(&ctx->tmr_no_leader, ctx->no_leader_timer_ms);

    ctx->running = true;
    if (pthread_create(&ctx->thread, NULL, dmo_thread, ctx) != 0) {
        ctx->running = false;
        return DMR_ERR_INVALID_PARAM;
    }
    return DMR_OK;
}

dmr_err_t dmr_dmo_stop(dmr_dmo_ctx_t *ctx)

{
    if (ctx == NULL || !ctx->initialised) return DMR_ERR_INVALID_PARAM;
    if (!ctx->running) return DMR_OK;

    if (ctx->mq_evt != (mqd_t)-1) {
        dmo_event_t e;
        memset(&e, 0, sizeof(e));
        e.type = DMO_EVT_SHUTDOWN;
        e.timestamp_us = dmr_time_now_us();
        mq_send(ctx->mq_evt, (const char *)&e, sizeof(e), 0u);
    }
    ctx->running = false;
    pthread_join(ctx->thread, NULL);
    return DMR_OK;
}

void dmr_dmo_destroy(dmr_dmo_ctx_t *ctx)
{
    if (ctx == NULL) return;
    if (!ctx->initialised) {
        memset(ctx, 0, sizeof(*ctx));
        return;
    }
    if (ctx->running) {
        dmr_dmo_stop(ctx);
    }

    dmo_disarm_timer(&ctx->tmr_ct_rhot);
    dmo_disarm_timer(&ctx->tmr_no_leader);
    dmo_disarm_timer(&ctx->tmr_sync_age);
    dmo_disarm_timer(&ctx->tmr_sync_age_warning);
    dmo_disarm_timer(&ctx->tmr_beacon_interval);
    dmo_disarm_timer(&ctx->tmr_response_wait);
    
    dmr_phy_timer_oneshot_destroy(&ctx->tmr_ct_rhot);
    dmr_phy_timer_oneshot_destroy(&ctx->tmr_no_leader);
    dmr_phy_timer_oneshot_destroy(&ctx->tmr_sync_age);
    dmr_phy_timer_oneshot_destroy(&ctx->tmr_sync_age_warning);
    dmr_phy_timer_oneshot_destroy(&ctx->tmr_beacon_interval);
    dmr_phy_timer_oneshot_destroy(&ctx->tmr_response_wait);

    if (ctx->mq_evt      != (mqd_t)-1) mq_close(ctx->mq_evt);
    if (ctx->mq_mac_tx   != (mqd_t)-1) mq_close(ctx->mq_mac_tx);
    if (ctx->mq_mac_conf != (mqd_t)-1) mq_close(ctx->mq_mac_conf);
    if (ctx->mq_mac_rx   != (mqd_t)-1) mq_close(ctx->mq_mac_rx);

    /* This module is the sole creator of mq_evt only — unlink just that
     * one (mirrors MOD-07's ownership contract for its own mq_evt). */
    const char *mq_evt_name = (ctx->slot == DMR_SLOT_1)
                              ? DMR_MQ_DMO_EVT_S1 : DMR_MQ_DMO_EVT_S2;
    mq_unlink(mq_evt_name);

    pthread_mutex_destroy(&ctx->state_mutex);
    memset(ctx, 0, sizeof(*ctx));
}