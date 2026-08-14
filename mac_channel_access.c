/**

/**

/**
 * @file mac_channel_access.c
 * @brief MOD-03 — MAC Channel Access State Machine + LBT implementation
 *
 * ETSI TS 102 361-1, Clauses 5.2–5.4
 *
 * Implements the Listen-Before-Transmit (LBT) polite protocol:
 *
 *   IDLE_MONITOR ──PTT/TX_req──► QUALIFY_IDLE (start T_IdleSrch)
 *       ▲                              │
 *       │                    channel busy detected
 *       │                              ▼
 *       │                         HOLDOFF (random T_Holdoff)
 *       │                              │
 *       │                      T_Holdoff expired
 *       │                              ▼
 *       │                         QUALIFY_IDLE (retry)
 *       │                              │
 *       │                    T_IdleSrch expired (idle confirmed)
 *       │                              ▼
 *       │                          TX_PENDING (wait for slot window)
 *       │                              │
 *       │                    slot window arrived
 *       │                              ▼
 *       │                         TRANSMITTING
 *       │                              │
 *       │              burst sent / T_DataTxLmt expired
 *       │                              ▼
 *       └──────────────────────── TX_ABORT / IDLE_MONITOR
 *
 * Timer mapping (Linux timerfd_create + epoll):
 *   tfd_idle_srch  → T_IdleSrch  (30 ms, CLOCK_MONOTONIC, one-shot)
 *   tfd_holdoff    → T_Holdoff   (random 0–120 ms, one-shot)
 *   tfd_tx_lmt     → T_DataTxLmt (120 ms, started when TX_PENDING, one-shot)
 *
 * PHY channel busy detection:
 *   The PHY layer sets AT bit in CACH when inbound is busy.
 *   If no valid DMR burst arrives within T_IdleSrch, channel is idle.
 *   If a burst arrives during QUALIFY_IDLE, channel is busy → HOLDOFF.
 *
 * Random holdoff computation:
 *   T_Holdoff = rand() % (MAC_MAX_HOLDOFF_BURSTS + 1) × MAC_TIMESLOT_MS
 *   (0–4 burst-lengths = 0–120 ms, uniform distribution)
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <mqueue.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>

#include "dmr_pdu.h"
#include "dmr_mac.h"
#include "dmr_types.h"
#include "dmr_llc.h"

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Arm a one-shot PHY timer in milliseconds */
static void mac_timer_arm_ms(dmr_phy_timer_oneshot_t *t, uint32_t ms)
{
    dmr_phy_timer_oneshot_arm_ms(t, ms);
}

/* Disarm a one-shot PHY timer */
static void mac_timer_disarm(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_disarm(t);
}

/* Drain a one-shot PHY timer after expiry */
static void mac_timer_drain(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_drain(t);
}

/* Compute random holdoff in milliseconds.
 * Uses this ctx's own rng_state via rand_r() rather than the process-global
 * rand()/srand(), which would otherwise be shared (and raced on) across the
 * independent per-slot MAC threads. */
static uint32_t random_holdoff_ms(mac_ctx_t *ctx)
{
    uint32_t bursts = (uint32_t)(rand_r(&ctx->rng_state) % (MAC_MAX_HOLDOFF_BURSTS + 1u));
    return bursts * MAC_TIMESLOT_MS;
}

/* Set MAC channel access state with logging */
static void mac_set_state(mac_ctx_t *ctx, mac_ch_access_state_t new_state)
{
    pthread_mutex_lock(&ctx->state_mutex);
    mac_ch_access_state_t old = ctx->ch_state;
    ctx->ch_state = new_state;
    pthread_mutex_unlock(&ctx->state_mutex);

    if (old != new_state) {
        DMR_LOGI("[MAC S%d] %s → %s",
                 ctx->slot,
                 MAC_STATE_NAMES[old],
                 MAC_STATE_NAMES[new_state]);
    }
}

/* Post TX confirmation to the CCL confirmation queue */
void mac_post_tx_conf(mac_ctx_t *ctx, uint32_t req_id,
                      dmr_mac_tx_result_t result, uint64_t tx_time,uint8_t origin_of_req)
{
    dmr_mac_tx_conf_t conf = {
        .req_id       = req_id,
        .result       = result,
        .actual_tx_us = tx_time,
    };

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    /* Non-blocking send — CCL must consume promptly */
  switch(origin_of_req) 
  {
    case    CCL_TX_ORIGIN_VOICE:
         if (mq_send(ctx->mq_tx_voice_conf, (const char *)&conf, sizeof(conf), 1u) < 0) {
                    DMR_LOGE("[MAC S%d] mq_send tx_conf failed: %s", ctx->slot, strerror(errno));
                }
        break;
    case    CCL_TX_ORIGIN_DATA:
         if (mq_send(ctx->mq_tx_data_conf, (const char *)&conf, sizeof(conf), 1u) < 0) {
                    DMR_LOGE("[MAC S%d] mq_send tx_conf failed: %s", ctx->slot, strerror(errno));
                }
        break;
    case    T3_TX_ORIGIN_TRUNK:
         if (mq_send(ctx->mq_tx_trunk_conf, (const char *)&conf, sizeof(conf), 1u) < 0) {
                    DMR_LOGE("[MAC S%d] mq_send tx_conf failed: %s", ctx->slot, strerror(errno));
                }
        break;
    case    DCDM_TX_ORIGIN_DCDM:
         if (mq_send(ctx->mq_tx_dcdm_conf, (const char *)&conf, sizeof(conf), 1u) < 0) {
                    DMR_LOGE("[MAC S%d] mq_send tx_conf failed: %s", ctx->slot, strerror(errno));
                }
        break;
        default:
             printf("UNKNOWN ORIGIN\n");
        break;

    }
}

/* =========================================================================
 * State transitions
 * ========================================================================= */

void mac_lbt_start(mac_ctx_t *ctx)
{
    mac_set_state(ctx, MAC_STATE_QUALIFY_IDLE);
    mac_timer_arm_ms(&ctx->tmr_idle_srch, MAC_T_IDLE_SRCH_MS);
    DMR_LOGT("[MAC S%d] T_IdleSrch armed (%u ms)", ctx->slot, MAC_T_IDLE_SRCH_MS);
}

void mac_lbt_channel_busy(mac_ctx_t *ctx)
{
    mac_timer_disarm(&ctx->tmr_idle_srch);
    ctx->lbt_holdoff_count++;
    ctx->holdoff_count++;

    /* NOTE: T_DataTxLmt is enforced by the real tmr_tx_lmt timerfd (armed in
     * mac_handle_tx_req(), dispatched in the event loop via
     * mac_tx_lmt_expired()) which tracks actual elapsed wall-clock time.
     * There used to be a second, count-based abort check here
     * (holdoff_count >= 2) that could fire far earlier than the real 120 ms
     * deadline, since each holdoff draw is itself random (0-120 ms) — two
     * unlucky short draws could trip it after only a few real milliseconds.
     * Removed as redundant and incorrect; the timerfd is authoritative. */

    uint32_t ho_ms = random_holdoff_ms(ctx);
    DMR_LOGD("[MAC S%d] Channel busy — holdoff %u ms (retry %u)",
             ctx->slot, ho_ms, ctx->holdoff_count);

    mac_set_state(ctx, MAC_STATE_HOLDOFF);
    mac_timer_arm_ms(&ctx->tmr_holdoff, ho_ms > 0u ? ho_ms : 1u);
}

void mac_holdoff_expired(mac_ctx_t *ctx)
{
    DMR_LOGT("[MAC S%d] T_Holdoff expired — retrying LBT", ctx->slot);
    mac_lbt_start(ctx);
}

void mac_idle_srch_expired(mac_ctx_t *ctx)
{
    DMR_LOGD("[MAC S%d] T_IdleSrch expired — channel idle, scheduling TX",
             ctx->slot);

    /* Disarm T_DataTxLmt (was started when request was accepted) */
    mac_timer_disarm(&ctx->tmr_tx_lmt);

    mac_set_state(ctx, MAC_STATE_TX_PENDING);
    mac_tx_slot_schedule(ctx, &ctx->pending_req);
}

void mac_tx_lmt_expired(mac_ctx_t *ctx)
{
    DMR_LOGW("[MAC S%d] T_DataTxLmt expired — aborting TX req_id=%u",
             ctx->slot, ctx->pending_req.req_id);

    mac_timer_disarm(&ctx->tmr_idle_srch);
    mac_timer_disarm(&ctx->tmr_holdoff);

    ctx->tx_abort_count++;
    ctx->has_pending    = false;
    ctx->holdoff_count  = 0;

    mac_post_tx_conf(ctx, ctx->pending_req.req_id,
                     DMR_MAC_TX_ABORTED, 0u, ctx->pending_req.originated_from);

    mac_set_state(ctx, MAC_STATE_IDLE_MONITOR);
}




void post_conf_from_phy(mac_ctx_t *ctx,dmr_phy_tx_conf_t *conf){

     mac_set_state(ctx, MAC_STATE_TRANSMITTING);
     
     
     #ifdef PRINTFDATA         
    
   printf("\n{0x%x",ctx->pending_req.burst.raw[0]);
    for(int i=1;i<33;i++)
    printf(",0x%x",ctx->pending_req.burst.raw[i]);
     printf("}\n");
    
#endif


    if( conf->result == DMR_PHY_TX_DONE_OK)
     {
     	mac_post_tx_conf(ctx, conf->req_id, DMR_MAC_TX_OK, 0u,conf->originated_from);
     	
    ctx->tx_burst_count++;
    ctx->has_pending   = false;
    ctx->holdoff_count = 0;
    
     	    DMR_LOGD("[MAC S%d] TX burst dispatched req_id=%u dtype=0x%02X  from SRC:%d",
             ctx->slot, ctx->pending_req.req_id,
             dmr_burst_get_dtype(ctx->pending_req.burst.raw),ctx->pending_req.originated_from);
     }
	else
	{
	    ctx->tx_abort_count ++;
	    ctx->has_pending   = false;
        ctx->holdoff_count = 0;
		mac_post_tx_conf(ctx, conf->req_id, DMR_MAC_TX_ABORTED, 0u,conf->originated_from);
	}
     
    mac_set_state(ctx, MAC_STATE_IDLE_MONITOR);
     
}
/* =========================================================================
 * TX scheduling
 * ========================================================================= */

/*
 * mac_tx_slot_schedule — transmit the burst in the next available 30 ms window.
 *
 * In a real implementation this calls into the PHY timing engine to get the
 * next TDMA slot boundary and schedules the burst there.
 *
 * Here we send immediately to the PHY TX queue (the PHY owns slot timing).
 * We set TRANSMITTING, push to mq_phy_tx, update stats, post TX_OK conf.
 */
void mac_tx_slot_schedule(mac_ctx_t *ctx, const dmr_mac_tx_req_t *req)
{
    

    uint64_t tx_time = dmr_time_now_us();

    /* Build AT/TC for CACH — mark our slot as busy */
    uint8_t tc = (ctx->slot == DMR_SLOT_1) ? 0u : 1u;
    mac_cach_build(&ctx->cach_tx, 1u /*AT=busy*/, tc, DMR_LCSS_SINGLE, 0u);

    /* Push to PHY TX queue (non-blocking) */
  if (mq_send(ctx->mq_phy_tx,
               (const char *)req,
               sizeof(dmr_mac_tx_req_t), 0u) < 0) {
         mac_tx_cancel(ctx, req->req_id);          
        DMR_LOGE("[MAC S%d] mq_send phy_tx failed: %s", ctx->slot, strerror(errno));
       
    }

}



/* =========================================================================
 * Public: TX request submission
 * ========================================================================= */
#ifndef TEST_CODE
dmr_err_t mac_tx_enqueue(mqd_t mq_tx, const dmr_mac_tx_req_t *req)
{
    if (mq_send(mq_tx, (const char *)req, sizeof(*req), 1u) < 0) {
        if (errno == EAGAIN) return DMR_ERR_QUEUE_FULL;
        return DMR_ERR_QUEUE_FULL;
    }
    return DMR_OK;
}
#endif

dmr_err_t mac_tx_request(mac_ctx_t *ctx, const dmr_mac_tx_req_t *req)
{
    return mac_tx_enqueue(ctx->mq_tx_req, req);
}

dmr_err_t mac_tx_cancel(mac_ctx_t *ctx, uint32_t req_id)
{
    pthread_mutex_lock(&ctx->state_mutex);
    bool found = (ctx->has_pending && ctx->pending_req.req_id == req_id);
    if (found) {
        ctx->has_pending = false;
        mac_timer_disarm(&ctx->tmr_idle_srch);
        mac_timer_disarm(&ctx->tmr_holdoff);
        mac_timer_disarm(&ctx->tmr_tx_lmt);
        ctx->holdoff_count = 0;
    }
    pthread_mutex_unlock(&ctx->state_mutex);

    if (found) {
        mac_post_tx_conf(ctx, req_id, DMR_MAC_TX_CANCELLED, 0u,ctx->pending_req.originated_from);
        mac_set_state(ctx, MAC_STATE_IDLE_MONITOR);
        return DMR_OK;
    }
    return DMR_ERR_INVALID_PARAM;
}

/* =========================================================================
 * Public: RX burst delivery to CCL
 * ========================================================================= */

dmr_err_t mac_rx_burst_get(mqd_t mq_rx, dmr_burst_t *burst, int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }

    ssize_t n;
    if (timeout_ms < 0) {
        /* Block indefinitely */
        n = mq_receive(mq_rx, (char *)burst, sizeof(*burst), NULL);
    } else {
        n = mq_timedreceive(mq_rx, (char *)burst, sizeof(*burst), NULL, &ts);
    }

    if (n < 0) {
        if (errno == ETIMEDOUT || errno == EAGAIN) return DMR_ERR_TIMEOUT;
        return DMR_ERR_QUEUE_EMPTY;
    }
    return DMR_OK;
}

dmr_err_t mac_tx_wait_conf(mqd_t mq_conf, dmr_mac_tx_conf_t *conf,
                            int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }

    ssize_t n;
    if (timeout_ms < 0) {
        n = mq_receive(mq_conf, (char *)conf, sizeof(*conf), NULL);
    } else {
        n = mq_timedreceive(mq_conf, (char *)conf, sizeof(*conf), NULL, &ts);
    }

    if (n < 0) {
        if (errno == ETIMEDOUT || errno == EAGAIN) return DMR_ERR_TIMEOUT;
        return DMR_ERR_QUEUE_EMPTY;
    }
    return DMR_OK;
}

/* =========================================================================
 * RC burst transmission
 * ========================================================================= */

dmr_err_t mac_rc_burst_tx(mac_ctx_t *ctx, uint8_t rc_payload)
{
    /*
     * RC burst is 96 bits — a standalone reverse channel signalling burst.
     * We build a dmr_burst_t with type=DMR_BURST_TYPE_RC and push
     * directly to the PHY TX queue (impolite — RC does not use LBT).
     *
     * The actual BPTC encoding of the RC payload is handled by MOD-02
     * (FEC engine). Here we store the raw 4-bit payload in the burst
     * and mark it for RC encoding by the burst processor.
     *
     * ETSI TS 102 361-1 Clause 6.4.1
     */
    dmr_burst_t rc_burst;
    memset(&rc_burst, 0, sizeof(rc_burst));
    rc_burst.type     = DMR_BURST_TYPE_RC;
    rc_burst.timeslot = (uint8_t)ctx->slot;

    /* Write RC SYNC pattern into the burst at the standard SYNC position.
     * For RC bursts the SYNC is at the same position as traffic bursts.  */
    dmr_burst_set_sync(rc_burst.raw, DMR_SYNC_RC);

    /* Store the 4-bit RC payload in INFO_1 byte 0 (will be BPTC-encoded
     * by MOD-02 before actual transmission) */
    rc_burst.raw[0] = rc_payload & 0x0Fu;

    DMR_LOGD("[MAC S%d] RC burst TX rc_cmd=0x%02X", ctx->slot, rc_payload);

    if (mq_send(ctx->mq_phy_tx,
                (const char *)&rc_burst,
                sizeof(rc_burst), 0u) < 0) {
        return DMR_ERR_BUSY;
    }
    return DMR_OK;
}

/* =========================================================================
 * Query functions
 * ========================================================================= */

mac_ch_access_state_t mac_get_ch_state(mac_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);
    mac_ch_access_state_t s = ctx->ch_state;
    pthread_mutex_unlock(&ctx->state_mutex);
    return s;
}

void mac_get_stats(mac_ctx_t *ctx,
                   uint64_t *tx_bursts, uint64_t *rx_bursts,
                   uint64_t *holdoffs,  uint64_t *aborts)
{
    pthread_mutex_lock(&ctx->state_mutex);
    if (tx_bursts) *tx_bursts = ctx->tx_burst_count;
    if (rx_bursts) *rx_bursts = ctx->rx_burst_count;
    if (holdoffs)  *holdoffs  = ctx->lbt_holdoff_count;
    if (aborts)    *aborts    = ctx->tx_abort_count;
    pthread_mutex_unlock(&ctx->state_mutex);
}

/* =========================================================================
 * Process an incoming TX request from the CCL mqueue
 * ========================================================================= */

static void mac_handle_tx_req(mac_ctx_t *ctx, const dmr_mac_tx_req_t *req)
{
    /* Deadline check */
    if (req->deadline_us > 0u) {
        uint64_t now = dmr_time_now_us();
        if (now >= req->deadline_us) {
            DMR_LOGW("[MAC S%d] TX req_id=%u deadline already passed",
                     ctx->slot, req->req_id);
                   
            mac_post_tx_conf(ctx, req->req_id, DMR_MAC_TX_DEADLINE, 0u,req->originated_from);
            return;
        }
    }

    /* Store as pending. Locked because mac_tx_cancel() (called from the
     * application/CCL thread, not this MAC thread) reads and clears these
     * same fields under state_mutex; without the lock here, a fresh
     * request landing at the same moment as a cancel of a stale req_id is
     * a data race. */
    pthread_mutex_lock(&ctx->state_mutex);
    ctx->pending_req  = *req;
    ctx->has_pending  = true;
    ctx->holdoff_count = 0;
    pthread_mutex_unlock(&ctx->state_mutex);

    /* Emergency or impolite: skip LBT, schedule immediately */
    if (req->impolite || req->priority == DMR_MAC_PRIORITY_EMERG) {
        DMR_LOGD("[MAC S%d] Impolite TX req_id=%u", ctx->slot, req->req_id);
        mac_tx_slot_schedule(ctx, req);
        return;
    }

    /* Start T_DataTxLmt — overall deadline for acquiring the channel */
    mac_timer_arm_ms(&ctx->tmr_tx_lmt, MAC_T_DATA_TX_LMT_MS);

    /* Begin LBT */
    mac_lbt_start(ctx);
}

/* =========================================================================
 * RX burst classification — which CCL-side module should receive this
 * burst. See the queue-ownership/fan-out rationale in dmr_mac.h
 * (DMR_MQ_MAC_RX_VOICE/DATA/TRUNK_S1/S2).
 * ========================================================================= */
typedef enum {
    MAC_RX_DEST_VOICE = 0,  /* → mq_rx_voice (CCL Voice)   */
    MAC_RX_DEST_DATA,       /* → mq_rx_data  (CCL Data)    */
    MAC_RX_DEST_TRUNK,      /* → mq_rx_trunk (Trunking)    */
    MAC_RX_DEST_DCDM,       /* → mq_rx_dcdm  (DCDM, MOD-15)*/
    MAC_RX_DEST_DROP,       /* Not classifiable — dropped  */
} mac_rx_dest_t;

/* =========================================================================
 * Voice superframe tracking helpers
 * ========================================================================= */

/**
 * @brief Build and post a synthetic burst event to mq_rx_voice.
 *        raw[] is zeroed; CCL Voice checks burst.type == SYNTHETIC_EVT
 *        before doing anything with raw[].
 */
static void mac_voice_inject_synth(mac_ctx_t *ctx,
                                    mac_synth_event_t event,
                                    uint8_t           pos)
{
    dmr_burst_t s;
    memset(&s, 0, sizeof(s));
    s.type        = DMR_BURST_TYPE_SYNTHETIC_EVT;
    s.timeslot    = (uint8_t)ctx->slot;
    s.synth_event = (uint8_t)event;
    s.synth_pos   = pos;

    if (mq_send(ctx->mq_rx_voice, (const char *)&s, sizeof(s), 0u) < 0) {
        DMR_LOGW("[MAC S%d] mq_rx_voice full — synthetic event %d dropped",
                 ctx->slot, (int)event);
    }
}

/**
 * @brief Enter ACTIVE voice state on receipt of a burst-A anchor or a
 *        Voice LC Header. Arms both timers and resets tracking counters.
 */
static void mac_voice_rx_enter_active(mac_ctx_t *ctx)
{
            DMR_LOGT("[MAC S%d] %d → %s",
                 ctx->slot,
                 ctx->voice_rx.state,
                 "MAC_VOICE_RX_ACTIVE");
    ctx->voice_rx.state        = MAC_VOICE_RX_ACTIVE;
    
    ctx->voice_rx.expected_pos = 1u;   /* next expected: B = position 1 */
    ctx->voice_rx.missed_count = 0u;
    ctx->voice_rx.last_burst_us = dmr_time_now_us();

    /* Arm B-F burst window watchdog — fires if no burst arrives in 80 ms */
    mac_timer_arm_ms(&ctx->tmr_voice_burst, MAC_VOICE_BURST_GUARD_MS);
    /* Arm superframe watchdog — fires if no new burst A in 400 ms */
    mac_timer_arm_ms(&ctx->tmr_voice_sf, MAC_VOICE_SUPERFRAME_GUARD_MS);
}
/**
 * @brief Acknowledge receipt of a B-F voice burst: reset missed counter,
 *        advance expected position, re-arm both timers.
 */
static void mac_voice_rx_got_burst(mac_ctx_t *ctx)
{
    ctx->voice_rx.missed_count = 0u;
    ctx->voice_rx.last_burst_us = dmr_time_now_us();
    ctx->voice_rx.expected_pos++;
    if (ctx->voice_rx.expected_pos > 5u) {
        ctx->voice_rx.expected_pos = 1u;   /* wrap: F(5)→B(1) */
    }
    mac_timer_arm_ms(&ctx->tmr_voice_burst, MAC_VOICE_BURST_GUARD_MS);
    mac_timer_arm_ms(&ctx->tmr_voice_sf,    MAC_VOICE_SUPERFRAME_GUARD_MS);
}

/**
 * @brief Terminate voice RX tracking cleanly: disarm timers, transition
 *        to IDLE, inject a synthetic call-end event.
 *        reason: one of CALL_ENDED_TERMINATOR / CALL_ENDED_CACH_IDLE /
 *                        CALL_ENDED_TIMEOUT / CALL_GONE
 * if terminator is recived then do not pass snythetic burst. 
 */
static void mac_voice_rx_end(mac_ctx_t *ctx, mac_synth_event_t reason)
{
    mac_timer_disarm(&ctx->tmr_voice_burst);
    mac_timer_disarm(&ctx->tmr_voice_sf);
    if(reason!=MAC_SYNTH_EVT_NONE)                                  // if the terminator is recived then we do not need to inject synthetic burst.
    mac_voice_inject_synth(ctx, reason, 0u);
}

/**
 * @brief Handle 80 ms B-F burst window expiry — one burst was missed.
 *
 * Injects a VOICE_BURST_LOST synthetic event so CCL Voice can insert PLC.
 * After MAC_VOICE_MAX_MISSED_BURSTS consecutive misses, transitions to
 * HANGOVER (keeps watching for resumption; superframe watchdog still armed).
 */
static void mac_voice_burst_timer_expired(mac_ctx_t *ctx)
{
    if (ctx->voice_rx.state != MAC_VOICE_RX_ACTIVE) return;

    uint8_t missed_pos = ctx->voice_rx.expected_pos;
    ctx->voice_rx.missed_count++;

    /* Advance position so if the next burst arrives it's placed correctly.
     * Do NOT touch .state here — whether we stay ACTIVE or move to
     * HANGOVER is decided below by the consecutive-miss count, exactly
     * like the equivalent wrap in mac_voice_rx_got_burst(). */
    ctx->voice_rx.expected_pos++;
    if (ctx->voice_rx.expected_pos > 5u) {
        ctx->voice_rx.expected_pos = 1u;   /* wrap: F(5)→B(1) */
    }

    DMR_LOGD("[MAC S%d] Voice burst B-F missed at pos=%u (missed_count=%u)",
             ctx->slot, missed_pos, ctx->voice_rx.missed_count);

    mac_voice_inject_synth(ctx, MAC_SYNTH_EVT_VOICE_BURST_LOST, missed_pos);

    if (ctx->voice_rx.missed_count >= MAC_VOICE_MAX_MISSED_BURSTS) {
        /* Two consecutive misses (120 ms gap) — likely end of superframe.
         * Move to HANGOVER: superframe watchdog stays armed; if a new
         * burst A arrives within 400 ms, resume; otherwise time out. */
        DMR_LOGD("[MAC S%d] %u consecutive missed voice bursts → HANGOVER",
                 ctx->slot, ctx->voice_rx.missed_count);
        mac_timer_disarm(&ctx->tmr_voice_burst);
                    DMR_LOGT("[MAC S%d] %d → %s",
                 ctx->slot,
                 ctx->voice_rx.state,
                 "MAC_VOICE_RX_HANGOVER");
        ctx->voice_rx.state = MAC_VOICE_RX_HANGOVER;
    } else {
        /* Re-arm burst window for the next expected position */
        mac_timer_arm_ms(&ctx->tmr_voice_burst, MAC_VOICE_BURST_GUARD_MS);
    }
}

/**
 * @brief Handle 400 ms superframe watchdog expiry.
 *
 * No new burst A (anchor) arrived within one superframe + guard.
 *   - If ACTIVE: probably missed the terminator; inject CALL_ENDED_TIMEOUT
 *     then enter HANGOVER for one more window to catch any late resumption.
 *   - If HANGOVER: no resumption; definitive end → inject CALL_GONE, go IDLE.
 */
static void mac_voice_sf_timer_expired(mac_ctx_t *ctx)
{
    if (ctx->voice_rx.state == MAC_VOICE_RX_ACTIVE) {
        DMR_LOGD("[MAC S%d] Voice superframe watchdog expired — "
                 "no terminator received; assuming call ended", ctx->slot);
        mac_timer_disarm(&ctx->tmr_voice_burst);
                            DMR_LOGT("[MAC S%d] %d → %s",
                 ctx->slot,
                 ctx->voice_rx.state,
                 "MAC_VOICE_RX_HANGOVER");
        ctx->voice_rx.state = MAC_VOICE_RX_HANGOVER;
        mac_voice_inject_synth(ctx, MAC_SYNTH_EVT_CALL_ENDED_TIMEOUT, 0u);
        /* Stay in HANGOVER for one more watchdog window — re-arm */
        mac_timer_arm_ms(&ctx->tmr_voice_sf, MAC_VOICE_HANGOVER_MS);

    } else if (ctx->voice_rx.state == MAC_VOICE_RX_HANGOVER) {
        DMR_LOGD("[MAC S%d] Voice hangover window expired — "
                 "call definitively ended", ctx->slot);
                                     DMR_LOGT("[MAC S%d] %d → %s",
                 ctx->slot,
                 ctx->voice_rx.state,
                 "MAC_VOICE_RX_IDLE");
        ctx->voice_rx.state = MAC_VOICE_RX_IDLE;
        mac_voice_inject_synth(ctx, MAC_SYNTH_EVT_CALL_GONE, 0u);
    }
}

/**
 * @brief Called by mac_update_slot_activity() (via mac_handle_rx_burst →
 *        CACH decode path) whenever a CACH PDU is decoded. Checks for the
 *        AT Busy→Idle transition that signals BS hang-time has ended.
 *
 * Only meaningful on Tier II / Tier III — skipped entirely on Tier I DMO
 * (no BS, no CACH). The slot parameter is derived from the TC bit of the
 * CACH; we only act if the CACH describes *our* slot.
 */
void mac_handle_cach_at_idle(mac_ctx_t *ctx, uint8_t tc)
{
    if (ctx->tier == DMR_TIER_1_DMO) return;
    if (ctx->voice_rx.state == MAC_VOICE_RX_IDLE) return;

    /* tc==0 → slot 1 described; tc==1 → slot 2 described.
     * Only react to the CACH that describes our own slot. */
    bool our_slot = (tc == 0) ? (ctx->slot == DMR_SLOT_1)
                               : (ctx->slot == DMR_SLOT_2);
    if (!our_slot) return;

    bool prev_busy = (ctx->slot == DMR_SLOT_1)
                         ? ctx->slot_activity.prev_slot1_busy
                         : ctx->slot_activity.prev_slot2_busy;
    bool curr_busy = (ctx->slot == DMR_SLOT_1)
                         ? ctx->slot_activity.slot1_busy
                         : ctx->slot_activity.slot2_busy;

    if (prev_busy && !curr_busy) {
        /* Busy → Idle transition: BS hang-time has ended, channel is free. */
        DMR_LOGD("[MAC S%d] CACH AT Busy→Idle — call ended (hang-time done)",
                 ctx->slot);
        mac_voice_rx_end(ctx, MAC_SYNTH_EVT_CALL_ENDED_CACH_IDLE);
    }
}

/**
 * @brief Classify a decoded RX burst by Data Type (and, for CSBK, by
 *        CSBK Opcode + the tier MAC was configured for) to decide which
 *        CCL-side module's RX queue it belongs on.
 *
 * Voice (no SYNC overwritten by EMB — i.e. genuine voice frame A, or
 * any burst dmr_burst_is_voice() recognises) always goes to CCL Voice,
 * before any Data-Type-based classification, since voice frames B-F
 * carry EMB where SYNC would be and are NOT data bursts in the
 * llc_rx_dispatch() sense (see the EMB/sync design notes in
 * dmr_ccl_voice.c) — they would otherwise fall through to "unknown".
 *
 * Data Type → module mapping, ETSI TS 102 361-1 Table 9.2 / Clause 9.1.1:
 *   VOICE_LC_HEADER, TERMINATOR_LC          → CCL Voice  (Cl.9.1.5/9.1.6)
 *   DATA_HEADER, RATE1_DATA,
 *   RATE12_DATA, RATE34_DATA                 → CCL Data   (Cl.8.2, 9.1.8/9)
 *   MBC_HEADER, MBC_CONT                      → Trunking   (Tier III
 *                                               multi-block CSBK carrier
 *                                               for longer messages, e.g.
 *                                               NET_STATUS/ADJ_SITE,
 *                                               TS 102 361-1 Cl.7.4 +
 *                                               TS 102 361-4 Cl.6 — not
 *                                               used by CCL Voice/Data)
 *   CSBK                                       → opcode + tier dependent,
 *                                               see the switch below
 *   PI_HEADER, IDLE, UNKNOWN                   → dropped; not consumed
 *                                               by any module today
 *
 * CSBK routing is tier-aware because some CSBK Opcode values are
 * reused with different meanings between Tier II and Tier III (e.g.
 * 0x24, 0x27 — see TS 102 361-1 Cl.7.2: opcode meaning is qualified by
 * the CSBKO+FID combination, but every CSBK builder in this codebase
 * currently emits FID=0x00 regardless of tier, so opcode alone is
 * ambiguous across tiers). Since a given MS instance runs exactly one
 * tier (dmr_ms.c composes only the modules that tier needs), routing
 * by (opcode, ctx->tier) is unambiguous in practice: a Tier II MS
 * never has a Trunking module to misroute a Tier III-only opcode to,
 * and vice versa.
 */
static mac_rx_dest_t mac_classify_rx_burst(mac_ctx_t *ctx,
                                             const dmr_burst_t *burst)
{
    /* ---------------------------------------------------------------
     * Burst A: genuine SYNC pattern — unambiguous voice frame anchor.
     * Enter/re-enter ACTIVE regardless of current voice_rx state so
     * late entry and resumption after HANGOVER both work correctly.
     * --------------------------------------------------------------- */

    if (dmr_burst_is_voice(burst->raw)) {
        /* Re-arm on every burst A, whether this is a fresh call, a late
         * entry, a resumption after HANGOVER, or just the next superframe
         * of a call already ACTIVE — mac_voice_rx_enter_active() resetting
         * expected_pos/missed_count and re-arming both watchdogs is correct
         * in all four cases. */
        mac_voice_rx_enter_active(ctx);
        return MAC_RX_DEST_VOICE;
    }

    llc_rx_result_t res;
    llc_rx_dispatch(burst, &res); /* always returns DMR_OK; check res.type */
    /**this condition resets the rx state to idle if it was in late entry and never got reset again**/
    if(res.type!=LLC_RX_UNKNOWN && ctx->voice_rx.state != MAC_VOICE_RX_IDLE)
    {
                DMR_LOGT("[MAC S%d] %d → %s",
                 ctx->slot,
                 ctx->voice_rx.state,
                 "MAC_VOICE_RX_IDLE");
        ctx->voice_rx.state = MAC_VOICE_RX_IDLE;

        
    }

    switch (res.type) {
    case LLC_RX_VOICE_LC_HDR:
        mac_voice_rx_enter_active(ctx);
        return MAC_RX_DEST_VOICE;
    /* -------------------------------------------------------------------
     * Terminator with LC: explicit, clean end of call.
     * Disarm timers, inject CALL_ENDED_TERMINATOR, go IDLE — then also
     * deliver the real Terminator burst to CCL Voice so it can read the
     * LC teardown information before processing the synthetic event.
     * ------------------------------------------------------------------- */
    case LLC_RX_TERMINATOR_LC:

       

    if(res.opcode!=0x30) //check if terminator is for Data
    {
        mac_voice_rx_end(ctx, MAC_SYNTH_EVT_NONE); //disarm timers and send pacet to ccl_voice.
        DMR_LOGD("[MAC S%d] Terminator for voice", ctx->slot);
        return MAC_RX_DEST_VOICE;
    }
    else{
         DMR_LOGD("[MAC S%d] Terminator for data", ctx->slot);
         return MAC_RX_DEST_DATA;
    }


    case LLC_RX_DATA_HEADER:
    case LLC_RX_DATA_BLOCK:
        return MAC_RX_DEST_DATA;

    case LLC_RX_MBC_HEADER:
    case LLC_RX_MBC_CONT:
        return (ctx->tier == DMR_TIER_3_TRUNKED) ? MAC_RX_DEST_TRUNK
                                                   : MAC_RX_DEST_DROP;

    case LLC_RX_CSBK:
        switch (res.opcode) {
        /* Tier-agnostic / Tier II conventional CSBKs consumed by CCL
         * Voice — these opcode values are NOT reused by any Tier III
         * meaning in this codebase, so they route the same regardless
         * of ctx->tier (harmless on a Tier III MS that also happens
         * to run CCL Voice for non-trunked individual calls). */
        case DMR_CSBKO_UU_V_REQ:
        case DMR_CSBKO_CALL_ALERT:
        case DMR_CSBKO_ACK_RSP:
        case DMR_CSBKO_CANCEL_CALL:
            return MAC_RX_DEST_VOICE;

        /* Channel Timing CSBK (CT_CSBK) — Tier II DCDM wide area timing
         * leader election (MOD-15, TS 102 361-2 Cl.6.2). Always routed
         * to mq_rx_dcdm regardless of tier; on a Tier I/III MS with no
         * DCDM module running, nothing ever opens/reads that queue and
         * messages simply age out — same harmless-when-unused pattern
         * already used for the Trunking queues on Tier I/II. */
        case DMR_CSBKO_CHANNEL_TIMING:
            return MAC_RX_DEST_DCDM;

        /* Preamble announces an upcoming CCL Data transfer (MOD-06). */
        case DMR_CSBKO_PREAMBLE:
            return MAC_RX_DEST_DATA;

        /* Tier III Random Access is TSCC→... no — C_RAND is MS→TSCC
         * (inbound only at a real TSCC, not relevant to MS-side RX);
         * grants are TSCC→MS and are the core Trunking RX path. */
        case DMR_CSBKO_T3_TV_GRANT:
        case DMR_CSBKO_T3_TD_GRANT:
        case DMR_CSBKO_T3_NET_STATUS:
        case DMR_CSBKO_T3_ADJ_SITE:
        case DMR_CSBKO_T3_MS_REGIST_RSP:
            return (ctx->tier == DMR_TIER_3_TRUNKED) ? MAC_RX_DEST_TRUNK
                                                       : MAC_RX_DEST_DROP;

        /* Opcode values reused between Tier II and Tier III — see the
         * function doc comment above. Resolved by tier: */
        case DMR_CSBKO_UU_ANS_RSP:       /* 0x24, Tier II CCL Voice    */
            /* NOTE: numerically identical to DMR_CSBKO_T3_MS_REGIST.
             * On a Tier III MS this opcode is presumed to mean MS
             * Registration (no registration consumer exists yet in
             * any module — dropped, not misrouted to CCL Voice). */
            return (ctx->tier == DMR_TIER_3_TRUNKED) ? MAC_RX_DEST_DROP
                                                        : MAC_RX_DEST_VOICE;

        case DMR_CSBKO_EMERG_ALARM_ACK:  /* 0x27, Tier II CCL Voice    */
            /* NOTE: numerically identical to DMR_CSBKO_T3_MS_DEREGIST.
             * Same resolution rationale as above. */
            return (ctx->tier == DMR_TIER_3_TRUNKED) ? MAC_RX_DEST_DROP
                                                        : MAC_RX_DEST_VOICE;

        case DMR_CSBKO_BS_DWNA:          /* 0x28, Tier II CCL Voice    */
            /* NOTE: numerically identical to DMR_CSBKO_T3_EMERG_ALARM.
             * No CCL module consumes a Tier III emergency alarm CSBK
             * yet (CCL Voice's own emergency path uses Full LC, not
             * this CSBK opcode) — dropped on a Tier III MS rather than
             * guessed, consistent with the other collisions above. */
            return (ctx->tier == DMR_TIER_3_TRUNKED) ? MAC_RX_DEST_DROP
                                                        : MAC_RX_DEST_VOICE;

        default:
            DMR_LOGT("[MAC S%d] Unclassified CSBK opcode=0x%02X tier=%s — dropping",
                     ctx->slot, res.opcode, dmr_tier_name(ctx->tier));
            return MAC_RX_DEST_DROP;
        }

    /* -------------------------------------------------------------------
     * LLC_RX_UNKNOWN: burst could not be FEC-decoded as a data type.
     * If a voice call is ACTIVE and this burst arrived within the 80 ms
     * B-F window, infer it is a voice burst B-F (EMB replaces SYNC so
     * it has neither a SYNC nor a Data Type field — it is not classifiable
     * by content alone; only temporal context tells us what it is).
     * ------------------------------------------------------------------- */
    case LLC_RX_UNKNOWN: {
        bool in_window = false;
        if (ctx->voice_rx.state == MAC_VOICE_RX_ACTIVE) {
            uint64_t now     = dmr_time_now_us();
            uint64_t elapsed = now - ctx->voice_rx.last_burst_us;
            /* 80 000 µs = MAC_VOICE_BURST_GUARD_MS */
            in_window = (elapsed <= (uint64_t)MAC_VOICE_BURST_GUARD_MS * 1000u);
        } else if (ctx->voice_rx.state == MAC_VOICE_RX_HANGOVER) {
            /* A burst arrived during hangover — treat as resumption of
             * the superframe; re-enter ACTIVE and count this as burst B
             * (we may have missed A; CCL Voice will handle the gap). */
            mac_voice_rx_enter_active(ctx);
            in_window = true;
        }

        if (in_window) {
            mac_voice_rx_got_burst(ctx);
            return MAC_RX_DEST_VOICE;
        }
        return MAC_RX_DEST_DROP;
    }

  
    case LLC_RX_IDLE:
        /* TODO: not yet handled by MAC. Routine/expected burst type —
         * trace level only, not ERROR, to avoid log flooding. */
        DMR_LOGT("[MAC S%d] Idle burst — drop (not yet handled by MAC)",
                 ctx->slot);
        return MAC_RX_DEST_DROP;

    case LLC_RX_PI_HEADER:
    default:
        return MAC_RX_DEST_DROP;
    }
}

/* =========================================================================
 * Process an incoming RX burst from PHY — delivered to CCL if CC matches
 * ========================================================================= */

static void mac_handle_rx_burst(mac_ctx_t *ctx, dmr_burst_t *burst)
{
    ctx->rx_burst_count++;

    /* Colour code check for data bursts. Exception (Cl.6.2.2.2): the
     * "All Site" colour code (0xF) is always a qualified colour code
     * on a TDMA direct mode channel — an MS decodes a Channel Timing
     * CSBK with CC=0xF regardless of its own configured colour code,
     * on either its provisioned or non-provisioned slot. Since only
     * CT_CSBK builders ever emit CC=0xF, this bypass cannot misroute
     * any other burst type. */
    if (dmr_burst_is_data(burst->raw)) {
        uint8_t cc = dmr_burst_get_cc(burst->raw);
        if (cc != ctx->colour_code && cc != DMR_CC_ALL_SITE) {
            DMR_LOGT("[MAC S%d] CC mismatch rx=%u cfg=%u — discarding",
                     ctx->slot, cc, ctx->colour_code);
            return;
        }

        /* If we're qualifying idle and a data burst arrives → channel busy */
        if (ctx->ch_state == MAC_STATE_QUALIFY_IDLE) {
            DMR_LOGD("[MAC S%d] Data burst during LBT → channel busy", ctx->slot);
            mac_lbt_channel_busy(ctx);
        }
    }

    /* Classify once, then forward the ORIGINAL raw burst unchanged to
     * exactly one destination queue — see mac_classify_rx_burst() and
     * the queue-ownership rationale in dmr_mac.h. Each CCL-side module
     * still runs its own llc_rx_dispatch()/FEC decode on whatever it
     * receives, exactly as before this classification was added; only
     * the queue a burst lands on has changed. */
    mac_rx_dest_t dest = mac_classify_rx_burst(ctx, burst);
    mqd_t dest_mq;
    const char *dest_name;

    switch (dest) {
    case MAC_RX_DEST_VOICE: dest_mq = ctx->mq_rx_voice; dest_name = "voice"; break;
    case MAC_RX_DEST_DATA:  dest_mq = ctx->mq_rx_data;  dest_name = "data";  break;
    case MAC_RX_DEST_TRUNK: dest_mq = ctx->mq_rx_trunk; dest_name = "trunk"; break;
    case MAC_RX_DEST_DCDM: dest_mq = ctx->mq_rx_dcdm; dest_name = "dcdm"; break;
    case MAC_RX_DEST_DROP:
    default:
        DMR_LOGT("[MAC S%d] RX burst dtype=0x%02X classified as DROP",
                 ctx->slot, dmr_burst_get_dtype(burst->raw));
        return;
    }

    if (mq_send(dest_mq, (const char *)burst, sizeof(*burst), 0u) < 0) {
        DMR_LOGW("[MAC S%d] mq_rx_%s full — burst dropped (dtype=0x%02X)",
                 ctx->slot, dest_name, dmr_burst_get_dtype(burst->raw));
    }
}

/* =========================================================================
 * MAC worker thread — epoll event loop
 * ========================================================================= */

#define MAX_EPOLL_EVENTS 8

void *mac_thread(void *arg)
{
    mac_ctx_t *ctx = (mac_ctx_t *)arg;
    struct epoll_event events[MAX_EPOLL_EVENTS];

    DMR_LOGI("[MAC S%d] Worker thread started", ctx->slot);

    while (ctx->running) {
        int nev = epoll_wait(ctx->epoll_fd, events, MAX_EPOLL_EVENTS, 100);

        if (nev < 0) {
            if (errno == EINTR) continue;
            DMR_LOGE("[MAC S%d] epoll_wait error: %s", ctx->slot, strerror(errno));
            break;
        }

        for (int i = 0; i < nev; i++) {
            int fd = events[i].data.fd;

            /* ----------------------------------------------------------
             * T_IdleSrch expired → channel qualified idle
             * ---------------------------------------------------------- */
            if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_idle_srch)) {
                mac_timer_drain(&ctx->tmr_idle_srch);
                if (ctx->ch_state == MAC_STATE_QUALIFY_IDLE) {
                    mac_idle_srch_expired(ctx);
                }
            }

            /* ----------------------------------------------------------
             * T_Holdoff expired → retry LBT
             * ---------------------------------------------------------- */
            else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_holdoff)) {
                mac_timer_drain(&ctx->tmr_holdoff);
                if (ctx->ch_state == MAC_STATE_HOLDOFF) {
                    mac_holdoff_expired(ctx);
                }
            }

            /* ----------------------------------------------------------
             * T_DataTxLmt expired → abort pending TX
             * ---------------------------------------------------------- */
            else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_tx_lmt)) {
                mac_timer_drain(&ctx->tmr_tx_lmt);
                if (ctx->has_pending) {
                    mac_tx_lmt_expired(ctx);
                }
            }
                        /* ----------------------------------------------------------
             * tfd_voice_burst expired → voice burst missed
             * ---------------------------------------------------------- */
            else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_voice_burst)) {
                mac_timer_drain(&ctx->tmr_voice_burst);
               /* if (ctx->has_pending)*/ {
                    mac_voice_burst_timer_expired(ctx);
                }
            }
                        /* ----------------------------------------------------------
             * tfd_voice_sf expired → superframe missed
             * ---------------------------------------------------------- */
            else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_voice_sf)) {
                mac_timer_drain(&ctx->tmr_voice_sf);
                /*if (ctx->has_pending)*/ {
                    mac_voice_sf_timer_expired(ctx);
                }
            }

            /* ----------------------------------------------------------
             * TX request from CCL
             * ---------------------------------------------------------- */
            else if (fd == (int)ctx->mq_tx_req) {
                dmr_mac_tx_req_t req;
                ssize_t n = mq_receive(ctx->mq_tx_req,
                                       (char *)&req, sizeof(req), NULL);
                if (n > 0) {
                    mac_handle_tx_req(ctx, &req);
                }
            }
            


            /* ----------------------------------------------------------
             * RX burst from PHY
             * ---------------------------------------------------------- */
            else if (fd == (int)ctx->mq_phy_rx) {
                dmr_burst_t burst;
                ssize_t n = mq_receive(ctx->mq_phy_rx,
                                       (char *)&burst, sizeof(burst), NULL);
                if (n > 0) {
                    mac_handle_rx_burst(ctx, &burst);
                }
            }
                                    /* ----------------------------------------------------------
             * TX conf from phy
             * ---------------------------------------------------------- */
            else if (fd == (int)ctx->mq_phy_tx_conf) {
                 dmr_phy_tx_conf_t resp;
                ssize_t n = mq_receive(ctx->mq_phy_tx_conf,
                                       (char *)&resp, sizeof(resp), NULL);
                if (n > 0) {
                    post_conf_from_phy(ctx,&resp);
                }
            }
        }
    }

    DMR_LOGI("[MAC S%d] Worker thread exiting", ctx->slot);
    return NULL;
}

/* =========================================================================
 * Initialisation / teardown
 * ========================================================================= */

static mqd_t open_mq(const char *name, int flags, size_t msg_size)
{
    struct mq_attr attr;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = DMR_MQ_MAX_MSGS;
    attr.mq_msgsize = (long)msg_size;
    attr.mq_curmsgs = 0;

    mqd_t mq = mq_open(name, flags | O_CREAT | O_NONBLOCK,
                        0600, &attr);
    if (mq == (mqd_t)-1) {
        DMR_LOGE("mq_open(%s) failed: %s", name, strerror(errno));
    }
    return mq;
}

static int add_to_epoll(int epfd, int fd)
{
    struct epoll_event ev;
    /* Level-triggered: this loop does a single mq_receive()/read() per fd
     * per epoll_wait() wakeup, not a drain-to-EAGAIN loop. Under EPOLLET a
     * second message queued before that single receive would not generate
     * a fresh edge and could sit unprocessed. Level-triggered re-notifies
     * every wakeup as long as data remains, which matches this loop. */
    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    return epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
}

dmr_err_t mac_init(mac_ctx_t *ctx,
                   dmr_slot_t slot,
                   uint8_t    colour_code,
                   uint32_t   radio_id,
                   dmr_tier_t tier)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->slot         = slot;
    ctx->colour_code  = colour_code;
    ctx->tier         = tier;
    ctx->ch_state     = MAC_STATE_IDLE_MONITOR;
    ctx->voice_rx.state =MAC_VOICE_RX_IDLE;
    ctx->running      = false;
    DMR_SET_ID(ctx->my_id, radio_id);

    pthread_mutex_init(&ctx->state_mutex, NULL);

    /* Seed this slot's own random holdoff generator (not the process-global
     * rand()/srand(), which two slot threads would otherwise share/race on) */
    ctx->rng_state = (unsigned)time(NULL) ^ (unsigned)slot
                     ^ (unsigned)(uintptr_t)ctx;

    /* ---- POSIX message queues ---- */
    const char *phy_rx_name   = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_PHY_RX_S1      : DMR_MQ_PHY_RX_S2;
    const char *tx_req_name   = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_REQ_S1  : DMR_MQ_MAC_TX_REQ_S2;
    const char *tx_conf_voice_name  = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_CONF_VOICE_S1 : DMR_MQ_MAC_TX_CONF_VOICE_S2;
    const char *tx_conf_data_name  = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_CONF_DATA_S1 : DMR_MQ_MAC_TX_CONF_DATA_S2;
    const char *tx_conf_trunk_name  = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_CONF_TRUNK_S1 : DMR_MQ_MAC_TX_CONF_TRUNK_S2;
    const char *rx_voice_name = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_RX_VOICE_S1: DMR_MQ_MAC_RX_VOICE_S2;
    const char *rx_data_name  = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_RX_DATA_S1 : DMR_MQ_MAC_RX_DATA_S2;
    const char *rx_trunk_name = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_RX_TRUNK_S1: DMR_MQ_MAC_RX_TRUNK_S2;
    const char *tx_conf_dcdm_name  = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_CONF_DCDM_S1 : DMR_MQ_MAC_TX_CONF_DCDM_S2;
    const char *rx_dcdm_name  = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_RX_DCDM_S1 : DMR_MQ_MAC_RX_DCDM_S2;
    const char *phy_tx_name   = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_PHY_TX_S1      : DMR_MQ_PHY_TX_S2;
    const char *phy_tx_conf_name   = (slot == DMR_SLOT_1)
                                    ? DMR_MQ_PHY_TX_CONF_S1      : DMR_MQ_PHY_TX_CONF_S2;
                                    



    ctx->mq_phy_rx    = open_mq(phy_rx_name,   O_RDWR, sizeof(dmr_burst_t));
    ctx->mq_tx_req    = open_mq(tx_req_name,   O_RDONLY, sizeof(dmr_mac_tx_req_t));
    ctx->mq_tx_voice_conf   = open_mq(tx_conf_voice_name,  O_WRONLY, sizeof(dmr_mac_tx_conf_t));
    ctx->mq_tx_data_conf   = open_mq(tx_conf_data_name,  O_WRONLY, sizeof(dmr_mac_tx_conf_t));
    ctx->mq_tx_trunk_conf   = open_mq(tx_conf_trunk_name,  O_WRONLY, sizeof(dmr_mac_tx_conf_t));
    ctx->mq_rx_voice  = open_mq(rx_voice_name, O_WRONLY, sizeof(dmr_burst_t));
    ctx->mq_rx_data   = open_mq(rx_data_name,  O_WRONLY, sizeof(dmr_burst_t));
    ctx->mq_rx_trunk  = open_mq(rx_trunk_name, O_WRONLY, sizeof(dmr_burst_t));
    ctx->mq_tx_dcdm_conf    = open_mq(tx_conf_dcdm_name,  O_WRONLY, sizeof(dmr_mac_tx_conf_t));
    ctx->mq_rx_dcdm   = open_mq(rx_dcdm_name,  O_WRONLY, sizeof(dmr_burst_t));
    ctx->mq_phy_tx    = open_mq(phy_tx_name,   O_RDWR, sizeof(dmr_mac_tx_req_t));
    ctx->mq_phy_tx_conf    = open_mq(phy_tx_conf_name,O_RDWR, sizeof(dmr_phy_tx_conf_t));
    
 //printf(" \nctx->mq_phy_tx %d \n",ctx->mq_phy_tx);
    if (ctx->mq_phy_rx   == (mqd_t)-1 ||
        ctx->mq_tx_req   == (mqd_t)-1 ||
        ctx->mq_tx_voice_conf  == (mqd_t)-1 ||
        ctx->mq_tx_data_conf  == (mqd_t)-1 ||
        ctx->mq_tx_trunk_conf  == (mqd_t)-1 ||
        ctx->mq_rx_voice == (mqd_t)-1 ||
        ctx->mq_rx_data  == (mqd_t)-1 ||
        ctx->mq_rx_trunk == (mqd_t)-1 ||
        ctx->mq_tx_dcdm_conf == (mqd_t)-1 ||
        ctx->mq_rx_dcdm  == (mqd_t)-1 ||
         ctx->mq_phy_tx_conf == (mqd_t)-1 ||
        ctx->mq_phy_tx   == (mqd_t)-1) {
        DMR_LOGE("[MAC S%d] Failed to open one or more message queues", slot);
        return DMR_ERR_NO_MEM;
    }

         /* ---- PHY timers (via dmr_phy_timer_oneshot — backend-agnostic) ---- */
    if (dmr_phy_timer_oneshot_init(&ctx->tmr_idle_srch)   != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_holdoff)     != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_tx_lmt)      != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_voice_burst) != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_voice_sf)    != DMR_OK) {
        DMR_LOGE("[MAC S%d] dmr_phy_timer_oneshot_init failed: %s",
                 slot, strerror(errno));
        return DMR_ERR_NO_MEM;
    }

    /* ---- epoll ---- */
    ctx->epoll_fd = epoll_create1(EPOLL_CLOEXEC);
    if (ctx->epoll_fd < 0) {
        DMR_LOGE("[MAC S%d] epoll_create1 failed: %s", slot, strerror(errno));
        return DMR_ERR_NO_MEM;
    }

    /* Register all event sources */
   add_to_epoll(ctx->epoll_fd, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_idle_srch));
    add_to_epoll(ctx->epoll_fd, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_holdoff));
    add_to_epoll(ctx->epoll_fd, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_tx_lmt));
    add_to_epoll(ctx->epoll_fd, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_voice_burst));
    add_to_epoll(ctx->epoll_fd, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_voice_sf));
    add_to_epoll(ctx->epoll_fd, (int)ctx->mq_tx_req);
    add_to_epoll(ctx->epoll_fd, (int)ctx->mq_phy_tx_conf);
    add_to_epoll(ctx->epoll_fd, (int)ctx->mq_phy_rx);

    DMR_LOGI("[MAC S%d] Initialised (CC=%u, RadioID=0x%06X, Tier=%s)",
             slot, colour_code, radio_id, dmr_tier_name(tier));
    return DMR_OK;
}

dmr_err_t mac_start(mac_ctx_t *ctx)
{
    ctx->running = true;
    if (pthread_create(&ctx->thread, NULL, mac_thread, ctx) != 0) {
        ctx->running = false;
        DMR_LOGE("[MAC S%d] pthread_create failed: %s", ctx->slot, strerror(errno));
        return DMR_ERR_NO_MEM;
    }
    DMR_LOGI("[MAC S%d] Worker thread started", ctx->slot);
    return DMR_OK;
}

dmr_err_t mac_stop(mac_ctx_t *ctx)
{
    if(ctx->running)
    {
        ctx->running = false;
        /* Wake up epoll_wait by writing a 1-shot timerfd arm of 1 ns */
        mac_timer_arm_ms(&ctx->tmr_idle_srch, 1u);
        pthread_join(ctx->thread, NULL);
        DMR_LOGI("[MAC S%d] Worker thread stopped", ctx->slot);
    }
    return DMR_OK;
}

void mac_destroy(mac_ctx_t *ctx)
{
    /* Close timerfd descriptors */
    if (ctx->tmr_idle_srch.fd   >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_idle_srch);
    if (ctx->tmr_holdoff.fd     >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_holdoff);
    if (ctx->tmr_tx_lmt.fd      >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_tx_lmt);
    if (ctx->tmr_voice_burst.fd >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_voice_burst);
    if (ctx->tmr_voice_sf.fd    >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_voice_sf);
    if (ctx->epoll_fd        >= 0) close(ctx->epoll_fd);


    /* Close message queues */
    if (ctx->mq_phy_rx   != (mqd_t)-1) mq_close(ctx->mq_phy_rx);
    if (ctx->mq_tx_req   != (mqd_t)-1) mq_close(ctx->mq_tx_req);
    if (ctx->mq_tx_voice_conf  != (mqd_t)-1) mq_close(ctx->mq_tx_voice_conf);
    if (ctx->mq_tx_data_conf  != (mqd_t)-1) mq_close(ctx->mq_tx_data_conf);
    if (ctx->mq_tx_trunk_conf  != (mqd_t)-1) mq_close(ctx->mq_tx_trunk_conf);
    if (ctx->mq_rx_voice != (mqd_t)-1) mq_close(ctx->mq_rx_voice);
    if (ctx->mq_rx_data  != (mqd_t)-1) mq_close(ctx->mq_rx_data);
    if (ctx->mq_rx_trunk != (mqd_t)-1) mq_close(ctx->mq_rx_trunk);
    if (ctx->mq_tx_dcdm_conf != (mqd_t)-1) mq_close(ctx->mq_tx_dcdm_conf);
    if (ctx->mq_rx_dcdm  != (mqd_t)-1) mq_close(ctx->mq_rx_dcdm);
    if (ctx->mq_phy_tx_conf   != (mqd_t)-1) mq_close(ctx->mq_phy_tx_conf);
    if (ctx->mq_phy_tx   != (mqd_t)-1) mq_close(ctx->mq_phy_tx);
    
   

    /* MAC is the sole creator (O_CREAT) of these queue names — see the
     * ownership contract in dmr_mac.h. As sole creator, MAC is also
     * responsible for unlinking them so a stale queue (with a stale
     * mq_attr from a previous run) cannot be silently reused by the
     * next mac_init() after a crash/restart. CCL-side modules must
     * never unlink these names themselves. */
    const char *tx_req_name   = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_REQ_S1   : DMR_MQ_MAC_TX_REQ_S2;
    const char *tx_conf_voice_name  = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_CONF_VOICE_S1 : DMR_MQ_MAC_TX_CONF_VOICE_S2;
    const char *tx_conf_data_name  = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_CONF_DATA_S1 : DMR_MQ_MAC_TX_CONF_DATA_S2;
    const char *tx_conf_trunk_name  = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_CONF_TRUNK_S1 : DMR_MQ_MAC_TX_CONF_TRUNK_S2;
    const char *rx_voice_name = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_RX_VOICE_S1 : DMR_MQ_MAC_RX_VOICE_S2;
    const char *rx_data_name  = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_RX_DATA_S1  : DMR_MQ_MAC_RX_DATA_S2;
    const char *rx_trunk_name = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_RX_TRUNK_S1 : DMR_MQ_MAC_RX_TRUNK_S2;
    const char *tx_conf_dcdm_name  = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_TX_CONF_DCDM_S1 : DMR_MQ_MAC_TX_CONF_DCDM_S2;
    const char *rx_dcdm_name  = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_MAC_RX_DCDM_S1  : DMR_MQ_MAC_RX_DCDM_S2;
    const char *phy_rx_name   = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_PHY_RX_S1       : DMR_MQ_PHY_RX_S2;
    const char *phy_tx_name   = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_PHY_TX_S1       : DMR_MQ_PHY_TX_S2;
    const char *phy_tx_conf_name   = (ctx->slot == DMR_SLOT_1)
                                    ? DMR_MQ_PHY_TX_CONF_S1       : DMR_MQ_PHY_TX_CONF_S2;
    mq_unlink(tx_req_name);
    mq_unlink(tx_conf_voice_name);
    mq_unlink(tx_conf_data_name);
    mq_unlink(tx_conf_trunk_name);
    mq_unlink(rx_voice_name);
    mq_unlink(rx_data_name);
    mq_unlink(rx_trunk_name);
    mq_unlink(tx_conf_dcdm_name);
    mq_unlink(rx_dcdm_name);
    mq_unlink(phy_rx_name);
    mq_unlink(phy_tx_name);
    mq_unlink(phy_tx_conf_name);

    pthread_mutex_destroy(&ctx->state_mutex);

    DMR_LOGI("[MAC S%d] Destroyed (tx=%lu rx=%lu holdoffs=%lu aborts=%lu)",
             ctx->slot,
             ctx->tx_burst_count, ctx->rx_burst_count,
             ctx->lbt_holdoff_count, ctx->tx_abort_count);

    memset(ctx, 0, sizeof(*ctx));
}


dmr_err_t dmr_mac_inject_rx_burst(mac_ctx_t *ctx, const dmr_burst_t *burst)
{
    mqd_t target_q =  ctx->mq_phy_rx;
    if (target_q == (mqd_t)-1 || !burst) return DMR_ERR_INVALID_PARAM;

    if (mq_send(target_q, (const char *)burst, sizeof(dmr_burst_t), 0) < 0) {
        perror("DMR TX Queue Send Failed");
    
    // Method 2: Programmatically triage or log specific failure states
    switch (errno) {
        case EAGAIN:
            // The queue is completely full and O_NONBLOCK was set on the descriptor
            printf("Queue is full! Dropping packet or retrying later.\n");
            break;
            
        case EMSGSIZE:
            // CRITICAL: Your packing function output size is bigger than 
            // the 'mq_msgsize' specified when mq_open was called!
            printf("Coding error: msg_len exceeds the maximum message size allowed by this queue.\n");
            break;
            
        case EBADF:
            // The queue descriptor is corrupted, closed, or invalid
            printf("Invalid queue descriptor handler.\n");
            break;
            
        case EINTR:
            // The call was interrupted mid-execution by a system signal
            printf("Interrupted by signal. Safe to retry call.\n");
            break;
            
        default:
            // Fallback for any unmapped OS-specific codes
            printf("Unexpected error code: %d (%s)\n", errno, strerror(errno));
            break;
    }
        return DMR_ERR_QUEUE_FULL;
    }
    return DMR_OK;
}