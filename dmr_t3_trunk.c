/**
 * @file dmr_t3_trunk.c
 * @brief MOD-07 — Tier III Trunking: Random Access & Channel Grant Flow
 *
 * See dmr_t3_trunk.h for module overview and scope.
 */

#define _POSIX_C_SOURCE 200809L

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>

#include "dmr_t3_trunk.h"
#include "dmr_llc.h"

/* =========================================================================
 * Local constants
 * ========================================================================= */
#define T3_TRUNK_MQ_BURST_MSG_SIZE   sizeof(dmr_burst_t)
#define T3_TRUNK_MQ_TX_REQ_MSG_SIZE  sizeof(dmr_mac_tx_req_t)
#define T3_TRUNK_MQ_TX_CONF_MSG_SIZE sizeof(dmr_mac_tx_conf_t)

/* Depth for mq_evt ONLY — the one queue this module creates/owns itself.
 * It must NOT be used for mq_mac_tx/conf/rx — those are MAC-owned (see
 * ownership contract in dmr_mac.h) and their depth is fixed by
 * DMR_MQ_MAX_MSGS, decided once by MAC at creation time.
 *
 * NOTE: kept at 10 (not higher) because some environments cap
 * mq_open()'s mq_maxmsg via /proc/sys/fs/mqueue/msg_max for
 * unprivileged processes (commonly 10); exceeding it makes mq_open()
 * fail with EINVAL. CCL Voice/CCL Data use the same ceiling for their
 * own private event queues for the same reason. */
#define T3_TRUNK_MQ_EVT_MAX_MSGS     10
#define T3_TRUNK_MQ_EVT_MSG_SIZE     sizeof(t3_trunk_event_t)

#define T3_TRUNK_EPOLL_MAX_EVENTS    8

/* =========================================================================
 * Small helpers
 * ========================================================================= */
static void t3_trunk_arm_timer(dmr_phy_timer_oneshot_t *t, uint32_t ms)
{
    dmr_phy_timer_oneshot_arm_ms(t, ms);
}

/* Disarm a one-shot PHY timer */
static void t3_trunk_disarm_timer(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_disarm(t);
}

/* Drain a one-shot PHY timer after expiry */
static void t3_trunk_timer_drain(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_drain(t);
}




/* For mq_evt — the one queue this module itself creates and owns.
 * Safe to use O_CREAT here because nothing else in the system ever
 * opens this name; there is no second-creator ambiguity. */
static mqd_t t3_trunk_mq_create_own(const char *name, int max_msgs,
                                      size_t msg_size)
{
    struct mq_attr attr;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = max_msgs;
    attr.mq_msgsize = (long)msg_size;
    attr.mq_curmsgs = 0;

    mqd_t mq = mq_open(name, O_CREAT | O_NONBLOCK | O_RDWR, 0600, &attr);
    if (mq == (mqd_t)-1) {
        DMR_LOGE("mq_open(%s) failed: %s", name, strerror(errno));
    }
    return mq;
}

/* Opens all three MAC-owned queues (mq_mac_tx/conf/rx) together as one
 * unit, retrying the WHOLE set with a single shared bounded backoff —
 * not each queue independently. All three are created atomically by
 * one mac_init() call, so if one is missing the others necessarily are
 * too; retrying them independently would each pay their own full
 * retry budget serially (3x the wait for no benefit). On success all
 * three mqd_t are filled in; on failure all are left as (mqd_t)-1 and
 * any partially-opened handles are closed before returning. */
static dmr_err_t t3_trunk_open_mac_queues(const char *tx_name,
                                            const char *conf_name,
                                            const char *rx_name,
                                            mqd_t *out_tx,
                                            mqd_t *out_conf,
                                            mqd_t *out_rx)
{
    int attempts = 0;

    do {
        mqd_t tx   = mq_open(tx_name,   O_NONBLOCK | O_WRONLY);
        mqd_t conf = mq_open(conf_name, O_NONBLOCK | O_RDONLY);
        mqd_t rx   = mq_open(rx_name,   O_NONBLOCK | O_RDONLY);

        if (tx != (mqd_t)-1 && conf != (mqd_t)-1 && rx != (mqd_t)-1) {
            *out_tx = tx;
            *out_conf = conf;
            *out_rx = rx;
            return DMR_OK;
        }

        /* Close whichever of the three did succeed before retrying —
         * we retry the set atomically, not partially. */
        if (tx   != (mqd_t)-1) mq_close(tx);
        if (conf != (mqd_t)-1) mq_close(conf);
        if (rx   != (mqd_t)-1) mq_close(rx);

        attempts++;
        if (attempts < DMR_MQ_OPEN_RETRY_COUNT) {
            struct timespec ts;
            ts.tv_sec  = 0;
            ts.tv_nsec = (long)DMR_MQ_OPEN_RETRY_DELAY_MS * 1000000L;
            nanosleep(&ts, NULL);
        }
    } while (attempts < DMR_MQ_OPEN_RETRY_COUNT);

    DMR_LOGE("mq_open(%s, %s, %s) failed after %d retries: %s — was "
             "mac_init() called for this slot before t3_trunk_init()?",
             tx_name, conf_name, rx_name, attempts, strerror(errno));
    *out_tx = (mqd_t)-1;
    *out_conf = (mqd_t)-1;
    *out_rx = (mqd_t)-1;
    return DMR_ERR_QUEUE_FULL;
}

/* xorshift32 — small, fast, deterministic PRNG for random backoff.
 * Not cryptographic; suitable for channel-access jitter only. */
static uint32_t t3_trunk_rand_next(t3_trunk_ctx_t *ctx)
{
    uint32_t x = ctx->rand_state;
    if (x == 0u) x = 0xA5A5A5A5u; /* avoid the absorbing zero state */
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    ctx->rand_state = x;
    return x;
}

/* Random backoff in [min_ms, max_ms] inclusive */
static uint32_t t3_trunk_rand_backoff_ms(t3_trunk_ctx_t *ctx)
{
    uint32_t span = (ctx->t_holdoff_max_ms >= ctx->t_holdoff_min_ms)
                    ? (ctx->t_holdoff_max_ms - ctx->t_holdoff_min_ms + 1u)
                    : 1u;
    return ctx->t_holdoff_min_ms + (t3_trunk_rand_next(ctx) % span);
}

/* =========================================================================
 * TX — submit the C_RAND burst for the current request
 * ========================================================================= */
dmr_err_t t3_trunk_submit_rand_access(t3_trunk_ctx_t *ctx)
{
    t3_trunk_req_ctx_t *req = &ctx->req;
    dmr_mac_tx_req_t    txreq;

    memset(&txreq, 0, sizeof(txreq));
    txreq.slot     = ctx->tscc_slot;
    txreq.priority = DMR_MAC_PRIORITY_NORMAL;
    txreq.deadline_us = 0u;
    txreq.impolite = false;
    txreq.originated_from=T3_TX_ORIGIN_TRUNK;
    llc_t3_rand_access_build(&txreq.burst, req->service_kind, req->is_group,
                              req->dst_id, ctx->my_radio_id,
                              ctx->colour_code, ctx->tscc_slot);
    txreq.req_id = ctx->tx_req_id_next++;

    dmr_err_t err = mac_tx_enqueue(ctx->mq_mac_tx, &txreq);
    if (err != DMR_OK) {
        return err;
    }

    req->rand_req_id = txreq.req_id;
    ctx->stats.rand_access_sent++;
    return DMR_OK;
}

/* =========================================================================
 * Public API — request / cancel
 * ========================================================================= */
dmr_err_t t3_trunk_request_channel(t3_trunk_ctx_t *ctx,
                                     uint8_t service_kind,
                                     bool    is_group,
                                     uint32_t dst_id)
{
    pthread_mutex_lock(&ctx->state_mutex);

    if (ctx->req.active) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return DMR_ERR_BUSY;
    }

    t3_trunk_req_ctx_t *req = &ctx->req;
    memset(req, 0, sizeof(*req));
    req->active        = true;
    req->service_kind  = service_kind;
    req->is_group       = is_group;
    req->dst_id          = dst_id;
    req->attempt        = 0u;

    dmr_err_t err = t3_trunk_submit_rand_access(ctx);
    if (err != DMR_OK) {
        req->active = false;
        ctx->stats.grant_tx_failures++;
        pthread_mutex_unlock(&ctx->state_mutex);
        if (ctx->on_grant_result) {
            ctx->on_grant_result(ctx, T3_GRANT_TX_FAILED, 0u, 0u, false, 0u);
        }
        return DMR_OK; /* request was handled (failure reported via callback) */
    }

    ctx->state = T3_TRUNK_STATE_GRANT_WAIT;
    t3_trunk_arm_timer(&ctx->tmr_grantwait, ctx->t_grant_wait_ms);

    pthread_mutex_unlock(&ctx->state_mutex);
    return DMR_OK;
}

dmr_err_t t3_trunk_cancel(t3_trunk_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);

    if (!ctx->req.active) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return DMR_OK;
    }

    uint8_t attempts = (uint8_t)(ctx->req.attempt + 1u);

    t3_trunk_disarm_timer(&ctx->tmr_grantwait);
    t3_trunk_disarm_timer(&ctx->tmr_holdoff);
    ctx->req.active = false;
    ctx->state = T3_TRUNK_STATE_IDLE;

    pthread_mutex_unlock(&ctx->state_mutex);

    if (ctx->on_grant_result) {
        ctx->on_grant_result(ctx, T3_GRANT_ABORTED, 0u, 0u, false, attempts);
    }
    return DMR_OK;
}

/* =========================================================================
 * Internal: finish the request (success or final failure).
 *           Caller must hold state_mutex; this drops it before invoking
 *           callbacks to avoid re-entrant deadlock if the application
 *           calls back into this module from the callback.
 * ========================================================================= */
static void t3_trunk_finish_locked(t3_trunk_ctx_t *ctx,
                                     t3_grant_outcome_t outcome,
                                     uint16_t ch_id, uint8_t slot,
                                     bool emergency)
{
    uint8_t attempts = (uint8_t)(ctx->req.attempt + 1u);

    t3_trunk_disarm_timer(&ctx->tmr_grantwait);
    t3_trunk_disarm_timer(&ctx->tmr_holdoff);
    ctx->req.active = false;
    ctx->state = T3_TRUNK_STATE_IDLE;

    pthread_mutex_unlock(&ctx->state_mutex);

    if (outcome == T3_GRANT_OK) {
        ctx->stats.grants_received++;
        if (ctx->on_channel_switch) {
            ctx->on_channel_switch(ctx, ch_id, slot);
        }
    } else if (outcome == T3_GRANT_TIMEOUT) {
        ctx->stats.grant_timeouts++;
    }

    if (ctx->on_grant_result) {
        ctx->on_grant_result(ctx, outcome, ch_id, slot, emergency, attempts);
    }
}

/* =========================================================================
 * Internal: retry the C_RAND after a grant-wait timeout, or fail if
 *           attempts are exhausted. Caller must hold state_mutex.
 *           Drops the mutex internally via t3_trunk_finish_locked() on
 *           the failure path, or re-arms holdoff on the retry path
 *           (which keeps the mutex held until return).
 * ========================================================================= */
static void t3_trunk_retry_or_fail_locked(t3_trunk_ctx_t *ctx)
{
    t3_trunk_req_ctx_t *req = &ctx->req;

    if (req->attempt + 1u >= T3_RAND_MAX_ATTEMPTS) {
        t3_trunk_finish_locked(ctx, T3_GRANT_TIMEOUT, 0u, 0u, false);
        return;
    }

    req->attempt++;
    ctx->stats.rand_access_retries++;
    ctx->state = T3_TRUNK_STATE_HOLDOFF;

    uint32_t backoff_ms = t3_trunk_rand_backoff_ms(ctx);
    t3_trunk_arm_timer(&ctx->tmr_holdoff, backoff_ms);
}

/* =========================================================================
 * Event handlers
 * ========================================================================= */
static void t3_trunk_handle_tx_conf(t3_trunk_ctx_t *ctx,
                                      const dmr_mac_tx_conf_t *conf)
{
    pthread_mutex_lock(&ctx->state_mutex);

    if (!ctx->req.active || conf->req_id != ctx->req.rand_req_id) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return;
    }

    if (conf->result != DMR_MAC_TX_OK) {
        /* C_RAND could not be transmitted — treat like a timeout for retry
         * purposes (random access channel may have been busy). */
        t3_trunk_retry_or_fail_locked(ctx);
    }
    /* On success: we are already in GRANT_WAIT (armed at request time) —
     * nothing further to do here. */

    pthread_mutex_unlock(&ctx->state_mutex);
}

static void t3_trunk_handle_grantwait_timeout(t3_trunk_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);

    if (ctx->req.active && ctx->state == T3_TRUNK_STATE_GRANT_WAIT) {
        t3_trunk_retry_or_fail_locked(ctx);
    }

    pthread_mutex_unlock(&ctx->state_mutex);
}

static void t3_trunk_handle_holdoff_timeout(t3_trunk_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);

    if (!ctx->req.active || ctx->state != T3_TRUNK_STATE_HOLDOFF) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return;
    }

    dmr_err_t err = t3_trunk_submit_rand_access(ctx);
    if (err != DMR_OK) {
        ctx->stats.grant_tx_failures++;
        t3_trunk_finish_locked(ctx, T3_GRANT_TX_FAILED, 0u, 0u, false);
        return;
    }

    ctx->state = T3_TRUNK_STATE_GRANT_WAIT;
    t3_trunk_arm_timer(&ctx->tmr_grantwait, ctx->t_grant_wait_ms);

    pthread_mutex_unlock(&ctx->state_mutex);
}

/* =========================================================================
 * RX — look for TV_GRANT/TD_GRANT addressed to our request
 * ========================================================================= */
dmr_err_t t3_trunk_rx_burst(t3_trunk_ctx_t *ctx, const dmr_burst_t *burst)
{
    llc_rx_result_t res;
    dmr_err_t err = llc_rx_dispatch(burst, &res);
    if (err != DMR_OK) {
        return err;
    }
    if (res.type != LLC_RX_CSBK || !res.crc_ok) {
        return DMR_OK;
    }
    if (res.opcode != DMR_CSBKO_T3_TV_GRANT &&
        res.opcode != DMR_CSBKO_T3_TD_GRANT) {
        return DMR_OK; /* Not a grant — out of scope for this module */
    }

    uint16_t ch_id;
    uint8_t  grant_slot;
    uint32_t dst_id;
    bool     emergency;
    dmr_err_t perr = llc_t3_grant_parse(res.body, &ch_id, &grant_slot,
                                          &dst_id, &emergency);
    if (perr != DMR_OK) {
        return DMR_OK; /* Grant CRC failed — cannot trust contents; drop */
    }

    pthread_mutex_lock(&ctx->state_mutex);

    if (!ctx->req.active || ctx->state != T3_TRUNK_STATE_GRANT_WAIT) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return DMR_OK;
    }
    if (dst_id != ctx->req.dst_id) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return DMR_OK; /* Grant for a different call — not ours */
    }

    t3_trunk_finish_locked(ctx, T3_GRANT_OK, ch_id, grant_slot, emergency);
    return DMR_OK;
}

/* =========================================================================
 * Worker thread
 * ========================================================================= */
static void *t3_trunk_thread(void *arg)
{
    t3_trunk_ctx_t *ctx = (t3_trunk_ctx_t *)arg;
    struct epoll_event events[T3_TRUNK_EPOLL_MAX_EVENTS];

    DMR_LOGI("[T3-TRUNK] Worker thread started (radio_id=0x%06X cc=%u slot=%d)",
             ctx->my_radio_id, ctx->colour_code, ctx->tscc_slot);

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        DMR_LOGE("[T3-TRUNK] epoll_create1 failed: %s", strerror(errno));
        return NULL;
    }

    struct epoll_event ev;
    ev.events = EPOLLIN;
 ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_holdoff);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_holdoff), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_grantwait);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_grantwait), &ev);
    ev.data.fd = (int)ctx->mq_evt;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_evt, &ev);
    ev.data.fd = (int)ctx->mq_mac_conf;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_mac_conf, &ev);
    ev.data.fd = (int)ctx->mq_mac_rx;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_mac_rx, &ev);

    while (ctx->running) {
        int nev = epoll_wait(epfd, events, T3_TRUNK_EPOLL_MAX_EVENTS, 200);
        if (nev < 0) {
            if (errno == EINTR) continue;
            DMR_LOGE("[T3-TRUNK] epoll_wait: %s", strerror(errno));
            break;
        }

        for (int i = 0; i < nev; i++) {
            int fd = events[i].data.fd;

            if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_holdoff)) {
                t3_trunk_timer_drain(&ctx->tmr_holdoff);
                t3_trunk_handle_holdoff_timeout(ctx);
            } else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_grantwait)) {
                t3_trunk_timer_drain(&ctx->tmr_grantwait);
                t3_trunk_handle_grantwait_timeout(ctx);
            } else if (fd == (int)ctx->mq_evt) {
                t3_trunk_event_t e;
                while (mq_receive(ctx->mq_evt, (char *)&e, sizeof(e), NULL) >= 0) {
                    if (e.type == T3_TRUNK_EVT_SHUTDOWN) {
                        ctx->running = false;
                    }
                }
            } else if (fd == (int)ctx->mq_mac_conf) {
                dmr_mac_tx_conf_t conf;
                while (mq_receive(ctx->mq_mac_conf, (char *)&conf,
                                   sizeof(conf), NULL) >= 0) {
                    t3_trunk_handle_tx_conf(ctx, &conf);
                }
            } else if (fd == (int)ctx->mq_mac_rx) {
                dmr_burst_t burst;
                while (mq_receive(ctx->mq_mac_rx, (char *)&burst,
                                   sizeof(burst), NULL) >= 0) {
                    t3_trunk_rx_burst(ctx, &burst);
                }
            }
        }
    }

    close(epfd);
    DMR_LOGI("[T3-TRUNK] Worker thread exiting (radio_id=0x%06X)", ctx->my_radio_id);
    return NULL;
}

/* =========================================================================
 * Lifecycle
 * ========================================================================= */
dmr_err_t t3_trunk_init(t3_trunk_ctx_t *ctx,
                          dmr_slot_t      tscc_slot,
                          uint32_t        my_radio_id,
                          uint8_t         colour_code)
{
    if (ctx == NULL) return DMR_ERR_INVALID_PARAM;

    memset(ctx, 0, sizeof(*ctx));
    ctx->tscc_slot      = tscc_slot;
    ctx->my_radio_id    = my_radio_id;
    ctx->colour_code    = colour_code;
    ctx->state          = T3_TRUNK_STATE_IDLE;
    ctx->t_grant_wait_ms  = T3_T_GRANT_WAIT_MS;
    ctx->t_holdoff_min_ms = T3_T_HOLDOFF_MIN_MS;
    ctx->t_holdoff_max_ms = T3_T_HOLDOFF_MAX_MS;
    ctx->rand_state       = my_radio_id ? my_radio_id : 0xA5A5A5A5u;
    ctx->tx_req_id_next   = 1u;

    if (pthread_mutex_init(&ctx->state_mutex, NULL) != 0) {
        return DMR_ERR_INVALID_PARAM;
    }

    const char *mq_evt_name = (tscc_slot == DMR_SLOT_1)
                                   ? DMR_MQ_T3_TRUNK_EVT_S1 : DMR_MQ_T3_TRUNK_EVT_S2;
    const char *mq_tx_name   = (tscc_slot == DMR_SLOT_1)
                                   ? DMR_MQ_MAC_TX_REQ_S1   : DMR_MQ_MAC_TX_REQ_S2;
    const char *mq_conf_name = (tscc_slot == DMR_SLOT_1)
                                   ? DMR_MQ_MAC_TX_CONF_TRUNK_S1  : DMR_MQ_MAC_TX_CONF_TRUNK_S2;
    const char *mq_rx_name   = (tscc_slot == DMR_SLOT_1)
                                   ? DMR_MQ_MAC_RX_TRUNK_S1 : DMR_MQ_MAC_RX_TRUNK_S2;

    /* mq_evt: this module creates/owns it — private, per-slot. */
    ctx->mq_evt = t3_trunk_mq_create_own(mq_evt_name,
                                           T3_TRUNK_MQ_EVT_MAX_MSGS,
                                           T3_TRUNK_MQ_EVT_MSG_SIZE);

    /* The following three queues are created by MAC (mac_init() for
     * this slot), never by this module — see the ownership contract in
     * dmr_mac.h. They are opened together as one atomic set (all three
     * exist iff mac_init() for this slot has completed), with a single
     * shared bounded retry rather than retrying each independently. */
    dmr_err_t mac_q_err = t3_trunk_open_mac_queues(mq_tx_name, mq_conf_name,
                                                      mq_rx_name,
                                                      &ctx->mq_mac_tx,
                                                      &ctx->mq_mac_conf,
                                                      &ctx->mq_mac_rx);

    if (ctx->mq_evt == (mqd_t)-1 || mac_q_err != DMR_OK) {
        t3_trunk_destroy(ctx);
        return DMR_ERR_QUEUE_FULL;
    }

    
    
     if (dmr_phy_timer_oneshot_init(&ctx->tmr_holdoff)   != DMR_OK ||
          dmr_phy_timer_oneshot_init(&ctx->tmr_grantwait)    != DMR_OK) {
       DMR_LOGE("[T3-TRUNK] timerfd_create failed: %s", strerror(errno));
        return DMR_ERR_NO_MEM;
     }

    return DMR_OK;
}

dmr_err_t t3_trunk_start(t3_trunk_ctx_t *ctx)
{
    if (ctx == NULL) return DMR_ERR_INVALID_PARAM;
    if (ctx->running) return DMR_OK;

    ctx->running = true;
    if (pthread_create(&ctx->thread, NULL, t3_trunk_thread, ctx) != 0) {
        ctx->running = false;
        return DMR_ERR_INVALID_PARAM;
    }
    return DMR_OK;
}

dmr_err_t t3_trunk_stop(t3_trunk_ctx_t *ctx)
{
    if (ctx == NULL) return DMR_ERR_INVALID_PARAM;
    if (!ctx->running) return DMR_OK;

    t3_trunk_event_t e;
    memset(&e, 0, sizeof(e));
    e.type = T3_TRUNK_EVT_SHUTDOWN;
    e.timestamp_us = dmr_time_now_us();
    mq_send(ctx->mq_evt, (const char *)&e, sizeof(e), 1u);

    ctx->running = false;
    pthread_join(ctx->thread, NULL);
    return DMR_OK;
}

void t3_trunk_destroy(t3_trunk_ctx_t *ctx)
{
    if (ctx == NULL) return;

    if (ctx->mq_evt != 0 && ctx->mq_evt != (mqd_t)-1) {
        mq_close(ctx->mq_evt);
    }
    if (ctx->mq_mac_tx != 0 && ctx->mq_mac_tx != (mqd_t)-1) {
        mq_close(ctx->mq_mac_tx);
    }
    if (ctx->mq_mac_conf != 0 && ctx->mq_mac_conf != (mqd_t)-1) {
        mq_close(ctx->mq_mac_conf);
    }
    if (ctx->mq_mac_rx != 0 && ctx->mq_mac_rx != (mqd_t)-1) {
        mq_close(ctx->mq_mac_rx);
    }
    if (ctx->tmr_holdoff.fd > 0) {
        dmr_phy_timer_oneshot_destroy(&ctx->tmr_holdoff);
    }
    if (ctx->tmr_grantwait.fd > 0) {
        dmr_phy_timer_oneshot_destroy(&ctx->tmr_grantwait);

    }

    /* Only mq_evt is unlinked — this module created it (see
     * t3_trunk_init / t3_trunk_mq_create_own) and is its sole owner.
     * mq_mac_tx/conf/rx are MAC-owned (see ownership contract in
     * dmr_mac.h); this module only ever closes its handle to them,
     * never unlinks the name — that is mac_destroy()'s responsibility. */
    if (ctx->mq_evt != (mqd_t)-1) {
        const char *mq_evt_name = (ctx->tscc_slot == DMR_SLOT_1)
                                       ? DMR_MQ_T3_TRUNK_EVT_S1
                                       : DMR_MQ_T3_TRUNK_EVT_S2;
        mq_unlink(mq_evt_name);
    }

    pthread_mutex_destroy(&ctx->state_mutex);
    memset(ctx, 0, sizeof(*ctx));
}

/* =========================================================================
 * Introspection
 * ========================================================================= */
void t3_trunk_get_stats(t3_trunk_ctx_t *ctx, t3_trunk_stats_t *out)
{
    if (ctx == NULL || out == NULL) return;
    pthread_mutex_lock(&ctx->state_mutex);
    *out = ctx->stats;
    pthread_mutex_unlock(&ctx->state_mutex);
}

t3_trunk_state_t t3_trunk_get_state(t3_trunk_ctx_t *ctx)
{
    t3_trunk_state_t s;
    pthread_mutex_lock(&ctx->state_mutex);
    s = ctx->state;
    pthread_mutex_unlock(&ctx->state_mutex);
    return s;
}
