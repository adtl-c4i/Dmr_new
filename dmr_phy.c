/**
 * @file dmr_phy.c
 * @brief MOD-01 — Physical Layer — Skeleton (timer subsystem first)
 *
 * See dmr_phy.h for design notes, especially why TFD_TIMER_ABSTIME is
 * used instead of a relative re-arm loop.
 */

#define _POSIX_C_SOURCE 200809L

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <mqueue.h>
#include <fcntl.h>
#include <sys/timerfd.h>

#include "dmr_phy.h"
#include "dmr_mac.h"    /* DMR_MQ_OPEN_RETRY_COUNT, DMR_MQ_OPEN_RETRY_DELAY_MS */

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

dmr_err_t dmr_phy_timer_init(dmr_phy_timer_ctx_t *ctx, uint32_t period_us)
{
    if (ctx == NULL) return DMR_ERR_INVALID_PARAM;
    if (period_us < DMR_PHY_TICK_MIN_US) return DMR_ERR_INVALID_PARAM;

    memset(ctx, 0, sizeof(*ctx));
    ctx->period_us = period_us;
    


    /* CLOCK_MONOTONIC: immune to wall-clock adjustments (NTP step,
     * user changing the date, etc.) — matches dmr_time_now_us() in
     * dmr_types.h, so tick timestamps and dmr_time_now_us() readings
     * stay on the same clock and remain directly comparable. */
    ctx->fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (ctx->fd < 0) {
        return DMR_ERR_NO_MEM;
    }

    return DMR_OK;
}

dmr_err_t dmr_phy_timer_start(dmr_phy_timer_ctx_t *ctx)
{
    if (ctx == NULL || ctx->fd < 0) return DMR_ERR_INVALID_PARAM;

    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return DMR_ERR_IO;
    }

    struct itimerspec its;
    memset(&its, 0, sizeof(its));

    /* First expiry: now + one period. Expressed as an absolute
     * timespec (TFD_TIMER_ABSTIME below) rather than letting the
     * kernel add it to "current time at settime() call" implicitly —
     * being explicit here makes the phase reference unambiguous and
     * matches what ctx->start_us records. */
    uint64_t period_ns = (uint64_t)ctx->period_us * 1000ull;
    uint64_t first_ns  = (uint64_t)now.tv_sec * 1000000000ull
                        + (uint64_t)now.tv_nsec
                        + period_ns;

    its.it_value.tv_sec  = (time_t)(first_ns / 1000000000ull);
    its.it_value.tv_nsec = (long)(first_ns % 1000000000ull);

    /* it_interval: fixed period — the kernel computes each successive
     * absolute expiry as it_value + N*it_interval, so processing
     * latency between our reads of the fd never accumulates as drift
     * relative to the original phase reference. This is the entire
     * reason TFD_TIMER_ABSTIME is used instead of re-arming a relative
     * timer after each tick — see the file header in dmr_phy.h. */
    its.it_interval.tv_sec  = (time_t)(period_ns / 1000000000ull);
    its.it_interval.tv_nsec = (long)(period_ns % 1000000000ull);

    if (timerfd_settime(ctx->fd, TFD_TIMER_ABSTIME, &its, NULL) != 0) {
        return DMR_ERR_IO;
    }

    ctx->start_us     = dmr_time_now_us();
    ctx->tick_count   = 0u;
    ctx->last_tick_us = 0u;
    ctx->running      = true;

    return DMR_OK;
}

dmr_err_t dmr_phy_timer_stop(dmr_phy_timer_ctx_t *ctx)
{
    if (ctx == NULL || ctx->fd < 0) return DMR_ERR_INVALID_PARAM;

    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    /* All-zero it_value disarms the timer (man timerfd_settime) —
     * works the same whether or not TFD_TIMER_ABSTIME is set. */
    if (timerfd_settime(ctx->fd, 0, &its, NULL) != 0) {
        return DMR_ERR_IO;
    }

    ctx->running = false;
    return DMR_OK;
}

void dmr_phy_timer_destroy(dmr_phy_timer_ctx_t *ctx)
{
    if (ctx == NULL) return;
    if (ctx->fd >= 0) {
        close(ctx->fd);
    }

    
    memset(ctx, 0, sizeof(*ctx));
    ctx->fd = -1;
}

/* =========================================================================
 * Accessors
 * ========================================================================= */

int dmr_phy_timer_get_fd(const dmr_phy_timer_ctx_t *ctx)
{
    if (ctx == NULL) return -1;
    return ctx->fd;
}

int64_t dmr_phy_timer_consume(dmr_phy_timer_ctx_t *ctx)
{
    if (ctx == NULL || ctx->fd < 0) {
        errno = EINVAL;
        return -1;
    }

    uint64_t expirations = 0u;
    ssize_t  n = read(ctx->fd, &expirations, sizeof(expirations));
    if (n < 0) {
        /* EAGAIN: fd was not actually readable — caller's event loop
         * should not have called this; pass errno through unchanged. */
        return -1;
    }
    if (n != (ssize_t)sizeof(expirations)) {
        /* Should not happen for a timerfd per its man page, but guard
         * against a short read rather than trusting the kernel here. */
        errno = EIO;
        return -1;
    }

    /* expirations may be >1 if this process fell behind and the
     * kernel coalesced missed ticks into a single readable count —
     * normal, expected behaviour for a periodic timerfd, not an
     * error. All of them are counted; last_tick_us reflects "now",
     * i.e. when we actually caught up and read them, not the
     * individual (unrecoverable) expiry times of each missed tick. */
    ctx->tick_count   += expirations;
    ctx->last_tick_us  = dmr_time_now_us();

    return (int64_t)expirations;
}

uint64_t dmr_phy_timer_tick_count(const dmr_phy_timer_ctx_t *ctx)
{
    if (ctx == NULL) return 0u;
    return ctx->tick_count;
}

uint64_t dmr_phy_timer_last_tick_us(const dmr_phy_timer_ctx_t *ctx)
{
    if (ctx == NULL) return 0u;
    return ctx->last_tick_us;
}

/* =========================================================================
 * One-shot timer implementation
 * ========================================================================= */

dmr_err_t dmr_phy_timer_oneshot_init(dmr_phy_timer_oneshot_t *t)
{
    if (t == NULL) return DMR_ERR_INVALID_PARAM;

    memset(t, 0, sizeof(*t));
        pthread_mutex_init(&t->state_mutex, NULL); // mutex for arm and disarm flag

    t->fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (t->fd < 0) {
        return DMR_ERR_NO_MEM;
    }
    pthread_mutex_lock(&t->state_mutex);
    t->armed = false;
    pthread_mutex_unlock(&t->state_mutex);
    
    return DMR_OK;
}

dmr_err_t dmr_phy_timer_oneshot_arm_ms(dmr_phy_timer_oneshot_t *t,
                                         uint32_t ms)
{
    if (t == NULL || t->fd < 0) return DMR_ERR_INVALID_PARAM;
    if (ms == 0u) ms = 1u; /* Zero would disarm; clamp to 1ms minimum */

    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec  = ms / 1000u;
    its.it_value.tv_nsec = (long)(ms % 1000u) * 1000000L;
    /* it_interval left zero — fires exactly once (one-shot) */

    if (timerfd_settime(t->fd, 0, &its, NULL) != 0) {
        return DMR_ERR_IO;
    }

        pthread_mutex_lock(&t->state_mutex);
    t->armed = true;
    pthread_mutex_unlock(&t->state_mutex);
    return DMR_OK;
}

dmr_err_t dmr_phy_timer_oneshot_disarm(dmr_phy_timer_oneshot_t *t)
{
    if (t == NULL || t->fd < 0) return DMR_ERR_INVALID_PARAM;

    pthread_mutex_lock(&t->state_mutex);
    bool was_armed = t->armed;
    pthread_mutex_unlock(&t->state_mutex);
    if (!was_armed) return DMR_OK; /* already idle — no-op */

    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    /* All-zero it_value disarms the timer */
    timerfd_settime(t->fd, 0, &its, NULL);

    pthread_mutex_lock(&t->state_mutex);
    t->armed = false;
    pthread_mutex_unlock(&t->state_mutex);
    return DMR_OK;
}

void dmr_phy_timer_oneshot_drain(dmr_phy_timer_oneshot_t *t)
{
    if (t == NULL || t->fd < 0) return;

    uint64_t exp = 0u;
    ssize_t n = read(t->fd, &exp, sizeof(exp));
    (void)n; /* EAGAIN is normal if drain() is called speculatively */
    pthread_mutex_lock(&t->state_mutex);
    t->armed = false;
    pthread_mutex_unlock(&t->state_mutex);
    
}

int dmr_phy_timer_oneshot_get_fd(const dmr_phy_timer_oneshot_t *t)
{
    if (t == NULL) return -1;
    return t->fd;
}

void dmr_phy_timer_oneshot_destroy(dmr_phy_timer_oneshot_t *t)
{
    if (t == NULL) return;
    if (t->fd >= 0) {
        pthread_mutex_destroy(&t->state_mutex);
        close(t->fd);
    }
    memset(t, 0, sizeof(*t));
    t->fd = -1;
}

/* =========================================================================
 * PHY → MAC TX-done notification
 * ========================================================================= */

dmr_err_t dmr_phy_open_tx_done_queue(uint8_t slot, mqd_t *out_mqd)
{
    if (out_mqd == NULL) return DMR_ERR_INVALID_PARAM;

    const char *name = (slot == 1u) ? DMR_MQ_PHY_DONE_S1
                                     : DMR_MQ_PHY_DONE_S2;
    int attempts = 0;

    do {
        mqd_t mq = mq_open(name, O_NONBLOCK | O_WRONLY);
        if (mq != (mqd_t)-1) {
            *out_mqd = mq;
            return DMR_OK;
        }
        if (errno != ENOENT) break;

        attempts++;
        if (attempts < DMR_MQ_OPEN_RETRY_COUNT) {
            struct timespec ts = { 0, (long)DMR_MQ_OPEN_RETRY_DELAY_MS * 1000000L };
            nanosleep(&ts, NULL);
        }
    } while (attempts < DMR_MQ_OPEN_RETRY_COUNT);

    *out_mqd = (mqd_t)-1;
    return DMR_ERR_IO;
}

void dmr_phy_post_tx_done(mqd_t mq_done, const dmr_phy_tx_conf_t *done)
{
    if (mq_done == (mqd_t)-1 || done == NULL) return;

    /* Non-blocking — if MAC's queue is full, drop silently.  A missed
     * done notification is less harmful than blocking the PHY TX path;
     * MAC's T_DataTxLmt watchdog provides a safety net for the case
     * where done notifications stop arriving altogether. */
    mq_send(mq_done, (const char *)done, sizeof(*done), 0u);
}