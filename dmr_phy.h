/**
 * @file dmr_phy.h
 * @brief MOD-01 — Physical Layer — Skeleton (timer subsystem first)
 *
 * STATUS: Skeleton. Only the timing subsystem is implemented in this
 * pass — periodic TDMA slot-boundary ticks with microsecond precision.
 * Symbol modulation/demodulation (4FSK), RSSI, and symbol sync are not
 * yet implemented; this file/module will grow to hold them.
 *
 * Backend
 * =======
 * Implemented today with Linux timerfd (CLOCK_MONOTONIC,
 * TFD_TIMER_ABSTIME) — software timing, suitable for development and
 * for any host-based testing/simulation. This is explicitly a
 * placeholder for a future hardware timer backend (e.g. a dedicated
 * timer peripheral, DMA-driven symbol clock, or an FPGA/SDR frame
 * clock) — see "Swapping in a hardware timer" below.
 *
 * Why TFD_TIMER_ABSTIME and not a relative re-arm loop
 * =====================================================
 * A periodic timer built by re-arming a relative timerfd after each
 * expiry (read → compute next interval → timerfd_settime again) drifts:
 * each re-arm is computed relative to "now", so processing latency
 * between expiry and re-arm accumulates as error over many ticks. DMR
 * TDMA timing must not drift relative to the BS outbound (the spec's
 * ±1.0/±2.0 ppm clock drift budget — ETSI TS 102 361-1 Clause 10.1.4 —
 * is for the whole radio, not extra slop from this layer). Using
 * timerfd_settime() with TFD_TIMER_ABSTIME and a fixed it_interval lets
 * the kernel compute each successive absolute expiry from the original
 * start time, so ticks stay phase-locked to the TDMA grid rather than
 * to "30ms after we last got around to checking".
 *
 * Tick period
 * ===========
 * ETSI TS 102 361-1 Clause 10.2.1: 4800 symbols/s, 2 bits/symbol.
 * One TDMA timeslot = 30 ms = MAC_TIMESLOT_MS (already defined and used
 * by mac_channel_access.c's LBT/holdoff timing) = 360 symbols = exactly
 * the 33-byte (264-bit / 2 = 132 dibit) burst period. This module's
 * default tick period is DMR_PHY_TICK_US (30000 µs = 30 ms), i.e. one
 * tick per timeslot boundary — the rate at which a real PHY would need
 * to hand off "slot N has just started" to MAC. A caller may request a
 * different period (e.g. a faster tick for symbol-level work later);
 * the API takes the period explicitly rather than hardcoding 30 ms.
 *
 * Swapping in a hardware timer later
 * ===================================
 * The public API (dmr_phy_timer_init/start/stop/destroy, the fd
 * accessor, and the tick-count/timestamp accessors) is deliberately
 * backend-agnostic — none of it leaks timerfd-specific types. A future
 * hardware backend only needs to:
 *   1. Replace the timerfd_create/settime calls inside dmr_phy.c with
 *      the hardware driver's equivalent arm/start calls.
 *   2. Provide a pollable/waitable fd (or change dmr_phy_timer_get_fd()
 *      callers to an alternative wait primitive if the hardware path
 *      cannot expose one) — epoll integration in mac_channel_access.c
 *      and elsewhere only depends on getting *a* fd, not on it being a
 *      Linux timerfd specifically.
 *   3. Update dmr_phy_timer_tick_count()/last_tick_us() to read from
 *      the hardware counter/timestamp register instead of the software
 *      tick counter.
 * No caller-visible API changes should be needed for that swap.
 */

#ifndef DMR_PHY_H
#define DMR_PHY_H

#define _POSIX_C_SOURCE 200809L

#include <stdint.h>
#include <stdbool.h>
#include <mqueue.h>

#include "dmr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Timing constants
 * ========================================================================= */

/**
 * Default PHY tick period: one DMR TDMA timeslot (30 ms), matching
 * MAC_TIMESLOT_MS in dmr_mac.h (ETSI TS 102 361-1, 4800 symbols/s,
 * 360 symbols per slot = 30 ms exactly).
 */
#define DMR_PHY_TICK_US   30000u

/**
 * Minimum tick period this module will accept. Linux timerfd has no
 * hard minimum, but sub-millisecond software timing is not meaningful
 * on a non-realtime kernel for DMR's needs (the finest grain any
 * current caller needs is the symbol period, 208.33 µs — and even that
 * is for a future hardware backend, not this software one). Reject
 * anything below this as a likely caller error.
 */
#define DMR_PHY_TICK_MIN_US  100u

/* =========================================================================
 * PHY timer context
 *
 * One instance drives one periodic tick stream. A real deployment will
 * likely want one per slot (mirroring mac_ctx_t's per-slot design) once
 * MAC is wired to consume ticks from this module; this header does not
 * assume a 1:1 relationship, since the timer itself has no concept of
 * "slot" — that's MAC's interpretation of when ticks arrive.
 * ========================================================================= */
typedef struct {
    int       fd;             /**< Pollable timer fd (epoll-compatible)    */
    uint32_t  period_us;      /**< Configured tick period, microseconds    */
    uint64_t  start_us;       /**< dmr_time_now_us() at dmr_phy_timer_start() */
    uint64_t  tick_count;     /**< Number of ticks observed so far          */
    uint64_t  last_tick_us;   /**< Timestamp of the most recent tick        */
    bool      running;
} dmr_phy_timer_ctx_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Create the periodic timer (does not start it running).
 *
 * @param ctx        Caller-allocated context, zeroed and filled in
 * @param period_us  Tick period in microseconds. Must be >=
 *                    DMR_PHY_TICK_MIN_US. Pass DMR_PHY_TICK_US for the
 *                    standard one-tick-per-TDMA-timeslot rate.
 * @return DMR_OK, or DMR_ERR_INVALID_PARAM / DMR_ERR_NO_MEM on failure.
 */
dmr_err_t dmr_phy_timer_init(dmr_phy_timer_ctx_t *ctx, uint32_t period_us);

/**
 * @brief Arm the timer to start ticking. The first tick fires
 *        period_us after this call; subsequent ticks are phase-locked
 *        to that start time (TFD_TIMER_ABSTIME — see file header for
 *        why this matters), not to "period_us after the last tick was
 *        actually read", so no drift accumulates from processing
 *        latency between ticks.
 *
 * Safe to call again after dmr_phy_timer_stop() to restart from a new
 * phase reference.
 */
dmr_err_t dmr_phy_timer_start(dmr_phy_timer_ctx_t *ctx);

/**
 * @brief Disarm the timer. The fd remains valid (still pollable, but
 *        will not fire) until dmr_phy_timer_destroy().
 */
dmr_err_t dmr_phy_timer_stop(dmr_phy_timer_ctx_t *ctx);

/**
 * @brief Close the timer fd and zero the context.
 */
void dmr_phy_timer_destroy(dmr_phy_timer_ctx_t *ctx);

/**
 * @brief Get the pollable fd for this timer, for epoll/select
 *        integration by a caller's own event loop (e.g. MAC's worker
 *        thread once wired up). Returns -1 if not yet initialised.
 */
int dmr_phy_timer_get_fd(const dmr_phy_timer_ctx_t *ctx);

/**
 * @brief Consume one or more pending tick notifications on the fd
 *        (equivalent to draining a timerfd: read() the expiry count)
 *        and update tick_count/last_tick_us accordingly.
 *
 * Call this after the fd becomes readable. Returns the number of
 * ticks consumed in this call (normally 1; >1 if the caller fell
 * behind and ticks coalesced — see "missed ticks" below).
 *
 * @return Number of ticks consumed (>=0), or -1 with errno set on a
 *         read error (e.g. EAGAIN if called when not actually
 *         readable — callers should only call this after their event
 *         loop indicates the fd is ready).
 */
int64_t dmr_phy_timer_consume(dmr_phy_timer_ctx_t *ctx);

/**
 * @brief Total number of ticks observed since dmr_phy_timer_start(),
 *        i.e. since the last (re)arm. Thread-safety: this struct is
 *        not internally synchronised — if accessed from a thread other
 *        than the one calling dmr_phy_timer_consume(), the caller is
 *        responsible for its own synchronisation (matches the
 *        single-owner-thread pattern used by mac_ctx_t/ccl_*_ctx_t
 *        elsewhere in this codebase).
 */
uint64_t dmr_phy_timer_tick_count(const dmr_phy_timer_ctx_t *ctx);

/**
 * @brief dmr_time_now_us() timestamp of the most recent consumed tick.
 *        0 if no tick has been consumed yet.
 */
uint64_t dmr_phy_timer_last_tick_us(const dmr_phy_timer_ctx_t *ctx);

/* =========================================================================
 * One-shot timer
 *
 * Used by every DMR protocol timer (T_IdleSrch, T_Holdoff, T_DataTxLmt,
 * T_Hangtime, T_Response, T_GrantWait, voice superframe watchdog, etc.).
 * All of these are one-shot: armed with a deadline, fire once, then stay
 * idle until explicitly re-armed. None of them need periodic behaviour.
 *
 * TODAY — Linux timerfd backend (CLOCK_MONOTONIC, TFD_NONBLOCK).
 * FUTURE — swap the init/arm/disarm/drain implementations in dmr_phy.c
 *   to call a hardware timer driver instead; the API and the fd returned
 *   by dmr_phy_timer_oneshot_get_fd() remain unchanged for all callers.
 *
 * Hardware migration guide (same as for the periodic timer above):
 *   If the hardware timer cannot expose a pollable fd, change
 *   dmr_phy_timer_oneshot_get_fd() callers to an alternative wait
 *   primitive — the only places that use the fd are add_to_epoll() and
 *   the epoll dispatch switch (fd == ctx->tmr_X.fd), so the blast
 *   radius is small and contained to each module's worker thread.
 * ========================================================================= */

/**
 * @brief One-shot timer context.
 *
 * Replace every raw `int tfd_X` field in module context structs
 * (mac_ctx_t, ccl_voice_ctx_t, ccl_data_ctx_t, t3_trunk_ctx_t) with
 * `dmr_phy_timer_oneshot_t tmr_X`. Use dmr_phy_timer_oneshot_get_fd()
 * wherever the raw fd was previously used (add_to_epoll, epoll dispatch
 * comparisons). No other changes to caller code are required.
 */
typedef struct {
    int  fd;     /**< Pollable fd — CLOCK_MONOTONIC timerfd today;
                      hardware timer fd in the future              */
    bool armed;  /**< True between arm_ms() and expiry/disarm()   */
     pthread_mutex_t     state_mutex;
} dmr_phy_timer_oneshot_t;

/**
 * @brief Create the one-shot timer fd.  Does not arm it.
 * @return DMR_OK, or DMR_ERR_NO_MEM if the OS cannot allocate a fd.
 */
dmr_err_t dmr_phy_timer_oneshot_init(dmr_phy_timer_oneshot_t *t);

/**
 * @brief Arm the timer to fire once after `ms` milliseconds.
 *        Safe to call again while already armed — re-arms to the new
 *        deadline, discarding the old one (same semantics as
 *        timerfd_settime() overwriting a pending timer).
 */
dmr_err_t dmr_phy_timer_oneshot_arm_ms(dmr_phy_timer_oneshot_t *t,
                                         uint32_t ms);

/**
 * @brief Disarm the timer without waiting for it to fire.
 *        Safe to call on a timer that is already disarmed (no-op).
 */
dmr_err_t dmr_phy_timer_oneshot_disarm(dmr_phy_timer_oneshot_t *t);

/**
 * @brief Consume the pending expiry notification after the fd becomes
 *        readable (equivalent to read()ing a timerfd to clear the
 *        POLLIN condition).  Must be called from the epoll dispatch
 *        handler before the next epoll_wait() to prevent the fd from
 *        remaining spuriously readable.  No-op if not readable.
 */
void dmr_phy_timer_oneshot_drain(dmr_phy_timer_oneshot_t *t);

/**
 * @brief Return the pollable fd for epoll/select integration.
 *        Returns -1 if not yet initialised or already destroyed.
 */
int dmr_phy_timer_oneshot_get_fd(const dmr_phy_timer_oneshot_t *t);

/**
 * @brief Disarm and close the fd.  Zeroes the struct.
 */
void dmr_phy_timer_oneshot_destroy(dmr_phy_timer_oneshot_t *t);

/* =========================================================================
 * PHY → MAC TX-done notification
 *
 * When MAC hands a burst to the PHY via mq_phy_tx, MAC must NOT post a
 * TX_CONF to the originating CCL module immediately — doing so would let
 * CCL submit the next burst before the current one has actually been
 * transmitted, causing the PHY to receive bursts faster than it can send
 * them (overwrite problem). Instead:
 *
 *   MAC → mq_phy_tx   → PHY: burst to transmit
 *   PHY → mq_phy_done → MAC: "burst N transmitted OK/FAIL" (after
 *                             one TDMA slot of actual air time)
 *   MAC: on receiving the done notification → mac_post_tx_conf()
 *
 * Queue ownership: MAC creates DMR_MQ_PHY_DONE_S1/S2 (O_CREAT) as part
 * of its MAC-owns-all-shared-queues contract (see dmr_mac.h). PHY opens
 * them O_WRONLY via dmr_phy_open_tx_done_queue() without O_CREAT.
 * ========================================================================= */

/** Per-slot PHY→MAC TX-done queue names.  MAC creates; PHY writes. */
#define DMR_MQ_PHY_DONE_S1   "/dmr_phy_done_s1"
#define DMR_MQ_PHY_DONE_S2   "/dmr_phy_done_s2"

/** Result codes carried in dmr_phy_tx_done_t.result */
typedef enum {
    DMR_PHY_TX_DONE_OK      = 0,  /**< Burst transmitted successfully      */
    DMR_PHY_TX_DONE_FAIL    = 1,  /**< TX failed (e.g. HW error, PA fault) */
    DMR_PHY_TX_DONE_TIMEOUT = 2,  /**< PHY did not receive burst in time   */
} dmr_phy_tx_result_t;

/** Message posted by PHY to mq_phy_done after each burst transmission. */
typedef struct {
    uint32_t           req_id;   /**< Mirrors dmr_mac_tx_req_t.req_id     */
    uint8_t            slot;     /**< DMR_SLOT_1 or DMR_SLOT_2             */
    uint8_t            originated_from; /**< Mirrors tx_req.originated_from*/
    dmr_phy_tx_result_t result;
} dmr_phy_tx_conf_t;

/**
 * @brief Open the PHY→MAC TX-done queue for a given slot (PHY side —
 *        O_WRONLY, no O_CREAT). Retries with bounded backoff exactly
 *        like the other CCL-side MAC queue openers; call after the MAC
 *        that owns the queue has been initialised.
 *
 * @param slot      DMR_SLOT_1 or DMR_SLOT_2
 * @param out_mqd   Output: the opened queue descriptor
 * @return DMR_OK, or DMR_ERR_IO after retries exhausted.
 */
dmr_err_t dmr_phy_open_tx_done_queue(uint8_t slot, mqd_t *out_mqd);

/**
 * @brief Post a TX-done notification.  Non-blocking; drops silently if
 *        MAC's queue is full (MAC's epoll will catch the next one;
 *        a missing done on an overloaded system is better than blocking
 *        the PHY's transmission path).
 */
void dmr_phy_post_tx_done(mqd_t mq_done, const dmr_phy_tx_conf_t *done);

#ifdef __cplusplus
}
#endif

#endif /* DMR_PHY_H */