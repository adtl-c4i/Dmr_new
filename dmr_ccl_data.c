/**
 * @file dmr_ccl_data.c
 * @brief MOD-06 — Call Control Layer (CCL) Data Services
 *
 * See dmr_ccl_data.h for module overview.
 */

#define _POSIX_C_SOURCE 200809L

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>

#include "dmr_ccl_data.h"
#include "dmr_llc.h"

/* =========================================================================
 * Local constants
 * ========================================================================= */
#define CCL_DATA_MQ_MAX_MSGS         8
#define CCL_DATA_MQ_EVT_MSG_SIZE     sizeof(ccl_data_event_t)
#define CCL_DATA_MQ_BURST_MSG_SIZE   sizeof(dmr_burst_t)
#define CCL_DATA_MQ_TX_REQ_MSG_SIZE  sizeof(dmr_mac_tx_req_t)
#define CCL_DATA_MQ_TX_CONF_MSG_SIZE sizeof(dmr_mac_tx_conf_t)

#define CCL_DATA_EPOLL_MAX_EVENTS    8

/* Response Header Class/Type/Status constants are public — see
 * CCL_DATA_RESP_CLASS_... and CCL_DATA_RESP_TYPE_... in dmr_ccl_data.h. */

/* =========================================================================
 * Small helpers
 * ========================================================================= */

static void ccl_data_arm_timer(dmr_phy_timer_oneshot_t *t, uint32_t ms)
{
    dmr_phy_timer_oneshot_arm_ms(t, ms);
}

/* Disarm a one-shot PHY timer */
static void ccl_data_disarm_timer(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_disarm(t);
}

/* Drain a one-shot PHY timer after expiry */
static void ccl_data_timer_drain(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_drain(t);
}



/* =========================================================================
 * TX side — build & submit Preamble + Header + Rate-1 blocks
 * ========================================================================= */

/**
 * @brief Submit exactly ONE burst from the current transfer sequence and
 *        record its req_id as pending_req_id. Called once at transfer start
 *        (phase = PREAMBLE) and then once per TX_CONF from MAC as the
 *        state machine advances through PREAMBLE → HEADER → BLOCK_0 … → DONE.
 *
 * This replaces the old loop-based ccl_data_submit_transfer() which dumped
 * all bursts into the MAC TX queue at once, causing overflows and meaning
 * only the last burst actually got transmitted reliably.
 *
 * Caller must hold state_mutex.
 */
static dmr_err_t ccl_data_tx_next_burst(ccl_data_ctx_t *ctx)
{
    ccl_data_tx_ctx_t *tx = &ctx->tx;
    dmr_mac_tx_req_t   req;

    memset(&req, 0, sizeof(req));
    req.slot        = ctx->slot;
    req.priority    = DMR_MAC_PRIORITY_NORMAL;
    req.deadline_us = 0u;
    req.impolite    = false;
    req.req_id      = ctx->tx_req_id_next++;

    switch (tx->tx_phase) {

    case CCL_DATA_TX_PHASE_PREAMBLE: {
        uint8_t cbf = tx->sack_retry_active
                          ? (uint8_t)(1u + tx->sack_retry_count)
                          : (uint8_t)(1u + tx->n_blocks);
        llc_csbk_preamble_build(&req.burst, true, tx->is_group,
                                 cbf,
                                 tx->dst_id, ctx->my_radio_id,
                                 ctx->colour_code, ctx->slot);
        break;
    }

    case CCL_DATA_TX_PHASE_HEADER:
        switch (tx->sd_kind) {
        case CCL_DATA_SD_KIND_RAW:
            llc_data_hdr_raw_build(&req.burst, tx->dst_id, ctx->my_radio_id,
                                    tx->is_group, tx->sap, tx->src_port, tx->dst_port,
                                    tx->n_blocks, tx->confirmed, !tx->sack_retry_active,
                                    ctx->colour_code, ctx->slot);
            break;
        case CCL_DATA_SD_KIND_DEFINED:
            llc_data_hdr_dd_build(&req.burst, tx->dst_id, ctx->my_radio_id,
                                   tx->is_group, tx->sap, tx->dd_format,
                                   tx->n_blocks, tx->confirmed, !tx->sack_retry_active,
                                   ctx->colour_code, ctx->slot);
            break;
        case CCL_DATA_SD_KIND_STATUS_PRECODED:
            llc_data_hdr_sp_build(&req.burst, tx->dst_id, ctx->my_radio_id,
                                   tx->is_group, tx->sap, tx->src_port, tx->dst_port,
                                   tx->status_value, tx->confirmed,
                                   ctx->colour_code, ctx->slot);
            break;
        case CCL_DATA_SD_KIND_IP:
        default:
            if (tx->confirmed) {
                llc_data_hdr_conf_build(&req.burst, tx->dst_id, ctx->my_radio_id,
                                         tx->is_group, tx->sap, tx->n_blocks,
                                         tx->pad_octets, tx->fsn,
                                         0u, true, !tx->sack_retry_active,
                                         ctx->colour_code, ctx->slot);
            } else {
                llc_data_hdr_unconf_build(&req.burst, tx->dst_id, ctx->my_radio_id,
                                           tx->is_group, tx->sap, tx->n_blocks,
                                           tx->pad_octets, tx->fsn,
                                           ctx->colour_code, ctx->slot);
            }
            break;
        }
        break;

    case CCL_DATA_TX_PHASE_BLOCKS: {
        uint8_t pos = tx->tx_block_idx; /* position within this send sequence */
        uint8_t i;      /* actual block index / DBSN to transmit */
        bool    last;   /* last burst of *this* sequence (full or selective) */

        if (tx->sack_retry_active) {
            i    = tx->sack_retry_list[pos];
            last = (pos == (uint8_t)(tx->sack_retry_count - 1u));
        } else {
            i    = pos;
            last = (pos == (uint8_t)(tx->n_blocks - 1u));
        }

        size_t off    = (size_t)i * CCL_DATA_BLOCK_PAYLOAD;
        size_t remain = tx->data_len - off;
        size_t plen   = (remain < CCL_DATA_BLOCK_PAYLOAD)
                             ? remain : CCL_DATA_BLOCK_PAYLOAD;

        /* last_block (LB) here reflects "last burst of this on-air
         * sequence", matching what llc_data_block_rate1_build's
         * doc/Cl.8.2.2.2 mean by LB — it is NOT "last block of the
         * overall message" during a selective retry, since earlier
         * blocks the peer already has correctly are not resent. The
         * peer's reassembly must key completion off DBSN coverage
         * (ccl_data_rx_block already does — see rx->seen[]), not
         * solely off LB, for this to interoperate with a selective
         * retry in progress. */
        llc_data_block_rate1_build(&req.burst, i, last,
                                    &tx->data[off], plen,
                                    ctx->colour_code, ctx->slot);
        ctx->stats.tx_blocks++;

        if (last) {
            tx->last_req_id = req.req_id;
        }
        break;
    }

    default:
        /* CCL_DATA_TX_PHASE_DONE — nothing left to send */
        return DMR_OK;
    }
    req.originated_from=CCL_TX_ORIGIN_DATA;
    dmr_err_t err = mac_tx_enqueue(ctx->mq_mac_tx, &req);
    if (err != DMR_OK) return err;

    tx->pending_req_id = req.req_id;
    return DMR_OK;
}

/* Keep ccl_data_submit_transfer as the public entry point used by tests —
 * it now simply initialises the phase and sends the first burst (preamble),
 * rather than dumping everything at once. Subsequent bursts are sent one
 * at a time from ccl_data_handle_tx_conf() as each confirmation arrives. */
dmr_err_t ccl_data_submit_transfer(ccl_data_ctx_t *ctx)
{
    ccl_data_tx_ctx_t *tx = &ctx->tx;
    tx->tx_phase     = CCL_DATA_TX_PHASE_PREAMBLE;
    tx->tx_block_idx = 0u;
    tx->pending_req_id = 0u;
    return ccl_data_tx_next_burst(ctx);
}

/**
 * @brief Common entry point for both unconfirmed and confirmed TX requests.
 */
static dmr_err_t ccl_data_tx_common(ccl_data_ctx_t *ctx,
                                      uint32_t dst_id, bool is_group,
                                      uint8_t sap, bool confirmed,
                                      const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0u || len > CCL_DATA_MAX_PAYLOAD) {
        return DMR_ERR_INVALID_PARAM;
    }

    pthread_mutex_lock(&ctx->state_mutex);

    if (ctx->tx.active) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return DMR_ERR_BUSY;
    }

    ccl_data_tx_ctx_t *tx = &ctx->tx;
    memset(tx, 0, sizeof(*tx));
    tx->confirmed = confirmed;
    tx->is_group  = is_group;
    tx->dst_id    = dst_id;
    tx->sap       = sap;
    memcpy(tx->data, data, len);
    tx->data_len  = len;
    tx->n_blocks  = (uint8_t)((len + CCL_DATA_BLOCK_PAYLOAD - 1u)
                               / CCL_DATA_BLOCK_PAYLOAD);
    tx->pad_octets = (uint8_t)((size_t)tx->n_blocks * CCL_DATA_BLOCK_PAYLOAD - len);
    tx->fsn       = 0u; /* single in-flight transfer; FSN tracking is local */
    tx->retry_count = 0u;
    tx->last_conf_seen = false;

    dmr_err_t err = ccl_data_submit_transfer(ctx);
    if (err != DMR_OK) {
        tx->active = false;
        pthread_mutex_unlock(&ctx->state_mutex);
        return err;
    }

    tx->active = true;
    ctx->state = CCL_DATA_STATE_TX_PENDING;

    pthread_mutex_unlock(&ctx->state_mutex);
    return DMR_OK;
}

dmr_err_t ccl_data_tx_unconfirmed(ccl_data_ctx_t *ctx,
                                    uint32_t dst_id, bool is_group,
                                    uint8_t sap,
                                    const uint8_t *data, size_t len)
{
    return ccl_data_tx_common(ctx, dst_id, is_group, sap, false, data, len);
}

dmr_err_t ccl_data_tx_confirmed(ccl_data_ctx_t *ctx,
                                  uint32_t dst_id, bool is_group,
                                  uint8_t sap,
                                  const uint8_t *data, size_t len)
{
    return ccl_data_tx_common(ctx, dst_id, is_group, sap, true, data, len);
}

/* =========================================================================
 * Short Data (TS 102 361-3 Cl.6) TX entry points
 *
 * Shares ccl_data_tx_common()'s setup shape but needs extra fields (sd_kind
 * plus ports/format/status) and, for STATUS_PRECODED, must allow len==0 —
 * ccl_data_tx_common() rejects that since every IP-bearer transfer needs
 * a real payload. A dedicated setup helper is smaller and safer than
 * bolting five extra optional parameters onto the existing one.
 * ========================================================================= */
static dmr_err_t ccl_data_tx_short_common(ccl_data_ctx_t *ctx,
                                            uint32_t dst_id, bool is_group,
                                            ccl_data_sd_kind_t kind,
                                            bool req_ack,
                                            uint8_t src_port, uint8_t dst_port,
                                            uint8_t dd_format, uint16_t status_value,
                                            const uint8_t *data, size_t len)
{
    bool header_only = (kind == CCL_DATA_SD_KIND_STATUS_PRECODED);

    if (header_only) {
        if (data != NULL || len != 0u) return DMR_ERR_INVALID_PARAM;
    } else if (data == NULL || len == 0u || len > CCL_DATA_MAX_PAYLOAD) {
        return DMR_ERR_INVALID_PARAM;
    }

    pthread_mutex_lock(&ctx->state_mutex);

    if (ctx->tx.active) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return DMR_ERR_BUSY;
    }

    ccl_data_tx_ctx_t *tx = &ctx->tx;
    memset(tx, 0, sizeof(*tx));
    tx->confirmed   = req_ack;
    tx->is_group    = is_group;
    tx->dst_id      = dst_id;
    tx->sap         = DMR_SAP_SHORT_DATA;
    tx->sd_kind     = kind;
    tx->src_port    = src_port;
    tx->dst_port    = dst_port;
    tx->dd_format   = dd_format;
    tx->status_value = status_value;

    if (header_only) {
        tx->data_len   = 0u;
        tx->n_blocks   = 0u;
        tx->pad_octets = 0u;
    } else {
        memcpy(tx->data, data, len);
        tx->data_len   = len;
        tx->n_blocks   = (uint8_t)((len + CCL_DATA_BLOCK_PAYLOAD - 1u)
                                    / CCL_DATA_BLOCK_PAYLOAD);
        tx->pad_octets = (uint8_t)((size_t)tx->n_blocks * CCL_DATA_BLOCK_PAYLOAD - len);
    }
    tx->fsn         = 0u;
    tx->retry_count = 0u;
    tx->last_conf_seen = false;

    dmr_err_t err = ccl_data_submit_transfer(ctx);
    if (err != DMR_OK) {
        tx->active = false;
        pthread_mutex_unlock(&ctx->state_mutex);
        return err;
    }

    tx->active = true;
    ctx->state = CCL_DATA_STATE_TX_PENDING;

    pthread_mutex_unlock(&ctx->state_mutex);
    return DMR_OK;
}

dmr_err_t ccl_data_tx_raw(ccl_data_ctx_t *ctx,
                           uint32_t dst_id, bool is_group,
                           uint8_t src_port, uint8_t dst_port,
                           bool req_ack,
                           const uint8_t *data, size_t len)
{
    return ccl_data_tx_short_common(ctx, dst_id, is_group,
                                     CCL_DATA_SD_KIND_RAW, req_ack,
                                     src_port, dst_port, 0u, 0u, data, len);
}

dmr_err_t ccl_data_tx_defined(ccl_data_ctx_t *ctx,
                               uint32_t dst_id, bool is_group,
                               uint8_t dd_format,
                               bool req_ack,
                               const uint8_t *data, size_t len)
{
    return ccl_data_tx_short_common(ctx, dst_id, is_group,
                                     CCL_DATA_SD_KIND_DEFINED, req_ack,
                                     0u, 0u, dd_format, 0u, data, len);
}

dmr_err_t ccl_data_tx_status_precoded(ccl_data_ctx_t *ctx,
                                       uint32_t dst_id, bool is_group,
                                       uint8_t src_port, uint8_t dst_port,
                                       uint16_t status_value,
                                       bool req_ack)
{
    return ccl_data_tx_short_common(ctx, dst_id, is_group,
                                     CCL_DATA_SD_KIND_STATUS_PRECODED, req_ack,
                                     src_port, dst_port, 0u, status_value,
                                     NULL, 0u);
}

/**
 * @brief Finish the current TX transfer (success or final failure).
 *        Caller must hold state_mutex.
 */
static void ccl_data_tx_finish(ccl_data_ctx_t *ctx, bool success)
{
    ccl_data_tx_ctx_t *tx = &ctx->tx;

    ccl_data_disarm_timer(&ctx->tmr_response);

    if (success) {
        ctx->stats.tx_transfers++;
        ctx->stats.tx_bytes += tx->data_len;
    } else {
        ctx->stats.tx_failures++;
        DMR_LOGI("[CCL-DATA S]DATA Packet retry limit reached.no ack");
    }

    tx->active = false;
    ctx->state = CCL_DATA_STATE_IDLE;

    /* MOD-15 (DCDM) Cl.6.2.2.3.3: send CT_CSBK_Term after this data TX
     * completes. This is the source side only — ccl_data_send_response()
     * (the confirmed-data target's C_RHEAD reply) is untouched, which is
     * exactly the spec's stated exception. channel_activity reflects
     * whether the *other* slot is currently busy (Cl.6.2.3.12: "other
     * TDMA direct mode activity on the frequency"), via MAC's existing
     * CACH AT-bit tracking (mac_slot_is_busy()). */
    if (ctx->dcdm != NULL) {
        dmr_slot_t other_slot = (ctx->slot == DMR_SLOT_1) ? DMR_SLOT_2 : DMR_SLOT_1;
        bool channel_activity = (ctx->mac != NULL) && mac_slot_is_busy(ctx->mac, other_slot);
        dmr_dmo_notify_tx(ctx->dcdm, channel_activity);
    }

    if (ctx->on_tx_complete) {
        ctx->on_tx_complete(ctx, success, tx->retry_count);
    }
}

/**
 * @brief Retransmit the current TX transfer (confirmed ARQ retry).
 *        Caller must hold state_mutex.
 */
static void ccl_data_tx_retry(ccl_data_ctx_t *ctx)
{
    ccl_data_tx_ctx_t *tx = &ctx->tx;

    ccl_data_disarm_timer(&ctx->tmr_response);

    if (tx->retry_count >= CCL_DATA_MAX_RETRIES) {
        ccl_data_tx_finish(ctx, false);
        return;
    }

    tx->retry_count++;
    ctx->stats.tx_retries++;
    tx->last_conf_seen = false;

    /* Reset paced dispatch to start of sequence */
    tx->tx_phase     = CCL_DATA_TX_PHASE_PREAMBLE;
    tx->tx_block_idx = 0u;
    tx->pending_req_id = 0u;

    dmr_err_t err = ccl_data_tx_next_burst(ctx);
    if (err != DMR_OK) {
        ccl_data_tx_finish(ctx, false);
        return;
    }

    ctx->state = CCL_DATA_STATE_TX_BURST_PENDING;
}

/**
 * @brief Begin transmitting a selective retry (Cl.8.2.2.3) using the
 *        block list already built into tx->sack_retry_list[] by
 *        ccl_data_rx_sack_data(). Sends Preamble (sized for just this
 *        retry) → Header (full_msg=false) → only the flagged blocks,
 *        driven by the normal ccl_data_tx_next_burst()/
 *        ccl_data_handle_tx_conf() paced-dispatch loop, same as any
 *        other transfer — sack_retry_active is what makes those
 *        functions take the retry-list path instead of the linear one.
 *
 *        Shares CCL_DATA_MAX_RETRIES/tx->retry_count with the
 *        full-message retry path in ccl_data_tx_retry() — a transfer
 *        that keeps needing SACK cycles is still bounded by the same
 *        overall retry ceiling, not a separate unbounded counter.
 */
static void ccl_data_tx_retry_selective(ccl_data_ctx_t *ctx)
{
    ccl_data_tx_ctx_t *tx = &ctx->tx;

    if (tx->retry_count >= CCL_DATA_MAX_RETRIES) {
        ccl_data_tx_finish(ctx, false);
        return;
    }

    tx->retry_count++;
    ctx->stats.tx_retries++;
    tx->last_conf_seen    = false;
    tx->sack_retry_active = true;

    tx->tx_phase       = CCL_DATA_TX_PHASE_PREAMBLE;
    tx->tx_block_idx   = 0u;
    tx->pending_req_id = 0u;

    dmr_err_t err = ccl_data_tx_next_burst(ctx);
    if (err != DMR_OK) {
        tx->sack_retry_active = false;
        ccl_data_tx_finish(ctx, false);
        return;
    }

    ctx->state = CCL_DATA_STATE_TX_BURST_PENDING;
}

/**
 * @brief Fires on every TX confirmation while an outgoing SACK response
 *        is in flight (rx->sack_tx_active) — sends the next C_RDATA
 *        bitmap burst once the prior burst (Response Header, or the
 *        first C_RDATA burst in the 2-burst case) is actually
 *        confirmed transmitted. Mirrors ccl_data_handle_tx_conf()'s
 *        req_id-matching discipline, but against rx->sack_tx_pending_
 *        req_id — a separate id namespace from ctx->tx.pending_req_id,
 *        since the two sequences (sending our own SACK as receiver,
 *        vs. our own outbound transfer as sender) can genuinely be
 *        concurrent on the same ctx and must not be able to advance
 *        each other. Caller must hold state_mutex. Called from
 *        ccl_data_handle_tx_conf() before that function's own req_id
 *        matching, since ccl_data_send_sack()/this function allocate
 *        req_ids from the same ctx->tx_req_id_next counter as the main
 *        transfer sequence and must claim any confirmation meant for
 *        them first.
 *
 * @return true if this confirmation belonged to an in-flight SACK-send
 *         sequence (whether or not it was the final burst) — the
 *         caller uses this to know the confirmation has been fully
 *         handled and should not also be matched against
 *         ctx->tx.pending_req_id.
 */
static bool ccl_data_handle_sack_tx_conf(ccl_data_ctx_t *ctx,
                                          const dmr_mac_tx_conf_t *conf)
{
    ccl_data_rx_ctx_t *rx = &ctx->rx;

    if (!rx->sack_tx_active || conf->req_id != rx->sack_tx_pending_req_id) {
        return false;
    }

    if (conf->result != DMR_MAC_TX_OK) {
        /* Whichever burst of the sequence failed to transmit — abandon
         * the SACK response rather than send a partial/misleading one
         * (e.g. a Response Header the peer received promising 2
         * C_RDATA bursts, only 1 of which actually goes out). rx is
         * left untouched, same reasoning as the enqueue-failure path
         * in ccl_data_send_sack(). */
        DMR_LOGW("[CCL-DATA S%d]SACK burst (phase %u) failed to transmit — "
                 "abandoning this SACK attempt", ctx->slot, rx->sack_tx_phase);
        rx->sack_tx_active = false;
        return true;
    }

    if (rx->sack_tx_phase >= rx->sack_tx_bursts_expected) {
        /* Confirmation for the final C_RDATA burst — sequence complete. */
        rx->sack_tx_active = false;
        return true;
    }

    /* Send the next C_RDATA burst: phase 0 -> first C_RDATA (covers
     * blocks 0-63), phase 1 -> second C_RDATA (blocks 64-127), matching
     * ccl_data_rx_sack_data()'s own half-indexing on the receiving end. */
    uint8_t half  = rx->sack_tx_phase;
    uint8_t base  = (uint8_t)(half * 64u);
    uint8_t count = (uint8_t)((rx->total_blocks - base > 64u)
                                  ? 64u : (rx->total_blocks - base));

    uint64_t flags = 0u;
    for (uint8_t i = 0u; i < count; i++) {
        if (rx->seen[base + i]) flags |= (1ULL << (63u - i));
    }
    for (uint8_t i = count; i < 64u; i++) {
        flags |= (1ULL << (63u - i));
    }

    dmr_mac_tx_req_t req2;
    memset(&req2, 0, sizeof(req2));
    req2.slot     = ctx->slot;
    req2.priority = DMR_MAC_PRIORITY_NORMAL;
    req2.impolite = false;
    llc_data_block_rdata_build(&req2.burst, flags, ctx->colour_code, ctx->slot);
    req2.req_id          = ctx->tx_req_id_next++;
    req2.originated_from = CCL_TX_ORIGIN_DATA;

    if (mac_tx_enqueue(ctx->mq_mac_tx, &req2) != DMR_OK) {
        DMR_LOGW("[CCL-DATA S%d]SACK C_RDATA burst (half %u) enqueue failed — "
                 "abandoning this SACK attempt", ctx->slot, half);
        rx->sack_tx_active = false;
        return true;
    }

    rx->sack_tx_phase++;
    rx->sack_tx_pending_req_id = req2.req_id;
    return true;
}

/* =========================================================================
 * TX confirmation handling (from mq_mac_conf)
 *
 * This is now the core of the paced TX dispatch state machine. Every
 * confirmation from MAC — for the preamble, header, or any data block —
 * drives exactly one more burst submission, rather than the old design
 * that only checked the final block's req_id and ignored everything else.
 *
 * Phase transitions per successful confirmation:
 *   PREAMBLE confirmed  → advance to HEADER, submit header burst
 *   HEADER confirmed    → advance to BLOCKS, submit block 0
 *   BLOCK_i confirmed   → advance block_idx, submit block i+1
 *   Last BLOCK confirmed → phase=DONE; if unconfirmed → tx_finish(true)
 *                          if confirmed → TX_WAIT_ACK, arm T_Response
 *
 * On MAC TX abort/failure at any phase:
 *   Unconfirmed transfer → tx_finish(false) — no retry
 *   Confirmed transfer   → tx_retry() — restarts from preamble
 * ========================================================================= */
static void ccl_data_handle_tx_conf(ccl_data_ctx_t *ctx,
                                      const dmr_mac_tx_conf_t *conf)
{
    pthread_mutex_lock(&ctx->state_mutex);

    /* An outgoing SACK response (Response Hdr + 1-2 C_RDATA bursts,
     * sent as the receiver) allocates req_ids from the same counter
     * as the main outbound-transfer sequence below, and can be in
     * flight concurrently with one. Give it first claim on any
     * confirmation matching its own pending req_id before falling
     * through to the unrelated tx->pending_req_id check. */
    if (ccl_data_handle_sack_tx_conf(ctx, conf)) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return;
    }

    ccl_data_tx_ctx_t *tx = &ctx->tx;

    /* Only react to the one outstanding burst we're waiting on */
    if (!tx->active || conf->req_id != tx->pending_req_id) {
        pthread_mutex_unlock(&ctx->state_mutex);
        return;
    }

    if (conf->result != DMR_MAC_TX_OK) {
        /* MAC could not transmit — treat as a failure for this transfer */
        if (tx->confirmed) {
            ccl_data_tx_retry(ctx);
        } else {
            ccl_data_tx_finish(ctx, false);
        }
        pthread_mutex_unlock(&ctx->state_mutex);
        return;
    }

    /* --- Successful confirmation: advance phase and send next burst --- */
    switch (tx->tx_phase) {

    case CCL_DATA_TX_PHASE_PREAMBLE:
        tx->tx_phase = CCL_DATA_TX_PHASE_HEADER;
        if (ccl_data_tx_next_burst(ctx) != DMR_OK) {
            ccl_data_tx_finish(ctx, false);
        }
        break;

    case CCL_DATA_TX_PHASE_HEADER:
        if (tx->n_blocks == 0u) {
            /* No data blocks (Status/Precoded always has AB=0; also
             * covers any other 0-length transfer) — the header IS the
             * only burst. Without this branch, falling through to
             * CCL_DATA_TX_PHASE_BLOCKS below would compute
             * tx->n_blocks-1u on a uint8_t, underflowing to 255 and
             * never satisfying tx_block_idx==255, so `last` would
             * never become true and the transfer would never finish. */
            tx->tx_phase = CCL_DATA_TX_PHASE_DONE;
            if (tx->confirmed) {
                tx->last_conf_seen = true;
                ctx->state = CCL_DATA_STATE_TX_WAIT_ACK;
                ccl_data_arm_timer(&ctx->tmr_response, ctx->t_response_ms);
            } else {
                ccl_data_tx_finish(ctx, true);
            }
        } else {
            tx->tx_phase     = CCL_DATA_TX_PHASE_BLOCKS;
            tx->tx_block_idx = 0u;
            if (ccl_data_tx_next_burst(ctx) != DMR_OK) {
                ccl_data_tx_finish(ctx, false);
            }
        }
        break;

    case CCL_DATA_TX_PHASE_BLOCKS: {
        uint8_t seq_len = tx->sack_retry_active ? tx->sack_retry_count : tx->n_blocks;
        bool last = (tx->tx_block_idx == (uint8_t)(seq_len - 1u));
        tx->tx_block_idx++;

        if (!last) {
            /* More blocks to send — submit the next one */
            if (ccl_data_tx_next_burst(ctx) != DMR_OK) {
                ccl_data_tx_finish(ctx, false);
            }
        } else {
            /* Final block of this sequence (full transfer or selective
             * retry) confirmed. Clear sack_retry_active now — the
             * response we're about to wait for is evaluated fresh; if
             * it's another SACK, ccl_data_rx_response() sets it again
             * for the next cycle. */
            tx->sack_retry_active = false;
            tx->tx_phase = CCL_DATA_TX_PHASE_DONE;
            if (tx->confirmed) {
                tx->last_conf_seen = true;
                ctx->state = CCL_DATA_STATE_TX_WAIT_ACK;
                ccl_data_arm_timer(&ctx->tmr_response, ctx->t_response_ms);
            } else {
                ccl_data_tx_finish(ctx, true);
            }
        }
        break;
    }

    case CCL_DATA_TX_PHASE_DONE:
    default:
        /* Stale confirmation after transfer complete — ignore */
        break;
    }

    pthread_mutex_unlock(&ctx->state_mutex);
}

/* =========================================================================
 * T_Response timeout handling
 * ========================================================================= */
static void ccl_data_handle_response_timeout(ccl_data_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);

    if (ctx->tx.active && ctx->tx.confirmed &&
        ctx->state == CCL_DATA_STATE_TX_WAIT_ACK) {
        ccl_data_tx_retry(ctx);
    } else if (ctx->tx.active && ctx->tx.confirmed &&
               ctx->state == CCL_DATA_STATE_TX_WAIT_SACK_DATA) {
        /* Promised C_RDATA bitmap burst(s) never arrived — we have no
         * usable partial bitmap (sack_bursts_received < expected), so
         * there is nothing selective to retry. Fall back to a full
         * retry, same as a plain response timeout. */
        DMR_LOGW("[CCL-DATA S%d]SACK bitmap burst(s) never arrived "
                 "(%u/%u received) — falling back to full retry",
                 ctx->slot, ctx->tx.sack_bursts_received, ctx->tx.sack_bursts_expected);
        ctx->tx.sack_pending = false;
        ccl_data_tx_retry(ctx);
    }

    pthread_mutex_unlock(&ctx->state_mutex);
}

/**
 * @brief Fires when tmr_rx_stall expires while a reassembly is still in
 *        CCL_DATA_STATE_RX_REASSEMBLE — no block has arrived (new or
 *        duplicate) for t_rx_stall_ms. Sized to outlast the sender's
 *        entire retry envelope (see CCL_DATA_T_RX_STALL_MS), so by the
 *        time this fires the peer has most likely already exhausted
 *        its own CCL_DATA_MAX_RETRIES and stopped listening for a
 *        response — sending one here would be to a peer that has
 *        likely moved on, and could land on an unrelated later
 *        transfer if the peer reuses the same address. Abandon locally
 *        and silently instead: free rx's context so a genuinely new
 *        inbound transfer isn't blocked waiting for this one to yield
 *        the single-outstanding-transfer slot this module keeps
 *        (see the module's own header comment on that design).
 */
static void ccl_data_handle_rx_stall_timeout(ccl_data_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);

    if (ctx->rx.active && ctx->state == CCL_DATA_STATE_RX_REASSEMBLE) {
        DMR_LOGW("[CCL-DATA S%d]RX reassembly stalled (no block for %ums) — "
                 "abandoning transfer from 0x%06X (%u/%u blocks received)",
                 ctx->slot, ctx->t_rx_stall_ms, ctx->rx.src_id,
                 ctx->rx.blocks_received, ctx->rx.total_blocks);
        memset(&ctx->rx, 0, sizeof(ctx->rx));
        ctx->state = CCL_DATA_STATE_IDLE;
    }

    pthread_mutex_unlock(&ctx->state_mutex);
}

/* =========================================================================
 * RX side
 * ========================================================================= */

/**
 * @brief Build and submit a Response Header for an inbound confirmed
 *        transfer, with an explicit Class/Type/Status per TS 102 361-1
 *        Table 8.3. Caller must hold state_mutex.
 *
 * Sent exactly once per transfer (see ccl_data_rx_block() / the
 * reassembly completion logic below) — never per intermediate block.
 */
static void ccl_data_send_response(ccl_data_ctx_t *ctx, uint32_t peer_id,
                                     uint8_t sap,
                                     uint8_t class_v, uint8_t type_v,
                                     uint8_t status_v)
{
    dmr_mac_tx_req_t req;
    memset(&req, 0, sizeof(req));
    req.slot     = ctx->slot;
    req.priority = DMR_MAC_PRIORITY_NORMAL;
    req.impolite = false;

    llc_data_hdr_resp_build(&req.burst, peer_id, ctx->my_radio_id, sap,
                             class_v, type_v, status_v,
                             0u, /* blocks to follow */
                             ctx->colour_code, ctx->slot);
    req.req_id = ctx->tx_req_id_next++;
    req.originated_from=CCL_TX_ORIGIN_DATA;
    bool ack = (class_v == CCL_DATA_RESP_CLASS_ACK);
    if (mac_tx_enqueue(ctx->mq_mac_tx, &req) == DMR_OK) {
        if (ack) ctx->stats.rx_acks_sent++;
        else     ctx->stats.rx_nacks_sent++;
    }
}

/**
 * @brief Build and send a SACK response (Cl.8.2.2.3): C_RHEAD naming
 *        Class=SACK, followed by 1 or 2 C_RDATA bursts carrying the
 *        selective-retry bitmap built from `seen[]` (bit=1 means
 *        received OK/no retry needed — the inverse of "seen"==missing).
 *        total_blocks > 64 needs two C_RDATA bursts (Cl.8.2.2.3: up to
 *        64 blocks per burst, up to 127 with two).
 */
/**
 * @brief Start sending a SACK response: builds and enqueues only the
 *        Response Header (Cl.8.2.2.3), then arms rx->sack_tx_* state so
 *        ccl_data_handle_sack_tx_conf() sends the 1-2 C_RDATA bitmap
 *        burst(s) one at a time as each prior burst's TX confirmation
 *        actually arrives — mirroring ctx->tx's own paced PREAMBLE/
 *        HEADER/BLOCKS dispatch (ccl_data_tx_next_burst()/
 *        ccl_data_handle_tx_conf()) rather than enqueuing all 2-3
 *        bursts back to back in one call, which is what this function
 *        did before and is not correct: mac_tx_enqueue() only queues a
 *        burst for MAC to send — it does not mean the burst has been
 *        transmitted, so nothing here may assume burst N+1 is safe to
 *        submit until burst N's confirmation is actually seen.
 *
 *        Caller must hold state_mutex (same contract as
 *        ccl_data_rx_block(), the sole caller).
 */
static void ccl_data_send_sack(ccl_data_ctx_t *ctx, uint32_t peer_id, uint8_t sap,
                                const bool *seen, uint8_t total_blocks)
{
    (void)seen; /* read directly from ctx->rx.seen[] by the C_RDATA-building
                   step below, not copied here — rx stays alive for the
                   whole SACK-send sequence, unlike a one-shot builder that
                   would need its own snapshot */
    ccl_data_rx_ctx_t *rx = &ctx->rx;

    rx->sack_tx_bursts_expected = (total_blocks > 64u) ? 2u : 1u;
    rx->sack_tx_phase           = 0u;
    rx->sack_tx_peer_id         = peer_id;
    rx->sack_tx_sap             = sap;
    rx->sack_tx_active          = true;

    dmr_mac_tx_req_t req;
    memset(&req, 0, sizeof(req));
    req.slot     = ctx->slot;
    req.priority = DMR_MAC_PRIORITY_NORMAL;
    req.impolite = false;
    llc_data_hdr_resp_build(&req.burst, peer_id, ctx->my_radio_id, sap,
                             CCL_DATA_RESP_CLASS_SACK, CCL_DATA_RESP_TYPE_ACK, 0u,
                             rx->sack_tx_bursts_expected, ctx->colour_code, ctx->slot);
    req.req_id           = ctx->tx_req_id_next++;
    req.originated_from  = CCL_TX_ORIGIN_DATA;

    if (mac_tx_enqueue(ctx->mq_mac_tx, &req) != DMR_OK) {
        /* Could not even queue the Response Header — abandon the SACK
         * attempt rather than leave sack_tx_active set with nothing
         * actually in flight to eventually confirm it forward. The
         * peer will not receive a response at all for this cycle;
         * ctx->rx itself is left untouched (still mid-reassembly) so
         * a subsequent block arrival can still trigger a fresh SACK
         * attempt through the normal completion check. */
        DMR_LOGW("[CCL-DATA S%d]SACK Response Hdr enqueue failed — abandoning "
                 "this SACK attempt", ctx->slot);
        rx->sack_tx_active = false;
        return;
    }

    rx->sack_tx_pending_req_id = req.req_id;
    ctx->stats.rx_nacks_sent++; /* SACK counted alongside NACK for stats purposes */
}

/**
 * @brief Fires on every TX confirmation while an outgoing SACK response
 *        is in flight (rx->sack_tx_active) — sends the next C_RDATA
 *        bitmap burst once the prior burst (Response Header, or the
 *        first C_RDATA burst in the 2-burst case) is actually
 *        confirmed transmitted. Mirrors ccl_data_handle_tx_conf()'s
 *        req_id-matching discipline, but against rx->sack_tx_pending_
 *        req_id — a separate id namespace from ctx->tx.pending_req_id,
 *        since the two sequences (sending our own SACK as receiver,
 *        vs. our own outbound transfer as sender) can genuinely be
 *        concurrent on the same ctx and must not be able to advance
 *        each other. Caller must hold state_mutex.
 *
 * @return true if this confirmation belonged to an in-flight SACK-send
 *         sequence (whether or not it was the final burst) — the
 *         caller (ccl_data_handle_tx_conf()) uses this to know the
 *         confirmation has been fully handled and should not also be
 *         matched against ctx->tx.pending_req_id.
 */
/**
 * @brief Start a new inbound reassembly from a Data Header dispatch result.
 *        Caller must hold state_mutex.
 */
static void ccl_data_rx_start(ccl_data_ctx_t *ctx, const llc_rx_result_t *res)
{
    ccl_data_rx_ctx_t *rx = &ctx->rx;

    if (!res->full_msg && rx->active && rx->confirmed &&
        rx->src_id == res->src_id && rx->dst_id == res->dst_id &&
        rx->sap == res->sap) {
        /* Selective-retry continuation header (F=0, Cl.8.2.2.3/TS
         * 102 361-3 Cl.5.4.3-6.5) for the transfer already in
         * progress — do NOT reset rx. The blocks that follow this
         * header are the specific ones we SACK'd; rx->seen[]/
         * rx->buffer must survive so they land in the right slots
         * and ccl_data_rx_block()'s completion check can re-evaluate
         * blocks_received against total_blocks once they arrive.
         * total_blocks/total_len/sap/ids are already correct from
         * the original header, so there is nothing to update here —
         * just let the transfer stay active and fall through to
         * accept the blocks that follow. */
        DMR_LOGI("[CCL-DATA S%d]Partial-retry header (F=0) for in-progress "
                 "transfer from 0x%06X — continuing existing reassembly",
                 ctx->slot, res->src_id);
        ccl_data_arm_timer(&ctx->tmr_rx_stall, ctx->t_rx_stall_ms);
        return;
    }

    memset(rx, 0, sizeof(*rx));

    rx->confirmed = (res->opcode == DMR_DPFT_CONFIRMED);
    rx->is_group  = (res->svc != 0u);
    rx->sap       = res->sap;
    rx->dst_id    = res->dst_id;
    rx->src_id    = res->src_id;
    rx->total_blocks = res->blocks_to_follow;

    /* Pad Octet Count (POC): body[0] bit4 (MSB) | body[1] bits[3:0] */
    uint8_t poc = (uint8_t)(((res->body[0] >> 4) & 0x01u) << 4)
                | (uint8_t)(res->body[1] & 0x0Fu);

    size_t total = (size_t)rx->total_blocks * CCL_DATA_BLOCK_PAYLOAD;
    if (poc <= total) {
        total -= poc;
    }
    if (total > CCL_DATA_MAX_PAYLOAD) {
        /* Header claims more payload than our reassembly buffer can
         * hold — TS 102 361-1 Table 8.3 Class=01 Type=010 "Memory of
         * the recipient is full". Record the failure now; the single
         * end-of-transfer response is still sent only once blocks stop
         * arriving (see ccl_data_rx_block()), consistent with every
         * other failure path. Clamp total_len so subsequent block
         * writes stay in-bounds. */
        total = CCL_DATA_MAX_PAYLOAD;
        rx->failed     = true;
        rx->fail_type   = CCL_DATA_RESP_TYPE_NACK_MEMORY_FULL;
        rx->fail_status = 0u;
    }
    rx->total_len = total;

    if (rx->total_blocks == 0u) {
        /* Degenerate zero-block transfer — deliver immediately. */
        if (ctx->on_data_received) {
            ctx->on_data_received(ctx, rx->src_id, rx->dst_id, rx->is_group,
                                   rx->sap, rx->buffer, 0u);
        }
        ctx->stats.rx_transfers++;
        if (rx->confirmed) {
            if (rx->failed) {
                ccl_data_send_response(ctx, rx->src_id, rx->sap,
                                        CCL_DATA_RESP_CLASS_NACK,
                                        rx->fail_type, rx->fail_status);
            } else {
                ccl_data_send_response(ctx, rx->src_id, rx->sap,
                                        CCL_DATA_RESP_CLASS_ACK,
                                        CCL_DATA_RESP_TYPE_ACK, 0u);
            }
        }
        memset(rx, 0, sizeof(*rx));
        ctx->state = CCL_DATA_STATE_IDLE;
        return;
    }

    ccl_data_arm_timer(&ctx->tmr_rx_stall, ctx->t_rx_stall_ms);
    rx->active = true;
    ctx->state = CCL_DATA_STATE_RX_REASSEMBLE;
}

/**
 * @brief Start (or immediately complete, for Status/Precoded) reception
 *        of a Short Data message (DD_HEAD/R_HEAD/SP_HEAD). Mirrors
 *        ccl_data_rx_start()'s shape but for the split-AB header family
 *        (see dmr_pdu.h struct comments) — different tail-field byte
 *        positions, no POC field, and a different completion callback.
 *
 * Known limitation: unlike U_HEAD/C_HEAD, R_HEAD/DD_HEAD carry no Pad
 * Octet Count field, so trailing padding in the final block cannot be
 * trimmed here — total_len is exactly total_blocks*24 bytes; the
 * application must handle any trailing padding itself (e.g. via its
 * own length-prefixed framing) until this is revisited.
 */
static void ccl_data_rx_short_start(ccl_data_ctx_t *ctx, const llc_rx_result_t *res)
{
    ccl_data_rx_ctx_t *rx = &ctx->rx;

    bool is_block_carrying = (res->opcode == DMR_DPFT_DEFINED_DATA) ||
        (res->opcode == DMR_DPFT_RAW_OR_STATUS && res->blocks_to_follow > 0u);
    /* SARQ/selective-retry only applies to the block-carrying short-data
     * kinds (Defined/Raw) — Status/Precoded (blocks_to_follow==0) is a
     * single header burst with no blocks to retry, so it can never
     * have a genuine continuation and is deliberately excluded here. */
    if (is_block_carrying && !res->full_msg && rx->active && rx->confirmed &&
        rx->src_id == res->src_id && rx->dst_id == res->dst_id &&
        rx->sap == res->sap) {
        /* Same continuation-preservation logic as ccl_data_rx_start()'s
         * IP-family guard — see that function's comment for the full
         * rationale. Duplicated rather than factored out since the two
         * functions' surrounding memset/field-population shapes differ
         * enough that a shared helper would need to take most of both
         * functions' state as parameters anyway. */
        DMR_LOGI("[CCL-DATA S%d]Partial-retry header (F=0) for in-progress "
                 "short-data transfer from 0x%06X — continuing existing reassembly",
                 ctx->slot, res->src_id);
        ccl_data_arm_timer(&ctx->tmr_rx_stall, ctx->t_rx_stall_ms);
        return;
    }

    memset(rx, 0, sizeof(*rx));

    rx->confirmed = ((res->body[0] >> 6) & 0x01u) != 0u; /* A-bit, same byte0 position as IP family */
    rx->is_group  = (res->svc != 0u);
    rx->sap       = res->sap;
    rx->dst_id    = res->dst_id;
    rx->src_id    = res->src_id;
    rx->total_blocks = res->blocks_to_follow;

    if (res->opcode == DMR_DPFT_DEFINED_DATA) {
        rx->sd_kind   = CCL_DATA_SD_KIND_DEFINED;
        rx->dd_format = (uint8_t)((res->body[8] >> 2) & 0x3Fu);
    } else if (rx->total_blocks == 0u) {
        /* DPFT==DMR_DPFT_RAW_OR_STATUS with AB==0 is SP_HEAD (Cl.6.3:
         * "AB shall be set to 0" is exactly how Status/Precoded is
         * distinguished from Raw Data on RX). */
        rx->sd_kind      = CCL_DATA_SD_KIND_STATUS_PRECODED;
        rx->src_port     = (uint8_t)((res->body[8] >> 5) & 0x07u);
        rx->dst_port     = (uint8_t)((res->body[8] >> 2) & 0x07u);
        rx->status_value = (uint16_t)(((res->body[8] & 0x03u) << 8) | res->body[9]);
    } else {
        rx->sd_kind  = CCL_DATA_SD_KIND_RAW;
        rx->src_port = (uint8_t)((res->body[8] >> 5) & 0x07u);
        rx->dst_port = (uint8_t)((res->body[8] >> 2) & 0x07u);
    }

    rx->total_len = (size_t)rx->total_blocks * CCL_DATA_BLOCK_PAYLOAD;
    if (rx->total_len > CCL_DATA_MAX_PAYLOAD) {
        rx->total_len  = CCL_DATA_MAX_PAYLOAD;
        rx->failed     = true;
        rx->fail_type   = CCL_DATA_RESP_TYPE_NACK_MEMORY_FULL;
        rx->fail_status = 0u;
    }

    if (rx->total_blocks == 0u) {
        /* Status/Precoded — the header IS the entire message; deliver now. */
        if (ctx->on_short_data_received) {
            ctx->on_short_data_received(ctx, rx->sd_kind, rx->src_id, rx->dst_id,
                                          rx->is_group, rx->src_port, rx->dst_port,
                                          rx->dd_format, rx->status_value, NULL, 0u);
        }
        ctx->stats.rx_transfers++;
        if (rx->confirmed) {
            if (rx->failed) {
                ccl_data_send_response(ctx, rx->src_id, rx->sap,
                                        CCL_DATA_RESP_CLASS_NACK,
                                        rx->fail_type, rx->fail_status);
            } else {
                ccl_data_send_response(ctx, rx->src_id, rx->sap,
                                        CCL_DATA_RESP_CLASS_ACK,
                                        CCL_DATA_RESP_TYPE_ACK, 0u);
            }
        }
        memset(rx, 0, sizeof(*rx));
        ctx->state = CCL_DATA_STATE_IDLE;
        return;
    }

    ccl_data_arm_timer(&ctx->tmr_rx_stall, ctx->t_rx_stall_ms);
    rx->active = true;
    ctx->state = CCL_DATA_STATE_RX_REASSEMBLE;
}

/**
 * @brief Accumulate one Rate-1 Data Block into the active reassembly.
 *        Caller must hold state_mutex.
 *
 * Failure detection per TS 102 361-1 Table 8.3 — recorded as soon as
 * encountered, but the actual NACK is only sent once, when the
 * transfer concludes (see the bottom of this function), never per
 * intermediate block. Once a failure is recorded for this transfer,
 * later blocks no longer overwrite the recorded reason (first failure
 * wins) — they still continue to be accumulated/counted so blocks-
 * received bookkeeping and the eventual single response stay correct
 * even if the peer keeps sending after the point of failure.
 */
static void ccl_data_rx_block(ccl_data_ctx_t *ctx, const llc_rx_result_t *res)
{
    ccl_data_rx_ctx_t *rx = &ctx->rx;

    if (!rx->active) {
        return; /* Block arrived with no active header — discard */
    }

    uint8_t dbsn;
    bool    last_block;
    uint8_t payload[CCL_DATA_BLOCK_PAYLOAD];
    size_t  payload_len = 0u;

    if (llc_data_block_rate1_parse(res->body, &dbsn, &last_block,
                                    payload, &payload_len) != DMR_OK) {
        /* Block CRC failed — Table 8.3 Class=01 Type=001 "Packet CRC
         * of a packet with NI failed". We cannot trust dbsn/last_block
         * from a CRC-failed block, so there is nothing further to do
         * with this particular block — but the transfer is not over
         * (more blocks may still follow), so just record the failure
         * and wait for the peer's retry/continuation or for the
         * transfer to otherwise conclude. */
        if (!rx->failed) {
            rx->failed     = true;
            rx->fail_type   = CCL_DATA_RESP_TYPE_NACK_CRC;
            rx->fail_status = 0u;
        }
        return;
    }

    if (dbsn >= CCL_DATA_MAX_BLOCKS || dbsn >= rx->total_blocks) {
        /* Out-of-range DBSN — cannot correspond to this header's
         * declared block count. Table 8.3 Class=01 Type=011 "The
         * received FSN is out of sequence". */
        if (!rx->failed) {
            rx->failed     = true;
            rx->fail_type   = CCL_DATA_RESP_TYPE_NACK_OUT_OF_SEQ_FSN;
            rx->fail_status = dbsn;
        }
    } else if (rx->seen[dbsn]) {
        /* Duplicate block (peer retransmitted) — not itself a failure;
         * ETSI's selective ARQ model expects this on retry. Re-copy
         * the payload (harmless, same data) but do not double-count
         * toward blocks_received. */
        size_t off = (size_t)dbsn * CCL_DATA_BLOCK_PAYLOAD;
        if (off < rx->total_len) {
            size_t avail = rx->total_len - off;
            size_t copy_len = (payload_len < avail) ? payload_len : avail;
            memcpy(&rx->buffer[off], payload, copy_len);
        }
    } else {
        rx->seen[dbsn] = true;
        size_t off = (size_t)dbsn * CCL_DATA_BLOCK_PAYLOAD;
        if (off < rx->total_len) {
            size_t avail = rx->total_len - off;
            size_t copy_len = (payload_len < avail) ? payload_len : avail;
            memcpy(&rx->buffer[off], payload, copy_len);
        }
        rx->blocks_received++;
        ctx->stats.rx_blocks++;
    }
    ccl_data_arm_timer(&ctx->tmr_rx_stall, ctx->t_rx_stall_ms);
    DMR_LOGI("[CCL-DATA S]DATA Packet content: %s (dbsn=%d payload len=%d)",
             payload, dbsn, payload_len);

    if (last_block || rx->blocks_received >= rx->total_blocks) {
        bool transfer_complete = (rx->blocks_received >= rx->total_blocks) && !rx->failed;

        if (!transfer_complete && !rx->failed && rx->confirmed) {
            /* last_block arrived but blocks are still missing, and
             * nothing hard-failed (no CRC/out-of-seq/buffer error) —
             * this is exactly the gap-from-lost-blocks case Cl.8.2.2.3
             * selective ARQ exists for. Do NOT deliver the (incomplete)
             * buffer to the application and do NOT reset rx — the
             * retry's blocks need somewhere to land. Send SACK instead
             * of ACK/NACK and wait for the peer's selective retry to
             * fill rx->seen[]'s remaining gaps; this same completion
             * check runs again on each subsequent block, so a later
             * retry naturally re-evaluates and completes normally once
             * blocks_received catches up to total_blocks. */
            ccl_data_send_sack(ctx, rx->src_id, rx->sap, rx->seen, rx->total_blocks);
            return;
        }

        if (transfer_complete) {
            if (rx->sd_kind == CCL_DATA_SD_KIND_IP) {
                if (ctx->on_data_received) {
                    ctx->on_data_received(ctx, rx->src_id, rx->dst_id, rx->is_group,
                                           rx->sap, rx->buffer, rx->total_len);
                }
            } else if (ctx->on_short_data_received) {
                ctx->on_short_data_received(ctx, rx->sd_kind, rx->src_id, rx->dst_id,
                                              rx->is_group, rx->src_port, rx->dst_port,
                                              rx->dd_format, rx->status_value,
                                              rx->buffer, rx->total_len);
            }
            ctx->stats.rx_transfers++;
            ctx->stats.rx_bytes += rx->total_len;
        }

        if (rx->confirmed) {
            /* Exactly one response for the whole transfer, here. Either
             * a genuine completion (ACK) or a hard, block-unspecific
             * failure not eligible for selective retry (NACK) — the
             * SACK-eligible case already returned above without
             * reaching here. */
            if (rx->failed) {
                ccl_data_send_response(ctx, rx->src_id, rx->sap,
                                        CCL_DATA_RESP_CLASS_NACK,
                                        rx->fail_type, rx->fail_status);
            } else {
                ccl_data_send_response(ctx, rx->src_id, rx->sap,
                                        CCL_DATA_RESP_CLASS_ACK,
                                        CCL_DATA_RESP_TYPE_ACK, 0u);
            }
        }

        ccl_data_disarm_timer(&ctx->tmr_rx_stall);
        memset(rx, 0, sizeof(*rx));
        ctx->state = CCL_DATA_STATE_IDLE;
    }
}

/**
 * @brief Handle a Response Header (ACK/NACK) addressed to us — i.e. a
 *        reply to our own outstanding confirmed TX. Caller must hold
 *        state_mutex.
 */
static void ccl_data_rx_response(ccl_data_ctx_t *ctx, const llc_rx_result_t *res)
{
    if (!ctx->tx.active || !ctx->tx.confirmed ||
        ctx->state != CCL_DATA_STATE_TX_WAIT_ACK) {
        return;
    }
    if (res->dst_id != ctx->my_radio_id) {
        return; /* Response not addressed to us */
    }

    uint8_t class_v = (uint8_t)((res->body[9] >> 6) & 0x03u);

    if (class_v == CCL_DATA_RESP_CLASS_ACK) {
        DMR_LOGI("[CCL-DATA S%d]ack For Data Received",
             ctx->slot );
        ccl_data_tx_finish(ctx, true);
    } else if (class_v == CCL_DATA_RESP_CLASS_SACK) {
        /* Cl.8.2.2.3: 1 or 2 C_RDATA bitmap bursts follow this header,
         * carried in blocks_to_follow — same field position/meaning as
         * ccl_data_send_sack()'s own blocks_to_follow on the send side.
         * Don't retry yet; wait for those burst(s) to actually arrive
         * (ccl_data_rx_sack_data()) so the retry only resends blocks
         * the peer genuinely still needs. Re-arm (not disarm) the
         * response timer as a bitmap-arrival watchdog — if the promised
         * C_RDATA burst(s) never show up, ccl_data_handle_response_
         * timeout() falls back to a full retry rather than hanging
         * forever in CCL_DATA_STATE_TX_WAIT_SACK_DATA. */
        ccl_data_tx_ctx_t *tx = &ctx->tx;
        tx->sack_pending          = true;
        tx->sack_bursts_expected  = (res->blocks_to_follow > 0u) ? res->blocks_to_follow : 1u;
        tx->sack_bursts_received  = 0u;
        tx->sack_retry_flags[0]   = 0u;
        tx->sack_retry_flags[1]   = 0u;
        ctx->state = CCL_DATA_STATE_TX_WAIT_SACK_DATA;
        ccl_data_arm_timer(&ctx->tmr_response, ctx->t_response_ms);
        DMR_LOGI("[CCL-DATA S%d]SACK For Data Received, awaiting %u bitmap burst(s)",
             ctx->slot, tx->sack_bursts_expected);
    } else {
        /* NACK — retry (or fail if retries exhausted) */
        DMR_LOGI("[CCL-DATA S%d]NACK For Data Received retry",
             ctx->slot );
        ccl_data_tx_retry(ctx);
    }
}

/**
 * @brief Accumulate one C_RDATA bitmap burst (Cl.8.2.2.3) into
 *        tx->sack_retry_flags[]. Once as many bursts have arrived as
 *        the Response Header promised (tx->sack_bursts_expected),
 *        builds tx->sack_retry_list[] from the accumulated bitmap(s)
 *        and starts the selective retransmission.
 *
 *        Ordering assumption: the two C_RDATA bursts (when there are
 *        two) are consumed in arrival order, first-received filling
 *        flags[0] (blocks 0-63), second filling flags[1] (blocks
 *        64-127). The spec does not appear to number these bursts
 *        explicitly for reordering, so out-of-order delivery between
 *        the pair is not handled here — consistent with this
 *        module's existing single-outstanding-transfer, no-reorder
 *        design (see the file's own header comment).
 */
static void ccl_data_rx_sack_data(ccl_data_ctx_t *ctx, const llc_rx_result_t *res)
{
    ccl_data_tx_ctx_t *tx = &ctx->tx;
    if (!tx->sack_pending || tx->sack_bursts_received >= tx->sack_bursts_expected) {
        return; /* Not expecting this burst (stray/duplicate) — ignore */
    }

    uint64_t flags = 0u;
    llc_data_block_rdata_parse(res->body, &flags);
    tx->sack_retry_flags[tx->sack_bursts_received] = flags;
    tx->sack_bursts_received++;

    if (tx->sack_bursts_received < tx->sack_bursts_expected) {
        return; /* Still waiting on the second bitmap burst */
    }

    /* Both (or the one) expected bursts are in — build the retry list:
     * every block index in [0, n_blocks) whose bit is 0 needs resending.
     * Block i's bit lives in flags[i/64], position (63 - i%64) — MSB-
     * first (block 0 is the top bit of the 64-bit word), matching both
     * llc_data_block_rdata_build()'s byte packing and
     * ccl_data_send_sack()'s sender-side bit-setting, which the DMR
     * field convention requires be MSB-first throughout. */
    tx->sack_retry_count = 0u;
    for (uint8_t i = 0u; i < tx->n_blocks && i < CCL_DATA_MAX_BLOCKS; i++) {
        uint8_t half = i / 64u;
        uint8_t bit  = (uint8_t)(63u - (i % 64u));
        bool block_ok = ((tx->sack_retry_flags[half] >> bit) & 1u) != 0u;
        if (!block_ok) {
            tx->sack_retry_list[tx->sack_retry_count++] = i;
        }
    }

    tx->sack_pending = false;

    if (tx->sack_retry_count == 0u) {
        /* Every block the peer flagged was actually fine — nothing to
         * resend. Treat as success; do not send a third response of
         * our own (the peer already told us what it saw). */
        DMR_LOGI("[CCL-DATA S%d]SACK bitmap(s) complete — no blocks flagged, "
                 "treating as success", ctx->slot);
        ccl_data_tx_finish(ctx, true);
        return;
    }

    DMR_LOGI("[CCL-DATA S%d]SACK bitmap(s) complete — %u block(s) need selective retry",
             ctx->slot, tx->sack_retry_count);
    ccl_data_tx_retry_selective(ctx);
}

dmr_err_t ccl_data_rx_burst(ccl_data_ctx_t *ctx, const dmr_burst_t *burst)
{
    llc_rx_result_t res;
    dmr_err_t err = llc_rx_dispatch(burst, &res);
    if (err != DMR_OK && err != DMR_ERR_CRC && err != DMR_ERR_FEC) {
        return err;
    }
    if (!res.crc_ok && res.type != LLC_RX_DATA_BLOCK) {
        /* Header CRC failed — cannot trust SAP/blocks/IDs; drop. */
        return DMR_OK;
    }

    pthread_mutex_lock(&ctx->state_mutex);

    switch (res.type) {
    case LLC_RX_DATA_HEADER:
        if (res.opcode == DMR_DPFT_RESPONSE) {
            ccl_data_rx_response(ctx, &res);
        } else if (res.opcode == DMR_DPFT_UNCONFIRMED ||
                   res.opcode == DMR_DPFT_CONFIRMED) {
            ccl_data_rx_start(ctx, &res);
        } else if (res.opcode == DMR_DPFT_DEFINED_DATA ||
                   res.opcode == DMR_DPFT_RAW_OR_STATUS) {
            ccl_data_rx_short_start(ctx, &res);
        }
        /* Other DPFTs (UDT, Proprietary) — deferred to future work */
        break;

    case LLC_RX_DATA_BLOCK:
        if (ctx->tx.sack_pending && ctx->state == CCL_DATA_STATE_TX_WAIT_SACK_DATA) {
            /* C_RDATA bitmap burst, not inbound reassembly payload —
             * see the DMR_DTYPE_RATE12_DATA case in llc_rx_dispatch(). */
            ccl_data_rx_sack_data(ctx, &res);
        } else {
            ccl_data_rx_block(ctx, &res);
        }
        break;

    case LLC_RX_UNKNOWN:
        /* llc_rx_dispatch() could not decode this burst at all (FEC
         * uncorrectable, or an unrecognised Data Type) — Table 8.3
         * Class=01 Type=001 "Packet CRC of a packet with NI failed" is
         * the closest applicable reason: we received something for
         * this slot but cannot trust its contents. We have no DBSN to
         * attribute this to, so we cannot tell whether it belonged to
         * our active reassembly — but if one is active, record the
         * failure so the eventual single end-of-transfer response
         * reflects it rather than silently claiming success. If no
         * reassembly is active, there is nothing to attribute this to;
         * drop it, consistent with prior behaviour. */
        if (ctx->rx.active && !ctx->rx.failed) {
            ctx->rx.failed     = true;
            ctx->rx.fail_type   = CCL_DATA_RESP_TYPE_NACK_CRC;
            ctx->rx.fail_status = 0u;
        }
        break;

    default:
        break;
    }

    pthread_mutex_unlock(&ctx->state_mutex);
    return DMR_OK;
}

/* =========================================================================
 * Worker thread
 * ========================================================================= */
static void *ccl_data_thread(void *arg)
{
    ccl_data_ctx_t *ctx = (ccl_data_ctx_t *)arg;
    struct epoll_event events[CCL_DATA_EPOLL_MAX_EVENTS];

    DMR_LOGI("[CCL-DATA S%d] Worker thread started (radio_id=0x%06X cc=%u)",
             ctx->slot, ctx->my_radio_id, ctx->colour_code);

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        DMR_LOGE("[CCL-DATA S%d] epoll_create1 failed: %s",
                 ctx->slot, strerror(errno));
        return NULL;
    }

    struct epoll_event ev;
    ev.events = EPOLLIN;

    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_response);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_response), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_rx_stall);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_rx_stall), &ev);
    ev.data.fd = (int)ctx->mq_evt;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_evt, &ev);
    ev.data.fd = (int)ctx->mq_mac_conf;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_mac_conf, &ev);
    ev.data.fd = (int)ctx->mq_mac_rx;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_mac_rx, &ev);

    while (ctx->running) {
        int nev = epoll_wait(epfd, events, CCL_DATA_EPOLL_MAX_EVENTS, 200);
        if (nev < 0) {
            if (errno == EINTR) continue;
            DMR_LOGE("[CCL-DATA S%d] epoll_wait: %s", ctx->slot, strerror(errno));
            break;
        }

        for (int i = 0; i < nev; i++) {
            int fd = events[i].data.fd;

            if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_response)) {
                ccl_data_timer_drain(&ctx->tmr_response);
                ccl_data_handle_response_timeout(ctx);
            } else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_rx_stall)) {
                ccl_data_timer_drain(&ctx->tmr_rx_stall);
                ccl_data_handle_rx_stall_timeout(ctx);
            } else if (fd == (int)ctx->mq_evt) {
                ccl_data_event_t e;
                while (mq_receive(ctx->mq_evt, (char *)&e, sizeof(e), NULL) >= 0) {
                    if (e.type == CCL_DATA_EVT_SHUTDOWN) {
                        ctx->running = false;
                    }
                }
            } else if (fd == (int)ctx->mq_mac_conf) {
                dmr_mac_tx_conf_t conf;
                while (mq_receive(ctx->mq_mac_conf, (char *)&conf,
                                   sizeof(conf), NULL) >= 0) {
                    ccl_data_handle_tx_conf(ctx, &conf);
                }
            } else if (fd == (int)ctx->mq_mac_rx) {
                dmr_burst_t burst;
                while (mq_receive(ctx->mq_mac_rx, (char *)&burst,
                                   sizeof(burst), NULL) >= 0) {
                    ccl_data_rx_burst(ctx, &burst);
                }
            }
        }
    }

    close(epfd);
    DMR_LOGI("[CCL-DATA S%d] Worker thread exiting", ctx->slot);
    return NULL;
}

/* =========================================================================
 * Lifecycle
 * ========================================================================= */
 
 
 /* For mq_mac_tx / mq_mac_conf / mq_mac_rx — queues MAC creates (see the
 * ownership contract in dmr_mac.h). CCL must NEVER pass O_CREAT here:
 * doing so would let CCL race MAC to create the queue with CCL's own
 * (possibly different) mq_attr, silently discarding whichever attr
 * loses the race with no error from mq_open(). Instead, if the queue
 * does not exist yet (MAC hasn't finished mac_init()), retry with a
 * short bounded backoff rather than failing immediately — this tolerates
 * ordinary process/thread startup scheduling without masking a genuine
 * "MAC was never started" misconfiguration, which still fails once the
 * retry budget is exhausted. */
static inline mqd_t ccl_mq_open_mac_owned(const char *name, int oflags)
{
    mqd_t mq;
    int   attempts = 0;

    do {
        mq = mq_open(name, O_NONBLOCK | oflags);
        if (mq != (mqd_t)-1) {
            return mq;
        }
        if (errno != ENOENT) {
            /* Some other failure (e.g. EACCES) — no point retrying */
            break;
        }
        attempts++;
        if (attempts < DMR_MQ_OPEN_RETRY_COUNT) {
            struct timespec ts;
            ts.tv_sec  = 0;
            ts.tv_nsec = (long)DMR_MQ_OPEN_RETRY_DELAY_MS * 1000000L;
            nanosleep(&ts, NULL);
        }
    } while (attempts < DMR_MQ_OPEN_RETRY_COUNT);

    DMR_LOGE("mq_open(%s) failed after %d retries: %s — was mac_init() "
             "called for this slot before this CCL module's init()?",
             name, attempts, strerror(errno));
    return (mqd_t)-1;
}   

/* For mq_evt — the one queue CCL Voice itself creates and owns. Safe to
 * use O_CREAT here because nothing else in the system ever opens this
 * name; there is no second-creator ambiguity. */
static inline mqd_t ccl_mq_create_own(const char *name, int max_msgs,
                                 size_t msg_size, int extra_flags)
{
    struct mq_attr attr;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = max_msgs;
    attr.mq_msgsize = (long)msg_size;
    attr.mq_curmsgs = 0;

    mqd_t mq = mq_open(name,
                        O_CREAT | O_NONBLOCK | extra_flags,
                        0600, &attr);
    if (mq == (mqd_t)-1) {
        DMR_LOGE("mq_open(%s) failed: %s", name, strerror(errno));
    }
    return mq;
}
 
  
 
dmr_err_t ccl_data_init(ccl_data_ctx_t *ctx,
                          dmr_slot_t      slot,
                          uint32_t        my_radio_id,
                          uint8_t         colour_code)
{
    if (ctx == NULL) return DMR_ERR_INVALID_PARAM;

    memset(ctx, 0, sizeof(*ctx));
    ctx->slot          = slot;
    ctx->my_radio_id   = my_radio_id;
    ctx->colour_code   = colour_code;
    ctx->state         = CCL_DATA_STATE_IDLE;
    ctx->t_response_ms = CCL_DATA_T_RESPONSE_MS;
    ctx->t_rx_stall_ms = CCL_DATA_T_RX_STALL_MS;

    if (pthread_mutex_init(&ctx->state_mutex, NULL) != 0) {
        return DMR_ERR_INVALID_PARAM;
    }

    const char *mq_evt_name  = (slot == DMR_SLOT_1)
                                 ? DMR_MQ_CCL_DATA_EVT_S1 : DMR_MQ_CCL_DATA_EVT_S2;
    /* MAC-owned, real shared queues (see ownership contract in
     * dmr_mac.h) — these are the SAME queues CCL Voice and Tier III
     * Trunking attach to on this slot, not a CCL-Data-private set.
     * Do not revert to DMR_MQ_DATA_MAC_* (the old placeholder names
     * defined in dmr_ccl_data.h); MAC never creates those. */
    const char *mq_tx_name   = (slot == DMR_SLOT_1)
                                 ? DMR_MQ_MAC_TX_REQ_S1 : DMR_MQ_MAC_TX_REQ_S2;
    const char *mq_conf_name = (slot == DMR_SLOT_1)
                                 ? DMR_MQ_MAC_TX_CONF_DATA_S1 : DMR_MQ_MAC_TX_CONF_DATA_S2;
    const char *mq_rx_name   = (slot == DMR_SLOT_1)
                                 ? DMR_MQ_MAC_RX_DATA_S1 : DMR_MQ_MAC_RX_DATA_S2;
                                 
                                 
    ctx->mq_evt = ccl_mq_create_own(mq_evt_name,
                                     CCL_DATA_MQ_MAX_MSGS, CCL_DATA_MQ_EVT_MSG_SIZE,
                                     O_RDWR);
                                     
                                     
     /* MAC TX request queue: CCL writes (via mac_tx_enqueue), MAC reads */
    ctx->mq_mac_tx = ccl_mq_open_mac_owned(mq_tx_name, O_WRONLY);
    /* MAC TX confirmation queue: MAC writes, CCL reads */
    ctx->mq_mac_conf = ccl_mq_open_mac_owned(mq_conf_name, O_RDONLY);
    /* MAC RX burst queue: MAC writes, CCL reads */
    ctx->mq_mac_rx = ccl_mq_open_mac_owned(mq_rx_name, O_RDONLY);

    if (ctx->mq_evt == (mqd_t)-1 || ctx->mq_mac_tx == (mqd_t)-1 ||
        ctx->mq_mac_conf == (mqd_t)-1 || ctx->mq_mac_rx == (mqd_t)-1) {
        ccl_data_destroy(ctx);
        return DMR_ERR_QUEUE_FULL;
    }

        if (dmr_phy_timer_oneshot_init(&ctx->tmr_response)   != DMR_OK ) {
        DMR_LOGE("[DATA S%d] dmr_phy_timer_oneshot_init failed: %s",
                 slot, strerror(errno));
        return DMR_ERR_NO_MEM;
    }

    if (dmr_phy_timer_oneshot_init(&ctx->tmr_rx_stall) != DMR_OK) {
        DMR_LOGE("[DATA S%d] dmr_phy_timer_oneshot_init (rx_stall) failed: %s",
                 slot, strerror(errno));
        return DMR_ERR_NO_MEM;
    }


    return DMR_OK;
}

dmr_err_t ccl_data_start(ccl_data_ctx_t *ctx)
{
    if (ctx == NULL) return DMR_ERR_INVALID_PARAM;
    if (ctx->running) return DMR_OK;

    ctx->running = true;
    if (pthread_create(&ctx->thread, NULL, ccl_data_thread, ctx) != 0) {
        ctx->running = false;
        return DMR_ERR_INVALID_PARAM;
    }
    return DMR_OK;
}

dmr_err_t ccl_data_stop(ccl_data_ctx_t *ctx)
{
    if (ctx == NULL) return DMR_ERR_INVALID_PARAM;
    if (!ctx->running) return DMR_OK;

    ccl_data_event_t e;
    memset(&e, 0, sizeof(e));
    e.type = CCL_DATA_EVT_SHUTDOWN;
    e.timestamp_us = dmr_time_now_us();
    mq_send(ctx->mq_evt, (const char *)&e, sizeof(e), 1u);

    ctx->running = false; /* also breaks epoll_wait timeout loop */
    pthread_join(ctx->thread, NULL);
    return DMR_OK;
}

void ccl_data_destroy(ccl_data_ctx_t *ctx)
{
    
    
          /* Close message queues. Only mq_evt is unlinked — CCL Voice created
     * it (see ccl_voice_init / ccl_mq_create_own) and is its sole owner.
     * mq_mac_tx/conf/rx are MAC-owned (see ownership contract in
     * dmr_mac.h); CCL only ever closes its handle to them, never
     * unlinks the name — that is mac_destroy()'s responsibility. */
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
    if (ctx->tmr_response.fd > 0) {
        dmr_phy_timer_oneshot_destroy(&ctx->tmr_response);
    }
    if (ctx->tmr_rx_stall.fd > 0) {
        dmr_phy_timer_oneshot_destroy(&ctx->tmr_rx_stall);
    }

    
    
    

    const char *mq_evt_name  = (ctx->slot == DMR_SLOT_1)
                                 ? DMR_MQ_CCL_DATA_EVT_S1 : DMR_MQ_CCL_DATA_EVT_S2;
   
    mq_unlink(mq_evt_name);
  

    pthread_mutex_destroy(&ctx->state_mutex);
    memset(ctx, 0, sizeof(*ctx));
}

/* =========================================================================
 * Introspection
 * ========================================================================= */
void ccl_data_get_stats(ccl_data_ctx_t *ctx, ccl_data_stats_t *out)
{
    if (ctx == NULL || out == NULL) return;
    pthread_mutex_lock(&ctx->state_mutex);
    *out = ctx->stats;
    pthread_mutex_unlock(&ctx->state_mutex);
}

ccl_data_state_t ccl_data_get_state(ccl_data_ctx_t *ctx)
{
    ccl_data_state_t s;
    pthread_mutex_lock(&ctx->state_mutex);
    s = ctx->state;
    pthread_mutex_unlock(&ctx->state_mutex);
    return s;
}