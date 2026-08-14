/**
 * @file dmr_ms.c
 * @brief DMR Mobile Station — Tier-Aware Composition Root
 *
 * See dmr_ms.h for design notes and scope.
 */

#define _POSIX_C_SOURCE 200809L

#include <string.h>

#include "dmr_ms.h"

/* =========================================================================
 * Per-tier slot resolution
 *
 * Every composed module needs a dmr_slot_t. Which one depends on tier:
 *   Tier I   -> cfg.tier1.provisioned_slot
 *   Tier II  -> cfg.tier2.fixed_slot (same field whether or not DCDM
 *               is enabled — DCDM's second slot is handled by MOD-15
 *               once implemented, not by this resolver)
 *   Tier III -> cfg.tier3.tscc_slot (MAC/CCL run on the TSCC's slot;
 *               the actual traffic channel/slot a granted call uses is
 *               a separate concern handled by the application via the
 *               Trunking grant callback — see dmr_ms.h design notes)
 * ========================================================================= */
static dmr_slot_t dmr_ms_resolve_slot(const dmr_ms_config_t *cfg)
{
    switch (cfg->tier) {
    case DMR_TIER_1_DMO:          return cfg->cfg.tier1.provisioned_slot;
    case DMR_TIER_2_CONVENTIONAL: return cfg->cfg.tier2.fixed_slot;
    case DMR_TIER_3_TRUNKED:      return cfg->cfg.tier3.tscc_slot;
    default:                      return DMR_SLOT_1;
    }
}

/* =========================================================================
 * Lifecycle
 * ========================================================================= */
dmr_err_t dmr_ms_init(dmr_ms_ctx_t *ctx, const dmr_ms_config_t *cfg)
{
    if (ctx == NULL || cfg == NULL) return DMR_ERR_INVALID_PARAM;

    dmr_err_t verr = dmr_ms_config_validate(cfg);
    if (verr != DMR_OK) return verr;

    memset(ctx, 0, sizeof(*ctx));
    ctx->config = *cfg;

    /* Tier I's T_TO is fixed by spec regardless of what the caller put
     * in cfg->t_to_ms — enforce that here so ctx->config always
     * reflects the value actually in effect (see dmr_tier.h). */
    if (cfg->tier == DMR_TIER_1_DMO) {
        ctx->config.t_to_ms = DMR_TIER1_T_TO_MS;
    }

    /* DCDM (leader election, MOD-15) composes only for Tier II with
     * dcdm_enabled=true — see dmr_tier.h tier semantics note. Tier I
     * never runs leader election. */
    bool want_dcdm = (cfg->tier == DMR_TIER_2_CONVENTIONAL) &&
                      cfg->cfg.tier2.dcdm_enabled;

    dmr_slot_t slot = dmr_ms_resolve_slot(&ctx->config);

    /* MAC first — every CCL-side module's queue-open retries depend on
     * MAC having created the shared queues for this slot (see the
     * ownership contract in dmr_mac.h). */
    dmr_err_t err = mac_init(&ctx->mac, slot, ctx->config.colour_code,
                              ctx->config.radio_id, ctx->config.tier);
    if (err != DMR_OK) {
        memset(ctx, 0, sizeof(*ctx));
        return err;
    }

    err = ccl_voice_init(&ctx->voice, slot, ctx->config.radio_id,
                          ctx->config.colour_code);
    if (err != DMR_OK) {
        mac_destroy(&ctx->mac);
        memset(ctx, 0, sizeof(*ctx));
        return err;
    }
    ctx->voice.mac = &ctx->mac;

    err = ccl_data_init(&ctx->data, slot, ctx->config.radio_id,
                         ctx->config.colour_code);
    if (err != DMR_OK) {
        ccl_voice_destroy(&ctx->voice);
        mac_destroy(&ctx->mac);
        memset(ctx, 0, sizeof(*ctx));
        return err;
    }
    ctx->data.mac = &ctx->mac;

    /* Tier-specific module */
    switch (ctx->config.tier) {

    case DMR_TIER_3_TRUNKED:
        err = t3_trunk_init(&ctx->trunk, ctx->config.cfg.tier3.tscc_slot,
                             ctx->config.radio_id, ctx->config.colour_code);
        if (err != DMR_OK) {
            ccl_data_destroy(&ctx->data);
            ccl_voice_destroy(&ctx->voice);
            mac_destroy(&ctx->mac);
            memset(ctx, 0, sizeof(*ctx));
            return err;
        }
        ctx->trunk_active = true;
        break;

    case DMR_TIER_1_DMO:
    case DMR_TIER_2_CONVENTIONAL:
    default:
        /* Nothing further to compose here — DCDM (if any) is handled
         * below, after the switch, since it's a Tier II sub-feature
         * rather than a distinct tier case. */
        break;
    }

    if (want_dcdm) {
        /* MOD-15 implemented — see dmr_dmo.h/dmr_dmo.c. Real leader
         * election worker thread; dmr_ms_start()/dmr_ms_stop() now
         * start/stop it alongside the other composed modules. */
     //    printf("DCDM############### \n" );
        err = dmr_dmo_init(&ctx->dcdm, &ctx->config.cfg.tier2,
                            ctx->config.radio_id, ctx->config.colour_code);
        if (err != DMR_OK) {
            if (ctx->trunk_active) t3_trunk_destroy(&ctx->trunk);
            ccl_data_destroy(&ctx->data);
            ccl_voice_destroy(&ctx->voice);
            mac_destroy(&ctx->mac);
            memset(ctx, 0, sizeof(*ctx));
            return err;
        }
        ctx->dcdm_active = true;

        /* Wire CCL Voice/Data to DCDM so their post-TX hooks can send
         * CT_CSBK_Term (Cl.6.2.2.3.3) — see dmr_ccl_voice.c's
         * ccl_voice_tx_terminator() and dmr_ccl_data.c's
         * ccl_data_tx_finish(). t3_trunk_ctx_t has no equivalent field:
         * DCDM only ever runs on DMR_TIER_2_CONVENTIONAL and Trunking
         * only on DMR_TIER_3_TRUNKED, so trunk_active and dcdm_active
         * can never both be true — there's nothing to wire there. */
        ctx->voice.dcdm = &ctx->dcdm;
        ctx->data.dcdm  = &ctx->dcdm;
    }

    ctx->initialised = true;
    return DMR_OK;
}

dmr_err_t dmr_ms_start(dmr_ms_ctx_t *ctx)
{
    if (ctx == NULL || !ctx->initialised) return DMR_ERR_INVALID_PARAM;
    if (ctx->running) return DMR_OK;

    dmr_err_t err = mac_start(&ctx->mac);
    if (err != DMR_OK) return err;

    err = ccl_voice_start(&ctx->voice);
    if (err != DMR_OK) {
        mac_stop(&ctx->mac);
        return err;
    }

    err = ccl_data_start(&ctx->data);
    if (err != DMR_OK) {
        ccl_voice_stop(&ctx->voice);
        mac_stop(&ctx->mac);
        return err;
    }

    if (ctx->trunk_active) {
        err = t3_trunk_start(&ctx->trunk);
        if (err != DMR_OK) {
            ccl_data_stop(&ctx->data);
            ccl_voice_stop(&ctx->voice);
            mac_stop(&ctx->mac);
            return err;
        }
    }

    if (ctx->dcdm_active) {
        err = dmr_dmo_start(&ctx->dcdm);
        if (err != DMR_OK) {
            if (ctx->trunk_active) t3_trunk_stop(&ctx->trunk);
            ccl_data_stop(&ctx->data);
            ccl_voice_stop(&ctx->voice);
            mac_stop(&ctx->mac);
            return err;
        }
    }

    ctx->running = true;
    return DMR_OK;
}

dmr_err_t dmr_ms_stop(dmr_ms_ctx_t *ctx)
{
    if (ctx == NULL || !ctx->initialised) return DMR_ERR_INVALID_PARAM;
    if (!ctx->running) return DMR_OK;

    /* Stop in roughly reverse-start order; each *_stop() is independently
     * safe to call regardless of the others' outcome, so we collect the
     * first error but still attempt every stop. */
    dmr_err_t first_err = DMR_OK;
    dmr_err_t err;

    if (ctx->dcdm_active) {
        err = dmr_dmo_stop(&ctx->dcdm);
        if (err != DMR_OK && first_err == DMR_OK) first_err = err;
    }

    if (ctx->trunk_active) {
        err = t3_trunk_stop(&ctx->trunk);
        if (err != DMR_OK && first_err == DMR_OK) first_err = err;
    }

    err = ccl_data_stop(&ctx->data);
    if (err != DMR_OK && first_err == DMR_OK) first_err = err;

    err = ccl_voice_stop(&ctx->voice);
    if (err != DMR_OK && first_err == DMR_OK) first_err = err;

    err = mac_stop(&ctx->mac);
    if (err != DMR_OK && first_err == DMR_OK) first_err = err;

    ctx->running = false;
    return first_err;
}

void dmr_ms_destroy(dmr_ms_ctx_t *ctx)
{
    if (ctx == NULL) return;
    if (!ctx->initialised) {
        memset(ctx, 0, sizeof(*ctx));
        return;
    }

    if (ctx->running) {
        dmr_ms_stop(ctx);
    }

    if (ctx->trunk_active) {
        t3_trunk_destroy(&ctx->trunk);
    }
    if (ctx->dcdm_active) {
        dmr_dmo_destroy(&ctx->dcdm);
    }

    ccl_data_destroy(&ctx->data);
    ccl_voice_destroy(&ctx->voice);
    mac_destroy(&ctx->mac);

    memset(ctx, 0, sizeof(*ctx));
}