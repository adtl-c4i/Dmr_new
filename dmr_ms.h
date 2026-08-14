/**
 * @file dmr_ms.h
 * @brief DMR Mobile Station — Tier-Aware Composition Root
 *
 * This is the single entry point an application uses to bring up a
 * complete DMR MS for a chosen ETSI conformance tier (see dmr_tier.h).
 * It owns exactly one mac_ctx_t, one ccl_voice_ctx_t and one
 * ccl_data_ctx_t (every tier needs these three), plus conditionally:
 *
 *   DMR_TIER_1_DMO            a dmr_dmo_ctx_t (currently the MOD-15
 *                             placeholder stub — see dmr_dmo.h)
 *   DMR_TIER_3_TRUNKED        a t3_trunk_ctx_t (MOD-07)
 *
 * DMR_TIER_2_CONVENTIONAL composes only the always-present three —
 * this is today's existing behaviour, unchanged, just reached through
 * this composition layer instead of the application wiring MAC/CCL up
 * by hand.
 *
 * Design notes
 * ============
 * - Tier III Trunking is composed ALONGSIDE CCL Voice/Data, not
 *   inserted into their call-setup path. dmr_ms_init() makes Trunking
 *   available (started, attached to MAC) but does not make
 *   ccl_voice_tx_lc_header() or any other call-start function require
 *   or wait for a grant first. An application wanting "request a
 *   channel, then call" composes that sequence itself using
 *   dmr_ms_get_trunk(ctx) and dmr_ms_get_voice(ctx) together. This is a
 *   deliberate decoupling, not an oversight — wiring CCL's call path
 *   directly to Trunking's grant flow is tracked as separate future
 *   work, should it be wanted.
 * - This layer composes existing modules; it does not change their
 *   behaviour. Tier II call paths are byte-for-byte what they were
 *   before this layer existed.
 * - All four underlying modules already follow the same
 *   init/start/stop/destroy lifecycle shape; dmr_ms_* simply sequences
 *   calls to whichever subset applies to ctx->config.tier, in MAC-first
 *   order (MAC must be initialised before any CCL-side module opens
 *   the MAC-owned queues it depends on — see the ownership contract in
 *   dmr_mac.h).
 */

#ifndef DMR_MS_H
#define DMR_MS_H

#include <stdint.h>
#include <stdbool.h>

#include "dmr_types.h"
#include "dmr_tier.h"
#include "dmr_mac.h"
#include "dmr_ccl_voice.h"
#include "dmr_ccl_data.h"
#include "dmr_t3_trunk.h"
#include "dmr_dmo.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Composed MS context
 * ========================================================================= */
typedef struct {
    dmr_ms_config_t  config;     /**< Copy of the config passed to init() */
    bool              initialised;
    bool              running;

    /* Always present, every tier */
    mac_ctx_t         mac;
    ccl_voice_ctx_t   voice;
    ccl_data_ctx_t    data;

    /* Present only for DMR_TIER_3_TRUNKED; zeroed/unused otherwise.
     * trunk_active reflects whether trunk was actually initialised, so
     * dmr_ms_get_trunk() and dmr_ms_destroy() can tell "not this tier"
     * apart from "this tier but init partially failed". */
    t3_trunk_ctx_t    trunk;
    bool              trunk_active;

    /* Present only for DMR_TIER_1_DMO; zeroed/unused otherwise.
     * Currently the MOD-15 placeholder stub — see dmr_dmo.h. */
    dmr_dmo_ctx_t     dcdm;
    bool              dcdm_active;
} dmr_ms_ctx_t;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Validate cfg, then initialise MAC plus whichever modules
 *        ctx->config.tier requires, in MAC-first order.
 *
 * On any failure partway through, everything successfully initialised
 * so far is torn down before returning, so ctx is left in a clean,
 * all-zero state — callers do not need to call dmr_ms_destroy() after
 * a failed dmr_ms_init().
 *
 * @return DMR_OK, or the first error encountered (DMR_ERR_INVALID_PARAM
 *         from config validation, or whatever the failing module's own
 *         init returned).
 */
dmr_err_t dmr_ms_init(dmr_ms_ctx_t *ctx, const dmr_ms_config_t *cfg);

/**
 * @brief Start worker threads for every module this tier composed.
 *        Must be called after dmr_ms_init() succeeds.
 */
dmr_err_t dmr_ms_start(dmr_ms_ctx_t *ctx);

/**
 * @brief Stop worker threads for every module this tier composed.
 *        Safe to call even if dmr_ms_start() was never called (no-op
 *        on modules that were never started).
 */
dmr_err_t dmr_ms_stop(dmr_ms_ctx_t *ctx);

/**
 * @brief Tear down and zero everything dmr_ms_init() brought up.
 *        Safe to call on a zeroed/never-initialised ctx (no-op).
 */
void dmr_ms_destroy(dmr_ms_ctx_t *ctx);

/* =========================================================================
 * Accessors
 *
 * Returns NULL if the requested module was not composed for this
 * tier (e.g. dmr_ms_get_trunk() on a Tier I or Tier II ctx), so callers
 * can use these directly as a "is this available" check.
 * ========================================================================= */
static inline mac_ctx_t *dmr_ms_get_mac(dmr_ms_ctx_t *ctx)
{
    return (ctx != NULL && ctx->initialised) ? &ctx->mac : NULL;
}

static inline ccl_voice_ctx_t *dmr_ms_get_voice(dmr_ms_ctx_t *ctx)
{
    return (ctx != NULL && ctx->initialised) ? &ctx->voice : NULL;
}

static inline ccl_data_ctx_t *dmr_ms_get_data(dmr_ms_ctx_t *ctx)
{
    return (ctx != NULL && ctx->initialised) ? &ctx->data : NULL;
}

static inline t3_trunk_ctx_t *dmr_ms_get_trunk(dmr_ms_ctx_t *ctx)
{
    return (ctx != NULL && ctx->initialised && ctx->trunk_active)
               ? &ctx->trunk : NULL;
}

static inline dmr_dmo_ctx_t *dmr_ms_get_dcdm(dmr_ms_ctx_t *ctx)
{
    return (ctx != NULL && ctx->initialised && ctx->dcdm_active)
               ? &ctx->dcdm : NULL;
}

static inline dmr_tier_t dmr_ms_get_tier(const dmr_ms_ctx_t *ctx)
{
    return (ctx != NULL) ? ctx->config.tier : DMR_TIER_2_CONVENTIONAL;
}

#ifdef __cplusplus
}
#endif

#endif /* DMR_MS_H */
