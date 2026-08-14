/**
 * @file dmr_tier.h
 * @brief DMR Tier Selection — Public Types
 *
 * Defines which ETSI TS 102 361 conformance tier a DMR MS instance
 * operates in, and the per-tier configuration needed to compose the
 * right set of modules (MAC, CCL Voice, CCL Data, Tier III Trunking,
 * Tier II DCDM Wide Area Timing) for that tier.
 *
 * Tier semantics (informative summary; see dmr_ms.h for the composition
 * logic that acts on these):
 *
 *   DMR_TIER_1_DMO            ETSI TS 102 361-1/-2, unconnected Direct
 *                             Mode Operation. No base station; MS units
 *                             communicate peer-to-peer on a shared
 *                             frequency, continuous (non-TDMA) channel
 *                             access. No leader election, ever — Tier I
 *                             hardware does not support the coordinated
 *                             2-slot TDMA channel-sharing architecture.
 *                             Tx Timeout (T_TO) is FIXED at 180s
 *                             (TS 102 361-2 Clause 6.1) — not
 *                             configurable in this tier.
 *
 *   DMR_TIER_2_CONVENTIONAL   ETSI TS 102 361-1/-2, conventional
 *                             repeater or simplex operation. Fixed
 *                             channel/slot, no trunking control
 *                             channel. T_TO is configurable (0-180s,
 *                             0=disabled). May optionally run Dual
 *                             Capacity Direct Mode (DCDM) — TDMA Direct
 *                             Mode Wide Area Timing / Channel Timing
 *                             Leader election, TS 102 361-2 Clause 6.2 —
 *                             when both timeslots of a direct-mode
 *                             (no-BS) channel are used independently.
 *
 *   DMR_TIER_3_TRUNKED        ETSI TS 102 361-1/-4, trunked operation.
 *                             MS monitors a Trunking System Control
 *                             Channel (TSCC) and is granted a traffic
 *                             channel per call (MOD-07, Random Access +
 *                             Grant flow). T_TO is configurable, same
 *                             range as Tier II.
 *
 * A single dmr_tier_t value applies to one MS instance on one slot
 * pairing (a physical channel's two timeslots are configured together;
 * see dmr_ms_config_t). Running both Tier I DMO and Tier III trunked
 * service simultaneously on different channels would mean running two
 * independent dmr_ms_ctx_t instances, one per tier/channel — this is a
 * supported pattern, not something this header needs to model directly.
 */

#ifndef DMR_TIER_H
#define DMR_TIER_H

#include <stdint.h>
#include <stdbool.h>

#include "dmr_types.h"
#include "dmr_lrrp.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Tier identifier
 * ========================================================================= */
typedef enum {
    DMR_TIER_1_DMO           = 1,
    DMR_TIER_2_CONVENTIONAL  = 2,
    DMR_TIER_3_TRUNKED       = 3,
} dmr_tier_t;

static inline const char *dmr_tier_name(dmr_tier_t tier)
{
    switch (tier) {
    case DMR_TIER_1_DMO:          return "Tier I (DMO)";
    case DMR_TIER_2_CONVENTIONAL: return "Tier II (Conventional)";
    case DMR_TIER_3_TRUNKED:      return "Tier III (Trunked)";
    default:                      return "Unknown";
    }
}

/* =========================================================================
 * Transmit Timeout (T_TO) — ETSI TS 102 361-2 Clause 6.1
 *
 * Tier I: fixed at 180s, not configurable — DMR_TIER1_T_TO_MS is the
 * only legal value and dmr_ms_init() enforces it regardless of what a
 * caller puts in dmr_ms_config_t.t_to_ms for that tier.
 * Tier II/III: configurable 0 (disabled) to 180s inclusive.
 * ========================================================================= */
#define DMR_TIER1_T_TO_MS        180000u   /* Fixed for Tier I, per spec   */
#define DMR_T_TO_MAX_MS          180000u   /* Upper bound for Tier II/III  */
#define DMR_T_TO_DISABLED_MS     0u        /* 0 = disabled (Tier II/III)   */

/* =========================================================================
 * Tier I (DMO) specific configuration
 * ========================================================================= */
typedef struct {
    /** This MS operates as plain (non-wide-area) Direct Mode on its
     *  provisioned slot only, with no leader election, no CT_CSBK
     *  transmission/reception — Tier I never elects a Channel Timing
     *  Leader (see file header). */

    /** This MS's provisioned timeslot for Direct Mode. Per spec, an MS
     *  changing slots on the same frequency keeps its current timing
     *  state — this field identifies which slot's sync pattern this MS
     *  transmits with. */
    dmr_slot_t provisioned_slot;
} dmr_tier1_config_t;

/* =========================================================================
 * Tier II (conventional) specific configuration
 * ========================================================================= */
typedef struct {
    /** Enable Dual Capacity Direct Mode (DCDM) — TDMA Direct Mode Wide
     *  Area Timing / Channel Timing Leader (CTL) election, TS 102 361-2
     *  Clause 6.2. This is a CAPABILITY flag, not an unconditional
     *  activation flag: the DCDM module still checks for repeater (BS)
     *  presence at startup before committing to leader election — see
     *  dmr_dmo.h's DMO_STATE_DETECTING for the auto-detect mechanism.
     *  A repeater-backed conventional channel should leave this false
     *  regardless, since a radio that will never operate on a bare
     *  direct-mode channel gains nothing from the detection wait. */
    bool dcdm_enabled;

    /** This radio's Wide Area Timing leader preference/eligibility —
     *  DMO_DI_UNKNOWN_OR_INELIGIBLE/LOW/MEDIUM/HIGH (dmr_dmo.h). Only
     *  meaningful when dcdm_enabled. Higher values are preferred as
     *  Channel Timing Leader; DMO_DI_UNKNOWN_OR_INELIGIBLE (0) means
     *  this radio will never volunteer to lead but can still follow.
     *  Ignored (treated as DMO_DI_MEDIUM) when left at 0 by callers
     *  that pre-date this field — see dmr_ms_config_default_tier2(). */
    uint8_t leader_di;

    /** Fixed timeslot this MS uses for conventional operation. For a
     *  single-slot conventional channel, set both ctx->slot and this
     *  to the same value; true 2-slot conventional (independent calls
     *  per slot) is composed as two dmr_ms_ctx_t instances. */
    dmr_slot_t fixed_slot;
} dmr_tier2_config_t;

/* =========================================================================
 * Tier III (trunked) specific configuration
 * ========================================================================= */
typedef struct {
    /** Slot the Trunking System Control Channel (TSCC) is monitored on. */
    dmr_slot_t tscc_slot;
} dmr_tier3_config_t;

/* =========================================================================
 * Top-level MS configuration — selects the tier and carries the
 * tier-specific parameters plus identity fields common to all tiers.
 * ========================================================================= */
typedef struct {
    dmr_tier_t tier;

    /** Common identity, used by every module regardless of tier. */
    uint32_t radio_id;
    uint8_t  colour_code;

    /** Transmit Timeout. For DMR_TIER_1_DMO this is ignored at runtime
     *  and DMR_TIER1_T_TO_MS is used instead — set here only for
     *  documentation/inspection purposes. For Tier II/III this is the
     *  actual value used (0 = disabled), must be <= DMR_T_TO_MAX_MS. */
    uint32_t t_to_ms;
    dmr_lrrp_type_t lrrp_type;

    /** Exactly one of the following is read, selected by `tier` above. */
    union {
        dmr_tier1_config_t tier1;
        dmr_tier2_config_t tier2;
        dmr_tier3_config_t tier3;
    } cfg;
} dmr_ms_config_t;

/* =========================================================================
 * Convenience constructors — fill in sane defaults for each tier so
 * callers don't have to remember every field. All return a fully valid
 * dmr_ms_config_t; callers may still override individual fields
 * afterward before passing to dmr_ms_init().
 * ========================================================================= */
static inline dmr_ms_config_t dmr_ms_config_default_tier1(
    uint32_t radio_id, uint8_t colour_code, dmr_slot_t provisioned_slot,
    dmr_lrrp_type_t lrrp_type)
{
    dmr_ms_config_t cfg;
    cfg.tier              = DMR_TIER_1_DMO;
    cfg.radio_id           = radio_id;
    cfg.colour_code        = colour_code;
    cfg.t_to_ms             = DMR_TIER1_T_TO_MS; /* fixed, see above */
    cfg.lrrp_type = lrrp_type;
    cfg.cfg.tier1.provisioned_slot          = provisioned_slot;
    return cfg;
}

static inline dmr_ms_config_t dmr_ms_config_default_tier2(
    uint32_t radio_id, uint8_t colour_code, dmr_slot_t fixed_slot,
    uint32_t t_to_ms, bool dcdm_enabled, uint8_t leader_di,
    dmr_lrrp_type_t lrrp_type)
{
    dmr_ms_config_t cfg;
    cfg.tier        = DMR_TIER_2_CONVENTIONAL;
    cfg.radio_id     = radio_id;
    cfg.colour_code  = colour_code;
    cfg.t_to_ms       = t_to_ms;
    cfg.lrrp_type = lrrp_type;
    cfg.cfg.tier2.dcdm_enabled = dcdm_enabled;
    /* 2 bits (0-3); default to MEDIUM (2) for any out-of-range value so
     * a stale caller passing an old-style non-DI argument doesn't
     * silently end up ineligible-to-lead (0) by accident. */
    cfg.cfg.tier2.leader_di = (leader_di <= 3u) ? leader_di : 2u;
    cfg.cfg.tier2.fixed_slot = fixed_slot;
    return cfg;
}

static inline dmr_ms_config_t dmr_ms_config_default_tier3(
    uint32_t radio_id, uint8_t colour_code, dmr_slot_t tscc_slot,
    uint32_t t_to_ms, dmr_lrrp_type_t lrrp_type)
{
    dmr_ms_config_t cfg;
    cfg.tier        = DMR_TIER_3_TRUNKED;
    cfg.radio_id     = radio_id;
    cfg.colour_code  = colour_code;
    cfg.t_to_ms       = t_to_ms;
    cfg.lrrp_type = lrrp_type;
    cfg.cfg.tier3.tscc_slot = tscc_slot;
    return cfg;
}

/**
 * @brief Validate a dmr_ms_config_t for internal consistency before
 *        passing it to dmr_ms_init(). Checks t_to_ms range for Tier
 *        II/III and that colour_code fits 4 bits. Does not mutate cfg.
 *
 * @return DMR_OK if valid, DMR_ERR_INVALID_PARAM otherwise.
 */
static inline dmr_err_t dmr_ms_config_validate(const dmr_ms_config_t *cfg)
{
    if (cfg == NULL) return DMR_ERR_INVALID_PARAM;
    if (cfg->colour_code > 0x0Fu) return DMR_ERR_INVALID_PARAM;

    switch (cfg->tier) {
    case DMR_TIER_1_DMO:
        /* t_to_ms is ignored/overridden at init time for Tier I, so no
         * range check needed here. provisioned_slot must be a real slot. */
        if (cfg->cfg.tier1.provisioned_slot != DMR_SLOT_1 &&
            cfg->cfg.tier1.provisioned_slot != DMR_SLOT_2) {
            return DMR_ERR_INVALID_PARAM;
        }
        return DMR_OK;

    case DMR_TIER_2_CONVENTIONAL:
        if (cfg->t_to_ms > DMR_T_TO_MAX_MS) return DMR_ERR_INVALID_PARAM;
        if (cfg->cfg.tier2.fixed_slot != DMR_SLOT_1 &&
            cfg->cfg.tier2.fixed_slot != DMR_SLOT_2) {
            return DMR_ERR_INVALID_PARAM;
        }
        return DMR_OK;

    case DMR_TIER_3_TRUNKED:
        if (cfg->t_to_ms > DMR_T_TO_MAX_MS) return DMR_ERR_INVALID_PARAM;
        if (cfg->cfg.tier3.tscc_slot != DMR_SLOT_1 &&
            cfg->cfg.tier3.tscc_slot != DMR_SLOT_2) {
            return DMR_ERR_INVALID_PARAM;
        }
        return DMR_OK;

    default:
        return DMR_ERR_INVALID_PARAM;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* DMR_TIER_H */
