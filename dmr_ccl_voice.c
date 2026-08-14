#define _GNU_SOURCE

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <mqueue.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <time.h>

#include "dmr_pdu.h"
#include "dmr_types.h"
#include "dmr_mac.h"
#include "dmr_llc.h"
#include "dmr_ccl_voice.h"
#include "dmr_fec.h"

/* =========================================================================
 * Internal constants
 * ========================================================================= */
#define CCL_VOCODER_TX_PIPE_FRAMES   16
#define CCL_VOCODER_RX_PIPE_FRAMES   16
#define CCL_EPOLL_MAX_EVENTS         8

/* Depth for mq_evt ONLY — this is the one queue CCL Voice itself creates
 * (it is private to this module; nothing else opens it). It must NOT be
 * used for mq_mac_tx / mq_mac_conf / mq_mac_rx — those are MAC-owned
 * (see ownership contract in dmr_mac.h) and their depth is fixed by
 * DMR_MQ_MAX_MSGS, decided once by MAC at creation time. */
#define CCL_MQ_EVT_MAX_MSGS          10
#define CCL_MQ_EVT_MSG_SIZE          sizeof(ccl_voice_event_t)
#define CCL_MQ_BURST_MSG_SIZE        sizeof(dmr_burst_t)
#define CCL_MQ_TX_REQ_MSG_SIZE       sizeof(dmr_mac_tx_req_t)
#define CCL_MQ_TX_CONF_MSG_SIZE      sizeof(dmr_mac_tx_conf_t)



/* Superframe burst indices */
#define SF_BURST_A  0
#define SF_BURST_B  1
#define SF_BURST_C  2
#define SF_BURST_D  3
#define SF_BURST_E  4
#define SF_BURST_F  5

/*
 * LCSS sequence for embedded LC delivery over superframe bursts B..F
 * ETSI TS 102 361-1, Clause 9.1.9 / Table 7.6 (inbound channel):
 *   B = First fragment  (LCSS=01)
 *   C = Continuation    (LCSS=11)
 *   D = Continuation    (LCSS=11)
 *   E = Last fragment   (LCSS=10)
 *   F = Single (null)   (LCSS=00)
 */
static const uint8_t sf_lcss_table[6] = {
    0x00,   /* A — Voice LC Header (DTYPE=0x01), no EMB used */
    DMR_LCSS_FIRST,   /* B — 0x01 First fragment  */
    DMR_LCSS_CONT,    /* C — 0x03 Continuation    */
    DMR_LCSS_CONT,    /* D — 0x03 Continuation    */
    DMR_LCSS_LAST,    /* E — 0x02 Last fragment   */
    DMR_LCSS_SINGLE,  /* F — 0x00 Null (complete) */
};

/* =========================================================================
 * SECTION A — Timer helpers
 * ========================================================================= */



static void ccl_voice_arm_timer(dmr_phy_timer_oneshot_t *t, uint32_t ms)
{
    dmr_phy_timer_oneshot_arm_ms(t, ms);
}

/* Disarm a one-shot PHY timer */
static void ccl_voice_disarm_timer(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_disarm(t);
}

/* Drain a one-shot PHY timer after expiry */
static void ccl_voice_timer_drain(dmr_phy_timer_oneshot_t *t)
{
    dmr_phy_timer_oneshot_drain(t);
}

/* =========================================================================
 * SECTION B — State transition
 * ========================================================================= */

void ccl_voice_set_state(ccl_voice_ctx_t *ctx, ccl_voice_state_t new_state)
{
    pthread_mutex_lock(&ctx->state_mutex);
    ccl_voice_state_t old = ctx->state;
    ctx->state = new_state;
    pthread_mutex_unlock(&ctx->state_mutex);

    if (old != new_state) {
        DMR_LOGI("[CCL S%d] %s → %s",
                 ctx->slot, CCL_STATE_NAMES[old], CCL_STATE_NAMES[new_state]);
        if (ctx->on_state_change)
            ctx->on_state_change(ctx, old, new_state);
    }
}

ccl_voice_state_t ccl_voice_get_state(ccl_voice_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);
    ccl_voice_state_t s = ctx->state;
    pthread_mutex_unlock(&ctx->state_mutex);
    return s;
}

/* =========================================================================
 * SECTION C — Call context helpers
 * ========================================================================= */

static void ccl_call_ctx_clear(ccl_call_ctx_t *c)
{
    memset(c, 0, sizeof(*c));
}

static void ccl_call_ctx_init_group(ccl_call_ctx_t *c,
                                     uint32_t src_id,
                                     uint32_t dst_id,
                                     uint8_t  cc,
                                     bool     emergency)
{
    ccl_call_ctx_clear(c);
    c->call_type     = emergency ? DMR_CALL_TYPE_EMERGENCY : DMR_CALL_TYPE_GROUP;
    c->src_id        = src_id;
    c->dst_id        = dst_id;
    c->colour_code   = cc;
    c->emergency     = emergency;
    c->call_start_us = dmr_time_now_us();
}

static void ccl_call_ctx_init_individual(ccl_call_ctx_t *c,
                                          uint32_t src_id,
                                          uint32_t dst_id,
                                          uint8_t  cc,
                                          bool     emergency)
{
    ccl_call_ctx_clear(c);
    c->call_type     = emergency ? DMR_CALL_TYPE_EMERGENCY : DMR_CALL_TYPE_INDIVIDUAL;
    c->src_id        = src_id;
    c->dst_id        = dst_id;
    c->colour_code   = cc;
    c->emergency     = emergency;
    c->call_start_us = dmr_time_now_us();
}

/* =========================================================================
 * SECTION D — MAC TX submission
 *
 * CCL holds mq_mac_tx (a write-end mqd_t) opened during init.
 * mac_tx_enqueue(mqd_t, req) is the correct API from dmr_mac.h.
 * ========================================================================= */

static dmr_err_t ccl_submit_burst(ccl_voice_ctx_t        *ctx,
                                   const dmr_burst_t      *burst,
                                   dmr_mac_priority_t      priority,
                                   bool                    impolite)
{
    dmr_mac_tx_req_t req;
    memset(&req, 0, sizeof(req));
    req.burst      = *burst;
    req.slot       = ctx->slot;
    req.priority   = priority;
    req.impolite   = impolite;
    req.req_id     = ctx->tx_req_id_next++;
    req.deadline_us= 0;
    req.originated_from=CCL_TX_ORIGIN_VOICE;

    ctx->tx_req_id_pending = req.req_id;

    dmr_err_t err = mac_tx_enqueue(ctx->mq_mac_tx, &req);
    if (err == DMR_OK) {
        ctx->stats.bursts_tx++;
    } else {
        DMR_LOGW("[CCL S%d] mac_tx_enqueue failed err=%d", ctx->slot, err);
        ctx->stats.tx_aborts++;
    }
    return err;
}

/* =========================================================================
 * SECTION E — TX burst builders  (use LLC layer — no FEC stubs)
 * ========================================================================= */

/*
 * Build Voice LC Header or Terminator body using LLC, then submit to MAC.
 * RS(12,9) parity bytes are left zero (filled by MOD-02 when integrated).
 */

dmr_err_t ccl_voice_tx_lc_header(ccl_voice_ctx_t *ctx)
{
   
    dmr_full_lc_t lc;
    uint8_t svc = 0u;
    DMR_SVC_SET_EMERGENCY(svc, ctx->call.emergency ? 1u : 0u);
    DMR_SVC_SET_PRIORITY(svc, ctx->call.priority);

    if (ctx->call.call_type == DMR_CALL_TYPE_INDIVIDUAL) {
        llc_full_lc_ind_voice_build(&lc, svc,ctx->fid, ctx->call.dst_id, ctx->call.src_id);
    } else {
        llc_full_lc_grp_voice_build(&lc, svc,ctx->fid, ctx->call.dst_id, ctx->call.src_id);
    }
    ctx->call.full_lc=lc;
     ctx->call.lc_valid=true;

                                  

    dmr_burst_t burst;
    /* outbound=false: MS→BS inbound uses MS_DATA sync for data bursts */
    llc_voice_lc_header_build(&burst, &lc, ctx->colour_code, false, ctx->slot);


    /* Reset the superframe position to A here, at LC Header build time —
     * not only in the CCL_EVT_TX_CONF handler for CCL_STATE_TX_LC_HEADER.
     * The Voice LC Header is superframe position A's data-burst
     * counterpart: the first AMBE voice burst transmitted after it MUST
     * be superframe burst A (full SYNC, no EMB) per ETSI TS 102 361-1
     * Clause 9.1.3. Relying solely on the later TX-confirmation callback
     * to perform this reset is fragile — if that event is ever dropped,
     * delayed across a race, or the dispatch path changes, sf_burst_idx
     * would carry over stale state from a previous call. A stale
     * non-zero sf_burst_idx makes ccl_voice_tx_next_burst() write the
     * EMB field (see dmr_burst_set_emb()) on what should be burst A,
     * overwriting the SYNC field the receiver depends on to recognise
     * burst A via dmr_burst_is_voice() — silently breaking late-entry
     * and first-burst AMBE delivery for that call. Setting it here, at
     * the single point where a new call's LC Header is always built,
     * removes that dependency. */
    ctx->sf_burst_idx = SF_BURST_A;

    DMR_LOGD("[CCL S%d] TX Voice LC Header dst=0x%06X src=0x%06X",
             ctx->slot, ctx->call.dst_id, ctx->call.src_id);

    return ccl_submit_burst(ctx, &burst, DMR_MAC_PRIORITY_HIGH, false);
}





dmr_err_t ccl_voice_tx_terminator(ccl_voice_ctx_t *ctx)
{
    
    dmr_full_lc_t lc;
    uint8_t svc = 0u;
    DMR_SVC_SET_EMERGENCY(svc, ctx->call.emergency ? 1u : 0u);

    if (ctx->call.call_type == DMR_CALL_TYPE_INDIVIDUAL) {
        llc_full_lc_ind_voice_build(&lc, svc,ctx->fid, ctx->call.dst_id, ctx->call.src_id);
    } else {
        llc_full_lc_grp_voice_build(&lc, svc,ctx->fid,  ctx->call.dst_id, ctx->call.src_id);
    }

    dmr_burst_t burst;
    llc_terminator_lc_build(&burst, &lc, ctx->colour_code, false, ctx->slot);

    DMR_LOGD("[CCL S%d] TX Terminator dst=0x%06X", ctx->slot, ctx->call.dst_id);

    dmr_err_t err = ccl_submit_burst(ctx, &burst, DMR_MAC_PRIORITY_HIGH, true);

    /* MOD-15 (DCDM) Cl.6.2.2.3.3: send CT_CSBK_Term after this voice call
     * TX completes. ccl_voice_tx_uu_ans_rsp() (Ack to a CSBK) is a
     * separate function this hook is never reached from, correctly
     * respecting the spec's stated exception. channel_activity reflects
     * whether the *other* slot is currently busy, via MAC's existing
     * CACH AT-bit tracking (mac_slot_is_busy()) — see the matching note
     * in dmr_ccl_data.c's ccl_data_tx_finish(). */
    if (ctx->dcdm != NULL) {
        dmr_slot_t other_slot = (ctx->slot == DMR_SLOT_1) ? DMR_SLOT_2 : DMR_SLOT_1;
        bool channel_activity = (ctx->mac != NULL) && mac_slot_is_busy(ctx->mac, other_slot);
        dmr_dmo_notify_tx(ctx->dcdm, channel_activity);
    }

    return err;
}

/*
 * ccl_voice_tx_next_burst — transmit the next voice burst in the superframe.
 *
 * Burst A is the Voice LC Header (handled separately in ccl_voice_tx_lc_header).
 * Bursts B..F are voice bursts with embedded LC fragments.
 *
 * For each voice burst (B..F):
 *   1. Read next AMBE+2 frame from vocoder TX pipe (non-blocking)
 *   2. Build a voice burst with:
 *      - Voice SYNC pattern (DMR_SYNC_MS_VOICE)
 *      - INFO_1 = AMBE frame first 13.5 bytes (108 bits)
 *      - INFO_2 = AMBE frame last 13.5 bytes (108 bits)
 *      - EMB field: CC, PI=0, LCSS from sf_lcss_table, embedded LC fragment
 *   3. Submit to MAC (impolite — in-call)
 *
 * AMBE+2 frame packing (ETSI TS 102 361-1, Clause 6.1):
 *   Each voice burst carries 216 bits = 2 × 108-bit vocoder socket.
 *   Two AMBE+2 frames (2 × 72 bits = 144 bits) plus 72 bits of FEC/stuff
 *   are distributed across INFO_1 and INFO_2.
 *
 *   Simplified: we pack the 9-byte (72-bit) AMBE frame into the first 9
 *   bytes of INFO_1, leaving the rest zero until MOD-09 (vocoder) is wired.
 */
dmr_err_t ccl_voice_tx_next_burst(ccl_voice_ctx_t *ctx)
{
    /* Advance burst index — wrap A→B after LC header sent */

    uint8_t burst_pos = ctx->sf_burst_idx;
    uint8_t lcss      = sf_lcss_table[burst_pos];

    /* Read AMBE+2 frame from vocoder TX pipe (non-blocking) */
    dmr_ambe_frame_t ambe;
    memset(&ambe, 0, sizeof(ambe));
    ssize_t n = read(ctx->vocoder.tx_read_fd,
                     ambe.data, DMR_AMBE_FRAME_BYTES);
    if (n < (ssize_t)DMR_AMBE_FRAME_BYTES) {
        /* No frame available — use silence frame (all zeros) */
        memset(ambe.data, 0, DMR_AMBE_FRAME_BYTES);
    }

    /* Build the 216-bit vocoder socket (INFO_1 + INFO_2 = 2 × 108 bits).
     * ETSI TS 102 361-1, Clause 6.1: VS(215)=MSB of VF(1), VS(0)=LSB of VF(M).
     * With one AMBE+2 frame per burst:
     *   VS(215..144) = AMBE[71..0]  (first 9 bytes in upper 9 bytes of INFO_1)
     *   VS(143..0)   = zeros / second frame */
    uint8_t info[27]; /* 108 bits upper-aligned */
  //  uint8_t info2[14]; /* 108 bits upper-aligned */
    memset(info, 0, sizeof(info));
 //   memset(info2, 0, sizeof(info2));

/**Nishant*fill all voice data in one array  info 1****/
    /* Pack AMBE into INFO_1 bits [107:36] (bytes 0..8) */
    memcpy(info, ambe.data, 9);  /* 9 bytes = 72 bits */
    for(int i=0;i<27;i++)
    info[i]=i+30;
    /* Build embedded LC fragment for bursts B..E */
    /* The Full LC is split into 4 × 18-bit fragments:
     *   Burst B: frag[0] = LC bits [71:54]
     *   Burst C: frag[1] = LC bits [53:36]
     *   Burst D: frag[2] = LC bits [35:18]
     *   Burst E: frag[3] = LC bits [17:0]
     * Each fragment is 3 bytes (24 bits) with 18 bits used.
     * ETSI TS 102 361-1, Clause 7.1.3 / B.2.1
     */
    uint8_t lc_raw[12];   /* 32-bit fragment for EMB (4 bytes) */
    
    
    memcpy(lc_raw, &ctx->call.full_lc, sizeof(lc_raw));
    

        uint8_t lc_frag[4][4];   /* 32-bit fragment for EMB (4 bytes) */
    memset(lc_frag, 0, sizeof(lc_frag));
    
    emblc_bptc_encode(lc_raw, lc_frag);

#if 0
    if (ctx->call.lc_valid && burst_pos >= SF_BURST_B && burst_pos <= SF_BURST_E) {
      //  lc_raw = (const uint8_t *)&ctx->call.full_lc;
        /* Fragment index: B=0, C=1, D=2, E=3 */
        uint8_t frag_idx = (uint8_t)(burst_pos - SF_BURST_B);
         uint8_t byte_start=0;
        /* Extract 18 bits starting at bit (71 - frag_idx*18) down */
        /* LC is 72 bits (9 bytes), MSB first.
         * Fragment 0: bits [71:54] = lc_raw bytes [0][7:0] + [1][7:6]
         * Use a byte-aligned 3-byte extraction: */
         if(frag_idx>0)
         byte_start = (uint8_t)(frag_idx * 3u); /* 18 bits ~ 2.25 bytes */
         for (int b = 0; b < 3; b++) {
            lc_frag[b] = lc_raw[byte_start + b];
            // printf("%x -[%d]",lc_frag[b],byte_start + b);
        }

    }  

    else{
      /*   printf("\n TX-raw error \n");
         for(int i=0;i<12;i++)
    printf("%x ",lc_raw[i]);
    printf(",\n");*/
        
    }
#endif

    /* Assemble the complete burst */
    dmr_burst_t burst;
    memset(&burst, 0, sizeof(burst));
    burst.type     = DMR_BURST_TYPE_VOICE;
    burst.timeslot = (uint8_t)ctx->slot;

    dmr_burst_clear(burst.raw);

    /* Set voice SYNC (inbound MS→BS voice) */
    dmr_burst_set_sync(burst.raw, DMR_SYNC_MS_VOICE);
    
    /* Write INFO_1 and INFO_2 */
    dmr_burst_set_info(burst.raw, info);
   // dmr_burst_set_info2(burst.raw, info1);

    /* Write EMB field (overrides SYNC position on voice bursts B..F) */
    if (burst_pos != SF_BURST_A) {
        
        uint8_t emb_data7 = (uint8_t)(((ctx->colour_code & 0x0Fu) << 3)
                                       | (0u << 2)   /* PI=0 (not encrypted) */
                                       | (lcss & 0x03u));
        uint16_t qr_cw = qr_16_7_encode(emb_data7);
        dmr_burst_set_emb(burst.raw,
                          ctx->colour_code,
                          0u,     /* PI=0 (not encrypted) */
                          lcss,
                          qr_cw & 0x1FFu,
                          (burst_pos <= SF_BURST_E) ? lc_frag[burst_pos-1] : NULL);

     }

       /* Advance superframe position — wrap F→B */
    ctx->sf_burst_idx = (uint8_t)(burst_pos < SF_BURST_F
                                     ? burst_pos + 1u
                                     : SF_BURST_A);

    return ccl_submit_burst(ctx, &burst, DMR_MAC_PRIORITY_HIGH, true);
}

dmr_err_t ccl_voice_tx_idle(ccl_voice_ctx_t *ctx)
{
    dmr_burst_t burst;
    /* outbound=false for MS→BS idle */
    llc_idle_burst_build(&burst, ctx->colour_code, false, ctx->slot);

    DMR_LOGT("[CCL S%d] TX Idle", ctx->slot);
    return ccl_submit_burst(ctx, &burst, DMR_MAC_PRIORITY_LOW, false);
}

/* =========================================================================
 * SECTION F — CSBK TX builders (use LLC layer)
 * ========================================================================= */

dmr_err_t ccl_voice_tx_uu_v_req(ccl_voice_ctx_t *ctx, uint32_t dst_id)
{
    uint8_t svc = 0u;
    DMR_SVC_SET_EMERGENCY(svc, ctx->call.emergency ? 1u : 0u);

    dmr_burst_t burst;
    llc_csbk_uu_v_req_build(&burst, svc, dst_id, ctx->my_radio_id,
                             ctx->colour_code, ctx->slot);

    DMR_LOGD("[CCL S%d] TX UU_V_REQ dst=0x%06X", ctx->slot, dst_id);
    ctx->stats.calls_originated++;
    return ccl_submit_burst(ctx, &burst, DMR_MAC_PRIORITY_NORMAL, false);
}

dmr_err_t ccl_voice_tx_uu_ans_rsp(ccl_voice_ctx_t *ctx,
                                    uint32_t dst_id,
                                    uint8_t  response)
{
    uint8_t svc = 0u;
    dmr_burst_t burst;
    llc_csbk_uu_ans_rsp_build(&burst, svc, response, dst_id,
                               ctx->my_radio_id, ctx->colour_code, ctx->slot);

    DMR_LOGD("[CCL S%d] TX UU_ANS_RSP dst=0x%06X resp=%u", ctx->slot, dst_id, response);
    return ccl_submit_burst(ctx, &burst, DMR_MAC_PRIORITY_HIGH, true);
}

/* =========================================================================
 * SECTION G — handlers (use llc_rx_dispatch)
 * ========================================================================= */

/*
 * Determine if an incoming call's destination address is addressed to us.
 * Accepts: our radio ID, any subscribed group ID, or the all-call address.
 */
static bool ccl_is_addressed_to_us(const ccl_voice_ctx_t *ctx,
                                    uint32_t dst_id,
                                    bool     is_group)
{
    if (!is_group) {
        return dst_id == ctx->my_radio_id;
    }
    /* Group call — check subscription list */
    if (dst_id == 0xFFFFFFu) return true;   /* All-call */
    for (uint8_t i = 0; i < ctx->n_subscribed_groups; i++) {
        if (ctx->subscribed_groups[i] == dst_id) return true;
    }
    return false;
}

dmr_err_t ccl_voice_rx_lc_header(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst)
{
    /* Use LLC dispatch to parse the burst */
    llc_rx_result_t rx;
    llc_rx_dispatch(burst, &rx);
    
if(rx.src_id== ctx->my_radio_id)
{
    DMR_LOGT("[CCL S%d] my own packet ",
                 ctx->slot);
        return DMR_OK;
}
    if (rx.type != LLC_RX_VOICE_LC_HDR) {
        DMR_LOGW("[CCL S%d] rx_lc_header: unexpected type %d", ctx->slot, rx.type);
        return DMR_ERR_INVALID_PARAM;
    }

    if (!rx.crc_ok) {
        ctx->stats.lc_errors++;
        DMR_LOGW("[CCL S%d] Voice LC Header CRC failed — ignoring", ctx->slot);
        printf("\n******ERROR \n" );
        return DMR_ERR_CRC;
    }

    uint8_t  flco   = rx.opcode;
    uint32_t dst_id = rx.dst_id;
    uint32_t src_id = rx.src_id;
    uint8_t  svc    = rx.svc;

    bool is_group = (flco == DMR_FLCO_GRP_V_CH_USR);
    bool emergency = (DMR_SVC_GET_EMERGENCY(svc) != 0);

    if (!ccl_is_addressed_to_us(ctx, dst_id, is_group)) {
        DMR_LOGT("[CCL S%d] Voice LC Hdr not addressed to us (dst=0x%06X)",
                 ctx->slot, dst_id);
        return DMR_OK;
    }

    /* Initialise call context */
    if (is_group) {
        ccl_call_ctx_init_group(&ctx->call, src_id, dst_id,
                                ctx->colour_code, emergency);
    } else {
        ccl_call_ctx_init_individual(&ctx->call, src_id, dst_id,
                                     ctx->colour_code, emergency);
    }
    ctx->call.priority = DMR_SVC_GET_PRIORITY(svc);
    ctx->call.encrypted = (DMR_SVC_GET_PRIVACY(svc) != 0);
    ctx->call.lc_valid  = true;
    memcpy(&ctx->call.full_lc, rx.body, sizeof(ctx->call.full_lc));

    ctx->stats.calls_received++;
    ctx->stats.bursts_rx++;
    ctx->call.last_burst_us = dmr_time_now_us();
    ctx->call.burst_count   = 1;

    DMR_LOGI("[CCL S%d] Incoming voice call from 0x%06X → 0x%06X%s",
             ctx->slot, src_id, dst_id, emergency ? " [EMERGENCY]" : "");

    ccl_voice_set_state(ctx, CCL_STATE_RECEIVING);

    if (ctx->on_call_start)
        ctx->on_call_start(ctx, &ctx->call);

    return DMR_OK;
}



dmr_err_t ccl_voice_rx_voice_burst(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst)
{
    ctx->stats.bursts_rx++;
    ctx->call.burst_count++;
    ctx->call.last_burst_us = dmr_time_now_us();
     if (dmr_burst_is_voice(burst->raw)) {
           DMR_LOGT("[CCL S%d] Voice burst Pack A rcvd",
                     ctx->slot);
                      ctx->sfVoice.index=0;
        dmr_burst_get_info(burst->raw, &ctx->sfVoice.data[ctx->sfVoice.index]);
    ctx->sfVoice.missed_frame=false;


/* data should not be sent to ambe for now.the receiver does not know this packet is 
 * for this ms or not complete lc is received at the end of superframe.then it should be 
 forwarded to ambe.it is therefore stored in ctx->sfVoice.data -added by developer */
 


     }
     else if(burst->type==DMR_BURST_TYPE_SYNTHETIC_EVT)
     {
                  
          DMR_LOGT("[CCL S%d] Synthetic event. Frame missed reset lc frag and sfVoice index",
                     ctx->slot);
         
          ctx->call.emblc_frag_mask = 0;//lc frag lost.we need to reset the counter.
          ctx->sfVoice.index=0;
          ctx->sfVoice.missed_frame=true;
           /*if a syntheic frame is received the ongoing emblc collection has to stop and reset.
           till new packet a is received there is no point of collecting the rest of the lc frags.*/


         
     }
  else if( ctx->sfVoice.missed_frame==false)   /* Extract EMB field from voice burst B..F */

    {
        uint8_t  cc, pi, lcss;
        uint16_t qr16;
        dmr_burst_get_emb_ctrl(burst->raw, &cc, &pi, &lcss, &qr16);

        if (cc != ctx->colour_code) {
            DMR_LOGT("[CCL S%d] Voice burst CC mismatch rx cc=%u cfg cc=%u %u",
                     ctx->slot, cc, ctx->colour_code, ctx->call.burst_count);
            return DMR_OK;
        }
        else
        {
             DMR_LOGT("[CCL S%d] Voice burst  rx=%u cfg=%u %u", ctx->slot, cc, ctx->colour_code, ctx->call.burst_count);
        }

                /* Collect embedded LC fragment for late entry or RECEIVING state */

        if (lcss != DMR_LCSS_SINGLE) {
            uint8_t lc_frag[4];
            dmr_burst_get_emb_lc(burst->raw, lc_frag);
            
            
                switch (lcss) {
                    case DMR_LCSS_FIRST:   
                                ctx->sfVoice.index=DMR_AMBE_FRAME_BYTES*3;
                                dmr_burst_get_info(burst->raw, &ctx->sfVoice.data[ctx->sfVoice.index]);
                        break;
                    case DMR_LCSS_CONT:
                                       if(ctx->call.emblc_frag_mask==0)
                                       {
                                             ctx->sfVoice.index=DMR_AMBE_FRAME_BYTES*3*2;
                                            dmr_burst_get_info(burst->raw, &ctx->sfVoice.data[ctx->sfVoice.index]);
                                       }
                                       else{
                                             ctx->sfVoice.index=DMR_AMBE_FRAME_BYTES*3*3;
                                            dmr_burst_get_info(burst->raw, &ctx->sfVoice.data[ctx->sfVoice.index]);
                                       }
                
                                    break;
                    case DMR_LCSS_LAST:  
                                         ctx->sfVoice.index=DMR_AMBE_FRAME_BYTES*3*4;
                                        dmr_burst_get_info(burst->raw, &ctx->sfVoice.data[ctx->sfVoice.index]);
                                    break;
                    default: 
                    break;
                }
	ccl_voice_process_emblc(ctx, lcss, lc_frag);


        }

        else{
            
                                               
            if(ccl_voice_get_state(ctx )==CCL_STATE_RECEIVING ) //check for burst F. DMR_LCSS_SINGLE is lcss for burst F
            {   
                 ctx->sfVoice.index=DMR_AMBE_FRAME_BYTES*3*5; //data from burst F Extracted
                 dmr_burst_get_info(burst->raw, &ctx->sfVoice.data[ctx->sfVoice.index]);

                DMR_LOGT("[CCL S%d] All frames recevied sending superFrame to ambe",
                     ctx->slot);

                if (ctx->on_ambe_rx) /* Extracted AMBE frames from superframe  can be sent to ambe*/
                    ctx->on_ambe_rx(ctx, &ctx->sfVoice, ctx->call.src_id);
            }
            
            

        }


    }
    else{
         DMR_LOGT("[CCL S%d] unusable Frame till Sf ends x",
                     ctx->slot);
    }

    return DMR_OK;
}


dmr_err_t ccl_voice_rx_synthetic_terminator(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst)
{
   
    DMR_LOGI("[CCL S] Call ended (Terminator received) from MAC.synthetic");

    ccl_voice_disarm_timer(&ctx->tmr_hangtime);

    if (ctx->on_call_end)
        ctx->on_call_end(ctx, &ctx->call);

    ccl_voice_set_state(ctx, CCL_STATE_HANGTIME);
    ccl_voice_arm_timer(&ctx->tmr_hangtime, CCL_T_HANGTIME_MS);

    return DMR_OK;
}


dmr_err_t ccl_voice_rx_terminator(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst)
{
    llc_rx_result_t rx;
    llc_rx_dispatch(burst, &rx);

    if (rx.type != LLC_RX_TERMINATOR_LC) {
        return DMR_ERR_INVALID_PARAM;
    }

    ctx->stats.bursts_rx++;

    /* Verify this terminator belongs to our active call */
    if (rx.dst_id != ctx->call.dst_id || rx.src_id != ctx->call.src_id) {
        DMR_LOGT("[CCL S%d] Terminator for different call — ignoring", ctx->slot);
        return DMR_OK;
    }

    DMR_LOGI("[CCL S%d] Call ended (Terminator received) from 0x%06X",
             ctx->slot, ctx->call.src_id);

    ccl_voice_disarm_timer(&ctx->tmr_hangtime);

    if (ctx->on_call_end)
        ctx->on_call_end(ctx, &ctx->call);

    ccl_voice_set_state(ctx, CCL_STATE_HANGTIME);
    ccl_voice_arm_timer(&ctx->tmr_hangtime, CCL_T_HANGTIME_MS);

    return DMR_OK;
}

dmr_err_t ccl_voice_rx_csbk(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst)
{
    llc_rx_result_t rx;
    llc_rx_dispatch(burst, &rx);

    if (rx.type != LLC_RX_CSBK) return DMR_ERR_INVALID_PARAM;

    if (!rx.crc_ok) {
        ctx->stats.lc_errors++;
        DMR_LOGW("[CCL S%d] CSBK CRC failed opcode=0x%02X", ctx->slot, rx.opcode);
        printf("\n******ERROR \n" );
        return DMR_ERR_CRC;
    }

    uint8_t  opcode = rx.opcode;
    uint32_t src_id = rx.src_id;
    uint32_t dst_id = rx.dst_id;

    DMR_LOGD("[CCL S%d] RX CSBK opcode=0x%02X src=0x%06X dst=0x%06X",
             ctx->slot, opcode, src_id, dst_id);

    switch (opcode) {

    case DMR_CSBKO_UU_V_REQ:
        /* Incoming individual call request addressed to us */
        if (dst_id != ctx->my_radio_id) break;
        DMR_LOGI("[CCL S%d] UU_V_REQ from 0x%06X — responding PROCEED", ctx->slot, src_id);
        ccl_call_ctx_init_individual(&ctx->call, src_id, dst_id,
                                     ctx->colour_code, false);
        ctx->call.lc_valid = false;
        ctx->stats.calls_received++;
        ccl_voice_tx_uu_ans_rsp(ctx, src_id, DMR_UU_ANS_PROCEED);
        ccl_voice_set_state(ctx, CCL_STATE_RECEIVING);
        break;

    case DMR_CSBKO_UU_ANS_RSP:
        /* Response to our UU_V_REQ (OACSU individual call) */
        if (ctx->state != CCL_STATE_UU_REQ_WAIT) break;
        if (src_id != ctx->call.dst_id)           break;

        ccl_voice_disarm_timer(&ctx->tmr_answer);

        /* Check answer response byte — from LLC body byte 3 */
        {
            /* re-parse raw body for answer response byte */
            uint8_t ans = rx.body[3];
            bool proceed = (ans == 0x40u);
            if (proceed) {
                DMR_LOGI("[CCL S%d] UU_ANS_RSP PROCEED from 0x%06X — starting TX",
                         ctx->slot, src_id);
                ccl_voice_set_state(ctx, CCL_STATE_TX_LC_HEADER);
                ccl_voice_tx_lc_header(ctx);
            } else {
                DMR_LOGI("[CCL S%d] UU_ANS_RSP DENY from 0x%06X", ctx->slot, src_id);
                ctx->stats.calls_denied++;
                ccl_call_ctx_clear(&ctx->call);
                ccl_voice_set_state(ctx, CCL_STATE_IDLE);
            }
        }
        break;

    case DMR_CSBKO_CALL_ALERT:
        if (dst_id != ctx->my_radio_id) break;
        DMR_LOGI("[CCL S%d] Call Alert from 0x%06X", ctx->slot, src_id);
        /* Auto-acknowledge for now */
        ccl_voice_tx_uu_ans_rsp(ctx, src_id, DMR_UU_ANS_PROCEED);
        break;

    case DMR_CSBKO_ACK_RSP:
        ccl_voice_disarm_timer(&ctx->tmr_callalert);
        DMR_LOGI("[CCL S%d] ACK_RSP from 0x%06X", ctx->slot, src_id);
        break;

    default:
        DMR_LOGT("[CCL S%d] Unhandled CSBK opcode=0x%02X", ctx->slot, opcode);
        break;
    }

    return DMR_OK;
}

/* =========================================================================
 * SECTION H — Embedded LC fragment reassembly
 * ETSI TS 102 361-1, Clause 7.1.3 / B.2.1
 *
 * Four 18-bit fragments arrive in EMB fields of bursts B..E.
 * LCSS sequence: First(01) → Cont(11) → Cont(11) → Last(10)
 * When all 4 fragments received, reassemble into 72-bit Full LC.
 * ========================================================================= */

dmr_err_t ccl_voice_process_emblc(ccl_voice_ctx_t *ctx,
                                   uint8_t          lcss,
                                   const uint8_t   *frag4)
{
    /* Map LCSS to fragment slot */
    uint8_t frag_idx = 0;
    switch (lcss) {
    case DMR_LCSS_FIRST: frag_idx = 0; break;
    case DMR_LCSS_CONT:/*frag_idx  (uint8_t)
                           ((ctx->call.emblc_frag_mask & 0x01u) ? 1u : 2u);*/
                           
                           if(ctx->call.emblc_frag_mask==0)
                           {
                            frag_idx=1;
                           // ctx->call.emblc_frag_mask=0;
                           }
                           else{
                               frag_idx=2;
                              // ctx->call.emblc_frag_mask=0;
                           }
    
                         break;
    case DMR_LCSS_LAST:  frag_idx = 3;
                           // ctx->call.emblc_frag_mask=0;
                            break;
    default: return DMR_OK;  /* Single frag = null message, ignore */
    }

    /* Store fragment (3 bytes / 24 bits per slot, 18 bits used) */
   
    memcpy(ctx->call.emblc_frags[frag_idx], frag4, 4u);
   /* printf("\n== RX %d\n",frag_idx);
     for(int i=0;i<3;i++)
    printf("%x ",ctx->call.emblc_frags[frag_idx][i]);
    printf(",\n");*/
    if(frag_idx==0)
    ctx->call.emblc_frag_mask=0;
    else
    ctx->call.emblc_frag_mask = (uint8_t)(frag_idx);

    /* Check if all 4 fragments collected */
    if (ctx->call.emblc_frag_mask != 3) return DMR_OK;
 // if (frag_idx!= 9) return DMR_OK;

    /* Reassemble 72-bit Full LC from 4 × 18-bit fragments:
     *   frag[0] = bits [71:54]  (3 bytes, use upper 18 bits)
     *   frag[1] = bits [53:36]
     *   frag[2] = bits [35:18]
     *   frag[3] = bits [17:0]
     *
     * Each fragment occupies 3 bytes (24 bits) but only 18 bits are payload.
     * The extra 6 bits (upper bits of byte 0) are the Golay FEC check bits.
     * Extract the 18 payload bits and concatenate into 9 bytes (72 bits).
     */
    uint8_t lc12[12];
    memset(lc12, 0, sizeof(lc12));

    /* Reassemble via BPTC(128) de-interleave + Hamming(16,11,4) decode per
     * row (ETSI TS 102 361-1 Annex B.2.1/B.3.4/B.3.11) — see emblc_bptc_decode()
     * in dmr_fec.c. lc12 is only meaningful if it returns true; false means
     * either a row had 2+ bit errors (uncorrectable) or the checksum
     * didn't match, and must not be used. */

    int corrected_rows = 0;
    bool lc_ok = emblc_bptc_decode(ctx->call.emblc_frags, lc12, &corrected_rows);

    ctx->call.emblc_frag_mask = 0;   /* fragment slots consumed either way — start fresh next time */

    if (!lc_ok) {
        /* Either a row had 2+ bit errors (Hamming(16,11,4) couldn't fix it —
         * a certain signal) or the checksum didn't match. lc12 is not
         * trustworthy — discard this reassembly attempt rather than acting
         * on corrupted data. Wait for the next LCSS_FIRST to try again. */
        DMR_LOGW("[CCL S%d] emblc_bptc_decode failed (uncorrectable row or checksum mismatch) — discarding late-entry LC",
                 ctx->slot);
        return DMR_OK;
    }
    if (corrected_rows > 0) {
        DMR_LOGD("[CCL S%d] emblc_bptc_decode corrected %d row(s)", ctx->slot, corrected_rows);
    }

    /* Update call context Full LC */
    memcpy(&ctx->call.full_lc, lc12, sizeof(ctx->call.full_lc));

    /* Parse addresses from reassembled LC */
    uint8_t  flco;
    uint32_t dst_id, src_id;
    uint8_t  svc;
    llc_full_lc_parse(lc12, &flco, &dst_id, &src_id, &svc);

    ctx->call.lc_valid = true;

    if (src_id != ctx->my_radio_id) 
   {
        bool is_group = (flco == DMR_FLCO_GRP_V_CH_USR);
        if (ccl_is_addressed_to_us(ctx, dst_id, is_group)) {
            ctx->call.src_id = src_id;
            ctx->call.dst_id = dst_id;
            ctx->stats.late_entries++;
            DMR_LOGI("[CCL S%d] — LC  reassembled from 0x%06X → 0x%06X",
                     ctx->slot, src_id, dst_id);

            if (ctx->on_call_start && ctx->state !=CCL_STATE_RECEIVING)
            {
                ccl_voice_set_state(ctx, CCL_STATE_RECEIVING);
                ctx->on_call_start(ctx, &ctx->call);
            }
        }
        else{
             ccl_voice_set_state(ctx, CCL_STATE_IDLE);
              DMR_LOGI("[CCL S%d] — LC not ours 1 0x%06X → 0x%06X",
                     ctx->slot, src_id, dst_id);
        }
    }
    else{
         ccl_voice_set_state(ctx, CCL_STATE_IDLE);
                DMR_LOGI("[CCL S%d] — LC not ours 2 0x%06X → 0x%06X",
                     ctx->slot, src_id, dst_id);
    }

    return DMR_OK;
}

/* =========================================================================
 * SECTION I — Incoming burst router
 * ========================================================================= */

static void ccl_route_rx_burst(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst)
{
    /* =================================================================
     * Category 1 — Synthetic events from MAC
     *
     * MAC injects these through mq_rx_voice when it detects conditions
     * that have no wire representation: missed B-F burst positions,
     * call-end without Terminator (timeout), CACH AT Busy→Idle (Tier
     * II/III hang-time done), and definitive end after hangover window.
     * raw[] is zeroed; only synth_event and synth_pos are meaningful.
     * ================================================================= */
    if (burst->type == DMR_BURST_TYPE_SYNTHETIC_EVT) {
        mac_synth_event_t evt = (mac_synth_event_t)burst->synth_event;
        ccl_voice_state_t s   = ccl_voice_get_state(ctx);

        switch (evt) {

        case MAC_SYNTH_EVT_VOICE_BURST_LOST:
            /* MAC's B-F burst window expired — one burst was not received.
             * burst->synth_pos holds the missed position (1=B..5=F).
             * Insert a PLC / comfort-noise frame so the vocoder pipeline
             * stays synchronised. Only meaningful while RECEIVING. */
            if (s == CCL_STATE_RECEIVING || s == CCL_STATE_LATE_ENTRY || s == CCL_STATE_HANGTIME) {
                DMR_LOGD("[CCL S%d] Voice burst pos=%u missed — inserting PLC",
                         ctx->slot, burst->synth_pos);
                /* TODO MOD-09: call vocoder PLC for burst->synth_pos.
                 * For now deliver a zeroed 9-byte AMBE frame to keep the
                 * frame counter advancing without emitting noise. */
                 ccl_voice_rx_voice_burst(ctx,burst);
               /* if (ctx->on_voice_frame) {
                    uint8_t silence[9] = {0};
                    ctx->on_voice_frame(ctx, silence, 9u);
                }*/
            }
            break;

        case MAC_SYNTH_EVT_CALL_ENDED_TERMINATOR:
            /* Terminator LC burst was received and processed; MAC has
             * already delivered the real Terminator burst before this
             * synthetic event, so ccl_voice_rx_terminator() will have
             * handled the LC teardown. This event is therefore only
             * relevant if we somehow missed the real burst processing
             * (e.g. late entry) — guard against double-teardown. */
            if (s == CCL_STATE_RECEIVING || s == CCL_STATE_LATE_ENTRY ||
                s == CCL_STATE_HANGTIME) {
                DMR_LOGD("[CCL S%d] Synthetic TERMINATOR confirm — "
                         "ensuring call torn down", ctx->slot);
                ccl_voice_disarm_timer(&ctx->tmr_hangtime);
                if (ctx->on_call_end) ctx->on_call_end(ctx, &ctx->call);
                ccl_call_ctx_clear(&ctx->call);
                ccl_voice_set_state(ctx, CCL_STATE_IDLE);
            }
            break;

        case MAC_SYNTH_EVT_CALL_ENDED_CACH_IDLE:
            /* Tier II/III: BS has set AT=Idle — hang-time is done and the
             * inbound channel is free. Treat identically to a Terminator
             * for state-machine purposes; log a distinct reason so it is
             * auditable. */
            if (s == CCL_STATE_RECEIVING || s == CCL_STATE_LATE_ENTRY ||
                s == CCL_STATE_HANGTIME) {
                DMR_LOGD("[CCL S%d] CACH AT Busy→Idle — call ended "
                         "(BS hang-time done)", ctx->slot);
                ccl_voice_disarm_timer(&ctx->tmr_hangtime);
                if (ctx->on_call_end) ctx->on_call_end(ctx, &ctx->call);
                ccl_call_ctx_clear(&ctx->call);
                ccl_voice_set_state(ctx, CCL_STATE_IDLE);
            }
            break;

        case MAC_SYNTH_EVT_CALL_ENDED_TIMEOUT:
            /* Superframe watchdog expired — peer likely dropped without
             * completing its transmission. Tear down call state and mark
             * end as abnormal (no LC teardown info available). The hangover
             * window is now running in MAC; we may still receive CALL_GONE
             * if no resumption occurs. */
            if (s == CCL_STATE_RECEIVING || s == CCL_STATE_LATE_ENTRY) {
                DMR_LOGW("[CCL S%d] Voice superframe timeout — no Terminator "
                         "received; tearing down call (abnormal end)", ctx->slot);
                ccl_voice_disarm_timer(&ctx->tmr_hangtime);
                if (ctx->on_call_end) ctx->on_call_end(ctx, &ctx->call);
                ccl_call_ctx_clear(&ctx->call);
                ccl_voice_set_state(ctx, CCL_STATE_IDLE);
            }
            break;

        case MAC_SYNTH_EVT_CALL_GONE:
            /* Hangover window expired — definitive end. If CALL_ENDED_TIMEOUT
             * already tore down the state, this is a no-op. If somehow we
             * are still in a voice state (e.g. HANGTIME from a Terminator
             * that arrived during hangover), force cleanup. */
            if (s != CCL_STATE_IDLE) {
                DMR_LOGD("[CCL S%d] Voice hangover window expired — "
                         "forced final cleanup", ctx->slot);
                ccl_voice_disarm_timer(&ctx->tmr_hangtime);
                if (ctx->on_call_end) ctx->on_call_end(ctx, &ctx->call);
                ccl_call_ctx_clear(&ctx->call);
                ccl_voice_set_state(ctx, CCL_STATE_IDLE);
            }
            break;

        default:
            DMR_LOGT("[CCL S%d] Unknown synthetic event %u — ignoring",
                     ctx->slot, burst->synth_event);
            break;
        }
        return;
    }

    /* =================================================================
     * Category 2 — Voice burst A (genuine SYNC pattern)
     *
     * dmr_burst_is_voice() matches only burst A (EMB overwrites SYNC
     * on B-F so they never match). MAC has already anchored the
     * superframe tracking state on seeing this burst.
     * ================================================================= */
    if (dmr_burst_is_voice(burst->raw)) {
        ccl_voice_state_t s = ccl_voice_get_state(ctx);
        if (s == CCL_STATE_RECEIVING || s == CCL_STATE_LATE_ENTRY) {
            ccl_voice_rx_voice_burst(ctx, burst);
        } else {
            /* Voice burst A while not in an active call — late entry */
            ccl_voice_set_state(ctx, CCL_STATE_LATE_ENTRY);
            ctx->call.emblc_frag_mask = 0;
            ccl_voice_rx_voice_burst(ctx, burst);
        }
        return;
    }

    /* =================================================================
     * Category 3 — Data bursts (have SYNC + Data Type field)
     *
     * Verify colour code first, then dispatch by Data Type.
     * ================================================================= */
    if (dmr_burst_is_data(burst->raw)) {
        uint8_t cc = dmr_burst_get_cc(burst->raw);
        if (cc != ctx->colour_code) {
            DMR_LOGT("[CCL S%d] Data burst CC mismatch rx=%u cfg=%u",
                     ctx->slot, cc, ctx->colour_code);
            return;
        }

        uint8_t dtype = dmr_burst_get_dtype(burst->raw);

        switch (dtype) {
        case DMR_DTYPE_VOICE_LC_HEADER:
            ccl_voice_rx_lc_header(ctx, burst);
            break;

        case DMR_DTYPE_TERMINATOR_LC:
            /* Deliver the real Terminator burst first (it carries LC info
             * needed for call teardown), then MAC will inject a synthetic
             * CALL_ENDED_TERMINATOR event as a confirmation guard. */
            ccl_voice_rx_terminator(ctx, burst);
            break;

        case DMR_DTYPE_CSBK:
            ccl_voice_rx_csbk(ctx, burst);
            break;

        case DMR_DTYPE_IDLE:
            /* Idle burst during hangtime signals end of call hangtime */
            if (ctx->state == CCL_STATE_HANGTIME) {
                DMR_LOGD("[CCL S%d] Idle burst in hangtime — end of call",
                         ctx->slot);
                ccl_voice_disarm_timer(&ctx->tmr_hangtime);
                if (ctx->on_call_end) ctx->on_call_end(ctx, &ctx->call);
                ccl_call_ctx_clear(&ctx->call);
                ccl_voice_set_state(ctx, CCL_STATE_IDLE);
            }
            break;

        default:
            DMR_LOGT("[CCL S%d] Unhandled dtype=0x%02X", ctx->slot, dtype);
            break;
        }
        return;
    }

    /* =================================================================
     * Category 4 — Voice bursts B-F (EMB replaces SYNC — no SYNC
     * pattern, not a data burst)
     *
     * MAC's superframe tracker inferred these as B-F and routed them
     * here. ccl_voice_rx_voice_burst() already handles the A vs B-F
     * distinction internally via dmr_burst_is_voice(). Only process
     * while RECEIVING or LATE_ENTRY; discard otherwise (could be a
     * stale burst from a call we didn't catch the header for).
     * ================================================================= */
    {
        ccl_voice_state_t s = ccl_voice_get_state(ctx);
        if (s == CCL_STATE_RECEIVING || s == CCL_STATE_LATE_ENTRY) {
            ccl_voice_rx_voice_burst(ctx, burst);
        } else {
            DMR_LOGT("[CCL S%d] B-F voice burst in state %s — discarding",
                     ctx->slot, CCL_STATE_NAMES[s]);
        }
    }
}

/* =========================================================================
 * SECTION J — Event dispatcher
 * ========================================================================= */

static void ccl_dispatch_event(ccl_voice_ctx_t *ctx, const ccl_voice_event_t *evt)
{
    switch (evt->type) {

    /* ---- PTT PRESS ---- */
    case CCL_EVT_PTT_PRESS: {
        ccl_voice_state_t s = ccl_voice_get_state(ctx);
        if (s != CCL_STATE_IDLE && s != CCL_STATE_HANGTIME) {
            DMR_LOGW("[CCL S%d] PTT press ignored in state %s",
                     ctx->slot, CCL_STATE_NAMES[s]);
            break;
        }
        ccl_voice_disarm_timer(&ctx->tmr_hangtime);

        uint32_t dst_id    = evt->u.ptt.dst_id;
        bool     emergency = evt->u.ptt.emergency;

        if (evt->u.ptt.call_type == DMR_CALL_TYPE_GROUP) {
            ccl_call_ctx_init_group(&ctx->call, ctx->my_radio_id, dst_id,
                                    ctx->colour_code, emergency);
            ccl_voice_set_state(ctx, CCL_STATE_TX_LC_HEADER);
            ccl_voice_tx_lc_header(ctx);
        } else {
            /* Individual call — OACSU: send UU_V_REQ first */
            ccl_call_ctx_init_individual(&ctx->call, ctx->my_radio_id, dst_id,
                                         ctx->colour_code, emergency);
            ccl_voice_set_state(ctx, CCL_STATE_UU_REQ_WAIT);
            ccl_voice_tx_uu_v_req(ctx, dst_id);
            ccl_voice_arm_timer(&ctx->tmr_answer, CCL_T_ANSWER_RESPONSE_MS);
        }
        break;
    }

    /* ---- PTT RELEASE ---- */
    case CCL_EVT_PTT_RELEASE: {
        ccl_voice_state_t s = ccl_voice_get_state(ctx);
        if (s == CCL_STATE_TRANSMITTING) {
            /* Finish the current superframe — send remaining bursts up to F even if it is just burst f left to transfer*/
             ccl_voice_set_state(ctx, CCL_STATE_TX_TERMINATOR_ONCE_SF_FINISHED);

        } else if (s == CCL_STATE_TX_LC_HEADER) {
            /* Released before first voice burst — send Terminator immediately */
            ccl_voice_set_state(ctx, CCL_STATE_TX_TERMINATOR);
            ccl_voice_tx_terminator(ctx);
        }
        break;
    }

    /* ---- TX CONFIRMATION ---- */
    case CCL_EVT_TX_CONF: {
        const dmr_mac_tx_conf_t *conf = &evt->u.tx_conf;
        if (conf->result != DMR_MAC_TX_OK) {
            DMR_LOGW("[CCL S%d] TX conf non-OK result=%d req_id=%u",
                     ctx->slot, conf->result, conf->req_id);
            ctx->stats.tx_aborts++;
        }

        ccl_voice_state_t s = ccl_voice_get_state(ctx);
        switch (s) {
        case CCL_STATE_TX_LC_HEADER:
            /* LC Header sent — transition to TRANSMITTING */
            ccl_voice_set_state(ctx, CCL_STATE_TRANSMITTING);
            ctx->sf_burst_idx = SF_BURST_A;
            ccl_voice_tx_next_burst(ctx);
            break;

        case CCL_STATE_TRANSMITTING:
            /* Continue sending voice bursts */
            ccl_voice_tx_next_burst(ctx);
            break;
        case CCL_STATE_TX_TERMINATOR_ONCE_SF_FINISHED:
             if(ctx->sf_burst_idx != SF_BURST_F)
        {
          ccl_voice_tx_next_burst(ctx);
        }
          else{
              ccl_voice_tx_next_burst(ctx); //sending last packet frame F and changing state to terminator
             ccl_voice_set_state(ctx, CCL_STATE_TX_SF_FINISHED_TERMINATOR);
          }
        break;
        case CCL_STATE_TX_SF_FINISHED_TERMINATOR:
          ccl_voice_set_state(ctx, CCL_STATE_TX_TERMINATOR);
          ccl_voice_tx_terminator(ctx);
          break;
        case CCL_STATE_TX_TERMINATOR:
      //  ccl_voice_set_state(ctx, CCL_STATE_TX_TERMINATOR);
            
            /* Terminator sent — enter hangtime */
            ccl_voice_set_state(ctx, CCL_STATE_HANGTIME);
            ccl_voice_arm_timer(&ctx->tmr_hangtime, CCL_T_HANGTIME_MS);
            /***idle to be sent if peer to peer in tier 2 and 3**/
          //  ccl_voice_tx_idle(ctx); // idle burst to be formed and managed by mac itself
            break;

        case CCL_STATE_HANGTIME:
            /* Idle burst confirmed — keep sending idle until hangtime expires */
           // ccl_voice_arm_timer(ctx->tfd_hangtime, CCL_T_HANGTIME_MS);
            break;

        default:
            break;
        }
        break;
    }

    /* ---- TX ABORTED ---- */
    case CCL_EVT_TX_ABORTED:
        DMR_LOGW("[CCL S%d] TX aborted — returning to IDLE", ctx->slot);
        ctx->stats.tx_aborts++;
        ccl_call_ctx_clear(&ctx->call);
        ccl_voice_set_state(ctx, CCL_STATE_IDLE);
        break;

    /* ---- BURST RECEIVED ---- */
    case CCL_EVT_BURST_RECEIVED:
        ccl_route_rx_burst(ctx, &evt->u.burst);
        break;

    /* ---- TIMER HANGTIME ---- */
    case CCL_EVT_TIMER_HANGTIME:
        DMR_LOGD("[CCL S%d] T_Hangtime expired", ctx->slot);
        if (ctx->on_call_end && ctx->state == CCL_STATE_HANGTIME)
            ctx->on_call_end(ctx, &ctx->call);
        ccl_call_ctx_clear(&ctx->call);
        ccl_voice_set_state(ctx, CCL_STATE_IDLE);
        break;

    /* ---- TIMER ANSWER RESPONSE ---- */
    case CCL_EVT_TIMER_ANSWER:
        DMR_LOGW("[CCL S%d] T_AnswerResponse expired — call denied", ctx->slot);
        ctx->stats.calls_denied++;
        ccl_call_ctx_clear(&ctx->call);
        ccl_voice_set_state(ctx, CCL_STATE_IDLE);
        break;

    /* ---- TIMER CALL ALERT ---- */
    case CCL_EVT_TIMER_CALLALERT:
        DMR_LOGW("[CCL S%d] T_CallAlert expired", ctx->slot);
        ccl_call_ctx_clear(&ctx->call);
        ccl_voice_set_state(ctx, CCL_STATE_IDLE);
        break;

    /* ---- TIMER GRANT REJECTED ---- */
    case CCL_EVT_TIMER_GRANTREJ:
        DMR_LOGD("[CCL S%d] T_GrantRejected backoff expired — retry", ctx->slot);
        ccl_voice_tx_lc_header(ctx);
        break;

    /* ---- EMERGENCY ---- */
    case CCL_EVT_EMERGENCY:
        DMR_LOGI("[CCL S%d] EMERGENCY button pressed", ctx->slot);
        ccl_call_ctx_init_group(&ctx->call, ctx->my_radio_id, 0xFFFFFFu,
                                ctx->colour_code, true);
        ctx->stats.calls_emergency++;
        ccl_voice_set_state(ctx, CCL_STATE_TX_LC_HEADER);
        ccl_voice_tx_lc_header(ctx);
        break;

    /* ---- SHUTDOWN ---- */
    case CCL_EVT_SHUTDOWN:
        DMR_LOGI("[CCL S%d] Shutdown event received", ctx->slot);
        ctx->running = false;
        break;

    default:
        DMR_LOGW("[CCL S%d] Unknown event type %d", ctx->slot, evt->type);
        break;
    }
}

/* =========================================================================
 * SECTION K — CCL worker thread
 * ========================================================================= */

void *ccl_voice_thread(void *arg)
{
    ccl_voice_ctx_t *ctx = (ccl_voice_ctx_t *)arg;
    struct epoll_event events[CCL_EPOLL_MAX_EVENTS];

    DMR_LOGI("[CCL S%d] Worker thread started (radio_id=0x%06X cc=%u)",
             ctx->slot, ctx->my_radio_id, ctx->colour_code);

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        DMR_LOGE("[CCL S%d] epoll_create1 failed: %s", ctx->slot, strerror(errno));
        return NULL;
    }

    /* Register event sources */
    struct epoll_event ev;
    ev.events = EPOLLIN | EPOLLET;

    /* Timer FDs */
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_hangtime);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_hangtime), &ev);
    ev.data.fd =dmr_phy_timer_oneshot_get_fd(&ctx->tmr_answer);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_answer), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_callalert);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_callalert), &ev);
    ev.data.fd = dmr_phy_timer_oneshot_get_fd(&ctx->tmr_grantrej);
    epoll_ctl(epfd, EPOLL_CTL_ADD, dmr_phy_timer_oneshot_get_fd(&ctx->tmr_grantrej), &ev);

    /* POSIX mqueue file descriptors */
    ev.data.fd = (int)ctx->mq_evt;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_evt, &ev);
    ev.data.fd = (int)ctx->mq_mac_conf;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_mac_conf, &ev);
    ev.data.fd = (int)ctx->mq_mac_rx;
    epoll_ctl(epfd, EPOLL_CTL_ADD, (int)ctx->mq_mac_rx, &ev);

    while (ctx->running) {
        int nev = epoll_wait(epfd, events, CCL_EPOLL_MAX_EVENTS, 200);

        if (nev < 0) {
            if (errno == EINTR) continue;
            DMR_LOGE("[CCL S%d] epoll_wait: %s", ctx->slot, strerror(errno));
            break;
        }

        for (int i = 0; i < nev; i++) {
            int fd = events[i].data.fd;

            /* ---- Timer: T_Hangtime ---- */
            if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_hangtime)) {
                ccl_voice_timer_drain(&ctx->tmr_hangtime);
                ccl_voice_event_t e = {.type=CCL_EVT_TIMER_HANGTIME,
                                       .timestamp_us=dmr_time_now_us()};
                ccl_dispatch_event(ctx, &e);
            }
            /* ---- Timer: T_AnswerResponse ---- */
            else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_answer)) {
                ccl_voice_timer_drain(&ctx->tmr_answer);
                ccl_voice_event_t e = {.type=CCL_EVT_TIMER_ANSWER,
                                       .timestamp_us=dmr_time_now_us()};
                ccl_dispatch_event(ctx, &e);
            }
            /* ---- Timer: T_CallAlert ---- */
            else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_callalert)) {
                ccl_voice_timer_drain(&ctx->tmr_callalert);
                ccl_voice_event_t e = {.type=CCL_EVT_TIMER_CALLALERT,
                                       .timestamp_us=dmr_time_now_us()};
                ccl_dispatch_event(ctx, &e);
            }
            /* ---- Timer: T_GrantRejected ---- */
            else if (fd == dmr_phy_timer_oneshot_get_fd(&ctx->tmr_grantrej)) {
                ccl_voice_timer_drain(&ctx->tmr_grantrej);
                ccl_voice_event_t e = {.type=CCL_EVT_TIMER_GRANTREJ,
                                       .timestamp_us=dmr_time_now_us()};
                ccl_dispatch_event(ctx, &e);
            }
            /* ---- CCL event queue ---- */
            else if (fd == (int)ctx->mq_evt) {
                ccl_voice_event_t evt;
                ssize_t n;
                /* Drain all pending events */
                while ((n = mq_receive(ctx->mq_evt,
                                       (char *)&evt, sizeof(evt), NULL)) > 0) {
                    ccl_dispatch_event(ctx, &evt);
                    if (!ctx->running) goto thread_exit;
                }
            }
            /* ---- MAC TX confirmation ---- */
            else if (fd == (int)ctx->mq_mac_conf) {
                dmr_mac_tx_conf_t conf;
                ssize_t n;
                while ((n = mq_receive(ctx->mq_mac_conf,
                                       (char *)&conf, sizeof(conf), NULL)) > 0) {
                    ccl_voice_event_t e;
                    memset(&e, 0, sizeof(e));
                    e.type         = (conf.result == DMR_MAC_TX_OK)
                                     ? CCL_EVT_TX_CONF : CCL_EVT_TX_ABORTED;
                    e.timestamp_us = dmr_time_now_us();
                    e.u.tx_conf    = conf;
                    ccl_dispatch_event(ctx, &e);
                }
            }
            /* ---- MAC RX burst ---- */
            else if (fd == (int)ctx->mq_mac_rx) {
                dmr_burst_t burst;
                ssize_t n;
                while ((n = mq_receive(ctx->mq_mac_rx,
                                       (char *)&burst, sizeof(burst), NULL)) > 0) {
                    ccl_voice_event_t e;
                    memset(&e, 0, sizeof(e));
                    e.type         = CCL_EVT_BURST_RECEIVED;
                    e.timestamp_us = dmr_time_now_us();
                    e.u.burst      = burst;
                    ccl_dispatch_event(ctx, &e);
                }
            }
        }
    }

thread_exit:
    close(epfd);
    DMR_LOGI("[CCL S%d] Worker thread exiting", ctx->slot);
    return NULL;
}

/* =========================================================================
 * SECTION L — Public API: event posting
 * ========================================================================= */

dmr_err_t ccl_voice_ptt_press(ccl_voice_ctx_t *ctx,
                               uint32_t         dst_id,
                               dmr_call_type_t  call_type,
                               bool             emergency)
{
    ccl_voice_event_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type               = CCL_EVT_PTT_PRESS;
    evt.timestamp_us       = dmr_time_now_us();
    evt.u.ptt.dst_id       = dst_id;
    evt.u.ptt.call_type    = call_type;
    evt.u.ptt.emergency    = emergency;

    if (mq_send(ctx->mq_evt, (const char *)&evt, sizeof(evt), 5u) < 0)
        return DMR_ERR_QUEUE_FULL;
    return DMR_OK;
}

dmr_err_t ccl_voice_ptt_release(ccl_voice_ctx_t *ctx)
{
    ccl_voice_event_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type         = CCL_EVT_PTT_RELEASE;
    evt.timestamp_us = dmr_time_now_us();

    if (mq_send(ctx->mq_evt, (const char *)&evt, sizeof(evt), 5u) < 0)
        return DMR_ERR_QUEUE_FULL;
    return DMR_OK;
}

dmr_err_t ccl_voice_rx_burst(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst)
{
    ccl_voice_event_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type         = CCL_EVT_BURST_RECEIVED;
    evt.timestamp_us = dmr_time_now_us();
    evt.u.burst      = *burst;

    if (mq_send(ctx->mq_evt, (const char *)&evt, sizeof(evt), 5u) < 0)
        return DMR_ERR_QUEUE_FULL;
    return DMR_OK;
}

dmr_err_t ccl_voice_submit_ambe_frame(ccl_voice_ctx_t       *ctx,
                                       const dmr_ambe_frame_t *frame)
{
    if (ctx->vocoder.tx_write_fd < 0) return DMR_ERR_IO;
    ssize_t n = write(ctx->vocoder.tx_write_fd,
                      frame->data, DMR_AMBE_FRAME_BYTES);
    if (n < (ssize_t)DMR_AMBE_FRAME_BYTES) return DMR_ERR_BUSY;
    return DMR_OK;
}

/* =========================================================================
 * SECTION M — Group subscription management
 * ========================================================================= */

dmr_err_t ccl_voice_subscribe_group(ccl_voice_ctx_t *ctx, uint32_t group_id)
{
    if (ctx->n_subscribed_groups >= 64u) return DMR_ERR_NO_MEM;
    for (uint8_t i = 0; i < ctx->n_subscribed_groups; i++) {
        if (ctx->subscribed_groups[i] == group_id) return DMR_OK; /* already subscribed */
    }
    ctx->subscribed_groups[ctx->n_subscribed_groups++] = group_id;
    DMR_LOGD("[CCL S%d] Subscribed group 0x%06X (%u total)",
             ctx->slot, group_id, ctx->n_subscribed_groups);
    return DMR_OK;
}

dmr_err_t ccl_voice_unsubscribe_group(ccl_voice_ctx_t *ctx, uint32_t group_id)
{
    for (uint8_t i = 0; i < ctx->n_subscribed_groups; i++) {
        if (ctx->subscribed_groups[i] == group_id) {
            ctx->subscribed_groups[i] =
                ctx->subscribed_groups[--ctx->n_subscribed_groups];
            return DMR_OK;
        }
    }
    return DMR_ERR_INVALID_PARAM;
}

bool ccl_voice_is_subscribed(const ccl_voice_ctx_t *ctx, uint32_t group_id)
{
    for (uint8_t i = 0; i < ctx->n_subscribed_groups; i++) {
        if (ctx->subscribed_groups[i] == group_id) return true;
    }
    return false;
}

/* =========================================================================
 * SECTION N — Initialisation / teardown
 * ========================================================================= */

/* For mq_evt — the one queue CCL Voice itself creates and owns. Safe to
 * use O_CREAT here because nothing else in the system ever opens this
 * name; there is no second-creator ambiguity. */
static mqd_t ccl_mq_create_own(const char *name, int max_msgs,
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
static mqd_t ccl_mq_open_mac_owned(const char *name, int oflags)
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

dmr_err_t ccl_voice_init(ccl_voice_ctx_t *ctx,
                          dmr_slot_t       slot,
                          uint32_t         my_radio_id,
                          uint8_t          colour_code)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->slot         = slot;
    ctx->my_radio_id  = my_radio_id;
    ctx->colour_code  = colour_code;
    ctx->state        = CCL_STATE_IDLE;
    ctx->running      = false;

    pthread_mutex_init(&ctx->state_mutex, NULL);

    /* Queue names depend on slot */
    const char *mq_evt_name  = (slot == DMR_SLOT_1)
                                   ? DMR_MQ_CCL_EVT_S1    : DMR_MQ_CCL_EVT_S2;
    const char *mq_tx_name   = (slot == DMR_SLOT_1)
                                   ? DMR_MQ_MAC_TX_REQ_S1 : DMR_MQ_MAC_TX_REQ_S2;
    const char *mq_conf_name = (slot == DMR_SLOT_1)
                                   ? DMR_MQ_MAC_TX_CONF_VOICE_S1: DMR_MQ_MAC_TX_CONF_VOICE_S2;
    const char *mq_rx_name   = (slot == DMR_SLOT_1)
                                   ? DMR_MQ_MAC_RX_VOICE_S1 : DMR_MQ_MAC_RX_VOICE_S2;

    /* Event queue: CCL owns/creates this one — private to this module,
     * read by CCL's own worker thread, written by the app and by CCL's
     * own timer/internal-event paths (hence O_RDWR). */
    ctx->mq_evt = ccl_mq_create_own(mq_evt_name,
                                     CCL_MQ_EVT_MAX_MSGS, CCL_MQ_EVT_MSG_SIZE,
                                     O_RDWR);

    /* The following three queues are created by MAC (mac_init()), never
     * by CCL — see the ownership contract in dmr_mac.h. CCL opens them
     * with the minimal directional flag matching how this module
     * actually uses each one, and retries briefly if MAC hasn't created
     * them yet rather than failing on an ordinary startup race. */

    /* MAC TX request queue: CCL writes (via mac_tx_enqueue), MAC reads */
    ctx->mq_mac_tx = ccl_mq_open_mac_owned(mq_tx_name, O_WRONLY);
    /* MAC TX confirmation queue: MAC writes, CCL reads */
    ctx->mq_mac_conf = ccl_mq_open_mac_owned(mq_conf_name, O_RDONLY);
    /* MAC RX burst queue: MAC writes, CCL reads */
    ctx->mq_mac_rx = ccl_mq_open_mac_owned(mq_rx_name, O_RDONLY);

    if (ctx->mq_evt     == (mqd_t)-1 || ctx->mq_mac_tx  == (mqd_t)-1 ||
        ctx->mq_mac_conf== (mqd_t)-1 || ctx->mq_mac_rx  == (mqd_t)-1) {
        DMR_LOGE("[CCL S%d] Failed to open message queues", slot);
        return DMR_ERR_IO;
    }

    /* Timer FDs */
      if (dmr_phy_timer_oneshot_init(&ctx->tmr_hangtime)   != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_answer)     != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_callalert)      != DMR_OK ||
        dmr_phy_timer_oneshot_init(&ctx->tmr_grantrej)    != DMR_OK) {
        DMR_LOGE("[CCL S%d] dmr_phy_timer_oneshot_init failed: %s",
                 slot, strerror(errno));
        return DMR_ERR_NO_MEM;
}

    /* Vocoder pipe pair */
    int tx_fds[2], rx_fds[2];
    if (pipe(tx_fds) < 0 || pipe(rx_fds) < 0) {
        DMR_LOGE("[CCL S%d] pipe() failed: %s", slot, strerror(errno));
        return DMR_ERR_IO;
    }
    /* Make pipe ends non-blocking */
    fcntl(tx_fds[0], F_SETFL, O_NONBLOCK);
    fcntl(tx_fds[1], F_SETFL, O_NONBLOCK);
    fcntl(rx_fds[0], F_SETFL, O_NONBLOCK);
    fcntl(rx_fds[1], F_SETFL, O_NONBLOCK);

    ctx->vocoder.tx_read_fd  = tx_fds[0];  /* CCL reads encoded AMBE from here */
    ctx->vocoder.tx_write_fd = tx_fds[1];  /* Vocoder writes encoded AMBE here */
    ctx->vocoder.rx_write_fd = rx_fds[1];  /* CCL writes decoded AMBE here */
    ctx->vocoder.rx_read_fd  = rx_fds[0];  /* Vocoder reads decoded AMBE from here */

    ctx->tx_req_id_next = 1u;

    DMR_LOGI("[CCL S%d] Initialised (radio_id=0x%06X cc=%u)",
             slot, my_radio_id, colour_code);
    return DMR_OK;
}

dmr_err_t ccl_voice_start(ccl_voice_ctx_t *ctx)
{
    ctx->running = true;
    if (pthread_create(&ctx->thread, NULL, ccl_voice_thread, ctx) != 0) {
        ctx->running = false;
        DMR_LOGE("[CCL S%d] pthread_create failed: %s", ctx->slot, strerror(errno));
        return DMR_ERR_IO;
    }
    return DMR_OK;
}

dmr_err_t ccl_voice_stop(ccl_voice_ctx_t *ctx)
{
    if (!ctx->running) return DMR_OK;

    ccl_voice_event_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = CCL_EVT_SHUTDOWN;
    mq_send(ctx->mq_evt, (const char *)&evt, sizeof(evt), 10u);

    pthread_join(ctx->thread, NULL);
    ctx->running = false;
    DMR_LOGI("[CCL S%d] Stopped", ctx->slot);
    return DMR_OK;
}

void ccl_voice_destroy(ccl_voice_ctx_t *ctx)
{
    /* Close timerfds */
    if (ctx->tmr_hangtime.fd  >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_hangtime);
    if (ctx->tmr_answer.fd    >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_answer);
    if (ctx->tmr_callalert.fd >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_callalert);
    if (ctx->tmr_grantrej.fd  >= 0) dmr_phy_timer_oneshot_destroy(&ctx->tmr_grantrej);

    /* Close message queues. Only mq_evt is unlinked — CCL Voice created
     * it (see ccl_voice_init / ccl_mq_create_own) and is its sole owner.
     * mq_mac_tx/conf/rx are MAC-owned (see ownership contract in
     * dmr_mac.h); CCL only ever closes its handle to them, never
     * unlinks the name — that is mac_destroy()'s responsibility. */
    if (ctx->mq_evt     != (mqd_t)-1) mq_close(ctx->mq_evt);
    if (ctx->mq_mac_tx  != (mqd_t)-1) mq_close(ctx->mq_mac_tx);
    if (ctx->mq_mac_conf!= (mqd_t)-1) mq_close(ctx->mq_mac_conf);
    if (ctx->mq_mac_rx  != (mqd_t)-1) mq_close(ctx->mq_mac_rx);
    if (ctx->mq_evt != (mqd_t)-1) {
        const char *mq_evt_name = (ctx->slot == DMR_SLOT_1)
                                       ? DMR_MQ_CCL_EVT_S1 : DMR_MQ_CCL_EVT_S2;
        mq_unlink(mq_evt_name);
    }

    /* Close vocoder pipes */
    if (ctx->vocoder.tx_read_fd  >= 0) close(ctx->vocoder.tx_read_fd);
    if (ctx->vocoder.tx_write_fd >= 0) close(ctx->vocoder.tx_write_fd);
    if (ctx->vocoder.rx_read_fd  >= 0) close(ctx->vocoder.rx_read_fd);
    if (ctx->vocoder.rx_write_fd >= 0) close(ctx->vocoder.rx_write_fd);

    pthread_mutex_destroy(&ctx->state_mutex);

    DMR_LOGI("[CCL S%d] Destroyed — calls: tx=%lu rx=%lu emerg=%lu late=%lu",
             ctx->slot,
             ctx->stats.calls_originated,
             ctx->stats.calls_received,
             ctx->stats.calls_emergency,
             ctx->stats.late_entries);
    memset(ctx, 0, sizeof(*ctx));
}

bool ccl_voice_get_call_ctx(ccl_voice_ctx_t *ctx, ccl_call_ctx_t *out)
{
    pthread_mutex_lock(&ctx->state_mutex);
    bool active = (ctx->state == CCL_STATE_TRANSMITTING   ||
                   ctx->state == CCL_STATE_RECEIVING       ||
                   ctx->state == CCL_STATE_TX_LC_HEADER    ||
                   ctx->state == CCL_STATE_TX_TERMINATOR   ||
                   ctx->state == CCL_STATE_HANGTIME);
    if (active && out)
        *out = ctx->call;
    pthread_mutex_unlock(&ctx->state_mutex);
    return active;
}

void ccl_voice_get_stats(ccl_voice_ctx_t *ctx, ccl_stats_t *out)
{
    pthread_mutex_lock(&ctx->state_mutex);
    *out = ctx->stats;
    pthread_mutex_unlock(&ctx->state_mutex);
}
