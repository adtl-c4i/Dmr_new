/**

/**
 * @file dmr_ccl_voice.h
 * @brief MOD-05 — Call Control Layer (CCL) Voice Services — Public Interface
 *
 * Implements the complete MS-side CCL state machine for voice calls as defined
 * in ETSI TS 102 361-2, Clauses 5 and 6.
 *
 * Supports:
 *   - Group voice call TX and RX (Tier II conventional & DMO)
 *   - Individual voice call TX and RX (OACSU procedure)
 *   - Late entry via Embedded LC fragment reassembly (4 fragments → Full LC)
 *   - Voice superframe assembly (bursts A..F, 360 ms)
 *   - Call priority and preemption
 *   - Emergency voice call
 *   - Hangtime management
 *   - AMBE+2 vocoder frame pipeline interface
 *   - Dual-slot support (slot 1 and slot 2 run independent CCL instances)
 *
 * Linux POSIX:
 *   - Each CCL instance runs as a pthread
 *   - MAC interaction via POSIX message queues (mqueue.h)
 *   - Vocoder pipeline via pipe(2) or shared ring buffer
 *   - Timers via timer_create / POSIX real-time signals or timerfd
 *
 * ETSI References:
 *   TS 102 361-2 Cl. 5   — Voice Call Control procedures (MS side)
 *   TS 102 361-2 Cl. 6   — Voice Call Facilities
 *   TS 102 361-2 Cl. 7   — Generic services (emergency, call alert)
 *   TS 102 361-1 Cl. 9   — PDU structures (dmr_pdu.h)
 */

#ifndef DMR_CCL_VOICE_H
#define DMR_CCL_VOICE_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <mqueue.h>
#include <signal.h>
#include <time.h>

#include "dmr_pdu.h"
#include "dmr_types.h"
#include "dmr_mac.h"
#include "dmr_phy.h"
#include "dmr_dmo.h"


#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * CCL Voice State Machine States
 * ETSI TS 102 361-2, Clause 5 SDL — MS CCL states
 * ========================================================================= */
typedef enum {
    CCL_STATE_IDLE           = 0,  /**< No call — monitoring for Voice LC Header      */
    CCL_STATE_CALL_INIT      = 1,  /**< PTT pressed — building Voice LC Header TX      */
    CCL_STATE_TX_LC_HEADER   = 2,  /**< Transmitting Voice LC Header burst             */
    CCL_STATE_TRANSMITTING   = 3,  /**< Sending voice superframe bursts A..F           */
    CCL_STATE_TX_TERMINATOR  = 4,  /**< Sending Terminator with LC                     */
    CCL_STATE_HANGTIME       = 5,  /**< Call ended — channel warm, idle PDUs           */
    CCL_STATE_RECEIVING      = 6,  /**< Incoming voice call — decoding superframe      */
    CCL_STATE_LATE_ENTRY     = 7,  /**< Late entry — collecting EMB LC fragments       */
    CCL_STATE_UU_REQ_WAIT    = 8,  /**< Individual call — waiting for UU_ANS_RSP       */
    CCL_STATE_CALL_ALERT     = 9,  /**< Waiting for remote to answer call alert        */
    CCL_STATE_INTERRUPTED    = 10, /**< Higher priority call preempting current call   */
    CCL_STATE_SCANNING       = 11, /**< Scanning — no slot activity                   */
    CCL_STATE_ERROR          = 12,
    CCL_STATE_TX_TERMINATOR_ONCE_SF_FINISHED  = 13,  /**< ptt released mid sf                    */
    CCL_STATE_TX_SF_FINISHED_TERMINATOR=14,/**< sf over terminator stage now*/
} ccl_voice_state_t;

/* Human-readable state names for logging */
static const char * const CCL_STATE_NAMES[] = {
    "IDLE", "CALL_INIT", "TX_LC_HEADER", "TRANSMITTING",
    "TX_TERMINATOR", "HANGTIME", "RECEIVING", "LATE_ENTRY",
    "UU_REQ_WAIT", "CALL_ALERT", "INTERRUPTED", "SCANNING", "ERROR","TRANSMITTING TILL SF COMPLETE","SF_FINISHED_TERMINATOR_NOW"
};

/* =========================================================================
 * CCL Events — posted to the CCL event queue from other threads/modules
 * ========================================================================= */
typedef enum {
    CCL_EVT_PTT_PRESS        = 0,  /**< User pressed PTT (from app layer)             */
    CCL_EVT_PTT_RELEASE      = 1,  /**< User released PTT                             */
    CCL_EVT_BURST_RECEIVED   = 2,  /**< MAC delivered a decoded burst                 */
    CCL_EVT_TX_CONF          = 3,  /**< MAC confirms burst was transmitted             */
    CCL_EVT_TX_ABORTED       = 4,  /**< MAC could not transmit (channel busy)         */
    CCL_EVT_TIMER_HANGTIME   = 5,  /**< T_Hangtime expired                            */
    CCL_EVT_TIMER_ANSWER     = 6,  /**< T_AnswerResponse expired (individual call)    */
    CCL_EVT_TIMER_CALLALERT  = 7,  /**< T_CallAlert expired                           */
    CCL_EVT_TIMER_GRANTREJ   = 8,  /**< T_GrantRejected backoff expired               */
    CCL_EVT_EMERGENCY        = 9,  /**< Emergency button pressed                      */
    CCL_EVT_SHUTDOWN         = 10, /**< Graceful shutdown requested                   */
} ccl_voice_event_type_t;

typedef struct {
    ccl_voice_event_type_t  type;
    uint64_t                timestamp_us;
    union {
        dmr_burst_t         burst;       /**< For CCL_EVT_BURST_RECEIVED              */
        dmr_mac_tx_conf_t   tx_conf;     /**< For CCL_EVT_TX_CONF / TX_ABORTED        */
        struct {
            uint32_t        dst_id;
            dmr_call_type_t call_type;
            bool            emergency;
        } ptt;                           /**< For CCL_EVT_PTT_PRESS                   */
    } u;
} ccl_voice_event_t;

/* POSIX mqueue name for CCL event posting */
#define DMR_MQ_CCL_EVT_S1   "/dmr_ccl_evt_s1"
#define DMR_MQ_CCL_EVT_S2   "/dmr_ccl_evt_s2"

 
/* =========================================================================
 * CCL Timers — ETSI TS 102 361-2, Clause 5 timer definitions
 * ========================================================================= */
#define CCL_T_HANGTIME_MS        3000    /**< Channel hangtime after call end (ms)      */
#define CCL_T_ANSWER_RESPONSE_MS 4000   /**< Wait for UU_ANS_RSP after UU_V_REQ (ms)  */
#define CCL_T_CALL_ALERT_MS      20000  /**< Wait for call alert response (ms)         */
#define CCL_T_GRANT_REJECTED_MS  500    /**< Backoff after call grant rejected (ms)    */
#define CCL_T_TX_GUARD_MS        5      /**< Guard between consecutive TX bursts (ms)  */
#define CCL_T_SUPERFRAME_MS      360    /**< Full superframe duration (ms)             */
#define CCL_T_BURST_MS           30     /**< Single burst / timeslot duration (ms)     */

/* =========================================================================
 * Active call context — describes a call in progress
 * ========================================================================= */
typedef struct {
    dmr_call_type_t   call_type;       /**< Group, individual, broadcast, emergency   */
    uint32_t          src_id;          /**< Source radio ID                           */
    uint32_t          dst_id;          /**< Destination group or radio ID             */
    uint8_t           colour_code;     /**< Active colour code                        */
    bool              encrypted;       /**< Privacy indicator from LC                 */
    bool              emergency;       /**< Emergency flag from service options        */
    uint8_t           priority;        /**< Call priority 0-3                         */
    uint64_t          call_start_us;   /**< When call was established                 */
    uint64_t          last_burst_us;   /**< Timestamp of last received burst          */
    uint32_t          burst_count;     /**< Bursts received in this call              */
    uint32_t          superframe_seq;  /**< Current superframe sequence number        */

    /* Embedded LC reassembly for late entry
     * 4 fragments of 18 bits each arrive in EMB fields of superframe bursts B,C,D,E
     * ETSI TS 102 361-1, Clause 9.1.9
     */
    uint8_t           emblc_frags[4][4];  /**< EMB LC fragments [fragment_index][bytes] */
    uint8_t           emblc_frag_mask;    /**< Bitmask: bit N set = fragment N received  */
    bool              lc_valid;           /**< Full LC has been decoded and verified      */
    dmr_full_lc_t     full_lc;            /**< Decoded Full LC for this call             */
} ccl_call_ctx_t;

/* =========================================================================
 * Vocoder pipeline interface
 * CCL owns two FDs: one for AMBE+2 encoded frames going to speaker (rx_pipe),
 * one for PCM/AMBE frames coming from mic (tx_pipe).
 * ========================================================================= */
typedef struct {
    int  rx_write_fd;  /**< CCL writes decoded AMBE frames here → vocoder reads     */
    int  rx_read_fd;   /**< Vocoder reads from this end                             */
    int  tx_write_fd;  /**< Vocoder writes encoded AMBE frames here → CCL reads     */
    int  tx_read_fd;   /**< CCL reads encoded AMBE frames from vocoder              */
} ccl_vocoder_pipe_t;

/* =========================================================================
 * CCL Statistics
 * ========================================================================= */
typedef struct {
    uint64_t calls_originated;
    uint64_t calls_received;
    uint64_t calls_denied;
    uint64_t calls_emergency;
    uint64_t bursts_tx;
    uint64_t bursts_rx;
    uint64_t late_entries;
    uint64_t tx_aborts;
    uint64_t lc_errors;
    uint64_t fec_errors;
} ccl_stats_t;

/* =========================================================================
 * CCL Voice Instance — one per TDMA timeslot
 * ========================================================================= */
typedef struct ccl_voice_ctx {
    /* Identity */
    dmr_slot_t          slot;
    uint32_t            my_radio_id;
    uint8_t             colour_code;
    uint32_t            subscribed_groups[64];
    uint8_t             n_subscribed_groups;
    uint8_t             fid;
    /* State machine */
    ccl_voice_state_t   state;
    pthread_mutex_t     state_mutex;

    /* Active call */
    ccl_call_ctx_t      call;

    /* TX superframe tracking */
    uint8_t             sf_burst_idx;      /**< 0..5 (A..F) current superframe position */
    uint32_t            tx_req_id_next;    /**< Rolling TX request ID counter           */
    uint32_t            tx_req_id_pending; /**< Last submitted TX req ID                */

    /* POSIX message queues */
    mqd_t               mq_evt;        /**< Inbound events to this CCL instance         */
    mqd_t               mq_mac_tx;     /**< TX burst requests → MAC                    */
    mqd_t               mq_mac_conf;   /**< TX confirmations ← MAC                     */
    mqd_t               mq_mac_rx;     /**< RX bursts ← MAC (decoded)                  */

    /* Vocoder pipe pair */
    ccl_vocoder_pipe_t  vocoder;

    dmr_phy_timer_oneshot_t                 tmr_hangtime;       /**< T_Hangtime timerfd                     */
    dmr_phy_timer_oneshot_t                 tmr_answer;         /**< T_AnswerResponse timerfd               */
    dmr_phy_timer_oneshot_t                 tmr_callalert;      /**< T_CallAlert timerfd                    */
    dmr_phy_timer_oneshot_t                 tmr_grantrej;       /**< T_GrantRejected timerfd               */


    /* Worker thread */
    pthread_t           thread;
    volatile bool       running;

    /* Callbacks (optional — application integration hooks) */
    void (*on_call_start)(struct ccl_voice_ctx *ctx, const ccl_call_ctx_t *call);
    void (*on_call_end)  (struct ccl_voice_ctx *ctx, const ccl_call_ctx_t *call);
    void (*on_ambe_rx)   (struct ccl_voice_ctx *ctx, const dmr_voice_superframe_t *sframe,
                          uint32_t src_id);
    void (*on_state_change)(struct ccl_voice_ctx *ctx,
                            ccl_voice_state_t old_state,
                            ccl_voice_state_t new_state);
                            
    dmr_voice_superframe_t sfVoice;

    /* Statistics */
    ccl_stats_t         stats;

    /* Optional MOD-15 (DCDM) back-reference — non-NULL only when this
     * MS is DMR_TIER_2_CONVENTIONAL with dcdm_enabled=true (wired by
     * dmr_ms.c after dmr_dmo_init() succeeds). Used to send the
     * appropriate CT_CSBK_Term after a voice TX completes, per
     * Cl.6.2.2.3.3 — except after UU_ANS_RSP (Ack to a CSBK), which is
     * explicitly excluded; see ccl_voice_tx_uu_ans_rsp(). */
    dmr_dmo_ctx_t      *dcdm;

    /* MAC back-reference, wired by dmr_ms.c — used alongside dcdm to
     * query mac_slot_is_busy() on the *other* slot for the DCDM
     * Transmit procedure's channel_activity argument (Cl.6.2.3.12). */
    mac_ctx_t          *mac;

} ccl_voice_ctx_t;
 dmr_err_t mapMACQueues(ccl_voice_ctx_t *);
/* =========================================================================
 * CCL Voice API — public functions
 * ========================================================================= */

/**
 * @brief Initialise a CCL voice instance for the given TDMA slot.
 *
 * Opens POSIX message queues, creates timerfd descriptors, initialises
 * mutexes. Does NOT start the worker thread — call ccl_voice_start() for that.
 *
 * @param ctx           Caller-allocated context structure
 * @param slot          DMR_SLOT_1 or DMR_SLOT_2
 * @param my_radio_id   24-bit radio ID (from codeplug)
 * @param colour_code   Active colour code 0-15
 * @return DMR_OK on success
 */
dmr_err_t ccl_voice_init(ccl_voice_ctx_t *ctx,
                          dmr_slot_t       slot,
                          uint32_t         my_radio_id,
                          uint8_t          colour_code);

/**
 * @brief Subscribe to a talkgroup ID.
 * Incoming group calls with this dst_id will be accepted.
 */
dmr_err_t ccl_voice_subscribe_group(ccl_voice_ctx_t *ctx, uint32_t group_id);

/**
 * @brief Unsubscribe from a talkgroup ID.
 */
dmr_err_t ccl_voice_unsubscribe_group(ccl_voice_ctx_t *ctx, uint32_t group_id);

/**
 * @brief Check if we are subscribed to a given group.
 */
bool ccl_voice_is_subscribed(const ccl_voice_ctx_t *ctx, uint32_t group_id);

/**
 * @brief Start the CCL worker thread.
 * Creates a POSIX thread that runs the event loop.
 */
dmr_err_t ccl_voice_start(ccl_voice_ctx_t *ctx);

/**
 * @brief Stop the CCL worker thread gracefully.
 * Posts CCL_EVT_SHUTDOWN and joins the thread.
 */
dmr_err_t ccl_voice_stop(ccl_voice_ctx_t *ctx);

/**
 * @brief Destroy all resources held by the CCL context.
 * Must be called after ccl_voice_stop().
 */
void ccl_voice_destroy(ccl_voice_ctx_t *ctx);

/**
 * @brief Application requests a PTT (Push-To-Talk) press.
 *
 * Triggers group or individual voice call initiation.
 *
 * @param ctx          CCL context
 * @param dst_id       Destination group or radio ID
 * @param call_type    DMR_CALL_TYPE_GROUP or DMR_CALL_TYPE_INDIVIDUAL
 * @param emergency    true to set the emergency service option
 */
dmr_err_t ccl_voice_ptt_press(ccl_voice_ctx_t *ctx,
                               uint32_t         dst_id,
                               dmr_call_type_t  call_type,
                               bool             emergency);

/**
 * @brief Application signals PTT release.
 */
dmr_err_t ccl_voice_ptt_release(ccl_voice_ctx_t *ctx);

/**
 * @brief Inject a received burst from the burst processor.
 * Called by the burst processor / MAC layer (from their thread) to
 * deliver an incoming burst to the CCL for processing.
 */
dmr_err_t ccl_voice_rx_burst(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst);

/**
 * @brief Post an AMBE+2 encoded frame from the vocoder for TX.
 * Called by the vocoder thread to deliver the next AMBE+2 frame
 * to be inserted into the next voice burst.
 *
 * @param ctx    CCL context
 * @param frame  AMBE+2 encoded frame (9 bytes)
 * @return DMR_OK, DMR_ERR_BUSY if vocoder pipe full
 */
dmr_err_t ccl_voice_submit_ambe_frame(ccl_voice_ctx_t    *ctx,
                                       const dmr_ambe_frame_t *frame);

/**
 * @brief Get the current CCL state (thread-safe).
 */
ccl_voice_state_t ccl_voice_get_state(ccl_voice_ctx_t *ctx);

/**
 * @brief Get a copy of the active call context (thread-safe).
 * @return true if a call is active, false if idle
 */
bool ccl_voice_get_call_ctx(ccl_voice_ctx_t *ctx, ccl_call_ctx_t *out);

/**
 * @brief Get statistics snapshot (thread-safe copy).
 */
void ccl_voice_get_stats(ccl_voice_ctx_t *ctx, ccl_stats_t *out);

/* =========================================================================
 * Internal (used by ccl_voice.c sub-modules — not part of public API)
 * ========================================================================= */

/** Transition state machine to new state (logs transition) */
void ccl_voice_set_state(ccl_voice_ctx_t *ctx, ccl_voice_state_t new_state);

/** Build and submit a Voice LC Header burst to MAC */
dmr_err_t ccl_voice_tx_lc_header(ccl_voice_ctx_t *ctx);

/** Build and submit a Terminator with LC burst to MAC */
dmr_err_t ccl_voice_tx_terminator(ccl_voice_ctx_t *ctx);

/** Build and submit the next voice burst (A..F) in the superframe */
dmr_err_t ccl_voice_tx_next_burst(ccl_voice_ctx_t *ctx);

/** Build and submit idle PDU burst */
dmr_err_t ccl_voice_tx_idle(ccl_voice_ctx_t *ctx);

/** Build and submit UU_V_REQ CSBK (individual call) */
dmr_err_t ccl_voice_tx_uu_v_req(ccl_voice_ctx_t *ctx, uint32_t dst_id);

/** Build and submit UU_ANS_RSP CSBK (answer individual call) */
dmr_err_t ccl_voice_tx_uu_ans_rsp(ccl_voice_ctx_t *ctx, uint32_t dst_id, uint8_t response);

/** Process an incoming Voice LC Header burst */
dmr_err_t ccl_voice_rx_lc_header(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst);

/** Process an incoming voice burst (A..F) */
dmr_err_t ccl_voice_rx_voice_burst(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst);

/** Process an incoming Terminator with LC burst */
dmr_err_t ccl_voice_rx_terminator(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst);

/** Process an incoming CSBK burst (UU_V_REQ, UU_ANS_RSP, etc.) */
dmr_err_t ccl_voice_rx_csbk(ccl_voice_ctx_t *ctx, const dmr_burst_t *burst);

/** Accumulate EMB fragment; reassemble Full LC when all 4 fragments present */
dmr_err_t ccl_voice_process_emblc(ccl_voice_ctx_t *ctx,
                                   uint8_t lcss,
                                   const uint8_t *emb_payload_18bits);

/** The CCL worker thread entry point */
void *ccl_voice_thread(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* DMR_CCL_VOICE_H */
