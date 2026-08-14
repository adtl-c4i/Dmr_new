#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <pthread.h>

#include "dmr_ccl_voice.h"
#include "dmr_pdu.h"
#include "dmr_mac.h"
#include "dmr_types.h"
#include "dmr_ccl_data.h"
#include "dmr_ms.h"
//#include "dmr_sds.h"
#include "dmr_lrrp.h"
#include "dmr_mac_timer.h"
#define Tx_DATA_TEST
//#define GROUP_CALL
// Sync patterns defined in dmr_pdu.h for confirmation
#define SYNC_MS_DATA   0xD5D7F77FD757ULL
#define SYNC_MS_VOICE  0x7F7D5DD57DFDULL
unsigned char sent =0;
static const char *const STATE_NAMES[] = {
    "IDLE", "CALL_INIT", "TX_LC_HEADER", "TRANSMITTING",
    "TX_TERMINATOR", "HANGTIME", "RECEIVING", "LATE_ENTRY",
    "UU_REQ_WAIT", "CALL_ALERT", "INTERRUPTED", "SCANNING", "ERROR","TRANSMITTING TILL SF COMPLETE","SF_FINISHED_TERMINATOR_NOW"
};

volatile bool g_mac_running = true;

void stub_mac_send_tx_burst(dmr_burst_t *out_burst) {
    sent++;
//    printf("SENT ***********%d",sent);
}

static const uint8_t SAMPLE_DATA_ACK[33] = {
0x40,0x2a,0xb0,0xfc,0x1,0xbe,0x8,0x90,0x6,0x81,0x8a,0xa2,0x5,0x8d,0x5d,0x7f,0x77,0xfd,0x75,0x72,0x38,0x84,0x61,0xa0,0x10,0xf0,0x1,0x20,0xf,0x96,0x91,0x81,0x21

};
static const uint8_t SAMPLE_DATA_ACK_for_new[2][33] = {
    
    
    {0x0,0x8a,0xb1,0x87,0x2,0x4a,0xa,0x0,0x1,0xb1,0x9d,0x42,0x5,0x8d,0x5d,0x7f,0x77,0xfd,0x75,0x73,0x3b,0x2c,0x60,0xb8,0x11,0xe4,0x4,0x64,0x4,0xc6,0x88,0xb1,0x44},
{0x7e,0x11,0x7e,0x15,0xfc,0x4f,0xf8,0xef,0xf1,0xbf,0xe1,0x1f,0xc5,0xed,0x5d,0x7f,0x77,0xfd,0x75,0x70,0x94,0x47,0xf8,0xeb,0xf0,0xa7,0xe0,0x3f,0xc2,0xf,0x82,0x1f,0x3}
    
    
    
    
    
  // 0x0,0x2f,0xb0,0x9b,0x0,0x62,0xb,0x28,0x6,0xb1,0x8b,0xe2,0x5,0x8d,0x5d,0x7f,0x77,0xfd,0x75,0x72,0x38,0x3c,0x61,0x80,0x10,0xc4,0x1,0x64,0xf,0x86,0x98,0x31,0x0

};

static const uint8_t SAMPLE_VOICE_RESP[33] = {
//0xa4,0x0,0x0,0x40,0x0,0x1f,0x48,0x0,0x0,0xc8,0x80,0xa7,0x4,0xcd,0x5d,0x7f,0x77,0xfd,0x75,0x7b,0xc8,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 //without FEC 
0x46,0x54,0x3,0xaa,0x5,0xac,0xb9,0x10,0x1b,0xe8,0x5a,0xa0,0x84,0xcd,0x5d,0x7f,0x77,0xfd,0x75,0x7b,0xc9,0x0,0x51,0x4,0x0,0x31,0x5,0x30,0x17,0xd1,0x16,0xa2,0x24 //WITH FEC

};

/**
 * @brief Helper function used by the Mock MAC layer to extract and verify
 * the 4-bit nibble-shifted sync boundary inside generated voice traffic bursts.
 */
static bool verify_tx_voice_sync(const uint8_t *raw, uint64_t expected_sync) {
    uint8_t extracted[6];
    extracted[0] = ((raw[13] & 0x0F) << 4) | ((raw[14] & 0xF0) >> 4);
    extracted[1] = ((raw[14] & 0x0F) << 4) | ((raw[15] & 0xF0) >> 4);
    extracted[2] = ((raw[15] & 0x0F) << 4) | ((raw[16] & 0xF0) >> 4);
    extracted[3] = ((raw[16] & 0x0F) << 4) | ((raw[17] & 0xF0) >> 4);
    extracted[4] = ((raw[17] & 0x0F) << 4) | ((raw[18] & 0xF0) >> 4);
    extracted[5] = ((raw[18] & 0x0F) << 4) | ((raw[19] & 0xF0) >> 4);

    uint8_t target[6];
    for (int i = 0; i < 6; i++) {
        target[i] = (uint8_t)(expected_sync >> ((5 - i) * 8));
    }
    return (memcmp(extracted, target, 6) == 0);
}

/**
 * @brief Background Mock MAC Layer Thread Loop
 * Drains bursts from the MAC TX queue, validates framing parameters, 
 * and returns feedback confirmations back to the CCL event stream.
 */
void *mock_mac_layer_loop(void *arg) {
    mqd_t mq_tx_req;
    mqd_t mq_ccl_evt;
    dmr_mac_tx_req_t tx_req;
    ccl_voice_event_t conf_evt;
    
    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(dmr_mac_tx_req_t);
    attr.mq_curmsgs = 0;

    // Unlink old queues to prevent cross-contamination from aborted runs
    mq_unlink(DMR_MQ_MAC_TX_REQ_S1);
    
    // Open standard descriptor queues shared with the CCL interface
    mq_tx_req = mq_open(DMR_MQ_MAC_TX_REQ_S1, O_CREAT | O_RDONLY, 0644, &attr);
    if (mq_tx_req == (mqd_t)-1) {
        perror("[MOCK MAC] Failed to open MAC TX Request Queue");
        return NULL;
    }

    // Open the Event queue to return confirmations to CCL
    mq_ccl_evt = mq_open(DMR_MQ_CCL_EVT_S1, O_WRONLY);

    printf("[MOCK MAC] Thread running. Listening for outbound bursts...\n");

    while (g_mac_running) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec +=1; // 1-second receive timeout loop
       // ts.tv_sec += 10 / 1000;
       // ssize_t bytes_read = mq_receive(mq_tx_req, (char *)&tx_req, sizeof(tx_req), NULL);
        ssize_t bytes_read = mq_timedreceive(mq_tx_req, (char *)&tx_req, sizeof(tx_req), NULL, &ts);
        if (bytes_read > 0) {
            printf("[MOCK MAC] ---> Intercepted Outbound Burst (Req ID: %u)\n", tx_req.req_id);
            
            // Inspect Slot Type or sync layout to display burst identification
            if (tx_req.burst.type == DMR_BURST_TYPE_DATA) {
                // Read Slot type from end of vector mapping or data sync offset
                uint8_t dtype = tx_req.burst.raw[32] & 0x0F;
                if (dtype == 0x01) {
                    printf("[MOCK MAC]      Burst Profile: VOICE LC HEADER (Data Type: 0x01)\n");
                } else if (dtype == 0x02) {
                    printf("[MOCK MAC]      Burst Profile: TERMINATOR WITH LC (Data Type: 0x02)\n");
                } else {
                    printf("[MOCK MAC]      Burst Profile: CONTROL/DATA (Data Type: 0x%02X)\n", dtype);
                }
            } 
            else if (tx_req.burst.type == DMR_BURST_TYPE_VOICE) {
                printf("[MOCK MAC]      Burst Profile: ACTIVE VOICE TRAFFIC BURST\n");
                
                // Perform Bit-Exact Validation against the Lower-13/Upper-14 Nibble Boundary rule
                if (verify_tx_voice_sync(tx_req.burst.raw, SYNC_MS_VOICE)) {
                    printf("[MOCK MAC]      Validation: PASS (Staggered Nibble Voice Sync OK)\n");
                } else {
                    printf("[MOCK MAC]      Validation: FAIL (Sync Alignment Corrupted!)\n");
                }
            }

            // Simulate hardware transmission latency before delivering confirmation (30ms per slot)
                struct timespec delay;
delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);

            // Construct and route back the transaction confirmation event
            memset(&conf_evt, 0, sizeof(ccl_voice_event_t));
            conf_evt.type = CCL_EVT_TX_CONF;
            conf_evt.u.tx_conf.req_id = tx_req.req_id;
           // conf_evt.u.tx_conf.status = DMR_OK;
            
            if (mq_ccl_evt != (mqd_t)-1) {
                mq_send(mq_ccl_evt, (const char *)&conf_evt, sizeof(conf_evt), 0);
                printf("[MOCK MAC] <--- Sent TX Confirmation for Req ID: %u\n", tx_req.req_id);
            }
        }
    }

    mq_close(mq_tx_req);
    if (mq_ccl_evt != (mqd_t)-1) mq_close(mq_ccl_evt);
    return NULL;
}

/**
 * @brief Diagnostic tracking callback hook for checking state engine compliance
 */
void on_tx_state_change(struct ccl_voice_ctx *ctx, ccl_voice_state_t old_state, ccl_voice_state_t new_state) {
    printf("[HARNESS-STATE] Transition Event: %s ===> %s\n", STATE_NAMES[old_state], STATE_NAMES[new_state]);
}

int main(void) {
    
      uint32_t target_talkgroup = 100;
  uint32_t dest=200;
  uint32_t src=5001;
  uint8_t CC=1;
   dmr_burst_t injection_burst;
  
  
    pthread_t mac_thread_id;
  //  ccl_voice_ctx_t ccl_tx_ctx;
   // memset(&ccl_tx_ctx, 0, sizeof(ccl_voice_ctx_t));

    printf("====================================================\n");
    printf("     DMR CCL VOICE TRANSMISSION SIDE TEST HARNESS    \n");
    printf("====================================================\n\n");

    // 1. Spawning background companion Mock MAC thread
  /*  if (pthread_create(&mac_thread_id, NULL, mock_mac_layer_loop, &ccl_tx_ctx) != 0) {
        fprintf(stderr, "Failed to launch Mock MAC supervisor thread\n");
        return EXIT_FAILURE;
    }*/
    
   // mac_ctx_t mac_instance;
   //   ccl_data_ctx_t ccl_d_instance;
dmr_lrrp_type_t lrrp_type=DMR_LRRP_TYPE_MOTOROLA;
   // memset(&mac_instance, 0, sizeof(mac_ctx_t));
 dmr_ms_config_t cfg = dmr_ms_config_default_tier2(src, CC,DMR_SLOT_1, 60000u,false,2,lrrp_type);    
// dmr_ms_config_t cfg = dmr_ms_config_default_tier3(src, CC,DMR_SLOT_1, 90000u);
 //   dmr_ms_config_t cfg = dmr_ms_config_default_tier1(src, CC,DMR_SLOT_2);
    dmr_ms_ctx_t ms;
    dmr_err_t err = dmr_ms_init(&ms, &cfg);
     if (err != DMR_OK) {
        fprintf(stderr, "dmr_ms_config_default_tier1 Initialization Aborted.\n");
       // g_mac_running = false;
       // pthread_join(mac_thread_id, NULL);
        return EXIT_FAILURE;
    }

      err = dmr_ms_start(&ms);
       if (err != DMR_OK) {
        fprintf(stderr, "dmr_ms_config_default_tier1 Initialization Aborted.\n");
       // g_mac_running = false;
       // pthread_join(mac_thread_id, NULL);
        return EXIT_FAILURE;
    }

    timer_t tdma_timer = init_mac_tdma_timer(&ms);
//dmr_err_t dmr_ms_init(dmr_ms_ctx_t *ctx, const dmr_ms_config_t *cfg);

     //   mac_init(&mac_instance, DMR_SLOT_1,  1,src,1); //last is tier
 //   mac_start(&mac_instance);
 //   ccl_data_init(&ccl_d_instance, DMR_SLOT_1,src,1);
 //   ccl_data_start(&ccl_d_instance);
//mqd_t slot1_tx_req_q = dmr_mac_get_queue_handle(DMR_SLOT_1, DMR_MAC_Q_TX_REQ);
 //   mqd_t slot1_tx_cnf_q = dmr_mac_get_queue_handle(DMR_SLOT_1, DMR_MAC_Q_TX_CONF);
    
    // Give queue initialization a brief head start
    struct timespec delay;
delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
    
   // ccl_voice_init(&ccl_slot1_ctx, DMR_SLOT_1, 8008, 1);//, slot1_tx_req_q, slot1_tx_cnf_q);

    // 2. Initializing standard transmitter tracking context (Radio ID: 8008, Color Code: 1)
    printf("[HARNESS] Initializing CCL Core Core Configuration Structure...\n");
  /*  dmr_err_t err = ccl_voice_init(&ccl_tx_ctx, DMR_SLOT_1, src, 1);
    if (err != DMR_OK) {
        fprintf(stderr, "CCL Layer Core Initialization Aborted.\n");
        g_mac_running = false;
        pthread_join(mac_thread_id, NULL);
        return EXIT_FAILURE;
    }*/

    // Attach reporting callback hooks
    ms.voice.on_state_change = on_tx_state_change;

    // 3. Fire up the central POSIX worker pipeline execution loop
  /*  printf("[HARNESS] Launching internal CCL Engine thread...\n");
    ccl_voice_start(&ccl_tx_ctx);
    usleep(50000);*/
    
    
    
    
     const uint8_t data12[] = "Hello DMR!";  
    printf("\n[TEST] Injecting unconfirmed data from %x ...\n",dest);
 dmr_err_t err12 = ccl_data_tx_unconfirmed(&ms.data, dest, false, 0x4,
                                              data12, sizeof(data12) - 1u);
  sent=0;
 while(sent<4);{                                             
delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
}
 #ifdef Tx_DATA_TEST   
         const uint8_t text[] = "Hello DMR!This is a multi data... more than 12 bytes";  
    printf("\n[TEST] Injecting unconfirmed data from ccl_data_send_sds %x ...\n",dest);
    
    
     if (ccl_data_tx_raw(&ms.data, dest, false, 1u, 2u, true,
                        text, sizeof(text)) != DMR_OK) {
        printf("FATAL: ccl_data_tx_raw (confirmed) submit failed\n");
        return 1;
    }

    
      sent=0;
 while(sent<8){                                             
delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
}
        // --- STEP 1: Inject reply of Voice req  ---()
    printf("\n[TEST] Injecting SAMPLE_DATA_ACK (DATA RESPONSE) from %x ...\n",dest);
  
   for(int i=0;i<2;i++)
   {
         memset(&injection_burst, 0, sizeof(dmr_burst_t));
   memcpy(injection_burst.raw, &SAMPLE_DATA_ACK_for_new[i][0], 33);
   
    err = dmr_mac_inject_rx_burst(&ms.mac, &injection_burst);
    if (err != DMR_OK) {
        fprintf(stderr, "[HARNESS-WARNING] Header injection rejected: Error %d\n", err);
    }
       delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)
}

while(1)
{
nanosleep(&delay, NULL);
nanosleep(&delay, NULL); 
}
    

    const uint8_t data[] = "Hello DMR!";  
    printf("\n[TEST] Injecting unconfirmed data from %x ...\n",dest);
 dmr_err_t err1 = ccl_data_tx_unconfirmed(&ms.data, target_talkgroup, true, 0x4,
                                              data, sizeof(data) - 1u);
  sent=0;
 while(sent<3);{                                             
delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
}
printf("\n[TEST] Injecting confirmed data from %x ...\n",dest);
  err1 = ccl_data_tx_confirmed(&ms.data, dest, false, 0x4,
                                              data, sizeof(data) - 1u);
                                              
                                              sent=0;
                                              while(sent<3)
                                              {
    delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);

}
    
nanosleep(&delay, NULL);

    
    
    
      // --- STEP 1: Inject reply of Voice req  ---()
    printf("\n[TEST] Injecting SAMPLE_DATA_ACK (DATA RESPONSE) from %x ...\n",dest);
    memset(&injection_burst, 0, sizeof(dmr_burst_t));
   memcpy(injection_burst.raw, SAMPLE_DATA_ACK, 33);
   
    err = dmr_mac_inject_rx_burst(&ms.mac, &injection_burst);
    if (err != DMR_OK) {
        fprintf(stderr, "[HARNESS-WARNING] Header injection rejected: Error %d\n", err);
    }
       delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
    
#endif
#ifdef GROUP_CALL
  
    // 4. STEP 1: Simulate user depressing the Push-To-Talk button (Targeting Talkgroup 500)
    printf("\n[APPLICATION TRIGGER] =====GROUP USER ACTION: PTT PRESSED =====\n");
    ccl_voice_ptt_press(& ms.voice, target_talkgroup, DMR_CALL_TYPE_GROUP, false);
    int timeCount=0;
while(timeCount<5)
{
timeCount++;
    // Allow time for the framework to run its initial checks, handovers, and headers
           delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
}

#endif 
#ifndef GROUP_CALL

    // 4. STEP 1: Simulate user depressing the Push-To-Talk button (Targeting Talkgroup 500)
    printf("\n[APPLICATION TRIGGER] ===== USER ACTION: PTT PRESSED =====\n");
    ccl_voice_ptt_press(& ms.voice, dest, DMR_CALL_TYPE_INDIVIDUAL, false);

    // Allow time for the framework to run its initial checks, handovers, and headers
    sent=0;
    while(sent==0)
    {
           delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);

}

 nanosleep(&delay, NULL);   
    
    
    
    
    
    

    // --- STEP 1: Inject reply of Voice req  ---()
    printf("\n[TEST] Injecting reply of voice req sent (Voice RESPONSE) from %x ...\n",dest);
    memset(&injection_burst, 0, sizeof(dmr_burst_t));
   memcpy(injection_burst.raw, SAMPLE_VOICE_RESP, 33);
   
    err = dmr_mac_inject_rx_burst(&ms.mac, &injection_burst);
    if (err != DMR_OK) {
        fprintf(stderr, "[HARNESS-WARNING] Header injection rejected: Error %d\n", err);
    }
          delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
#endif     
    
    // Now packets have to transfer to ccl from ambe.so it can make packets and send 

    // 5. STEP 2: Stream consecutive synthetic AMBE+2 vocoder audio frame matrices
 /*   printf("\n[APPLICATION TRIGGER] ===== STREAMING AUDIO VOICE FRAMES =====\n");
    dmr_ambe_frame_t audio_sample;
    
    // Fill sample structure with mock vocoder tone data bytes
    memset(audio_sample.data, 0xA5, DMR_AMBE_FRAME_BYTES);

    for (int frame_seq = 0; frame_seq < 2; frame_seq++) {
      //  printf("[HARNESS] Submitting AMBE+2 Frame Segment %d...\n", frame_seq + 1);
        
        // Pass payload bits into the active transmission pipeline buffer
        dmr_err_t tx_status = ccl_voice_submit_ambe_frame(&ccl_tx_ctx, &audio_sample);
        if (tx_status == DMR_ERR_BUSY) {
            printf("[HARNESS-WARNING] Vocoder buffer choke. Retrying insertion...\n");
            usleep(10000);
            frame_seq--; // Remap retry step
            continue;
        }
        
        // Intermittently pace frame delivery to mimic real-time hardware vocoder generation boundaries
        usleep(30000); 
    }*/

       delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
       delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)
nanosleep(&delay, NULL);
       delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)
nanosleep(&delay, NULL);
       delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)
nanosleep(&delay, NULL);
    // 6. STEP 3: Simulate user releasing the Push-To-Talk button
    printf("\n[APPLICATION TRIGGER] ===== USER ACTION: PTT RELEASED =====\n");
    ccl_voice_ptt_release(& ms.voice);

    // Hold monitoring open to observe the transmission of the Terminator packet and Hangtime cooldown states
    printf("[HARNESS] Waiting for Hangtime countdown and session cooldown to settle...\n");
           delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
       delay.tv_sec = 0;             // Seconds
delay.tv_nsec = 500000000;    // Nanoseconds (500ms)

nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);
nanosleep(&delay, NULL);

    // 7. Graceful teardown of both execution environments
    printf("\n[HARNESS] Shutting down simulation architecture safely...\n");
    dmr_ms_stop(&ms);
    dmr_ms_destroy(&ms);

   // g_mac_running = false;
  //  pthread_join(mac_thread_id, NULL);

    printf("\n====================================================\n");
    printf("         TX HARNESS SIMULATION COMPLETED CLEANLY      \n");
    printf("====================================================\n");
    return 0;
}