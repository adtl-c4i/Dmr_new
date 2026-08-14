#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include "dmr_mac_timer.h"
#include "dmr_mac.h"
#include "dmr_ms.h"
#include "dmr_phy.h"
#include <errno.h>

 /* Global MAC Timing State */
volatile dmr_slot_t current_slot = DMR_SLOT_1;
volatile uint32_t frame_counter = 0;
pthread_mutex_t mac_timing_lock = PTHREAD_MUTEX_INITIALIZER;

 mqd_t mq;
/* Mock Transmit Hook representing your internal DMR pipeline */
void mac_trigger_slot_processing(dmr_slot_t slot, uint32_t frame_num,dmr_ms_ctx_t *ms) {
    // This is where your state machine checks if it has data queued for this slot
    
    if(slot==DMR_SLOT_1)
    {
        
         dmr_mac_tx_req_t req; 
         dmr_phy_tx_conf_t resp;
         
         
      /*   if (mq_receive(ms->mac.mq_phy_tx,
                                       (char *)&burst, sizeof(burst), NULL) < 0) {
                                         printf("%d \n", ms->mac.mq_phy_tx);
        perror("DMR RX Queue Send Failed");
    
    // Method 2: Programmatically triage or log specific failure states
    switch (errno) {
        case EAGAIN:
            // The queue is completely full and O_NONBLOCK was set on the descriptor
            printf("Queue is full! Dropping packet or retrying later.\n");
            break;
            
        case EMSGSIZE:
            // CRITICAL: Your packing function output size is bigger than 
            // the 'mq_msgsize' specified when mq_open was called!
            printf("Coding error: msg_len exceeds the maximum message size allowed by this queue.\n");
            break;
            
        case EBADF:
            // The queue descriptor is corrupted, closed, or invalid
            printf("Invalid queue descriptor handler.\n");
            break;
            
        case EINTR:
            // The call was interrupted mid-execution by a system signal
            printf("Interrupted by signal. Safe to retry call.\n");
            break;
            
        default:
            // Fallback for any unmapped OS-specific codes
            printf("Unexpected error code: %d (%s)\n", errno, strerror(errno));
            break;
    }
        }*/
        
     /*   if (mq_receive(ms->mac.mq_phy_tx,
                                       (char *)&burst, sizeof(burst), NULL) < 0) {
                                         printf("%d \n", ms->mac.mq_phy_tx);
        perror("DMR RX Queue Send Failed");
    
    // Method 2: Programmatically triage or log specific failure states
    switch (errno) {
        case EAGAIN:
            // The queue is completely full and O_NONBLOCK was set on the descriptor
            printf("Queue is full! Dropping packet or retrying later.\n");
            break;
            
        case EMSGSIZE:
            // CRITICAL: Your packing function output size is bigger than 
            // the 'mq_msgsize' specified when mq_open was called!
            printf("Coding error: msg_len exceeds the maximum message size allowed by this queue.\n");
            break;
            
        case EBADF:
            // The queue descriptor is corrupted, closed, or invalid
            printf("Invalid queue descriptor handler.\n");
            break;
            
        case EINTR:
            // The call was interrupted mid-execution by a system signal
            printf("Interrupted by signal. Safe to retry call.\n");
            break;
            
        default:
            // Fallback for any unmapped OS-specific codes
            printf("Unexpected error code: %d (%s)\n", errno, strerror(errno));
            break;
    }
        }
        else
        {
             stub_mac_send_tx_burst(&burst);
                      printf("[MAC] --- SLOT 1 START --- (Frame: %u) \n", frame_num);
                      
                     
                       if (mq_send(ms->mac.mq_phy_tx_conf,
                (const char *)&burst,
                sizeof(burst), 0u) < 0) {
                     printf("[MAC] --- confirm not sent --- (Frame: %u) \n", frame_num);
                    return;
        }
        }*/
        
            ssize_t n = mq_receive(ms->mac.mq_phy_tx,
                                       (char *)&req, sizeof(req), NULL);
            if (n > 0) {
                    stub_mac_send_tx_burst(&req.burst);
                    resp.req_id=req.req_id;
                    resp.slot=slot;
                    resp.originated_from=req.originated_from;
                    resp.result=DMR_PHY_TX_DONE_OK;
                     
                    if (mq_send(ms->mac.mq_phy_tx_conf,
                    (const char *)&resp,
                    sizeof(resp), 0u) < 0) {
                         printf("[MAC] --- confirm not sent --- (Frame: %u) %d\n", frame_num,n);
                        return;
                    }    //transmit to socket here.
            }
            else
            {
                    if(errno!=EAGAIN){
                            resp.result=DMR_PHY_TX_DONE_FAIL;
                            printf("[MAC] --- FAILLLLLLLLLLLLLLLLLLLLLLLLLLLLL***************--- (Frame: %u) %d\n", frame_num,n);
                            if (mq_send(ms->mac.mq_phy_tx_conf,
                            (const char *)&resp,
                            sizeof(resp), 0u) < 0) {
                            printf("[MAC] --- confirm not sent --- (Frame: %u) %d\n", frame_num,n);
                            return;
                       }    //transmit to socket here.
                    }

                }
               
                
                
    }
   /* if (slot == DMR_SLOT_1) {
        printf("[MAC] --- SLOT 1 START --- (Frame: %u)\n", frame_num);
        
        
        
        // Execute Slot 1 TX or RX bursts here
    } else {
        printf("[MAC] --- SLOT 2 START --- (Frame: %u)\n", frame_num);
        // Execute Slot 2 TX or RX bursts here
    }*/
}

/* Precision Timer Interrupt Handler */
static void mac_slot_timer_handler(int sig, siginfo_t *si, void *uc) {
    pthread_mutex_lock(&mac_timing_lock);
    
    
     dmr_ms_ctx_t *ms = (dmr_ms_ctx_t *)si->si_value.sival_ptr;
    // Toggle the active logical TDMA channel
    if (current_slot == DMR_SLOT_1) {
        current_slot = DMR_SLOT_2;
    } else {
        current_slot = DMR_SLOT_1;
        frame_counter++; // A full TDMA frame consists of both Slot 1 and Slot 2 (60ms)
    }
    
    dmr_slot_t slot_to_process = current_slot;
    uint32_t active_frame = frame_counter;
    
    pthread_mutex_unlock(&mac_timing_lock);

    // Call your processing loop outside the lock to prevent deadlocks
    mac_trigger_slot_processing(slot_to_process, active_frame,ms);
}


 void *rxfromq_thread(void *arg)
 {
      dmr_ms_ctx_t *ms = (dmr_ms_ctx_t *)arg;
    // dmr_burst_t burst;
          dmr_mac_tx_req_t req; 
         dmr_phy_tx_conf_t resp;
     while(1)
     {
      ssize_t n = mq_receive(mq,
                                       (char *)&req, sizeof(req), NULL);
                if (n > 0) {
                    stub_mac_send_tx_burst(&req.burst);
                      printf("[MAC] --- SLOT 1 START --- (Frame: ) %d\n", n);
                     printf("\n%d %d{0x%x",n,sizeof(req),req.burst.raw[0]);
    for(int i=1;i<33;i++)
    printf(",0x%x",req.burst.raw[i]);
     printf("}\n");
                     
                       if (mq_send(ms->mac.mq_phy_tx_conf,
                (const char *)&resp,
                sizeof(resp), 0u) < 0) {
                  //   printf("[MAC] --- confirm not sent --- (Frame: ) %d\n", n);
                    
                      
                      
                }    //transmit to socket here.
                }
                else
                {
                   // usleep(20000);
                }
     }
     
 }
/* Initialize and Arm the 30ms Engine */

timer_t init_mac_tdma_timer(dmr_ms_ctx_t *ms) {
    timer_t timer_id;
    struct sigevent sev;
    struct itimerspec its;
    struct sigaction sa;
 pthread_t  thread;

/*
 const char *phy_tx_name   = (ms->mac.slot == DMR_SLOT_1)
                                    ? DMR_MQ_PHY_TX_S1      : DMR_MQ_PHY_TX_S2;


    int   attempts = 0;

        mq = mq_open(phy_tx_name,  O_RDWR);
        if (mq != (mqd_t)-1) {
           // return mq;
        }else
        {
             printf("[error opening queue \n" );
        }
       

   pthread_create(&thread, NULL, rxfromq_thread,ms) ;
*/



    /* 1. Register the real-time signal action */
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = mac_slot_timer_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGRTMIN, &sa, NULL) == -1) {
        perror("Fatal: Failed to catch real-time signal structure");
        exit(EXIT_FAILURE);
    }

    /* 2. Link the timer instance to our specific signal */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = ms;
    if (timer_create(CLOCK_MONOTONIC, &sev, &timer_id) == -1) {
        perror("Fatal: OS Precision Timer allocation failed");
        exit(EXIT_FAILURE);
    }

    /* 3. Configure absolute intervals (30ms periodic) */
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = SLOT_DURATION_NS; // First tick delay
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = SLOT_DURATION_NS; // Successive interval loops
    

    /* 4. Arm the system */
    if (timer_settime(timer_id, 0, &its, NULL) == -1) {
        perror("Fatal: Failed to arm TDMA intervals");
        exit(EXIT_FAILURE);
    }

    printf("[SYSTEM] Precision MAC Timer initialized. 30ms intervals operational.\n");
    return timer_id;
}


