
#ifndef DMR_MAC_TIMER_H
#define DMR_MAC_TIMER_H


/* Feature-test macro — must precede all system headers */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <time.h>
#include <signal.h>
#include "dmr_ms.h"
#ifdef __cplusplus
extern "C" {
#endif
#define SLOT_DURATION_NS 30000000L // 30 ms in nanoseconds





timer_t init_mac_tdma_timer(dmr_ms_ctx_t *ms);

#ifdef __cplusplus
}
#endif

#endif
