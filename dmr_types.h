#define PRINTFDATA //used for debug.

//#define PRINTFDATA //used for debug.

/**
 * @file dmr_types.h
 * @brief DMR Mobile Station — Common Types, Error Codes, Logging, Timing
 *
 * Shared across all modules. No DMR-protocol-specific content here —
 * this is pure infrastructure glue.
 */

#ifndef DMR_TYPES_H
#define DMR_TYPES_H

/* Feature-test macro — must precede all system headers */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <time.h>




#ifdef __cplusplus
extern "C" {
#endif
/* =========================================================================
 * Call type — shared by CCL and application layer
 * ========================================================================= */
typedef enum {
    DMR_CALL_TYPE_GROUP       = 0,
    DMR_CALL_TYPE_INDIVIDUAL  = 1,
    DMR_CALL_TYPE_BROADCAST   = 2,
    DMR_CALL_TYPE_EMERGENCY   = 3,
} dmr_call_type_t;
/* =========================================================================
 * Error codes
 * ========================================================================= */
typedef enum {
    DMR_OK                 =  0,
    DMR_ERR_INVALID_PARAM  = -1,
    DMR_ERR_NO_MEM         = -2,
    DMR_ERR_TIMEOUT        = -3,
    DMR_ERR_QUEUE_FULL     = -4,
    DMR_ERR_QUEUE_EMPTY    = -5,
    DMR_ERR_BUSY           = -6,
    DMR_ERR_FEC            = -7,
    DMR_ERR_CRC            = -8,
    DMR_ERR_NOT_SUPPORTED  = -9,
    DMR_ERR_IO             = -10,
} dmr_err_t;

/* =========================================================================
 * TDMA slot identifier
 * ========================================================================= */
typedef enum {
    DMR_SLOT_1 = 1,
    DMR_SLOT_2 = 2,
} dmr_slot_t;

/* =========================================================================
 * Log levels
 * ========================================================================= */
typedef enum {
    DMR_LOG_TRACE = 0,
    DMR_LOG_DEBUG = 1,
    DMR_LOG_INFO  = 2,
    DMR_LOG_WARN  = 3,
    DMR_LOG_ERROR = 4,
} dmr_log_level_t;

/* Global log level (default INFO) — set at startup */
extern volatile dmr_log_level_t g_dmr_log_level;

#define DMR_LOG(level, lvlstr, fmt, ...) do { \
    if ((level) >= g_dmr_log_level) { \
        fprintf(stderr, "[" lvlstr "] " fmt "\n", ##__VA_ARGS__); \
    } \
} while (0)

#define DMR_LOGT(fmt, ...) DMR_LOG(DMR_LOG_TRACE, "TRC", fmt, ##__VA_ARGS__)
#define DMR_LOGD(fmt, ...) DMR_LOG(DMR_LOG_DEBUG, "DBG", fmt, ##__VA_ARGS__)
#define DMR_LOGI(fmt, ...) DMR_LOG(DMR_LOG_INFO,  "INF", fmt, ##__VA_ARGS__)
#define DMR_LOGW(fmt, ...) DMR_LOG(DMR_LOG_WARN,  "WRN", fmt, ##__VA_ARGS__)
#define DMR_LOGE(fmt, ...) DMR_LOG(DMR_LOG_ERROR, "ERR", fmt, ##__VA_ARGS__)
#define PRINTFDATA //used for debug.
/* =========================================================================
 * Timing helper — monotonic microsecond clock
 * ========================================================================= */
static inline uint64_t dmr_time_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000L);
}

/* Global log level definition (define once in one .c file) */
#ifdef DMR_TYPES_DEFINE_GLOBALS
volatile dmr_log_level_t g_dmr_log_level =5;// DMR_LOG_TRACE;
#endif

#ifdef __cplusplus
}
#endif

#endif /* DMR_TYPES_H */
