/**
 * @file dmr_lrrp.h
 * @brief MOD-10 — Location Request and Response Protocol (LRRP) Facade
 *
 * Provides a vendor-agnostic interface for parsing location requests and
 * building location responses across Motorola, ETSI LIP, and NMEA formats.
 */

#ifndef DMR_LRRP_H
#define DMR_LRRP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "dmr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Enumerations & Types
 * ========================================================================= */

/**
 * @brief Supported Location Protocol Flavors
 */
typedef enum {
    DMR_LRRP_TYPE_NONE = 0,    /**< Location services disabled */
    DMR_LRRP_TYPE_MOTOROLA,    /**< Motorola LRRP (Binary/XML over UDP 4001) */
    DMR_LRRP_TYPE_ETSI_LIP,    /**< ETSI TS 100 392-18 Location Information Protocol */
    DMR_LRRP_TYPE_NMEA         /**< Raw ASCII NMEA 0183 Strings */
} dmr_lrrp_type_t;

/**
 * @brief Request types from the base station/polling radio
 */
typedef enum {
    DMR_LRRP_REQ_UNKNOWN = 0,
    DMR_LRRP_REQ_IMMEDIATE,    /**< Send single location update right now */
    DMR_LRRP_REQ_START_TRIG,   /**< Start sending periodic updates */
    DMR_LRRP_REQ_STOP_TRIG     /**< Stop sending periodic updates */
} dmr_lrrp_req_type_t;

/**
 * @brief Unified Location Request Structure
 */
typedef struct {
    uint32_t            request_id;   /**< Token/ID to match response to request */
    dmr_lrrp_req_type_t type;         /**< The action required */
    uint16_t            interval_s;   /**< Trigger interval in seconds (if applicable) */
} dmr_lrrp_req_t;

/**
 * @brief Unified Location Coordinate Structure (WGS84)
 */
typedef struct {
    float               latitude;     /**< Decimal degrees, North positive */
    float               longitude;    /**< Decimal degrees, East positive */
    uint16_t            speed;        /**< Speed in km/h */
    uint16_t            heading;      /**< 0-359 degrees */
    bool                fix_valid;    /**< True if GPS has a valid 2D/3D fix */
} dmr_lrrp_coord_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Parses an incoming data payload to determine if it is a valid location request.
 *
 * @param type       The expected LRRP protocol format.
 * @param payload    Pointer to the reassembled data payload from LLC.
 * @param length     Length of the payload in bytes.
 * @param[out] req   Populated with the request details if parsing is successful.
 * @return           DMR_OK on success, DMR_ERR_INVALID_FORMAT on parse failure.
 */
dmr_err_t dmr_lrrp_parse_request(dmr_lrrp_type_t type, 
                                 const uint8_t *payload, 
                                 size_t length,
                                 dmr_lrrp_req_t *req);

/**
 * @brief Builds an outgoing data payload containing the radio's current location.
 *
 * @param type       The expected LRRP protocol format.
 * @param request_id The token/ID from the parsed request (0 if unprompted beacon).
 * @param coord      The current GPS coordinates to pack.
 * @param[out] buf   Buffer to write the formatted payload into.
 * @param max_len    Maximum size of the output buffer.
 * @param[out] w_len Number of bytes actually written to the buffer.
 * @return           DMR_OK on success, DMR_ERR_NO_MEM if max_len is too small.
 */
dmr_err_t dmr_lrrp_build_response(dmr_lrrp_type_t type,
                                  uint32_t request_id,
                                  const dmr_lrrp_coord_t *coord,
                                  uint8_t *buf,
                                  size_t max_len,
                                  size_t *w_len);

#ifdef __cplusplus
}
#endif

#endif /* DMR_LRRP_H */