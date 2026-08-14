
/**
 * @file dmr_lrrp.c
 * @brief MOD-10 — LRRP Facade Implementation
 */

#include "dmr_lrrp.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* =========================================================================
 * Private Serializers: Motorola LRRP (Binary over UDP)
 * ========================================================================= */

static dmr_err_t parse_motorola_lrrp(const uint8_t *payload, size_t length, dmr_lrrp_req_t *req)
{
    /* Motorola LRRP typically uses a tokenized binary structure.
     * Example Token: 0x11 = Immediate Request, 0x12 = Triggered Request
     * This is a structural proxy; adjust exact token values to match your network config.
     */
    if (length < 4) return DMR_ERR_INVALID_PARAM;

    req->request_id = ((uint32_t)payload[0] << 24) | ((uint32_t)payload[1] << 16) |
                      ((uint32_t)payload[2] << 8)  | payload[3];


    uint8_t opcode = payload[4];
   
    if (opcode == 0x11) {
        req->type = DMR_LRRP_REQ_IMMEDIATE;
    } else if (opcode == 0x12) {
        req->type = DMR_LRRP_REQ_START_TRIG;
        req->interval_s = (length >= 7) ? ((uint16_t)payload[5] << 8) | payload[6] : 60;
    } else if (opcode == 0x13) {
        req->type = DMR_LRRP_REQ_STOP_TRIG;
    } else {
        return DMR_ERR_INVALID_PARAM;
    }

    return DMR_OK;
}

static dmr_err_t build_motorola_lrrp(uint32_t request_id, const dmr_lrrp_coord_t *coord,
                                     uint8_t *buf, size_t max_len, size_t *w_len)
{
    /* Response structure proxy. 
     * Packs standard 32-bit floats (IEEE 754) for Lat/Lon.
     */
    if (max_len < 17) return DMR_ERR_NO_MEM;

    buf[0] = (uint8_t)(request_id >> 24);
    buf[1] = (uint8_t)(request_id >> 16);
    buf[2] = (uint8_t)(request_id >> 8);
    buf[3] = (uint8_t)(request_id & 0xFF);

    buf[4] = 0x22; /* Arbitrary Response Opcode */

    /* Copy floats byte-by-byte to avoid strict aliasing rule violations */
    memcpy(&buf[5], &coord->latitude, sizeof(float));
    memcpy(&buf[9], &coord->longitude, sizeof(float));

    buf[13] = (uint8_t)(coord->speed >> 8);
    buf[14] = (uint8_t)(coord->speed & 0xFF);
    buf[15] = (uint8_t)(coord->heading >> 8);
    buf[16] = (uint8_t)(coord->heading & 0xFF);

    *w_len = 17;
    return DMR_OK;
}

/* =========================================================================
 * Private Serializers: ETSI TS 100 392-18 Location Information Protocol
 * ========================================================================= */

static dmr_err_t parse_etsi_lip(const uint8_t *payload, size_t length, dmr_lrrp_req_t *req)
{
    /* ETSI LIP Short Location Request (PDU Type 0) */
    if (length < 2) return DMR_ERR_INVALID_PARAM;

    uint8_t pdu_type = (payload[0] >> 6) & 0x03;
    if (pdu_type != 0x00) return DMR_ERR_INVALID_PARAM; 
printf("##############%x\n",pdu_type);
    /* Simplified bit parsing for reference */
    req->request_id = payload[1]; /* ETSI uses 8-bit reference IDs */
    req->type = DMR_LRRP_REQ_IMMEDIATE;
    
    return DMR_OK;
}

static dmr_err_t build_etsi_lip(uint32_t request_id, const dmr_lrrp_coord_t *coord,
                                uint8_t *buf, size_t max_len, size_t *w_len)
{
    /* ETSI LIP Short Location Report (PDU Type 1) 
     * Uses 24-bit/32-bit mapped representations of degrees. 
     */
    if (max_len < 10) return DMR_ERR_NO_MEM;

    /* Base PDU Type 1 */
    buf[0] = 0x40; /* 01xxxxxx */
    buf[1] = (uint8_t)(request_id & 0xFF);

    /* Simplified mapping: In production, ETSI maps coordinates to a 32-bit integer grid */
    int32_t lat_map = (int32_t)(coord->latitude * 1000000.0f);
    int32_t lon_map = (int32_t)(coord->longitude * 1000000.0f);

    buf[2] = (uint8_t)(lat_map >> 24);
    buf[3] = (uint8_t)(lat_map >> 16);
    buf[4] = (uint8_t)(lat_map >> 8);
    buf[5] = (uint8_t)(lat_map & 0xFF);

    buf[6] = (uint8_t)(lon_map >> 24);
    buf[7] = (uint8_t)(lon_map >> 16);
    buf[8] = (uint8_t)(lon_map >> 8);
    buf[9] = (uint8_t)(lon_map & 0xFF);

    *w_len = 10;
    return DMR_OK;
}

/* =========================================================================
 * Private Serializers: NMEA 0183 ($GPRMC)
 * ========================================================================= */

static dmr_err_t parse_nmea(const uint8_t *payload, size_t length, dmr_lrrp_req_t *req)
{
    /* NMEA doesn't have an inherent "request" protocol in the standard string format,
     * but radios utilizing NMEA often send a specific trigger string to request a read. */
    if (length < 6) return DMR_ERR_INVALID_PARAM;

    if (strncmp((const char *)payload, "$REQLOC", 7) == 0) {
        req->type = DMR_LRRP_REQ_IMMEDIATE;
        req->request_id = 0;
        return DMR_OK;
    }
    
    return DMR_ERR_INVALID_PARAM;
}

static dmr_err_t build_nmea(uint32_t request_id, const dmr_lrrp_coord_t *coord,
                            uint8_t *buf, size_t max_len, size_t *w_len)
{
    (void)request_id; /* NMEA ignores request IDs */

    /* Convert decimal degrees to NMEA standard (Degrees + Decimal Minutes) */
    int lat_deg = (int)coord->latitude;
    float lat_min = (coord->latitude - lat_deg) * 60.0f;
    char lat_dir = (coord->latitude >= 0) ? 'N' : 'S';
    if (lat_deg < 0) lat_deg = -lat_deg;
    if (lat_min < 0) lat_min = -lat_min;

    int lon_deg = (int)coord->longitude;
    float lon_min = (coord->longitude - lon_deg) * 60.0f;
    char lon_dir = (coord->longitude >= 0) ? 'E' : 'W';
    if (lon_deg < 0) lon_deg = -lon_deg;
    if (lon_min < 0) lon_min = -lon_min;

    /* Build the $GPRMC string (Time/Date hardcoded for proxy) */
    int written = snprintf((char *)buf, max_len,
                           "$GPRMC,120000,A,%02d%05.2f,%c,%03d%05.2f,%c,%.1f,%.1f,010124,,*00\r\n",
                           lat_deg, lat_min, lat_dir,
                           lon_deg, lon_min, lon_dir,
                           (float)coord->speed * 0.539957f, /* km/h to knots */
                           (float)coord->heading);

    if (written < 0 || (size_t)written >= max_len) {
        return DMR_ERR_NO_MEM;
    }

    *w_len = (size_t)written;
    return DMR_OK;
}

/* =========================================================================
 * Public Facade Routing
 * ========================================================================= */

dmr_err_t dmr_lrrp_parse_request(dmr_lrrp_type_t type, 
                                 const uint8_t *payload, 
                                 size_t length,
                                 dmr_lrrp_req_t *req)
{
    if (payload == NULL || req == NULL) return DMR_ERR_INVALID_PARAM;
    if (length == 0 || type == DMR_LRRP_TYPE_NONE) return DMR_ERR_INVALID_PARAM;

    memset(req, 0, sizeof(dmr_lrrp_req_t));

    switch (type) {
        case DMR_LRRP_TYPE_MOTOROLA: return parse_motorola_lrrp(payload, length, req);
        case DMR_LRRP_TYPE_ETSI_LIP: return parse_etsi_lip(payload, length, req);
        case DMR_LRRP_TYPE_NMEA:     return parse_nmea(payload, length, req);
        default:                     return DMR_ERR_INVALID_PARAM;
    }
}

dmr_err_t dmr_lrrp_build_response(dmr_lrrp_type_t type,
                                  uint32_t request_id,
                                  const dmr_lrrp_coord_t *coord,
                                  uint8_t *buf,
                                  size_t max_len,
                                  size_t *w_len)
{
    if (coord == NULL || buf == NULL || w_len == NULL) return DMR_ERR_INVALID_PARAM;
    if (type == DMR_LRRP_TYPE_NONE || !coord->fix_valid) return DMR_ERR_INVALID_PARAM;

    switch (type) {
        case DMR_LRRP_TYPE_MOTOROLA: return build_motorola_lrrp(request_id, coord, buf, max_len, w_len);
        case DMR_LRRP_TYPE_ETSI_LIP: return build_etsi_lip(request_id, coord, buf, max_len, w_len);
        case DMR_LRRP_TYPE_NMEA:     return build_nmea(request_id, coord, buf, max_len, w_len);
        default:                     return DMR_ERR_INVALID_PARAM;
    }
}