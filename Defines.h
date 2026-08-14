/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2023,2025,2026 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *   ---
 *   Converted from C++ to C. Differences from the original:
 *     - `enum class` (scoped enums) are not part of the C language, so each
 *       becomes a plain C `typedef enum`. C enumerators live in the global
 *       namespace (there is no RPT_RF_STATE::LISTENING scoping), so this
 *       header prefixes each enumerator with its former enum name to avoid
 *       collisions, e.g. RPT_RF_STATE::LISTENING -> RPT_RF_STATE_LISTENING.
 *   ---
 *   Trimmed to DMR-only. Removed: the USE_DSTAR/USE_YSF/USE_P25/USE_NXDN/
 *   USE_POCSAG/USE_FM feature macros (only USE_DMR is kept/defined),
 *   DSTAR_MODEM_DATA_LEN, and the DSTAR_ACK enum, none of which are
 *   referenced anywhere in the DMR code path (BPTC19696/Hamming/CRC/Utils).
 */

#if !defined(Defines_H)
#define	Defines_H

/* Define the wanted modes to compile into the host here.
   Trimmed to DMR-only; the other USE_* protocol macros have been removed
   along with the code that was gated on them. */
#define	USE_DMR

static const unsigned char MODE_IDLE    = 0U;
static const unsigned char MODE_DSTAR   = 1U;
static const unsigned char MODE_DMR     = 2U;
static const unsigned char MODE_YSF     = 3U;
static const unsigned char MODE_P25     = 4U;
static const unsigned char MODE_NXDN    = 5U;
static const unsigned char MODE_POCSAG  = 6U;

static const unsigned char MODE_FM      = 10U;

static const unsigned char MODE_CW      = 98U;
static const unsigned char MODE_LOCKOUT = 99U;
static const unsigned char MODE_ERROR   = 100U;
static const unsigned char MODE_QUIT    = 110U;

static const unsigned char TAG_HEADER = 0x00U;
static const unsigned char TAG_DATA   = 0x01U;
static const unsigned char TAG_LOST   = 0x02U;
static const unsigned char TAG_EOT    = 0x03U;
static const unsigned char TAG_RSSI   = 0x04U;

typedef enum {
	RPT_RF_STATE_LISTENING,
	RPT_RF_STATE_LATE_ENTRY,
	RPT_RF_STATE_AUDIO,
	RPT_RF_STATE_DATA_AUDIO,
	RPT_RF_STATE_DATA,
	RPT_RF_STATE_REJECTED,
	RPT_RF_STATE_INVALID
} RPT_RF_STATE;

typedef enum {
	RPT_NET_STATE_IDLE,
	RPT_NET_STATE_AUDIO,
	RPT_NET_STATE_DATA_AUDIO,
	RPT_NET_STATE_DATA
} RPT_NET_STATE;

typedef enum {
	DMR_BEACONS_OFF,
	DMR_BEACONS_NETWORK,
	DMR_BEACONS_TIMED
} DMR_BEACONS;

typedef enum {
	DMR_OVCM_OFF,
	DMR_OVCM_RX_ON,
	DMR_OVCM_TX_ON,
	DMR_OVCM_ON,
	DMR_OVCM_FORCE_OFF
} DMR_OVCM;

#endif
