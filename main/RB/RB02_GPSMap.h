#pragma once

/**
 * This file is part of RB.
 *
 * Copyright (C) 2024 XIAPROJECTS SRL
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.

 * This source is part of the project RB:
 * 01 -> Display with Synthetic vision, Autopilot and ADSB
 * 02 -> Display with SixPack
 * 03 -> Display with Autopilot, ADSB, Radio, Flight Computer
 * 04 -> Display with EMS: Engine monitoring system
 * 
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#include "RB02_Defines.h"


#ifdef RB_ENABLE_MAP

#include "lvgl.h"
#include "RB02_NMEA.h"



// 1.1.22 MBTILES support
typedef struct
{
    int x;
    int y_tms;
} TileXY;

typedef struct
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    lv_obj_t *tiles[9];

    uint8_t zoomLevel;
    uint8_t mapDirty;

    bool enableMercatoreLatLon;
#endif
    uint16_t tileSizeHeight;
    uint16_t tileSizeWidth;
    int32_t latitude100;
    int32_t longitude100;
    lv_obj_t *poiMy;
    lv_obj_t *labelLatitude;
    lv_obj_t *labelTilePath;
    lv_obj_t *labelLongitude;
    int32_t mapLatitudeBegin;
    int32_t mapLongitudeBegin;
    int32_t mapLatitudeEnd;
    int32_t mapLongitudeEnd;
    TileXY lastTile;
} RB02_GpsMapStatus;

lv_obj_t *RB02_GPSMap_CreateScreen(RB02_GpsMapStatus *gpsMapStatus, lv_obj_t *parent);

void RB02_GPSMap_Tick(RB02_GpsMapStatus *gpsMapStatus, gps_t *gpsStatus,lv_obj_t *parent);
void RB02_GPSMap_Touch_N(RB02_GpsMapStatus *gpsMapStatus);
void RB02_GPSMap_Touch_S(RB02_GpsMapStatus *gpsMapStatus);
#endif