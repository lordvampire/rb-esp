/**
 * Copyright (c) 2024 XIAPROJECTS SRL
 * Distributable under the terms of The "BSD New" License
 * that can be found in the LICENSE file, herein included
 * as part of this header.
 * This source is part of the project RB:
 * 01 -> Display 2.8" with Synthetic vision, Autopilot and ADSB
 * 02 -> Display 2.8" with SixPack
 * 03 -> Autopilot, ADSB, Radio, Flight Computer
 */
#pragma once
#include "lvgl.h"
#include "RB02_NMEA.h"
#include "RB02.h"

#ifdef RB_ENABLE_MAP
#ifdef RB_02_ENABLE_EXTERNALMAP
#define RB_02_ENABLE_EXTERNALMAP_WIDTH 30 // Maximun zoom out
#endif

typedef struct
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    lv_obj_t *tiles[(SCREEN_WIDTH/RB_02_ENABLE_EXTERNALMAP_WIDTH)*(SCREEN_WIDTH/RB_02_ENABLE_EXTERNALMAP_WIDTH)];
    uint16_t tileSizeHeight;
    uint16_t tileSizeWidth;
    uint8_t zoomLevel;
    uint8_t mapDirty;
    int32_t latitude100;
    int32_t longitude100;
#endif
    lv_obj_t *poiMy;
    lv_obj_t *labelLatitude;
    lv_obj_t *labelTilePath;
    lv_obj_t *labelLongitude;
    int32_t mapLatitudeBegin;
    int32_t mapLongitudeBegin;
    int32_t mapLatitudeEnd;
    int32_t mapLongitudeEnd;
} RB02_GpsMapStatus;

lv_obj_t *RB02_GPSMap_CreateScreen(RB02_GpsMapStatus *gpsMapStatus, lv_obj_t *parent);

void RB02_GPSMap_Tick(RB02_GpsMapStatus *gpsMapStatus, gps_t *gpsStatus,lv_obj_t *parent);
void RB02_GPSMap_Touch_N(RB02_GpsMapStatus *gpsMapStatus);
void RB02_GPSMap_Touch_S(RB02_GpsMapStatus *gpsMapStatus);
#endif