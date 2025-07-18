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

#include "lvgl.h"
#include "RB02_Defines.h"

#ifdef RB_ENABLE_MAP
#include "RB02_GPSMap.h"
#endif

#define RB02_STRUCTURE_CONFIG 1

typedef struct
{
    uint8_t settingsCalibrateOnBoot;
    uint8_t settingsAutoQNH;
    int32_t bmp280override;
    uint8_t structureVersion;
#ifdef RB_ENABLE_DATALOGGER
    uint8_t settingsEnableDataLoggerRecording;
#endif
#ifdef RB_ENABLE_MAP
RB02_GpsMapStatus gpsMapStatus;
#endif
} RB02_Status;

extern RB02_Status *rb02Status;

RB02_Status *singletonConfig();