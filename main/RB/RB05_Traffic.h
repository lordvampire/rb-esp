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
#pragma once
#include "lvgl.h"
#include "RB02_Defines.h"

#ifdef RB_ENABLE_TRAFFIC


typedef struct
{
        uint16_t lastTimeDirty;
        uint8_t zoomLevel;
        uint8_t verticalFilter;
        lv_obj_t *lv_parent;
        lv_obj_t *lv_lines;
        lv_obj_t *lv_myself;
        lv_obj_t *lv_parent_radar;
        lv_obj_t *lv_label_lower;
        lv_obj_t *Numbers[4];
        lv_obj_t *lv_obstacles[RB_ENABLE_TRAFFIC];
        lv_obj_t *lv_obstacles_label[RB_ENABLE_TRAFFIC];
}RB05_TrafficStatus;


lv_obj_t *RB05_Traffic_CreateScreen(RB05_TrafficStatus *tStatus);
void RB05_Traffic_Tick(RB05_TrafficStatus *tStatus);
void RB05_Traffic_Touch_N(RB05_TrafficStatus *tStatus);
void RB05_Traffic_Touch_S(RB05_TrafficStatus *tStatus);
void RB05_Traffic_Touch_Center(RB05_TrafficStatus *tStatus);
#endif