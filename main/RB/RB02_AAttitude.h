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
#include "RB02.h"
#ifdef RB_ENABLE_AAT
#include "RB02_NMEA.h"

#define RB_AAT_ARC_NUMBERS 15

typedef struct
{
    lv_obj_t *lv_parent;
    lv_obj_t *lv_pitch;
    lv_obj_t *lv_roll;
    lv_obj_t *lv_speed;
    lv_obj_t *lv_speed_background;
    lv_obj_t *lv_gyro;
    lv_obj_t *lv_gyro_pink;
    lv_obj_t *lv_track;
    lv_obj_t *lv_gmeter;
    lv_obj_t *lv_variometer;
    lv_obj_t *lv_altimeter;
    lv_obj_t *lv_altimeter_background;
    lv_obj_t *lv_qnh;
    lv_obj_t *lv_ball;
    lv_obj_t *lv_left_arcs[RB_AAT_ARC_NUMBERS];
    lv_obj_t *lv_right_arcs[RB_AAT_ARC_NUMBERS];
    
} RB02_AdvancedAttitude_Status;


extern RB02_AdvancedAttitude_Status advancedAttitude_Status;

lv_obj_t *RB02_AdvancedAttitude_CreateScreen(RB02_AdvancedAttitude_Status *aaStatus, const lv_img_dsc_t *AircraftIndicatorMiddle, const lv_img_dsc_t *AircraftIndicatorTop);
void RB02_AdvancedAttitude_Tick(RB02_AdvancedAttitude_Status *aaStatus, gps_t *gpsStatus,int32_t Altimeter,int32_t QNH,int32_t Variometer);
#endif