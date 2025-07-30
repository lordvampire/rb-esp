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
#include "RB02_Gyro.h"

#include "RB02_GUIHelpers.h"
#include <stdio.h>
// Gyro
#include "RoundGyro.c"
#include "images/Bearing_0.c"
#include "images/Bearing_3.c"
#include "images/Bearing_6.c"
#include "images/Bearing_9.c"
#include "images/Bearing_12.c"
#include "images/Bearing_15.c"
#include "images/Bearing_18.c"
#include "images/Bearing_21.c"
#include "images/Bearing_24.c"
#include "images/Bearing_27.c"
#include "images/Bearing_30.c"
#include "images/Bearing_33.c"

lv_obj_t *RB02_Gyro_CreateScreen(lv_obj_t *parent)
{
    RB02_GUIHelpers_CreateBase(parent, &RoundGyro);

    singletonConfig()->ui.Gyro.Numbers[0] = RB02_GUIHelpers_CreateBase(parent, &Bearing_0);
    singletonConfig()->ui.Gyro.Numbers[1] = RB02_GUIHelpers_CreateBase(parent, &Bearing_3);
    singletonConfig()->ui.Gyro.Numbers[2] = RB02_GUIHelpers_CreateBase(parent, &Bearing_6);
    singletonConfig()->ui.Gyro.Numbers[3] = RB02_GUIHelpers_CreateBase(parent, &Bearing_9);
    singletonConfig()->ui.Gyro.Numbers[4] = RB02_GUIHelpers_CreateBase(parent, &Bearing_12);
    singletonConfig()->ui.Gyro.Numbers[5] = RB02_GUIHelpers_CreateBase(parent, &Bearing_15);
    singletonConfig()->ui.Gyro.Numbers[6] = RB02_GUIHelpers_CreateBase(parent, &Bearing_18);
    singletonConfig()->ui.Gyro.Numbers[7] = RB02_GUIHelpers_CreateBase(parent, &Bearing_21);
    singletonConfig()->ui.Gyro.Numbers[8] = RB02_GUIHelpers_CreateBase(parent, &Bearing_24);
    singletonConfig()->ui.Gyro.Numbers[9] = RB02_GUIHelpers_CreateBase(parent, &Bearing_27);
    singletonConfig()->ui.Gyro.Numbers[10] = RB02_GUIHelpers_CreateBase(parent, &Bearing_30);
    singletonConfig()->ui.Gyro.Numbers[11] = RB02_GUIHelpers_CreateBase(parent, &Bearing_33);

    // 1.1.1 Branding RB-02 on every screen
    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, 96, 40);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "RB 02");
    }

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, 300, 40);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, -26);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
#ifdef RB_ENABLE_GPS
        lv_label_set_text(label, "GPS TRACK");
#else
        lv_label_set_text(label, "GYRO");
#endif

        singletonConfig()->ui.Gyro.Screen_Track_TrackSource = label;
    }

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, 128, 48);
        lv_obj_align(label, LV_ALIGN_CENTER, 8, -8); // Displacement to center the "°"
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_style_bg_color(label, lv_color_black(), 0);
        lv_obj_set_style_radius(label, LV_RADIUS_CIRCLE, 0);
        char buf[4];
        snprintf(buf, sizeof(buf), "%d°", 0);
        lv_label_set_text(label, buf);

        singletonConfig()->ui.Gyro.Screen_Track_TrackText = label;
    }

    return NULL;
}

