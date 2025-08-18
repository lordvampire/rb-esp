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
#include "RB02_GUIHelpers.h"
#include <stdio.h>


float nmea_to_decimal(float nmea_coord) {
    int degrees = (int)(nmea_coord / 100);
    double minutes = nmea_coord - (degrees * 100);
    return degrees + (minutes / 60.0);
}

uint16_t RB02_SuggestedQNH(float GPSAltitudeMeters,int32_t CurrentPressure)
{
    if(GPSAltitudeMeters<0.001 && GPSAltitudeMeters > -0.001)
    {
        return 1013;
    }
    // Altimeter is *100 feet
    int32_t SuggestedQNH = (((GPSAltitudeMeters * 3.28084 * 100.0)/30.0)+CurrentPressure)/100.0;
    return SuggestedQNH;
}

lv_obj_t *RB02_GUIHelpers_CreateBase(lv_obj_t *parent, const lv_img_dsc_t *backgroundImageName)
{
    lv_obj_t *backgroundImage = lv_img_create(parent);
    lv_img_set_src(backgroundImage, backgroundImageName);
    lv_obj_set_size(backgroundImage, backgroundImageName->header.w, backgroundImageName->header.h);
    // TODO: Migrate to SizeContent
    // lv_obj_set_size(backgroundImage,LV_SIZE_CONTENT,LV_SIZE_CONTENT);
    lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);
    // 1.1.9 Remove scrolling for Turbolence touch screen
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
    return backgroundImage;
}


uint8_t RB02_CheckfileExists(const char *filename)
{
    uint8_t ret = 0;
    // Test file exists:
    FILE *f = fopen(filename, "r");
    if (f == NULL)
    {
    }
    else
    {
        ret = 1;
        fclose(f);
    }

    return ret;
}