/**
 * This file is part of RB.
 *
 * Copyright (C) 2025 XIAPROJECTS SRL
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
 * 05 -> Display with Stratux BLE Traffic
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#include "RB04_EMS.h"
#ifdef RB_ENABLE_EMS
#include "RB02_GUIHelpers.h"
 #include "images/EMSBackground.c" // DEMO

#define RB_EMS_VERTICAL_BARS_NUMBER 8

typedef struct
{
    lv_obj_t *label;
    lv_obj_t *labelContainer;
    lv_obj_t *display;
    int16_t thresholdGreen;
    int16_t thresholdYellow;
    int16_t thresholdRed;
    lv_obj_t *triangleDirection;
    int16_t value;

} RB04_EMSSWidget;

typedef struct
{
    RB04_EMSSWidget topLeft;
    RB04_EMSSWidget topRight;
    RB04_EMSSWidget bottomLeft;
    RB04_EMSSWidget bottomRight;
    RB04_EMSSWidget verticalBars[RB_EMS_VERTICAL_BARS_NUMBER];

} RB04_EMSStatus;

lv_obj_t *RB04_EMS_DrawLine(lv_obj_t *parent, lv_coord_t width, lv_coord_t height, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *line = lv_obj_create(parent);
    lv_obj_set_scrollbar_mode(line, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(line, width, height);
    lv_obj_set_style_bg_color(line, lv_color_make(255, 255, 255), 0);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_align(line, LV_ALIGN_CENTER, x, y);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
    return line;
}

lv_obj_t *RB04_EMS_CreateHorizontalThresholds(lv_obj_t *parent, RB04_EMSStatus *emsStatus)
{
    lv_coord_t x = 0;
    lv_coord_t y = 0;
    for (uint8_t k = 0; k < 5; k++)
    {
        RB04_EMS_DrawLine(parent, 440, 2, x, y + k * 25 - 200);
    }

    return NULL;
}

lv_obj_t *RB04_EMS_CreateVerticalBar(lv_obj_t *parent, RB04_EMSSWidget *widget, uint8_t index)
{
    lv_coord_t x = 0;
    lv_coord_t y = 0;
    lv_coord_t w = 24;
    lv_coord_t h = 380;
    widget->display = RB04_EMS_DrawLine(parent, w, h, x + (index * 32) - (RB_EMS_VERTICAL_BARS_NUMBER / 2.0 * 32), y);

    return NULL;
}

lv_obj_t *RB04_EMS_CreateScreen(lv_obj_t *parent, RB02_Status *status)
{
     RB02_GUIHelpers_CreateBase(parent, &EMSBackground); // DEMO
    return NULL;
    
     if (status->ui.ems == NULL)
    {
        status->ui.ems = lv_mem_alloc(sizeof(RB04_EMSStatus));
        if (rb02Status != NULL)
        {
            lv_memset_00(status->ui.ems, sizeof(RB04_EMSStatus));
        }
    }

    RB04_EMSStatus *emsStatus = (RB04_EMSStatus *)status->ui.ems;

    for (uint8_t x = 0; x < RB_EMS_VERTICAL_BARS_NUMBER; x++)
    {
        RB04_EMS_CreateVerticalBar(parent, &(emsStatus->verticalBars[x]), x);
    }
    RB04_EMS_CreateHorizontalThresholds(parent, emsStatus);

    return NULL;
}


void RB04_EMS_Tick(void *status)
{
    
}
void RB04_EMS_Touch_N(void *status)
{

}
void RB04_EMS_Touch_S(void *status)
{

}
void RB04_EMS_Touch_W(void *status)
{

}
void RB04_EMS_Touch_E(void *status)
{

}
#endif