#pragma once

#include "lvgl.h"

#define EXAMPLE1_LVGL_TICK_PERIOD_MS  1000


// 1.1.18 Unified single source for both displays
#define RB_02_DISPLAY_21 21
#define RB_02_DISPLAY_28 28

//  1.1.3 Supports for 2.1 and 2.8 displays
#define RB_02_DISPLAY_SIZE RB_02_DISPLAY_28

void Backlight_adjustment_event_cb(lv_event_t * e);

void RB02_Example1(void);
void LVGL_Backlight_adjustment(uint8_t Backlight);