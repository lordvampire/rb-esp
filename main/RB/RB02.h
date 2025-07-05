#pragma once

#include "lvgl.h"

#define EXAMPLE1_LVGL_TICK_PERIOD_MS  1000


// 1.1.18 Unified single source for both displays
#define RB_02_DISPLAY_21 21
#define RB_02_DISPLAY_28 28

//  1.1.3 Supports for 2.1 and 2.8 displays
#define RB_02_DISPLAY_SIZE RB_02_DISPLAY_21
#define SCREEN_HEIGHT 480
#define SCREEN_WIDTH 480

// 1.1.19 Add Map capability
#define RB_ENABLE_MAP 1
//#define RB_02_ENABLE_INTERNALMAP 1
#define RB_02_ENABLE_EXTERNALMAP 1
#define RB_ENABLE_CHECKLIST 1

// 1.1.20 Advanced Attitude Indicator
#define RB_ENABLE_AAT 1

void Backlight_adjustment_event_cb(lv_event_t * e);

void RB02_Example1(void);
void LVGL_Backlight_adjustment(uint8_t Backlight);
void draw_arch(lv_obj_t *parent, const lv_img_dsc_t *t, uint16_t degreeStartSlide, uint16_t degreeEndSlide);
extern const lv_res_t Screen_TurnSlip_Obj_Ball_Size;