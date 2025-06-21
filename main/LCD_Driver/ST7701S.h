#include "../RB/RB02.h"

#ifndef ST7701S_INCLUDE
#define ST7701S_INCLUDE
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_28
// ESP32-S3-2.8C
#include "ST7701S_28.h"
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_21
// ESP32-S3-2.1C
#include "ST7701S_21.h"
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
extern SemaphoreHandle_t sem_vsync_end;
extern SemaphoreHandle_t sem_gui_ready;
#endif
#endif
#endif