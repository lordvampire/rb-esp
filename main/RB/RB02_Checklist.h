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

lv_obj_t *RB02_Checklist_CreateScreen(lv_obj_t *parent, char * filename);