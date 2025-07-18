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



#define EXAMPLE1_LVGL_TICK_PERIOD_MS  1000


// 1.1.18 Unified single source for both displays
#define RB_02_DISPLAY_21 21
#define RB_02_DISPLAY_28 28
#define RB_02_DISPLAY_TOUCH 1

//  1.1.3 Supports for 2.1 and 2.8 displays
#define RB_02_DISPLAY_SIZE RB_02_DISPLAY_28
#define SCREEN_HEIGHT 480
#define SCREEN_WIDTH 480


#define RB_LICENSE_TYPE_COMMUNITY   0
#define RB_LICENSE_TYPE_COMMERCIAL  1
#define RB_LICENSE_TYPE_CERTIFIED   2
#define RB_LICENSE_TYPE_DEVELOPER   3
#define RB_LICENSE_TYPE RB_LICENSE_TYPE_COMMUNITY

// 1.1.24 int16_t
#define RB_GYRO_CALIBRATION_PRECISION 1000.0

// 1.1.19 Add Map capability
#define RB_ENABLE_MAP 1
//#define RB_02_ENABLE_INTERNALMAP 1
#define RB_02_ENABLE_EXTERNALMAP 1
#define RB_ENABLE_CHECKLIST 1
#define RB_ENABLE_DATALOGGER 1

// 1.1.20 Advanced Attitude Indicator
#define RB_ENABLE_AAT 1


// 1.1.23 GPS Diagnostic Screen
//#define RB_ENABLE_GPS_DIAG 1
//#define RB_ENABLE_CONSOLE_DEBUG 1

// 1.1.2 Version is here
#define RB_VERSION "1.1.24"
// 1.1.1 Remove tabs with GPS if not installed
#define RB_ENABLE_GPS 1
// 1.1.19 Starting getting rid of demo screens
// #define ENABLE_DEMO_SCREENS 1
#define VIBRATION_TEST 1
