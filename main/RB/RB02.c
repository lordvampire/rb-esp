/**
 * Copyright (c) 2024 XIAPROJECTS SRL
 * Distributable under the terms of The "BSD New" License
 * that can be found in the LICENSE file, herein included
 * as part of this header.
 * This source is part of the project RB:
 * 01 -> Display 2.8" with Synthetic vision, Autopilot and ADSB
 * 02 -> Display 2.8" with SixPack
 * 03 -> Autopilot, ADSB, Radio, Flight Computer
 *
 *
 * RB02.c
 * Implementation of RoastBeef PN 02 The basic six pack version
 *
 * Features:
 * - GPS Speed -> Requires NMEA TTL RS232 GPS Reveiver such as ublox
 * - Attitude indicator
 * - Turn & Slip
 * - GPS Gyro-Track -> Requires NMEA TTL RS232 GPS Reveiver such as ublox
 * - Altimeter
 * - Variometer
 * - Chronometer
 *
 * Demo Features:
 * - Synthetic vision lateral, implemented in RB-01
 * - Synthetic vision front, implemented in RB-01
 *
 * Integrated Features:
 * - Speed indicator
 * - Track indicator
 * - Radar display
 * - GPS Time
 *
 * Supported hardware:
 * - ESP32-S3 2.8" Inch Round display 480x480
 * - https://www.waveshare.com/esp32-s3-touch-lcd-2.8c.htm
 */
#include "RB02.h"
// 1.1.2 Version is here
#define RB_VERSION "1.1.2"
// 1.1.1 Remove tabs with GPS if not installed
//#define RB_ENABLE_GPS 1

// Images pre-loaded
#ifdef ENABLE_DEMO_SCREENS
#include "RoundSynthViewAttitude.c"
#endif

#include "RoundAltimeter.c"

#include "RoundGyro.c"
#include "RoundGyroHeading.c"
#include "RoundVariometer.c"
#include "turn_coordinator.c"
#include "fi_tc_airplane.c"
#include "horizon_ball.c"

#include "fi_needle.c"
#include "fi_needle_small.c"
#include "GMeter.c"
#include "AttitudeBackground2.c"
#include "Sky.c"
#include "AttitudePlain.c"
#include "AttitudeMiddle.c"
#ifdef ENABLE_DEMO_SCREENS
#include "RoundSynthViewSide.c"
#include "Radar.c"
#include "RoundMapWithControlledSpaces.c"
#include "RoundHSI.c"
#endif
#include "att_circle_top_TL.c"
#include "att_circle_top_TR.c"
#include "att_circle_top_T.c"
#include "att_circle_middle.c"
#include "att_middle_big.c"
#include "att_aircraft.c"
#include "att_tri.c"
#include "DigitFont100x25.c"
#include "DigitFont70x20.c"
#include "arcRed.c"
#include "arcYellow.c"
#include "arcGreen.c"
#include "arcWhite.c"

// External dependencies
#include "LVGL_Example.h"

extern uint8_t LCD_Backlight;
extern IMUdata AccelFilteredMax;
extern IMUdata GyroBias;
extern lv_coord_t TouchPadLastX;
extern lv_coord_t TouchPadLastY;
extern float AttitudeBalanceAlpha;
extern float FilterMoltiplier;
extern lv_style_t style_text_muted;
extern lv_style_t style_title;
extern float GFactor;
extern float GFactorMax;
extern float GFactorMin;
extern uint8_t GFactorDirty;

/* 1.0.9 For future compatibility imported NMEA ESP32 Example header */
#define CONFIG_NMEA_PARSER_RING_BUFFER_SIZE 2048
#define CONFIG_NMEA_PARSER_TASK_STACK_SIZE 4 * 1024
#define CONFIG_NMEA_PARSER_TASK_PRIORITY 11
#define CONFIG_NMEA_PARSER_UART_RXD 44
#define CONFIG_NMEA_STATEMENT_RMC 1

#define GPS_MAX_SATELLITES_IN_USE (12)
#define GPS_MAX_SATELLITES_IN_VIEW (16)

/**
 * @brief Declare of NMEA Parser Event base
 *
 */
ESP_EVENT_DECLARE_BASE(ESP_NMEA_EVENT);

/**
 * @brief GPS fix type
 *
 */
typedef enum
{
  GPS_FIX_INVALID, /*!< Not fixed */
  GPS_FIX_GPS,     /*!< GPS */
  GPS_FIX_DGPS,    /*!< Differential GPS */
} gps_fix_t;

/**
 * @brief GPS fix mode
 *
 */
typedef enum
{
  GPS_MODE_INVALID = 1, /*!< Not fixed */
  GPS_MODE_2D,          /*!< 2D GPS */
  GPS_MODE_3D           /*!< 3D GPS */
} gps_fix_mode_t;

/**
 * @brief GPS satellite information
 *
 */
typedef struct
{
  uint8_t num;       /*!< Satellite number */
  uint8_t elevation; /*!< Satellite elevation */
  uint16_t azimuth;  /*!< Satellite azimuth */
  uint8_t snr;       /*!< Satellite signal noise ratio */
} gps_satellite_t;

/**
 * @brief GPS time
 *
 */
typedef struct
{
  uint8_t hour;      /*!< Hour */
  uint8_t minute;    /*!< Minute */
  uint8_t second;    /*!< Second */
  uint16_t thousand; /*!< Thousand */
} gps_time_t;

/**
 * @brief GPS date
 *
 */
typedef struct
{
  uint8_t day;   /*!< Day (start from 1) */
  uint8_t month; /*!< Month (start from 1) */
  uint16_t year; /*!< Year (start from 2000) */
} gps_date_t;

/**
 * @brief GPS object
 *
 */
typedef struct
{
  float latitude;                                                /*!< Latitude (degrees) */
  float longitude;                                               /*!< Longitude (degrees) */
  float altitude;                                                /*!< Altitude (meters) */
  gps_fix_t fix;                                                 /*!< Fix status */
  uint8_t sats_in_use;                                           /*!< Number of satellites in use */
  gps_time_t tim;                                                /*!< time in UTC */
  gps_fix_mode_t fix_mode;                                       /*!< Fix mode */
  uint8_t sats_id_in_use[GPS_MAX_SATELLITES_IN_USE];             /*!< ID list of satellite in use */
  float dop_h;                                                   /*!< Horizontal dilution of precision */
  float dop_p;                                                   /*!< Position dilution of precision  */
  float dop_v;                                                   /*!< Vertical dilution of precision  */
  uint8_t sats_in_view;                                          /*!< Number of satellites in view */
  gps_satellite_t sats_desc_in_view[GPS_MAX_SATELLITES_IN_VIEW]; /*!< Information of satellites in view */
  gps_date_t date;                                               /*!< Fix date */
  bool valid;                                                    /*!< GPS validity */
  float speed;                                                   /*!< Ground speed, unit: m/s */
  float cog;                                                     /*!< Course over ground */
  float variation;                                               /*!< Magnetic variation */
} gps_t;

gps_t NMEA_DATA;
int32_t GpsSpeed0ForDisable = 0;

// DEFINES
#define RB02_TOUCH_SECTION 3
#define RB02_TOUCH_SECTION_SIZE 160
#define DIGIT_BIG_SEGMENTS 7
#define DIGIT_BIG_DIGIT 4
#define DIGIT_MINOR_DIGIT 2
#define SCREEN_HEIGHT 480
#define SCREEN_WIDTH 480
#define BMP280_S64_t int64_t
#define BMP280_U32_t uint32_t
#define BMP280_S32_t int32_t
#ifdef RB_ENABLE_GPS
static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_43)
#define RXD_PIN (GPIO_NUM_44)
#define UART_N UART_NUM_1
#endif
// Prototype declaration
static lv_obj_t *Onboard_create_Base(lv_obj_t *parent, const lv_img_dsc_t *backgroundImage);
#ifdef RB_ENABLE_GPS
static void Onboard_create_Speed(lv_obj_t *parent);
#endif
static void Onboard_create_Attitude(lv_obj_t *parent);
static void Onboard_create_Altimeter(lv_obj_t *parent);
static void Onboard_create_AltimeterDigital(lv_obj_t *parent);
static void Onboard_create_TurnSlip(lv_obj_t *parent);
static void Onboard_create_Clock(lv_obj_t *parent);
static void Onboard_create_Setup(lv_obj_t *parent);
static void Onboard_create_Track(lv_obj_t *parent);
static void Onboard_create_Variometer(lv_obj_t *parent);
static void Onboard_create_GMeter(lv_obj_t *parent);
static void Onboard_create_Setup(lv_obj_t *parent);
static void speedBgClicked(lv_event_t *event);
uint32_t timerDiffByIndex(int st);
void rb_increase_lvgl_tick(lv_timer_t *t);
void update_GMeter_lvgl_tick(lv_timer_t *t);
void update_Clock_lvgl_tick(lv_timer_t *t);
void update_AltimeterDigital_lvgl_tick(lv_timer_t *t);
void update_Altimeter_lvgl_tick(lv_timer_t *t);
#ifdef RB_ENABLE_GPS
void uart_fetch_data();
#endif
void nvsStorePCal();
void nvsStoreUARTBaudrate();

static void CreateDigitArray(lv_obj_t *parent, lv_img_dsc_t *font, int howMany, int dx, int dy);
static void CreateSingleDigit(lv_obj_t *parent, lv_img_dsc_t *font, lv_obj_t **segments, int dx, int dy);
void nvsStoreGMeter();
void nvsRestoreGMeter();

// Variables

typedef enum
{
  RB02_TOUCH_NW,
  RB02_TOUCH_N,
  RB02_TOUCH_NE,
  RB02_TOUCH_W,
  RB02_TOUCH_CENTER,
  RB02_TOUCH_E,
  RB02_TOUCH_SW,
  RB02_TOUCH_S,
  RB02_TOUCH_SE,
  RB02_TOUCH_UNKNOWN
} touchLocation;

typedef enum
{
#ifdef ENABLE_DEMO_SCREENS
  RB02_TAB_SYS,
  RB02_TAB_SYN,
  RB02_TAB_RDR,
  // RB02_TAB_HSI,
  RB02_TAB_MAP,
#endif
#ifdef RB_ENABLE_GPS
  RB02_TAB_SPD,
#endif
  RB02_TAB_ATT,
  RB02_TAB_ALT,
  RB02_TAB_ALD,
  RB02_TAB_TRN,
#ifdef RB_ENABLE_GPS
  RB02_TAB_TRK,
#endif
  RB02_TAB_VAR,
  RB02_TAB_GMT,
  RB02_TAB_CLK,
  RB02_TAB_SET,
  RB02_TAB_DEV
} tabs;

static lv_obj_t *kb;
lv_obj_t *uartDropDown = NULL;
lv_obj_t *kmhDropDown = NULL;
lv_obj_t *SettingStatus0 = NULL;
lv_obj_t *SettingStatus1 = NULL;
lv_obj_t *SettingStatus2 = NULL;
lv_obj_t *SettingStatus3 = NULL;
lv_obj_t *SettingStatus4UART = NULL;

lv_obj_t *SettingLabelFilter = NULL;

lv_obj_t *SettingAttitudeCompensation = NULL;

lv_obj_t *Ball = NULL;
lv_obj_t *Screen_Attitude_Pitch = NULL;
lv_obj_t *Screen_Attitude_RollIndicator = NULL;
void *Screen_Attitude_Rounds[4];
uint16_t QNH = 1013;
lv_obj_t *Screen_Gyro_Gear = NULL;
lv_obj_t *Screen_Altitude_Miles = NULL;
lv_obj_t *Screen_Altitude_Cents = NULL;
lv_obj_t *Screen_Variometer_Cents = NULL;
lv_obj_t *Screen_Altitude_QNH = NULL;
lv_obj_t *Screen_Altitude_QNH2 = NULL;
lv_obj_t *Screen_Altitude_Variometer2 = NULL;
lv_obj_t *Screen_Altitude_Pressure = NULL;
int lastAttitudePitch = 0;
int lastAttitudeRoll = 0;
datetime_t stopwatch = {0};
uint8_t DeviceIsDemoMode = 0;
tabs StartupPage = 0;
lv_obj_t *Loading_slider = NULL;
lv_timer_t *auto_step_timer = NULL;
lv_obj_t *tv = NULL;
const lv_font_t *font_large;
const lv_font_t *font_normal;
lv_obj_t *TimerLabelTop = NULL;
lv_obj_t *TimerLabelSW = NULL;
lv_obj_t *TimerLabelSE = NULL;
lv_obj_t *TimerSW = NULL;
lv_obj_t *TimerSE = NULL;
uint8_t selectedTimer = 0;
datetime_t datetimeTimer1 = {0};
datetime_t datetimeTimer2 = {0};
datetime_t datetimeTimer3 = {0};
lv_res_t Screen_TurnSlip_Obj_Ball_Size = SCREEN_HEIGHT / 12;
lv_obj_t *Screen_TurnSlip_Obj_Ball = NULL;
lv_obj_t *Screen_TurnSlip_Obj_Turn = NULL;
lv_obj_t *Screen_Speed_SpeedText = NULL;
lv_obj_t *Screen_Speed_SpeedTick = NULL;
lv_obj_t *Screen_Track_TrackText = NULL;
lv_obj_t *Screen_GMeter_Ball = NULL;
lv_obj_t *Screen_GMeter_BallMax = NULL;
lv_obj_t *GMeterLabel = NULL;
lv_obj_t *GMeterLabelMax = NULL;
float GMeterScale = 3.0;
IMUdata GyroBiasAcquire[3];
lv_obj_t *mbox1 = NULL;
uint8_t isKmh = 0;
uint16_t degreeStart = 0;
uint16_t degreeEnd = 320;
uint16_t speedKtStart = 0;
uint16_t speedKtEnd = 200;
uint16_t speedWhite = 60;
uint16_t speedGreen = 105;
uint16_t speedYellow = 225;
uint16_t speedRed = 315;
lv_obj_t *t_speedSummary = NULL;
lv_obj_t *t_speedStart = NULL;
lv_obj_t *t_speedEnd = NULL;
lv_obj_t *t_speedStartSpeed = NULL;
lv_obj_t *t_speedEndSpeed = NULL;
lv_obj_t *t_speedWhite = NULL;
lv_obj_t *t_speedGreen = NULL;
lv_obj_t *t_speedYellow = NULL;
lv_obj_t *t_speedRed = NULL;

uint8_t workflow = 0;
int32_t bmp280override = 0;
int32_t bmp280Calibration[12];
BMP280_S32_t t_fine = 0;
lv_obj_t *bmp280overrideLabel = NULL;
lv_obj_t *SegmentsA[DIGIT_BIG_DIGIT][DIGIT_BIG_SEGMENTS];
lv_obj_t *SegmentsAltDigit[5][DIGIT_BIG_SEGMENTS];
// lv_obj_t *SegmentsB[DIGIT_MINOR_DIGIT][DIGIT_BIG_SEGMENTS];

/**********************
 *  STATIC PROTOTYPES
 **********************/

touchLocation getTouchLocation(lv_coord_t x, lv_coord_t y)
{
  int8_t sx = x / RB02_TOUCH_SECTION_SIZE;
  int8_t sy = y / RB02_TOUCH_SECTION_SIZE;
  touchLocation returnCode = sy * RB02_TOUCH_SECTION + sx;
  return returnCode;
}

void ApplyCoding(void)
{
  DeviceIsDemoMode = 1;
  // StartupPage = RB02_TAB_ATT;
}

void RB02_Example1(void)
{

  font_large = LV_FONT_DEFAULT;
  font_normal = LV_FONT_DEFAULT;

  lv_coord_t tab_h;
  tab_h = 0;
#if LV_FONT_MONTSERRAT_20
  font_large = &lv_font_montserrat_20;
#else
  LV_LOG_WARN("LV_FONT_MONTSERRAT_20 is not enabled for the widgets demo. Using LV_FONT_DEFAULT instead.");
#endif
#if LV_FONT_MONTSERRAT_14
  font_normal = &lv_font_montserrat_14;
#else
  LV_LOG_WARN("LV_FONT_MONTSERRAT_14 is not enabled for the widgets demo. Using LV_FONT_DEFAULT instead.");
#endif

  ApplyCoding();
  nvsRestoreGMeter();
  LCD_Backlight = 0;
  LVGL_Backlight_adjustment(LCD_Backlight);

  lv_style_init(&style_title);
  // lv_style_set_text_font(&style_title, font_large);
  lv_style_set_text_font(&style_title, &lv_font_montserrat_16);
  lv_style_set_text_color(&style_title, lv_color_white());
  tv = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, tab_h);
  lv_obj_set_style_text_font(lv_scr_act(), font_normal, 0);
  lv_obj_set_style_bg_color(tv, lv_color_black(), LV_STATE_DEFAULT);

#ifdef ENABLE_DEMO_SCREENS
  lv_obj_t *tu = lv_tabview_add_tab(tv, "SynthSide");
  lv_obj_t *tz = lv_tabview_add_tab(tv, "SynthBack");
  lv_obj_t *tw = lv_tabview_add_tab(tv, "Radar");
  // lv_obj_t *ty = lv_tabview_add_tab(tv, "HSI");
  lv_obj_t *t0 = lv_tabview_add_tab(tv, "Map");
#endif
#ifdef RB_ENABLE_GPS
  lv_obj_t *t1 = lv_tabview_add_tab(tv, "Speed");
#endif
  lv_obj_t *t2 = lv_tabview_add_tab(tv, "Attitude");
  lv_obj_t *t3 = lv_tabview_add_tab(tv, "Altimeter");
  lv_obj_t *t3b = lv_tabview_add_tab(tv, "Altimeter");
  lv_obj_t *t4 = lv_tabview_add_tab(tv, "TurnSlip");
#ifdef RB_ENABLE_GPS
  lv_obj_t *t5 = lv_tabview_add_tab(tv, "Track");
#endif
  lv_obj_t *t6 = lv_tabview_add_tab(tv, "Variometer");
  lv_obj_t *t7 = lv_tabview_add_tab(tv, "GMeter");
  lv_obj_t *t8 = lv_tabview_add_tab(tv, "Clock");
  lv_obj_t *t9 = lv_tabview_add_tab(tv, "Setup");
  // lv_obj_t *t10 = lv_tabview_add_tab(tv, "Demo");
#ifdef ENABLE_DEMO_SCREENS
  Onboard_create_Base(tu, &RoundSynthViewSide);
  lv_obj_add_event_cb(tu, speedBgClicked, LV_EVENT_CLICKED, NULL);
  Onboard_create_Base(tz, &RoundSynthViewAttitude);
  lv_obj_add_event_cb(tz, speedBgClicked, LV_EVENT_CLICKED, NULL);
  Onboard_create_Base(tw, &Radar);
  lv_obj_add_event_cb(tw, speedBgClicked, LV_EVENT_CLICKED, NULL);
  // Onboard_create_Base(ty, &RoundHSI);
  Onboard_create_Base(t0, &RoundMapWithControlledSpaces);
  lv_obj_add_event_cb(t0, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif
#ifdef RB_ENABLE_GPS
  Onboard_create_Speed(t1);
#endif
  Onboard_create_Attitude(t2);
  Onboard_create_Altimeter(t3);
  Onboard_create_AltimeterDigital(t3b);
  Onboard_create_TurnSlip(t4);
#ifdef RB_ENABLE_GPS
  Onboard_create_Track(t5);
#endif
  Onboard_create_Variometer(t6);
  Onboard_create_GMeter(t7);
  Onboard_create_Clock(t8);
  Onboard_create_Setup(t9);
  // Onboard_create(t10);

  // BMP280
  uint8_t bmp280BufferReset[1] = {0xB6};
  I2C_Write(0x76, 0xE0, &bmp280BufferReset[0], 1);

  //
  auto_step_timer = lv_timer_create(rb_increase_lvgl_tick, 100, NULL);

  //
  Loading_slider = lv_slider_create(lv_scr_act());

  lv_obj_set_size(Loading_slider, 240, 35);
  lv_obj_set_style_radius(Loading_slider, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
  lv_obj_set_style_bg_opa(Loading_slider, LV_OPA_TRANSP, LV_PART_KNOB);
  // lv_obj_set_style_pad_all(Backlight_slider, 0, LV_PART_KNOB);
  lv_obj_set_style_bg_color(Loading_slider, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
  lv_obj_set_style_bg_color(Loading_slider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
  lv_obj_set_style_outline_width(Loading_slider, 2, LV_PART_INDICATOR);
  lv_obj_set_style_outline_color(Loading_slider, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
  lv_slider_set_range(Loading_slider, 0, 100);
  lv_slider_set_value(Loading_slider, workflow, LV_ANIM_ON);
  lv_obj_align(Loading_slider, LV_ALIGN_CENTER, 0, 0);

  if (StartupPage != 0)
  {
    lv_tabview_set_act(tv, StartupPage, LV_ANIM_OFF);
  }
}

void nmea_RMC_UpdatedValueFor(uint8_t csvCounter, int32_t finalNumber, uint8_t decimalCounter)
{
  float conversionValue = finalNumber;
  for (uint8_t c = 0; c < decimalCounter; c++)
  {
    conversionValue = conversionValue / 10.0;
  }

  /*
  0=-12.00
  0=-97.00
  0=-938.00
  0=-9346.00
  0=-93431.00
  0=-934291.00
  1=-9342910.00
  1=-93429088.00
  1=-934290880.00
  1=-752974528.00
  1=1060189632.00
  1=2011961600.00
  1=-135522112.00
  1=-6673091.00
  2=19168436.00
  3=19885676.00
  3=-15891594.00
  3=12882754.00
  3=-21477.37
  3=-21477.37
  3=-21477.37
  3=21472.30
  3=-2.53
  3=-2.53
  4=-25.32
  5=176.26
  5=44.63
  5=16.77
  5=167.68
  5=-41.18
  5=1.77
  5=1.77
  5=0.05
  5=0.01
  5=-0.00
  6=0.00
  7=0.00
  7=-0.00
  7=0.00
  7=0.00
  9=0.00
  9=-0.00
  9=-0.00
  9=-0.00
  9=0.00
  9=0.00
  12=0.00
  12=-0.00
  12=0.00
  12=-0.00
  0=-12.00
  0=-97.00
  0=-938.00
  0=-9346.00
  0=-93431.00
  0=-934291.00
  1=-9342910.00
  1=-93429088.00
  1=-934290880.00
  1=-752974528.00
  1=1060189632.00
  1=2011961600.00
  1=-135522112.00
  1=-6673090.50
  2=19168438.00
  3=19885698.00
  3=-15891394.00
  3=12884754.00
  3=-1477.37
  3=-1477.37
  3=-1477.37
  3=-1477.37
  3=-1477.37
  3=-188.88
  4=-170.80
  5=9.99
  5=99.92
  5=140.17
  5=113.26
  5=-155.91
  5=15.88
  5=-1.30
  5=-0.01
  5=-0.01
  5=0.00
  6=-0.00
  7=0.00
  7=-0.00
  7=0.00
  7=0.00
  9=-0.00
  9=0.00

  */

  switch (csvCounter)
  {
  case 0: // $GPRMC
    /* code */
    break;
  case 1: // UTC hhmmss.ss
    NMEA_DATA.tim.thousand = finalNumber % 100;
    NMEA_DATA.tim.second = (finalNumber / 100) % 60;
    NMEA_DATA.tim.minute = (finalNumber / 10000) % 60;
    NMEA_DATA.tim.hour = (finalNumber / 1000000);
    /* code */
    break;
  case 2: // A
    /* code */
    break;
  case 3: // 4311.11936
    /* code */
    break;
  case 4: // N
    /* code */
    break;
  case 5: // 01208.18660
    /* code */
    break;
  case 6: // E
    /* code */
    break;
  case 7: // Speed KT

    NMEA_DATA.speed = conversionValue * 1.852;
    /* code */
    break;
  case 8: // Track
    /* code */

    NMEA_DATA.cog = conversionValue;
    break;
  case 9: // Date ddmmyy
    /* code */
    break;
  }
}

bool nmea_RMC_mini_parser(const uint8_t *sentence, uint16_t length)
{

  int8_t csvCounter = 0;
  int8_t decimalCounter = 0;
  int8_t decimalEnabled = 0;

  int32_t finalNumber = 0;

  for (uint8_t x = 0; x < length; x++)
  {
    if (sentence[x] == 0)
      break;
    if (sentence[x] == ',')
    {
      // printf("%d=%ld/%d\n", csvCounter, finalNumber,decimalCounter);
      nmea_RMC_UpdatedValueFor(csvCounter, finalNumber, decimalCounter);
      decimalCounter = 0;
      csvCounter++;
      decimalEnabled = 0;
      finalNumber = 0;
      continue;
    }
    if (sentence[x] == '\n')
      break;
    if (sentence[x] == '\r')
      break;
    if (sentence[x] == '.')
    {
      decimalEnabled = 1;
      continue;
    }
    if (sentence[x] == '-')
    {
      finalNumber = finalNumber - 1;
    }
    else
    {
      decimalCounter += decimalEnabled;
      finalNumber = 10 * finalNumber + (sentence[x] - '0');
    }
  }

  return true;
}
#ifdef RB_ENABLE_GPS
void uart_fetch_data()
{
  if (GpsSpeed0ForDisable == 0)
    return;
  uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
  const int rxBytes = uart_read_bytes(UART_N, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
  if (rxBytes > 0)
  {
    data[rxBytes] = 0;
    /*
    printf("UART[%d]=", rxBytes);
    data[rxBytes] = 0;
    for (int x = 0; x < rxBytes; x++)
    {
      printf("%c", data[x]);
    }
    printf("\n");
    */
    for (int x = 0; x < rxBytes - 23; x++)
    {
      //$GPRMC,,,,,,,,,,,,A*79
      if (data[x] == '$')
      {
        if (strncmp((char *)(data + x + 1), "GPRMC,", 5) == 0)
        {
          lv_label_set_text(SettingStatus4UART, "DATA RECEIVED");
          if (nmea_RMC_mini_parser(data + x, rxBytes - x))
          {
            break;
          }
          break; // Fast as possibile we go back and do not parse anymore
        }
      }
    }
  }
  free(data);
}
#endif
void nvsRestoreGMeter()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    printf("Reading GMeter from NVS ... ");
    int8_t gmeter_max = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i8(my_handle, "gmeter_max", &gmeter_max);
    switch (err)
    {
    case ESP_OK:
      printf("gmeter_max = %d\n", gmeter_max);
      float f = gmeter_max;
      GFactorMax = f / 10.0;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      printf("The value is not initialized yet!\n");
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
    int8_t gmeter_min = 10; // value will default to 0, if not set yet in NVS
    err = nvs_get_i8(my_handle, "gmeter_min", &gmeter_min);
    switch (err)
    {
    case ESP_OK:
      printf("gmeter_min = %d\n", gmeter_min);
      float f = gmeter_min;
      GFactorMin = f / 10.0;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      printf("The value is not initialized yet!\n");
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    int32_t pcal = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(my_handle, "pcal", &pcal);
    switch (err)
    {
    case ESP_OK:
      printf("pcal = %ld\n", pcal);
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      printf("The value is not initialized yet!\n");
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    bmp280override = pcal;

    uint8_t defaultPageOrDemo = 0xff; // value will default to 0, if not set yet in NVS
    err = nvs_get_u8(my_handle, "default", &defaultPageOrDemo);
    switch (err)
    {
    case ESP_OK:
      printf("defaultPageOrDemo = %u\n", defaultPageOrDemo);
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      printf("The value is not initialized yet!\n");
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    if (defaultPageOrDemo == 0xff)
    {
      DeviceIsDemoMode = 1;
    }
    else
    {
      DeviceIsDemoMode = 0;
      StartupPage = defaultPageOrDemo;
    }

    err = nvs_get_i32(my_handle, "uart", &GpsSpeed0ForDisable);
    switch (err)
    {
    case ESP_OK:
      printf("GpsSpeed0ForDisable = %ld\n", GpsSpeed0ForDisable);
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      printf("The value is not initialized yet!\n");
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    GFactorDirty = 0;

    nvs_get_u8(my_handle, "isKmh", &isKmh);
    nvs_get_u16(my_handle, "degreeStart", &degreeStart);
    nvs_get_u16(my_handle, "degreeEnd", &degreeEnd);
    nvs_get_u16(my_handle, "speedKtStart", &speedKtStart);
    nvs_get_u16(my_handle, "speedKtEnd", &speedKtEnd);
    nvs_get_u16(my_handle, "speedWhite", &speedWhite);
    nvs_get_u16(my_handle, "speedGreen", &speedGreen);
    nvs_get_u16(my_handle, "speedYellow", &speedYellow);
    nvs_get_u16(my_handle, "speedRed", &speedRed);

    nvs_close(my_handle);
  }
}

void nvsStoreSpeedArc()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u8(my_handle, "isKmh", isKmh);
    printf((err != ESP_OK) ? "Failed to update isKmh!\n" : "Done\n");
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u16(my_handle, "degreeStart", degreeStart);
    printf((err != ESP_OK) ? "Failed to update degreeStart!\n" : "Done\n");
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u16(my_handle, "degreeEnd", degreeEnd);
    printf((err != ESP_OK) ? "Failed to update degreeEnd!\n" : "Done\n");
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u16(my_handle, "speedKtStart", speedKtStart);
    printf((err != ESP_OK) ? "Failed to update speedKtStart!\n" : "Done\n");
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u16(my_handle, "speedKtEnd", speedKtEnd);
    printf((err != ESP_OK) ? "Failed to update speedKtEnd!\n" : "Done\n");
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u16(my_handle, "speedWhite", speedWhite);
    printf((err != ESP_OK) ? "Failed to update speedWhite!\n" : "Done\n");
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u16(my_handle, "speedGreen", speedGreen);
    printf((err != ESP_OK) ? "Failed to update speedGreen!\n" : "Done\n");
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u16(my_handle, "speedYellow", speedYellow);
    printf((err != ESP_OK) ? "Failed to update speedYellow!\n" : "Done\n");
    printf("Writing Speed to NVS ... ");
    err = nvs_set_u16(my_handle, "speedRed", speedRed);
    printf((err != ESP_OK) ? "Failed to update speedRed!\n" : "Done\n");
    nvs_close(my_handle);
  }
}

void nvsStoreUARTBaudrate()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    printf("Writing UART from NVS ... ");
    err = nvs_set_i32(my_handle, "uart", GpsSpeed0ForDisable);
    printf((err != ESP_OK) ? "Failed to update GpsSpeed0ForDisable!\n" : "Done\n");
    nvs_close(my_handle);
  }
}

void nvsStorePCal()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    printf("Writing pcal from NVS ... ");
    err = nvs_set_i32(my_handle, "pcal", bmp280override);
    printf((err != ESP_OK) ? "Failed to update bmp280override!\n" : "Done\n");
    nvs_close(my_handle);
  }
}

void nvsStoreGMeter()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    printf("Writing GMeter from NVS ... ");
    err = nvs_set_i8(my_handle, "gmeter_max", 10.0 * GFactorMax);
    printf((err != ESP_OK) ? "Failed to update GMeterMax!\n" : "Done\n");
    err = nvs_set_i8(my_handle, "gmeter_min", 10.0 * GFactorMin);
    printf((err != ESP_OK) ? "Failed to update GMeterMin!\n" : "Done\n");

    nvs_close(my_handle);
  }
}

void nvsStoreDefaultScreenOrDemo()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    printf("Writing Default Screen from NVS ... ");
    uint8_t value = 0xff;

    if (DeviceIsDemoMode == 1)
    {
    }
    else
    {
      lv_tabview_t *tabview = (lv_tabview_t *)tv;
      value = (uint8_t)tabview->tab_cur;
    }

    err = nvs_set_u8(my_handle, "default", value);
    printf((err != ESP_OK) ? "Failed to update default!\n" : "Done\n");
    nvs_close(my_handle);
  }
}

void update_Attitude_lvgl_tick(lv_timer_t *t)
{
  float moveY = -4.0 * AttitudePitch;
  if (moveY > 120)
    moveY = 120;
  else
  {
    if (moveY < -100)
    {
      moveY = -100;
    }
  }

  if (moveY == lastAttitudePitch && lastAttitudeRoll == -AttitudeRoll)
  {
    return;
  }
  lastAttitudePitch = moveY;
  lastAttitudeRoll = -AttitudeRoll;
  // 1.1.1 Rotation of Aircraft symbol
  lv_img_set_angle(Screen_Attitude_Pitch, 1800 + lastAttitudeRoll * 10.0);
  lv_obj_set_pos(Screen_Attitude_Pitch, 0, -lastAttitudePitch + att_aircraft.header.h / 2);

  for (int a = 0; a < 4; a++)
  {
    if (Screen_Attitude_Rounds[a] == NULL)
      continue;
    int32_t c20 = lv_trigo_sin(lastAttitudeRoll + a * 90) * 19 / 3276;
    int32_t s20 = lv_trigo_sin(lastAttitudeRoll + 90 + a * 90) * 19 / 3276;
    int32_t c21 = lv_trigo_sin(lastAttitudeRoll + a * 90) * 12 / 3276;
    int32_t s21 = lv_trigo_sin(lastAttitudeRoll + 90 + a * 90) * 12 / 3276;
    switch (a)
    {
    case 0:
      lv_obj_set_pos(Screen_Attitude_Rounds[a], -c20, s20);
      lv_img_set_angle(Screen_Attitude_Rounds[a], 1800 + lastAttitudeRoll * 10.0);
      break;
    case 3: // TL
      lv_obj_set_pos(Screen_Attitude_Rounds[a], -c20 - s21, s20 - c21);
      lv_img_set_angle(Screen_Attitude_Rounds[a], 1800 + lastAttitudeRoll * 10.0);
      break;
    case 1: // TR
      lv_obj_set_pos(Screen_Attitude_Rounds[a], -c20 + s21, s20 + c21);
      lv_img_set_angle(Screen_Attitude_Rounds[a], 1800 + lastAttitudeRoll * 10.0);
      break;
    }
  }
}

void update_TurnSlip_lvgl_tick(lv_timer_t *t)
{

  float fay = AccelFiltered.y;
  if (fay < -1.1)
    fay = -1.1;
  else if (fay > 1.1)
    fay = 1.1;

  float range_X = 100;
  float range_Y = 16;
  lv_obj_set_pos(Screen_TurnSlip_Obj_Ball, -range_X * fay, 88 - abs((int)(range_Y * fay)));
  lv_img_set_angle(Screen_TurnSlip_Obj_Turn, -10.0 * GyroFiltered.x);
}

void update_TurnSlip2_lvgl_tick(lv_timer_t *t)
{
  int16_t BallAngle = 1800 - (Accel.x * 1800);
  if (Accel.y < 0)
    BallAngle = BallAngle * -1;
  lv_img_set_angle(Ball, BallAngle / 2);
}
int32_t bmp280Pressure = 0;
int32_t bmp280Temperature = 0;
int32_t Altimeter = 0;
int32_t Variometer = 0;
int32_t pressureCompensation2(int32_t adc_T, int32_t adc_P)
{
  int32_t var1t, var2t;
  int32_t t_fine;
  adc_T >>= 4;

  var1t = ((((adc_T >> 3) - ((int32_t)bmp280Calibration[0] << 1))) *
           ((int32_t)bmp280Calibration[1])) >>
          11;

  var2t = (((((adc_T >> 4) - ((int32_t)bmp280Calibration[0])) *
             ((adc_T >> 4) - ((int32_t)bmp280Calibration[0]))) >>
            12) *
           ((int32_t)bmp280Calibration[2])) >>
          14;

  t_fine = var1t + var2t;

  float T = (t_fine * 5 + 128) >> 8;

  bmp280Temperature = T / 100;

  int64_t var1p, var2p, p;

  adc_P >>= 4;

  var1p = ((int64_t)t_fine) - 128000;
  var2p = var1p * var1p * (int64_t)bmp280Calibration[8];
  var2p = var2p + ((var1p * (int64_t)bmp280Calibration[7]) << 17);
  var2p = var2p + (((int64_t)bmp280Calibration[6]) << 35);
  var1p = ((var1p * var1p * (int64_t)bmp280Calibration[5]) >> 8) +
          ((var1p * (int64_t)bmp280Calibration[4]) << 12);
  var1p =
      (((((int64_t)1) << 47) + var1p)) * ((int64_t)bmp280Calibration[3]) >> 33;

  if (var1p == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2p) * 3125) / var1p;
  var1p = (((int64_t)bmp280Calibration[11]) * (p >> 13) * (p >> 13)) >> 25;
  var2p = (((int64_t)bmp280Calibration[10]) * p) >> 19;

  p = ((p + var1p + var2p) >> 8) + (((int64_t)bmp280Calibration[9]) << 4);
  bmp280Pressure = bmp280override + (float)(p / 256.0);
  return bmp280Pressure;
}

int32_t temperatureCompensation(int32_t adc_T)
{
  BMP280_S32_t var1, var2, T;
  BMP280_S32_t dig_T1 = bmp280Calibration[0];
  BMP280_S32_t dig_T2 = bmp280Calibration[1];
  BMP280_S32_t dig_T3 = bmp280Calibration[2];

  var1 = ((((adc_T >> 3) - (dig_T1 << 1))) * ((BMP280_S32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((BMP280_S32_t)dig_T1)) * ((adc_T >> 4) - ((BMP280_S32_t)dig_T1))) >> 12) *
          ((BMP280_S32_t)dig_T3)) >>
         14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;

  bmp280Temperature = T;
  return T;
}

uint32_t pressureCompensation(int32_t adc_P)
{

  BMP280_S32_t dig_P1 = bmp280Calibration[3];
  BMP280_S32_t dig_P2 = bmp280Calibration[4];
  BMP280_S32_t dig_P3 = bmp280Calibration[5];
  BMP280_S32_t dig_P4 = bmp280Calibration[6];
  BMP280_S32_t dig_P5 = bmp280Calibration[7];
  BMP280_S32_t dig_P6 = bmp280Calibration[8];
  BMP280_S32_t dig_P7 = bmp280Calibration[9];
  BMP280_S32_t dig_P8 = bmp280Calibration[10];
  BMP280_S32_t dig_P9 = bmp280Calibration[11];

  BMP280_S64_t var1, var2, p;
  var1 = ((BMP280_S64_t)t_fine) - 128000;
  var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
  var2 = var2 + ((var1 * (BMP280_S64_t)dig_P5) << 17);
  var2 = var2 + (((BMP280_S64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3) >> 8) + ((var1 * (BMP280_S64_t)dig_P2) << 12);
  var1 = (((((BMP280_S64_t)1) << 47) + var1)) * ((BMP280_S64_t)dig_P1) >> 33;
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((BMP280_S64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7) << 4);
  bmp280Pressure = bmp280override + p / 256;
  return bmp280Pressure;
}

void example1_BMP280_lvgl_tick(lv_timer_t *t)
{
  uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
  I2C_Read(0x76, 0xF7, &buf[0], 6);

  int32_t adc_t = ((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
  int32_t adc_p = ((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
  temperatureCompensation(adc_t);
  pressureCompensation(adc_p);

  // Altimeter is *100 feet
  int32_t AltimeterNew = (((QNH * 100) - bmp280Pressure) * 27.5);
  // Polling is 1Hz = 0.01 Feet/Second
  int32_t VariometerInstant = AltimeterNew - Altimeter;
  // 1.0.9 Variometer is filterd
  Variometer = (Variometer + VariometerInstant) / 2;
  Altimeter = AltimeterNew;
// 1.1.1
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("BMP280 ");
  for (int i = 0; i < 6; i++)
  {
    printf("%X ", buf[i]);
  }

  for (int i = 0; i < 12; i++)
  {
    printf("%ld ", bmp280Calibration[i]);
  }

  printf(" Pressure: %lu Temperature: %ld Altimeter: %ld Variometer: %ld Read T: %ld Read P: %ld V: %ld Gy:%.1f %.1f %.1f\n",
         bmp280Pressure,
         bmp280Temperature,
         Altimeter,
         Variometer,
         adc_t, adc_p, ((Variometer * 6) * 36 / 400),
         Gyro.x,
         Gyro.y,
         Gyro.z);
#endif
}
void Get_BMP280(void)
{
  example1_BMP280_lvgl_tick(NULL);
}
void update_Track_lvgl_tick(lv_timer_t *t)
{
  static int32_t lastCOG = 0;
  int32_t COG = -NMEA_DATA.cog * 10;
  if (COG != lastCOG)
  {
    printf("NMEA: Track %ld-->%ld\n", lastCOG, COG);
    lastCOG = COG;
    lv_img_set_angle(Screen_Gyro_Gear, COG);

    char buf[15];
    snprintf(buf, sizeof(buf), "%.0f°", NMEA_DATA.cog);
    lv_label_set_text(Screen_Track_TrackText, buf);
  }
}

void update_Speed_lvgl_tick(lv_timer_t *t)
{
  static int lastSpeed = -1;
  int speed = NMEA_DATA.speed * 10;
  if (speed != lastSpeed)
  {
    // printf("NMEA: Speed %d-->%d m/s Angle: %ld\n", lastSpeed, speed, (int32_t)(speed));
    lastSpeed = speed;
    // 1.1.2 You can configure the ARCs and Min-Max speed displayed
    float rangeDegree = degreeEnd - degreeStart;
    float rangeKt = speedKtEnd - speedKtStart;
    float ratioDK = (rangeDegree) / (rangeKt);

    float isKt = 1.852;
    if (isKmh == 1)
    {
      isKt = 1.0;
    }
    int speedCapped = (speed / isKt) - (speedKtStart * 10);
    /*
    if (speedCapped < 0)
    {
      // We decided to stay "before" nearby the first indicator
      speedCapped = -rangeKt/5;
    }
      */
    int32_t speedAngle = degreeStart * 10 + (ratioDK * speedCapped);
    if(speedAngle<0){
      speedAngle=0;
    }

    lv_img_set_angle(Screen_Speed_SpeedTick, speedAngle + 900);

    char buf[15];
    snprintf(buf, sizeof(buf), "%.0f", NMEA_DATA.speed / isKt); // KT
    lv_label_set_text(Screen_Speed_SpeedText, buf);
  }
}

void update_Variometer_lvgl_tick(lv_timer_t *t)
{
  // 1.0.9 Variometer is *100 and 1Hz
  // Variometer => 0.01 Feet/Second => *60/100 => Feet/min
  // UI 90° => 1000ft/min => 180° = 2000ft/min => 360° = 4000ft/min
  // LVGL is *10 Degree;
  int32_t VDegree = ((Variometer * 6.0 / 10.0) * 36.0 / 40);
  lv_img_set_angle(Screen_Variometer_Cents, VDegree);
}

void update_Altimeter_lvgl_tick(lv_timer_t *t)
{
  // Altimeter Analog Screen
  // 1.0.9 Altimter is *100 Feet
  int32_t AltimeterInFeet = Altimeter / 100.0;
  float milesDegree = (36.0 * AltimeterInFeet) / 100.0;
  float centsDegree = (360.0 * ((AltimeterInFeet) % 1000)) / 100.0;
  lv_img_set_angle(Screen_Altitude_Miles, milesDegree + 900);
  lv_img_set_angle(Screen_Altitude_Cents, centsDegree + 900);
}
#ifdef RB_ENABLE_GPS
void uartApplyRates()
{
  lv_label_set_text(SettingStatus4UART, "NO DATA RECEIVED");

  if (GpsSpeed0ForDisable > 0)
  {
    // 1.0.9 Enable UART for NMEA GPS Input
    const uart_config_t uart_config = {
        .baud_rate = GpsSpeed0ForDisable, // TODO: Delay the setup up to the LVGL started
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // Setup Baud Rate
    uart_param_config(1, &uart_config);
  }
}
#endif 
void bmp280Setup()
{
  uint8_t bmp280Control[1] = {0x57};
  uint8_t bmp280Settings[1] = {0x9C};
  I2C_Write(0x76, 0xF4, &bmp280Control[0], 1);
  I2C_Write(0x76, 0xF5, &bmp280Settings[0], 1);
}

void readCalibration()
{
  uint8_t buf[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  I2C_Read(0x76, 0x88, &buf[0], 24);
  /* Endianess. */
  printf("Calibration: ");
  for (int i = 0; i < 12; i++)
  {
    printf("%X%X ", buf[2 * i + 1], buf[2 * i]);
    int16_t signedPass = (((buf[2 * i + 1]) << 8) | buf[2 * i]);
    uint16_t usignedPass = (((buf[2 * i + 1]) << 8) | buf[2 * i]);
    if (i == 0 || i == 3)
      bmp280Calibration[i] = usignedPass;
    else
      bmp280Calibration[i] = signedPass;
  }
  printf("\n");
}

void rb_increase_lvgl_tick(lv_timer_t *t)
{
  static int stepDown = 50;
  if (workflow > 100)
  {
    // Completed

    stepDown = stepDown - 1;
    if (stepDown < 1)
    {
      stepDown = 50;

      if (GFactorDirty != 0 && DeviceIsDemoMode == 0)
      {
        GFactorDirty = 0;
        nvsStoreGMeter();
      }

      if (DeviceIsDemoMode > 0 && mbox1 == NULL)
      {

        lv_tabview_t *tabview = (lv_tabview_t *)tv;
        uint16_t cur = tabview->tab_cur;
        cur++;
        // Skip setup page
        if (cur >= tabview->tab_cnt - 2)
        {
          cur = 0;
        }
        lv_tabview_set_act(tv, cur, LV_ANIM_ON);
      }
    }
  }
  else
  {
    switch (workflow)
    {
    case 3:
      bmp280Setup();
      break;
    case 7:
      readCalibration();
      LCD_Backlight = 1;
      LVGL_Backlight_adjustment(LCD_Backlight);
      break;
    case 8:
      GyroBiasAcquire[0] = GyroFiltered;
    case 50:
      GyroBiasAcquire[1] = GyroFiltered;
    case 80:
      GyroBiasAcquire[2] = GyroFiltered;
      GyroBias.x = -(GyroBiasAcquire[0].x + GyroBiasAcquire[1].x + GyroBiasAcquire[2].x) / 3.0;
      GyroBias.y = -(GyroBiasAcquire[0].y + GyroBiasAcquire[1].y + GyroBiasAcquire[2].y) / 3.0;
      GyroBias.z = -(GyroBiasAcquire[0].z + GyroBiasAcquire[1].z + GyroBiasAcquire[2].z) / 3.0;
    case 90:
      stopwatch = datetime;
      break;
    case 91:
    #ifdef RB_ENABLE_GPS
      uartApplyRates();
      switch (GpsSpeed0ForDisable)
      {
      case 0:
        lv_dropdown_set_selected(uartDropDown, 0);
        break;
      case 4800:
        lv_dropdown_set_selected(uartDropDown, 1);
        break;
      case 9600:
        lv_dropdown_set_selected(uartDropDown, 2);
        break;
      case 19200:
        lv_dropdown_set_selected(uartDropDown, 3);
        break;
      case 38400:
        lv_dropdown_set_selected(uartDropDown, 4);
        break;
      case 115200:
        lv_dropdown_set_selected(uartDropDown, 5);
        break;
      }
    #else
    GpsSpeed0ForDisable = 0;
    #endif
      break;
    case 100:

      datetimeTimer1 = datetime;
      datetimeTimer2 = datetime;
      datetimeTimer3 = datetime;

      lv_obj_add_flag(Loading_slider, LV_OBJ_FLAG_HIDDEN);

      LCD_Backlight = 100;
      LVGL_Backlight_adjustment(LCD_Backlight);
      break;
    default:
      break;
    }

    workflow++;
    lv_slider_set_value(Loading_slider, workflow, LV_ANIM_ON);
  }

  static uint16_t lastTab = 0;

  if (lastTab != ((lv_tabview_t *)tv)->tab_cur)
  {
    printf("Tab Changed to: %d\n", (((lv_tabview_t *)tv)->tab_cur));
    lastTab = ((lv_tabview_t *)tv)->tab_cur;
    stepDown = 50;
    return;
  }

  switch (((lv_tabview_t *)tv)->tab_cur)
  {
#ifdef RB_ENABLE_GPS
  case RB02_TAB_SPD:
    uart_fetch_data();
    update_Speed_lvgl_tick(t);
    break;
  case RB02_TAB_TRK:
    uart_fetch_data();
    update_Track_lvgl_tick(t);
    break;
#endif
  case RB02_TAB_DEV:
    break;
  case RB02_TAB_VAR:
    update_Variometer_lvgl_tick(t);
    break;
  case RB02_TAB_ALT:
    update_Altimeter_lvgl_tick(t);
    break;
  case RB02_TAB_ALD:
    update_AltimeterDigital_lvgl_tick(t);
    break;
  case RB02_TAB_ATT:
    update_Attitude_lvgl_tick(t);
    break;
  case RB02_TAB_TRN:
    update_TurnSlip_lvgl_tick(t);
    break;
  case RB02_TAB_GMT:
    update_GMeter_lvgl_tick(t);
    break;
  case RB02_TAB_CLK:
    update_Clock_lvgl_tick(t);
    break;

  case RB02_TAB_SET:
  {
    char buf[4 + 4 + 4 + 4 + 4 + 4];
    sprintf(buf, "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f",
            Accel.x,
            Accel.y,
            Accel.z,
            Gyro.x,
            Gyro.y,
            Gyro.z);
    lv_label_set_text(SettingStatus0, buf);
    sprintf(buf, "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f",
            AccelFiltered.x,
            AccelFiltered.y,
            AccelFiltered.z,
            GyroFiltered.x + GyroBias.x,
            GyroFiltered.y + GyroBias.y,
            GyroFiltered.z + GyroBias.z);
    lv_label_set_text(SettingStatus1, buf);

    sprintf(buf, "%2.1f %2.1f %2.1f %2.1f %2.1f",
            AttitudeRoll,
            AttitudePitch,
            GyroBias.x,
            GyroBias.y,
            GyroBias.z);
    lv_label_set_text(SettingStatus2, buf);

    sprintf(buf, "%.1f°C %.2fhPa",
            bmp280Temperature / 100.0,
            bmp280Pressure / 100.0);
    lv_label_set_text(SettingStatus3, buf);
#ifdef RB_ENABLE_GPS
    uart_fetch_data();
#endif
  }
  break;
  }
}
/*
44
44
20
26
27
21
22
23
44
44
26
27
24
25
33

*/
static void mbox1_event_cb(lv_event_t *e)
{

  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *msgbox = lv_event_get_current_target(e);

  if (code == LV_EVENT_VALUE_CHANGED)
  {
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    if (txt)
    {
      if (strcmp("RESET", txt) == 0)
      {
        GFactorMax = 0;
        GFactorMin = 1.0;
        GFactorDirty = 1;
      }
      lv_msgbox_close(msgbox);
    }
  }

  if (code == LV_EVENT_DELETE)
  {
    printf("Popup closed\n");
    mbox1 = NULL;
  }
}

static void mbox1_cage_event_cb(lv_event_t *e)
{

  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *msgbox = lv_event_get_current_target(e);

  if (code == LV_EVENT_VALUE_CHANGED)
  {
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    if (txt)
    {
      if (strcmp("CAGE", txt) == 0)
      {
        GyroBias.x = -GyroFiltered.x;
        GyroBias.y = -GyroFiltered.y;
        GyroBias.z = -GyroFiltered.z;
      }
      lv_msgbox_close(msgbox);
    }
  }

  if (code == LV_EVENT_DELETE)
  {
    printf("Popup closed\n");
    mbox1 = NULL;
  }
}

void lv_att_reset_msgbox(void)
{
  if (mbox1 != NULL)
    return;
  static const char *btns[] = {"CAGE", "CANCEL", ""};

  mbox1 = lv_msgbox_create(NULL, "Attitude", "Would you like to cage sensors?", btns, true);
  lv_obj_set_style_text_font(mbox1, &lv_font_montserrat_16, 0);
  lv_obj_add_event_cb(mbox1, mbox1_cage_event_cb, LV_EVENT_ALL, mbox1);
  lv_obj_center(mbox1);
}

void lv_gmeter_reset_msgbox(void)
{
  if (mbox1 != NULL)
    return;
  static const char *btns[] = {"RESET", "CANCEL", ""};

  mbox1 = lv_msgbox_create(NULL, "G-Meter", "Would you like to reset G max-neg?", btns, true);
  lv_obj_set_style_text_font(mbox1, &lv_font_montserrat_16, 0);
  lv_obj_add_event_cb(mbox1, mbox1_event_cb, LV_EVENT_ALL, mbox1);
  lv_obj_center(mbox1);
}

static void actionInTab(touchLocation location)
{
  switch (((lv_tabview_t *)tv)->tab_cur)
  {
  case RB02_TAB_ALT:
  case RB02_TAB_ALD:
    switch (location)
    {
    case RB02_TOUCH_N:
      QNH++;
      break;
    case RB02_TOUCH_S:
      QNH--;
      break;
    default:
      break;
    }

    char buf[25];
    snprintf(buf, sizeof(buf), "%03u", QNH);
    lv_label_set_text(Screen_Altitude_QNH, buf);
        // 1.1.2 added mmHg conversion
    snprintf(buf, sizeof(buf), "QNH: %u %.02f", QNH, ((float)QNH)/33.8639);
    lv_label_set_text(Screen_Altitude_QNH2, buf);
    snprintf(buf, sizeof(buf), "%+ld", Variometer);
    lv_label_set_text(Screen_Altitude_Variometer2, buf);

    example1_BMP280_lvgl_tick(NULL);

    break;

  case RB02_TAB_CLK:
    switch (location)
    {
    case RB02_TOUCH_SE:
    case RB02_TOUCH_SW:
    case RB02_TOUCH_N:
    {
      selectedTimer++;
      if (selectedTimer > 2)
      {
        selectedTimer = 0;
      }
      char buf[8];
      sprintf(buf, "TIMER %u", selectedTimer + 1);
      lv_label_set_text(TimerLabelTop, buf);
      update_Clock_lvgl_tick(NULL);

      switch (selectedTimer)
      {
      case 0:
        sprintf(buf, "TIMER %u", 2);
        lv_label_set_text(TimerLabelSW, buf);
        sprintf(buf, "TIMER %u", 3);
        lv_label_set_text(TimerLabelSE, buf);
        break;
      case 1:
        sprintf(buf, "TIMER %u", 1);
        lv_label_set_text(TimerLabelSW, buf);
        sprintf(buf, "TIMER %u", 3);
        lv_label_set_text(TimerLabelSE, buf);
        break;
      case 2:
        sprintf(buf, "TIMER %u", 1);
        lv_label_set_text(TimerLabelSW, buf);
        sprintf(buf, "TIMER %u", 2);
        lv_label_set_text(TimerLabelSE, buf);
        break;
      }
    }
    break;
    case RB02_TOUCH_S:
      break;
    case RB02_TOUCH_CENTER:

      switch (selectedTimer)
      {
      case 0:
        datetimeTimer1 = datetime;
        break;
      case 1:
        datetimeTimer2 = datetime;
        break;
      case 2:
        datetimeTimer3 = datetime;
        break;
      }
      update_Clock_lvgl_tick(NULL);
      break;
    default:
      break;
    }
    break;

  case RB02_TAB_GMT:
    switch (location)
    {
    case RB02_TOUCH_N:
    case RB02_TOUCH_S:
    case RB02_TOUCH_CENTER:
      lv_gmeter_reset_msgbox();
      break;
    default:
      break;
    }
    break;

  case RB02_TAB_ATT:
    switch (location)
    {
    case RB02_TOUCH_CENTER:
      lv_att_reset_msgbox();
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

static void speedBgClicked(lv_event_t *event)
{
  if (mbox1 != NULL)
  {
    return;
  }

  bool changedTab = false;
  touchLocation location = getTouchLocation(TouchPadLastX, TouchPadLastY);
  printf("Clicked at %ux%u => %u\n", TouchPadLastX, TouchPadLastY, location);
  lv_tabview_t *tabview = (lv_tabview_t *)tv;
  uint16_t cur = tabview->tab_cur;

  switch (location)
  {
  case RB02_TOUCH_W:
    if (cur > 0)
    {
      cur--;
      changedTab = true;
    }
    break;
  case RB02_TOUCH_E:
    cur++;
    changedTab = true;
    break;
  default:
    actionInTab(location);
    break;
  }

  // Skip setup page
  if (cur >= tabview->tab_cnt - 1)
  {
    cur = tabview->tab_cnt - 2;
  }
  lv_tabview_set_act(tv, cur, LV_ANIM_ON);
  if (DeviceIsDemoMode == 0 && changedTab)
  {
    nvsStoreDefaultScreenOrDemo();
  }
}

static lv_obj_t *Onboard_create_Base(lv_obj_t *parent, const lv_img_dsc_t *backgroundImageName)
{

  lv_obj_t *backgroundImage = lv_img_create(parent);
  lv_img_set_src(backgroundImage, backgroundImageName);
  lv_obj_set_size(backgroundImage, backgroundImageName->header.w, backgroundImageName->header.h);
  lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);

  return backgroundImage;
}

static void ChangeAttitudeBalanceAlphaChanged(lv_event_t *e)
{
  uint8_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
  printf("Changed Filtering ratio from: %.2f to: %d\n", AttitudeBalanceAlpha, FilterMoltiplierInt);
  float f = FilterMoltiplierInt;
  AttitudeBalanceAlpha = f / 100.0;
  char buf[23 + 8];
  sprintf(buf, "Attitude compensation: %.0f", AttitudeBalanceAlpha * 100);
  lv_label_set_text(SettingAttitudeCompensation, buf);
}

static void DeviceIsDemoModeChanged(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    DeviceIsDemoMode = 1;
    nvsStoreDefaultScreenOrDemo();
  }
  else
  {
    DeviceIsDemoMode = 0;
  }
}

static void AltimeterOverrideChanged(lv_event_t *e)
{
  int32_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
  printf("Changed BMP280 Override from: %ld to: %ld\n", bmp280override, FilterMoltiplierInt);
  bmp280override = FilterMoltiplierInt - 500;

  example1_BMP280_lvgl_tick(NULL);

  char buf[50];
  sprintf(buf, "Altimeter QNH: %d mmHg %ld feet (%ld)", QNH, Altimeter, bmp280override);
  lv_label_set_text(bmp280overrideLabel, buf);

  // Store
  nvsStorePCal();
}

static void SpeedDegreeStartChanged(lv_event_t *e)
{
  uint16_t new_Start = 15*lv_slider_get_value(t_speedStart);
  uint16_t new_End = 15*lv_slider_get_value(t_speedEnd);
  uint16_t new_StartSpeed = 10*lv_slider_get_value(t_speedStartSpeed);
  uint16_t new_EndSpeed = 10*lv_slider_get_value(t_speedEndSpeed);
  uint16_t new_White = 15*lv_slider_get_value(t_speedWhite);
  uint16_t new_Green = 15*lv_slider_get_value(t_speedGreen);
  uint16_t new_Yellow = 15*lv_slider_get_value(t_speedYellow);
  uint16_t new_Red = 15*lv_slider_get_value(t_speedRed);

  degreeStart = new_Start;
  degreeEnd = new_End;
  speedKtStart = new_StartSpeed;
  speedKtEnd = new_EndSpeed;
  speedWhite = new_White;
  speedGreen = new_Green;
  speedYellow = new_Yellow;
  speedRed = new_Red;

  char buf[60];
  snprintf(buf, 60, "%u->%u %u->%u W:%u G:%u Y:%u R:%u",
           degreeStart,
           degreeEnd,
           speedKtStart,
           speedKtEnd,
           speedWhite,
           speedGreen,
           speedYellow,
           speedRed);
  lv_label_set_text(t_speedSummary, buf);
}
static void DisableFilteringChanged(lv_event_t *e)
{
  uint8_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
  printf("Changed Filtering ratio from: %.1f to: %d\n", FilterMoltiplier, FilterMoltiplierInt);
  FilterMoltiplier = FilterMoltiplierInt;

  char buf[16 + 8];
  sprintf(buf, "Sensor filtering: %.0f", FilterMoltiplier);
  lv_label_set_text(SettingLabelFilter, buf);
}
static void Backlight_adjustment_event_Changed(lv_event_t *e)
{
  uint8_t Backlight = lv_slider_get_value(lv_event_get_target(e));
  printf("Changed Backlight ratio from: %d to: %d\n", LCD_Backlight, Backlight);
  LVGL_Backlight_adjustment(Backlight);
  LCD_Backlight = Backlight;
}
static void GMeterMaxChanged(lv_event_t *e)
{
  uint8_t value = lv_slider_get_value(lv_event_get_target(e));
  printf("Changed GMeter from: %.1f to: %d\n", GMeterScale, value);
  GMeterScale = value;
}

// 1.1.2
static void event_handler_kmh_menu(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    if (strcmp(buf, "Imperial Units") == 0)
      isKmh = 0;
    if (strcmp(buf, "Metric Units") == 0)
      isKmh = 1;

    nvsStoreSpeedArc();
  }
}
#ifdef RB_ENABLE_GPS
static void event_handler_gps_menu(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    GpsSpeed0ForDisable = 0;
    if (strcmp(buf, "4800") == 0)
      GpsSpeed0ForDisable = 4800;
    if (strcmp(buf, "9600") == 0)
      GpsSpeed0ForDisable = 9600;
    if (strcmp(buf, "19200") == 0)
      GpsSpeed0ForDisable = 19200;
    if (strcmp(buf, "38400") == 0)
      GpsSpeed0ForDisable = 38400;
    if (strcmp(buf, "115200") == 0)
      GpsSpeed0ForDisable = 115200;

    uartApplyRates();

    nvsStoreUARTBaudrate();
  }
}
#endif
static void Onboard_create_Setup(lv_obj_t *parent)
{
  int lineY = -200;

  // Version
  if (true)
  {
    lv_obj_t *VersionLabel = lv_label_create(parent);
    lv_obj_set_size(VersionLabel, 200, 48);
    lv_obj_align(VersionLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(VersionLabel, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(VersionLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(VersionLabel, "RB-02");
    lv_obj_add_style(VersionLabel, &style_title, LV_STATE_DEFAULT);
    lineY += 44;
  }
  if (true)
  {
    lv_obj_t *VersionLabel = lv_label_create(parent);
    lv_obj_set_size(VersionLabel, 200, 20);
    lv_obj_align(VersionLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(VersionLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(VersionLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(VersionLabel, "Version " RB_VERSION);
    lv_obj_add_style(VersionLabel, &style_title, LV_STATE_DEFAULT);
    lineY += 40;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, 6);
    lv_slider_set_value(DisableFiltering, FilterMoltiplier, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, DisableFilteringChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    char buf[16 + 8];
    sprintf(buf, "Sensor filtering: %.0f", FilterMoltiplier);
    lv_label_set_text(DisableFilteringLabel, buf);
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    SettingLabelFilter = DisableFilteringLabel;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, 100);
    lv_slider_set_value(DisableFiltering, AttitudeBalanceAlpha * 100, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, ChangeAttitudeBalanceAlphaChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    char buf[23 + 8];
    sprintf(buf, "Attitude compensation: %.0f", AttitudeBalanceAlpha * 100);
    lv_label_set_text(DisableFilteringLabel, buf);
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    SettingAttitudeCompensation = DisableFilteringLabel;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, 1000);
    lv_slider_set_value(DisableFiltering, 500, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, AltimeterOverrideChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    bmp280overrideLabel = lv_label_create(parent);
    lv_obj_set_size(bmp280overrideLabel, 400, 20);
    lv_obj_align(bmp280overrideLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(bmp280overrideLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(bmp280overrideLabel, &lv_font_montserrat_16, 0);
    char buf[50];
    sprintf(buf, "Altimeter QNH: %d mmHg %ld feet (%ld)", QNH, Altimeter, bmp280override);
    lv_label_set_text(bmp280overrideLabel, buf);
    lv_obj_add_style(bmp280overrideLabel, &style_title, LV_STATE_DEFAULT);

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 2, 12);
    lv_slider_set_value(DisableFiltering, 3, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, GMeterMaxChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "GMeter Scale 2G 12G");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, Backlight_MAX);
    lv_slider_set_value(DisableFiltering, LCD_Backlight, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, Backlight_adjustment_event_Changed, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "Backlight brightness");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 150, 16);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, -75, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "Demo mode");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);
    lv_obj_t *sw = lv_switch_create(parent);
    if (DeviceIsDemoMode != 0)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, lineY);
    lv_obj_add_event_cb(sw, DeviceIsDemoModeChanged, LV_EVENT_VALUE_CHANGED, sw);
    lineY += 40;
  }
  if (true)
  {
    SettingStatus0 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus0, 400, 20);
    lv_obj_align(SettingStatus0, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus0, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus0, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus0, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }
  if (true)
  {
    SettingStatus1 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus1, 400, 20);
    lv_obj_align(SettingStatus1, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus1, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus1, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }
  if (true)
  {
    SettingStatus2 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus2, 400, 20);
    lv_obj_align(SettingStatus2, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus2, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus2, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus2, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }

  if (true)
  {
    SettingStatus3 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus3, 400, 20);
    lv_obj_align(SettingStatus3, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus3, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus3, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus3, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }
#ifdef RB_ENABLE_GPS
  if (true)
  {
    lineY += 20;
    /*Create a normal drop down list*/
    uartDropDown = lv_dropdown_create(parent);
    lv_dropdown_set_options(uartDropDown, "GPS Disabled\n"
                                          "4800\n"
                                          "9600\n"
                                          "19200\n"
                                          "38400\n"
                                          "115200");

    lv_obj_align(uartDropDown, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_add_event_cb(uartDropDown, event_handler_gps_menu, LV_EVENT_ALL, NULL);

    lineY += 30;
  }

  if (true)
  {
    SettingStatus4UART = lv_label_create(parent);
    lv_obj_set_size(SettingStatus4UART, 400, 20);
    lv_obj_align(SettingStatus4UART, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus4UART, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus4UART, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus4UART, &style_title, LV_STATE_DEFAULT);
    lv_label_set_text(SettingStatus4UART, "NO DATA RECEIVED");
    lineY += 30;
  }
#else
  if (true)
  {
    SettingStatus4UART = lv_label_create(parent);
    lv_obj_set_size(SettingStatus4UART, 400, 20);
    lv_obj_align(SettingStatus4UART, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus4UART, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus4UART, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus4UART, &style_title, LV_STATE_DEFAULT);
    lv_label_set_text(SettingStatus4UART, "GPS NOT ENABLED");
    lineY += 30;
  }
#endif
    if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 400, 20);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    lv_label_set_text(label, "SPEED SETUP");

    lineY += 25;
  }


  if (true)
  {
    lineY += 10;
    /*Create a normal drop down list*/
    kmhDropDown = lv_dropdown_create(parent);
    lv_dropdown_set_options(kmhDropDown, "Imperial Units\n"
                                         "Metric Units");

    lv_obj_align(kmhDropDown, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_add_event_cb(kmhDropDown, event_handler_kmh_menu, LV_EVENT_ALL, NULL);
    lv_dropdown_set_selected(kmhDropDown, isKmh);
    lineY += 30;
  }


  if (true)
  {
    t_speedSummary = lv_label_create(parent);
    lv_obj_set_size(t_speedSummary, 400, 20);
    lv_obj_align(t_speedSummary, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(t_speedSummary, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(t_speedSummary, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(t_speedSummary, &style_title, LV_STATE_DEFAULT);
    char buf[60];
    snprintf(buf, 60, "%u->%u %u->%u W:%u G:%u Y:%u R:%u",
             degreeStart,
             degreeEnd,
             speedKtStart,
             speedKtEnd,
             speedWhite,
             speedGreen,
             speedYellow,
             speedRed);
    lv_label_set_text(t_speedSummary, buf);

    lineY += 25;
  }

  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 23);
    lv_slider_set_value(progressObject, degreeStart/15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Speed start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedStart = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 1, 24);
    lv_slider_set_value(progressObject, degreeEnd/15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Speed end degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedEnd = progressObject;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 10);
    lv_slider_set_value(progressObject, speedKtStart/10, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Speed start");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedStartSpeed = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 18, 40);
    lv_slider_set_value(progressObject, speedKtEnd/10, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Speed end");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedEndSpeed = progressObject;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 24);
    lv_slider_set_value(progressObject, speedWhite/15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "White arc start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedWhite = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 24);
    lv_slider_set_value(progressObject, speedGreen/15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 0, 360);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Green arc start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedGreen = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 24);
    lv_slider_set_value(progressObject, speedYellow/15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Yellow arc start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedYellow = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 24);
    lv_slider_set_value(progressObject, speedRed/15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Red arc start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedRed = progressObject;

    lineY += 70;
  }

  lineY += 10;
  if (true)
  {
    lv_obj_t *VersionLabel = lv_label_create(parent);
    lv_obj_set_size(VersionLabel, 200, 20);
    lv_obj_align(VersionLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(VersionLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(VersionLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(VersionLabel, "XIAPROJECTS SRL");
    lv_obj_add_style(VersionLabel, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }
}

static void Onboard_create_Clock(lv_obj_t *parent)
{

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "RB 02");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -160);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "TIMER 1");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerLabelTop = label;
  }

  // CreateDigitArray(parent, &DigitFont100x25, DIGIT_BIG_DIGITXXX,0,0);
  CreateSingleDigit(parent, &DigitFont100x25, SegmentsA[0], -(DigitFont100x25.header.w + 8 + DigitFont100x25.header.h) - 24, 0);
  CreateSingleDigit(parent, &DigitFont100x25, SegmentsA[1], -24, 0);
  CreateSingleDigit(parent, &DigitFont70x20, SegmentsA[2], (DigitFont100x25.header.w / 2 + DigitFont100x25.header.h + 16), 15);
  CreateSingleDigit(parent, &DigitFont70x20, SegmentsA[3], (DigitFont100x25.header.w / 2 + DigitFont100x25.header.h + 24) + (DigitFont70x20.header.w + DigitFont70x20.header.h), 15);

  // 1.1.1 Displays always 3 timers
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, -95, 165);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "TIMER 1");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerSW = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, -95, 150);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "TIMER 2");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerLabelSW = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 95, 165);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "--:--");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerSE = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 95, 150);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "TIMER 3");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerLabelSE = label;
  }

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}

static void Onboard_create_AltimeterDigital(lv_obj_t *parent)
{
  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "RB 02");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  // 1.1.2 Added feet label
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 185, 100);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "feet");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -160);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "ALTIMETER");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 440, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -160 + 48);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    // 1.1.2 added mmHg conversion
    lv_label_set_text(label, "QNH: 1013 - 29.91");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    Screen_Altitude_QNH2 = label;
  }

  const int numDigit = 5;
  for (int c = 0; c < numDigit; c++)
  {
    int k = (DigitFont70x20.header.w + DigitFont70x20.header.h + 8) * (c - 2) - 8; // we estimate that you will not fly more than 19999
    CreateSingleDigit(parent, &DigitFont70x20, SegmentsAltDigit[c], k, 0);
  }

  // 1.0.6 Adding the digital variometer
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 106);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "---");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    Screen_Altitude_Variometer2 = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 150);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "feet/min");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 170);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "---.--");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    Screen_Altitude_Pressure = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 214);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "hPa");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}

static void CreateSingleDigit(lv_obj_t *parent, lv_img_dsc_t *font, lv_obj_t **segments, int dx, int dy)
{
  int16_t k = font->header.w / 2;
  int16_t j = font->header.w;
  int x = 0;

  for (int y = 0; y < DIGIT_BIG_SEGMENTS; y++)
  {
    segments[y] = Onboard_create_Base(parent, font);
    int16_t a = 0;
    int16_t b = 0;
    switch (y)
    {
    case 0:
      a = 0;
      b = 0;
      break;
    case 1:
      a = 0;
      b = -j;
      break;
    case 2:
      a = 0;
      b = j;
      break;
    case 3:
      a = -k;
      b = j / 2;
      lv_img_set_angle(segments[y], 900);
      break;
    case 4:
      lv_img_set_angle(segments[y], 900);
      a = k;
      b = j / 2;
      break;
    case 5:
      lv_img_set_angle(segments[y], 900);
      a = k;
      b = -j / 2;
      break;
    case 6:
      lv_img_set_angle(segments[y], 900);
      a = -k;
      b = -j / 2;
      break;
    }

    lv_obj_align(segments[y], LV_ALIGN_CENTER, dx + a + x * (j + font->header.h + 4) - 1 / 2 * (j + font->header.h + 4), dy + b);
  }
}
static void CreateDigitArray(lv_obj_t *parent, lv_img_dsc_t *font, int howMany, int dx, int dy)
{

  int16_t k = DigitFont100x25.header.w / 2;
  int16_t j = DigitFont100x25.header.w;

  for (int x = 0; x < howMany; x++)
  {
    for (int y = 0; y < DIGIT_BIG_SEGMENTS; y++)
    {
      SegmentsA[x][y] = Onboard_create_Base(parent, &DigitFont100x25);
      int16_t a = 0;
      int16_t b = 0;
      switch (y)
      {
      case 0:
        a = 0;
        b = 0;
        break;
      case 1:
        a = 0;
        b = -j;
        break;
      case 2:
        a = 0;
        b = j;
        break;
      case 3:
        a = -k;
        b = j / 2;
        lv_img_set_angle(SegmentsA[x][y], 900);
        break;
      case 4:
        lv_img_set_angle(SegmentsA[x][y], 900);
        a = k;
        b = j / 2;
        break;
      case 5:
        lv_img_set_angle(SegmentsA[x][y], 900);
        a = k;
        b = -j / 2;
        break;
      case 6:
        lv_img_set_angle(SegmentsA[x][y], 900);
        a = -k;
        b = -j / 2;
        break;
      }

      lv_obj_align(SegmentsA[x][y], LV_ALIGN_CENTER, a + x * (j + DigitFont100x25.header.h + 4) - howMany / 2 * (j + DigitFont100x25.header.h + 4), b);
    }
  }
}

void draw_arch(lv_obj_t *parent, const lv_img_dsc_t *t, uint16_t degreeStartSlide, uint16_t degreeEndSlide)
{
  for (uint16_t degree = degreeStartSlide; degree <= degreeEndSlide; degree += 15)
  {

    int16_t sin = lv_trigo_sin(degree - 90) / 327;
    int16_t cos = lv_trigo_sin(degree) / 327;

    uint32_t y = (sin) * 32 / 100;

    lv_obj_t *slice = lv_img_create(parent);
    lv_img_set_src(slice, t);
    lv_obj_set_size(slice, t->header.w, t->header.h);
    lv_img_set_pivot(slice, 32, 240);
    lv_obj_align(slice, LV_ALIGN_CENTER, 0, -120 - 96 - 16);
    lv_img_set_angle(slice, degree * 10);
  }
}
#ifdef RB_ENABLE_GPS
static void Onboard_create_Speed(lv_obj_t *parent)
{
  // Degree shall be multiples of 7.5
  draw_arch(parent, &arcWhite, speedWhite, speedGreen - 15);
  draw_arch(parent, &arcGreen, speedGreen, speedYellow - 15);
  draw_arch(parent, &arcYellow, speedYellow, speedRed - 15);
  draw_arch(parent, &arcRed, speedRed, speedRed);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "RB 02");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 60);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    if (isKmh == 1)
    {
      lv_label_set_text(label, "GPS SPEED KMH");
    }
    else
    {
      lv_label_set_text(label, "GPS SPEED KT");
    }
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  uint8_t radius = (460 - 98) / 2;
  uint16_t speedKt = 0;

  uint8_t numberOfItems = (degreeEnd - degreeStart) / 30;

  uint16_t speedKtIncrement = (speedKtEnd - speedKtStart) / numberOfItems;
  uint16_t degreeIncrement = (degreeEnd - degreeStart) / numberOfItems;

  for (uint16_t degree = degreeStart; degree <= degreeEnd; degree += degreeIncrement)
  {

    int16_t sin = lv_trigo_sin(degree - 90) / 327;
    int16_t cos = lv_trigo_sin(degree) / 327;

    uint32_t x = (cos)*radius / 100;
    uint32_t y = (sin)*radius / 100;

    // printf("Speed: %d %d %ld %ld\n", degree, speedKt, x, y);

    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 94, 48);
    lv_obj_align(label, LV_ALIGN_CENTER, x + 4, y);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);

    char buf[8];
    snprintf(buf, sizeof(buf), "%u", speedKtStart + (speedKt));
    speedKt += speedKtIncrement;
    lv_label_set_text(label, buf);
  }

  Screen_Speed_SpeedTick = lv_img_create(parent);
  lv_img_set_src(Screen_Speed_SpeedTick, &fi_needle);
  lv_obj_set_size(Screen_Speed_SpeedTick, fi_needle.header.w, fi_needle.header.h);
  lv_obj_align(Screen_Speed_SpeedTick, LV_ALIGN_CENTER, 0, 0);

  Screen_Speed_SpeedText = lv_label_create(parent);
  lv_obj_set_size(Screen_Speed_SpeedText, 128, 48);
  lv_obj_align(Screen_Speed_SpeedText, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(Screen_Speed_SpeedText, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(Screen_Speed_SpeedText, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(Screen_Speed_SpeedText, lv_color_white(), 0);
  lv_obj_set_style_bg_color(Screen_Speed_SpeedText, lv_color_black(), 0);
  lv_obj_set_style_radius(Screen_Speed_SpeedText, LV_RADIUS_CIRCLE, 0);
  char buf[4];
  snprintf(buf, sizeof(buf), "%d", 0);
  lv_label_set_text(Screen_Speed_SpeedText, buf);

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}
#endif
static void Onboard_create_Attitude(lv_obj_t *parent)
{

  lv_obj_set_style_bg_color(parent, lv_color_hex(0x4f3822), LV_PART_ITEMS | LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_CHECKED);
  Onboard_create_Base(parent, &att_middle_big);

  Screen_Attitude_Pitch = Onboard_create_Base(parent, &att_aircraft);
  // 1.1.1 Aircraft Symbol rotation displacement
  lv_img_set_pivot(Screen_Attitude_Pitch, att_aircraft.header.w / 2, 0);
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  lv_obj_t *att_img_top = Onboard_create_Base(parent, &att_circle_top_T);
  lv_obj_set_pos(att_img_top, 0, -240 + att_circle_top_T.header.h / 2);
  // lv_obj_t *att_img_bottom=Onboard_create_Base(parent, &att_circle_top_B);
  // lv_obj_set_pos(att_img_bottom, 0,240-att_circle_top_B.header.h/2);
  lv_obj_t *att_img_tl = Onboard_create_Base(parent, &att_circle_top_TL);
  lv_obj_set_pos(att_img_tl, -240 + att_circle_top_TL.header.w / 2, 0);
  lv_obj_t *att_img_tr = Onboard_create_Base(parent, &att_circle_top_TR);
  lv_obj_set_pos(att_img_tr, 240 - att_circle_top_TR.header.w / 2, 0);

  Screen_Attitude_Rounds[0] = att_img_top;
  // Screen_Attitude_Rounds[2]=att_img_bottom; // Fixed image does not need any rotation
  Screen_Attitude_Rounds[2] = NULL;
  Screen_Attitude_Rounds[3] = att_img_tl;
  Screen_Attitude_Rounds[1] = att_img_tr;

  Screen_Attitude_RollIndicator = Onboard_create_Base(parent, &att_tri);
  lv_obj_set_pos(Screen_Attitude_RollIndicator, 0, -240 + att_tri.header.h / 2);


    // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -150);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "RB 02");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
}

static void Onboard_create_Altimeter(lv_obj_t *parent)
{
  Onboard_create_Base(parent, &RoundAltimeter);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -130);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "RB 02");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  Screen_Altitude_QNH = lv_label_create(parent);
  lv_obj_set_size(Screen_Altitude_QNH, 128, 48);
  lv_obj_align(Screen_Altitude_QNH, LV_ALIGN_CENTER, 172, 0);
  lv_obj_set_style_text_font(Screen_Altitude_QNH, &lv_font_montserrat_48, 0);
  char buf[6];
  snprintf(buf, sizeof(buf), "%03u", QNH);
  lv_label_set_text(Screen_Altitude_QNH, buf);

  Screen_Altitude_Miles = lv_img_create(parent);
  lv_img_set_src(Screen_Altitude_Miles, &fi_needle_small);
  lv_obj_set_size(Screen_Altitude_Miles, fi_needle_small.header.w, fi_needle_small.header.h);
  lv_obj_align(Screen_Altitude_Miles, LV_ALIGN_CENTER, 0, 0);

  Screen_Altitude_Cents = lv_img_create(parent);
  lv_img_set_src(Screen_Altitude_Cents, &fi_needle);
  lv_obj_set_size(Screen_Altitude_Cents, fi_needle.header.w, fi_needle.header.h);
  lv_obj_align(Screen_Altitude_Cents, LV_ALIGN_CENTER, 0, 0);
}

static void Onboard_create_TurnSlip(lv_obj_t *parent)
{
  Onboard_create_Base(parent, &turn_coordinator);
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  lv_obj_t *_screenBall = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(_screenBall, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(_screenBall, Screen_TurnSlip_Obj_Ball_Size, Screen_TurnSlip_Obj_Ball_Size);

  lv_obj_align(_screenBall, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(_screenBall, lv_color_black(), 0);
  lv_obj_set_style_radius(_screenBall, LV_RADIUS_CIRCLE, 0);
  static lv_style_t style;
  lv_style_init(&style);
  Screen_TurnSlip_Obj_Turn = lv_img_create(parent);

  lv_img_set_src(Screen_TurnSlip_Obj_Turn, &fi_tc_airplane);
  lv_obj_set_size(Screen_TurnSlip_Obj_Turn, fi_tc_airplane.header.w, fi_tc_airplane.header.h);
  lv_style_set_img_recolor(&style, lv_color_white());

  lv_obj_align(Screen_TurnSlip_Obj_Turn, LV_ALIGN_CENTER, 0, 0);
  Screen_TurnSlip_Obj_Ball = _screenBall;
}
#ifdef RB_ENABLE_GPS
static void Onboard_create_Track(lv_obj_t *parent)
{
  Screen_Gyro_Gear = Onboard_create_Base(parent, &RoundGyro);
  Onboard_create_Base(parent, &RoundGyroHeading);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "RB 02");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -26);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "GPS TRACK");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  Screen_Track_TrackText = lv_label_create(parent);
  lv_obj_set_size(Screen_Track_TrackText, 128, 48);
  lv_obj_align(Screen_Track_TrackText, LV_ALIGN_CENTER, 8, -8); // Displacement to center the "°"
  lv_obj_set_style_text_align(Screen_Track_TrackText, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(Screen_Track_TrackText, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(Screen_Track_TrackText, lv_color_white(), 0);
  lv_obj_set_style_bg_color(Screen_Track_TrackText, lv_color_black(), 0);
  lv_obj_set_style_radius(Screen_Track_TrackText, LV_RADIUS_CIRCLE, 0);
  char buf[4];
  snprintf(buf, sizeof(buf), "%d°", 0);
  lv_label_set_text(Screen_Track_TrackText, buf);
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}
#endif
static void Onboard_create_Variometer(lv_obj_t *parent)
{

  Onboard_create_Base(parent, &RoundVariometer);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "RB 02");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  Screen_Variometer_Cents = lv_img_create(parent);
  lv_img_set_src(Screen_Variometer_Cents, &fi_needle);
  lv_obj_set_size(Screen_Variometer_Cents, fi_needle.header.w, fi_needle.header.h);
  lv_obj_align(Screen_Variometer_Cents, LV_ALIGN_CENTER, 0, 0);
}

static void Onboard_create_GMeter(lv_obj_t *parent)
{
  Onboard_create_Base(parent, &GMeter);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "RB 02");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  Screen_GMeter_BallMax = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(Screen_GMeter_BallMax, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(Screen_GMeter_BallMax, 32, 32);
  lv_obj_align(Screen_GMeter_BallMax, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(Screen_GMeter_BallMax, lv_color_make(0xff, 0, 0), 0);
  lv_obj_set_style_radius(Screen_GMeter_BallMax, LV_RADIUS_CIRCLE, 0);

  Screen_GMeter_Ball = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(Screen_GMeter_Ball, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(Screen_GMeter_Ball, 32, 32);
  lv_obj_align(Screen_GMeter_Ball, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(Screen_GMeter_Ball, lv_color_white(), 0);
  lv_obj_set_style_radius(Screen_GMeter_Ball, LV_RADIUS_CIRCLE, 0);

  GMeterLabel = lv_label_create(parent);
  lv_label_set_text(GMeterLabel, "--");
  lv_obj_set_size(GMeterLabel, 120, 48);
  lv_obj_align(GMeterLabel, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(GMeterLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(GMeterLabel, &lv_font_montserrat_48, 0);
  lv_obj_add_style(GMeterLabel, &style_title, LV_STATE_DEFAULT);

  GMeterLabelMax = lv_label_create(parent);
  lv_label_set_text(GMeterLabelMax, "+1.0 -0.0");
  lv_obj_set_size(GMeterLabelMax, 240, 48);
  lv_obj_align(GMeterLabelMax, LV_ALIGN_CENTER, 0, 200);
  lv_obj_set_style_text_align(GMeterLabelMax, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(GMeterLabelMax, &lv_font_montserrat_48, 0);
  lv_obj_add_style(GMeterLabelMax, &style_title, LV_STATE_DEFAULT);
}
void turnOnOffDigitsSymbols(lv_obj_t **segments, uint8_t symbol)
{
  switch (symbol)
  {
  case 0: // -
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  };
}

void turnOnOffDigits(lv_obj_t **segments, uint8_t number)
{
  switch (number)
  {
  case 0:
    lv_obj_add_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 1:
    lv_obj_add_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 2:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 3:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 4:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 5:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 6:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 7:
    lv_obj_add_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 8:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 9:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  default:
    lv_obj_add_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  }
}

void update_AltimeterDigital_lvgl_tick(lv_timer_t *t)
{
  // 1.0.9 Altimter is *100 Feet
  int32_t AltimeterAbsolute = Altimeter / 100.0;
  if (Altimeter < 0)
  {
    AltimeterAbsolute = -Altimeter;
    // TODO set minus
  }

  int k = 1;
  bool firstZero = false;
  for (int c = 4; c >= 0; c--)
  {
    int v = (AltimeterAbsolute / k);
    if (v > 0)
    {
      turnOnOffDigits(SegmentsAltDigit[c], v % 10);
      firstZero = false;
    }
    else
    {
      if (firstZero == false && Altimeter < 0)
      {
        turnOnOffDigitsSymbols(SegmentsAltDigit[c], 0);
      }
      else
      {
        turnOnOffDigits(SegmentsAltDigit[c], -1);
      }
      firstZero = true;
    }
    k = k * 10;
  }
  /*
  turnOnOffDigits(SegmentsAltDigit[0], (AltimeterAbsolute / 10000)%10);
  turnOnOffDigits(SegmentsAltDigit[1], (AltimeterAbsolute / 1000)%10);
  turnOnOffDigits(SegmentsAltDigit[2], (AltimeterAbsolute / 100)%10);
  turnOnOffDigits(SegmentsAltDigit[3], (AltimeterAbsolute / 10)%10);
  turnOnOffDigits(SegmentsAltDigit[4], AltimeterAbsolute % 10);
  */

  char buf[10];
  // 1.0.9 Variometer is *100 and 1Hz
  // Variometer => 0.01 Feet/Second => *60/100 => Feet/min
  int32_t VariometerFeetMin = Variometer * 6.0 / 10.0;
  snprintf(buf, sizeof(buf), "%+ld", Variometer);
  lv_label_set_text(Screen_Altitude_Variometer2, buf);

  snprintf(buf, sizeof(buf), "%.02f", bmp280Pressure / 100.0);
  lv_label_set_text(Screen_Altitude_Pressure, buf);
}

uint32_t timerDiffByIndex(int st)
{

  datetime_t datetimeTimer = {0};

  switch (st)
  {
  case 0:
    datetimeTimer = datetimeTimer1;
    break;
  case 1:
    datetimeTimer = datetimeTimer2;
    break;
  case 2:
    datetimeTimer = datetimeTimer3;
    break;

  default:
    break;
  }

  uint32_t epochA = datetime.second + 60 * datetime.minute + 60 * 60 * datetime.hour;
  uint32_t epochB = datetimeTimer.second + 60 * datetimeTimer.minute + 60 * 60 * datetimeTimer.hour;
  uint32_t diff = epochA - epochB;

  return diff;
}

void update_Clock_lvgl_tick(lv_timer_t *t)
{
  uint32_t diff = timerDiffByIndex(selectedTimer);

  turnOnOffDigits(SegmentsA[0], ((diff / 60) / 10) % 10);
  turnOnOffDigits(SegmentsA[1], ((diff / 60)) % 10);
  turnOnOffDigits(SegmentsA[2], ((diff % 60) / 10) % 10);
  turnOnOffDigits(SegmentsA[3], (diff % 60) % 10);
  char buf[20];
  uint32_t diffSW = 0;
  uint32_t diffSE = 0;
  switch (selectedTimer)
  {
  case 0:
    diffSW = timerDiffByIndex(1);
    sprintf(buf, "%02lu:%02lu", (diffSW / 60), (diffSW % 60));
    lv_label_set_text(TimerSW, buf);
    diffSE = timerDiffByIndex(2);
    sprintf(buf, "%02lu:%02lu", (diffSE / 60), (diffSE % 60));
    lv_label_set_text(TimerSE, buf);
    break;
  case 1:
    diffSW = timerDiffByIndex(0);
    sprintf(buf, "%02lu:%02lu", (diffSW / 60), (diffSW % 60));
    lv_label_set_text(TimerSW, buf);
    diffSE = timerDiffByIndex(2);
    sprintf(buf, "%02lu:%02lu", (diffSE / 60), (diffSE % 60));
    lv_label_set_text(TimerSE, buf);
    break;
  case 2:
    diffSW = timerDiffByIndex(0);
    sprintf(buf, "%02lu:%02lu", (diffSW / 60), (diffSW % 60));
    lv_label_set_text(TimerSW, buf);
    diffSE = timerDiffByIndex(1);
    sprintf(buf, "%02lu:%02lu", (diffSE / 60), (diffSE % 60));
    lv_label_set_text(TimerSE, buf);
    break;
  }
}

void update_GMeter_lvgl_tick(lv_timer_t *t)
{
  lv_obj_set_pos(Screen_GMeter_Ball, -180 * AccelFiltered.y / GMeterScale, 180 * AccelFiltered.x / GMeterScale);
  char buf[12]; // +99.9 -99.9
  snprintf(buf, sizeof(buf), "%.1f", GFactor);
  lv_label_set_text(GMeterLabel, buf);

  // TODO flag MAX Dirty
  lv_obj_set_pos(Screen_GMeter_BallMax, -60 * AccelFilteredMax.y, 60 * AccelFilteredMax.x);
  snprintf(buf, sizeof(buf), "+%.1f %.1f", GFactorMax, GFactorMin);
  lv_label_set_text(GMeterLabelMax, buf);

  float BeepFactor = GMeterScale / 3.0 * 2.0;
  if (GFactor > BeepFactor)
  {
    Buzzer_On();
  }
  else
  {
    Buzzer_Off();
  }
}
