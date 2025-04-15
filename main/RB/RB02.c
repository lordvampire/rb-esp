/**
 * RB02.c
 * Implementation of RoastBeef PN 02 The basic six pack version
 *
 * Features:
 * - Attitude indicator
 * - Turn & Slip
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
// Images pre-loaded
#include "RoundSynthViewAttitude.c"
#include "RoundAirSpeed.c"
#include "RoundAltitude.c"
#include "RoundTurnCoordinator.c"
#include "RoundGyro.c"
#include "RoundVariometer.c"
#include "turn_coordinator.c"
#include "fi_tc_airplane.c"
#include "horizon_ball.c"
#include "attitude_foreground.c"
#include "attitude_background.c"
#include "fi_needle.c"
#include "fi_needle_small.c"
#include "GMeter.c"
#include "AttitudeBackground2.c"
#include "Sky.c"
#include "AttitudePlain.c"
#include "AttitudeMiddle.c"
#include "RoundSynthViewSide.c"
#include "Radar.c"
#include "RoundMapWithControlledSpaces.c"
#include "RoundHSI.c"
#include "att_circle_top_TL.c"
#include "att_circle_top_TR.c"
#include "att_circle_top_T.c"
#include "att_circle_middle.c"
#include "att_middle_big.c"
#include "att_aircraft.c"
#include "att_tri.c"
#include "DigitFont100x25.c"
#include "DigitFont70x20.c"

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

// Prototype declaration
static lv_obj_t *Onboard_create_Base(lv_obj_t *parent, const lv_img_dsc_t *backgroundImage);
static void Onboard_create_Speed(lv_obj_t *parent);
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
void rb_increase_lvgl_tick(lv_timer_t *t);
void update_GMeter_lvgl_tick(lv_timer_t *t);
void update_Clock_lvgl_tick(lv_timer_t *t);
void update_AltimeterDigital_lvgl_tick(lv_timer_t *t);
void update_Altimeter_lvgl_tick(lv_timer_t *t);

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
  RB02_TAB_SYS,
  RB02_TAB_SYN,
  RB02_TAB_RDR,
  // RB02_TAB_HSI,
  RB02_TAB_MAP,
  RB02_TAB_SPD,
  RB02_TAB_ATT,
  RB02_TAB_ALT,
  RB02_TAB_ALD,
  RB02_TAB_TRN,
  RB02_TAB_TRK,
  RB02_TAB_VAR,
  RB02_TAB_GMT,
  RB02_TAB_CLK,
  RB02_TAB_SET,
  RB02_TAB_DEV
} tabs;

lv_obj_t *SettingStatus0 = NULL;
lv_obj_t *SettingStatus1 = NULL;
lv_obj_t *SettingStatus2 = NULL;
lv_obj_t *SettingStatus3 = NULL;
lv_obj_t *SettingLabelFilter = NULL;

lv_obj_t *SettingAttitudeCompensation = NULL;

lv_obj_t *Ball = NULL;
lv_obj_t *Screen_Attitude_Pitch = NULL;
lv_obj_t *Screen_Attitude_RollIndicator = NULL;
void *Screen_Attitude_Rounds[4];
uint16_t QNH = 1013;
lv_obj_t *Screen_Altitude_Miles = NULL;
lv_obj_t *Screen_Altitude_Cents = NULL;
lv_obj_t *Screen_Variometer_Cents = NULL;
lv_obj_t *Screen_Altitude_QNH = NULL;
lv_obj_t *Screen_Altitude_QNH2 = NULL;
lv_obj_t *Screen_Altitude_Variometer2 = NULL;
int lastAttitudePitch = 0;
int lastAttitudeRoll = 0;
datetime_t stopwatch = {0};
uint8_t DeviceIsDemoMode = 0;
tabs StartupPage = RB02_TAB_SYS;
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
lv_obj_t *Screen_GMeter_Ball = NULL;
lv_obj_t *Screen_GMeter_BallMax = NULL;
lv_obj_t *GMeterLabel = NULL;
lv_obj_t *GMeterLabelMax = NULL;
float GMeterScale = 3.0;
IMUdata GyroBiasAcquire[3];
lv_obj_t *mbox1 = NULL;

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

  lv_obj_t *tu = lv_tabview_add_tab(tv, "SynthSide");
  lv_obj_t *tz = lv_tabview_add_tab(tv, "SynthBack");
  lv_obj_t *tw = lv_tabview_add_tab(tv, "Radar");
  // lv_obj_t *ty = lv_tabview_add_tab(tv, "HSI");
  lv_obj_t *t0 = lv_tabview_add_tab(tv, "Map");
  lv_obj_t *t1 = lv_tabview_add_tab(tv, "Speed");
  lv_obj_t *t2 = lv_tabview_add_tab(tv, "Attitude");
  lv_obj_t *t3 = lv_tabview_add_tab(tv, "Altimeter");
  lv_obj_t *t3b = lv_tabview_add_tab(tv, "Altimeter");
  lv_obj_t *t4 = lv_tabview_add_tab(tv, "TurnSlip");
  lv_obj_t *t5 = lv_tabview_add_tab(tv, "Track");
  lv_obj_t *t6 = lv_tabview_add_tab(tv, "Variometer");
  lv_obj_t *t7 = lv_tabview_add_tab(tv, "GMeter");
  lv_obj_t *t8 = lv_tabview_add_tab(tv, "Clock");
  lv_obj_t *t9 = lv_tabview_add_tab(tv, "Setup");
  //lv_obj_t *t10 = lv_tabview_add_tab(tv, "Demo");

  Onboard_create_Base(tu, &RoundSynthViewSide);
  lv_obj_add_event_cb(tu, speedBgClicked, LV_EVENT_CLICKED, NULL);
  Onboard_create_Base(tz, &RoundSynthViewAttitude);
  lv_obj_add_event_cb(tz, speedBgClicked, LV_EVENT_CLICKED, NULL);
  Onboard_create_Base(tw, &Radar);
  lv_obj_add_event_cb(tw, speedBgClicked, LV_EVENT_CLICKED, NULL);
  // Onboard_create_Base(ty, &RoundHSI);
  Onboard_create_Base(t0, &RoundMapWithControlledSpaces);
  lv_obj_add_event_cb(t0, speedBgClicked, LV_EVENT_CLICKED, NULL);

  Onboard_create_Speed(t1);
  Onboard_create_Attitude(t2);
  Onboard_create_Altimeter(t3);
  Onboard_create_AltimeterDigital(t3b);
  Onboard_create_TurnSlip(t4);
  Onboard_create_Track(t5);
  Onboard_create_Variometer(t6);
  Onboard_create_GMeter(t7);
  Onboard_create_Clock(t8);
  Onboard_create_Setup(t9);
  //Onboard_create(t10);

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

    if(defaultPageOrDemo==0xff){
      DeviceIsDemoMode = 1;
    }
    else
    {
      DeviceIsDemoMode = 0;
      StartupPage = defaultPageOrDemo;
    }


    GFactorDirty = 0;
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

if(DeviceIsDemoMode == 1){

} else {
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

  lv_obj_set_pos(Screen_Attitude_Pitch, 0, -lastAttitudePitch + att_aircraft.header.h/2);

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

  int32_t AltimeterNew = (((QNH * 100) - bmp280Pressure) * 33) / 100;
  // 0.1 Feet/second
  Variometer = AltimeterNew - Altimeter;
  Altimeter = AltimeterNew;

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
         Gyro.z
        );
}

void update_Variometer_lvgl_tick(lv_timer_t *t)
{
  // Variometer => feet/5 seconds => *60/5 => feet/min
  // UI 90° => 1000ft/min => 180° = 2000ft/min => 360° = 4000ft/min
  //
  int32_t VDegree = ((Variometer * 60 / 5) * 36 / 400);
  lv_img_set_angle(Screen_Variometer_Cents, VDegree);
}

void update_Altimeter_lvgl_tick(lv_timer_t *t)
{
  // Altimeter
  float milesDegree = (36.0 * Altimeter) / 100.0;
  float centsDegree = (360.0 * ((Altimeter) % 1000)) / 100.0;
  lv_img_set_angle(Screen_Altitude_Miles, milesDegree + 900);
  lv_img_set_angle(Screen_Altitude_Cents, centsDegree + 900);
}

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
      example1_BMP280_lvgl_tick(t);

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
      GyroBias.x = -(GyroBiasAcquire[0].x+GyroBiasAcquire[1].x+GyroBiasAcquire[2].x)/3.0;
      GyroBias.y = -(GyroBiasAcquire[0].y+GyroBiasAcquire[1].y+GyroBiasAcquire[2].y)/3.0;
      GyroBias.z = -(GyroBiasAcquire[0].z+GyroBiasAcquire[1].z+GyroBiasAcquire[2].z)/3.0;
    case 90:
      stopwatch = datetime;
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
            GyroFiltered.x+GyroBias.x,
            GyroFiltered.y+GyroBias.y,
            GyroFiltered.z+GyroBias.z);
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
            bmp280Pressure/ 100.0);
    lv_label_set_text(SettingStatus3, buf);
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
      if(strcmp("RESET",txt)==0){
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
      if(strcmp("CAGE",txt)==0){
        GyroBias = GyroFiltered;
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

    char buf[15];
    snprintf(buf, sizeof(buf), "%03u", QNH % 1000);
    lv_label_set_text(Screen_Altitude_QNH, buf);
    snprintf(buf, sizeof(buf), "QNH: %u", QNH);
    lv_label_set_text(Screen_Altitude_QNH2, buf);
    snprintf(buf, sizeof(buf), "%+ld", Variometer);
    lv_label_set_text(Screen_Altitude_Variometer2, buf);
  
    example1_BMP280_lvgl_tick(NULL);


    break;

  case RB02_TAB_CLK:
    switch (location)
    {
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
  if(mbox1!=NULL){
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
      changedTab=true;
    }
    break;
  case RB02_TOUCH_E:
    cur++;
      changedTab=true;
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
  if(DeviceIsDemoMode == 0 && changedTab){
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
    lv_label_set_text(VersionLabel, "Version 1.0.6");
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
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}

static void Onboard_create_AltimeterDigital(lv_obj_t *parent)
{
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
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -160 + 48);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "QNH: 1013");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    Screen_Altitude_QNH2 = label;
  }

  const int numDigit = 5;
  for (int c = 0; c < numDigit; c++)
  {
    int k = (DigitFont70x20.header.w + DigitFont70x20.header.h + 8) * (c - 2)-8; // we estimate that you will not fly more than 19999
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

static void Onboard_create_Speed(lv_obj_t *parent)
{
  lv_obj_t *bg = Onboard_create_Base(parent, &RoundAirSpeed);
  if (bg != NULL)
  {
  }
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}

static void Onboard_create_Attitude(lv_obj_t *parent)
{

  lv_obj_set_style_bg_color(parent, lv_color_hex(0x4f3822), LV_PART_ITEMS | LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_CHECKED);
  Onboard_create_Base(parent, &att_middle_big);
  Screen_Attitude_Pitch = Onboard_create_Base(parent, &att_aircraft);
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
}

static void Onboard_create_Altimeter(lv_obj_t *parent)
{
  Onboard_create_Base(parent, &RoundAltimeter);
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  Screen_Altitude_QNH = lv_label_create(parent);
  lv_obj_set_size(Screen_Altitude_QNH, 128, 48);
  lv_obj_align(Screen_Altitude_QNH, LV_ALIGN_CENTER, 184, 0);
  lv_obj_set_style_text_font(Screen_Altitude_QNH, &lv_font_montserrat_48, 0);
  char buf[6];
  snprintf(buf, sizeof(buf), "%03u", QNH % 1000);
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

static void Onboard_create_Track(lv_obj_t *parent)
{
  Onboard_create_Base(parent, &RoundGyro);
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}

static void Onboard_create_Variometer(lv_obj_t *parent)
{
  Onboard_create_Base(parent, &RoundVariometer);
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  Screen_Variometer_Cents = lv_img_create(parent);
  lv_img_set_src(Screen_Variometer_Cents, &fi_needle);
  lv_obj_set_size(Screen_Variometer_Cents, fi_needle.header.w, fi_needle.header.h);
  lv_obj_align(Screen_Variometer_Cents, LV_ALIGN_CENTER, 0, 0);
}

static void Onboard_create_GMeter(lv_obj_t *parent)
{
  Onboard_create_Base(parent, &GMeter);
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
  int32_t AltimeterAbsolute = Altimeter;
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
  snprintf(buf, sizeof(buf), "%+ld", Variometer);
  lv_label_set_text(Screen_Altitude_Variometer2, buf);
}

void update_Clock_lvgl_tick(lv_timer_t *t)
{

  datetime_t datetimeTimer = {0};

  switch (selectedTimer)
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

  turnOnOffDigits(SegmentsA[0], ((diff / 60) / 10) % 10);
  turnOnOffDigits(SegmentsA[1], ((diff / 60)) % 10);
  turnOnOffDigits(SegmentsA[2], ((diff % 60) / 10) % 10);
  turnOnOffDigits(SegmentsA[3], (diff % 60) % 10);
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
