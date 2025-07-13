#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TCA9554PWR.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "ST7701S.h"
#include "RB02.h"
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_28
// ESP32-S3-2.8C
#ifdef RB_02_DISPLAY_TOUCH
// TODO: we shall remove the compilation of the C file
#include "GT911.h"
#endif
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_21
// ESP32-S3-2.1C
#include "CST820.h"
#endif
#include "SD_MMC.h"
#include "LVGL_Driver.h"
#include "lvgl.h"
#include "demos/lv_demos.h"
#include "Buzzer.h"
#include "BAT_Driver.h"
#include "nvs_flash.h"
#include "BAT_Driver.h"
#include "driver/uart.h"

// 1.1.9
uint8_t DriverLoopMilliseconds = 40;

// 1.0.9 Install in the loop BMP280
void Get_BMP280(void);   // Declaration
extern uint8_t workflow; // When sensor is ready (After Calibration)
void Driver_Loop(void *parameter)
{
    int loopThreshold = 10; // Delay the polling of certain sensors
    while (1)
    {
        QMI8658_Loop();
        // Delay the polling of certain sensors
        if (loopThreshold == 0)
        {
            RTC_Loop();
            // 1.1.17 Improving BMP Read, by default the loop is 20Hz and read at 1Hz
            loopThreshold = 1000/(10+DriverLoopMilliseconds);
            // When sensor is ready (After Calibration)
            if (workflow > 100)
            {
                BAT_Get_Volts();
                Get_BMP280();
            }
        }
        loopThreshold--;
        vTaskDelay(pdMS_TO_TICKS(10+DriverLoopMilliseconds));
    }
    vTaskDelete(NULL);
}
void Driver_Init(void)
{
    Flash_Searching();
    BAT_Init();
    I2C_Init();
    PCF85063_Init();
    QMI8658_Init();
    EXIO_Init(); // Example Initialize EXIO
    xTaskCreatePinnedToCore(
        Driver_Loop,
        "Other Driver task",
        4096,
        NULL,
        3,
        NULL,
        0);
}
void app_main(void)
{

    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // Wireless_Init();
    Driver_Init();

    // 1.0.9
    // Install the UART Driver as soon as possible
    uart_driver_install(1, 256, 0, 0, NULL, 0);
    // Set PIN for Waveshare 2.8" Round based on Wiki Schematics
    uart_set_pin(1, UART_PIN_NO_CHANGE, GPIO_NUM_44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    LCD_Init();
// 1.1.23 Add non touch unified official support
#ifdef RB_02_DISPLAY_TOUCH
    Touch_Init();
#endif
    SD_Init();
    LVGL_Init();
    /********************* Demo *********************/
    // Lvgl_Example1();
    RB02_Example1();
    // lv_demo_widgets();
    // lv_demo_keypad_encoder();
    // lv_demo_benchmark();
    // lv_demo_stress();
    // lv_demo_music();

    while (1)
    {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
