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
 * 05 -> Display with Stratux BLE Traffic
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/

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
#include "RB02_Config.h"
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
#ifdef RB02_ESP_BLUETOOTH
#include "RB05_ESP_Bluetooth.h"
#endif

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
            loopThreshold = 1000 / (10 + DriverLoopMilliseconds);
            // When sensor is ready (After Calibration)
            if (workflow > 100)
            {
                BAT_Get_Volts();
                Get_BMP280();
            }
        }
        loopThreshold--;
        vTaskDelay(pdMS_TO_TICKS(10 + DriverLoopMilliseconds));
    }
    vTaskDelete(NULL);
}

uint8_t RB02_Config_NVS_Get_BluetoothSettings();

// v1.4: Enhanced boot sequence with delays for stability
void Driver_Init(void)
{
    ESP_LOGI("BOOT", "Starting peripheral initialization...");

    Flash_Searching();
    BAT_Init();

    // v1.4: Delay after battery init for voltage stabilization
    vTaskDelay(pdMS_TO_TICKS(50));

    // v1.4: I2C init with retry logic (see I2C_Driver.c)
    I2C_Init();

    // v1.4: Critical delay for I2C bus stabilization
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI("BOOT", "Initializing I2C sensors...");
    PCF85063_Init();
    vTaskDelay(pdMS_TO_TICKS(50));

    QMI8658_Init();
    vTaskDelay(pdMS_TO_TICKS(50));

    EXIO_Init(); // GPIO Expander
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI("BOOT", "I2C devices initialized");

#ifdef RB02_ESP_BLUETOOTH
    //if (RB02_Config_NVS_Get_BluetoothSettings() != 0)
    {
        RB02_BluetoothLowEnergy_Init();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#endif

    ESP_LOGI("BOOT", "Starting driver loop task...");
    xTaskCreatePinnedToCore(
        Driver_Loop,
        "Other Driver task",
        4096,
        NULL,
        3,
        NULL,
        0);
}
void RB02_Main();
void Set_Backlight(uint8_t Light);

// v1.4: Enhanced boot sequence for maximum stability
void app_main(void)
{
    ESP_LOGI("BOOT", "=== RB02 Boot Sequence v1.4 ===");

    // Initialize NVS.
    ESP_LOGI("BOOT", "Initializing NVS flash...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOGI("BOOT", "NVS initialized");

    // v1.4: System stabilization delay after NVS init
    vTaskDelay(pdMS_TO_TICKS(100));

    Driver_Init();

    // v1.4: Critical delay for sensors to complete initialization
    vTaskDelay(pdMS_TO_TICKS(200));

#ifdef RB_ENABLE_UART
    // v1.4: UART init moved after sensor init to avoid GPIO conflicts
    ESP_LOGI("BOOT", "Initializing UART for GPS...");

    // v1.4: Load baudrate from NVS (user-configurable in settings)
    int32_t gps_baudrate = RB02_Config_NVS_Get_GpsBaudrate();

    // v1.4: Configure UART parameters BEFORE driver install
    const uart_config_t uart_config = {
        .baud_rate = gps_baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_N, &uart_config));

    // Set PIN for Waveshare Round Display
    uart_set_pin(UART_N, UART_PIN_NO_CHANGE, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install driver
    uart_driver_install(UART_N, 256, 0, 0, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI("BOOT", "UART initialized (GPIO43/44, %ld baud)", gps_baudrate);

    // v1.4: GPS boot test - check if UART receiving data from NEO-6M
    ESP_LOGI("BOOT", "Testing GPS UART reception...");
    ESP_LOGI("BOOT", "NEO-6M INFO: Blue LED should BLINK when GPS fix acquired");
    ESP_LOGI("BOOT", "NEO-6M INFO: First fix may take several minutes (needs sky view)");
    static uint8_t gps_test_buf[256];
    int test_attempts = 0;
    int total_bytes = 0;

    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(300)); // Wait for GPS data
        int rxBytes = uart_read_bytes(UART_N, gps_test_buf, sizeof(gps_test_buf) - 1, 150 / portTICK_PERIOD_MS);
        test_attempts++;

        if (rxBytes > 0) {
            total_bytes += rxBytes;
            gps_test_buf[rxBytes] = 0;
            ESP_LOGI("BOOT", "GPS test #%d: %d bytes received", i + 1, rxBytes);

            // Show as ASCII text (NMEA sentences are readable)
            ESP_LOGI("GPS_ASCII", "%.*s", rxBytes, gps_test_buf);

            // Also show hex for first packet
            if (i == 0) {
                ESP_LOG_BUFFER_HEXDUMP("GPS_HEX", gps_test_buf, rxBytes > 32 ? 32 : rxBytes, ESP_LOG_INFO);
            }
        } else {
            ESP_LOGI("BOOT", "GPS test #%d: No data", i + 1);
        }
    }

    if (total_bytes > 0) {
        ESP_LOGI("BOOT", "=== GPS UART OK: %d bytes in %d attempts ===", total_bytes, test_attempts);
        ESP_LOGI("BOOT", "If you see invalid NMEA: Module has no GPS fix yet (wait for blue LED blink)");
    } else {
        ESP_LOGW("BOOT", "=== GPS UART: No data received in %d attempts ===", test_attempts);
        ESP_LOGW("BOOT", "Possible causes:");
        ESP_LOGW("BOOT", "  1. Wrong wiring: NEO-6M TX must connect to ESP32 GPIO44 (RXD)");
        ESP_LOGW("BOOT", "  2. Wrong baudrate: Try changing GPS baudrate in settings");
        ESP_LOGW("BOOT", "  3. Faulty module: Try different NEO-6M module");
        ESP_LOGW("BOOT", "  4. Power issue: Check VCC/GND connections");
    }
#endif

    // v1.4: LCD initialization with extended delays
    ESP_LOGI("BOOT", "Initializing LCD display...");
    LCD_Init();

    // v1.4: Critical delay for LCD stabilization (ST7701S is sensitive)
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI("BOOT", "LCD initialized");

// 1.1.23 Add non touch unified official support
#ifdef RB_02_DISPLAY_TOUCH
    ESP_LOGI("BOOT", "Initializing touch controller...");
    Touch_Init();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI("BOOT", "Touch initialized");
#endif

    ESP_LOGI("BOOT", "Initializing SD card...");
    SD_Init();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI("BOOT", "SD card initialized");

    ESP_LOGI("BOOT", "Initializing LVGL graphics...");
    LVGL_Init();
    ESP_LOGI("BOOT", "LVGL initialized");

    // v1.4: Final delay before starting application
    vTaskDelay(pdMS_TO_TICKS(100));

    /********************* Demo *********************/
    // Lvgl_Example1();
    //
    if (false) // Display test minimal routine
    {
        Set_Backlight(100);
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(255, 0, 0), 0);
    }
    else
    {
        ESP_LOGI("BOOT", "Starting RB02 application...");
        RB02_Main();
        ESP_LOGI("BOOT", "=== Boot Complete ===");
    }
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
