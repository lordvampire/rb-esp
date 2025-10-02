/**
 * RB02 Named Constants
 * v1.3 - Replace magic numbers with descriptive names
 */

#pragma once

// Display dimensions
#define DISPLAY_WIDTH_PIXELS        480
#define DISPLAY_HEIGHT_PIXELS       480
#define DISPLAY_CENTER_X            240
#define DISPLAY_CENTER_Y            240

// Attitude indicator constants
#define ATTITUDE_ANGLE_SCALE        10.0f     // LVGL angle scale (degrees * 10)
#define ATTITUDE_PITCH_OFFSET       900       // Pitch indicator offset
#define ATTITUDE_UPDATE_THRESHOLD   0.2f      // Degrees - skip update if change < this
#define ATTITUDE_BALANCE_ALPHA_SCALE 250.0f   // Balance alpha conversion

// GPS acceleration limits (G-force)
#define GPS_ACCEL_MAX_LIMIT         1.5f
#define GPS_ACCEL_MIN_LIMIT         -1.5f

// Speed conversion factors
#define KMH_TO_MS_FACTOR            3.6f      // km/h to m/s
#define METERS_TO_FEET_FACTOR       3.28084f  // Altitude conversion
#define G_ACCELERATION              9.81f     // Gravitational constant (m/s²)

// GPS timing
#define GPS_DT_TO_SECONDS           1000000.0f // Microseconds to seconds
#define GPS_ALTITUDE_SMOOTHING      6.0f       // Altitude smoothing factor

// Sensor thresholds
#define GPS_ALTITUDE_MIN_VALID      0.0001f    // Minimum valid GPS altitude

// UI positioning
#define UI_WARNING_Y_OFFSET         200
#define UI_CENTER_ALIGN             0

// String formatting thresholds (from v1.1)
#define BMP280_TEMP_THRESHOLD       0.5f   // °C
#define BMP280_PRESS_THRESHOLD      0.5f   // hPa
#define BATTERY_VOLTAGE_THRESHOLD   0.1f   // V
#define IMU_DATA_THRESHOLD          0.2f   // Units
#define ATTITUDE_DISPLAY_THRESHOLD  0.2f   // Degrees

// Workflow states
#define WORKFLOW_SENSORS_READY      100    // When sensors are calibrated

// Memory/Performance
#define UART_RX_BUFFER_SIZE         256    // UART receive buffer
#define DRIVER_LOOP_BASE_MS         10     // Base loop delay

// Color values (frequently used)
#define COLOR_WHITE                 0xFFFFFF
#define COLOR_BLACK                 0x000000
#define COLOR_RED                   0xFF0000
#define COLOR_GREEN                 0x00FF00
#define COLOR_BLUE                  0x0000FF

// v1.3: Floating-point optimization - Replace division with multiplication
#define INV_2_0F                    0.5f       // 1/2.0
#define INV_6_0F                    0.16666667f // 1/6.0
#define INV_10_0F                   0.1f        // 1/10.0
#define INV_100_0F                  0.01f       // 1/100.0
#define INV_180_0F                  0.00555556f // 1/180.0 (for deg to rad)
#define INV_32767_0F                0.00003051851f // 1/32767.0
#define DEG_TO_RAD                  0.017453292f // PI/180
#define RAD_TO_DEG                  57.29578f    // 180/PI
