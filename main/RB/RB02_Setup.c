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
#include "RB02_Console.h"
#ifdef RB_ENABLE_SETUP
#include <stdio.h>

#ifdef RB02_ESP_BLUETOOTH
static void RB02_Setup_Changed_EnableBluetooth(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    singletonConfig()->settingsBluetoothEnabled = 1;
  }
  else
  {
    singletonConfig()->settingsBluetoothEnabled = 0;
  }
  RB02_Config_NVS_Store_BluetoothSettings();
}
static void RB02_Setup_Changed_EnableBluetoothGps(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    singletonConfig()->settingsBluetoothGPS = 1;
  }
  else
  {
    singletonConfig()->settingsBluetoothGPS = 0;
  }
  RB02_Config_NVS_Store_BluetoothSettings();
}
#endif

lv_obj_t *RB02_Setup_CreateScreen(RB02_Status *status, lv_obj_t *parent, int *lineY)
{
    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, *lineY);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        *lineY += 40;

        status->ui.SettingsOperativeSummary = label;
    }
#ifdef RB02_ESP_BLUETOOTH
    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, 150, 16);
        lv_obj_align(label, LV_ALIGN_CENTER, -75, *lineY);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_label_set_text(label, "Enable Bluetooth");
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_t *sw = lv_switch_create(parent);
        if (status->settingsBluetoothEnabled != 0)
        {
            lv_obj_add_state(sw, LV_STATE_CHECKED);
        }
        lv_obj_set_size(sw, 65, 40);
        lv_obj_align(sw, LV_ALIGN_CENTER, 40, *lineY);
        lv_obj_add_event_cb(sw, RB02_Setup_Changed_EnableBluetooth, LV_EVENT_VALUE_CHANGED, sw);
        *lineY += 40;
    }
    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, 150, 16);
        lv_obj_align(label, LV_ALIGN_CENTER, -75, *lineY);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_label_set_text(label, "Use BT GPS");
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_t *sw = lv_switch_create(parent);
        if (status->settingsBluetoothGPS != 0)
        {
            lv_obj_add_state(sw, LV_STATE_CHECKED);
        }
        lv_obj_set_size(sw, 65, 40);
        lv_obj_align(sw, LV_ALIGN_CENTER, 40, *lineY);
        lv_obj_add_event_cb(sw, RB02_Setup_Changed_EnableBluetoothGps, LV_EVENT_VALUE_CHANGED, sw);
        *lineY += 40;
    }
#endif
    return NULL;
}
#endif