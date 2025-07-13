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
#pragma once
#include "RB02_GPSDiag.h"

#ifdef RB_ENABLE_GPS_DIAG

extern lv_style_t style_title;
lv_obj_t *GPSDiag_UARTBaud = NULL;
lv_obj_t *GPSDiag_NMEADebugRMC = NULL;
lv_obj_t *GPSDiag_NMEADebugGGA = NULL;
lv_obj_t *GPSDiag_NMEADebugLine = NULL;
lv_obj_t *GPSDiag_NMEADebugSummary = NULL;

lv_obj_t *RB02_GPSDiag_CreateScreen(lv_obj_t *parent)
{
    int y = 24;
    if (parent != NULL && GPSDiag_UARTBaud == NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
        lv_obj_set_width(label, 400);
        lv_obj_set_height(label, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
        lv_label_set_text(label, "UART Baud: --");
        // //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
        //lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);

        GPSDiag_UARTBaud = label;
        y += 24;
    }

    if (parent != NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
        lv_obj_set_width(label, 400);
        lv_obj_set_height(label, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
        lv_label_set_text(label, "GGA Status:");
        // //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
        ////lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        y += 20;
    }

    if (parent != NULL && GPSDiag_NMEADebugGGA == NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
        lv_obj_set_width(label, 400);
        lv_obj_set_height(label, 32);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
        lv_label_set_text(label, "NO G_GGA RECEIVED!");
        // //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
        //lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);

        GPSDiag_NMEADebugGGA=label;
        y += 32;
    }
    if (parent != NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
        lv_obj_set_width(label, 400);
        lv_obj_set_height(label, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
        lv_label_set_text(label, "RMC Status:");
        // //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
        //lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        y += 20;
    }
    if (parent != NULL && GPSDiag_NMEADebugRMC == NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
        lv_obj_set_width(label, 400);
        lv_obj_set_height(label, 32);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
        lv_label_set_text(label, "NO G_RMC RECEIVED!");
        // //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
        //lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        GPSDiag_NMEADebugRMC=label;
        y += 32;
    }
    if (parent != NULL && GPSDiag_NMEADebugSummary == NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
        lv_obj_set_width(label, 400);
        lv_obj_set_height(label, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
        lv_label_set_text(label, "Parsed: NOT WORKING!");
        // //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
        //lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        GPSDiag_NMEADebugSummary=label;
        y += 32;
    }
    if (parent != NULL && GPSDiag_NMEADebugLine == NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
        lv_obj_set_width(label, 440);
        lv_obj_set_height(label, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
        lv_label_set_text(label, "Here will be placed the NMEA String received by the GPS if any. If nothing is here, change the speed.");
        // //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);

        GPSDiag_NMEADebugLine = label;
    }

    return NULL;
}

#endif