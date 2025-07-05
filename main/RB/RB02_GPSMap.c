/**
 * Copyright (c) 2024 XIAPROJECTS SRL
 * Distributable under the terms of The "BSD New" License
 * that can be found in the LICENSE file, herein included
 * as part of this header.
 * This source is part of the project RB
 */

#include "RB02_GPSMap.h"
#ifdef RB_ENABLE_MAP
#include "lvgl.h"
#include <stdio.h>
#ifdef RB_02_ENABLE_INTERNALMAP
#include "images/GPSMapInternal.c"
#endif

// #define RB_MAP_MULTI_TILE 1

#define RB_02_ZOOM_SUPPORTED 3
uint8_t zoomSlicerMapper[RB_02_ZOOM_SUPPORTED] = {100, 50, 20};

void RB02_GPSMap_SquareGenerator(RB02_GpsMapStatus *gpsMapStatus, float centerLatitude, float centerLongitude)
{
#ifdef RB_02_ENABLE_EXTERNALMAP
#ifdef RB_MAP_MULTI_TILE
    switch (gpsMapStatus->zoomLevel)
    {
    case 1:
        gpsMapStatus->tileSizeWidth = 60;
        gpsMapStatus->tileSizeHeight = 80;
        break;
    case 2:
        gpsMapStatus->tileSizeWidth = 30;
        gpsMapStatus->tileSizeHeight = 40;
        break;
    default:
        gpsMapStatus->tileSizeWidth = 96;
        gpsMapStatus->tileSizeHeight = 120;
        break;
    }
#else
    gpsMapStatus->tileSizeWidth = 360;
    gpsMapStatus->tileSizeHeight = 480;
#endif

    int16_t centerLatitudeInt = centerLatitude * 100;
    int16_t centerLongitudeInt = centerLongitude * 100;
    centerLatitudeInt = (centerLatitudeInt / zoomSlicerMapper[gpsMapStatus->zoomLevel]) * zoomSlicerMapper[gpsMapStatus->zoomLevel];
    centerLongitudeInt = (centerLongitudeInt / zoomSlicerMapper[gpsMapStatus->zoomLevel]) * zoomSlicerMapper[gpsMapStatus->zoomLevel];

#ifdef RB_MAP_MULTI_TILE
    int cx = SCREEN_WIDTH / gpsMapStatus->tileSizeWidth;
    int cy = SCREEN_HEIGHT / gpsMapStatus->tileSizeHeight;
    int32_t latb = 10 * centerLatitudeInt + (zoomSlicerMapper[gpsMapStatus->zoomLevel] * cx) / 2;
    int32_t late = 10 * centerLatitudeInt - (zoomSlicerMapper[gpsMapStatus->zoomLevel] * cx) / 2;

    int32_t lob = 10 * centerLongitudeInt - (zoomSlicerMapper[gpsMapStatus->zoomLevel] * cy) / 2;
    int32_t loe = 10 * centerLongitudeInt + (zoomSlicerMapper[gpsMapStatus->zoomLevel] * cy) / 2;
#else
    int32_t latb = centerLatitudeInt + (zoomSlicerMapper[gpsMapStatus->zoomLevel]);
    int32_t late = centerLatitudeInt;

    int32_t lob = centerLongitudeInt;
    int32_t loe = centerLongitudeInt + (zoomSlicerMapper[gpsMapStatus->zoomLevel]);
#endif
    if (gpsMapStatus->mapLatitudeBegin != latb || gpsMapStatus->mapLongitudeBegin != lob)
    {
        gpsMapStatus->mapDirty = 1;
    }

    gpsMapStatus->mapLatitudeBegin = latb;
    gpsMapStatus->mapLatitudeEnd = late;
    gpsMapStatus->mapLongitudeBegin = lob;
    gpsMapStatus->mapLongitudeEnd = loe;

    printf("RB02_GPSMap_SquareGenerator {%d %d},%f,%f, %ld %ld  %ld %ld\n", centerLatitudeInt, centerLongitudeInt, centerLatitude, centerLongitude, gpsMapStatus->mapLongitudeBegin, gpsMapStatus->mapLongitudeEnd, gpsMapStatus->mapLatitudeBegin, gpsMapStatus->mapLatitudeEnd);
#endif
}

void RB02_GPSMap_Touch_N(RB02_GpsMapStatus *gpsMapStatus)
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    if (gpsMapStatus->zoomLevel + 1 >= RB_02_ZOOM_SUPPORTED)
    {
    }
    else
    {
        gpsMapStatus->zoomLevel++;
    }
    gpsMapStatus->mapLatitudeBegin = 0;
    gpsMapStatus->latitude100 = 0;
#endif
}
void RB02_GPSMap_Touch_S(RB02_GpsMapStatus *gpsMapStatus)
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    if (gpsMapStatus->zoomLevel > 0)
    {
        gpsMapStatus->zoomLevel--;
    }
    else
    {
    }
    gpsMapStatus->mapLatitudeBegin = 0;
    gpsMapStatus->latitude100 = 0;
#endif
}

void RB02_GPSMap_ReloadTiles(RB02_GpsMapStatus *gpsMapStatus, lv_obj_t *parent)
{

    //
    char filename[41];

    for (int y = 0; y < SCREEN_HEIGHT / gpsMapStatus->tileSizeHeight; y++)
    {
        for (int x = 0; x < SCREEN_WIDTH / gpsMapStatus->tileSizeWidth; x++)
        {

            lv_obj_t *backgroundImage = gpsMapStatus->tiles[x * (SCREEN_WIDTH / gpsMapStatus->tileSizeWidth) + y];
            if (backgroundImage == NULL)
            {
                backgroundImage = lv_img_create(parent);
                gpsMapStatus->tiles[x * (SCREEN_WIDTH / gpsMapStatus->tileSizeWidth) + y] = backgroundImage;
            }
            else
            {
            }

            // Round display optimisation
            /*
            if ((x == 0 && y == 0) ||
                (x == (SCREEN_WIDTH / gpsMapStatus->tileSizeWidth) - 1 && y == 0) ||
                (x == 0 && y == (SCREEN_HEIGHT / gpsMapStatus->tileSizeHeight) - 1) ||
                (x == (SCREEN_WIDTH / gpsMapStatus->tileSizeWidth) - 1 && y == (SCREEN_HEIGHT / gpsMapStatus->tileSizeHeight) - 1)

            )
            {
                continue;
            }
            */

            //= 43660
#ifdef RB_MAP_MULTI_TILE
            int32_t currentLatitudeTile = (gpsMapStatus->mapLatitudeBegin - y * zoomSlicerMapper[gpsMapStatus->zoomLevel]) / zoomSlicerMapper[gpsMapStatus->zoomLevel];
            int32_t currentLongitudeTile = (gpsMapStatus->mapLongitudeBegin + x * zoomSlicerMapper[gpsMapStatus->zoomLevel]) / zoomSlicerMapper[gpsMapStatus->zoomLevel];
#else
            int32_t currentLatitudeTile = (gpsMapStatus->mapLatitudeBegin);
            int32_t currentLongitudeTile = (gpsMapStatus->mapLongitudeBegin);
#endif
            uint8_t folderIndex = 100 / zoomSlicerMapper[gpsMapStatus->zoomLevel];
            snprintf(filename, sizeof(filename), "S:/z%d/%04ld%04ld.bmp", folderIndex, currentLatitudeTile, currentLongitudeTile);
            lv_label_set_text(gpsMapStatus->labelTilePath, filename);
            lv_img_set_src(backgroundImage, filename);
            // lv_obj_set_size(backgroundImage, gpsMapStatus->tileSizeWidth, gpsMapStatus->tileSizeHeight);
            lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
#ifdef RB_MAP_MULTI_TILE
            int16_t lx = x * gpsMapStatus->tileSizeWidth;
            int16_t ly = y * gpsMapStatus->tileSizeHeight;
#else
            int16_t lx = 0;
            int16_t ly = 0;
#endif
            lv_obj_align(backgroundImage, LV_ALIGN_CENTER, lx, ly);
            lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
            printf("Tile:  %dx%d %s %dx%d %dx%d %dx%d\n", x, y, filename, lv_obj_get_width(backgroundImage), lv_obj_get_height(backgroundImage), lx, ly, gpsMapStatus->tileSizeWidth, gpsMapStatus->tileSizeHeight);
        }
    }

    lv_obj_move_foreground(gpsMapStatus->poiMy);
    lv_obj_move_foreground(gpsMapStatus->labelLatitude);
    lv_obj_move_foreground(gpsMapStatus->labelLongitude);
    gpsMapStatus->mapDirty = 0;
}

lv_obj_t *RB02_GPSMap_CreateScreen(RB02_GpsMapStatus *gpsMapStatus, lv_obj_t *parent)
{

    lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    gpsMapStatus->mapLatitudeBegin = 0;
    gpsMapStatus->mapLongitudeBegin = 0;
    gpsMapStatus->mapLatitudeEnd = 0;
    gpsMapStatus->mapLongitudeEnd = 0;

#ifdef RB_02_ENABLE_INTERNALMAP

    if (true)
    {
        lv_obj_t *backgroundImage = lv_img_create(parent);
        lv_img_set_src(backgroundImage, &GPSMapInternal);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);
        // 1.1.9 Remove scrolling for Turbolence touch screen
        lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    }

    //
    gpsMapStatus->mapLatitudeBegin = 426380;
    gpsMapStatus->mapLongitudeBegin = 111950;
    gpsMapStatus->mapLatitudeEnd = 433430;
    gpsMapStatus->mapLongitudeEnd = 125000;

    //
    gpsMapStatus->mapLatitudeBegin = 43660;
    gpsMapStatus->mapLongitudeBegin = 11460;
    gpsMapStatus->mapLatitudeEnd = 42980;
    gpsMapStatus->mapLongitudeEnd = 12350;
#endif

#ifdef RB_02_ENABLE_EXTERNALMAP
    for (int x = 0; x < (SCREEN_WIDTH / RB_02_ENABLE_EXTERNALMAP_WIDTH) * (SCREEN_WIDTH / RB_02_ENABLE_EXTERNALMAP_WIDTH); x++)
    {
        gpsMapStatus->tiles[x] = NULL;
    }
#else

#endif

    if (true)
    {
        lv_obj_t *poi = lv_obj_create(parent);
        lv_obj_set_scrollbar_mode(poi, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(poi, 16, 16);
        lv_obj_align(poi, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(poi, lv_color_make(0x0, 0x0, 0xff), 0);
        lv_obj_set_style_radius(poi, LV_RADIUS_CIRCLE, 0);
        lv_obj_clear_flag(poi, LV_OBJ_FLAG_CLICKABLE);
        gpsMapStatus->poiMy = poi;
    }

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "-------");
        gpsMapStatus->labelTilePath = label;
    }

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, 150, 20);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 215);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "-------");
        gpsMapStatus->labelLatitude = label;
    }

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, 150, 20);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 230);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "--------");
        gpsMapStatus->labelLongitude = label;
    }

    return NULL;
}

void RB02_GPSMap_Tick(RB02_GpsMapStatus *gpsMapStatus, gps_t *gpsStatus, lv_obj_t *parent)
{

    // Some example GPS Coordinates
    // Arezzo
    // gpsStatus->latitude= 43.45513012173618;
    // gpsStatus->longitude=11.847354299755676;
    // gpsStatus->latitude= 43.11126;
    // gpsStatus->longitude = 12.08190;

    // Skip no sense GPS Positions
    if (gpsStatus->latitude < 1 && gpsStatus->latitude > -1)
    {
        return;
    }

    float currLon100 = (gpsStatus->longitude * 100.0);
    float currLat100 = (gpsStatus->latitude * 100.0);

    if (gpsMapStatus->latitude100 == (int32_t)currLat100 && gpsMapStatus->longitude100 == (int32_t)currLon100)
    {
        return;
    }

    RB02_GPSMap_SquareGenerator(gpsMapStatus, gpsStatus->latitude, gpsStatus->longitude);

    if (gpsMapStatus->mapLongitudeBegin - gpsMapStatus->mapLongitudeEnd == 0)
    {
        return;
    }
    if (gpsMapStatus->mapLatitudeBegin - gpsMapStatus->mapLatitudeEnd == 0)
    {
        return;
    }

    if (gpsMapStatus->mapDirty != 0)
    {
        RB02_GPSMap_ReloadTiles(gpsMapStatus, parent);
    }

    char buf[20];
    snprintf(buf, sizeof(buf), "LAT:%.5f", gpsStatus->latitude);
    lv_label_set_text(gpsMapStatus->labelLatitude, buf);
    snprintf(buf, sizeof(buf), "LON:%.5f", gpsStatus->longitude);
    lv_label_set_text(gpsMapStatus->labelLongitude, buf);

    //

    float longitudeEscursion = (gpsMapStatus->mapLongitudeEnd - gpsMapStatus->mapLongitudeBegin);
    float latitudeEscursion = -(gpsMapStatus->mapLatitudeEnd - gpsMapStatus->mapLatitudeBegin);

    gpsMapStatus->latitude100 = currLat100;
    gpsMapStatus->longitude100 = currLon100;

    float diffLatitude = currLat100 - gpsMapStatus->mapLatitudeEnd;
    float diffLongitude = currLon100 - gpsMapStatus->mapLongitudeBegin;

    float percentageX = diffLongitude / longitudeEscursion;
    float percentageY = diffLatitude / latitudeEscursion;

    /*
        if (percentageX > 1.0)
        {
            percentageX = 1.0;
        }
        if (percentageX < 0)
        {
            percentageX = 0;
        }
        if (percentageY > 1.0)
        {
            percentageY = 1.0;
        }
        if (percentageY < 0)
        {
            percentageY = 0;
        }
    */

    lv_obj_align(gpsMapStatus->poiMy, LV_ALIGN_CENTER, gpsMapStatus->tileSizeWidth * (percentageX)-gpsMapStatus->tileSizeWidth / 2, gpsMapStatus->tileSizeHeight / 2 - gpsMapStatus->tileSizeHeight * (percentageY));
    printf("%f %f %f %f %f %f %f %f %ld %ld %ld %ld\n", gpsStatus->latitude, gpsStatus->longitude,
           percentageX,
           percentageY,
           longitudeEscursion,
           latitudeEscursion,
           diffLatitude,
           diffLongitude,
           gpsMapStatus->mapLatitudeEnd,
           gpsMapStatus->mapLatitudeBegin,
           gpsMapStatus->mapLongitudeEnd,
           gpsMapStatus->mapLongitudeBegin);
}
#endif