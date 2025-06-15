#include "Vendor.h"

extern float GMeterScale;
extern uint8_t isKmh;
extern uint16_t degreeStart;
extern uint16_t degreeEnd;
extern uint16_t speedKtStart;
extern uint16_t speedKtEnd;
extern uint16_t speedWhite;
extern uint16_t speedGreen;
extern uint16_t speedYellow;
extern uint16_t speedRed;
extern int32_t bmp280override;
extern uint8_t DriverLoopMilliseconds;
extern float AttitudeBalanceAlpha;
extern float FilterMoltiplier;
extern float FilterMoltiplierGyro;

void nvsStorePCal();
void nvsStoreSpeedArc();
void nvsStoreFilters();

void VendorMakeDefaultsDefault()
{
    bmp280override = -260;

    GMeterScale = 3;

    FilterMoltiplier = 3.0;
    FilterMoltiplierGyro = 10.0;
    AttitudeBalanceAlpha = 0.90;
    DriverLoopMilliseconds = 90;
}

#if ENABLE_VENDOR == RB_VENDOR_1
void VendorMakeDefaults()
{
    VendorMakeDefaultsDefault();

    isKmh = 0;
    degreeStart = 0;
    degreeEnd = 320;
    speedKtStart = 0;
    speedKtEnd = 55;
    speedWhite = 60;
    speedGreen = 105;
    speedYellow = 225;
    speedRed = 315;

    nvsStoreSpeedArc();
    nvsStorePCal();
    nvsStoreFilters();
}
#endif
#if ENABLE_VENDOR == RB_VENDOR_2
void VendorMakeDefaults()
{
    VendorMakeDefaultsDefault();

    nvsStoreSpeedArc();
    nvsStorePCal();
    nvsStoreFilters();
}
#endif
#if ENABLE_VENDOR == RB_VENDOR_3
void VendorMakeDefaults()
{
    VendorMakeDefaultsDefault();

    nvsStoreSpeedArc();
    nvsStorePCal();
    nvsStoreFilters();
}
#endif
