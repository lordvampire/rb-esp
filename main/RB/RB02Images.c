/**
 * Copyright (c) 2024 XIAPROJECTS SRL
 * Distributable under the terms of The "BSD New" License
 * that can be found in the LICENSE file, herein included
 * as part of this header.
 * This source is part of the project RB
 */


#ifndef RB02_IMAGES
#define RB02_IMAGES

// Altimeter Digital
#include "DigitFont100x25.c"
#include "DigitFont70x20.c"


// Speed
#include "arcRed.c"
#include "arcYellow.c"
#include "arcGreen.c"
#include "arcWhite.c"


// Attitude
#include "att_circle_top_TL.c"
#include "att_circle_top_TR.c"
#include "att_circle_top_T.c"
#include "att_middle_big.c"
#include "att_aircraft.c"
#include "att_tri.c"

// Gyro
#include "RoundGyro.c"
#include "RoundGyroHeading.c"

// Variometer
#include "RoundVariometer.c"

// Altimeter
#include "RoundAltimeter.c"
#include "fi_needle.c"
#include "fi_needle_small.c"

// Slip & Turn
#include "turn_coordinator.c"
#include "fi_tc_airplane.c"



#ifdef ENABLE_DEMO_SCREENS
#include "RoundSynthViewSide.c"
#include "Radar.c"
#include "RoundMapWithControlledSpaces.c"
#include "RoundHSI.c"
#endif

#endif