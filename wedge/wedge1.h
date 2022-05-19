#ifndef WEDGE1_H
#define WEDGE1_H

#include "../coord_interp.h"

// Calibration/config to match the power wedge hardware 

// Mounting angle offset to calibrated angle?

// Separate calibration for each angle sensor...

// Calibration: angle sensor 1 -- voltage to angle
const int numCalibPoints1 = 8; // match next line
coord_t voltToAngle1[8] = 
{
    {0.0000, -15},
    {0.3533, -12},
    {0.5189, -11},
    {3.8300,   9},
    {4.0000,  10},
    {4.4480,  90},
    {4.5040, 100},
    {5.2880, 240}
};

// Calibration: angle sensor 2 -- voltage to angle
const int numCalibPoints2 = 8; // match next line
coord_t voltToAngle2[8] = 
{
    {0.0000, -15},
    {0.4017, -12},
    {0.5644, -11},
    {3.8200,   9},
    {4.0000,  10},
    {4.4373,  90},
    {4.4920, 100},
    {5.2573, 240}
};

const double toleranceHighRes = 1.0; // +/- angle (degrees)
const double toleranceLowRes = 5.0; // +/- angle (degrees)

// Additional angle (overshoot) due to relay turn off delay, etc.
const double stopTimeUp = 0.7; // angle (degrees)
const double stopTimeDn = 0.7; // angle (degrees)

// Time to wait after movement for relay to turn off and movement to settle
const unsigned long delayAfterMove = 200; // milliseconds

// Safety / Error limits
const double maxSetpoint = 91.0;          // angle (degrees)
const double minSetpoint = -11.0;         // angle (degrees)
const double sensorsConvergeToleranceHighRes = 1.0; // angle (degrees)
const double sensorsConvergeToleranceLowRes = 10.0; // angle (degrees)
const double sensorTransistionAngle = 10.0; // angle (degrees) High-res to Low-res
const unsigned long maxRelayOnTime = 15000; // milliseconds
const unsigned long maxRelayCyclesPer  = 6;
const unsigned long maxRelayCyclesTime = 2000; // milliseconds

// Angle sensor error thresholds; Angle sensor valid range
// Don't use this to limit movement (use maxSetpoint or something else).
const double vMinValidSensor1 = 0.30; // minimum valid sensor1 voltage
const double vMaxValidSensor1 = 4.70; // maximum valid sensor1 voltage
const double vMinValidSensor2 = 0.30; // minimum valid sensor2 voltage
const double vMaxValidSensor2 = 4.65; // maximum valid sensor2 voltage

#endif
