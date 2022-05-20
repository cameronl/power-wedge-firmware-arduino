#ifndef Z_FAKE_WEDGE_H
#define Z_FAKE_WEDGE_H

#include "../coord_interp.h"

// Alternate config to match an emulated power wedge (~ 0.3v - 3.0v angle sensor)

// Mounting angle offset to calibrated angle?

// Separate calibration for each angle sensor...

// Calibration: angle sensor 1 -- voltage to angle
const int numCalibPoints1 = 5; // match next line
coord_t voltToAngle1[5] = 
{
    {0.0000, -15},
    {0.3000, -12},
    {2.5000,  10},
    {3.0000, 100},
    {3.5000, 240}
};

// Calibration: angle sensor 2 -- voltage to angle
const int numCalibPoints2 = 5; // match next line
coord_t voltToAngle2[5] = 
{
    {0.0000, -15},
    {0.3000, -12},
    {2.5000,  10},
    {3.0000, 100},
    {3.5000, 240}
};

const double toleranceHighRes = 1.0; // +/- angle (degrees)
const double toleranceLowRes = 5.0; // +/- angle (degrees)

// Additional angle (overshoot) due to relay turn off delay, etc.
const double stopTimeUp = 0.0; // angle (degrees)
const double stopTimeDn = 0.0; // angle (degrees)

// Time to wait after movement for relay to turn off and movement to settle
const unsigned long delayAfterMove = 200; // milliseconds

// Safety / Error limits
const double maxSetpoint = 60.0;          // angle (degrees)
const double minSetpoint = -11.0;         // angle (degrees)
const double maxParkAngle = 120.0;        // angle (degrees)
const double minParkAngle = 0.0;          // angle (degrees)
const double sensorsConvergeToleranceHighRes = 1.0; // angle (degrees)
const double sensorsConvergeToleranceLowRes = 10.0; // angle (degrees)
const double sensorTransistionAngle = 10.0; // angle (degrees) High-res to Low-res
const unsigned long maxRelayOnTime = 15000; // milliseconds
const unsigned long maxRelayCyclesPer  = 6;
const unsigned long maxRelayCyclesTime = 2000; // milliseconds

// Angle sensor error thresholds; Angle sensor valid range
// Don't use this to limit movement (use maxSetpoint or something else).
const double vMinValidSensor1 = 0.20; // minimum valid sensor1 voltage
const double vMaxValidSensor1 = 3.10; // maximum valid sensor1 voltage
const double vMinValidSensor2 = 0.20; // minimum valid sensor2 voltage
const double vMaxValidSensor2 = 3.10; // maximum valid sensor2 voltage

// Time to drive beyond end of angle sensor when "going off the map"
// to extend the range of the power wedge beyond what the sensor can read
const unsigned long overdriveUpTime  = 1000; // milliseconds
const unsigned long overdriveDnTime  = 1000; // milliseconds
// const unsigned long overdriveTimeout = 5000; // milliseconds

#endif
