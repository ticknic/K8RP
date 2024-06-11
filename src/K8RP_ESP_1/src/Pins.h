#ifndef __Pins_H_
#define __Pins_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pin Mapping
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define enablePinS2S 25        // Pin that provides power to motor driver enable pins
#define enablePin 23           // Pin that provides power to motor driver enable pins
#define enablePinFW 14         // Pin that provides power to motor driver enable pins
#define drivePWM1 4            // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
#define drivePWM2 5            // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
#define s2sPWM1 27             // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
#define s2sPWM2 26             // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
#define domeSpinPWM1 15        // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
#define domeSpinPWM2 23        // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
#define flywheelSpinPWM1 13    // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
#define flywheelSpinPWM2 12    // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
#define leftDomeTiltServo 32   // Signal pin for the left dome tilt servo
#define rightDomeTiltServo 33  // Signal pin for the right dome tilt servo
#define mpuInterruptPin 35     // MPU6050 interrupt pin

#endif