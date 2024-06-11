#include <Arduino.h>
#include <EasyTransfer.h>
#include <PID_v1.h>
#include <pwmWrite.h>

#include "ImuProMini.h"
#include "MotorPWM.h"
#include "Offsets.h"
#include "PS4Controller.h"
#include "Pins.h"
#include "Reverse.h"
#include "Structs.h"
// #include "Animation.h"
// #include "AnimationRunner.h"
// #include "AnalogInHandler.h"
// #include "ButtonHandler.h"
// #include "EaseApplicator.h"

#include "I2Cdev.h"
#include "Libraries/NaigonAnimations/src/Animation.h"
#include "Libraries/NaigonAnimations/src/AnimationRunner.h"
#include "Libraries/NaigonIO/src/AnalogInHandler.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "src/Libraries/NaigonIO/src/ButtonHandler.h"
#include "Libraries/NaigonUtil/src/EaseApplicator.h"

using Naigon::BB_8::ImuProMini;
using Naigon::BB_8::MotorPWM;
using Naigon::BB_8::Offsets;
Offsets offsets;

// I/O Wrappers
using Naigon::IO::AnalogInHandler;
// using Naigon::IO::ButtonState;
// using Naigon::IO::ButtonHandler;

// Ease Applicators
using Naigon::Util::FunctionEaseApplicator;
using Naigon::Util::FunctionEaseApplicatorType;
using Naigon::Util::IEaseApplicator;
using Naigon::Util::LinearEaseApplicator;
using Naigon::Util::ScalingEaseApplicator;

//==================================================================================
/*                                   EspSerial2                                   */
//==================================================================================
#define RXD2 18
#define TXD2 19
HardwareSerial EspSerial2(2);

EasyTransfer SendESP;
EasyTransfer RcvESP;

struct SEND_DATA_STRUCTURE_ESP2 {
    int8_t play_num = 0;
    uint8_t ps4_bat = 0;
    bool ps4_chrg = false;
    bool showHeadBat = false;
};
SEND_DATA_STRUCTURE_ESP2 sendESPData;

struct RECEIVE_DATA_STRUCTURE_ESP2 {
    int16_t distance;
    uint16_t pot_s2s;
    uint16_t pot_dome;
};
RECEIVE_DATA_STRUCTURE_ESP2 rcvESPData;

int distance = 0;
int pot_s2s = 0;
int pot_dome = 0;

unsigned long lastDataSendTime = 0;
//==================================================================================

//==================================================================================
/*                                   Bluetooth                                    */
//==================================================================================
#define BT_MAC "a0:78:17:83:fa:b9"

int8_t psLX = 0;   // left joystick X
int8_t psLY = 0;   // left joystick Y
int8_t psRX = 0;   // right joystick X
int8_t psRY = 0;   // right joystick Y
int8_t psL1 = 0;   // left top button
int16_t psL2 = 0;  // left bottom button
int8_t psR1 = 0;   // right top button
int16_t psR2 = 0;  // right bottom button

bool psTriangle = false;
bool psCircle = false;
bool psSquare = false;
bool psCross = false;

bool psUp = false;
bool psDown = false;
bool psLeft = false;
bool psRight = false;
bool psUpLeft = false;
bool psUpRight = false;
bool psDownLeft = false;
bool psDownRight = false;

bool psOptions = false;
bool psShare = false;
bool psTouchpad = false;

Pwm pwm = Pwm();

DriveState drive;
bool idleMode = false;
unsigned long lastIdleModeTime = 0;
unsigned long closeDistanceTime = 0;
bool soundPlayed = false;
bool obstacleSoundPlayed = false;

bool needsDrive = false;
//==================================================================================

//==================================================================================
/*                                    Handlers                                    */
//==================================================================================
AnalogInHandler driveStickHandlerSlow(-128, 127, reverseDrive, -MaxDriveSlow, MaxDriveSlow, 2.0f);
AnalogInHandler driveStickHandlerMed(-128, 127, reverseDrive, -MaxDriveMed, MaxDriveMed, 2.0f);
AnalogInHandler driveStickHandlerFast(-128, 127, reverseDrive, -MaxDriveFast, MaxDriveFast, 2.0f);
AnalogInHandler sideToSideStickHandler(-128, 127, reverseS2S, -MaxSideToSide, MaxSideToSide, 2.0f);
AnalogInHandler domeTiltStickHandlerFR(-128, 127, reverseDomeTiltFR, -MaxDomeTiltY, MaxDomeTiltY, 2.0f);
AnalogInHandler domeTiltStickHandlerLR(-128, 127, reverseDomeTiltLR, -MaxDomeTiltX, MaxDomeTiltX, 2.0f);
AnalogInHandler domeSpinStickHandler(-255, 255, reverseDomeSpin, -MaxDomeSpin, MaxDomeSpin, 15.0f);
AnalogInHandler domeSpinAutoStickHandler(-255, 255, reverseDomeSpin, -MaxDomeSpinAuto, MaxDomeSpinAuto, 15.0f);
AnalogInHandler domeServoStickHandler(-128, 127, reverseDomeSpin, -MaxDomeSpinServo, MaxDomeSpinServo, 15.0f);
AnalogInHandler domeServoAutoStickHandler(-128, 127, reverseDomeSpin, -MaxDomeServoAuto, MaxDomeServoAuto, 15.0f);
AnalogInHandler flywheelStickHandler(-127, 127, reverseFlywheel, -MaxFlywheelDrive, MaxFlywheelDrive, 10.0f);

AnalogInHandler sideToSidePotHandler(0, 26850, reverseS2SPot, -MaxS2SPot, MaxS2SPot, 0.0f);
AnalogInHandler domeSpinPotHandler(0, 26850, reverseDomeSpinPot, -MaxDomeSpinPot, MaxDomeSpinPot, 0.0f);

AnalogInHandler *driveStickPtr = &driveStickHandlerSlow;
AnalogInHandler *sideToSideStickPtr = &sideToSideStickHandler;
AnalogInHandler *domeTiltStickPtr = &domeTiltStickHandlerFR;
AnalogInHandler *domeTiltStickLRPtr = &domeTiltStickHandlerLR;
AnalogInHandler *domeSpinStickPtr = &domeSpinStickHandler;
AnalogInHandler *flywheelStickPtr = &flywheelStickHandler;

FunctionEaseApplicator driveApplicatorSlow(0.0, 20.0, 0.2, FunctionEaseApplicatorType::Quadratic);
FunctionEaseApplicator driveApplicatorMed(0.0, 28.0, 0.2, FunctionEaseApplicatorType::Quadratic);
FunctionEaseApplicator driveApplicatorHigh(0.0, 38.0, 0.2, FunctionEaseApplicatorType::Quadratic);
FunctionEaseApplicator driveApplicatorWiggle(0.0, 5.0, 0.1, FunctionEaseApplicatorType::Quadratic);
// Naigon - Ease Applicator: for the drive this pointer will always be the one in use.
FunctionEaseApplicator *driveApplicatorPtr = &driveApplicatorSlow;
LinearEaseApplicator sideToSideEase(0.0, easeS2S);  //, easeMsS2SA, easeMsS2SD, 20);
ScalingEaseApplicator domeServoEase(0.0, easeDomeServo, easeDomeServoMsA, easeDomeServoMsD, 20);
LinearEaseApplicator domeSpinEase(0.0, easeDome);
LinearEaseApplicator flywheelEase(0.0, easeFlywheel);

ScalingEaseApplicator domeTiltEaseFR(0.0, easeDomeFR, easeTiltMK3MsA, 300, 15);
ScalingEaseApplicator domeTiltEaseLR(0.0, easeDomeLR, easeTiltMK3MsA, 300, 15);

MotorPWM drivePWM(drivePWM1, drivePWM2, 0, 2);
MotorPWM sideToSidePWM(s2sPWM1, s2sPWM2, MaxSideToSide, 1);
MotorPWM domeSpinPWM(domeSpinPWM1, domeSpinPWM2, 0, 20);
MotorPWM domeServoPWM(domeSpinPWM1, domeSpinPWM2, 0, 4);
MotorPWM flywheelPWM(flywheelSpinPWM1, flywheelSpinPWM2, 0, 35);

PIDVals s2sTiltVals;
PID PID1(&s2sTiltVals.input, &s2sTiltVals.output, &s2sTiltVals.setpoint, Pk1, Ik1, Dk1, DIRECT);

PIDVals s2sServoVals;
PID PID2(&s2sServoVals.input, &s2sServoVals.output, &s2sServoVals.setpoint, Pk2, Ik2, Dk2, DIRECT);  // PID Setup - S2S stability

PIDVals driveVals;
PID PID3(&driveVals.input, &driveVals.output, &driveVals.setpoint, Pk3, Ik3, Dk3, DIRECT);  // Main drive motor

PIDVals domeServoVals;
PID PID4(&domeServoVals.input, &domeServoVals.output, &domeServoVals.setpoint, Pk4, Ik4, Dk4, DIRECT);

// // Global for debugging; these act like the PID output values.
int servoLeft, servoRight;
// // Global for debugging; this acts like the PID output for dome spin.
int currentDomeSpeed;
// // Global for debugging; this acts like the PID output for the flywheel spin.
int flywheelRotation;
//==================================================================================

//==================================================================================
/*                                     MPU6050                                    */
//==================================================================================
MPU6050 mpu;
ImuProMini imu;

float mpuLoop;
float mpuPitch;
float mpuRoll;

uint8_t bodyStatus = 0;
uint8_t bodyMode = 0;
uint8_t bodyDirection = 0;

bool buttonHold = false;
bool doneOnce;

bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
//==================================================================================

//==================================================================================
/*                                  PS4 Controller                                */
//==================================================================================
static unsigned long controllerLedUpdatePeriod = 60000;
static unsigned long ontrollerDischargeUpdtaePeriod = 30000;
#define LONG_PRESS_TIME 3000

unsigned long longPressStart;

uint8_t currentRemoteMode;
uint8_t lastRemoteMode;

static unsigned long lastControllerBlink = 0;
static unsigned long lastControllerBlinkOn = 0;
static unsigned long lastControllerBlinkOff = 0;
static unsigned long lastSentToController = 0;

uint8_t blinkCount = 0;
bool blinkController = false;
bool ledOn = false;
bool sendExplicitly = false;
//==================================================================================

void gamepadHandler();
void setupMPU();
void setupPID();
void setupDrives();
void setRemoteMode();
void dmpDataReady();
void updateInputHandlers();
void updateWireless();
void updateBodyMode();
void movement();
void mainDrive(IEaseApplicator *easeApplicatorPtr);
void sideTilt(IEaseApplicator *easeApplicatorPtr);
void domeTiltMK3(IEaseApplicator *easeApplicatorFRPtr, IEaseApplicator *easeApplicatorLRPtr);
void domeSpin(IEaseApplicator *easeApplicatorPtr);
void domeSpinServo(IEaseApplicator *easeApplicatorPtr, bool isCentering);
void flywheelSpin(IEaseApplicator *easeApplicatorPtr);
void playMP3(int number);
void sendData();
void resetOffsets();
void disableMotors();
void readMPU();
void writeMotorPwm(MotorPWM &motorPwm, int output, int input, bool requireBT);

void setOffsetsAndSaveToEEPROM();
void sendControllerState();
void controllerDischargeLed();

void gamepadHandler() {
    psLX = PS4.LStickX();
    psLY = PS4.LStickY();
    psRX = PS4.RStickX();
    psRY = PS4.RStickY();
    psR2 = PS4.R2Value();
    psL1 = 0;

    psCircle = PS4.Circle();
    psTriangle = PS4.Triangle();
    psSquare = PS4.Square();
    psCross = PS4.Cross();
    psUp = PS4.Up();
    psDown = PS4.Down();
    psLeft = PS4.Left();
    psRight = PS4.Right();

    psUpLeft = PS4.UpLeft();
    psUpRight = PS4.UpRight();
    psDownLeft = PS4.DownLeft();
    psDownRight = PS4.DownRight();
    psOptions = PS4.Options();
    psShare = PS4.Share();
    psTouchpad = PS4.Touchpad();

    if (PS4.L2()) {
        psR2 = -PS4.L2Value();
    }

    if (PS4.L1()) {
        psL1 = -127;
    }

    if (PS4.R1()) {
        psL1 = 127;
    }
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200, SERIAL_8N1);
    EspSerial2.begin(460800, SERIAL_8N1, RXD2, TXD2);
    SendESP.begin(details(sendESPData), &EspSerial2);
    RcvESP.begin(details(rcvESPData), &EspSerial2);

    PS4.begin(BT_MAC);
    PS4.attach(gamepadHandler);

    setupMPU();
    setupPID();
    setupDrives();

    offsets.LoadOffsetsFromMemory();

    drive.PreviousNormalMode = BodyMode::Slow;
    drive.PreviousAnimationMode = BodyMode::AutomatedServo;
    drive.HasDomeMoved = false;

    randomSeed(analogRead(0));
}

void setupMPU() {
    Wire.begin();
    mpu.initialize();
    Serial.println(F("Testing MPU6050 connection..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connected successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        attachInterrupt(mpuInterruptPin, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERRORS
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void setupPID() {
    PID1.SetMode(AUTOMATIC);  // PID Setup -  S2S SERVO
    PID1.SetOutputLimits(-255, 255);
    PID1.SetSampleTime(15);

    PID2.SetMode(AUTOMATIC);  // PID Setup -  S2S Stability
    PID2.SetOutputLimits(-255, 255);
    PID2.SetSampleTime(15);

    PID3.SetMode(AUTOMATIC);  // PID Setup - main drive
    PID3.SetOutputLimits(-255, 255);
    PID3.SetSampleTime(15);

    PID4.SetMode(AUTOMATIC);
    PID4.SetOutputLimits(-255, 255);  // PID Setup - dome spin 'servo'
    PID4.SetSampleTime(15);
}

void setupDrives() {
    pinMode(enablePin, OUTPUT);
    pinMode(enablePinS2S, OUTPUT);
    pinMode(enablePinFW, OUTPUT);

    pinMode(drivePWM1, OUTPUT);
    pinMode(drivePWM2, OUTPUT);
    pinMode(s2sPWM1, OUTPUT);
    pinMode(s2sPWM2, OUTPUT);
    pinMode(domeSpinPWM1, OUTPUT);
    pinMode(domeSpinPWM2, OUTPUT);
    pinMode(flywheelSpinPWM1, OUTPUT);
    pinMode(flywheelSpinPWM2, OUTPUT);
    pinMode(leftDomeTiltServo, OUTPUT);
    pinMode(rightDomeTiltServo, OUTPUT);

    digitalWrite(enablePinS2S, HIGH);
    digitalWrite(enablePinFW, HIGH);

    pwm.writeServo(leftDomeTiltServo, 115);
    pwm.writeServo(rightDomeTiltServo, 65);
}

void updateInputHandlers() {
    psRX = psRX == -1 ? 0 : psRX;
    psRY = psRY == -1 ? 0 : psRY;
    psLX = psLX == -1 ? 0 : psLX;
    psLY = psLY == -1 ? 0 : psLY;

    domeSpinStickPtr = &domeServoStickHandler;

    if (currentRemoteMode == remoteMode::RemoteModeDrive) {
        if (psLY == 0) {
            obstacleSoundPlayed = false;
        }

        if (distance == 0 || distance > 100) {
            driveStickPtr->UpdateState(psLY);
            obstacleSoundPlayed = false;
        } else if (distance > 0 && distance < 100 && (psLY != 0)) {
            driveStickPtr->UpdateState(0);

            if (!obstacleSoundPlayed) {
                obstacleSoundPlayed = true;
                playMP3(random(SOUNDS_NUMBER + 1));
                SendESP.sendData();
            }
        }

        sideToSideStickPtr->UpdateState(-psRX);
        flywheelStickPtr->UpdateState(-psL1);
        domeSpinStickPtr->UpdateState(psR2);
    }
    if (currentRemoteMode == remoteMode::RemoteModeHead) {
        domeTiltStickPtr->UpdateState(psLY);
        domeTiltStickLRPtr->UpdateState(psLX);
        domeSpinStickPtr->UpdateState(psR2);
        driveStickPtr->UpdateState(0);

        if (psSquare) {
            if (millis() - lastIdleModeTime >= 1000) {
                lastIdleModeTime = millis();
                idleMode = !idleMode;
            }
        }
    }
    if (currentRemoteMode == remoteMode::RemoteModeManual) {
        driveStickPtr->UpdateState(psLY);
        // drivePWM.WritePWM(psLY, 0);
        domeTiltStickPtr->UpdateState(psRY);
        domeTiltStickLRPtr->UpdateState(psRX);
        sideToSidePWM.WritePWM(psLX, 0);
        flywheelPWM.WritePWM(-psL1, 0);
        domeSpinPWM.WritePWM(psR2 / 4, 0);
    }

    if (currentRemoteMode == remoteMode::RemoteModeService) {
        disableMotors();
    }

    sideToSidePotHandler.UpdateState(pot_s2s);
    domeSpinPotHandler.UpdateState(pot_dome);
}

void updateWireless() {
    drive.IsConnected = (currentRemoteMode == remoteMode::RemoteModeDrive) || (currentRemoteMode == remoteMode::RemoteModeHead) || (currentRemoteMode == remoteMode::RemoteModeManual);
}

void reverseDirection() {
    // Naigon - Safe Joystick Button Toggle.
    //
    // I've had some pretty catastropic issues where I accidentally hit reverse when driving and didn't realize it.
    // This is because the reverse is pressing the drive stick.
    //
    // To prevent that, I'm only going to accept the input when joysticks are below a threshold.
    // if (button5Handler.GetState() == ButtonState::Pressed
    //     && !driveStickPtr->HasMovement()
    //     && !sideToSideStickPtr->HasMovement()
    //     && !domeTiltStickPtr->HasMovement()
    //     && !domeTiltStickLRPtr->HasMovement()
    //     && !domeSpinStickPtr->HasMovement()
    //     && !flywheelStickPtr->HasMovement())
    // {
    //     sendToRemote.bodyDirection = sendToRemote.bodyDirection == Direction::Forward
    //         ? Direction::Reverse
    //         : Direction::Forward;
    // }
}

void updateBodyMode() {
    driveApplicatorPtr = &driveApplicatorSlow;
    driveStickPtr = &driveStickHandlerMed;
}

//=====================================================================================================================
// Motor movement
//=====================================================================================================================
void movement() {
    updateBodyMode();

    if (currentRemoteMode == remoteMode::RemoteModeDrive) {
        sideTilt(&sideToSideEase);
        mainDrive(driveApplicatorPtr);
        flywheelSpin(&flywheelEase);
        domeTiltMK3(&domeTiltEaseFR, &domeTiltEaseLR);
        domeSpinServo(&domeServoEase, true);
    }

    if (currentRemoteMode == remoteMode::RemoteModeHead) {
        domeTiltMK3(&domeTiltEaseFR, &domeTiltEaseLR);
        domeSpin(&domeSpinEase);

        mainDrive(driveApplicatorPtr);
        flywheelPWM.WriteZeros();
    }

    if (currentRemoteMode == remoteMode::RemoteModeManual) {
        mainDrive(driveApplicatorPtr);
    }
    //  domeSpinServo(&domeServoEase, true);
    //  } else {
    //   turnOffAllTheThings(true /*disableDrive*/);
    //  }

    // DomeMode currentDomeMode =
    //   animationRunner.IsRunning() && animation.AnimationDomeMode != DomeMode::UnspecifiedDomeSpin
    //     ? animation.AnimationDomeMode
    //     : drive.CurrentDomeMode;

    // if ((currentDomeMode == DomeMode::ServoMode || drive.IsDomeCentering)
    //     && !drive.AutoDisable
    //     && recFromRemote.motorEnable == 0) {
    //   domeSpinServo(&domeServoEase, drive.IsDomeCentering);
    // } else if (currentDomeMode == DomeMode::FullSpinMode || drive.AutoDisable || recFromRemote.motorEnable == 1) {
    //   domeSpin(&domeSpinEase);
    // }

    // // Naigon - Animations
    // // Stop forcing the head servo mode now that it is back into position.
    // if (drive.IsDomeCentering && abs(domeServoVals.output) < 2) {
    //   drive.IsDomeCentering = false;
    // }
}

// ------------------------------------------------------------------------------------
// Main drive
// ------------------------------------------------------------------------------------
void mainDrive(IEaseApplicator *easeApplicatorPtr) {
    // Naigon - Stationary/Wiggle Mode
    // When in wiggle/stationary mode, don't use the joystick to move at all.
    int joystickDrive = (int)driveStickPtr->GetMappedValue();

    driveVals.setpoint = constrain(
        easeApplicatorPtr->ComputeValueForCurrentIteration(joystickDrive),
        -MaxDrive,
        MaxDrive);

    driveVals.input = (imu.Pitch() + offsets.PitchOffset());  // - domeOffset;
    // domeTiltOffset used to keep the ball from rolling when dome is tilted front/back

    PID3.Compute();
    writeMotorPwm(drivePWM, driveVals.output, 0 /*input*/, true /*requireBT*/);
}

// ------------------------------------------------------------------------------------
// Side to Side
// ------------------------------------------------------------------------------------
//
// s2s left joystick goes from 0(LEFT) to 512(RIGHT)
// The IMU roll should go DOWN as it tilts to the right, and UP as it tilts to the left
// The side to side pot should go UP as the ball tilts left, and LOW as it tilts right
//
void sideTilt(IEaseApplicator *easeApplicatorPtr) {
    int joystickS2S = (int)sideToSideStickPtr->GetMappedValue();

    // Setpoint will increase/decrease by S2SEase each time the code runs until it matches the joystick. This slows the side to side movement.
    s2sServoVals.setpoint = constrain(
        easeApplicatorPtr->ComputeValueForCurrentIteration(joystickS2S),
        -MaxSideToSide,
        MaxSideToSide);

    int S2Spot = (int)sideToSidePotHandler.GetMappedValue();

    s2sServoVals.input = imu.Roll() + offsets.RollOffset();

    PID2.Compute();  // PID2 is used to control the 'servo' control of the side to side movement.

    s2sTiltVals.input = S2Spot + offsets.SideToSidePotOffset();

    s2sTiltVals.setpoint = map(
        constrain(s2sServoVals.output, -MaxSideToSide, MaxSideToSide),
        -MaxSideToSide,
        MaxSideToSide,
        MaxSideToSide,
        -MaxSideToSide);

    PID1.Compute();  // PID1 is for side to side stabilization

    writeMotorPwm(sideToSidePWM, s2sTiltVals.output, s2sTiltVals.input, true /*requireBT*/);
}

// ------------------------------------------------------------------------------------
// Dome tilt
// ------------------------------------------------------------------------------------
int joy2Ya, joy2XLowOffset, joy2XHighOffset;
int joy2XLowOffsetA, joy2XHighOffsetA;
int joyX = 0;
int joyY = 0;

void domeTiltMK3(IEaseApplicator *easeApplicatorFRPtr, IEaseApplicator *easeApplicatorLRPtr) {
    // #ifdef HeadTiltStabilization
    // Naigon - Head Tilt Stabilization
    // Calculate the pitch to input into the head tilt input in order to keep it level.
    // Naigon - TODO: once the ease applicator is created, use it here to increment to pitch adjust.
    int pitchAdjust = (imu.FilteredPitch() + offsets.PitchOffset()) * HeadTiltPitchAndRollProportion;
    int rollAdjust = (imu.FilteredRoll() + offsets.RollOffset()) * HeadTiltPitchAndRollProportion;

    // Naigon - use the new reverse values to flip the sign if needed
    if (currentRemoteMode != remoteMode::RemoteModeHead) {
        pitchAdjust = reverseAutoDomeY ? pitchAdjust * -1 : pitchAdjust;
        rollAdjust = reverseAutoDomeX ? rollAdjust * -1 : rollAdjust;
    } else {
        pitchAdjust = 0;
        rollAdjust = 0;
    }

    if (idleMode && currentRemoteMode == remoteMode::RemoteModeHead) {
        joyY = 0;
        joyX = 0;

        if (distance > 0 && distance <= 100) {
            if (!soundPlayed || millis() - closeDistanceTime >= 10000) {
                soundPlayed = true;
                closeDistanceTime = millis();
                playMP3(random(SOUNDS_NUMBER + 1));
                SendESP.sendData();
            }

            joyY = -10;

        } else {
            soundPlayed = false;
        }
    } else {
        joyY = constrain(
            domeTiltStickPtr->GetMappedValue() + pitchAdjust,
            -MaxDomeTiltY,
            MaxDomeTiltY);

        joyX = constrain(
            domeTiltStickLRPtr->GetMappedValue() + rollAdjust,
            -MaxDomeTiltX,
            MaxDomeTiltX);
    }
    //
    // Naigon - MK3 Head Tilt
    // This is loosely based on Joe's existing code, but from testing if the initial run the Y is not moved a miniscule
    // amount, the left and right will blow by their end stops. This is the code the ensures the Y moves a bit the
    // first time.
    //
    if (!drive.HasDomeMoved && joyY <= 2.0 && joyY >= -2.0) {
        joyY = 2.0;
    } else if (joyY > 2.0 || joyY < -2.0) {
        drive.HasDomeMoved = true;
    }

    int joy2YEaseMap = easeApplicatorFRPtr->ComputeValueForCurrentIteration(joyY);
    int joy2XEaseMap = easeApplicatorLRPtr->ComputeValueForCurrentIteration(joyX);

    if (joy2YEaseMap < 0) {
        joy2Ya = map(joy2YEaseMap, -20, 0, 70, 0);
        joy2XLowOffset = map(joy2Ya, 1, 70, -15, -50);
        joy2XHighOffset = map(joy2Ya, 1, 70, 30, 20);

    } else if (joy2YEaseMap > 0) {
        joy2Ya = map(joy2YEaseMap, 0, 24, 0, -80);
        joy2XLowOffset = map(joy2Ya, -1, -80, -15, 10);
        joy2XHighOffset = map(joy2Ya, -1, -80, 30, 90);
    } else {
        joy2Ya = 0;
    }

    if (joy2XEaseMap > 0) {
        joy2XLowOffsetA = map(joy2XEaseMap, 0, 18, 0, joy2XLowOffset);
        joy2XHighOffsetA = map(joy2XEaseMap, 0, 18, 0, joy2XHighOffset);
        servoLeft = joy2Ya + joy2XHighOffsetA;
        servoRight = joy2Ya + joy2XLowOffsetA;
    } else if (joy2XEaseMap < 0) {
        joy2XLowOffsetA = map(joy2XEaseMap, -18, 0, joy2XLowOffset, 0);
        joy2XHighOffsetA = map(joy2XEaseMap, -18, 0, joy2XHighOffset, 0);
        servoRight = joy2Ya + joy2XHighOffsetA;
        servoLeft = joy2Ya + joy2XLowOffsetA;
    } else {
        joy2XHighOffsetA = 0;
        joy2XLowOffsetA = 0;
        servoRight = joy2Ya;
        servoLeft = joy2Ya;
    }

    pwm.writeServo(leftDomeTiltServo, constrain(map(servoLeft + 25, -90, 90, 0, 180), 0, 180));
    pwm.writeServo(rightDomeTiltServo, constrain(map(servoRight + 25, -90, 90, 180, 0), 0, 180));
}

// ------------------------------------------------------------------------------------
// Dome spin - Full Spin
// ------------------------------------------------------------------------------------
void domeSpin(IEaseApplicator *easeApplicatorPtr) {
    int domeRotation = (int)domeSpinStickPtr->GetMappedValue();

    currentDomeSpeed = constrain(
        easeApplicatorPtr->ComputeValueForCurrentIteration(domeRotation),
        -255,
        255);

    // Joe has always allowed the dome to spin regardless of whether the motors were enabled or not, which I like.
    writeMotorPwm(domeSpinPWM, currentDomeSpeed, 0 /*input*/, false /*requireBT*/);
}

// ------------------------------------------------------------------------------------
// Dome spin - Servo
// ------------------------------------------------------------------------------------
void domeSpinServo(IEaseApplicator *easeApplicatorPtr, bool isCentering) {
    int ch4Servo = (int)domeSpinStickPtr->GetMappedValue();

    domeServoVals.input = (int)domeSpinPotHandler.GetMappedValue() + offsets.DomeSpinPotOffset();

    if (domeServoVals.input <= -180) {
        domeServoVals.input += 360;
    } else if (domeServoVals.input >= 180) {
        domeServoVals.input -= 360;
    }

    domeServoVals.setpoint = constrain(
        easeApplicatorPtr->ComputeValueForCurrentIteration(ch4Servo),
        -MaxDomeSpinServo,
        MaxDomeSpinServo);

    PID4.Compute();

    // Naigon - Animations
    // To prevent the head from spinning really fast at the end of an animation when re-centering, scale the pwm speed.
    domeServoVals.output = isCentering ? domeServoVals.output * 3 / 4 : domeServoVals.output;

    writeMotorPwm(domeServoPWM, domeServoVals.output, 0, false /*requireBT*/);
}

// ------------------------------------------------------------------------------------
// Flywheel spin
// ------------------------------------------------------------------------------------
void flywheelSpin(IEaseApplicator *easeApplicatorPtr) {
    int flywheelStick = (int)flywheelStickPtr->GetMappedValue();

    flywheelRotation = constrain(
        easeApplicatorPtr->ComputeValueForCurrentIteration(flywheelStick),
        -255,
        255);

    writeMotorPwm(flywheelPWM, flywheelRotation, 0 /*input*/, false /*requireBT*/);
}
// ------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------------------------
void loop() {
    readMPU();

    if (RcvESP.receiveData()) {
        distance = rcvESPData.distance;
        pot_s2s = rcvESPData.pot_s2s;
        pot_dome = rcvESPData.pot_dome;
    }

    setRemoteMode();

    updateWireless();

    updateInputHandlers();

    resetOffsets();

    sendData();

    imu.UpdateIteration(mpuPitch, mpuRoll, mpuLoop);

    setOffsetsAndSaveToEEPROM();

    controllerDischargeLed();

    movement();
}

void playMP3(int number) {
    sendESPData.play_num = number;
}

void sendData() {
    if (millis() - lastDataSendTime >= sendDelay) {
        lastDataSendTime = millis();
        sendControllerState();
        SendESP.sendData();
        sendESPData.play_num = 0;
    }
}

void resetOffsets() {
    if (currentRemoteMode == remoteMode::RemoteModeService && psCircle && !buttonHold) {
        longPressStart = millis();
        buttonHold = true;
    }

    if (!(currentRemoteMode == remoteMode::RemoteModeService && psCircle)) {
        buttonHold = false;
        doneOnce = false;
    }

    if (millis() - longPressStart >= LONG_PRESS_TIME && currentRemoteMode == remoteMode::RemoteModeService && psCircle && buttonHold && !doneOnce) {
        doneOnce = true;

        playMP3(50);
        SendESP.sendData();

        offsets.ClearOffsets();
        offsets.UpdateOffsets(
            mpuPitch,
            mpuRoll,
            (int)sideToSidePotHandler.GetMappedValue(),
            (int)domeSpinPotHandler.GetMappedValue());

        offsets.WriteOffsets();
        delay(1500);
        offsets.LoadOffsetsFromMemory();
        playMP3(51);
    }
}

void disableMotors() {
    drivePWM.WriteZeros();
    domeSpinPWM.WriteZeros();
    flywheelPWM.WriteZeros();
    sideToSidePWM.WriteZeros();
    domeServoPWM.WriteZeros();

    s2sServoVals.input = 0;
    s2sServoVals.setpoint = 0;
    s2sServoVals.output = 0;
}

void setRemoteMode() {
    if (PS4.isConnected()) {
        lastRemoteMode = currentRemoteMode;

        currentRemoteMode = psTouchpad && !psOptions ? remoteMode::RemoteModeManual : currentRemoteMode;
        currentRemoteMode = !psTouchpad && psOptions ? remoteMode::RemoteModeDrive : currentRemoteMode;
        currentRemoteMode = psTouchpad && psOptions ? remoteMode::RemoteModeService : currentRemoteMode;
        currentRemoteMode = !psTouchpad && !psOptions ? remoteMode::RemoteModeHead : currentRemoteMode;

        if (currentRemoteMode != lastRemoteMode) {
            sendExplicitly = true;
        }

        if (!blinkController) {
            if (currentRemoteMode == remoteMode::RemoteModeManual) {
                PS4.setLed(0, 0, 255);
            }
            if (currentRemoteMode == remoteMode::RemoteModeDrive) {
                PS4.setLed(0, 255, 0);
            }
            if (currentRemoteMode == remoteMode::RemoteModeService) {
                PS4.setLed(255, 0, 0);
            }
            if (currentRemoteMode == remoteMode::RemoteModeHead) {
                PS4.setLed(220, 60, 240);
            }

            if (millis() - lastSentToController >= controllerLedUpdatePeriod) {
                lastSentToController = millis();
                PS4.sendToController();
            }

            if (sendExplicitly) {
                PS4.sendToController();
                sendExplicitly = false;
            }
        }
    } else {
        currentRemoteMode = remoteMode::RemoteModeNone;
    }
}

void readMPU() {
    if (!dmpReady)
        return;

    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        mpuPitch = (ypr[1] * 180 / M_PI);
        mpuRoll = (ypr[2] * 180 / M_PI);
    }

    mpuLoop++;

    if (mpuLoop >= 1000) {
        mpuLoop = 1;
    }
}

void writeMotorPwm(MotorPWM &motorPwm, int output, int input, bool requireBT, bool requireNeedsDrive) {
    if (requireBT == true && !drive.IsConnected || requireNeedsDrive == true && !needsDrive) {
        motorPwm.WriteZeros();
    } else {
        motorPwm.WritePWM(output, input);
    }
}

// ------------------------------------------------------------------------------------
// EEPROM
// ------------------------------------------------------------------------------------
void setOffsetsAndSaveToEEPROM() {
    if (!offsets.AreValuesLoaded()) {
        if (mpuPitch != 0.0f && mpuRoll != 0.0f && domeSpinPotHandler.GetMappedValue() != 0) {
            offsets.UpdateOffsets(
                mpuPitch,
                mpuRoll,
                (int)sideToSidePotHandler.GetMappedValue(),
                (int)domeSpinPotHandler.GetMappedValue());
        }
    }

    offsets.WriteOffsets();
}

void sendControllerState() {
    sendESPData.ps4_chrg = PS4.Charging();
    sendESPData.ps4_bat = PS4.Battery();

    if (psTriangle) {
        playMP3(random(SOUNDS_NUMBER + 1));
    }

    if (currentRemoteMode == remoteMode::RemoteModeService) {
        sendESPData.showHeadBat = psSquare;
    }
}

void controllerDischargeLed() {
    if (PS4.isConnected()) {
        if (PS4.Battery() <= 2 && !PS4.Charging() && !blinkController) {
            if (millis() - lastControllerBlink >= 30000) {
                blinkController = true;
                ledOn = true;
                return;
            }
        }

        if (millis() - lastControllerBlinkOn >= 200 && blinkController && ledOn) {
            lastControllerBlinkOff = millis();
            ledOn = false;
            PS4.setLed(0, 0, 0);
            PS4.sendToController();
            blinkCount++;
        }

        if (millis() - lastControllerBlinkOff >= 200 && blinkController && !ledOn) {
            if (blinkCount >= 4) {
                blinkCount = 0;
                blinkController = false;
                lastControllerBlink = millis();
                return;
            }

            lastControllerBlinkOn = millis();
            ledOn = true;

            if (PS4.Battery() == 2) {
                PS4.setLed(255, 70, 0);
            }

            if (PS4.Battery() < 2) {
                PS4.setLed(255, 0, 0);
            }
        }
        PS4.sendToController();
    }
}