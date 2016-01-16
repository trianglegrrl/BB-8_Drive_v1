/*
   Download the Dynamixel and updated SoftwareSerial libraries from here: http://sourceforge.net/projects/dynamixelforarduino/files/
   Just download the updated SoftwareSerial and stick it in `Arduino/libraries`. It will supercede the one in the app at
        /Applications/Arduino.app/Contents/Java/hardware/arduino/avr/libraries/SoftwareSerial
   The only reason you need the new one is that it makes some private methods public, and those methods are used in the Dynamixel lib.

   If you aren't using Dynamixels, you don't need the updated SoftwareSerial. The default one will do just fine.
 */
#include "SoftwareSerial.h" // Note that this is the replaced version required by the DynamixelSoftSerial - see link below.
#include "DynamixelSerial1.h" // http://savageelectronics.blogspot.com.es/2011/08/actualizacion-biblioteca-dynamixel.html

#include "Wire.h"
#include "I2Cdev.h" // https://github.com/jrowberg/i2cdevlib/

#include "DualVNH5019MotorShield.h" // https://github.com/pololu/dual-vnh5019-motor-shield

#include "MPU6050_6Axis_MotionApps20.h" // https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "PID_v1.h" // https://github.com/br3ttb/Arduino-PID-Library/

#include "RobotController.h"

// ====================================================================================================================
// ******* Global variables
// ====================================================================================================================

RobotController robotController;

/* =================================================================
 * Set up Pololu VNH5019 dual motor driver.

 Customize pinouts on VNH5019 and set up motor driver. I had to change INA1 from 2
 to 3 because the MPU6050 mpu needs the INTerrupt on pin 2 on Arduinos for DMP mode.

 For more information on customizing the leads on the VNH5019, see the Pololu Dual
 VNH5019 Motor Driver Shield User’s Guide, section "6.a. Remapping the Arduino ,
 Connections", at https://www.pololu.com/docs/0J49/6.a
*/

//#define VNH5019_INA1 2 // THIS IS THE DEFAULT
// VNH5019_INA1 customized from 2 to 3 by AH
#define VNH5019_INA1 3
#define VNH5019_INB1 4
#define VNH5019_EN1DIAG1 6
#define VNH5019_CS1 A0
#define VNH5019_INA2 7
#define VNH5019_INB2 8
#define VNH5019_EN2DIAG2 12
#define VNH5019_CS2 A1

#define DRIVE_MOTOR 1
#define ROTATE_MOTOR 2

#define DRIVE_MOTOR_RAMP_TIME 1000
#define ROTATE_MOTOR_RAMP_TIME 1000

DualVNH5019MotorShield mainMotorDriver(VNH5019_INA1, VNH5019_INB1, VNH5019_EN1DIAG1, VNH5019_CS1,
                                       VNH5019_INA2, VNH5019_INB2, VNH5019_EN2DIAG2, VNH5019_CS2);

int speedForRollingForward = 200, // Default speed for rolling forward
    speedForRollingBackward = -200, // Default speed for rolling backward
    speedForBrakingRoll = 0; // Speed for stopping roll
int speedForRotatingLeft = 300, // Default speed for rotating left
    speedForRotatingRight = -300, // Default speed for rotatingRight
    speedForBrakingRotation = 0; // Speed for stopping rotation

int driveMotorCurrentSpeed = 0;
int rotateMotorCurrentSpeed = 0;

int fwdBkwdSpeed, // M1 on Pololu VNH5019 is both Fwd/Bkwd motors
    rotateSpeed = 0; // M2 on Pololu VNH5019 is flywheel motor

// Counters for actions
int counter1Start = millis();
int counter1TargetElapsed = millis();
int counter1ActuallyElapsed = millis();

/* =================================================================
 * Set up gy521 MPU6050 mpu
 */
MPU6050 mpu;

// Config from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/Examples/MPU6050_DMP6/MPU6050_DMP6.ino
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


/* =================================================================
 * Set up Dynamixel AX-12a Servos

 This was a little annoying. There's a good explanation at http://savageelectronics.blogspot.ca/2011/01/arduino-y-dynamixel-ax-12.html,
 which is really quite easy. There's a hand-drawn diagram that shows you how to connect the 74LS241 buffer so the Arduino can
 talk to the servos, but I've also drawn that up in the Fritzing diagram.

 The Dynamixel library requires a custom SoftwareSerial library, which you can download from the link at the top of this file.

 I'm currently using two servos for the head pan/tilt.

 API docs are at http://austinlpalmer.com/Projects/Documentr/#/home
*/

#define DYNAMIXEL_FLOW_CONTROL_PIN 5
#define HEAD_PAN_SERVO_ID 1
#define HEAD_TILT_SERVO_ID 2

// Variables to track the details the Dynamixel servos give us.
int headPanTemp = 0,
    headPanVoltage = 0,
    headPanPosition = 0,
    headPanLoad = 0,
    headTiltTemp = 0,
    headTiltVoltage = 0,
    headTiltPosition = 0,
    headTiltLoad = 0;

// ====================================================================================================================
// ******* Functions
// ====================================================================================================================

/* =================================================================
 * readDynamixelState() - Read the current states of the Dynamixel servos
 *
 */
void readDynamixelState() {
  headPanTemp = Dynamixel.readTemperature(HEAD_PAN_SERVO_ID);
  headTiltTemp = Dynamixel.readTemperature(HEAD_TILT_SERVO_ID);
  headPanVoltage = Dynamixel.readVoltage(HEAD_PAN_SERVO_ID);
  headTiltVoltage = Dynamixel.readVoltage(HEAD_TILT_SERVO_ID);
  headPanPosition = Dynamixel.readPosition(HEAD_PAN_SERVO_ID);
  headTiltPosition = Dynamixel.readPosition(HEAD_TILT_SERVO_ID);
  headPanLoad = Dynamixel.readLoad(HEAD_PAN_SERVO_ID);
  headTiltLoad = Dynamixel.readLoad(HEAD_TILT_SERVO_ID);
}

/* =================================================================
 * setupMPU() - Configure the mpu for DMP mode
 *
 * Copied from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/Examples/MPU6050_DMP6/MPU6050_DMP6.ino
 */
void setupMPU() {
  // Set up I2C bus for mpu
  Wire.begin();
  Wire.setClock(31000L); // Slow down the refresh rate to allow it to do other robot-y things like move

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void checkForCommandAndDriveRobot() {
/* =================================================================
 * checkForCommandAndDriveRobot() - Check for incoming control
 * commands and act accordingly.
*/
  Serial.println("Checking for command");
  robotController.requestControllerState();
  Serial.println("Checked for command");
//  performRobotAction();
}

/* =====================================================================
 * void rampMotor(int motorNumber, int speedToRampTo, int overHowManyMilliseconds)
 *
 * Returns the speed we ramped to.
 */
void rampMotor(int motorNumber, int speedToRampTo, int overHowManyMilliseconds) {
  int workingSpeed = 0;

  int startTime = millis(); // This is the time the function starts
  int currentTime = millis(); // We'll use this later to check the currentTime
  int endTime = startTime + overHowManyMilliseconds; // The time we should stop

  if (motorNumber == DRIVE_MOTOR) {
    while(currentTime < endTime) {
      currentTime = millis();

      workingSpeed = map(currentTime, startTime, endTime, driveMotorCurrentSpeed, speedToRampTo);

      mainMotorDriver.setM1Speed(workingSpeed);
    }

    // If it takes too long to go through the loop, at least make sure we end up at the right speed
    mainMotorDriver.setM1Speed(speedToRampTo);

    driveMotorCurrentSpeed = speedToRampTo;
  } else if (motorNumber == ROTATE_MOTOR) {
    while(currentTime < endTime) {
      currentTime = millis();

      workingSpeed = map(currentTime, startTime, endTime, rotateMotorCurrentSpeed, speedToRampTo);

      mainMotorDriver.setM2Speed(workingSpeed);
    }

    // If it takes too long to go through the loop, at least make sure we end up at the right speed
    mainMotorDriver.setM2Speed(speedToRampTo);

    rotateMotorCurrentSpeed = speedToRampTo;
  }
}

void driveForward() {
  rampMotor(DRIVE_MOTOR, speedForRollingForward, DRIVE_MOTOR_RAMP_TIME);
}

void driveBackward() {
  rampMotor(DRIVE_MOTOR, speedForRollingBackward, DRIVE_MOTOR_RAMP_TIME);
}

void rotateLeft() {
  rampMotor(ROTATE_MOTOR, speedForRotatingLeft, DRIVE_MOTOR_RAMP_TIME);
}

void rotateRight() {
  rampMotor(ROTATE_MOTOR, speedForRotatingRight, DRIVE_MOTOR_RAMP_TIME);
}

void stopDriving() {
  rampMotor(DRIVE_MOTOR, speedForBrakingRoll, DRIVE_MOTOR_RAMP_TIME);
}

void stopRotating() {
  rampMotor(ROTATE_MOTOR, speedForBrakingRotation, DRIVE_MOTOR_RAMP_TIME);
}

void stopAllMotors() {
  stopDriving();

  stopRotating();
}
/* =================================================================
 * performRobotAction() - Actuate robot as requested.
 */
void performRobotAction(char navCommand) {
  switch (navCommand) {
     case 'F': driveForward(); break;
     case 'B': driveBackward(); break;
     case 'L': rotateLeft(); break;
     case 'R': rotateRight(); break;
     case 'I': processMPU(); break;
     default : stopAllMotors(); break;
  }

  if ((char)navCommand == 'S') { return; } // Don't print anything; just return if it's a stop
  Serial.print(", Fwd/bkwd current: ");
  Serial.print(mainMotorDriver.getM1CurrentMilliamps());
  Serial.print(", Rotate current: ");
  Serial.print(mainMotorDriver.getM2CurrentMilliamps());
  Serial.print(", Fwd/bkwd speed: ");
  Serial.print(fwdBkwdSpeed);
  Serial.print(", rotate speed: ");
  Serial.print(rotateSpeed);
  Serial.print(", command: ");
  Serial.print((char)navCommand);
  Serial.println("");
}

/* =================================================================
 * processMPU() - Process fancy MPU shit
 *
 * Yanked right out of https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/Examples/MPU6050_DMP6/MPU6050_DMP6.ino
 */
void processMPU() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  String BTOutput = "";

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here

      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // display Euler angles in degrees
    mpu.dmpGetEuler(euler, &q);

    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    Serial.print((int)(euler[0] * 180/M_PI));
    Serial.print("|");
    Serial.print((int)(euler[1] * 180/M_PI));
    Serial.print("|");
    Serial.println((int)(euler[2] * 180/M_PI));

    mpu.resetFIFO();
  }
}

/* =================================================================
 * setup() - Run once at startup
 */
void setup()
{
  // Set up local serial output
  Serial.begin(115200);
  Serial.println("Starting");
//  setupMPU();
//
//  Dynamixel.begin(1000000, DYNAMIXEL_FLOW_CONTROL_PIN);  // Inicialize the head servos at 1Mbps with the specified flow control pin

  Serial.println("BB-8 Drive v1");
}

/* =================================================================
 * loop() - Call over and over until reset
 */
void loop()
{
  checkForCommandAndDriveRobot();
}
