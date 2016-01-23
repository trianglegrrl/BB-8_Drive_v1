/*
   Download the Dynamixel and updated SoftwareSerial libraries from here: http://sourceforge.net/projects/dynamixelforarduino/files/
   Just download the updated SoftwareSerial and stick it in `Arduino/libraries`. It will supercede the one in the app at
        /Applications/Arduino.app/Contents/Java/hardware/arduino/avr/libraries/SoftwareSerial
   The only reason you need the new one is that it makes some private methods public, and those methods are used in the Dynamixel lib.

   If you aren't using Dynamixels, you don't need the updated SoftwareSerial. The default one will do just fine.
 */
#include "DynamixelSerial1.h" // http://savageelectronics.blogspot.com.es/2011/08/actualizacion-biblioteca-dynamixel.html

#include "Wire.h"
#include "I2Cdev.h" // https://github.com/jrowberg/i2cdevlib/

#include "DualVNH5019MotorShield.h" // https://github.com/pololu/dual-vnh5019-motor-shield

#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.println(x)

  #include <MemoryFree.h>
  #define PRINT_FREE_MEMORY() DEBUG_PRINT(freeMemory());
#else
  #define DEBUG_PRINT(x)
  #define PRINT_FREE_MEMORY()
#endif

#define console Serial
#define bluetooth Serial2

/* =================================================================
 * Set up Pololu VNH5019 dual motor driver.

 Customize pinouts on VNH5019 and set up motor driver. I had to change INA1 from 2
 to 3 because the MPU6050 mpu needs the INTerrupt on pin 2 on Arduinos for DMP mode.

 For more information on customizing the leads on the VNH5019, see the Pololu Dual
 VNH5019 Motor Driver Shield User’s Guide, section "6.a. Remapping the Arduino ,
 Connections", at https://www.pololu.com/docs/0J49/6.a
*/

#define VNH5019_INA1 3 // PIN 2 IS THE DEFAULT
#define VNH5019_INB1 4
#define VNH5019_EN1DIAG1 6
#define VNH5019_CS1 A0
#define VNH5019_INA2 7
#define VNH5019_INB2 8
#define VNH5019_EN2DIAG2 12
#define VNH5019_CS2 A1

DualVNH5019MotorShield mainMotorDriver(VNH5019_INA1, VNH5019_INB1, VNH5019_EN1DIAG1, VNH5019_CS1,
                                       VNH5019_INA2, VNH5019_INB2, VNH5019_EN2DIAG2, VNH5019_CS2);

#define FWD_SPEED 200 // Default speed for rolling forward
#define BKWD_SPEED  -200 // Default speed for rolling backward
#define ROTATE_LEFT_SPEED 300 // Default speed for rotating right
#define ROTATE_RIGHT_SPEED -300 // Default speed for rotating right
#define BRAKE_SPEED 0 // Speed for stopping rotation

// RAMP_DELAY specifies how many milliseconds to wait after increasing the speed by
// one. This gives a smooth ramp-up for speed.
#define RAMP_DELAY 2

// Variables for tracking/reporting current motor speed.
int fwdBkwdSpeed = 0, // M1 on Pololu VNH5019 is both Fwd/Bkwd motors
    rotateSpeed = 0;  // M2 on Pololu VNH5019 is flywheel motor

/* =================================================================
 * Set up Dynamixel AX-12a Servos
 *
 * This was a little annoying. There's a good explanation at
 * http://savageelectronics.blogspot.ca/2011/01/arduino-y-dynamixel-ax-12.html,
 * which is really quite easy. There's a hand-drawn diagram that shows you how to connect
 * the 74LS241 buffer so the Arduino can talk to the servos, but I've also drawn that up
 * in the Fritzing diagram.
 *
 * The servo is on Serial1.
 *
 * I'm currently using two servos for the head pan/tilt.
 *
 * API docs are at http://austinlpalmer.com/Projects/Documentr/#/home
 */

#define DYNAMIXEL_FLOW_CONTROL_PIN 5
#define HEAD_PAN_SERVO_ID 1
#define HEAD_TILT_SERVO_ID 2

// Limits for range of tilt servo. This keeps BB-8 from trying to tilt his head
// too far, i.e., into the motors/axle
#define MIN_TILT_VALUE 160
#define MAX_TILT_VALUE 200

// Variables to track the details the Dynamixel servos give us.
int headPanTemp = 0,
    headPanVoltage = 0,
    headPanPosition = 0,
    headPanLoad = 0,
    headTiltTemp = 0,
    headTiltVoltage = 0,
    headTiltPosition = 0,
    headTiltLoad = 0;

unsigned long whenLastReceivedMessage = millis();
#define MINIMUM_MESSAGE_INTERVAL 100 // Minimum number of milliseconds to wait before requesting controller status

String requestString; // Requesting controller state
String controllerCommandString; // Response from controller

/* =================================================================
 * setup() - Run once at startup
 */
void setup()
{
  // Set up local serial output
  console.begin(115200);

  setupBlueSMiRF();

  Dynamixel.begin(1000000, DYNAMIXEL_FLOW_CONTROL_PIN);  // Inicialize the head servos at 1Mbps with the specified flow control pin

  console.println("Bweep bwoooo, I'm BB-8!");
}

/* =================================================================
 * loop() - Call over and over until reset
 */
void loop()
{
  checkForCommandAndDriveRobot();
}

/* =================================================================
 * getValue() - Get the specified value from the controller
 * command string -
 *
 * e.g. getStringValue()
 *
 * Snagged from http://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string
 */
String getValue(String data, char separator, int position)
{
  int found = 0;
  int strIndex[] = { 0, -1  };
  int maxIndex = data.length() - 1;

  for(int i=0; i<=maxIndex && found<=position; i++) {
    if(data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>position ? data.substring(strIndex[0], strIndex[1]) : "";
}

/* =================================================================
 * readDynamixelState() - Read the current states of the Dynamixel servos
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
 * setupBlueSMiRF() - Configure BlueSMiRF for communication
 */
void setupBlueSMiRF() {
  bluetooth.begin(115200);
}

/* =================================================================
 * requestControllerState() - Ask the controller for its current
 * state.
 */
void requestControllerState() {
  if (millis() < whenLastReceivedMessage + MINIMUM_MESSAGE_INTERVAL) {
    return;
  }

  readDynamixelState();

  requestString = "";

  requestString.concat("|");requestString.concat("?");
  requestString.concat("|");requestString.concat(666) ;
  requestString.concat("|");requestString.concat(headPanPosition);
  requestString.concat("|");requestString.concat(headTiltPosition);
  requestString.concat("|");requestString.concat(mainMotorDriver.getM1CurrentMilliamps());
  requestString.concat("|");requestString.concat(mainMotorDriver.getM2CurrentMilliamps());
  requestString.concat("|");requestString.concat(fwdBkwdSpeed);
  requestString.concat("|");requestString.concat(rotateSpeed);
  requestString.concat("|");


  // Send request to controller
  bluetooth.println(requestString);
}

/* =================================================================
 * readAndProcessControllerMessageIfAvailable() - Check the
 * specified stream a response from the controller, and if there is
 * one, process the data and react accordingly.
 */
void readAndProcessControllerMessageIfAvailable() {
  if (bluetooth.available()) {
    controllerCommandString = bluetooth.readStringUntil('\n');
    whenLastReceivedMessage = millis();
    processControllerMessage(controllerCommandString);
  }

  #ifdef DEBUG // If debugging, also receive commands from the console serial port
  if (console.available()) {
    controllerCommandString = console.readStringUntil('\n');

    processControllerMessage(controllerCommandString);
  }
  #endif
}

/* =================================================================
 * processControllerMessage() - Parse and process the received message.
 * Right now, all we're looking for is the command the controller is
 * sending. See handleCommand() for more details.
 */
void processControllerMessage(String controllerCommandString) {
  // Extract the "command" so we know what to do... and then do it.
  performRobotAction(getValue(controllerCommandString, '|', 1));
}

/* =================================================================
 * checkForCommandAndDriveRobot() - Check for incoming Bluetooth
 * commands and react accordingly.
 */
void checkForCommandAndDriveRobot() {
  // Ask the controller for its current state (and, by implication, the command)
  requestControllerState();

  // Process results from the Bluetooth stream
  readAndProcessControllerMessageIfAvailable();
}

/* =================================================================
 * performRobotAction() - Actuate robot as requested.
 */
void performRobotAction(String navCommand) {
  console.println(navCommand);

  // Languages that let you switch() strings are so much nicer than this....
  if(navCommand == "BF") { // Body forward
    driveBody(FWD_SPEED);
  } else if(navCommand == "BB") { // Body backward
    driveBody(BKWD_SPEED);
  } else if(navCommand == "BL") { // Body rotate left
    rotateBody(ROTATE_LEFT_SPEED);
  } else if(navCommand == "BR") { // Body rotate right
    rotateBody(ROTATE_RIGHT_SPEED);
  } else if(navCommand == "BS") { // Body stop
    stopBody();
  } else if(navCommand == "PHL") { // Pan head left
    rotateHead(20);
  } else if(navCommand == "PHR") { // Pan head right
    rotateHead(-20);
  } else if(navCommand == "THL") { // Tilt head left
    tiltHead(20);
  } else if(navCommand == "THR") { // Tilt head right
    tiltHead(-20);
  } else { // Default is to stop all motion
    stopBody();
  }
}

/* =================================================================
 * stopBody() - Stop moving the body completely.
 */
void stopBody() {
  driveBody(BRAKE_SPEED);
  rotateBody(BRAKE_SPEED);
}

/* =================================================================
 * driveBody() - Drive the body in the requested fwd/bkwd direction.
 * Speed ramps up from 0 to give a longer, smoother acceleration.
 */
void driveBody(int motorSpeed) {
  // First, stop the rotation motor
  mainMotorDriver.setM2Speed(BRAKE_SPEED);
  rotateSpeed = BRAKE_SPEED;

  // Ramp up driving speed
  for (int currentSpeed = 0; currentSpeed < motorSpeed; currentSpeed++) {
    mainMotorDriver.setM1Speed(currentSpeed);
    fwdBkwdSpeed = currentSpeed;

    delay(RAMP_DELAY);
  }
}

/* =================================================================
 * rotateBody() - Rotate the body in the requested L/R direction.
 */
void rotateBody(int motorSpeed) {
  // First, stop the driving motor
  mainMotorDriver.setM1Speed(BRAKE_SPEED);
  fwdBkwdSpeed = BRAKE_SPEED;

  mainMotorDriver.setM2Speed(motorSpeed);
  rotateSpeed = motorSpeed;
}

/* =================================================================
 * rotateHead() - Rotate the head the specified number of degrees.
 */
void rotateHead(int degrees) {
  // Change pan position relative to current position
  int requestedPosition = headPanPosition + degrees;

  Dynamixel.move(HEAD_PAN_SERVO_ID, requestedPosition);
}

/* =================================================================
 * tiltHead() - Rotate the head the specified number of degrees.
 * Includes a guard to make sure the head doesn't tilt too far and
 * fall off or bang into something important.
 */
void tiltHead(int degrees) {
  // Change tilt position relative to current position.
  int requestedPosition = headTiltPosition + degrees;

  // Guard to prevent tilt from going too far
  if (requestedPosition > MIN_TILT_VALUE && requestedPosition < MAX_TILT_VALUE) {
    Dynamixel.move(HEAD_TILT_SERVO_ID, requestedPosition);
  }
}
