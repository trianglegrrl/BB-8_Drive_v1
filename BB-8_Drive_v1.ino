#include "DualVNH5019MotorShield.h" // https://github.com/pololu/dual-vnh5019-motor-shield
#include "SoftwareSerial.h"
#include "Wire.h"
#include "I2Cdev.h" // https://github.com/jrowberg/i2cdevlib/
#include "MPU6050_6Axis_MotionApps20.h"// https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "PID_v1.h"

/*
 * Set up Bluetooth module
 */
int bluetoothTx = 11;  // TX-O pin of bluetooth mate, Arduino D11
int bluetoothRx = 13;  // RX-I pin of bluetooth mate, Arduino D13
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

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

DualVNH5019MotorShield mainMotorDriver(VNH5019_INA1, VNH5019_INB1, VNH5019_EN1DIAG1, VNH5019_CS1,
                                       VNH5019_INA2, VNH5019_INB2, VNH5019_EN2DIAG2, VNH5019_CS2);


int speedForRollingForward = 200, // Default speed for rolling forward
    speedForRollingBackward = -200, // Default speed for rolling backward
    speedForBrakingRoll = 0; // Speed for stopping roll
int speedForRotatingLeft = 300, // Default speed for rotating left
    speedForRotatingRight = -300, // Default speed for rotatingRight
    speedForBrakingRotation = 0; // Speed for stopping rotation

int fwdBkwdSpeed, // M1 on Pololu VNH5019 is both Fwd/Bkwd motors
    rotateSpeed = 0; // M2 on Pololu VNH5019 is flywheel motor


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
 * Configuration options
 */
char navCommand; // Variable that holds the navigation command read from Bluetooth

/* =================================================================
 * setupMPU() - Configure the mpu for DMP mode
 */
void setupMPU() {
  // Set up I2C bus for mpu
  Wire.begin();
  Wire.setClock(31000L);

  // Set up mpu
  mpu.initialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // Turn on the DMP, now that it's ready
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  dmpReady = true;
}

/* =================================================================
 * setupBluetoothMate() - Configure BluetoothMate
 */
void setupBluetoothMate() {
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
}

/* =================================================================
 * setup() - Run once at startup
 */
void setup()
{
  // Set up local serial output
  Serial.begin(115200);

  setupMPU();

  setupBluetoothMate();
}

void checkForCommandAndDriveRobot() {
  if(bluetooth.available())  // If the bluetooth sent any characters
  {
    navCommand =  bluetooth.read(); // Get them

    if (navCommand == -1) { return; } // If there's nothing, just GTFO

    switch ((char)navCommand) {
       case 'F':
         fwdBkwdSpeed = speedForRollingForward;
         rotateSpeed = speedForBrakingRotation;
         break;
       case 'B':
         fwdBkwdSpeed = speedForRollingBackward;
         rotateSpeed = speedForBrakingRotation;
         break;
       case 'L':
         fwdBkwdSpeed = speedForBrakingRoll;
         rotateSpeed = speedForRotatingLeft;
         break;
       case 'R':
         fwdBkwdSpeed = speedForBrakingRoll;
         rotateSpeed = speedForRotatingRight;
         break;
       default:
         fwdBkwdSpeed = speedForBrakingRoll;
         rotateSpeed = speedForBrakingRotation;
         break;
    }
    mainMotorDriver.setM1Speed(fwdBkwdSpeed);
    mainMotorDriver.setM2Speed(rotateSpeed);

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
}

/* =================================================================
 * processMPU() - Process fancy MPU shit
 *
 * Yanked right out of https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/Examples/MPU6050_DMP6/MPU6050_DMP6.ino
 */
void processMPU() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.print(q.z);
    Serial.print("\t");

    // display Euler angles in degrees
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[2] * 180/M_PI);
    Serial.print("\t");
    // display Euler angles in degrees
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180/M_PI);
    Serial.print("\t");
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.print(aaReal.z);
    Serial.print("\t");
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.print(aaWorld.z);
    Serial.println("");
  }
}

/* =================================================================
 * loop() - Call over and over until reset
 */
void loop()
{
  checkForCommandAndDriveRobot();

  processMPU();
}
