# BB-8 drive prototype

Includes a Fritzing diagram

You need to install the following libraries in your `Arduino/libraries` folder:

 - https://github.com/pololu/dual-vnh5019-motor-shield
 - https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 - https://github.com/br3ttb/Arduino-PID-Library/
 - The custom SoftwareSerial at http://sourceforge.net/projects/dynamixelforarduino/files/?source=navbar (required for Dynamixel)
 - One of the Dynamixel serial libraries at http://savageelectronics.blogspot.com.es/2011/08/actualizacion-biblioteca-dynamixel.html

This assumes:

 - Arduino Mega 2560
 - Qty 2, gearmotors for drive
 - Qty 1, gearmotor for rotation disk
 - RN42 BlueSMiRF Silver Bluetooth module (https://www.sparkfun.com/products/12577)
 - MPU-6050 Accelerator/gyro 6DOF IMU (http://playground.arduino.cc/Main/MPU-6050)
 - Pololu VNH5019 Dual Motor Driver (https://www.pololu.com/product/2507)
 - 74LS241 bridge
 - Qty 2, AX-12a servos