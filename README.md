# BB-8 drive prototype

I'm using this hardware. There's a [Fritzing](https://www.fritzing.org) diagram in the repo that shows how it's all connected: BB-8-v1-Electronics.fzz. 

 - Arduino Mega 2560
 - RN42 BlueSMiRF Silver Bluetooth module (https://www.sparkfun.com/products/12577)
 - MPU-6050 Accelerator/gyro 6DOF IMU (http://playground.arduino.cc/Main/MPU-6050)
 - Pololu VNH5019 Dual Motor Driver (https://www.pololu.com/product/2507)

You need to install the following libraries in your `Arduino/libraries` folder:

 - https://github.com/pololu/dual-vnh5019-motor-shield
 - https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 - https://github.com/br3ttb/Arduino-PID-Library/

This will work with an Uno as I write this, but real soon now there won't be enough digital pins and something like a Mega will be required.
