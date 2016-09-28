# ROS-node-to-use-VL53L0X-in-python
This repository is ROS node to use the VL53L0X written in C++(not python).

!!! N O W  C O D I N G !!!


//python code for ROS node 
C++ code for ROS node

in reference to the pololu's repository https://github.com/pololu/vl53l0x-arduino (Use code:single.ino,VL53L0X.cpp,VL53L0X.h) 

//use "smbus" module instead of "Wire.h"(Arduino I2C Library)
use "i2c-dev.h" header instead of "Wire.h"(Arduino I2C Library)

//directrory "src" is only test
directory "script" is only test

//"vl53l0x_test.cpp" is Not used for ROS node


//"single.py" in "script" is (test) node

"Single.cpp" in "src" is main code for node
