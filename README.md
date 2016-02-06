# pythonMpu6050

Python wrapper for the MPU650 C++ class provided by Jeff Rowberg

After cloning from git:

To compile
sudo python setup.py install

To test from pyt hon:
sudo ./testMpu6050.py 
(Connect your MPU6050 before! Tested on Raspberry PI 1/2 and Odroid C1)

Here is a short summary API:

#!/usr/bin/python
import imu
imu.ypr() # read yaw pitch roll and store the result in a floating point 3D array
imu.acc() # read X Y Z accelerations and store the result in a floating point 3D array
imu.gyro() # read angular rotation speed and store the result in a floating point 3D array
imu.quaternion()# read Omega X Y Z accelerations and store the result in a floating point 4D array
imu .euler() # read euler angle Psi Teta Phi and store the result in a floating point 3D array

