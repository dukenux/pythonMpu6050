import os
from distutils.core import setup, Extension
os.environ["CC"] = 'g++-4.9'
os.environ["CXX"] = 'g++-4.9'
setup(name='module_imu', version='1.0', ext_modules=[Extension('imu', sources = ['mod_imu.cpp','I2Cdev.cpp','MPU6050.cpp'],libraries = ["m","rt","pthread","wiringPi"],extra_compile_args = ["-Wextra","-O2","-std=c++0x"])])
