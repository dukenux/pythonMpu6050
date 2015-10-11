#!/usr/bin/python
import imu
from time import sleep
yawPitchRoll = []
quat = [] 
acclin = []
e = []
q = []

for i in range(0,20):
	yawPitchRoll = imu.ypr()
	print "yaw="+str(yawPitchRoll[0])+" Pitch="+str(yawPitchRoll[1])+" Roll="+str(yawPitchRoll[2])
	sleep(0.5)

for i in range(0,20):
	acclin = imu.acc()
	print "accX="+str(acclin[0])+" accY="+str(acclin[1])+" accZ="+str(acclin[2])
	sleep(0.5)


for i in range(0,20):
	g = imu.gyro()
	print "gyroX="+str(g[0])+" gyroY="+str(g[1])+" gyroZ="+str(g[2])
	sleep(0.5)

for i in range(0,20):
	q = imu.quaternion()
	print "qW="+str(q[0])+" qX="+str(q[1])+" qY="+str(q[2])+" qZ="+str(q[3])
	sleep(0.5)
	

for i in range(0,20):
	e = imu.euler()
	print "Psi="+str(e[0])+" Theta="+str(e[1])+" Phi="+str(e[2])
	sleep(0.5)
	
