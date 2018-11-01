import numpy as np
import spidev
import time
import argparse 
import sys
import navio.mpu9250
import navio.util
#from  EKF import *
import EKF as ekf

navio.util.check_apm()

imu = navio.mpu9250.MPU9250()
imu2 = navio.lsm9ds1.LSM9DS1()
imu_count=0

ekf.initialized = False
ekf.imu_initialized = False
ekf.magnetic_initialized = False

if imu.testConnection():
    print "Connection established: True"
else:
	print "Connection established: False"
	
imu.initialize()
#ekf = EKF()
i = 1
A = np.array([0,0,0])
B = np.array([0,0,0])
bA = np.array([0.0,0.0,0.0])
bb = np.array([0.0,0.0,0.0])
while True:
	t=time.time()
	m9a, m9g, m9m = imu.getMotion9()
	#m9a2,m9g2,m9m2 = imu2.getMotion9()
	acc, gyro = m9a, m9g
	#acc2, gyro2 = m9a2, m9g2
	print "acc raw data: ", acc, "|| gyro raw data:", gyro
	
	if i <= 20:
		A = A + acc
		B = B + gyro

	else : 
		if i ==21:
			bA = A/20.0
			bb = B/20.0
			bA = bA -[0,0,9.8]
		print bA,bb
		ekf.predict(gyro, acc, t,bA,bb)
		imu_count+=1
		#if imu_count%10==0:
		ekf.update(acc,t)
	#print("position: ", ekf.x[4:7], "velocity", ekf.x[7:10])
	print "---------------------------------------------------------------------------------"
	i = i + 1
	time.sleep(0.5)
