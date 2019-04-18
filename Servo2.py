import sys
import time
import math

import numpy as np
import navio.pwm
import navio.util
import datetime
import curses
import navio.rcinput


from collections import deque
import zmq
import json

from TrajectoryGenerator import TrajectoryGenerator

import matplotlib.pyplot as plt
import matplotlib.patches as patches

navio.util.check_apm()


rcin = navio.rcinput.RCInput()

#PWM SETTING######################################################
PWM_OUTPUT_0  = 0
PWM_OUTPUT_1  = 1
PWM_OUTPUT_2  = 2
PWM_OUTPUT_3  = 3
SERVO_MIN = 1.072 #ms
SERVO_MAX = 1.935 #ms  571.4 - 800 
SERVO_NOM = 1.500 #ms
SERVO_NOM_1 = 1.350
SERVO_STOP = 0.000

pwm0 = navio.pwm.PWM(PWM_OUTPUT_0)
pwm1 = navio.pwm.PWM(PWM_OUTPUT_1)
pwm2 = navio.pwm.PWM(PWM_OUTPUT_2)
pwm3 = navio.pwm.PWM(PWM_OUTPUT_3)


pwm0.initialize()
pwm1.initialize()
pwm2.initialize()
pwm3.initialize()

pwm0.set_period(50)
pwm1.set_period(50)
pwm2.set_period(50)
pwm3.set_period(50)

pwm0.enable()
pwm1.enable()
pwm2.enable()
pwm3.enable()
##################################################################


#reading OptiTrack information via Internet SETTING #############################################################
savet = (0.0,0.0,0.0)
savet2 = (0.0,0.0,0.0)
savepos = deque(maxlen=2)
saveangu = deque(maxlen=2)
sub_port = 5556
context = zmq.Context()
#connect to socket we subscrib
socket_sub = context.socket(zmq.SUB)
#socket_sub.connect("tcp://localhost:5556")
socket_sub.connect("tcp://192.168.1.9:%d" %sub_port)
#socket_sub.setsockopt(zmq.SUBSCRIBE, b"")
socket_sub.setsockopt(zmq.SUBSCRIBE,b'')
#socket_sub.setsockopt(zmq.CONFLATE, 1)
def recv_array(socket, flags=0, copy=True, track=False):
	"""recv a numpy array"""
	md = socket.recv_json(flags=flags)
	msg = socket.recv(flags=flags, copy=copy, track=track)
	buf = buffer(msg)
	A = np.frombuffer(buf, dtype=md['dtype'])
	return A.reshape(md['shape'])
####################################################################################################################



def loop_for(seconds, func, *args):
	endTime = datetime.datetime.now() + datetime.timedelta(seconds=seconds)

	while True:
		if datetime.datetime.now() >= endTime:
			break
		func(*args)



def calibration_ESC():

	print("set all pwm to 0")
	pwm0.set_duty_cycle(SERVO_STOP)
	pwm1.set_duty_cycle(SERVO_STOP)
	pwm2.set_duty_cycle(SERVO_STOP)
	pwm3.set_duty_cycle(SERVO_STOP)


	print("SERVO_MAX")
	loop_for(2, pwm0.set_duty_cycle, SERVO_MAX)
	print("SERVO_MIN")
	loop_for(4, pwm0.set_duty_cycle, SERVO_MIN)
	print("SERVO_MIN_AGAIN")
	loop_for(6, pwm0.set_duty_cycle, SERVO_MIN)


	print("SERVO_MAX")
	loop_for(2, pwm1.set_duty_cycle, SERVO_MAX)
	print("SERVO_MIN")
	loop_for(4, pwm1.set_duty_cycle, SERVO_MIN)
	print("SERVO_MIN_AGAIN")
	loop_for(6, pwm1.set_duty_cycle, SERVO_MIN)

	print("SERVO_MAX")
	loop_for(2, pwm2.set_duty_cycle, SERVO_MAX)
	print("SERVO_MIN")
	loop_for(4, pwm2.set_duty_cycle, SERVO_MIN)
	print("SERVO_MIN_AGAIN")
	loop_for(6, pwm2.set_duty_cycle, SERVO_MIN)

	print("SERVO_MAX")
	loop_for(2, pwm3.set_duty_cycle, SERVO_MAX)
	print("SERVO_MIN")
	loop_for(4, pwm3.set_duty_cycle, SERVO_MIN)
	print("SERVO_MIN_AGAIN")
	loop_for(6, pwm3.set_duty_cycle, SERVO_MIN)


def test_throttle():
	wait_until_motor_is_ready()
	i = 0.000
	Loop = True
	while Loop:

		print("i: ",i)
		SERVO_INPUT = 1.250 + i
		loop_for(0.01, pwm0.set_duty_cycle, SERVO_INPUT)
		loop_for(0.01, pwm1.set_duty_cycle, SERVO_INPUT)
		loop_for(0.01, pwm2.set_duty_cycle, SERVO_INPUT)
		loop_for(0.01, pwm3.set_duty_cycle, SERVO_INPUT)

		i = i + 0.050 

		if i >= 0.500:
			Loop = False
		time.sleep(0.5)

	loop_for(0.1, pwm0.set_duty_cycle, SERVO_MIN)
	loop_for(0.1, pwm1.set_duty_cycle, SERVO_MIN)
	loop_for(0.1, pwm2.set_duty_cycle, SERVO_MIN)
	loop_for(0.1, pwm3.set_duty_cycle, SERVO_MIN)

def test_motor():
	wait_until_motor_is_ready()
	while True:
		loop_for(1, pwm0.set_duty_cycle, 1.420)
		loop_for(1, pwm0.set_duty_cycle, 1.380)




def test_throttle_via_RC():
	wait_until_motor_is_ready()
	Loop = True
	while Loop:
		period = float(rcin.read(2))/1000.000
		print("rcinput_value: ", period)
		loop_for(0.01, pwm0.set_duty_cycle, period)
		loop_for(0.01, pwm1.set_duty_cycle, period)
		loop_for(0.01, pwm2.set_duty_cycle, period)
		loop_for(0.01, pwm3.set_duty_cycle, period)



def wait_until_motor_is_ready():
	print("I am in the wait_until_motor_is_ready()")
	loop_for(0.01, pwm0.set_duty_cycle, SERVO_MIN)
	loop_for(0.01, pwm1.set_duty_cycle, SERVO_MIN)
	loop_for(0.01, pwm2.set_duty_cycle, SERVO_MIN)
	loop_for(0.01, pwm3.set_duty_cycle, SERVO_MIN)

	time.sleep(5)


def reading_positional_info():
	start = time.time()
	#print('reading info.')
	contents  =  recv_array(socket_sub,copy=False)
	#############################
	# Rotation Matrix Readjusting the coordinates
	Rotation_mat=np.array([[1,0,0],[0,0,-1],[0,1,0]])
	#############################
	#############################
	# Reading 3 axis positions
	position = contents[0:3]
	position = np.dot(Rotation_mat.T, position)
	savet = (position[0],position[1],position[2])
	#print("position:  ", position)
	#############################
	#############################
	# Reading orientation in quaternions
	a = contents[3]
	b = contents[4]
	c = contents[5]
	d = contents[6]
	orientation = np.array([a,b,c,d])
	orientation[0] *= -1.0
	#############################
	#print("orientation: ", orientation)

	#############################
	#Get Euler angles
	Euler = quaternion2euler(orientation)
	Euler = np.dot(Rotation_mat.T, Euler)
	Euler = mod360(Euler)
	savet2 = (Euler[0],Euler[1],Euler[2])
	#############################
	#print("Angle180", Euler[0]*180/np.pi,Euler[1]*180/np.pi,Euler[2]*180/np.pi)

	savepos.appendleft(savet)
	saveangu.appendleft(savet2)

	elapsed = (time.time() - start)# record the time to calculate the velocities.
	#print("time",elapsed)
	vx = (savepos[0][0] - savepos[-1][0])/elapsed*1.0
	vy = (savepos[0][1] - savepos[-1][1])/elapsed*1.0
	vz = (savepos[0][2] - savepos[-1][2])/elapsed*1.0
	vel = (vx,vy,vz)
	ax = (saveangu[0][0] - saveangu[-1][0])/elapsed*1.0
	ay = (saveangu[0][1] - saveangu[-1][1])/elapsed*1.0
	az = (saveangu[0][2] - saveangu[-1][2])/elapsed*1.0
	Angu = (ax,ay,az)
	#print("Angu speed",Angu)# currently in optitrack coordinates
	#print("Angu speed",ax*180/np.pi,ay*180/np.pi,az*180/np.pi)# currently in optitrack coordinates
	#print("speed",vel)# currently in optitrack coordinates
	#print ("---------------------------------------------------------------------------------")
	return position, Euler, vel, Angu

def mod360(Euler):
	for i in range(3):
		if Euler[i] > 0:
			while Euler[i] >= 360:
				Euler[i] -= 360
		elif Euler[i] < 0:
			while Euler[i] <= -360:
				Euler[i] += 360
	return Euler

def quaternion2euler(q):
	m = np.zeros((3,3))
	a = q[0]
	b = q[1]
	c = q[2]
	d = q[3]
	m = np.array([[a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d+a*c)],
		[2*(b*c+a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b)],
		[2*(b*d - a*c), 2*(c*d+a*b), a*a-b*b - c*c + d*d]])

	r = math.atan2(m[2, 1], m[2, 2])
	p = math.asin(-1*m[2, 0])
	y = math.atan2(m[1, 0], m[0, 0])
	rpy=np.array([r, p, y])
	return rpy

	

#calibration_ESC()
#test_throttle_via_RC()

#test_throttle()

'''

we need first give some desire values of positional info. 
this part should also be coordinate with reading_positional_info function which is used to read info from optitrack

the equations of torque and errors i.e. PID control command 
relationship between propellers rotation speed and thrust


the final control command from this py file should always be pwm signals in unit 'ms'

several parameters to be determined e.x. the moment inertia. some coeffients of certain relationships 

'''
'''
#parameterss for quadcopter control: 
mass
moment of inertia
length of frame

#some coefficients
k relationship between thrust and pwms
b the relationship between torque and summation of raotation speed
'''

# Simulation parameters
g = 9.81
m = 2.0 #kg
L = 0.28 # m 
Ixx = 3.503e-2#3.613e-2 
Iyy = 3.503e-2#3.613e-2 
Izz = 6.658e-2#6.707e-2 
Inertia = np.array([[Ixx,0,0],[0,Iyy,0],[0,0,Izz]])
T = 5 # time for complete trajectory

# Model coefficients with Forces and Torques
k = 3.876e-5
b = 7.233e-7        
gamma = b/k

# Motor coefficients
wb = 137.74#144.37 # rad/s
Cr = 615.69#615.1 # rad/s

Max_PWM_Hz = 800  # Hz
Min_PWM_Hz = 571.4 # Hz

pwm_thres_max = SERVO_MAX * 0.95
pwm_thres_min = SERVO_MIN + (SERVO_MAX - SERVO_MIN) * 0.3

cof = 0.5*np.sqrt(2)
#Motor_mix = np.linalg.inv(np.array([[1,1,1,1],[-cof * L, cof * L, cof * L, -cof * L],
#                        [-cof * L, cof * L, -cof * L, cof * L],[gamma, gamma, -gamma, -gamma]]))


Euler_error_store = []



#PD control parameters#######################################
K_p_roll = 40 #100
K_p_pitch = 50 #70
K_p_yaw = 5

K_d_roll = 0.5 #0.1
K_d_pitch = 0.5 #0.1
K_d_yaw = 0.5  # 0.1

K_p_x = 1.5 #1
K_p_y = 1.5 #1
K_p_z = 15 #1

K_d_x = 0.025 #0.01
K_d_y = 0.025 #0.01
K_d_z = 0.025 #0.01
#############################################################

K_p = np.array([[K_p_x,0,0],[0,K_p_y,0],[0,0,K_p_z]])
K_d = np.array([[K_d_x,0,0],[0,K_d_y,0],[0,0,K_d_z]])

K_p_Pose = np.array([[K_p_roll,0,0],[0,K_p_pitch,0],[0,0,K_p_yaw]])
K_d_Pose = np.array([[K_d_roll,0,0],[0,K_d_pitch,0],[0,0,K_d_yaw]])


def position_control(desired_pos_info, pos, vel, loop): #input should be generated trajectory
	#calculate errors of position, velocity and acceleration.
	desired_pos, desired_vel, acc_desired, desired_yaw = desired_pos_info
	pos_error = desired_pos - pos
	vel_error = desired_vel - vel 
	#acc_desired come from derivative of trajectory

	#Store position errors
	#pos_error_store.append(pos_error)
	#vel_error_store.append(vel_error)

	#calculate acc command
	acc_command = acc_desired + np.dot(K_d, vel_error.T) + np.dot(K_p, pos_error.T)

	#calculate the total thrust for 4 motors
	u1 = m*g + m*acc_command[2] # scalar i.e. summation of 4 thrust

	# derive desired roll and desired pitch
	desired_roll = 1/g * (acc_command[0] * np.sin(desired_yaw) - acc_command[1] * np.cos(desired_yaw))
	desired_pitch = 1/g * (acc_command[0] * np.cos(desired_yaw) - acc_command[1] * np.sin(desired_yaw))

	desired_pose = np.array([desired_roll, desired_pitch, desired_yaw])

	#judge = (np.abs(pos_error) <= 0.02)
	#if judge[0] == True and judge[1] == True and judge[2] == True:
	#	loop = False
	judge = np.abs(pos_error[2]) <= 0.02
	if judge == True:
		loop = False



	#des_Euler_store.append(desired_pose)

	return u1, desired_pose, loop


def attitude_control(Euler, A_vel, desired_pose): #the inputs are desired Euler angle and angular rate and feedback pose info
	#specify desired Angular velocity
	#Note for hover or near hover state, the desired angular velocity for roll and pitch should be 0
	desired_roll_vel = 0
	desired_pitch_vel = 0
	desired_yaw_vel = 0 # angular velocity for yaw is not necessarily 0
	desired_A_vel = np.array([desired_roll_vel, desired_pitch_vel, desired_yaw_vel])    

	#calculate errors of Euler angle and angular rate
	A_vel_error = desired_A_vel - A_vel
	Euler_error = desired_pose - Euler

	#print("Euler_error: ", Euler_error*180/np.pi)
	#print("Euler_error: ", Euler_error)

	#Store pose errors in lists.
	Euler_error_store.append(Euler_error)
	#A_vel_error_store.append(A_vel_error)

	#implement control for attitude
	u2 = np.zeros(3)
	#print("K_p_Pose * Euler_error", np.dot(K_p_Pose, Euler_error))
	#print("Inertia * K_p_Pose * Euler_error: ", np.dot(Inertia, (np.dot(K_p_Pose, Euler_error))))
	u2 = np.dot(Inertia, (np.dot(K_p_Pose, Euler_error) + np.dot(K_d_Pose, A_vel_error)))
	return u2


def motor_mix_controller(u1, u2):
	# upper right is the #1 motor and bottom left is #2.
	# upper left is the #3 motor and bottom right is #4.
	# The total torque for ROLL is calculated by 1/sqrt(2)* (F2 + F3 - F1 - F4)
	# the total torque for PITCH is calculated by 1/sqrt(2)* (F2 + F4 - F1 - F3)
	# the total torque for YAW is calculated by M1 + M2 - M3 -M4
	# the total thrust is F1 + F2 + F3 + F4

	# The transform matrix between [u1,u2] and [F1, F2, F3, F4] is :
	# thrust for each motor is
	# the Force vector is force for each motor.
	roll_torque, pitch_torque, yaw_torque = u2

	if roll_torque < 0:
		motor_mix_roll = np.array([-cof * L, cof * L, cof * L, -cof * L])
	elif roll_torque >= 0: 
		motor_mix_roll = np.array([cof * L, -cof * L, -cof * L, cof * L])

	if pitch_torque < 0:
		motor_mix_pitch = np.array([cof * L, -cof * L, cof * L, -cof * L])
	elif pitch_torque >= 0:
		motor_mix_pitch = np.array([-cof * L, cof * L, -cof * L, cof * L])

	if yaw_torque < 0:
		motor_mix_yaw = np.array([gamma, gamma, -gamma, -gamma])
	elif yaw_torque >= 0:
		motor_mix_yaw = np.array([-gamma, -gamma, gamma, gamma])

	Motor_mix = np.linalg.inv(np.array([[1,1,1,1], motor_mix_roll,
										motor_mix_pitch, motor_mix_yaw]))

	u2 = np.abs(u2)
	Force = np.dot(Motor_mix, np.array([u1, u2[0],u2[1],u2[2]]))
	for index in range(4):
		if Force[index] < m*g/4:
			Force[index] = m*g/4
	#Force = np.maximum(Force, m*g/10)
	#print("u1: ", u1)
	#print("u2: ", u2)
	#print("Motor_mix: ", Motor_mix)
	#print("Force: ", Force)

	# transform force of each motor into rotation speed :
	omega = np.sqrt(1/k * Force)

	#print("omega: ", omega)

	# dutycycle and rotation speed
	dutycycle = (omega - wb) / Cr

	#print("dutycycle: ", dutycycle)

	#control_PWM = 1000/(dutycycle * (Max_PWM_Hz - Min_PWM_Hz) + Min_PWM_Hz)
	control_PWM = dutycycle * (SERVO_MAX - SERVO_MIN) + SERVO_MIN

	# transform rotation speed into PWM duty cycles : 
		# note that PWM duty cycles may need saturation.
	for i in range(4):
		if control_PWM[i] > pwm_thres_max:
			control_PWM[i] = pwm_thres_max
		#elif control_PWM[i] < pwm_thres_min:
		#    control_PWM[i] = pwm_thres_min
	#print("control_PWM", control_PWM)

	return control_PWM

def drive_motor(control_PWM):
	# this function is mainly used to pass duty cycle into navio hardware to drive motor.
	#print("I am in")
	#control_PWM = np.int0(control_PWM*1000)/1000.0
	#for k in range(4):
		#control_PWM[k] = ("%.3f" % control_PWM[k])
		#control_PWM[k] = format(control_PWM[k], '.4g')
	#print(control_PWM)
	loop_for(0.0001, pwm0.set_duty_cycle, control_PWM[0])
	loop_for(0.0001, pwm1.set_duty_cycle, control_PWM[1])
	loop_for(0.0001, pwm2.set_duty_cycle, control_PWM[2])
	loop_for(0.0001, pwm3.set_duty_cycle, control_PWM[3])

def calculate_position(c, t):
	"""
	Calculates a position given a set of quintic coefficients and a time.

	Args
		c: List of coefficients generated by a quintic polynomial 
			trajectory generator.
		t: Time at which to calculate the position

	Returns
		Position
	"""
	return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]


def calculate_velocity(c, t):
	"""
	Calculates a velocity given a set of quintic coefficients and a time.

	Args
		c: List of coefficients generated by a quintic polynomial 
			trajectory generator.
		t: Time at which to calculate the velocity

	Returns
		Velocity
	"""
	return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]


def calculate_acceleration(c, t):
	"""
	Calculates an acceleration given a set of quintic coefficients and a time.

	Args
		c: List of coefficients generated by a quintic polynomial 
			trajectory generator.
		t: Time at which to calculate the acceleration

	Returns
		Acceleration
	"""
	return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]




def main_control_loop(x_c, y_c, z_c):
	#getting the desired position and yaw angle from trajectory planner: 
	#desired_pos_info = traj_planner()
	
	i = 0
	n_run = 6
	irun = 0
	#pos, Euler, vel, A_vel = reading_positional_info()

	des_yaw = 0 # This just set for 0 temporarily

	t = 0
	num_via_points = 10
	index = 1

	t_list = np.linspace(t, T, num_via_points)

	while True: # This is the loop for trajectory generation.
		loop = True
		while loop:
			start_loop = time.time()

			des_x_pos = calculate_position(x_c[i], t_list[index])[0]
			des_y_pos = calculate_position(y_c[i], t_list[index])[0]
			des_z_pos = calculate_position(z_c[i], t_list[index])[0]
			des_x_vel = calculate_velocity(x_c[i], t_list[index])[0]
			des_y_vel = calculate_velocity(y_c[i], t_list[index])[0]
			des_z_vel = calculate_velocity(z_c[i], t_list[index])[0]
			des_x_acc = calculate_acceleration(x_c[i], t_list[index])[0]
			des_y_acc = calculate_acceleration(y_c[i], t_list[index])[0]
			des_z_acc = calculate_acceleration(z_c[i], t_list[index])[0]

			#put the desired values into arrays
			desired_pos = np.array ([des_x_pos, des_y_pos, des_z_pos])
			desired_vel = np.array ([des_x_vel, des_y_vel, des_z_vel])
			desired_acc = np.array ([des_x_acc, des_y_acc, des_z_acc])

			#store the desired values
			#des_pos_store.append(desired_pos)
			#des_vel_store.append(desired_vel)

			#stack the value in a list
			desired_pos_info = []    
			desired_pos_info.append(desired_pos)
			desired_pos_info.append(desired_vel)
			desired_pos_info.append(desired_acc)
			desired_pos_info.append(des_yaw)

			#put the pose values in array
			#Euler = np.array([roll, pitch, yaw])
			#A_vel = np.array([roll_vel, pitch_vel, yaw_vel])
			#put the pos in array
			#pos = np.array([x_pos, y_pos, z_pos])
			#vel = np.array([x_vel, y_vel, z_vel])

			# reading positional info from optitrack:
			pos, Euler, vel, A_vel = reading_positional_info()
			#print("pos in the position: ", pos)


			u1, desired_pos, loop = position_control(desired_pos_info, pos, vel, loop)

			#for ii in range(2):
			#pos, Euler, vel, A_vel = reading_positional_info()
			#print("pos in the attitude: ", pos)
			u2 = attitude_control(Euler, A_vel, desired_pos)
			control_PWM = motor_mix_controller(u1, u2)
			#print(control_PWM)
			drive_motor(control_PWM)
			#store_PWM = np.vstack((store_PWM,control_PWM))
			#np.save("store_PWM.npy", store_PWM)
			#store_Euler = np.vstack((store_Euler,Euler_error))
			#np.save("store_Euler.npy", store_Euler)


			#store_pos = np.vstack((store_pos,pos_error))
			#np.save("store_pos.npy", store_pos)

			t += time.time() - start_loop
			#print("time spend: ", t)
			t = 0
		 
		print("reach the %d via point: " % index)

		if index < num_via_points - 1:
			index += 1

	print("Done")
		

def main():
	"""
	Calculates the x, y, z coefficients for the four segments 
	of the trajectory
	"""
	#calibration_ESC()
	wait_until_motor_is_ready()
	loop_for(0.5, pwm0.set_duty_cycle, SERVO_MIN)
	loop_for(0.5, pwm1.set_duty_cycle, SERVO_MIN)
	loop_for(0.5, pwm2.set_duty_cycle, SERVO_MIN)
	loop_for(0.5, pwm3.set_duty_cycle, SERVO_MIN)

	pos, _, _, _ = reading_positional_info()
	x_start = pos[0]
	y_start = pos[1]
	z_start = pos[2]

	x_coeffs = [[], [], []]#[[], [], [], [],[], [], [], [],[],[]]
	y_coeffs = [[], [], []]#[[], [], [], [],[], [], [], [],[],[]]
	z_coeffs = [[], [], []]#[[], [], [], [],[], [], [], [],[],[]]
	waypoints = [[x_start, y_start, z_start], [x_start, y_start, z_start - 0.5], [x_start, y_start, z_start]]

	for i in range(3):
		traj = TrajectoryGenerator(waypoints[i], waypoints[(i + 1) % 3], T)
		traj.solve()
		x_coeffs[i] = traj.x_c
		y_coeffs[i] = traj.y_c
		z_coeffs[i] = traj.z_c

	#store_PWM = np.zeros(4)
	#store_Euler = np.zeros(3)
	#store_pos = np.zeros(3)

	warm_up = 0

	t1 = time.time()
	while warm_up < 1.0:
		drive_motor(np.array([pwm_thres_min,pwm_thres_min,pwm_thres_min,pwm_thres_min]))
		warm_up = time.time() - t1

	#main_control_loop(x_coeffs, y_coeffs, z_coeffs)
	#visulization()

def before_test():
	pos, Euler, vel, A_vel = reading_positional_info()
	print("pos", pos)
	print("Euler: ", Euler)
	print("vel", vel)
	print("A-vel", A_vel)

def visulization():
	plt.figure(1)
	plt.title("pos_store")
	plt.plot(np.arange(len(store_PWM)), store_PWM,linewidth = '1')
	#plt.subplot(2,2,2)
	#plt.title("vel_store")
	#plt.plot(np.arange(len(vel_store)), vel_store,linewidth = '1')
	plt.show()
	

if __name__ == "__main__":
	main()
