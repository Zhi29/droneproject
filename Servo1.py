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

navio.util.check_apm()


rcin = navio.rcinput.RCInput()

#PWM SETTING######################################################
PWM_OUTPUT_0  = 0
PWM_OUTPUT_1  = 1
PWM_OUTPUT_2  = 2
PWM_OUTPUT_3  = 3
SERVO_MIN = 1.250 #1.072 #ms
SERVO_MAX = 1.750 #1.935 #ms  571.4 - 800 
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
savepose = deque(maxlen=2)
saveangu = deque(maxlen=2)
sub_port = 5556
context = zmq.Context()
#connect to socket we subscrib
socket_sub = context.socket(zmq.SUB)
#socket_sub.connect("tcp://localhost:5556")
socket_sub.connect("tcp://192.168.1.9:%d" %sub_port)
#socket_sub.setsockopt(zmq.SUBSCRIBE, b"")
socket_sub.setsockopt(zmq.SUBSCRIBE,b'')
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
    loop_for(0.1, pwm0.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm1.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm2.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm3.set_duty_cycle, SERVO_MIN)

    time.sleep(8)


def reading_positional_info():
    start = time.clock()
    #print('reading info.')
    contents  =  recv_array(socket_sub,copy=False)
    position = contents[0:3]
    a = contents[3]
    b = contents[4]
    c = contents[5]
    d = contents[6]
    orientation = np.array([a,b,c,d])
    orientation[0] *= -1.0
    #print("orientation: ", orientation)

    Rotation_mat=np.array([[1,0,0],[0,0,-1],[0,1,0]])
    Euler = quaternion2euler(orientation)
    Euler = np.dot(Rotation_mat.T, Euler)
    savet2 = (Euler[0],Euler[1],Euler[2])
    #print("Angle180", Euler[0]*180/np.pi,Euler[1]*180/np.pi,Euler[2]*180/np.pi)

    saveangu.appendleft(savet2)

    position = np.dot(Rotation_mat.T, position)
    #print("position:  ", position)

    savet = (position[0],position[1],position[2])
    savepose.appendleft(savet)

    elapsed = (time.clock() - start)# record the time to calculate the velocities.
    #print("time",elapsed)
    vx = (savepose[0][0] - savepose[-1][0])/elapsed*1.0
    vy = (savepose[0][1] - savepose[-1][1])/elapsed*1.0
    vz = (savepose[0][2] - savepose[-1][2])/elapsed*1.0
    vel = (vx,vy,vz)
    ax = (saveangu[0][0] - saveangu[-1][0])/elapsed*1.0
    ay = (saveangu[0][1] - saveangu[-1][1])/elapsed*1.0
    az = (saveangu[0][2] - saveangu[-1][2])/elapsed*1.0
    Angu = (ax,ay,az)
    #print("Angu speed",Angu)# currently in optitrack coordinates
    #print("Angu speed",ax*180/np.pi,ay*180/np.pi,az*180/np.pi)# currently in optitrack coordinates
    #print("speed",vel)# currently in optitrack coordinates
    print ("---------------------------------------------------------------------------------")
    return position, Euler, vel, Angu


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
m = 2.3 #kg
L = 0.28 # m 
Ixx = 3.613e-2 
Iyy = 3.613e-2 
Izz = 6.707e-2 
Inertia = np.array([[Ixx,0,0],[0,Iyy,0],[0,0,Izz]])
T = 5 # time for complete trajectory

# Model coefficients with Forces and Torques
k = 3.876e-5
b = 7.233e-7        
gamma = b/k

# Motor coefficients
wb = 144.37 # rad/s
Cr = 615.1 # rad/s

Max_PWM_Hz = 800  # Hz
Min_PWM_Hz = 571.4 # Hz

pwm_thres_max = SERVO_MAX
pwm_thres_min = SERVO_MAX * 0.3

cof = 0.5*np.sqrt(2)
Motor_mix = np.linalg.inv(np.array([[0.8,0.8,0.8,0.8],[-cof * L, cof * L, cof * L, -cof * L],
                        [-cof * L, cof * L, -cof * L, cof * L],[gamma, gamma, -gamma, -gamma]]))

#PD control parameters#######################################
K_p_roll = 25
K_p_pitch = 5
K_p_yaw = 25

K_d_roll = 0.5 #0.1
K_d_pitch = 0.5 #0.1
K_d_yaw = 0.5  # 0.1

K_p_x = 1.5 #1
K_p_y = 1.5 #1
K_p_z = 0.8 #1

K_d_x = 0.025 #0.01
K_d_y = 0.025 #0.01
K_d_z = 0.025 #0.01
#############################################################

K_p = np.array([[K_p_x,0,0],[0,K_p_y,0],[0,0,K_p_z]])
K_d = np.array([[K_d_x,0,0],[0,K_d_y,0],[0,0,K_d_z]])

K_p_Pose = np.array([[K_p_roll,0,0],[0,K_p_pitch,0],[0,0,K_p_yaw]])
K_d_Pose = np.array([[K_d_roll,0,0],[0,K_d_pitch,0],[0,0,K_d_yaw]])


def position_control(desired_pos_info, pos, vel): #input should be generated trajectory
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

    #des_Euler_store.append(desired_pose)

    return u1, desired_pose 


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

    #Store pose errors in lists.
    #Euler_error_store.append(Euler_error)
    #A_vel_error_store.append(A_vel_error)

    #implement control for attitude
    u2 = np.zeros(3)
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
    Force = np.dot(Motor_mix, np.array([u1,u2[0],u2[1],u2[2]]))
    Force = np.maximum(Force, 0)
    print("u1: ", u1)
    print("u2: ", u2)
    print("Motor_mix: ", Motor_mix)
    print("Force: ", Force)

    # transform force of each motor into rotation speed :
    omega = np.sqrt(1/k * Force)

    print("omega: ", omega)

    # dutycycle and rotation speed
    dutycycle = (omega - wb) / Cr

    print("dutycycle: ", dutycycle)

    control_PWM = 1000/(dutycycle * (Max_PWM_Hz - Min_PWM_Hz) + Min_PWM_Hz)

    # transform rotation speed into PWM duty cycles : 
        # note that PWM duty cycles may need saturation.
    for i in range(4):
        if control_PWM[i] > pwm_thres_max:
            control_PWM[i] = pwm_thres_max
        elif control_PWM[i] < pwm_thres_min:
            control_PWM[i] = pwm_thres_min
    print("control_PWM", control_PWM)

    return control_PWM

def drive_motor(control_PWM):
    # this function is mainly used to pass duty cycle into navio hardware to drive motor.
    loop_for(0.001, pwm0.set_duty_cycle, control_PWM[0])
    loop_for(0.001, pwm1.set_duty_cycle, control_PWM[1])
    loop_for(0.001, pwm2.set_duty_cycle, control_PWM[2])
    loop_for(0.001, pwm3.set_duty_cycle, control_PWM[3])

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
    n_run = 8
    irun = 0
    pos, Euler, vel, A_vel = reading_positional_info()

    des_yaw = 0 # This just set for 0 temporarily

    t = 0

    while True: # This is the loop for trajectory generation.
        while t < T:
            start_loop = time.clock()

            des_x_pos = calculate_position(x_c[i], t)[0]
            des_y_pos = calculate_position(y_c[i], t)[0]
            des_z_pos = calculate_position(z_c[i], t)[0]
            des_x_vel = calculate_velocity(x_c[i], t)[0]
            des_y_vel = calculate_velocity(y_c[i], t)[0]
            des_z_vel = calculate_velocity(z_c[i], t)[0]
            des_x_acc = calculate_acceleration(x_c[i], t)[0]
            des_y_acc = calculate_acceleration(y_c[i], t)[0]
            des_z_acc = calculate_acceleration(z_c[i], t)[0]

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
            u1, desired_pos = position_control(desired_pos_info, pos, vel)

            for ii in range(5):
                pos, Euler, vel, A_vel = reading_positional_info()
                u2 = attitude_control(Euler, A_vel, desired_pos)
                control_PWM = motor_mix_controller(u1, u2)
                drive_motor(control_PWM)

            t += time.clock() - start_loop

        t = 0
        i = (i + 1) % 4
        irun += 1
        if irun >= n_run:
            break

    print("Done")


def main():
    """
    Calculates the x, y, z coefficients for the four segments 
    of the trajectory
    """
    #wait_until_motor_is_ready()

    pos, _, _, _ = reading_positional_info()
    x_start = pos[0]
    y_start = pos[0]
    z_start = pos[0]

    x_coeffs = [[], [], [], []]
    y_coeffs = [[], [], [], []]
    z_coeffs = [[], [], [], []]
    waypoints = [[x_start, y_start, z_start], [x_start, y_start, z_start - 0.2], [x_start, y_start, z_start - 0.2], [x_start, y_start, z_start]]

    for i in range(4):
        traj = TrajectoryGenerator(waypoints[i], waypoints[(i + 1) % 4], T)
        traj.solve()
        x_coeffs[i] = traj.x_c
        y_coeffs[i] = traj.y_c
        z_coeffs[i] = traj.z_c

    main_control_loop(x_coeffs, y_coeffs, z_coeffs)

def before_test():
    pos, Euler, vel, A_vel = reading_positional_info()
    print("pos", pos)
    print("Euler: ", Euler)
    print("vel", vel)
    print("A-vel", A_vel)


if __name__ == "__main__":
    main()