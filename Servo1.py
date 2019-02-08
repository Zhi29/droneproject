import sys
import time
import math

import navio.pwm
import navio.util
import datetime
import curses
import navio.rcinput


from collections import deque
import zmq
import json


navio.util.check_apm()


rcin = navio.rcinput.RCInput()

#PWM SETTING######################################################

PWM_OUTPUT_0  = 0
PWM_OUTPUT_1  = 1
PWM_OUTPUT_2  = 2
PWM_OUTPUT_3  = 3
SERVO_MIN = 1.072 #ms
SERVO_MAX = 1.935 #ms
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


#reading OptiTrack information via Internet SETTING#############################################################
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

#calibration_ESC() 

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
    print('reading info.')
    contents  =  recv_array(socket_sub,copy=False)
    position = contents[0:3]
    a = contents[3]
    b = contents[4]
    c = contents[5]
    d = contents[6]
    orientation = np.array([a,b,c,d])
    orientation[0] *= -1.0
    print("orientation: ", orientation)

    Euler = quaternion2euler(orientation)
    print("Enler angle: ", Euler)
    savet2 = (Euler[0],Euler[1],Euler[2])
    print("Angle180", Euler[0]*180/np.pi,Euler[1]*180/np.pi,Euler[2]*180/np.pi)

    saveangu.appendleft(savet2)

    Rotation_mat=np.dot(np.array([[0,0,1],[0,1,0],[-1,0,0]]),np.array([[0,1,0],[-1,0,0],[0,0,1.0]]))
    position = np.dot(Rotation_mat, position)
    position[2] += 0.22

    savet = (contents[0],contents[1],contents[2])
    savepose.appendleft(savet)
    print(contents)

    elapsed = (time.clock() - start)# record the time to calculate the velocities.
    print("time",elapsed)
    vx = (savepose[0][0] - savepose[-1][0])/elapsed*1.0
    vy = (savepose[0][1] - savepose[-1][1])/elapsed*1.0
    vz = (savepose[0][2] - savepose[-1][2])/elapsed*1.0
    vel = (vx,vy,vz)
    ax = (saveangu[0][0] - saveangu[-1][0])/elapsed*1.0
    ay = (saveangu[0][1] - saveangu[-1][1])/elapsed*1.0
    az = (saveangu[0][2] - saveangu[-1][2])/elapsed*1.0
    Angu = (ax,ay,az)
    print("Angu speed",Angu)# currently in optitrack coordinates
    print("Angu speed",ax*180/np.pi,ay*180/np.pi,az*180/np.pi)# currently in optitrack coordinates
    print("speed",vel)# currently in optitrack coordinates
    print ("---------------------------------------------------------------------------------")
    return pos, Euler, vel, Angu


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

    

calibration_ESC()
test_throttle_via_RC()

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

m = 5   # MASS
Ixx = 
Iyy = 
Izz = 
Inertia = np.array([[Ixx,0,0],[0,Iyy,0],[0,0,Izz]])
L =   #LENGTH
g = 9.8 # gravity

#model coefficients
k = 
b =         

#PD control parameters
K_p_phi = 
K_p_theta = 
K_p_psi = 
K_d_phi = 
K_d_theta = 
K_d_psi = 


def main_control_loop(): 
    pos,Euler,vel,A_vel = reading_positional_info()
    # calculate errors
    # assume they are numpy array
    pos_error = desired_pos - pos
    Euler_error = desired_Euler - Euler
    vel_error = desired_vel - vel
    A_vel_error = desired_A_vel - A_vel

    # Assign PD parameters into diagnoal matrix
    K_p_Pose = np.array([[K_p_phi,0,0],[0,K_p_theta,0],[0,0,K_p_psi]])
    # PD law for orientation control
    e_Pose = K_p_Pose * Euler_error + K_d_Pose * A_vel_error # matrix multiplication

    common_term = m*g/(4*k*cos(Euler[1])*cos(Euler[0]))

    gamma_1 = common_term - (2*b*e_Pose[0]*Ixx + e_Pose[2]*Izz*k*L)/(4*b*k*L)
    gamma_2 = common_term + (e_Pose[2]*Izz)/(4*b) - (e_Pose[1]*Iyy)/(2*k*L)
    gamma_3 = common_term - (-2*b*e_Pose[0]*Ixx + e_Pose[2]*Izz*k*L)/(4*b*k*L)
    gamma_4 = common_term + e_Pose[2]*Izz/(4*b) + e_Pose[1]*Iyy/(2*k*L)

    #assume linear relation between omega = k * pwm 

    loop_for(0.01, pwm0.set_duty_cycle, period)
    loop_for(0.01, pwm1.set_duty_cycle, period)
    loop_for(0.01, pwm2.set_duty_cycle, period)
    loop_for(0.01, pwm3.set_duty_cycle, period)




















#Sample codes of Interaction through keyboards arrow button.#####################################################
'''

# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

try:
    while True:
        char = screen.getch()
        if char == ord('q'): 
            break
        elif char == curses.KEY_RIGHT:
            # print doesn't work with curses, use addstr instead
            screen.addstr(0, 0, 'right')
        elif char == curses.KEY_LEFT:
            screen.addstr(0, 0, 'left ')        
        elif char == curses.KEY_UP:
            screen.addstr(0, 0, 'up   ')        
        elif char == curses.KEY_DOWN:
            screen.addstr(0, 0, 'down ')
finally:
    # shut down cleanly
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
'''
##########################################################################################################