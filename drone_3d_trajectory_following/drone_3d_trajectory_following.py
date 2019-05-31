"""
Simulate a quadrotor following a 3D trajectory

Author: Daniel Ingram (daniel-s-ingram)
"""

from math import cos, sin
import numpy as np
from Quadrotor import Quadrotor
from TrajectoryGenerator import TrajectoryGenerator
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.patches as patches

show_animation = False

SERVO_MAX = 1.75

# Simulation parameters
g = 9.81
m = 2.3 #0.2
L = 0.28 
Ixx = 3.613e-2 #2
Iyy = 3.613e-2 #2
Izz = 6.707e-2 #1
T = 5
Inertia = np.array([[Ixx,0,0],[0,Iyy,0],[0,0,Izz]])

#model coefficients
k = 3.876e-5
b = 7.233e-7        
gamma = b/k

wb = 144.37 # rad/s
Cr = 615.1 # rad/s

cof = 0.5*np.sqrt(2)
#Motor_mix = np.linalg.inv(np.array([[0.8,0.8,0.8,0.8],[-cof * L, cof * L, cof * L, -cof * L],
#                        [-cof * L, cof * L, -cof * L, cof * L],[gamma, gamma, -gamma, -gamma]]))
#Motor_mix = np.linalg.inv(np.array([[0.8,0.8,0.8,0.8],[-cof * L, cof * L, cof * L, -cof * L],
#                        [cof * L, -cof * L, cof * L, -cof * L],[gamma, gamma, -gamma, -gamma]]))
# Motor coefficients

Max_PWM_Hz = 800  # Hz
Min_PWM_Hz = 571.4 # Hz

pwm_thres_max = SERVO_MAX
pwm_thres_min = SERVO_MAX * 0.3
"""
# Proportional coefficients
Kp_x = 1
Kp_y = 1
Kp_z = 1
Kp_roll = 25
Kp_pitch = 25
Kp_yaw = 25

# Derivative coefficients
Kd_x = 10
Kd_y = 10
Kd_z = 1
"""

#PD control parameters
K_p_roll = 40
K_p_pitch = 50
K_p_psi = 5

K_d_roll = 0.5#1 #0.1
K_d_pitch = 0.5#1 #0.1
K_d_psi = 0.5  #0.1

K_p_x = 1.5 #1
K_p_y = 1.5 #1
K_p_z = 15 #1

K_d_x = 0.025 #0.01
K_d_y = 0.025 #0.01
K_d_z = 0.025 #0.01

K_p = np.array([[K_p_x,0,0],[0,K_p_y,0],[0,0,K_p_z]])
K_d = np.array([[K_d_x,0,0],[0,K_d_y,0],[0,0,K_d_z]])

K_p_Pose = np.array([[K_p_roll,0,0],[0,K_p_pitch,0],[0,0,K_p_psi]])
K_d_Pose = np.array([[K_d_roll,0,0],[0,K_d_pitch,0],[0,0,K_d_psi]])

#storage varaibles: 
pos_store = []
vel_store = []
Euler_store = []
A_vel_store = []
des_pos_store = []
des_vel_store = []
des_Euler_store = []
des_A_vel_store = []


pos_error_store = []
vel_error_store = []
Euler_error_store = []
A_vel_error_store = []

dutycycle_store = []
control_PWM_store =[]


def quad_sim(x_c, y_c, z_c):
    """
    Calculates the necessary thrust and torques for the quadrotor to
    follow the trajectory described by the sets of coefficients
    x_c, y_c, and z_c.
    """
    x_pos = -5 #-5
    y_pos = -5 #-5
    z_pos = 5 #5
    x_vel = 0
    y_vel = 0
    z_vel = 0
    x_acc = 0
    y_acc = 0
    z_acc = 0
    roll = 0
    pitch = 0
    yaw = 0
    roll_vel = 0
    pitch_vel = 0
    yaw_vel = 0

    des_yaw = 0

    dt = 0.1
    t = 0

    q = Quadrotor(x=x_pos, y=y_pos, z=z_pos, roll=roll,
                  pitch=pitch, yaw=yaw, size=1, show_animation=show_animation)

    i = 0
    n_run = 8
    irun = 0

    while True:
        while t <= T:
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
            des_pos_store.append(desired_pos)
            des_vel_store.append(desired_vel)

            #stack the value in a list
            desired_pos_info = []    
            desired_pos_info.append(desired_pos)
            desired_pos_info.append(desired_vel)
            desired_pos_info.append(desired_acc)
            desired_pos_info.append(des_yaw)

            #put the pose values in array
            Euler = np.array([roll, pitch, yaw])
            A_vel = np.array([roll_vel, pitch_vel, yaw_vel])
            #put the pos in array
            pos = np.array([x_pos, y_pos, z_pos])
            vel = np.array([x_vel, y_vel, z_vel])

            pos_store.append(pos)
            vel_store.append(vel)
            Euler_store.append(Euler)
            A_vel_store.append(A_vel)

            thrust, desired_pose = position_control(desired_pos_info, pos, vel)
            for k in range(5):
                Euler = np.array([roll, pitch, yaw])
                A_vel = np.array([roll_vel, pitch_vel, yaw_vel])
                u2 = attitude_control(Euler, A_vel, desired_pose) #the inputs are desired Euler angle and angular rate and feedback pose info
                roll_torque, pitch_torque, yaw_torque = u2
                motor_mix_controller(thrust, u2)

                #insert the simulation of motors


                roll_vel += roll_torque * dt / Ixx
                pitch_vel += pitch_torque * dt / Iyy
                yaw_vel += yaw_torque * dt / Izz

                roll += roll_vel * dt
                pitch += pitch_vel * dt
                yaw += yaw_vel * dt

            R = rotation_matrix(roll, pitch, yaw)
            acc = (np.matmul(R, np.array(
                [0, 0, thrust]).T) - np.array([0, 0, m * g]).T) / m
            x_acc = acc[0]
            y_acc = acc[1]
            z_acc = acc[2]
            x_vel += x_acc * dt
            y_vel += y_acc * dt
            z_vel += z_acc * dt
            x_pos += x_vel * dt
            y_pos += y_vel * dt
            z_pos += z_vel * dt

            q.update_pose(x_pos, y_pos, z_pos, roll, pitch, yaw)

            t += dt

        t = 0
        #i = (i + 1) % 4
        i = (i + 1) % 4
        irun += 1
        if irun >= n_run:
            break

    print("Done")


def position_control(desired_pos_info, pos, vel): #input should be generated trajectory
    #calculate errors of position, velocity and acceleration.
    desired_pos, desired_vel, acc_desired, desired_yaw = desired_pos_info
    pos_error = desired_pos - pos
    vel_error = desired_vel - vel 
    #acc_desired come from derivative of trajectory

    #Store position errors
    pos_error_store.append(pos_error)
    vel_error_store.append(vel_error)

    #calculate acc command
    acc_command = acc_desired + np.dot(K_d, vel_error.T) + np.dot(K_p, pos_error.T)

    #calculate the total thrust for 4 motors
    u1 = m*g + m*acc_command[2] # scalar i.e. summation of 4 thrust

    # derive desired roll and desired pitch
    desired_roll = 1/g * (acc_command[0] * sin(desired_yaw) - acc_command[1] * cos(desired_yaw))
    desired_pitch = 1/g * (acc_command[0] * cos(desired_yaw) - acc_command[1] * sin(desired_yaw))

    desired_pose = np.array([desired_roll, desired_pitch, desired_yaw])

    des_Euler_store.append(desired_pose)

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
    Euler_error_store.append(Euler_error)
    A_vel_error_store.append(A_vel_error)

    #implement control for attitude
    u2 = np.zeros(3)
    u2 = np.dot(Inertia, (np.dot(K_p_Pose, Euler_error) + np.dot(K_d_Pose, A_vel_error)))
    return u2 

Force_store = []
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
    print("u2: ", u2)

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

    Motor_mix = np.linalg.inv(np.array([[1, 1, 1, 1], motor_mix_roll,
                                        motor_mix_pitch, motor_mix_yaw]))

    Force = np.dot(Motor_mix, np.array([u1,np.abs(u2[0]),np.abs(u2[1]),np.abs(u2[2])]))
    Force_store.append(Force)
    # transform force of each motor into rotation speed :
    omega = np.sqrt(1/k * Force)

    # dutycycle and rotation speed
    dutycycle = (omega - wb) / Cr
    #print ("dutycycle: ", dutycycle)

    #control_PWM = 1000/(dutycycle * (Max_PWM_Hz - Min_PWM_Hz) + Min_PWM_Hz)

    #control_PWM = 1000/(dutycycle * (Max_PWM_Hz - Min_PWM_Hz) + Min_PWM_Hz)
    control_PWM = dutycycle * 0.5 + 1.25

    # transform rotation speed into PWM duty cycles : 
        # note that PWM duty cycles may need saturation.
    for i in range(4):
        if control_PWM[i] > pwm_thres_max:
            control_PWM[i] = pwm_thres_max
        elif control_PWM[i] < pwm_thres_min:
            control_PWM[i] = pwm_thres_min
    #print ("control_PWM: ", control_PWM)
    control_PWM_store.append(control_PWM)
    return control_PWM

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


def rotation_matrix(roll, pitch, yaw):
    """
    Calculates the ZYX rotation matrix.

    Args
        Roll: Angular position about the x-axis in radians.
        Pitch: Angular position about the y-axis in radians.
        Yaw: Angular position about the z-axis in radians.

    Returns
        3x3 rotation matrix as NumPy array
    """
    return np.array(
        [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)],
         [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
          sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
         [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw)]
         ])

 
def main():
    """
    Calculates the x, y, z coefficients for the four segments 
    of the trajectory
    """
    x_coeffs = [[], [], [], []]
    y_coeffs = [[], [], [], []]
    z_coeffs = [[], [], [], []]
    waypoints = [[-5, -5, 5], [5, -5, 5], [5, 5, 5], [-5, 5, 5]]

    for i in range(4):
        traj = TrajectoryGenerator(waypoints[i], waypoints[(i + 1) % 4], T)
        traj.solve()
        x_coeffs[i] = traj.x_c
        y_coeffs[i] = traj.y_c
        z_coeffs[i] = traj.z_c

    quad_sim(x_coeffs, y_coeffs, z_coeffs)
    visulization()


    #print("pos_error_store: ", pos_error_store)
def visulization():
    plt.figure()
    #ax = fig.add_subplot(1,1,1)
    x_pos_store = []
    y_pos_store = []
    z_pos_store = []
    for k in range(len(pos_error_store)):
        x_pos_store.append(pos_error_store[k][0])
        y_pos_store.append(pos_error_store[k][1])
        z_pos_store.append(pos_error_store[k][2])
        #print("x_pos_error_store: ", x_pos_store)
    plt.subplot(2,2,1)
    plt.title("pos_store")
    plt.plot(np.arange(len(pos_store)), pos_store,linewidth = '1')
    plt.plot(np.arange(len(des_pos_store)), des_pos_store, linewidth= '3')
    #plt.subplot(2,2,2)
    #plt.title("vel_store")
    #plt.plot(np.arange(len(vel_store)), vel_store,linewidth = '1')
    #plt.plot(np.arange(len(des_vel_store)), des_vel_store, linewidth = '3')
    plt.subplot(2,2,2)
    plt.title("Force")
    plt.plot(np.arange(len(Force_store)), Force_store, linewidth = '1')
    plt.subplot(2,2,3)
    plt.title("Euler_store")
    plt.plot(np.arange(len(Euler_store)), Euler_store, linewidth = '1')
    plt.plot(np.arange(len(des_Euler_store)), des_Euler_store, linewidth = '3')
    #plt.subplot(2,2,4)
    #plt.title("A_vel_store")
    #plt.plot(np.arange(len(A_vel_store)), A_vel_store, linewidth = '1')
    plt.subplot(2,2,4)
    plt.title("control_PWM_store")
    plt.plot(np.arange(len(control_PWM_store)), control_PWM_store, linewidth = '0.5')
    plt.show()





if __name__ == "__main__":
    main()
