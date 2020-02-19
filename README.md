# Drone project

This is a project of controlling a drone in and indoor environment(GPS denied)

## Simulation

This simulation is based on the following git repo:
    https://github.com/AtsushiSakai/PythonRobotics
    
All the code for simulation is in drone_3d_trajectory_following folder. 
    
    python drone_simulation.py

Here is the gif of the simulation

This simulation is for drone hovering with disturbances. Those disturbances is to check whether the controller is work.

## Hardware implementation

We use Navio2 as our main control board. The Navio2 is attached on the Respberry pi B+ and give PWM to the ESC of the drone.

The navio folder contains some neccesary file for hardware read and out.

The indoor localization is realized by using 6 cameras optitrack motion capture system. We use ROS package vrpn_client_ros to receive the streaming data from ground station of optitrack.

The ROS topic is converted to basic numpy array and sending to the control board of the drone by using zeromq technique.

The diagram for the system is showing in the following figure:

![](/pic/system.png)
