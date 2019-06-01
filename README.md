# Drone project

This is a project of controlling a drone in and indoor environment(GPS denied)

## Simulation

This simulation is based on the following git repo:
    https://github.com/AtsushiSakai/PythonRobotics
    
All the code for simulation is in drone_3d_trajectory_following folder. 
    
    python drone_simulation.py

Here is the gif of the simulation


## Hardware implementation

The indoor localization is realized by using 6 cameras optitrack motion capture system. We use ROS package vrpn_client_ros to receive the streaming data from ground station of optitrack.

The ROS topic is converted to basic numpy array and sending to the control board of the drone by using zeromq technique.
