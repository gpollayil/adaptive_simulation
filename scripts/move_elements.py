"""
This auxiliary file contains the some fuctions used in main.py.
These functions move the object and the robot in the Klampt world
"""

from klampt.math import so3

def set_moving_base_xform(robot,R,t):
    """For a moving base robot model, set the current base rotation
    matrix R and translation t.  (Note: if you are controlling a robot
    during simulation, use send_moving_base_xform_command)
    """
    q = robot.getConfig()
    for i in range(3):
        q[i] = t[i]
    roll,pitch,yaw = so3.rpy(R)
    q[3]=yaw
    q[4]=pitch
    q[5]=roll
    robot.setConfig(q)