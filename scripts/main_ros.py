#! /usr/bin/env python

"""
This script creates the Klampt simulation for testing the adaptive_grasping.
ATTENTION! The package IROS2016ManipulationChallenge package is needed in order to use its
robot models and objects.
"""

# Generic Imports
import sys
import os
import importlib
import time

# ROS Imports
import roslib; roslib.load_manifest('adaptive_simulation')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Checking for good Klampt version
import pkg_resources
pkg_resources.require("klampt>=0.7")

# Imports, assuming that Klampt version is ok
from klampt import *
from klampt import vis
from klampt.io import resource
from klampt.vis.glrobotprogram import *
from klampt.sim import *

# Auxiliary functions file
import make_elements as mk_el
import move_elements as mv_el

DEBUG = True

# Absolute path of the current file folder
file_path = os.path.abspath(os.path.dirname(__file__))

# Some global constants
path_prefix = file_path + '/../../IROS2016ManipulationChallenge/'
print path_prefix
objects = {'apc2015': [f for f in os.listdir(path_prefix + 'data/objects/apc2015')]}

# Some global params
robot_name = "soft_hand"                                                # robot model
terrain_file = path_prefix + "data/terrains/plane.env"                  # terrain

# ROS Params
joints_pub_topic_name = '/soft_hand_klampt/joint_states'
syn_joint_name = 'soft_hand_synergy_joint'

"""
Functions
"""

def launch_grasping(robot_name, object_set, object_name):
    """
    Launches simple program that simulates a robot grasping and object.
    It first allows a user to position the robot's free-floating base in a GUI.
    Then, it sets up a simulation with initial conditions, and launches a visualization.
    The controller waits for and executes the references given from outside.
    :param robot_name: the name of the robot to be spawned (in data/robot)
    :param object_set: the name of the object set (in data/objects)
    :param object_name: the name of the object (in data/objects/<object_set>)
    :return: nothing
    """

    # Creating world and importing terrain
    world = WorldModel()
    world.loadElement(terrain_file)

    # Making the robot and the object (from make_elements.py)
    robot = mk_el.make_robot(robot_name, world)
    mk_el.make_object(object_set, object_name, world)

    # NOT CLEAR WHAT THIS DOES... TODO: CHECK WHILE TESTING!!!
    do_edit = True
    xform = resource.get("%s/default_initial_%s.xform" % (object_set, robot_name), description="Initial hand transform",
                         default=robot.link(5).getTransform(), world=world)
    mv_el.set_moving_base_xform(robot, xform[0], xform[1])
    xform = resource.get("%s/initial_%s_%s.xform" % (object_set, robot_name, object_name),
                         description="Initial hand transform", default=robot.link(5).getTransform(), world=world,
                         doedit=False)
    if xform:
        mv_el.set_moving_base_xform(robot, xform[0], xform[1])
    xform = resource.get("%s/initial_%s_%s.xform" % (object_set, robot_name, object_name),
                         description="Initial hand transform", default=robot.link(5).getTransform(), world=world,
                         doedit=do_edit)
    if not xform:
        print "User quit the program"
        return
    # this sets the initial condition for the simulation
    mv_el.set_moving_base_xform(robot, xform[0], xform[1])

    # Launch the simulation
    program = GLSimulationPlugin(world)
    sim = program.sim

    # Setting up simulation parameters
    visPreShrink = True # turn this to true if you want to see the "shrunken" models used for collision detection
    for l in range(robot.numLinks()):
        sim.body(robot.link(l)).setCollisionPreshrink(visPreShrink)
    for l in range(world.numRigidObjects()):
        sim.body(world.rigidObject(l)).setCollisionPreshrink(visPreShrink)

    # Creating Hand emulator from the robot name
    sys.path.append('../../IROS2016ManipulationChallenge')
    module = importlib.import_module('plugins.' + robot_name)
    # emulator takes the robot index (0), start link index (6), and start driver index (6)
    hand = module.HandEmulator(sim, 0, 6, 6)
    sim.addEmulator(0, hand)

    # The result of adaptive_controller.make() is now attached to control the robot
    import adaptive_controller
    sim.setController(robot, adaptive_controller.make(sim, hand, program.dt))

    # Latches the current configuration in the PID controller
    sim.controller(0).setPIDCommand(robot.getConfig(), robot.getVelocity())

    # Updating simulation and visualization
    return update_simulation(world, sim)


def update_simulation(world, sim):
    # this code manually updates the visualization

    # Creating the JointState msg to be published
    syn_joint = JointState()
    syn_joint.name = [syn_joint_name]
    syn_joint.effort = []
    syn_joint.velocity = []

    vis.add("world", world)
    vis.show()
    t0 = time.time()

    while vis.shown():

        # Sumulating and updating visualization
        vis.lock()
        sim.simulate(0.01)
        sim.updateWorld()
        vis.unlock()

        # Publishing joint states
        present_robot = world.robot(world.numRobots() - 1)

        # num_joints = present_robot.numDrivers();
        # if DEBUG:
        #     print "The number of joints is " + str(num_joints)

        joint_states = present_robot.getConfig()
        if DEBUG:
            print "The present synergy joint is " + str(joint_states[34])

        # Setting the synergy joint and publishing
        syn_joint.header = Header()
        syn_joint.header.stamp = rospy.Time.now()
        syn_joint.position = [joint_states[34]]
        joints_pub.publish(syn_joint)

        # Sleeping a little bit
        t1 = time.time()
        time.sleep(max(0.01 - (t1 - t0), 0.001))
        t0 = t1

    return


""" 
Main 
"""

def main():

    # Initializing ROS Node
    rospy.init_node('main_ros_node')

    # Publisher of the synergy joint for ROS
    global joints_pub
    joints_pub = rospy.Publisher(joints_pub_topic_name, JointState, queue_size=10)

    # Checking for data_set
    try:
        data_set = sys.argv[1]
    except IndexError:
        print "Oops! Please indicate a data set and try again!"
        return

    # Checking for object
    try:
        index = int(sys.argv[2])
        obj_name = objects[data_set][index]
    except IndexError:
        print "Oops! The specified object seems not to be there. Please indicate an object of the data set and try again!"
        return
    except ValueError:
        obj_name = sys.argv[2]

    # Launching the grasp simulation and when it finishes killing visualization
    launch_grasping(robot_name, data_set, obj_name)

    vis.kill()


if __name__ == '__main__':

    print "Adaptive Grasping Simulation ROS Node."

    # This file needs to be called in the following format
    # format: rosrun adaptive_simulation main_ros.py <data_set> <object>

    if len(sys.argv) < 3:
        print "Usage: : rosrun adaptive_simulation main_ros.py <data_set> <object>"
    else:
        try:
            main()
        except rospy.ROSInterruptException:
            pass
