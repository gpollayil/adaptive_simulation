"""
This script creates the Klampt simulation for testing the adaptive_grasping.
ATTENTION! The package IROS2016ManipulationChallenge package is needed in order to use its
robot models and objects.
"""

# Generic imports
import sys
import os
import importlib
import time

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

# Some global constants
path_prefix = '../../IROS2016ManipulationChallenge/'
objects = {}
objects['apc2015'] = [f for f in os.listdir(path_prefix + 'data/objects/apc2015')]

# Some global params
robot_name = "reflex_col"                               # robot model
terrain_file = path_prefix + "data/terrains/plane.env"                # terrain

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
    object = mk_el.make_object(object_set, object_name, world)

    # NOT CLEAR WHAT THIS DOES... TODO: CHECK WHILE TESTING!!!
    doedit = True
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
                         doedit=doedit)
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
    vis.add("world", world)
    vis.show()
    t0 = time.time()
    while vis.shown():
        vis.lock()
        sim.simulate(0.01)
        sim.updateWorld()
        vis.unlock()
        t1 = time.time()
        time.sleep(max(0.01 - (t1 - t0), 0.001))
        t0 = t1
    return


""" 
Main 
"""

def main():
    print "Adaptive Grasping Simulation."

    # This file needs to be called in the following format
    # format: python main.py <data_set> <object>

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
    main()


