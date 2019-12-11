#! /usr/bin/env python

import importlib
# Generic Imports
import os
import time
import numpy as np

# ROS Imports
import roslib

roslib.load_manifest('adaptive_simulation')
import rospy
import tf2_ros
# ROS msg Imports
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Int8
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64

# Checking for good Klampt version
import pkg_resources
pkg_resources.require("klampt>=0.7")

# Imports, assuming that Klampt version is ok
from klampt import vis
from klampt.io import resource
from klampt.vis.glrobotprogram import *
from klampt.sim import *
from klampt.math import so3

# Auxiliary functions file
import make_elements as mk_el
import move_elements as mv_el

# Custom Adaptive Grasping imports for calling the run adaptive grasper
from adaptive_grasping.srv import adaptiveGrasp, adaptiveGraspRequest

# Importing global variables
import global_vars

"""
This script creates the Klampt simulation for testing the adaptive_grasping.
ATTENTION! The package IROS2016ManipulationChallenge package is needed in order to use its
robot models and objects.
"""

DEBUG = False

# Absolute path of the current file folder
file_path = os.path.abspath(os.path.dirname(__file__))

# Some global constants
path_prefix = file_path + '/../../IROS2016ManipulationChallenge/'
print path_prefix
objects = {'apc2015': [f for f in os.listdir(path_prefix + 'data/objects/apc2015')]}

# Some global params
robot_name = "soft_hand"                                                # robot model
terrain_file = path_prefix + "data/terrains/plane.env"                  # terrain

# ROS PARAMS

# For joint state and floating frame publishing
hand_topic = '/right_hand/velocity_controller/command'
arm_topic = '/panda_arm/cartesian_velocity_controller/command'
joints_pub_topic_name = '/soft_hand_klampt/joint_states'
syn_joint_name = 'soft_hand_synergy_joint'
world_frame_name = 'world'
floating_frame_name = 'soft_hand_kuka_coupler_bottom'

# For object pose and twist publishing
obj_pose_pub_topic_name = '/object_pose'
obj_twist_pub_topic_name = '/object_twist'
object_frame_name = 'object'

# For Adaptive Grasping service
adaptive_grasping_service_name = '/adaptive_grasper_service'

# For contact publishing
palm_force_thresh = 2
palm_link_name = 'soft_hand_palm_link'
touch_pub_topic_name = '/touching_finger_topic'
touch_id_dict = \
    {'soft_hand_thumb_distal_link': 1,
     'soft_hand_index_distal_link': 2,
     'soft_hand_middle_distal_link': 3,
     'soft_hand_ring_distal_link': 4,
     'soft_hand_little_distal_link': 5}                               # Convention in adaptive_params.yaml

joints_dict = {
    6: 'soft_hand_index_abd_joint', 7: 'soft_hand_index_inner_joint_mimic', 8: 'soft_hand_index_inner_joint',
    9: 'soft_hand_index_middle_joint_mimic', 10: 'soft_hand_index_middle_joint',
    11: 'soft_hand_index_outer_joint_mimic',
    12: 'soft_hand_index_outer_joint',
    13: 'soft_hand_little_abd_joint', 14: 'soft_hand_little_inner_joint_mimic', 15: 'soft_hand_little_inner_joint',
    16: 'soft_hand_little_middle_joint_mimic', 17: 'soft_hand_little_middle_joint',
    18: 'soft_hand_little_outer_joint_mimic',
    19: 'soft_hand_little_outer_joint',
    20: 'soft_hand_middle_abd_joint', 21: 'soft_hand_middle_inner_joint_mimic', 22: 'soft_hand_middle_inner_joint',
    23: 'soft_hand_middle_middle_joint_mimic', 24: 'soft_hand_middle_middle_joint',
    25: 'soft_hand_middle_outer_joint_mimic',
    26: 'soft_hand_middle_outer_joint',
    27: 'soft_hand_ring_abd_joint', 28: 'soft_hand_ring_inner_joint_mimic', 29: 'soft_hand_ring_inner_joint',
    30: 'soft_hand_ring_middle_joint_mimic', 31: 'soft_hand_ring_middle_joint', 32: 'soft_hand_ring_outer_joint_mimic',
    33: 'soft_hand_ring_outer_joint',
    34: 'soft_hand_synergy_joint',
    35: 'soft_hand_thumb_abd_joint', 36: 'soft_hand_thumb_inner_joint_mimic', 37: 'soft_hand_thumb_inner_joint',
    38: 'soft_hand_thumb_outer_joint_mimic', 39: 'soft_hand_thumb_outer_joint'
}

"""
Functions
"""


def launch_grasping(passed_robot_name, object_set, object_name):
    """
    Launches simple program that simulates a robot grasping and object.
    It first allows a user to position the robot's free-floating base in a GUI.
    Then, it sets up a simulation with initial conditions, and launches a visualization.
    The controller waits for and executes the references given from outside.
    :param passed_robot_name: the name of the robot to be spawned (in data/robot)
    :param object_set: the name of the object set (in data/objects)
    :param object_name: the name of the object (in data/objects/<object_set>)
    :return: nothing
    """

    # Creating world and importing terrain
    world = WorldModel()
    world.loadElement(terrain_file)

    # Making the robot and the object (from make_elements.py)
    robot = mk_el.make_robot(passed_robot_name, world)
    mk_el.make_object(object_set, object_name, world)

    # NOT CLEAR WHAT THIS DOES... AS OF NOW THIS WORKS... HOWEVER -> TODO: Check this out!!!
    do_edit = True
    xform = resource.get("%s/default_initial_%s.xform" % (object_set, passed_robot_name),
                         description="Initial hand transform",
                         default=robot.link(5).getTransform(), world=world)
    mv_el.set_moving_base_xform(robot, xform[0], xform[1])
    xform = resource.get("%s/initial_%s_%s.xform" % (object_set, passed_robot_name, object_name),
                         description="Initial hand transform", default=robot.link(5).getTransform(), world=world,
                         doedit=False)
    if xform:
        mv_el.set_moving_base_xform(robot, xform[0], xform[1])
    xform = resource.get("%s/initial_%s_%s.xform" % (object_set, passed_robot_name, object_name),
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
    vis_preshrink = True    # turn this to true if you want to see the "shrunken" models used for collision detection
    for k in range(robot.numLinks()):
        sim.body(robot.link(k)).setCollisionPreshrink(vis_preshrink)
    for k in range(world.numRigidObjects()):
        sim.body(world.rigidObject(k)).setCollisionPreshrink(vis_preshrink)

    # Creating Hand emulator from the robot name
    sys.path.append('../../IROS2016ManipulationChallenge')
    module = importlib.import_module('plugins.' + passed_robot_name)
    # emulator takes the robot index (0), start link index (6), and start driver index (6)
    hand = module.HandEmulator(sim, 0, 6, 6)
    sim.addEmulator(0, hand)

    # The result of adaptive_controller.make() is now attached to control the robot
    import adaptive_controller
    sim_time = program.dt
    sim.setController(robot, adaptive_controller.make(sim, hand, sim_time))

    # Latches the current configuration in the PID controller
    rob_conf = robot.getConfig()
    print 'The starting robot configuration is ', rob_conf
    sim.controller(0).setPIDCommand(robot.getConfig(), robot.getVelocity())

    # Enabling contact feedback (NOT NEEDED FOR GETTING CONTACTS)
    # sim.enableContactFeedbackAll()

    # Updating simulation and visualization
    return update_simulation(world, sim, sim_time)


def update_simulation(world, sim, d_time):
    # This code manually updates the visualization while doing other stuff for ROS: publishing joint_states, floating
    # frame's tf, publishing touch data, object pose and twist

    # Creating the present robot
    present_robot = world.robot(world.numRobots() - 1)

    # Creating the joints name array
    joint_names_array = []
    for i in joints_dict:
        joint_names_array.append(joints_dict[i])

    # Creating the JointState msg to be published
    joints_msg = JointState()
    joints_msg.name = joint_names_array
    joints_msg.effort = []
    joints_msg.velocity = []

    # Creating a TransformStamped
    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    static_transform_stamped.header.frame_id = world_frame_name

    # Creating the Int8 msg for touch publishing and the touch memory
    touch_msg = Int8()
    touch_memory = []

    vis.add("world", world)
    vis.show()
    t0 = time.time()
    sim_time = d_time

    print 'The sim time in main is ', d_time

    # Publishing all what is needed for adaptive grasping and calling it
    publish_joints(present_robot, joints_msg)
    publish_palm(present_robot, static_transform_stamped)
    publish_object(world, static_transform_stamped)

    # Waiting and calling the adaptive grasping
    rospy.wait_for_service(adaptive_grasping_service_name)
    call_adaptive_grasping(True)

    while vis.shown():

        # Simulating and updating visualization
        vis.lock()
        sim.simulate(sim_time)
        sim.updateWorld()
        vis.unlock()

        # Publishing joint states, palm frame and object frame, pose and twist
        publish_joints(present_robot, joints_msg)
        publish_palm(present_robot, static_transform_stamped)
        publish_object(world, static_transform_stamped)

        # Getting the new touch id and publishing it while checking for algorithm stopping conditions
        check_contacts(world, sim, touch_memory, touch_msg)

        # Sleeping a little bit
        t1 = time.time()
        time.sleep(max(0.01 - (t1 - t0), 0.001))
        t0 = t1

    return


def call_adaptive_grasping(bool_run_grasp):
    """ Auxiliary function to start or end the adaptive grasping"""
    adaptive_req = adaptiveGraspRequest()
    adaptive_req.run_adaptive_grasp = bool_run_grasp
    print 'calling adaptive service!'
    adaptive_res = global_vars.adaptive_service_client(adaptive_req)
    print 'called adaptive service!'

    if not adaptive_res:
        rospy.logerr("Could not call the adaptive grasping... Exiting!")
        return


def publish_joints(robot, msg_joints):
    # Simple function for publishing hand joints

    # num_joints = present_robot.numDrivers();
    # if DEBUG:
    #     print "The number of joints is " + str(num_joints)

    joint_states = robot.getConfig()
    # if DEBUG:
    #     print "The present synergy joint is " + str(joint_states[34])

    # print "The present joints are " + str(joint_states)

    # Creating the joint values array
    joint_values_array = []
    for i in joints_dict:
        joint_values_array.append(joint_states[i])

    # Setting the synergy joint and publishing
    msg_joints.header = Header()
    msg_joints.header.stamp = rospy.Time.now()
    msg_joints.position = joint_values_array
    global_vars.joints_pub.publish(msg_joints)


def publish_palm(robot, msg):
    # Simple function for publishing the palm frame

    # Getting the transform from world to floating frame
    floating_link = robot.link(5)       # This id comes from trial and error (should be kuka coupler bottom)
    (R, t) = floating_link.getTransform()
    quat = so3.quaternion(R)

    # Setting the transform message and broadcasting palm frame
    msg.header.stamp = rospy.Time.now()
    msg.child_frame_id = floating_frame_name
    msg.transform.translation.x = t[0]
    msg.transform.translation.y = t[1]
    msg.transform.translation.z = t[2]
    msg.transform.rotation.x = quat[1]
    msg.transform.rotation.y = quat[2]
    msg.transform.rotation.z = quat[3]
    msg.transform.rotation.w = quat[0]

    global_vars.palm_broadcaster.sendTransform(msg)


def publish_object(world_, msg):
    # A simple function for publishing object frame, pose and twist

    # Getting the transform from world to object and object velocity
    obj = world_.rigidObject(world_.numRigidObjects() - 1)
    (R_obj, t_obj) = obj.getTransform()
    (w_obj, v_obj) = obj.getVelocity()
    quat_obj = so3.quaternion(R_obj)

    # Setting the transform message and broadcasting object frame
    msg.header.stamp = rospy.Time.now()
    msg.child_frame_id = object_frame_name
    msg.transform.translation.x = t_obj[0]
    msg.transform.translation.y = t_obj[1]
    msg.transform.translation.z = t_obj[2]
    msg.transform.rotation.x = quat_obj[1]
    msg.transform.rotation.y = quat_obj[2]
    msg.transform.rotation.z = quat_obj[3]
    msg.transform.rotation.w = quat_obj[0]

    global_vars.obj_broadcaster.sendTransform(msg)

    # Publishing the object pose and twist
    obj_pose = Pose()
    obj_pose.position.x = msg.transform.translation.x
    obj_pose.position.y = msg.transform.translation.y
    obj_pose.position.z = msg.transform.translation.z
    obj_pose.orientation.x = msg.transform.rotation.x
    obj_pose.orientation.y = msg.transform.rotation.y
    obj_pose.orientation.z = msg.transform.rotation.z
    global_vars.obj_pose_pub.publish(obj_pose)
    obj_twist = Twist()
    obj_twist.angular.x = w_obj[0]
    obj_twist.angular.y = w_obj[1]
    obj_twist.angular.z = w_obj[2]
    obj_twist.linear.x = v_obj[0]
    obj_twist.linear.y = v_obj[1]
    obj_twist.linear.z = v_obj[2]
    global_vars.obj_twist_pub.publish(obj_twist)


def check_contacts(world, sim, touch_memory, touch_msg):
    """
    Checks for contacts and publishes available contacts
    outputs:    bool big_palm_force -> true if the force between palm and object is over a threshold (defined in head)
                int len(touch_memory) -> number of fingers that touch object (not touches_now because of repetitions)
    """
    touches_now = []
    big_palm_force = False
    for i in range(world.numIDs()):
        for j in range(i+1, world.numIDs()):
            # Looping over everything and checking if the distal links are present and creating touches array
            # TODO: Do this for all links (check if mods needed also in contact state where the finger_FK is called)
            if sim.inContact(i, j):
                cont_f = sim.contactForce(i, j)
                cont_t = sim.contactTorque(i, j)
                first_link = world.getName(i)
                second_link = world.getName(j)

                if DEBUG or True:
                    print " ", first_link, "-", second_link, "contact force", cont_f, "and torque", cont_t

                # Checking if palm is squeezing object with high force
                if palm_link_name in first_link or palm_link_name in second_link:
                    if np.linalg.norm(cont_f) > palm_force_thresh:
                        big_palm_force = True

                # Checking if in touch id dict and saving to touches now
                for k in touch_id_dict:
                    if k in first_link or k in second_link:
                        if np.linalg.norm(cont_f):
                            touches_now.append(touch_id_dict.get(k))

    # The first id in touches now which is not in touch memory will be published
    if DEBUG:
        print 'The touches_now are ', touches_now

    # Updating touch memory from present touches
    id_for_pub = 0
    for k in touches_now:
        if k not in touch_memory:
            id_for_pub = k
            touch_memory.append(id_for_pub)
            break

    if DEBUG or True:
        print 'touch_memory is ', touch_memory

    if touch_memory:
        touch_msg.data = touch_memory[-1]
    else:
        touch_msg.data = id_for_pub

    # Publishing
    # global_vars.touch_pub.publish(touch_msg)

    if DEBUG or True:
        print 'The big_palm_force is', big_palm_force, 'and the number of touches are', len(touch_memory)

    # Returning
    return big_palm_force, len(touch_memory)


""" 
Callbacks 
"""


def arm_callback(data):
    if DEBUG:
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    global_vars.arm_command = data


def hand_callback(data):
    if DEBUG:
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    global_vars.hand_command = data.data


""" 
Main 
"""


def main():

    # Initializing ROS Node
    rospy.init_node('main_ros_node')

    # Publishers for synergy joint, touch data and object pose and twist
    global_vars.joints_pub = rospy.Publisher(joints_pub_topic_name, JointState, queue_size=10)
    global_vars.touch_pub = rospy.Publisher(touch_pub_topic_name, Int8, queue_size=10)
    global_vars.obj_pose_pub = rospy.Publisher(obj_pose_pub_topic_name, Pose, queue_size=10)
    global_vars.obj_twist_pub = rospy.Publisher(obj_twist_pub_topic_name, Twist, queue_size=10)

    # Publisher for run adaptive grasper
    global_vars.adaptive_service_client = rospy.ServiceProxy(adaptive_grasping_service_name, adaptiveGrasp)

    # Subscribers to command topics and their variables
    rospy.Subscriber(arm_topic, Twist, arm_callback)
    rospy.Subscriber(hand_topic, Float64, hand_callback)

    # TF broadcaster for floating frame and for object frame
    global_vars.palm_broadcaster = tf2_ros.StaticTransformBroadcaster()
    global_vars.obj_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Checking for data_set
    try:
        data_set = sys.argv[1]
    except IndexError:
        print "Oops! Please specify a data set and try again!"
        return

    # Checking for object
    try:
        index = int(sys.argv[2])
        obj_name = objects[data_set][index]
    except IndexError:
        print "Oops! Specified object seems not to be there. Please specify an object of the data set and try again!"
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
