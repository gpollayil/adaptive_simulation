#! /usr/bin/env python

import importlib
# Generic Imports
import os
import time

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

# ROS Params
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
adaptive_grasping_service_name = '/adaptive_task_service'

# For contact publishing
touch_pub_topic_name = '/touching_finger_topic'
finger_links_id_dict = \
    {39: 'soft_hand_thumb_distal_link',
     12: 'soft_hand_index_distal_link',
     26: 'soft_hand_middle_distal_link',
     33: 'soft_hand_ring_distal_link',
     19: 'soft_hand_little_distal_link'}                              # Looked this up as klampt simulation starts
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

    # NOT CLEAR WHAT THIS DOES... TODO: CHECK WHILE TESTING!!!
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
    for l in range(robot.numLinks()):
        sim.body(robot.link(l)).setCollisionPreshrink(vis_preshrink)
    for l in range(world.numRigidObjects()):
        sim.body(world.rigidObject(l)).setCollisionPreshrink(vis_preshrink)

    # Creating Hand emulator from the robot name
    sys.path.append('../../IROS2016ManipulationChallenge')
    module = importlib.import_module('plugins.' + passed_robot_name)
    # emulator takes the robot index (0), start link index (6), and start driver index (6)
    hand = module.HandEmulator(sim, 0, 6, 6)
    sim.addEmulator(0, hand)

    # The result of adaptive_controller.make() is now attached to control the robot
    import adaptive_controller
    sim.setController(robot, adaptive_controller.make(sim, hand, program.dt))

    # Latches the current configuration in the PID controller
    sim.controller(0).setPIDCommand(robot.getConfig(), robot.getVelocity())

    # Enabling contact feedback
    sim.enableContactFeedbackAll()

    # Updating simulation and visualization
    return update_simulation(world, sim)


def update_simulation(world, sim):
    # This code manually updates the visualization while doing other stuff for ROS: publishing joint_states, floating
    # frame's tf, publishing touch data

    # Creating the joints name array
    joint_names_array = []
    for i in joints_dict:
        joint_names_array.append(joints_dict[i])

    # Creating the JointState msg to be published
    syn_joint = JointState()
    syn_joint.name = joint_names_array
    syn_joint.effort = []
    syn_joint.velocity = []

    # Creating a TransformStamped
    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    static_transform_stamped.header.frame_id = world_frame_name

    # Creating the Int8 msg for touch publishing and the touch memory
    touch_msg = Int8()
    touch_memory = []

    vis.add("world", world)
    vis.show()
    t0 = time.time()

    sim_time = 0.01

    while vis.shown():

        # Simulating and updating visualization
        vis.lock()
        sim.simulate(sim_time)
        sim.updateWorld()
        vis.unlock()

        # Publishing joint states
        present_robot = world.robot(world.numRobots() - 1)

        # num_joints = present_robot.numDrivers();
        # if DEBUG:
        #     print "The number of joints is " + str(num_joints)

        joint_states = present_robot.getConfig()
        # if DEBUG:
        #     print "The present synergy joint is " + str(joint_states[34])

        # print "The present joints are " + str(joint_states)

        # Creating the joint values array
        joint_values_array = []
        for i in joints_dict:
            joint_values_array.append(joint_states[i])

        # Setting the synergy joint and publishing
        syn_joint.header = Header()
        syn_joint.header.stamp = rospy.Time.now()
        syn_joint.position = joint_values_array
        global_vars.joints_pub.publish(syn_joint)

        # Getting the transform from world to floating frame
        floating_link = present_robot.link(5)       # This id comes from trial and error (should be kuka coupler bottom)
        (R, t) = floating_link.getTransform()
        quat = so3.quaternion(R)

        # Setting the transform message and broadcasting palm frame
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.child_frame_id = floating_frame_name
        static_transform_stamped.transform.translation.x = t[0]
        static_transform_stamped.transform.translation.y = t[1]
        static_transform_stamped.transform.translation.z = t[2]
        static_transform_stamped.transform.rotation.x = quat[1]
        static_transform_stamped.transform.rotation.y = quat[2]
        static_transform_stamped.transform.rotation.z = quat[3]
        static_transform_stamped.transform.rotation.w = quat[0]

        global_vars.palm_broadcaster.sendTransform(static_transform_stamped)

        # Getting the transform from world to object and object velocity
        obj = world.rigidObject(world.numRigidObjects() - 1)
        (R_obj, t_obj) = obj.getTransform()
        (w_obj, v_obj) = obj.getVelocity()
        quat_obj = so3.quaternion(R_obj)

        # Setting the transform message and broadcasting object frame
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.child_frame_id = object_frame_name
        static_transform_stamped.transform.translation.x = t_obj[0]
        static_transform_stamped.transform.translation.y = t_obj[1]
        static_transform_stamped.transform.translation.z = t_obj[2]
        static_transform_stamped.transform.rotation.x = quat_obj[1]
        static_transform_stamped.transform.rotation.y = quat_obj[2]
        static_transform_stamped.transform.rotation.z = quat_obj[3]
        static_transform_stamped.transform.rotation.w = quat_obj[0]

        global_vars.obj_broadcaster.sendTransform(static_transform_stamped)

        # Publishing the object pose and twist
        obj_pose = Pose()
        obj_pose.position.x = static_transform_stamped.transform.translation.x
        obj_pose.position.y = static_transform_stamped.transform.translation.y
        obj_pose.position.z = static_transform_stamped.transform.translation.z
        obj_pose.orientation.x = static_transform_stamped.transform.rotation.x
        obj_pose.orientation.y = static_transform_stamped.transform.rotation.y
        obj_pose.orientation.z = static_transform_stamped.transform.rotation.z
        global_vars.obj_pose_pub.publish(obj_pose)
        obj_twist = Twist()
        obj_twist.angular.x = w_obj[0]
        obj_twist.angular.y = w_obj[1]
        obj_twist.angular.z = w_obj[2]
        obj_twist.linear.x = v_obj[0]
        obj_twist.linear.y = v_obj[1]
        obj_twist.linear.z = v_obj[2]
        global_vars.obj_twist_pub.publish(obj_twist)

        # TODO: here add auxiliary function for getting the new touch id and publish it
        check_contacts(world, sim, touch_memory, touch_msg)

        # Sleeping a little bit
        t1 = time.time()
        time.sleep(max(0.01 - (t1 - t0), 0.001))
        t0 = t1

    return


def check_contacts(world, sim, touch_memory, touch_msg):
    # Checks for contacts and returns an ids of touching fingers
    touches_now = []
    for i in range(world.numIDs()):
        for j in range(i+1, world.numIDs()):
            # Looping over everything and checking if the distal links are present
            # TODO: Do this for all links (Mods will be needed also in contact state where the finger_FK is called)
            if sim.inContact(i, j):
                cont_f = sim.contactForce(i, j)
                cont_t = sim.contactTorque(i, j)
                first_link = world.getName(i)
                second_link = world.getName(j)

                if DEBUG:
                    print " ", first_link, "-", second_link, "contact force", cont_f, "and torque", cont_t

                # Checking if in touch id dict and saving to touches now
                for k in touch_id_dict:
                    if (k in first_link or k in second_link):
                        if DEBUG:
                            print 'FOUND IN TOUCH_DICT!!!!'
                        touches_now.append(touch_id_dict.get(k))

    # The first id in touches now which is not in touch memory will be published
    if DEBUG:
        print 'The touch_memory is ', touch_memory
        print 'The touches_now are ', touches_now

    id_for_pub = 0
    for k in touches_now:
        if k not in touch_memory:
            id_for_pub = k
            touch_memory.append(id_for_pub)
            break
    if id_for_pub is not 0:
        touch_msg.data = id_for_pub
        global_vars.touch_pub.publish(touch_msg)



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
    global_vars.adaptive_service_client = rospy.ServiceProxy(adaptive_grasping_service_name, adaptiveGrasp) # TODO FROM HERE TOMORROW!!!!!

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
