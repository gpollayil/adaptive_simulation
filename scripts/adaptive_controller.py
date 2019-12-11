"""
This auxiliary file contains the adaptive controller used in main.py.
"""

import plugins.actuators.CompliantHandEmulator  # TODO: Does this work? Specify this to be imported from other package
import plugins.reflex  # TODO: Does this work? Specify this to be imported from other package
import plugins.soft_hand  # TODO: Does this work? Specify this to be imported from other package
from klampt.math import se3, so3, vectorops
import math as sym
import numpy as np

import global_vars
import move_elements as mv_el

DEBUG = False

int_const_syn = 1
int_const_t = 1
int_const_eul = 1

# Not reading synergy always but keeping memory
start_time = 0.0
now_dur = 0.0
syn_curr = 0.0
syn_next = 0.0
entered_once = False
got_syn = False


def integrate_velocities(controller, sim, dt):
    """The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
    processing is needed when it is invoked."""

    global start_time
    global now_dur
    global syn_curr
    global syn_next
    global entered_once
    global got_syn

    if not entered_once:
        syn_curr = controller.getCommandedConfig()
        if not syn_curr:
            got_syn = False
            entered_once = False
        else:
            got_syn = True
            entered_once = True
            syn_curr = syn_curr[34]
            start_time = sim.getTime()
    else:
        syn_curr = syn_next
        now_dur = sim.getTime() - start_time
        # print 'The current simulation duration is', now_dur

    rob = sim.controller(0).model()
    palm_curr = mv_el.get_moving_base_xform(rob)
    R = palm_curr[0]
    t = palm_curr[1]

    if DEBUG:
        print 'The current palm rotation is ', R
        print 'The current palm translation is ', t
        print 'The simulation dt is ', dt

    # Converting to rpy
    euler = so3.rpy(R)

    # Checking if list empty and returning false
    if not got_syn:
        return False, None, None

    if DEBUG:
        print 'The integration time is ', dt
        print 'The adaptive velocity of hand is ', global_vars.hand_command
        print 'The adaptive twist of palm is \n', global_vars.arm_command
        # print 'The commanded position of the hand encoder (in memory) is ', syn_curr
        # print 'The present position of the hand encoder (sensed) is ', controller.getCommandedConfig()[34]
        v = rob.getVelocity()
        print 'The actual velocity of the robot is ', v
        # print 'The present pose of the palm is \n', palm_curr
        # print 'The present position of the palm is ', t, 'and its orientation is', euler

    # Getting linear and angular velocities
    lin_vel_vec = global_vars.arm_command.linear
    ang_vel_vec = global_vars.arm_command.angular
    lin_vel = [lin_vel_vec.x, lin_vel_vec.y, lin_vel_vec.z]
    ang_vel = [ang_vel_vec.x, ang_vel_vec.y, ang_vel_vec.z]

    ang_vel_loc = so3.apply(so3.inv(R), ang_vel)  # rotating ang vel to local frame

    # Transform ang vel into rpy vel
    euler_vel = transform_ang_vel(euler, ang_vel_loc)
    palm_vel = []
    palm_vel.extend(lin_vel)
    palm_vel.extend([euler_vel[0], euler_vel[1], euler_vel[2]])  # appending
    syn_vel = global_vars.hand_command

    # Integrating
    syn_next = syn_curr + syn_vel * int_const_syn * dt
    t_next = vectorops.madd(t, lin_vel, int_const_t * dt)
    euler_next = vectorops.madd(euler, euler_vel, int_const_eul * dt)

    # Saturating synergy
    if syn_next >= 1.0:
        syn_next = 1.0

    # Convert back for send xform
    palm_R_next = so3.from_rpy(euler_next)
    palm_t_next = t_next
    palm_next = (palm_R_next, palm_t_next)

    if DEBUG or True:
        print 'euler is ', euler, ' and is of type ', type(euler)
        print 'euler_vel is ', euler_vel, ' and is of type ', type(euler_vel)
        print 'euler_next is ', euler_next, ' and is of type ', type(euler_next)
        #
        # print 't is ', t, ' and is of type ', type(t)
        # print 't_next is ', t_next, ' and is of type ', type(t_next)
        #
        # print 'R is ', R, ' and is of type ', type(R)
        # print 'palm_R_next is ', palm_R_next, ' and is of type ', type(palm_R_next)
        #
        # print 'palm_curr is ', palm_curr, ' and is of type ', type(palm_curr)
        # print 'palm_next is ', palm_next, ' and is of type ', type(palm_next)

    return True, syn_next, palm_next, syn_vel, palm_vel


def transform_ang_vel(euler, ang_vel):
    """ Transforms an omega (ang. vel.) into rpy or ypr parametrization """

    r = euler[0]
    p = euler[1]
    y = euler[2]

    # Transformation matrix (ref. https://davidbrown3.github.io/2017-07-25/EulerAngles/)
    # YPR
    Typr = np.array([[1, sym.tan(p) * sym.sin(r), sym.cos(r) * sym.tan(p)],
                     [0, sym.cos(r), -sym.sin(r)],
                     [0, sym.sin(r) / sym.cos(p), sym.cos(r) / sym.cos(p)]])

    # RPY
    Trpy = np.array([[sym.cos(y) / sym.cos(p), -sym.sin(y) / sym.cos(p), 0],
                     [sym.sin(y), sym.cos(y), 0],
                     [-(sym.cos(y) * sym.sin(p)) / sym.cos(p), (sym.sin(p) * sym.sin(y)) / sym.cos(p), 1]])

    vec = np.array([ang_vel[0], ang_vel[1], ang_vel[2]])
    vec_transformed = np.matmul(Typr, vec)
    vec_tup = totuple(vec_transformed)  # conversion to tuple for xform

    return vec_tup


def totuple(a):
    # Auxiliary function for convertion to tuple
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a


def make(sim, hand, dt):
    """The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
    processing is needed when it is invoked."""

    sim.updateWorld()

    def control_func(controller):
        """
        Place your code here... for a more sophisticated controller you could also create a class
        where the control loop goes in the __call__ method.
        """

        # Integrating the velocities
        (success, syn_comm, palm_comm, syn_vel_comm, palm_vel_comm) = integrate_velocities(controller, sim, dt)

        # print 'The integration of velocity -> success = ', success

        t_lift = 1000.0
        lift_traj_duration = 0.5

        if sim.getTime() < t_lift:
            if success:
                if DEBUG:
                    print 'The commanded position of the hand encoder (in memory) is ', syn_comm
                    print 'The commanded position of the palm is \n', palm_comm
                    print 'The commanded velocity of the palm is \n', palm_vel_comm
                hand.setCommandVel([syn_comm], syn_vel_comm)
                mv_el.send_moving_base_xform_PID_vel(controller, palm_comm[0], palm_comm[1], palm_vel_comm)
                # mv_el.send_moving_base_xform_PID(controller, palm_comm[0], palm_comm[1])

        if sim.getTime() > t_lift:
            xform = mv_el.get_moving_base_xform(sim.controller(0).model())  # for later lifting
            # print 'xform is ', xform
            # the controller sends a command to the base after t_lift s to lift the object
            t_traj = min(1, max(0, (sim.getTime() - t_lift) / lift_traj_duration))
            desired = se3.mul((so3.identity(), [0, 0, 0.10 * t_traj]), xform)
            mv_el.send_moving_base_xform_PID(controller, desired[0], desired[1])

        # need to manually call the hand emulator
        hand.process({}, dt)

        if DEBUG:
            print "The simulation time is " + str(sim.getTime())

    return control_func
