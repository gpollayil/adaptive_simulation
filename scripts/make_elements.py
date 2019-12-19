"""
This auxiliary file contains the some fuctions used in main.py.
These functions build the object and the robot in the Klampt world
"""

from klampt.math import *

DEBUG = True

# Some global constants
path_prefix = '../../IROS2016ManipulationChallenge/'
config_prefix = '../config/'
moving_base_template_fn = path_prefix + 'data/robots/moving_base_template.rob'
robot_files = {
    'reflex_col': path_prefix + 'data/robots/reflex_col.rob',
    'soft_hand': path_prefix + 'data/robots/soft_hand.urdf',
    'reflex': path_prefix + 'data/robots/reflex.rob'
}
object_geom_file_patterns = {
    'apc2015': [path_prefix + 'data/objects/apc2015/%s/textured_meshes/optimized_tsdf_textured_mesh.ply']
}
simple_geom_file_pattern = path_prefix + 'data/objects/%s'
object_masses = {
    'apc2015': dict(),
}
default_object_mass = 0.1
object_template_fn = path_prefix + 'data/objects/object_template.obj'


def make_robot(robot_name, world):
    """Converts the given fixed-base robot into a moving base robot
    and loads it into the given world.
    """

    f = open(moving_base_template_fn, 'r')
    pattern = ''.join(f.readlines())
    f.close()

    f2 = open(config_prefix + "temp.rob", 'w')
    f2.write(pattern
             % (robot_files[robot_name], robot_name))
    f2.close()

    world.loadElement(config_prefix + "temp.rob")

    return world.robot(world.numRobots() - 1)


def make_object(object_set, object_name, world):
    """Adds an object to the world using its geometry / mass properties
    and places it in a default location (x,y)=(0,0) and resting on plane."""

    for pattern in object_geom_file_patterns[object_set]:
        obj_file = pattern % (object_name,)
        obj_mass = object_masses[object_set].get('mass', default_object_mass)

        f = open(object_template_fn, 'r')
        pattern = ''.join(f.readlines())
        f.close()

        f2 = open(config_prefix + "temp.obj", 'w')
        f2.write(pattern % (obj_file, obj_mass))
        f2.close()

        n_objs = world.numRigidObjects()
        if world.loadElement(config_prefix + 'temp.obj') < 0:
            continue
        assert n_objs < world.numRigidObjects(), "Hmm... the object didn't load, but loadElement didn't return -1?"

        obj = world.rigidObject(world.numRigidObjects() - 1)
        obj.geometry().scale(0.8, 1.1 , 1)
        obj.setTransform(*se3.identity())
        b_min, b_max = obj.geometry().getBB()

        T = obj.getTransform()
        spacing = 0.006
        T = (T[0], vectorops.add(T[1], (-(b_min[0] + b_max[0]) * 0.5, -(b_min[1] + b_max[1]) * 0.5, -b_min[2] + spacing)))

        obj.setTransform(*T)
        obj.appearance().setColor(0.2, 0.5, 0.7, 1.0)
        obj.setName(object_name)

        return obj
    raise RuntimeError("Unable to load object name %s from set %s" % (object_name, object_set))


def make_simple_object(object_name, world):
    """Adds a simple object to the world with defined geometry / mass properties
    and places it in a default location (x,y)=(0,0) and resting on plane."""

    obj_file = simple_geom_file_pattern % (object_name,)
    if not obj_file:
        raise RuntimeError("Unable to load object name %s from pattern %s" % (object_name, simple_geom_file_pattern))

    obj_mass = default_object_mass

    f = open(object_template_fn, 'r')
    pattern = ''.join(f.readlines())
    f.close()

    f2 = open(config_prefix + "temp.obj", 'w')
    f2.write(pattern % (obj_file, obj_mass))
    f2.close()

    n_objs = world.numRigidObjects()
    if world.loadElement(config_prefix + 'temp.obj') < 0:
        raise RuntimeError("Unable to load object name %s into world!" % object_name)
    assert n_objs < world.numRigidObjects(), "Hmm... the object didn't load, but loadElement didn't return -1?"

    obj = world.rigidObject(world.numRigidObjects() - 1)

    obj.geometry().scale(0.1, 0.025, 0.1)
    obj.setTransform(*se3.identity())
    b_min, b_max = obj.geometry().getBB()

    T = obj.getTransform()
    spacing = 0.002
    T = (T[0], vectorops.add(T[1], (-(b_min[0] + b_max[0]) * 0.5, -(b_min[1] + b_max[1]) * 0.5, -b_min[2] + spacing)))

    obj.setTransform(*T)
    obj.appearance().setColor(0.2, 0.5, 0.7, 1.0)
    obj.setName(object_name)

    return obj
