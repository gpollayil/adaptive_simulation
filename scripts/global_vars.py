# All the global variables are instantiated by calling the init function

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# For the subscriber to the robot commands
global hand_command
hand_command = Float64()
global arm_command
arm_command = Twist()

# For publishing the joint states, the touch data and the object pose and twist
global joints_pub
global touch_pub
global obj_pose_pub
global obj_twist_pub

# Service client for adaptive grasping and the bool stating if adaptive grasping is running or not
global adaptive_service_client
global is_adaptive_running

# For broadcasting the palm frame and object frame
global palm_broadcaster
global obj_broadcaster

# For sending lift trajectory correctly: post grasp pose is sensed only once
global got_pres_pose
got_pres_pose = False
