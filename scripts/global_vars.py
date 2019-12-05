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

# Service client for adaptive grasping
global adaptive_service_client

# For broadcasting the palm frame and object frame
global palm_broadcaster
global obj_broadcaster

