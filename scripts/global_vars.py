# All the global variables are instantiated by calling the init function

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# For the subscriber to the robot commands
global hand_command
hand_command = Float64()
global arm_command
arm_command = Twist()

# For publishing the joint states and the touch data
global joints_pub
global touch_pub

# For broadcasting the palm frame
global broadcaster
