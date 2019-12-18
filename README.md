# adaptive_simulation
#### George Jose & Mathew Jose Pollayil ###
#### 18/12/2019 ###

This package contains the simulation framework for the [adaptive grasping approach](https://github.com/gpollayil/adaptive_grasping), implemented using Klampt.

### Requirements
This package requires Ubuntu 18.04 with ROS Melodic and Klampt 0.7 or above.
Obviously, the above mentioned package is required with all its needed third packages. Moreover, the package [soft_hand_klampt](https://github.com/gpollayil/soft_hand_klampt) and the fork of the [IROS 2016 Grasping and Manipulation Challenge Package](https://github.com/gpollayil/IROS2016ManipulationChallenge) should be present in the same src folder.
Download the object dataset using the instructions provided in the above mentioned package (will be downoladed in IROS2016 package).

### Run the simulation
1. `roslaunch soft_hand_klampt_description floating_soft_hand_klampt.launch` (Launches floating SoftHand robot in RViz)
2. `rosrun adaptive_simulation main_ros.py apc2015 stanley_66_052 ` (Launches Klampt and waits for service adaptive_grasping)
3. `roslaunch adaptive_grasping launchRobotCommAdaptiveGrasp.launch ` (The main server for adaptive grasping, and launches robotCommander... The service is called in the above python node)
4. `rostopic pub -r 50 /touching_finger_topic std_msgs/Int8 "data: 4"` (From 0 to 5 for fingers from thumb to pinky)
