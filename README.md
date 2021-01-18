# mgbot3
This is a DIY movable manipulator simulation based on ROS. This robot can be used for test the techneques about motion, manipulation and perception.
This is a priliminary platfom for beginner in ROS. I hope you can enjoy it.O(∩_∩)O

#Version1.0 2021.1.18
Before using this you need to put these files in your work space and install the following packages: gmapping, navigation, AMCL, openCV, Moveit......
I recommand you to use sudo apt-get install to install these packages.

Functions and operations:

#Motion#
move robot by keyboard:run gazebo_xacro.launch and mgbot3_teleop.launch in mgbot3
SLAM(gmapping):run gazebo_xacro.launch and gmapping.launch in mgbot3
Navigation;run gazebo_xacro.launch and nav.launch in mgbot3
(you can build your own maps just referring to the tutorial of gmapping)

#Manipulation#
Simply operate manipulator: run demo_gazebo in mgbot3_moveit_config

#Perception#
motion detection(openCV):run gazebo_xacro.launch and motion_detector.launch in mgbot3(you have to let robot move)

You can also realize manipulation, navigation and motion detection simultaniously by runningdemo_gazebo.launch, nav.launch and motion_detector.launch

There must be some bugs but I will fix it as soon as possible. In the next version, I will bring more interesting functions.
