import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shreya/masterclass_construct/gazebo-ros2-bridge-project/ros2_ws/install/robot_description'
