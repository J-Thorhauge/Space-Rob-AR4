import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/johnbuntu/ar4_ros_driver/install/moveit_configs_utils'
