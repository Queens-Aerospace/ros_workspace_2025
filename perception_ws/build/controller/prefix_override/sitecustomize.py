import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/em/keshavTest/ros_workspace_2025/perception_ws/install/controller'
