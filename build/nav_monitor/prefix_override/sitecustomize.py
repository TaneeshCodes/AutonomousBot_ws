import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/taneesh/Nav2_Assessment_ws/install/nav_monitor'
