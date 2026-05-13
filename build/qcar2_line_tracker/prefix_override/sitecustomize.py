import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hajisaeed/qcar2_sim/install/qcar2_line_tracker'
