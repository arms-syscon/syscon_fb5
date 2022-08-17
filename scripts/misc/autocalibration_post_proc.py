#!/usr/bin/env python3

import rospy
import rospkg
import glob



dirname = rospkg.RosPack().get_path('syscon_fb5')
calib_folder = dirname + '/calibration_files'

calib_files = glob.glob(calib_folder + '/pos_*.csv')


