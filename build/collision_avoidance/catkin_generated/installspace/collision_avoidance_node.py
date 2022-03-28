#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import signal, sys

from wp_gen_collision_avoidance import WaypointGen
def signal_handler(sig, frame):
    print('crash_detector_node You pressed Ctrl+C!')
    rospy.signal_shutdown('Ctrl+C')
    sys.exit(0)

def main():
    print("the right function is running")
    #initialize ROS
    rospy.init_node('collision_avoidance', disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)
    
    wp_gen = WaypointGen()
    
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
        wp_gen.step()
        r.sleep()

#this is where collision avoidance code all starts
if __name__=="__main__":
    main()
