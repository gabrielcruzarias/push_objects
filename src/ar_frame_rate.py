#!/usr/bin/env python

import roslib
import rospy
from ar_track_alvar.msg import AlvarMarkers
import sys, select, termios, tty
import time
import actionlib

def objectPose(AlvarMarkers):
    global first_t, n
    if (len(AlvarMarkers.markers) > 0):
        if (first_t == 0):
            first_t = time.time()
        else:
            n += 1
            if (n % 10 == 0):
                print "time =", time.time() - first_t
                print "frame rate =", n / (time.time() - first_t)


first_t = 0
n = 1

if __name__=="__main__":
    rospy.init_node('ar_frame_rate')
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, objectPose)
    time.sleep(60)
