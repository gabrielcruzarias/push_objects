#!/usr/bin/env python

# This script will make the robot push an object to a given point in a map.
# Initial conditions: Robot is within ~1.5 meters of the object.
# Goal: Push the object to a given goal point.

import roslib
import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar.msg import AlvarMarkers
from kobuki_msgs.msg import BumperEvent
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import sys, select, termios, tty
import time
import math
import actionlib
import tf

###############################################################################################################################
######### HIGH LEVEL METHODS ########## HIGH LEVEL METHODS ########## HIGH LEVEL METHODS ########## HIGH LEVEL METHODS ########
###############################################################################################################################

# Uses the navigation stack to move the robot to the specified pose
def goToPose(frame, position, orientation):
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp.secs = rospy.get_time()

    goal.target_pose.pose.position.x = position[0]; goal.target_pose.pose.position.y = position[1]
    goal.target_pose.pose.orientation.z = orientation[0]; goal.target_pose.pose.orientation.w = orientation[1]
    #print "SENDING GOAL..." ###---------------------------------------------------------------------------------------------###
    nav.send_goal(goal)
    #print "DONE SENDING GOAL" ###-------------------------------------------------------------------------------------------###
    nav.wait_for_result(rospy.Duration.from_sec(5.0))


# Starts the pushing mode and goes back to the navigation mode once it's done
# INITIAL CONDITIONS: Robot is close to the object (within ~1.50 meters of it)
# GOAL: The box is pushed to the desired goal, and the robot goes back to the navigation mode
def startPushingMode(MoveBaseActionResult):
    global push_mode, obj_x, obj_y, t0, t1, t2, t3, boxGoal
    obj_x = 1.0
    obj_y = 1.0
    #print "BASE ACTION ", MoveBaseActionResult.status.status
    if (MoveBaseActionResult.status.status == 3): # Navigation goal was reached
        if (not push_mode):
            t1 = time.time() - t0 # Time it takes to go to a point close enough to the box for the robot to see it
            push_mode = True
            #print "PUSH_MODE!!!!!!!!!!!!!!!!!" ###--------------------------------------------------------------------------###
            face(0.3)
            goToIntermediateGoal()
        else:
            t2 = time.time() - t0 # Time it takes to move to the position needed in order to push the box
            face(0.03)
            approach()
            time.sleep(5)
            goToPose("map", boxGoal, [0, 1]) # frame, point, orientation
            #push(distance)
            t3 = time.time() - t0 # Time it takes to push the box to its goal
            #print "BACK TO NAVIGATION MODE!!" ###---------------------------------------------------------------------------###
            #print "t1 =", t1, " (time to initial goal)"
            #print "t2 =", t2, " (time to correct position)"
            #print "t3 =", t3, " (time to final goal)"
            push_mode = False
    elif (MoveBaseActionResult.status.status == 4): # Report failure
        push_mode = False # Reset to non-pushing mode
        print MoveBaseActionResult.status.text


# Goes to a point that lies on the same line as the object and the goal
# INITIAL CONDITIONS: The robot is close to the object (within ~1.50 meters of it)
# GOAL: The robot, object, and goal all lie on the same line
def goToIntermediateGoal():
    global boxGoal, mapPose
    (rx, ry, alpha) = mapPose
    g = boxGoal
    o = [rx + obj_x*math.cos(alpha) - obj_y*math.sin(alpha), ry + obj_x*math.sin(alpha) + obj_y*math.cos(alpha)]
    print "r_x =", rx, ", ry =", ry, ", alpha =", alpha / math.pi, " pi radians"
    print "o_x =", o[0], ", o_y =", o[1]
    #g = transformGoal(boxGoal[0], boxGoal[1])
    #o = [obj_x, obj_y]
    distance = math.sqrt((o[0] - g[0]) ** 2 + (o[1] - g[1]) ** 2)
    k = 0.8 / distance
    mg = [o[0] + k * (o[0] - g[0]), o[1] + k * (o[1] - g[1])] # middle goal
    print "mg_x =", mg[0], ", mg_y =", mg[1]
    time.sleep(15)
    goToPose("map", mg, [0, 1])

# Transform the goal from /map frame to /base_footprint frame
def transformGoal(x, y):
    global transform
    j = 0
    x_r = 0
    y_r = 0
    for i in range(5):
        try:
            (trans, rot) = transform.lookupTransform('/map', 'base_footprint', rospy.Time(0))
            j += 1
            gx = x - trans[0]
            gy = y - trans[1]
            theta = getTheta(rot[3], rot[2])
            x_r += x*math.cos(theta) + y*math.sin(theta)
            y_r += -x*math.sin(theta) + y*math.cos(theta)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return (x_r/j, y_r/j)


# Face the object
# INITIAL CONDITIONS: Robot is close to the object
# GOAL: The robot faces the object and the robot, object, and goal all lie on the same line (the robot can just move straight
# forward and it'll hit the object and eventually the goal)
def face(error):
    global obj_y
    #print "STARTED FACE" ###------------------------------------------------------------------------------------------------###
    CENTER = 0.0
    w = 0.4
    ang_vel = 0
    while (obj_y < CENTER - error or obj_y > CENTER + error):
        t = time.time()
        while (obj_y < CENTER - error or obj_y > CENTER + error):
            if (time.time() - t > 6 and abs(obj_y) > 0.3):
                twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                pub.publish(twist)
                time.sleep(1.5)
                t = time.time()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = math.copysign(max(min(abs(obj_y), w), 0.2), obj_y)
            pub.publish(twist)
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        obj_y = math.copysign(1.0, obj_y)
        time.sleep(2)
        #if (obj_y < CENTER - error or obj_y > CENTER + error):
            #print "OVERSHOOT!!" ###-----------------------------------------------------------------------------------------###
    #print "DONE WITH FACE" ###----------------------------------------------------------------------------------------------###


# Approach the object. Ends when the bumper sensor is activated
# Assumes the robot is facing the object and within ~1.5 meters of it
def approach():
    global found_object
    #print "STARTED APPROACH" ###--------------------------------------------------------------------------------------------###
    while (not found_object):
        twist.linear.x = v; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 # Look at the object's pose every time to adjust the angular velocity.
        pub.publish(twist)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    #print "DONE WITH APPROACH" ###------------------------------------------------------------------------------------------###
    time.sleep(2) # This is just to let me know that the robot is transitioning from approaching the object to pushing it.


# Push object d meters
# Assumes the robot is already touching the object and
# facing in the direction it has to move the object
def push(d):
    ERROR_TIME = 0.2 # This is needed because it'll take some time for the robot to accelerate. The ideal time will be determined experimentally (it might depend on d).
    t = time.time()
    while (time.time() - t < 1.15 * d / v + ERROR_TIME):
        twist.linear.x = v; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)







###############################################################################################################################
########### GLOBAL VARIABLES ########### GLOBAL VARIABLES ########### GLOBAL VARIABLES ########### GLOBAL VARIABLES ###########
###############################################################################################################################

#distance = 0.3 # Distance (in meters) that the robot will push the object

found_object = False

v = 0.3 # Default velocity
w = 0.75 # Default angular velocity

obj_x = 1.0 # x of AR tag's pose
obj_y = 1.0 # y of AR tag's pose
obj_theta = 0.0 # theta of AR tag's pose

push_mode = False

#i = 0

t0 = 0
t1 = 0
t2 = 0
t3 = 0

mapPose = [0, 0, 0]

boxGoal = (0, 1.5)


###############################################################################################################################
############### MAIN PROGRAM ############## MAIN PROGRAM ############## MAIN PROGRAM ############## MAIN PROGRAM ##############
###############################################################################################################################

if __name__=="__main__":
    rospy.init_node('push_objects')

    pub = rospy.Publisher('~cmd_vel', Twist)
    nav = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #print "WAITING FOR SERVER..." ###---------------------------------------------------------------------------------------###
    nav.wait_for_server()
    #print "DONE WAITING FOR SERVER" ###-------------------------------------------------------------------------------------###

    twist = Twist()
    goal = MoveBaseGoal()

    rospy.Subscriber('/move_base/result', MoveBaseActionResult, startPushingMode)
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, objectPose)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, processSensing)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, mapLocation)

    transform = tf.TransformListener()
    rate = rospy.Rate(1.0)

    t0 = time.time()
    goToPose("map", [1.5, 0.6], [0, 1])
    #goToPose("base_footprint", [1, 0], [0, 1]) # frame, [x, y] (position), [z, w] (orientation quaternion)
    
    while(True):
        if (not push_mode):
            time.sleep(100)
            print "",
            #twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            #pub.publish(twist) 

    # Return to navigation state

