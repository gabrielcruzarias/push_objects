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
import smach
import smach_ros



##############################################################
######### GLOBAL VARIABLES ###################################
##############################################################

found_object = False

v = 0.3 # Default velocity
w = 0.75 # Default angular velocity

obj_x = 1.0 # x of AR tag's pose
obj_y = 1.0 # y of AR tag's pose
obj_theta = 0.0 # theta of AR tag's pose

goalReached = None

boxGoal = [0, 1.5]

robotPos = [0, 0, 0]

##############################################################
##############################################################



###############################################################################################################################
######### HIGH LEVEL METHODS ########## HIGH LEVEL METHODS ########## HIGH LEVEL METHODS ########## HIGH LEVEL METHODS ########
###############################################################################################################################


# Uses the navigation stack to move the robot to the specified pose
def goToPose(frame, position, orientation): #frame, [x, y], [z, w]
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp.secs = rospy.get_time()

    goal.target_pose.pose.position.x = position[0]; goal.target_pose.pose.position.y = position[1]
    goal.target_pose.pose.orientation.z = orientation[0]; goal.target_pose.pose.orientation.w = orientation[1]
    #print "SENDING GOAL..." ###---------------------------------------------------------------------------------------------###
    nav.send_goal(goal)
    #print "DONE SENDING GOAL" ###-------------------------------------------------------------------------------------------###
    nav.wait_for_result(rospy.Duration.from_sec(5.0))


# Face the object
# INITIAL CONDITIONS: Robot is close to the object
# GOAL: The robot faces the object and the robot, object, and goal all lie on the same line (the robot can just move straight
# forward and it'll hit the object and eventually the goal)
def face(error):
    global obj_y
    obj_y = 1.0
    #print "STARTED FACE" ###------------------------------------------------------------------------------------------------###
    CENTER = 0.0
    w = 0.4
    start_t = time.time()
    TIME_TO_FAIL = 60 # Check this later, it shouldn't need to be this big
    while (obj_y < CENTER - error or obj_y > CENTER + error):
        t = time.time()
        while (obj_y < CENTER - error or obj_y > CENTER + error):
            if (time.time() - start_t > TIME_TO_FAIL):
                return "box_not_found"
            if (abs(obj_y) < 0.95):
                start_t = time.time()
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
    return "box_found"
        #if (obj_y < CENTER - error or obj_y > CENTER + error):
            #print "OVERSHOOT!!" ###-----------------------------------------------------------------------------------------###
    #print "DONE WITH FACE" ###----------------------------------------------------------------------------------------------###


# Goes to a point that lies on the same line as the object and the goal
# INITIAL CONDITIONS: The robot is close to the object (within ~1.50 meters of it)
# GOAL: The robot, object, and goal all lie on the same line
def goToIntermediateGoal():
    global boxGoal, robotPos
    (rx, ry, alpha) = robotPos
    g = boxGoal
    o = [rx + obj_x*math.cos(alpha) - obj_y*math.sin(alpha), ry + obj_x*math.sin(alpha) + obj_y*math.cos(alpha)]
    distance = math.sqrt((o[0] - g[0]) ** 2 + (o[1] - g[1]) ** 2)
    k = 0.8 / distance
    mg = [o[0] + k * (o[0] - g[0]), o[1] + k * (o[1] - g[1])] # middle goal
    goToPose("map", mg, [0, 1])
    #print "r_x =", rx, ", ry =", ry, ", alpha =", alpha / math.pi, " pi radians"
    #print "o_x =", o[0], ", o_y =", o[1]
    #print "mg_x =", mg[0], ", mg_y =", mg[1]
    return mg


# Approach the object. Ends when the bumper sensor is activated
# Assumes the robot is facing the object and within ~1.5 meters of it
def approach():
    global found_object
    #print "STARTED APPROACH" ###--------------------------------------------------------------------------------------------###
    start_t = time.time()
    TIME_TO_FAIL = 15 # This might not have to be this big
    result = 'box_reached'
    found_object = False
    while (not found_object):
        if (time.time() - start_t > TIME_TO_FAIL):
            result = 'box_not_reached'
            break
        twist.linear.x = v; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 # Look at the object's pose every time to adjust the angular velocity.
        pub.publish(twist)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    #print "DONE WITH APPROACH" ###------------------------------------------------------------------------------------------###
    time.sleep(2) # This is just to let me know that the robot is transitioning from approaching the object to pushing it.
    return result




###############################################################################################################################
######### SENSORS AND TOPICS ########## SENSORS AND TOPICS ########## SENSORS AND TOPICS ########## SENSORS AND TOPICS ########
###############################################################################################################################

def goalResult(MoveBaseActionResult):
    global goalReached
    goalReached = MoveBaseActionResult.status.status

# Gets the pose of the object relative to the robot (through the AR tags)
def objectPose(AlvarMarkers):
    global obj_x, obj_y, obj_theta
    if (len(AlvarMarkers.markers) > 0):
        obj_y = AlvarMarkers.markers[0].pose.pose.position.y
        obj_x = AlvarMarkers.markers[0].pose.pose.position.x
        obj_theta = getTheta(AlvarMarkers.markers[0].pose.pose.orientation.w, AlvarMarkers.markers[0].pose.pose.orientation.z)
        #print "y = ", obj_y, ", theta = ", obj_theta / math.pi, "*pi"

# This tells us when the robot is in contact with the object
def processSensing(BumperEvent):
    global found_object
    if (not found_object and BumperEvent.PRESSED == 1):
        found_object = True
    elif (found_object and BumperEvent.PRESSED == 0):
        found_object = False


def mapLocation(PoseWithCovarianceStamped):
    global robotPos # [x, y, theta]
    robotPos[0] = PoseWithCovarianceStamped.pose.pose.position.x
    robotPos[1] = PoseWithCovarianceStamped.pose.pose.position.y
    robotPos[2] = (getTheta(PoseWithCovarianceStamped.pose.pose.orientation.w, PoseWithCovarianceStamped.pose.pose.orientation.z) + 2*math.pi) % (2*math.pi)


###############################################################################################################################
########## LOW LEVEL METHODS ########## LOW LEVEL METHODS ########## LOW LEVEL METHODS ########## LOW LEVEL METHODS ###########
###############################################################################################################################

# Turns the turtlebot alpha radians
def turn(alpha):
    ERROR_TIME = 0.0 # This is needed because it'll take some time for the robot to accelerate. The ideal time will be determined experimentally (it might depend on alpha).
    t = time.time()
    while (time.time() - t < abs(alpha) / w + ERROR_TIME):
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = math.copysign(w, alpha)
        pub.publish(twist)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)


# Gets the yaw angle from the quaternion describing the object's position
def getTheta(w, z):
    mag = math.sqrt(w ** 2 + z ** 2)
    w /= mag
    z /= mag
    return math.atan2(2 * w * z, w ** 2 - z ** 2) #- math.pi / 2


# verifies that the input has the form "<float>, <float>"
def is_valid(inp):
    #TODO
    return True


# Returns the robot's distance to a given point in the map
def distanceTo(p):
    return math.sqrt((p[0] - robotPos[0])**2 + (p[1] - robotPos[1])**2)







##############################################################
######### STATES #############################################
##############################################################


# State WAIT_FOR_GOAL
class Wait_for_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['valid_goal','invalid_goal'])

    def execute(self, userdata):
        global goalReached, boxGoal
        rospy.loginfo('Executing state WAIT_FOR_GOAL')
        ACCEPTED_ERROR = 0.5 # Error allowed when goal was not reached (MoveBaseActionResult.status.status == 4 but the robot is close to the goal)
        inp = raw_input('Enter goal: x, y = ')
        if is_valid(inp):
            g = [float(x) for x in inp.split(',')]
            if (len(g) == 4):
                boxGoal = g[2:]
                g = g[:2]
            goToPose("map", g, [1, 0]) #frame, [x, y], [z, w]
            while (goalReached == None):
                time.sleep(1)
            if (goalReached == 4 and distanceTo(g) > ACCEPTED_ERROR):
                goalReached = None
                return 'invalid_goal'
            else:
                goalReached = None
                return 'valid_goal'
        else:
            return 'invalid_goal'


# State FIND_BOX
class Find_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['box_found', 'box_not_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FIND_BOX')
        transition = face(0.3)
        return transition


# State GO_TO_CORRECT_POSITION
class Go_to_correct_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['valid_goal', 'invalid_goal'])

    def execute(self, userdata):
        global goalReached
        rospy.loginfo('Executing state GO_TO_CORRECT_POSITION')
        ACCEPTED_ERROR = 0.2 # Error allowed when goal was not reached (MoveBaseActionResult.status.status == 4 but the robot is close to the goal)
        mg = goToIntermediateGoal()
        while (goalReached == None):
            time.sleep(1)
        if (goalReached == 4 and distanceTo(mg) > ACCEPTED_ERROR):
            goalReached = None
            return 'invalid_goal'
        else:
            goalReached = None
            return 'valid_goal'


# State FACE_BOX
class Face_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['box_found', 'box_not_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FACE_BOX')
        transition = face(0.03)
        return transition


# State APPROACH_BOX
class Approach_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['box_reached', 'box_not_reached'])

    def execute(self, userdata):
        rospy.loginfo('Executing state APPROACH_BOX')
        transition = approach()
        return transition


# State GO_TO_GOAL
class Go_to_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['valid_goal', 'invalid_goal'])

    def execute(self, userdata):
        global goalReached
        rospy.loginfo('Executing state GO_TO_GOAL')
        ACCEPTED_ERROR = 0.5 # Error allowed when goal was not reached (MoveBaseActionResult.status.status == 4 but the robot is close to the goal)
        goToPose("map", boxGoal, [0, 1]) # frame, point, orientation
        while (goalReached == None):
            time.sleep(1)
        if (goalReached == 4 and distanceTo(boxGoal) > ACCEPTED_ERROR):
            goalReached = None
            return 'invalid_goal'
        else:
            goalReached = None
            return 'valid_goal'


# State MOVE_BACK
class Move_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        global found_object
        rospy.loginfo('Executing state MOVE_BACK')
        t = time.time()
        while (time.time() - t < 2):
            twist.linear.x = -v; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        t = time.time()
        while (time.time() - t < 5):
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        found_object = False
        return 'done'



###############################################################################################################################
############### MAIN PROGRAM ############## MAIN PROGRAM ############## MAIN PROGRAM ############## MAIN PROGRAM ##############
###############################################################################################################################

if __name__=="__main__":
    rospy.init_node('smach_push_objects')

    pub = rospy.Publisher('~cmd_vel', Twist)
    nav = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #print "WAITING FOR SERVER..." ###---------------------------------------------------------------------------------------###
    nav.wait_for_server()
    #print "DONE WAITING FOR SERVER" ###-------------------------------------------------------------------------------------###

    twist = Twist()
    goal = MoveBaseGoal()

    rospy.Subscriber('/move_base/result', MoveBaseActionResult, goalResult)
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, objectPose)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, processSensing)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, mapLocation)

    transform = tf.TransformListener()
    rate = rospy.Rate(1.0)


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT_FOR_GOAL', Wait_for_goal(), 
                               transitions={'valid_goal':'FIND_BOX', 
                                            'invalid_goal':'WAIT_FOR_GOAL'})
        smach.StateMachine.add('FIND_BOX', Find_box(), 
                               transitions={'box_found':'GO_TO_CORRECT_POSITION',
                                            'box_not_found':'WAIT_FOR_GOAL'})
        smach.StateMachine.add('GO_TO_CORRECT_POSITION', Go_to_correct_position(), 
                               transitions={'valid_goal':'FACE_BOX',
                                            'invalid_goal':'FIND_BOX'})
        smach.StateMachine.add('FACE_BOX', Face_box(), 
                               transitions={'box_found':'APPROACH_BOX',
                                            'box_not_found':'WAIT_FOR_GOAL'})
        smach.StateMachine.add('APPROACH_BOX', Approach_box(), 
                               transitions={'box_reached':'GO_TO_GOAL',
                                            'box_not_reached':'FIND_BOX'})
        smach.StateMachine.add('GO_TO_GOAL', Go_to_goal(), 
                               transitions={'valid_goal':'MOVE_BACK',
                                            'invalid_goal':'MOVE_BACK'})
        smach.StateMachine.add('MOVE_BACK', Move_back(), 
                               transitions={'done':'WAIT_FOR_GOAL'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


    #t0 = time.time()
    #goToPose("map", [1.5, 0.6], [0, 1])
    #goToPose("base_footprint", [1, 0], [0, 1]) # frame, [x, y] (position), [z, w] (orientation quaternion)
    
    #while(True):
    #    if (not push_mode):
    #        time.sleep(100)
    #        print "",
            #twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            #pub.publish(twist) 

    # Return to navigation state

