#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used

#drive to a goal subscribed as /move_base_simple/goal
#def navToPose(goal):
#    print "spin!"
#    print "move!"
#	print "spin!"
#	print "done"
#	pass




#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(0.25,0.3)
    print "done1"
    rotate(45)
    print "done2"
    driveStraight(0.1,0.3)
    print "done3"
    rotate(90)
    print "done4"
    driveStraight(-.25,0.3)
    print "done"




#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub

    twist = Twist()
    time1 = 0.0
    while(time1<time):
        linear = (u1+u2)
        angular = (u1-u2)
        publishTwist(linear,angular)
        time1 += .1
        rospy.sleep(.1)
    publishTwist(0,0)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    spinWheels(speed,speed,distance/abs(speed))
 #   self._pub_cmd = rospy.Publisher('turtle1/cmd_vel',Twist)



    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
     # global odom_list
     # global pose
     # if (angle > 180 or angle<-180):
     #    print "Angle is too large"
     #    vel = Twist()
     #    done = True
     #    error = angle-math.degrees(pose.orientation.z)
     #
     #    if(angle > 0):
     #        turnCW = 1
     #    else:
     #        turnCW = -1
     #
     #    while((abs(error) >= 2) and not rospy.is_shutdown()):
     #        vel.angular.z = 0.25*turnCW
     #        error = angle - math.degrees(pose.orientation.z)
     #    vel.angular.z = 0.0
     #    pub.publish(vel)
     time1 = 0.0
     time2 = 4.0
     alpha = angle*0.0174533/2
     while(time1<=time2):
         publishTwist(0, alpha)
         time1 +=.1
         rospy.sleep(.1)
     publishTwist(0,0)


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented

def publishTwist(linearVelocity,angularVelocity):
    global pub
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
	global bump
	bump = True;
        print "Bumper pressed! Ayyy"
          # Delete this 'pass' once implemented

def getOdomData():
    sub = rospy.Subscriber('/odom',Odometry,odomCallBack)

def odomCallBack(data):
    px = data.pose.position.x
    py = data.pose.position.y
    quat = data.pose.orientation
    q = [quat.x,quat.y,quat.z,quat.w]
    roll,pitch,yaw = euler_from_quaternion(q)

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    global xPosition
    global yPosition
    global theta
    pose = Pose()

    odom_list.waitForTransform('map','base_footprint',rospy.Time(0),rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    pose.position.x = position[0]
    pose.position.y = position[1]
    xPosition = position[0]
    yPosition = position[1]

    odomW = orientation
    q = [odomW[0],odomW[1],odomW[2],odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    pose.orientation.z = yaw
    theta = math.degrees(yaw)
    pass # Delete this 'pass' once implemented




# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
    global bump 
   
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size = 10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"
    bump = False
    while(bump == False):
	bump = False

    executeTrajectory()



    #make the robot keep doing something...
    #rospy.Timer(rospy.Duration(publishTwist(1,1)), timerCallback)

    # Make the robot do stuff...
    print "Lab 2 complete!"

