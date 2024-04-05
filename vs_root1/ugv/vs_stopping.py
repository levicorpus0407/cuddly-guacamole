#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
# topic
# /cmd_vel_diff: the command velocity that publish for the moving of the differential vehicle in its own frame
# /cmd_vel_enu: the feedforward velocity in the enu frame
# /cmd_vel_diff: in the vs_landing.cpp, we assume that the feed forward velocity is in the enu frame
# we need to transfer the velocity from the robot frame to the enu frame in this python script, and publish the topic /cmd_vel_enu
# This node is to stop the robot after the drone has landed.

def circle():
    # Initialize node
    rospy.init_node('prometheus_gazebo', anonymous=True)
    # Create publisher to publish Twist messages to the car's velocity topic
    # pub_husky = rospy.Publisher('/cmd_vel3', Twist, queue_size=10)
    # pub2 = rospy.Publisher('/cmd_vel_pad', Twist, queue_size=10)
    pub = rospy.Publisher('/cmd_vel_diff', Twist, queue_size=10)
    # Set the publishing rate (in Hz)
    rate = rospy.Rate(10)
    # Create Twist message
    cmd_vel = Twist()
    # Set the linear velocity to move the car forward
    cmd_vel.linear.x =  0.0 # Change this value to adjust the speed
    # Set the angular velocity to make the car turn
    cmd_vel.angular.z = 0.0  # Change this value to adjust the turning rate
    # Publish the Twist message until the node is stopped
    while not rospy.is_shutdown():
        # pub.publish(cmd_vel)
        # pub2.publish(cmd_vel)
        pub.publish(cmd_vel)
        # pub_husky.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        circle()
    except rospy.ROSInterruptException:
        pass
