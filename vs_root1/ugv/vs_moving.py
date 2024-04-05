#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# topic
# /cmd_vel_diff: the command velocity that publish for the moving of the differential vehicle in its own frame
# /cmd_vel_enu: the feedforward velocity in the enu frame
# /cmd_vel_diff: in the vs_landing.cpp, we assume that the feed forward velocity is in the enu frame
# we need to transfer the velocity from the robot frame to the enu frame in this python script, and publish the topic /cmd_vel_enu

# Define global variables to store robot position
x_pos = 0.0

def groundTruth_callback(data):
    # Process the Odometry message
    # rospy.loginfo(rospy.get_caller_id()+"I heard %s", data)
    # Access the x position of the robot
    global x_pos
    x_pos = data.pose.pose.position.x

def circle():
    # Initialize node
    rospy.init_node('ugv_motion_control', anonymous=True)
    # subscribe the ground truth from the gazebo model
    sub = rospy.Subscriber('/ground_truth/diff', Odometry, groundTruth_callback)
    # Create publisher to publish Twist messages to the car's velocity topic
    # pub_husky = rospy.Publisher('/cmd_vel3', Twist, queue_size=10)
    # pub2 = rospy.Publisher('/cmd_vel_pad', Twist, queue_size=10)
    pub = rospy.Publisher('/cmd_vel_diff', Twist, queue_size=10)
    # pub = rospy.Publisher('/vs/object/cmd_vel', Twist, queue_size=10)
    # Set the publishing rate (in Hz)  run 10 times per second
    rate = rospy.Rate(10)
    # Create Twist message
    cmd_vel = Twist()
    # Set the linear velocity to move the car forward
    # parameters
    # vx = 0.15, wz = 0.1, R = 1.5
    # vx = 0.2, wz = 0.125, R = 1.6
    # vx = 0.2, wz = 0.2, R = 1.0
    # vx = 0.3, wz = 0.25, R = 1.2
    # vx = 0.3, wz = 0.3, R = 1.0
    # vx = 0.3, wz = 0.1, R = 3.0
    cmd_vel.linear.x = 0.15
    cmd_vel.angular.z = 0.1
    # cmd_vel.linear.x = 0.7 # Change this value to adjust the speed
    # Set the angular velocity to make the car turn
    # cmd_vel.angular.z = 0.0  # Change this value to adjust the turning rate
    cmd_vel_stop = Twist()
    cmd_vel_stop.linear.x = 0.0
    cmd_vel_stop.angular.z = 0.0
    # Publish the Twist message until the node is stopped
    while not rospy.is_shutdown():
        if x_pos <=100:
            pub.publish(cmd_vel)
            # pub2.publish(cmd_vel)
            # pub_husky.publish(cmd_vel)
        else:
            print("x_pos out of limit, stopping the ugv...")
            pub.publish(cmd_vel_stop)
        rate.sleep()

if __name__ == '__main__':
    try:
        circle()
    except rospy.ROSInterruptException:
        pass
