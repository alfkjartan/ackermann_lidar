#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Point  
from ackermann_msgs.msg import AckermannDriveStamped

Ka = 2
Kd = 0.2


def callback(point):
    # If distance to the obstacle is less than a meter, stop
    dist = point.z
    speed = 0
    theta = 0
    
    if (dist > 1) :
        # Steer towards the obstacle
        theta = min(0.4, max(-0.4, Ka*np.arctan2(point.y, point.x))) # Clamped to +/- 27deg
        speed = (dist-1)*Kd;

    rospy.loginfo("Sending command speed=%f, steering_angle=%f", speed, theta);

    am = AckermannDriveStamped()
    am.drive.steering_angle = theta
    am.drive.speed = speed

    am_pub.publish(am)


rospy.init_node("topic_subscriber")

am_pub = rospy.Publisher("/ackermann_vehicle/ackermann_cmd", AckermannDriveStamped, queue_size=4)
sub = rospy.Subscriber("/obstacle", Point, callback)

rospy.spin()
