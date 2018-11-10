#! /usr/bin/env python
"""
Server node implementing the following actions:
   go_to_obstacle   ->  Makes the car turn and move toward the closest obstacle. 
                        It will stop at a distance to the obstacle given by the
                        goal message
   turn_car         ->  Turns the car at low speed until a desired angle to 
                        the obstacle is reached
"""

import numpy as np
import rospy
from geometry_msgs.msg import Point  
from ackermann_msgs.msg import AckermannDriveStamped
import actionlib

from ackermann_lidar.msg import GoToObstacleAction, TurnCarAction
from ackermann_lidar.msg import GoToObstacleFeedback, TurnCarFeedback
from ackermann_lidar.msg import GoToObstacleResult, TurnCarResult


class ObstacleAvoiderServer:
  def __init__(self):
    self.go_to_obstacle_server = actionlib.SimpleActionServer('go_to_obstacle', GoToObstacleAction,
                        self.executeGoToObstacle, False)
    self.go_to_obstacle_server.start()

    self.turn_car_server = actionlib.SimpleActionServer('turn_car', TurnCarAction,
                        self.executeTurnCar, False)
    self.turn_car_server.start()

    self.goal_distance = -1
    self.distance_converged = False
    self.current_distance = -1

    self.goal_angle = 3.14
    self.angle_to_obstacle = 0
    
  def executeGoToObstacle(self, goal):
    am_pub = rospy.Publisher("/ackermann_vehicle/ackermann_cmd", AckermannDriveStamped, queue_size=4)
    sub = rospy.Subscriber("/obstacle", Point,
                           self.goToObstacleCallback, am_pub)

    fb = GoToObstacleFeedback()

    self.goal_distance = goal.distance
    while (not self.distance_converged) :
      rospy.sleep(0.1)
      fb.current_distance = self.current_distance
      self.go_to_obstacle_server.publish_feedback(fb)

    sub.unregister()
    
    res = GoToObstacleResult()
    res.actual_distance = self.current_distance
    
    self.go_to_obstacle_server.set_succeeded(result=res)

  def executeTurnCar(self, goal):
    am_pub = rospy.Publisher("/ackermann_vehicle/ackermann_cmd", AckermannDriveStamped, queue_size=4)
    sub = rospy.Subscriber("/obstacle", Point, self.turnAroundCallback, am_pub)

    self.goal_angle = goal.angle

    # Turn until the angle to the obstacle matches the goal (+/- 2 degrees)
    while (np.abs(self.angle_to_obstacle -self.goal_angle) > 2.0*np.pi/180.0) :
      rospy.sleep(0.01)

    sub.unregister()

    # Stop the car
    am = AckermannDriveStamped()
    am.drive.steering_angle = 0
    am.drive.speed = 0
    am.drive.acceleration = 1.0
    am_pub.publish(am)

    
    res = TurnCarResult()
    res.actual_angle = self.angle_to_obstacle
    self.turn_car_server.set_succeeded(result=res)

    
  def goToObstacleCallback(self, point, pub):
    # Hardcoded controller gains
    Ka = 2
    Kd = 0.1

    dist_change = np.abs(self.current_distance - point.z)
    distance_to_go = point.z - self.goal_distance
    
    self.current_distance = point.z

    speed = 0
    theta = 0
    
    # If change in distance is less than 2mm or 2cm from goal consider converged
    #if ( dist_change < 0.002 or distance_to_go < 0.02) :
    if ( distance_to_go < 0.02) :
      self.distance_converged = True
      am = AckermannDriveStamped()
      am.drive.steering_angle = 0
      am.drive.speed = 0
      am.drive.acceleration = 1.0
      pub.publish(am)
      return

    
    if not self.distance_converged :
      # Steer towards the obstacle
      theta = min(0.4, max(-0.4, Ka*np.arctan2(point.y, point.x))) # Clamped to +/- 27deg
      speed = distance_to_go*Kd + 0.3;

      #rospy.loginfo("Moving toward obstacle with speed=%f, steering_angle=%f", speed, theta);

      am = AckermannDriveStamped()
      am.drive.steering_angle = theta
      am.drive.speed = speed
      am.drive.acceleration = 1.0
      am.drive.jerk = 1.0

      pub.publish(am)

  def turnAroundCallback(self, point, pub):

    speed = 0.4

    self.angle_to_obstacle = np.arctan2(point.y, point.x)
    if (self.goal_angle - self.angle_to_obstacle) > 0:
      # Turn right
      theta = -0.4
    else:
      # Turn left
      theta = 0.4
      
    #rospy.loginfo("Turning away from obstacle with speed=%f, steering_angle=%f", speed, theta);

    am = AckermannDriveStamped()
    am.drive.steering_angle = theta
    am.drive.speed = speed
    am.drive.acceleration = 1.0

    pub.publish(am)

if __name__ == '__main__':
  rospy.init_node('avoid_obstacle_server')
  server = ObstacleAvoiderServer()
  rospy.spin()
