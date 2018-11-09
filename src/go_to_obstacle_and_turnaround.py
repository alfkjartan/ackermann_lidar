#! /usr/bin/env python
"""
Client node making use of the actions provided by the car behavior server. 
The available actions are 
   go_to_obstacle   ->  Makes the car turn and move toward the closest obstacle. 
                        It will stop at a distance to the obstacle given by the
                        goal message
   turn_car         ->  Turns the car at low speed until a desired angle to 
                        the obstacle is reached
"""
import numpy as np
import rospy
import actionlib

from ackermann_lidar.msg import GoToObstacleAction, TurnCarAction
from ackermann_lidar.msg import GoToObstacleGoal, TurnCarGoal

def go_to_obstacle_feedback(fb):
    """ Callback function to receive feedback during the execution of the action go_to_obstacle"""
    rospy.loginfo("Distance to go: %f" %fb.current_distance)

if __name__ == '__main__':
    rospy.init_node('avoid_obstacle_client')

    # First step is to move towards the obstacle, and stop at a distance in front of it
    
    # Create obstacle client and connect to server
    go_to_obstacle_client = actionlib.SimpleActionClient('go_to_obstacle', GoToObstacleAction)
    go_to_obstacle_client.wait_for_server()

    # Set the goal and send it to the server, wait for action to complete
    goal = GoToObstacleGoal()
    goal.distance = 1.5
    rospy.loginfo("Sending goal %f to go_to_obstacle" % goal.distance)
    go_to_obstacle_client.send_goal(goal, feedback_cb = go_to_obstacle_feedback)
    go_to_obstacle_client.wait_for_result(rospy.Duration.from_sec(25.0))

    # Get the result of the action and present it
    res = go_to_obstacle_client.get_result()
    if res == None:
      rospy.loginfo("Failed")
    else:
      rospy.loginfo("Got result %f" % res.actual_distance)


    # Step 2 is to turn the car, so that the obstacle is at a certain angle wrt the car
      
    turn_car_client = actionlib.SimpleActionClient('turn_car', TurnCarAction)
    turn_car_client.wait_for_server()

    goal = TurnCarGoal()
    goal.angle = np.pi/2

    rospy.loginfo("Sending goal %f to turn_car" % goal.angle)
    turn_car_client.send_goal(goal)
    turn_car_client.wait_for_result(rospy.Duration.from_sec(25.0))

    res = turn_car_client.get_result()
    rospy.loginfo("Got result %f" % res.actual_angle)
    

