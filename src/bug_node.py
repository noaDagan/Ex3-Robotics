#!/usr/bin/python
#
# bug_node.py
#
#Noa Dagan 311302137
#Nofar Bar-zakai 205643638


import rospy, sys
from bug import Bug

if __name__ == "__main__":
    #rospy.init_node("bug_node", argv=sys.argv)
    forward_speed = 0.5
    rotate_speed = 20
    x_goal = -7
    y_goal = -5
    min_dist_from_obstacle = 1.2
    if rospy.has_param('~forward_speed'):
        forward_speed = rospy.get_param('~forward_speed')
    if rospy.has_param('~rotate_speed'):
        forward_speed = rospy.get_param('~rotate_speed')
    if rospy.has_param('~min_dist_from_obstacle'):
        min_dist_from_obstacle = rospy.get_param('~min_dist_from_obstacle')
    if rospy.has_param('~x_goal'):
        x_goal = rospy.get_param('~x_goal')
    if rospy.has_param('~y_goal'):
        y_goal = rospy.get_param('~y_goal')
    my_bug = Bug(forward_speed,rotate_speed,min_dist_from_obstacle,x_goal,y_goal)
    my_bug.start()
