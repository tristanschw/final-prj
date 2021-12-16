#!/usr/bin/env python
import rospy
import numpy as np
import traceback
import tf2_ros
import sys

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

#table obstacle 
table = PoseStamped()
table.header.frame_id = "base"
table.pose.position.x = 0.5
table.pose.position.y = 0.0
table.pose.position.z = -0.2
table.pose.orientation.x = 0.0
table.pose.orientation.y = 0.0
table.pose.orientation.z = 0.0
table.pose.orientation.w = 1.0
planner.add_box_obstacle(np.array([0.4, 0.8, 0.1]), "table", table)

orien_const = OrientationConstraint()
orien_const.link_name = "hand"                    #change this
orien_const.header.frame_id = "base"
orien_const.orientation.y = -1.0
orien_const.absolute_x_axis_tolerance = 0.1
orien_const.absolute_y_axis_tolerance = 0.1
orien_const.absolute_z_axis_tolerance = 0.1
orien_const.weight = 1.0

def tf_shit(ar_frame, goal_frame):
    #tfBuffer primed with Listener
    pub = rospy.Publisher('tf/visualization_marker', PoseStamped, queue_size=10)              #find out topic
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():

        trans = tfBuffer.lookup_transform(ar_frame, goal_frame, rospy.Time())

        tag_1 = PoseStamped()
        tag_1.header.frame_id = "base"
        tag_1.pose.position.x = trans.transform.translation.x
        tag_1.pose.position.y = trans.transform.translation.x
        tag_1.pose.position.z = trans.transform.translation.x
        tag_1.pose.orientation.x = trans.transform.rotation.qx
        tag_1.pose.orientation.y = trans.transform.rotation.qy
        tag_1.pose.orientation.z = trans.transform.rotation.qz
        tag_1.pose.orientation.w = trans.transform.rotation.qw
