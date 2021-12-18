#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
# This node was initially adapted from lab 7 and edited by Alex Chemali and Michelle Boulos then edits were made by Suphakorn and Aryaman
# to integrate the Allegro hand code.
# After running the joint tracjectories action server and the intera moveit launch file, this node passes the goal poses into the planner.
# It also accounts for the geometry of the table in front of the Sawyer as well as the orientation constraint for the path.

import tf2_ros
import tf
import sys
import rospy
import sensor_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker
# from ar_track_alvar import AlvarMarkers
from std_msgs.msg import Header, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import numpy as np
import time
import sys

ROBOT = "sawyer"

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

import rospy
import numpy as np
import traceback


from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, TransformStamped

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass
#predetermined offsets
goals = [[0, 0, 0.45], [-0.05, 0.42, 0.21], [-0.08, 0.42, 0.55], [-0.5, 0.42, 0.1], [-0.05,0.36,0.21], [-0.05,0.36,0.45], [0,0.14,0.21]]


class Pub_and_Sub():
    def __init__(self):
        self.pixel_offset = None

    def pixel_offset_cb(self, data):
        self.pixel_offset = data #should just be a string including the pixel offset and left or right

    

    def main(self, target_index):
        
        pub1 = rospy.Publisher('otc_chopsticks_topic', String, queue_size=10)
        pub2 = rospy.Publisher('control_topic', String, queue_size=10)
        sub = rospy.Subscriber('Moksh_Chopstick_VS', String, queue_size=10, callback=self.pixel_offset_cb)
        
        pixels_away = self.pixel_offset
        if pixels_away == "0.0":
            pub1.publish("calling open to close")
        else:
            print(pixels_away)

        # Getting TF from Base to AR Tag for using the planner
        ################################################################################
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        r = rospy.Rate(10)
        
        ################################################################################

        planner = PathPlanner("right_arm")

        # control constants (not used)
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

        # Add the table to the planning scene here
        table = PoseStamped()
        table.header.frame_id = "base"
        table.pose.position.x = 0.94
        table.pose.position.y = 0
        table.pose.position.z = -0.23
        table.pose.orientation.x = 0
        table.pose.orientation.y = 0
        table.pose.orientation.z = 0
        table.pose.orientation.w = 1
        planner.add_box_obstacle(np.array([0.457, 1.447, 0.1]), "table", table)

        # Create a path constraint for the arm
        orien_const = OrientationConstraint()
        orien_const.link_name = "hand"
        orien_const.header.frame_id = "base"
        quaternion = np.array([0.757, -0.033, 0.630, -0.171])
        orientation_constraint = quaternion/np.linalg.norm(quaternion)
        orien_const.orientation.x = orientation_constraint[0]
        orien_const.orientation.y = orientation_constraint[1]
        orien_const.orientation.z = orientation_constraint[2]
        orien_const.orientation.w = orientation_constraint[3]
        orien_const.absolute_x_axis_tolerance = 0.2
        orien_const.absolute_y_axis_tolerance = 0.2
        orien_const.absolute_z_axis_tolerance = 0.2
        orien_const.weight = 1.0

        # lookup the transform between base and ar_marker_10
        # tf tree updated from lab4_cam nodes
        trans_headcam_to_ar = tfBuffer.lookup_transform('base', 'ar_marker_10', rospy.Time())      
        t= time.time() * 10**3
        rate = rospy.Rate(100)
        
        while (time.time() * 10**3) < t+500:  
            try:
                # create the goal to be passed to the planner as a PoseStamped
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
                
                # AR tag x,y,z but offset by predetermined values
                goal_1.pose.position.x = trans_headcam_to_ar.transform.translation.x + goals[target_index][0]
                goal_1.pose.position.y = trans_headcam_to_ar.transform.translation.y + goals[target_index][1]
                goal_1.pose.position.z = trans_headcam_to_ar.transform.translation.z + goals[target_index][2]

                # create the quaternion for the goal and normalize it
                quaternion = np.array([0.757, -0.033, 0.630, -0.171])
                goal_orientation = quaternion/np.linalg.norm(quaternion)
                goal_1.pose.orientation.x = goal_orientation[0]
                goal_1.pose.orientation.y = goal_orientation[1]
                goal_1.pose.orientation.z = goal_orientation[2]
                goal_1.pose.orientation.w = goal_orientation[3]

                # plan to the goal position with the predefined orientation constraint
                plan = planner.plan_to_pose(goal_1, [orien_const])

                # publish a completion message
                pub2.publish("planner done") #planner should be done
                rate.sleep()
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break


if __name__ == '__main__':
    target_index = int(sys.argv[1])
    print(target_index)
    rospy.init_node('moveit_node', anonymous=True)
    class_instance = Pub_and_Sub()
    try:
        class_instance.main(target_index)
    except rospy.ROSInterruptException:
        pass
