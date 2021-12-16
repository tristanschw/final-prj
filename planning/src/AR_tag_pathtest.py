#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""

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

        """
        Main Script
        """
        # Process trans to get your state error
        # Make sure that you've looked at and understand path_planner.py before starting

        # Getting TF from Base to AR Tag for using the planner
        ################################################################################
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        r = rospy.Rate(10)
        
        ################################################################################


        planner = PathPlanner("right_arm")

        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

        # controller = Controller(Kp,Ki,Kd,Kw,Limb("right"))

        #
        # Add the table to the planning scene here
        #
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

        #Create a path constraint for the arm
        #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
        orien_const = OrientationConstraint()
        orien_const.link_name = "hand"
        orien_const.header.frame_id = "base"
        # quaternion_old = np.array([0.681, 0.149, 0.708, 0.111])
        quaternion = np.array([0.757, -0.033, 0.630, -0.171])
        orientation_constraint = quaternion/np.linalg.norm(quaternion)
        orien_const.orientation.x = orientation_constraint[0]
        orien_const.orientation.y = orientation_constraint[1]
        orien_const.orientation.z = orientation_constraint[2]
        orien_const.orientation.w = orientation_constraint[3]
        ##tilt_constraint.orientation = Quaternion(0.681, 0.147, 0.708, -0.113)
        #orien_const.orientation = Quaternion(0.681, 0.147, 0.708, -0.113)
        orien_const.absolute_x_axis_tolerance = 0.2
        orien_const.absolute_y_axis_tolerance = 0.2
        orien_const.absolute_z_axis_tolerance = 0.2
        #orien_const.absolute_w_axis_tolerance = 0.1
        orien_const.weight = 1.0

        trans_headcam_to_ar = tfBuffer.lookup_transform('base', 'ar_marker_10', rospy.Time())
        t= time.time() * 10**3
        rate = rospy.Rate(100)
        while (time.time() * 10**3) < t+500:  
        # while not rospy.is_shutdown():
            try:
                #TF should handle the math - double check trans_webcam_to_bsae.py

                #Post x,y,z coordinates 
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
                

                # #x, y, and z position
                
                goal_1.pose.position.x = trans_headcam_to_ar.transform.translation.x + goals[target_index][0] #PoseStamped.position.x
                goal_1.pose.position.y = trans_headcam_to_ar.transform.translation.y + goals[target_index][1]#PoseStamped.position.y
                goal_1.pose.position.z = trans_headcam_to_ar.transform.translation.z + goals[target_index][2] #PoseStamped.position.z

                # quaternion_old = np.array([0.681, 0.149, 0.708, 0.111])
                quaternion = np.array([0.757, -0.033, 0.630, -0.171])
                goal_orientation = quaternion/np.linalg.norm(quaternion)
                print(goal_orientation)
                goal_1.pose.orientation.x = goal_orientation[0]
                goal_1.pose.orientation.y = goal_orientation[1]
                goal_1.pose.orientation.z = goal_orientation[2]
                goal_1.pose.orientation.w = goal_orientation[3]


                # print(goal_1.pose.position.x)
                # print(goal_1.pose.position.y)
                # print(goal_1.pose.position.z)
                # print(trans_headcam_to_ar.transform.rotation.x)mmander.MoveGroupCommander(group_name)
                # print(trans_headcam_to_ar.transform.rotation.y)
                # print(trans_headcam_to_ar.transform.rotation.z)
                # print(trans_headcam_to_ar.transform.rotation.w)


                    # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal_1, [orien_const])

                pub2.publish("planner done") #planner should be done
                rate.sleep()
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break
            
            # while not rospy.is_shutdown():
            #     try:
            #         zero_pose = PoseStamped()
            #         zero_pose.header.frame_id = "base"

            #         zero_pose.pose.position.x = 1.152 #PoseStamped.position.x
            #         zero_pose.pose.position.y = 0.155 #PoseStamped.position.y
            #         zero_pose.pose.position.z = 0.319 #PoseStamped.position.z
            #         # adding 0.1 to offset for the fact that the marker box is higher than the paper AR tag
            #         zero_pose.pose.orientation.x = 0.541
            #         zero_pose.pose.orientation.y = -0.450
            #         zero_pose.pose.orientation.z = 0.549
            #         zero_pose.pose.orientation.w = -0.452
            #         plan = planner.plan_to_pose(zero_pose, [])
            #         if not planner.execute_plan(plan):
            #             raise Exception("Execution failed")
            #     except Exception as e:
            #         print e
            #         traceback.print_exc()
            #     else:
            #         break
            
            # while not rospy.is_shutdown():
            #     try:
            #         #TF should handle the math - double check trans_webcam_to_bsae.py

            #         #Post x,y,z coordinates 
            #         goal_1 = PoseStamped()
            #         goal_1.header.frame_id = "base"
                    

            #         # #x, y, and z position
            #         goal_1.pose.position.x = trans_headcam_to_ar.transform.translation.x #PoseStamped.position.x
            #         goal_1.pose.position.y = trans_headcam_to_ar.transform.translation.y #PoseStamped.position.y
            #         goal_1.pose.position.z = trans_headcam_to_ar.transform.translation.z + 0.16 #PoseStamped.position.z

            #         quaternion = np.array([0.681, 0.149, 0.708, 0.111])
            #         goal_orientation = quaternion/np.linalg.norm(quaternion)
            #         print(goal_orientation)
            #         goal_1.pose.orientation.x = goal_orientation[0]
            #         goal_1.pose.orientation.y = goal_orientation[1]
            #         goal_1.pose.orientation.z = goal_orientation[2]
            #         goal_1.pose.orientation.w = goal_orientation[3]


            #         # print(goal_1.pose.position.x)
            #         # print(goal_1.pose.position.y)
            #         # print(goal_1.pose.position.z)
            #         # print(trans_headcam_to_ar.transform.rotation.x)mmander.MoveGroupCommander(group_name)
            #         # print(trans_headcam_to_ar.transform.rotation.y)
            #         # print(trans_headcam_to_ar.transform.rotation.z)
            #         # print(trans_headcam_to_ar.transform.rotation.w)


            #             # Might have to edit this . . . 
            #         plan = planner.plan_to_pose(goal_1, [])

            #         pub2.publish("planner done") #planner should be done

            #         if not planner.execute_plan(plan):
            #             raise Exception("Execution failed")
            #     except Exception as e:
            #         print e
            #         traceback.print_exc()
            #     else:
            #         break

            # while not rospy.is_shutdown():
            #     try:
            #         goal_2 = PoseStamped()
            #         goal_2.header.frame_id = "base"

            #         #x, y, and z position
            #         goal_2.pose.position.x = trans_headcam_to_ar.transform.translation.x
            #         goal_2.pose.position.y = trans_headcam_to_ar.transform.translation.y + .5
            #         goal_2.pose.position.z = trans_headcam_to_ar.transform.translation.z + 0.21

            #         quaternion = np.array([0.681, 0.149, 0.708, 0.111])
            #         goal_orientation = quaternion/np.linalg.norm(quaternion)
            #         #Orientation as a quaternion
            #         goal_2.pose.orientation.x = goal_orientation[0]
            #         goal_2.pose.orientation.y = goal_orientation[1]
            #         goal_2.pose.orientation.z = goal_orientation[2]
            #         goal_2.pose.orientation.w = goal_orientation[3]

            #         plan = planner.plan_to_pose(goal_2, [])

            #         # raw_input("Press <Enter> to move the right arm to goal pose 2: ")
            #         if not planner.execute_plan(plan):
            #             raise Exception("Execution failed")
            #     except Exception as e:
            #         print e
            #         traceback.print_exc()
            #     else:
            #         break

                # while not rospy.is_shutdown():
                #     try:
                #         goal_3 = PoseStamped()
                #         goal_3.header.frame_id = "base"

                #         #x, y, and z position
                #         goal_3.pose.position.x = 0.6
                #         goal_3.pose.position.y = -0.1
                #         goal_3.pose.position.z = 0.1

                #         #Orientation as a quaternion
                #         goal_3.pose.orientation.x = 0.0
                #         goal_3.pose.orientation.y = -1.0
                #         goal_3.pose.orientation.z = 0.0
                #         goal_3.pose.orientation.w = 0.0

                #         plan = planner.plan_to_pose(goal_3, [])

                #         raw_input("Press <Enter> to move the right arm to goal pose 3: ")
                #         if not controller.execute_path(plan):
                #             raise Exception("Execution failed")
                #     except Exception as e:
                #         print e
                #     else:
                #         break

if __name__ == '__main__':
    target_index = int(sys.argv[1])
    print(target_index)
    rospy.init_node('moveit_node', anonymous=True)
    class_instance = Pub_and_Sub()
    try:
        class_instance.main(target_index)
    except rospy.ROSInterruptException:
        pass
    # main() previously just init node and then main()
