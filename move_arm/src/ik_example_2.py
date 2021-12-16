#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper

def main(robo):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    arm = 'left'
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open() #init open 
    rospy.sleep(1.0)
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
    	arm = 'right'
    while not rospy.is_shutdown():
        command = raw_input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = arm + "_gripper"
        if robo == 'sawyer':
        	link += '_tip'


        if command == "pick":    
            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = 0.765
            request.ik_request.pose_stamped.pose.position.y = -0.243
            request.ik_request.pose_stamped.pose.position.z = -0.127      
            request.ik_request.pose_stamped.pose.orientation.x = 0
            request.ik_request.pose_stamped.pose.orientation.y = 1
            request.ik_request.pose_stamped.pose.orientation.z = 0
            request.ik_request.pose_stamped.pose.orientation.w = 0
        elif command == "pick up":
            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = 0.765
            request.ik_request.pose_stamped.pose.position.y = -0.243
            request.ik_request.pose_stamped.pose.position.z = -0.027     
            request.ik_request.pose_stamped.pose.orientation.x = 0
            request.ik_request.pose_stamped.pose.orientation.y = 1
            request.ik_request.pose_stamped.pose.orientation.z = 0
            request.ik_request.pose_stamped.pose.orientation.w = 0  
        elif command == "place up":  
            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = 0.685
            request.ik_request.pose_stamped.pose.position.y = -0.413
            request.ik_request.pose_stamped.pose.position.z = -0.027         
            request.ik_request.pose_stamped.pose.orientation.x = 0
            request.ik_request.pose_stamped.pose.orientation.y = 1
            request.ik_request.pose_stamped.pose.orientation.z = 0
            request.ik_request.pose_stamped.pose.orientation.w = 0
        elif command == "place":
            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = 0.685
            request.ik_request.pose_stamped.pose.position.y = -0.413
            request.ik_request.pose_stamped.pose.position.z = -0.127      
            request.ik_request.pose_stamped.pose.orientation.x = 0
            request.ik_request.pose_stamped.pose.orientation.y = 1
            request.ik_request.pose_stamped.pose.orientation.z = 0
            request.ik_request.pose_stamped.pose.orientation.w = 0
        elif command == "close":
            right_gripper.close()
            rospy.sleep(1.0)
            continue
        elif command == "open":
            right_gripper.open()
            rospy.sleep(1.0)
            continue
       



        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])

