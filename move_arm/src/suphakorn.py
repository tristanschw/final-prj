import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper

import tf2_ros

def main(robo):

    #Suphakorn: Getting the Transform to the AR tag
    # tfBuffer = tf2_ros.Buffer()
    # while not rospy.is_shutdown():
    #     try:
    #         transform = tfBuffer.lookup_transform("base_link", "ar_marker_0", rospy.Time())
    #         print(trans)
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         pass



    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    #rospy.init_node('gripper_test')
    right_gripper = robot_gripper.Gripper('right')
    arm = 'left'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
        arm = 'right'
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        
        # print('Calibrating...')
        # right_gripper.calibrate()
        # rospy.sleep(2.0)
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = arm + "_gripper"
        if robo == 'sawyer':
            link += '_tip'

        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 50
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.803
        request.ik_request.pose_stamped.pose.position.y = -0.201
        request.ik_request.pose_stamped.pose.position.z = -0.122       
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        # request.ik_request.pose_stamped.pose.position.x = transform.translation.x
        # request.ik_request.pose_stamped.pose.position.y = transform.translation.y
        # request.ik_request.pose_stamped.pose.position.z = transform.translation.z       
        # request.ik_request.pose_stamped.pose.orientation.x = transform.rotation.x
        # request.ik_request.pose_stamped.pose.orientation.y = transform.rotation.y
        # request.ik_request.pose_stamped.pose.orientation.z = transform.rotation.z
        # request.ik_request.pose_stamped.pose.orientation.w = transform.rotation.w

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


        
        # request.ik_request.pose_stamped.pose.position.x = 0.792
        # request.ik_request.pose_stamped.pose.position.y = -0.235
        # request.ik_request.pose_stamped.pose.position.z = 0.067 
        # try:
        #     # Send the request to the service
        #     response = compute_ik(request)
            
        #     # Print the response HERE
        #     print(response)
        #     group = MoveGroupCommander(arm + "_arm")

        #     # Setting position and orientation target
        #     group.set_pose_target(request.ik_request.pose_stamped)

        #     # TRY THIS
        #     # Setting just the position without specifying the orientation
        #     ###group.set_position_target([0.5, 0.5, 0.0])

        #     # Plan IK and execute
        #     group.go()
            
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e


# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])