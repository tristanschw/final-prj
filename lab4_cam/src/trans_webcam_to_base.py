#!/usr/bin/env python

# This node was written by Alex Chemali
# It looks up the tf between the base and headcam of the sawyer and subscribes to the visualization marker
# topics being publised by the AR tracking nodes. It then matrix multiplies the poses and broadcasts the 
# result to the tf static tree. Effectively calibrating the position between the base and webcam. 

import tf2_ros
import tf
import sys
import rospy
import sensor_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker
# from ar_track_alvar import AlvarMarkers
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import numpy as np
import time

# Helper Functions
# were determined to be needed and written with the help of this post:
# https://stackoverflow.com/questions/55150211/how-to-calculate-relative-pose-between-two-objects-and-put-them-in-a-transformat
###########################################################################################

def PoseStamped_2_mat(p): # convert posestamped to matrix (leaves out header)
    q = p.pose.orientation
    pos = p.pose.position
    T = quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = np.array([pos.x,pos.y,pos.z])
    return T

def TransformStamped_2_mat(p): # convert posestamped to matrix (leaves out header)
    q = p.rotation
    pos = p.translation
    T = quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = np.array([pos.x,pos.y,pos.z])
    return T

def Pose_2_mat(p): # convert posestamped to matrix (leaves out header)
    q = p.orientation
    pos = p.position
    T = quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = np.array([pos.x,pos.y,pos.z])
    return T

def Mat_2_posestamped(m,f_id="base"): # convert matrix back to posestamped (include header again)
    q = quaternion_from_matrix(m)
    p = PoseStamped(header = Header(frame_id=f_id), #robot.get_planning_frame()
                    pose=Pose(position=Point(*m[:3,3]), 
                    orientation=Quaternion(*q)))
    return p    

def T_inv(T_in):
    R_in = T_in[:3,:3]
    t_in = T_in[:3,[-1]]
    R_out = R_in.T
    t_out = -np.matmul(R_out,t_in)
    return np.vstack((np.hstack((R_out,t_out)),np.array([0, 0, 0, 1])))

##########################################################################################


#Class
##########################################################################################
class Pose_from_AR(object):
	def __init__(self):
		self.marker_pose_webcam = None
		self.marker_pose_headcam = None     
		self.startTimeWebcam = None
		self.startTimeHeadcam = None   

	def marker_pose_webcam_cb(self, data): #need this callback function to extract the data for Subscriber
		self.marker_pose_webcam = data.pose
		if self.startTimeWebcam is None:
			self.startTimeWebcam = time.time()
		# print(data.pose)

	def marker_pose_headcam_cb(self, data): #need this callback function to extract the data for Subscriber
		self.marker_pose_headcam = data.pose
		if self.startTimeHeadcam is None:
			self.startTimeHeadcam = time.time()
		# print(data.pose)

	def webcam_to_base(self):
		# Subscribe to the both topics that hold the pose of the AR tag with reference from the webcam and headcam
		sub1 = rospy.Subscriber('visualization_marker', Marker, queue_size=10, callback = self.marker_pose_webcam_cb)
		sub2 = rospy.Subscriber('visualization_marker_sawyer', Marker, queue_size=10, callback = self.marker_pose_headcam_cb)
		
		# Create a tf listener with a buffer
		tfBuffer = tf2_ros.Buffer()
		tfListener = tf2_ros.TransformListener(tfBuffer)
		r = rospy.Rate(10)

		numSent = 0
		numReceived = 0

		while not rospy.is_shutdown():
			try:		

				#skip this iteration if there are no poses being read from either AR tracking nodes
				if (self.marker_pose_webcam is None or self.marker_pose_webcam is None):
					continue

				#check if we are reciving poses from both the AR marker topics and 10 seconds has passed for both of them to start calibration
				if self.marker_pose_webcam is not None and self.marker_pose_headcam is not None and (time.time()-self.startTimeWebcam)>10 and (time.time()-self.startTimeHeadcam)>10:
					# print('TRUE')
					numReceived = numReceived + 1
					
					# lookup tf between the base and head_camera (static)
					trans_headcam_to_base = tfBuffer.lookup_transform('base', 'head_camera', rospy.Time())

					webcam_ar_pose = self.marker_pose_webcam
					headcam_ar_pose = self.marker_pose_headcam
					
					# extracting the poses from each message type using the helper functions
					T_webcam_to_ar = Pose_2_mat(webcam_ar_pose)
					T_headcam_to_ar = Pose_2_mat(headcam_ar_pose)
					T_headcam_to_base = TransformStamped_2_mat(trans_headcam_to_base.transform)

					# correct matrix multiplication in order to get right pose from the base to usb_cam
					T_webcam_to_base = np.linalg.inv(np.matmul(np.matmul(T_webcam_to_ar, np.linalg.inv(T_headcam_to_ar)), np.linalg.inv(T_headcam_to_base)))
					
					# extract the quaternion to convert to rotation matrix
					quaternion_T_webcam_to_base = quaternion_from_matrix(T_webcam_to_base)

					# create the TransformStamped message to be published
					tf_webcam_to_base = TransformStamped()
					tf_webcam_to_base.header.frame_id = 'base'
					tf_webcam_to_base.child_frame_id = 'usb_cam'
					tf_webcam_to_base.transform.translation.x = T_webcam_to_base[0,3]
					tf_webcam_to_base.transform.translation.y = T_webcam_to_base[1,3]
					tf_webcam_to_base.transform.translation.z = T_webcam_to_base[2,3]
					tf_webcam_to_base.transform.rotation.x = quaternion_T_webcam_to_base[0]
					tf_webcam_to_base.transform.rotation.y = quaternion_T_webcam_to_base[1]
					tf_webcam_to_base.transform.rotation.z = quaternion_T_webcam_to_base[2]
					tf_webcam_to_base.transform.rotation.w = quaternion_T_webcam_to_base[3]
					
					# print to terminal
					print('Publish {} {}: {}'.format(numSent, numReceived, tf_webcam_to_base))
					
					# stop broadcasting after 10 times
					if (numSent < 10):
						br = tf2_ros.StaticTransformBroadcaster() #tf.TransformBroadcaster()
						br.sendTransform(tf_webcam_to_base)
						numSent = numSent + 1

				else:
					#print('else {} {}'.format(time.time()-self.startTimeHeadcam, time.time()-self.startTimeWebcam))

					continue

				

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				print(e)

			r.sleep()

if __name__ == '__main__':
    
    rospy.init_node('trans_webcam_to_base', anonymous=True)

    class_instance = Pose_from_AR()

    try:
    	class_instance.webcam_to_base()
    except rospy.ROSInterruptException:
		pass

