#!/usr/bin/env python
# left of chopsticks is green and right is blue
#
#
import rospy
from std_msgs.msg import String
import cv2
import time
import numpy as np


class rgb_segmentation():
    def __init__(self):
      pass

    def callback(self,data):
        print("in-callback  " + str(data))
        rans = self.return_ans()
        print("in-ans  " + str(rans))
        rospy.loginfo(rans)
        self.pub.publish(rans)
        return


    def verify_placemenent(self,green_coordinates, red_coordinates, blue_coordinates):
        if((green_coordinates[0][0] > red_coordinates[0][0]) and (red_coordinates[0][0] > blue_coordinates[0][0])):
            #this will tell if it is on the center.
            return 0
        else:
            if(blue_coordinates[0][0] > red_coordinates[0][0]):
            #off by to the left
                return  (blue_coordinates[0][0] - red_coordinates[0][0])
            elif(red_coordinates[0][0] > green_coordinates[0][0]):
                #off by to the right
                return red_coordinates[0][0] - green_coordinates[0][0]

    # 
    

    def return_ans(self):
        print("helo")
        cap = cv2.VideoCapture(0)
        # This drives the program to run for 3 seconds
        t_end = time.time() + 1

        while(time.time() < t_end):       
            # Captures the live stream frame-by-frame
            _, frame = cap.read()
            # Converts images from BGR to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_green = np.array([38,100,100])
            upper_green = np.array([75,255,255])

            lower_blue = np.array([100,50,50])
            upper_blue = np.array([120,255,255])

            lower_red = np.array([120, 150, 50])
            upper_red = np.array([180,255,255])
        
            # This creates a mask of green colored, blue colored, and red colored
            # objects found in the frame.
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            red_mask = cv2.inRange(hsv, lower_red, upper_red)



        
            # The bitwise and of the frame and mask is done so
            # that only the blue coloured objects are highlighted
            # and stored in res
            res_green = cv2.bitwise_and(frame,frame, mask= green_mask)
            res_red = cv2.bitwise_and(frame,frame, mask= red_mask)
            res_blue = cv2.bitwise_and(frame,frame, mask= blue_mask)


            cv2.imshow('frame',frame)
            cv2.imshow('mask',red_mask)
            cv2.imshow('res_green',res_green)
            cv2.imshow('res_red',res_red)
            cv2.imshow('res_blue', res_blue)
            # cv2.imshow('res_blue',res_bpub.publish(return_ans())
            count_green = cv2.countNonZero(green_mask)
            count_blue = cv2.countNonZero(blue_mask)
            count_red = cv2.countNonZero(red_mask)

            

            if((count_green > 0 ) and (count_blue > 0) and (count_red > 0)):
                # print("Tpub.publish(return_ans())hese are the coordinates for greeen: " )
                green_coordinates = np.average(cv2.findNonZero(green_mask),axis = 0)
                # print("These are the coordinates for blue: ") 
                blue_coordinates = np.average(cv2.findNonZero(blue_mask), axis = 0)
                # print("These are the coordinates for red: ") 
                red_coordinates = np.average(cv2.findNonZero(red_mask), axis = 0)

                if(self.verify_placemenent(green_coordinates, red_coordinates, blue_coordinates) == 0):
                    ans = "0.0"

                else:
                   # y_coor = self.verify_placemenent_y(green_coordinates, red_coordinates)
                    if(blue_coordinates[0][0] > red_coordinates[0][0]):
                        x_coor = self.verify_placemenent(green_coordinates, red_coordinates, blue_coordinates)
                        ans = str(x_coor)
                    else:
                        ans = str(self.verify_placemenent(green_coordinates, red_coordinates, blue_coordinates))
            else:
                ans = "None found"                                                                                                                                                                                 
        cv2.destroyAllWindows()
        cap.release()
        return ans

    def talker(self):
        self.pub = rospy.Publisher('Moksh_Chopstick_VS', String, queue_size=10)
        #this.sub = rospy.Subscriber('control_topic',String,queue_size = 10, callback = self.callback, callback_args = pub)
        self.sub = rospy.Subscriber('control_topic',String,queue_size = 10, callback = self.callback)
        
        
        rospy.spin()
   
if __name__ == '__main__':
    rospy.init_node('thresholding', anonymous=True)
    class_instance = rgb_segmentation()
    try: 
        class_instance.talker()
    except rospy.ROSInterruptException:
        pass