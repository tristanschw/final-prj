#!/usr/bin/env python
# license removed for brevity

# This node was a work in progress, unfortuneately we were not able to fully integrate it.
# The idea was that it would run by iteself based on messages it recieved from other nodes.

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np
import os

class open_to_close():
    def __init__(self):
        steps = 50
        #open = np.array([-0.37716148727445675, 0.7910325023333449, 0.7579606877172846, -0.02417521716086561, -0.06404217643918661, 1.1058299215313325, 1.0336055713211791, 0.4762021248871414, 0.10795021891623754, 1.280400397732933, 0.8828298818818983, 0.33095825443927385, 1.5127420418285396, 0.22672975046925622, 0.33077336982428873, 0.502304818786258])
        open = np.array([-0.4544882180231765, 0.9385368696393318, 0.7390912269552227, -0.06894226970355727, -0.3127687481649788, 1.108102715282322, 1.0350556698657738, 0.48760094361757855, -0.1891814034481676, 1.2768303235180714, 0.8834499558975041, 0.34283025457759353, 1.5148682361396588, 0.25393788015816987, 0.387205266483693, 0.46550924371423])

        closed = np.array([-0.13563331826143926, 1.2312634750831006, 0.9069657411016043, -0.1100786085291394, -0.1791499542428171, 1.6981353702790094, 1.0011153309179666, 0.5001013630769563, 0.022603119476372432, 1.7265652716472935, 1.2910120968193919, -0.2774761978172064, 1.4072040639865064, -0.0005677437280994912, 0.18899816764141025, 0.7607931989892316])
        positions = [(((closed - open)/steps * i) + open) for i in range(steps)]

        initial_torque = np.array([0.05,0.6,0.4,0.1 ,0.1,0,0,0, 0,0,0,0,0,0.0,0.7,0.4])
        final_torque = np.array([0.05,0.35,0.2,0.06,0.1,0,0,0, 0,0,0,0,0,0.0,0.4,0.35])
        torques = [(((final_torque - initial_torque)/steps * i) + initial_torque) for i in range(steps)]

        joint_state_pub = rospy.Publisher('/allegroHand_0/joint_cmd', JointState, queue_size=10)
        lib_cmd_pub = rospy.Publisher('allegroHand_0/lib_cmd', String, queue_size=5)
        torque_state_pub = rospy.Publisher('/allegroHand_0/torque_cmd', JointState, queue_size=10)

    


    def disable_torques(self):
        apply_torque(torque_state_pub,[0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0],200)
        lib_cmd_pub.publish("off")
        print("torque off")
        

    def apply_torque(self, pub, torque_array,duration):
        rate = rospy.Rate(100)
        t= time.time() * 10**3
        while (time.time() * 10**3) < t+duration:  
            js = JointState()
            js.position = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]
            
            js.header = Header()
            js.header.stamp = rospy.Time.now()
            js.name = [ "joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0", "joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0",
                        "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0", "joint_12.0", "joint_13.0", "joint_14.0",
                        "joint_15.0"]
            js.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            js.effort = torque_array
            pub.publish(js)
            rate.sleep()



    def move_to_position(self, pub, position, duration):
        rate = rospy.Rate(100)
        t= time.time() * 10**3
        while (time.time() * 10**3) < t+duration:        
            js = JointState()
            js.position = position
            
            js.header = Header()
            js.header.stamp = rospy.Time.now()
            js.name = ["joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0", "joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0",
        "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0", "joint_12.0", "joint_13.0", "joint_14.0",
        "joint_15.0"]
            js.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            js.effort = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            pub.publish(js)
            rate.sleep()

    def callback(self, data):

        print("michelle_callback")
        #disable_torques()
        
        # for j in range(len(positions)):
        #     print("moving")
        self.move_to_position(self.joint_state_pub,open, 200)
        time.sleep(3)
        print("done with stage 1")
        print("switching to torque control")

        lib_cmd_pub.publish("on"),0.35,0.2,0.06,0.1,0,0,0, 0,0,0,0,0,0.02,0.4,0.35],200)
        #apply_torque(torque_state_pub,[0,0,0,0.0, 0,0,0,0, 0,0,0,0, 0,0,1,0],200)
        #disable_torques()

    def talker(self):
        print("michelle_talker")
        self.sub = rospy.Subscriber('otc_chopsticks_topic', String, queue_size=10, callback=self.callback)
        # print("done")
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('chopstick_closer', anonymous=True)
    class_instance = open_to_close()
    try:
        class_instance.talker()
    except rospy.ROSInterruptException:
        pass

