#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np
import os

steps = 100


new_inital = np.array([-0.4362727369225076, 1.0082741588284123, 0.7168070353125869, -0.11432735422490008, -0.46112298539757773, 1.1469968654759386, 1.0420951801575506, 0.4943383082997488, -0.4599036825818679, 1.271357544700629, 0.8796909496181735, 0.34166307701801457, 1.5135894397572989, 0.2834864654480982, 0.39792946469884616, 0.4331492996034923])
new_final = np.array([-0.4544882180231765, 0.9385368696393318, 0.7390912269552227, -0.06894226970355727, -0.3127687481649788, 1.108102715282322, 1.0350556698657738, 0.48760094361757855, -0.1891814034481676, 1.2768303235180714, 0.8834499558975041, 0.34283025457759353, 1.5148682361396588, 0.25393788015816987, 0.387205266483693, 0.46550924371423])
positions = [(((new_final - new_inital)/steps * i) + new_inital) for i in range(steps)]

initial_torque = np.array([0.05,0.6,0.4,0.1 ,0.1,0,0,0, 0,0,0,0,0,0.02,0.7,0.4])
final_torque = np.array([0.05,0.35,0.2,0.06,0.1,0,0,0, 0,0,0,0,0,0.02,0.4,0.35])
torques = [(((final_torque - initial_torque)/steps * i) + initial_torque) for i in range(steps)]
s_f = 0.53
joint_state_pub = rospy.Publisher('/allegroHand_0/joint_cmd', JointState, queue_size=10)
lib_cmd_pub = rospy.Publisher('allegroHand_0/lib_cmd', String, queue_size=5)
torque_state_pub = rospy.Publisher('/allegroHand_0/torque_cmd', JointState, queue_size=10)


def talker():
    
    rospy.init_node('chopstick_closer', anonymous=True)
    
    #disable_torques()
    
    # for j in range(len(positions)):
    #     print("moving")
    # move_to_position(joint_state_pub,open, 200)
    # time.sleep(3)
    # print("done with stage 1")
    # print("switching to torque control")

    lib_cmd_pub.publish("on")

    # print("about to apply torque")
    # for j in range(len(torques)):
    #     apply_torque(torque_state_pub,torques[j],5)
    apply_torque(torque_state_pub,[0.05    ,0.35    ,0.2    ,0.06    ,0.1    ,0,0,0,0,0,0,0,0,0.03,0.4    ,0.35    ],100)
    print("switch")
    #apply_torque(torque_state_pub,[0.05*s_f,(0.35*s_f)-0,0.2*s_f - 0.02,0.06*s_f, 0.1*s_f,0,0,0, 0,0,0,0, 0,0.03,0.4    ,0.35    ],100)
    apply_torque(torque_state_pub,s_f*np.array([0.05    ,0.35    ,0.2    ,0.06    ,0.1    ,0,0,0,0,0,0,0,0,0.03,0.4    ,0.35    ]),100)
    lib_cmd_pub.publish("off")
    print("torques off")
    for j in range(len(positions)):
        move_to_position(joint_state_pub,positions[j],5)

    #apply_torque(torque_state_pub,[0.05,0,0,0.0 , 0.1,0,0,0, 0,0,0,0, 0,0.02,0.0,0.00],10)

    
    #apply_torque(torque_state_pub,[0.05,0.6,0.4,0.1 ,0.1,0,0,0, 0,0,0,0,0,0.02,0.7,0.4],100)

    
    #apply_torque(torque_state_pub,[0,0,0,0.0, 0,0,0,0, 0,0,0,0, 0,0,1,0],200)
    #disable_torques()
    print("done")

def disable_torques():
    lib_cmd_pub.publish("off")
    apply_torque(torque_state_pub,[0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0],200)
    lib_cmd_pub.publish("off")
    print("torque off")
    

def apply_torque(pub, torque_array,duration):
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



def move_to_position(pub, position, duration):
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


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

