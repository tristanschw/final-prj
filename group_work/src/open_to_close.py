#!/usr/bin/env python
# license removed for brevity

# This file is very similar to close_to_open, except in closes
# the chopsticks instead of opening them

# standard imports
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np
import os

# number of steps for linear interpolation
steps = 50
# Position data for the final and initial positions, in this case initial is open, and final is closed.
open = np.array([-0.4544882180231765, 0.9385368696393318, 0.7390912269552227, -0.06894226970355727, -0.3127687481649788, 1.108102715282322, 1.0350556698657738, 0.48760094361757855, -0.1891814034481676, 1.2768303235180714, 0.8834499558975041, 0.34283025457759353, 1.5148682361396588, 0.25393788015816987, 0.387205266483693, 0.46550924371423])
closed = np.array([-0.13563331826143926, 1.2312634750831006, 0.9069657411016043, -0.1100786085291394, -0.1791499542428171, 1.6981353702790094, 1.0011153309179666, 0.5001013630769563, 0.022603119476372432, 1.7265652716472935, 1.2910120968193919, -0.2774761978172064, 1.4072040639865064, -0.0005677437280994912, 0.18899816764141025, 0.7607931989892316])

# This creates an array of positions that is a linear interpolation of the initial and final
# It allows the position controller to move the hand much more smoothly, and prevents the hand from jerking
# this allows us to maintain grip of the chopsticks
positions = [(((closed - open)/steps * i) + open) for i in range(steps)]

# Once the chopsticks are in the closed position we need to apply torques
# to put a force on them and allow us to grip the food.
initial_torque = np.array([0.05,0.6,0.4,0.1 ,0.1,0,0,0, 0,0,0,0,0,0.0,0.7,0.4])
final_torque = np.array([0.05,0.35,0.2,0.06,0.1,0,0,0, 0,0,0,0,0,0.0,0.4,0.35])
torques = [(((final_torque - initial_torque)/steps * i) + initial_torque) for i in range(steps)]

# Sets up our publishers, to publish our data as messages to our controller
joint_state_pub = rospy.Publisher('/allegroHand_0/joint_cmd', JointState, queue_size=10)
lib_cmd_pub = rospy.Publisher('allegroHand_0/lib_cmd', String, queue_size=5)
torque_state_pub = rospy.Publisher('/allegroHand_0/torque_cmd', JointState, queue_size=10)


def talker():
    
    # Starts the node
    rospy.init_node('chopstick_closer', anonymous=True)

    # Moves into the open position
    move_to_position(joint_state_pub,open, 200)

    # The one second wait helped with our testing.
    time.sleep(1)

    print("done with stage 1")
    print("switching to torque control")

    # Switches to torque control.
    lib_cmd_pub.publish("on")


    print("about to apply torque")
    # Loops through the array of torques and applies each one for 5 miliseconds.
    for j in range(len(torques)):
        apply_torque(torque_state_pub,torques[j],5)

    print("done")

# Helper function to disable all torques, helped with testing
def disable_torques():
    apply_torque(torque_state_pub,[0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0],200)
    lib_cmd_pub.publish("off")
    print("torque off")

# Takes a torque array and sends it as a message to the torque controller,
# pub is the publisher, and duration is the time the message is published for.
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


# Same as the function above but this time to interface with the position controller.
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

