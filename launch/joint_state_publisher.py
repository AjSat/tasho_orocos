#!/usr/bin/env python

# writing a new subscriber-publisher to communicate ros messages for visualization purpose

import rospy
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState

pub = rospy.Publisher("/joint_states", JointState, queue_size=10)


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard")
    # print(data.data)

    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = [
        "yumi_joint_1_l",
        "yumi_joint_2_l",
        "yumi_joint_7_l",
        "yumi_joint_3_l",
        "yumi_joint_4_l",
        "yumi_joint_5_l",
        "yumi_joint_6_l",
        "yumi_joint_1_r",
        "yumi_joint_2_r",
        "yumi_joint_7_r",
        "yumi_joint_3_r",
        "yumi_joint_4_r",
        "yumi_joint_5_r",
        "yumi_joint_6_r",
    ]
    hello_str.position = data.data
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)


def callback_left(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard")
    # print(data.data)

    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = [
        "yumi_joint_1_l",
        "yumi_joint_2_l",
        "yumi_joint_7_l",
        "yumi_joint_3_l",
        "yumi_joint_4_l",
        "yumi_joint_5_l",
        "yumi_joint_6_l",
    ]
    hello_str.position = data.data
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)


def callback_right(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard")
    # print(data.data)

    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = [
        "yumi_joint_1_r",
        "yumi_joint_2_r",
        "yumi_joint_7_r",
        "yumi_joint_3_r",
        "yumi_joint_4_r",
        "yumi_joint_5_r",
        "yumi_joint_6_r",
    ]
    hello_str.position = data.data
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)


def listener():

    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/joint_states_from_orocos", Float64MultiArray, callback)
    rospy.Subscriber("/joint_states_left_from_orocos", Float64MultiArray, callback_left)
    rospy.Subscriber(
        "/joint_states_right_from_orocos", Float64MultiArray, callback_right
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
