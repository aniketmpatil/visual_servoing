#!/usr/bin/env python3
#!/usr/bin/env python3
#import math
#import random
import rospy
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64


def talker():
    pub_q1_pos = rospy.Publisher(
        '/vbmbot/joint1_position_controller/command', Float64, queue_size=10)
    pub_q2_pos = rospy.Publisher(
        '/vbmbot/joint2_position_controller/command', Float64, queue_size=10)
    pub_q1_vel = rospy.Publisher(
        '/vbmbot/joint1_velocity_controller/command', Float64, queue_size=10)
    pub_q2_vel = rospy.Publisher(
        '/vbmbot/joint2_velocity_controller/command', Float64, queue_size=10)

    rospy.init_node('joint_manip_talker', anonymous=True)
    rate = rospy.Rate(1)  # meaning 1 message published in 1 sec
    rospy.sleep(5)

    q1_pos = 2.09
    q2_pos = 1.57
    pub_q1_pos.publish(q1_pos)
    pub_q2_pos.publish(q2_pos)
    rospy.sleep(7)

    while(1):
        q1_pos = -0.25
        q2_pos = 0.25
        pub_q1_pos.publish(q1_pos)
        pub_q2_pos.publish(q2_pos)
        rospy.sleep(5)

        q1_pos = 1.2

        q2_pos = -1.6
        pub_q1_pos.publish(q1_pos)
        pub_q2_pos.publish(q2_pos)
        rospy.sleep(5)


if __name__ == '__main__':
    try:
        talker()
        rospy.sleep(30)
    except rospy.ROSInterruptException:
        pass
