#! /usr/bin/python3

import rospy
from std_msgs.msg import Int16

if __name__ == '__main__':
    rospy.init_node("counter_publisher")

    rate = rospy.Rate(5)       # Hz
    pub = rospy.Publisher("/set_esc_pwm_val", Int16, queue_size=10)
    counter = 0

    rospy.loginfo("Publisher has been started.")

    esc_min, esc_middle, esc_max = 45, 90, 135

    rospy.wait_for_message("esc_pwm_val", Int16, timeout=5)

    msg = Int16()

    for x in range(esc_middle, esc_max):
        msg.data = x
        pub.publish(msg)
        rate.sleep()

    for _ in range(40):
        msg.data = x
        pub.publish(msg)
        rate.sleep()

    for x in range(esc_max, esc_middle - 1, -1):
        msg.data = x
        pub.publish(msg)
        rate.sleep()
