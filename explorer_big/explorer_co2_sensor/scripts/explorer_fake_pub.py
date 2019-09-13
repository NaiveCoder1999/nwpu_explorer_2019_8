#!/usr/bin/env python3
#coding=utf-8

import rospy
from explorer_msgs.msg import explorer_low_level_data
from std_msgs.msg import String

fake_data = explorer_low_level_data()
fake_data.low_level_id = 9
fake_data.can_serial_data_1 = 400


def talker():
    pub = rospy.Publisher('/explorer_serial_data/9', explorer_low_level_data, queue_size=10)
    rospy.init_node('explorer_co2_fake_pub', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("publish: %s", str(fake_data.can_serial_data_1))
        pub.publish(fake_data)
        fake_data.can_serial_data_1 += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
