#!usr/bin/env python3

import rospy
from std_msgs.msg import String

def heartbeat():
    pub = rospy.Publisher('acri_hw/status', String, queue_size=1)
    rospy.init_node('heartbeat_NUC', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status = "HEALTHY"
        pub.publish(status)
        rate.sleep()

if __name__ == '__main__':
    try:
        heartbeat()
    except rospy.ROSInterruptException:
        pass