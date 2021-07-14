#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/rscuad/r_ank_roll_position/command', Float64)
    rospy.init_node('walking', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(1.070059500058)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass