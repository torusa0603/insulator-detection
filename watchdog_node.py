#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


class WatchdogNode():

    def __init__(self):
        print("init WatchdogNode")
        self.pub = rospy.Publisher('watchdog', String, queue_size=10)


    def start(self):
        while not rospy.is_shutdown():
            # rospy.sleep(1)
            rospy.sleep(0.5)
            self.pub.publish("ntf_interval")


if __name__ == '__main__':
    rospy.init_node('watchdog_node', anonymous=True)
    try:
        watchdog = WatchdogNode()
        watchdog.start()
    except rospy.ROSInterruptException:
        pass
