#!/usr/bin/env python
'''djfire ROS Node'''
import rospy
import time 
from std_msgs.msg import UInt16
global distance 
distance = UInt16()
distance.data = 20000 
def callback(data):

    distance.data=data.data/58


if __name__ == '__main__':
    rospy.init_node('djfire', anonymous=True)

    sub = rospy.Subscriber("distance", UInt16, callback)
    pub = rospy.Publisher("DJ",UInt16, queue_size=10)
    while(1):
        if distance.data <= 30:
            pub.publish(distance)
        time.sleep(5)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
