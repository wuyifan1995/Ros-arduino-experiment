#!/usr/bin/env python
'''serialtest ROS Node'''
import rospy
import serial
from std_msgs.msg import String

def callback(msg):
    ser = serial.Serial('/dev/ttyUSB0',9600, timeout=1)
    ser.write(msg.data)
        
        
if __name__ == '__main__':
    
    rospy.init_node('serialtest')
    rate = rospy.Rate(10)
    serial_pub = rospy.Publisher('serial', String, queue_size=1)
    ser = serial.Serial('/dev/ttyUSB0',9600, timeout=1)
    sdata = String()
    sdata.data = ser.read(1)
    
    sub = rospy.Subscriber("led", String, callback)
    while not rospy.is_shutdown():

        serial_pub.publish(sdata)
        rate.sleep()
  
  
