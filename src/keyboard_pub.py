#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import UInt16
import sys, select, termios, tty
#角度

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('talkerpy_pub')
    pub = rospy.Publisher('servo', UInt16, queue_size=5)

    try:
        angle = 0
        speed = 1
        pub.publish(angle)
        print "a代表加一，b代表-1"
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key == 'a':
                angle=angle+speed
                pub.publish(angle)
                print "+1,目前角度为： " 
                print angle
            # 速度修改键
            elif key == 'd':
                angle = angle-speed
                pub.publish(angle)
                print "-1,目前角度为： "
                print angle 
            else:
                
                if (key == '\x03'):
                    break
        
    except:
        print angle