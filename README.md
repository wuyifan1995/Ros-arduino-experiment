# Ros-arduino-experiment
## 前言
制作一款用PC主机控制的arduino小机器人.设计物料：arduino， 舵机，超声波传感器，小马达风扇。目的：把雷达装在舵机上面，用键盘左右摇晃舵机，一旦雷达的返回值小于一定的范围，则风扇开始运动；-------Evan 写于2018 9月6日
## ROS原理
在ROS的控制模型中，一般会把对底盘的驱动，及马达的PID调节放在一起称之为base controller，我们可以用rosserial_arduino 进行arduino的封装，arduino单独形成一个node，这个node不断地发布超声波的传感器信息以及订阅一个叫做DJ的topic，一旦接收到topic的值，电机就会立马运行。PC端也会创建两个node，一个是键盘的node：keyboard_pub.py 这个是不断读取键盘值的node：a键位舵机加一，d位舵机减一，来累计angle发布servo topic。另外一个是djfire_sub.py 这个node的作用是订阅arduino 超声波的topic 然后换算成距离，一旦小于30cm 就会发布一个DJ的topic达到控制马达的作用

## arduino接线
- 电机是用了ULN2003 ：直流电机一个脚接在arduino的5v上，另外一只脚接在ULN2003的16脚上（右边第一）；arduino的数字8脚接ULN2003的1脚（左边第一个）；arduino的GND脚接ULN2003的8脚（左边最后一个）；arduino的5v接ULN2003的9脚（右边最后一个）；
- 舵机接线：舵机橙色线：arduino 数字9脚；舵机红色线：arduino的5v；舵机棕色线：arduino的GND；
- 超声波传感器：TRI接数字7，Echo接数字6；
## arduino端代码
- 用rosserial_python包生成node
~~~C++
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
const int U=8; 
const int TrigPin = 7;
const int EchoPin = 6;

ros::NodeHandle  nh;
std_msgs::UInt16 distance;

Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
servo.write(cmd_msg.data); //set servo angle, should be from 0-180  

}

void DJ_cb( const std_msgs::UInt16& cmd_msg){
analogWrite(U,200);
delay(5000);
analogWrite(U,0);

}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);
ros::Subscriber<std_msgs::UInt16> dj("DJ", DJ_cb); //电机
ros::Publisher utrasound("distance", &distance);

void setup(){
pinMode(13, OUTPUT);
pinMode(TrigPin, OUTPUT);
pinMode(EchoPin, INPUT);
servo.attach(9); //attach it to pin 9
nh.initNode();
nh.subscribe(sub);
nh.subscribe(dj);
nh.advertise(utrasound);
}
void getdistance(){
digitalWrite(TrigPin, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin, LOW);
distance.data = pulseIn(EchoPin, HIGH);
}

void loop(){
getdistance();
utrasound.publish( &distance );
nh.spinOnce();
delay(1000);
}
~~~
## PC端代码
-键盘的node：
~~~python 
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

~~~
- 处理超声波的node
~~~python
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

~~~
- launch文件：
~~~C++
<launch>
<node name="talker_pub" pkg="myfirstpkg"
type="keyboard_pub.py" output="screen" />
<node name="djfire" pkg="myfirstpkg"
type="djfire_sub.py" output="screen" />
<node name="arduino" pkg="rosserial_python"
type="serial_node.py" args="/dev/ttyUSB0" output="screen" />     
</launch>

~~~
