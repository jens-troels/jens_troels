#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from pymodbus.client.sync import ModbusTcpClient

color = 0
flag = False

def callback_color(data):
    global color, flag
    color = data.data
    flag = True

PiSignalPub = rospy.Publisher('PISignalTopic', String, queue_size=1)

PiColorSub = rospy.Subscriber('PIColorTopic', Int32, callback_color)

rospy.init_node('modbuspy', anonymous=True)
rate = rospy.Rate(10)

ip = "192.168.0.1"
port = 5000

def read():
    client = ModbusTcpClient(ip,port)
    result = client.read_coils(1,3)
    print(result.bits[0],result.bits[1],result.bits[2])
    client.close()
    return result

print("Started client")
while not rospy.is_shutdown():
	if flag:
		result = read()
		if result.bits[color-1]:
			PiSignalPub.publish("True")
			print(True)
			rate.sleep()
		else:
			PiSignalPub.publish("False")
			print(False)
			rate.sleep()
		flag = False
