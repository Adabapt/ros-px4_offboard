#!/usr/bin/python3
 
import time
import socket
import rospy
import signal
import sys
import math
import numpy as np


from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from mavros_msgs.msg import State

################VAR GLOBALE########################

node_name = str(rospy.get_param('node_name'))
topic_name = str(rospy.get_param('topic_name'))
ip_srv = str(rospy.get_param('ip_srv'))
port_srv = int(rospy.get_param('port_srv'))
drone_nb = int(rospy.get_param('drone_nb'))

buffer=1024

################UDP CLIENT CLASS####################


class UdpClientNode:

	def __init__(self):


		self.uav_pose = PoseStamped()

		rospy.init_node(node_name)

		self.pose_sub = rospy.Subscriber("/"+topic_name+"/local_position/pose", PoseStamped, self.pose_cb)
		self.data = ""
		self.last_data = ""

	def pose_cb(self,pose):
		self.uav_pose = pose

		deg_x,deg_y,deg_z = quaternion_to_euler(self.uav_pose.pose.orientation)

		self.data=str(round(self.uav_pose.pose.position.x,1))+":"+str(round(self.uav_pose.pose.position.y,1))+":"+str(round(self.uav_pose.pose.position.z,1))+":"+str(round(deg_z,1))

	def send_data(self):

		if self.last_data != self.data :

			print("creation socket")

			sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

			print("connexion socket")

			sock.connect((ip_srv, port_srv))

			print("envoie data")

			msg = "UAV"+str(drone_nb)+".position=\""+self.data+"\""

			self.last_data = self.data

			print("msg env :"+msg)

			msg_enc = msg.encode()
			sock.send(msg_enc)

			"""print("reception data")

			buf=sock.recv(buffer)
			buf_dec = buf.decode()
			print(buf_dec)"""

			print("fermeture socket")

			sock.close()
			

######################AUTRES FONCTIONS#####################

def quaternion_to_euler(orientation):
	"""
	Convert a quaternion into euler angles (roll, pitch, yaw)
	roll is rotation around x in radians (counterclockwise)
	pitch is rotation around y in radians (counterclockwise)
	yaw is rotation around z in radians (counterclockwise)
	"""
	x=orientation.x
	y=orientation.y
	z=orientation.z
	w=orientation.w

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = math.atan2(t0, t1)

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)

	deg_x = math.degrees(roll_x)
	deg_y = math.degrees(pitch_y)
	deg_z = math.degrees(yaw_z)

	return deg_x, deg_y, deg_z# in degrees


####################MAIN###############################


if __name__ == '__main__':
	try:

		node = UdpClientNode()

		time.sleep(1)

		while 1:
			node.send_data()
			time.sleep(0.01) # 10HZ

	except rospy.ROSInterruptException:
		pass