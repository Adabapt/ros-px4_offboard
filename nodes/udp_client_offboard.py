#!/usr/bin/python3
 
import time
import socket
import rospy
import signal
import sys
import math
import numpy as np
import mavros

from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

################VAR GLOBALE########################

node_name = str(rospy.get_param('node_name'))
topic_name = str(rospy.get_param('topic_name'))
ip_srv = str(rospy.get_param('ip_srv'))
port_srv = int(rospy.get_param('port_srv'))
drone_nb = int(rospy.get_param('drone_nb'))

buffer=1024

################UDP CLIENT CLASS####################


class UdpClientOffbNode:

	def __init__(self):

		self.uav_pose = PoseStamped()
		self.pose_dest = PoseStamped()
		self.data = ""

		rospy.init_node(node_name)

		# Load parameters
		self.loc_pos_sub = rospy.Publisher("/"+topic_name+"/setpoint_position/local", PoseStamped, queue_size=10)
	
		self.state_sub = rospy.Subscriber("/"+topic_name+"/state", State, self.state_cb)
		self.pose_sub = rospy.Subscriber("/"+topic_name+"/local_position/pose", PoseStamped, self.pose_cb)

		self.arming_client = rospy.ServiceProxy("/"+topic_name+"/cmd/arming", CommandBool)
		self.set_mode_client = rospy.ServiceProxy("/"+topic_name+"/set_mode", SetMode)


		self.current_state = State()
		self.prev_state = self.current_state

		self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

		self.last_request = 0
		self.now = 0

		rospy.loginfo('Autopilot offboard node initialized')

		pose = PoseStamped()

		# send a few setpoints before starting
		for i in range(100):
			self.loc_pos_sub.publish(pose)
			self.rate.sleep()
		
		rospy.loginfo('Autopilot offboard overflow cmd')

		# wait for FCU connection
		while not self.current_state.connected:
			self.rate.sleep()

		rospy.loginfo('Autopilot offboard node connected')
		self.last_request = rospy.get_rostime()


	def state_cb(self,state):
		self.current_state = state


	def pose_cb(self,pose):
		self.uav_pose = pose

		deg_x,deg_y,deg_z = quaternion_to_euler(self.uav_pose.pose.orientation)

		self.data=str(round(self.uav_pose.pose.position.x,1))+":"+str(round(self.uav_pose.pose.position.y,1))+":"+str(round(self.uav_pose.pose.position.z,1))+":"+str(round(deg_z,1))


	def offboard_mode(self):

		while not self.current_state.armed:
			if self.current_state.mode != "OFFBOARD":
				self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
			else:
				if not self.current_state.armed:
					self.arming_client(True)
					
			self.rate.sleep()

			# older versions of PX4 always return success==True, so better to check Status instead
			if self.prev_state.armed != self.current_state.armed:
				rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
			if self.prev_state.mode != self.current_state.mode: 
				rospy.loginfo("Current mode: %s" % self.current_state.mode)
			self.prev_state = self.current_state
			self.rate.sleep()

	def move(self):

		self.offboard_mode()
		self.loc_pos_sub.publish(self.pose_dest)
		self.rate.sleep()

	def send_data(self):

		print("creation socket")

		sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

		print("connexion socket")

		sock.connect((ip_srv, port_srv))

		print("envoie data")

		msg = "UAV."+str(drone_nb)+"position=\""+self.data+"\""

		print("msg env :"+msg)

		msg_enc = msg.encode()
		sock.send(msg_enc)

		print("reception data")

		buf=sock.recv(buffer)
		buf_dec = buf.decode()
		print(buf_dec)

		part = buf_dec.split(':')

		self.pose_dest.pose.position.x = float(part[0])
		self.pose_dest.pose.position.y = float(part[1])
		self.pose_dest.pose.position.z = float(part[2])

		qua_x, qua_y, qua_z, qua_w = euler_to_quaternion(0,0,float(part[3]))

		self.pose_dest.pose.orientation.x = qua_x
		self.pose_dest.pose.orientation.y = qua_y
		self.pose_dest.pose.orientation.z = qua_z
		self.pose_dest.pose.orientation.w = qua_w

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


def euler_to_quaternion(deg_x, deg_y, deg_z):
	"""
	Convert euler angles into a quaternion 
	yaw = deg_z
	pitch = deg_y
	roll = deg_x
	"""
	roll = math.radians(deg_x)
	pitch = math.radians(deg_y)
	yaw = math.radians(deg_z)

	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return qx, qy, qz, qw


####################MAIN###############################


if __name__ == '__main__':
	try:

		node = UdpClientOffbNode()
		node.offboard_mode()

		t_fin = time.time()
		t_act = time.time()

		while 1:

			node.offboard_mode()
			node.move()

			if t_act > t_fin :

				node.send_data()
				t_fin = t_act + 3

			t_act = time.time()

	except rospy.ROSInterruptException:
		pass