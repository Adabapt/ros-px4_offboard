#!/usr/bin/env python3

import rospy
import mavros
import roslib
import argparse
import math
import numpy as np
import time

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Twist
from mavros_msgs.msg import State, Thrust
from mavros_msgs.srv import CommandBool, SetMode

################VAR GLOBALE########################

precision_lin = 0.05 #5 cm d'écart par rapport à la distance voulue
precision_alt = 0.1 #10 cm d'écart par rapport à la distance voulue
precision_ang = 2 #2 degre d'ecart par rapport à l'angle voulu

#recup les arguments du launch
topic_name = str(rospy.get_param('topic_name'))
node_name = str(rospy.get_param('node_name'))

################AUTOPILOT CLASS####################
"""
node pour l'autopilotage du drone en utilisant l'antenne 4333MHz pour la télémetrie
"""
class AutopilotOffboardNode:
	"""
	initialisation de la node de pilotage automatique du drone à partir de fonction de déplacement élementaire en utilisant des objectifs de position
	"""
	def __init__(self):

		#last pos
		self.last_pos = PoseStamped()

		#speed
		self.speed = 0.2 # en m/s
		self.speed_corr = 0.1 # en m/s
		self.speed_ang = 0.2 # en rad/s
		self.speed_alt = 0.5 # en m/s

		# Start ROS node
		rospy.init_node(node_name)

		# Load parameters

		self.cmd_vel_pub = rospy.Publisher("/"+topic_name+"/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
	
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

		cmd = Twist()
		cmd.linear.x = 0
		cmd.linear.y = 0
		cmd.linear.z = 0
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = 0

		# send a few setpoints before starting
		for i in range(100):
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()
		
		rospy.loginfo('Autopilot offboard overflow cmd')

		# wait for FCU connection
		while not self.current_state.connected:
			self.rate.sleep()

		rospy.loginfo('Autopilot offboard node connected')
		self.last_request = rospy.get_rostime()

	################CALLBACK FUNCTION####################

	"""
	fonction de callback pour raffraichir la variable état du drone pour le conserver en offboard et armé
	"""
	def state_cb(self,state):
		self.current_state = state

	"""
	fonction de callback pour raffraichir la variable de position du drone (à changer par optitrack ou px4flow ou marvelmind plus tard)
	actuellement odométrie pas très fiable
	"""
	def pose_cb(self,pose):
		self.last_pos = pose

	################METHODES####################

	"""
	fonction qui raffraichi l'état de pilotage du drone sur offboard pour pouvoir lui envoyer des instructions depuis l'ordinateur
	elle sert aussi à armer le drone (peut être à séparer dans une autre fonction)
	"""
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

	"""
	fonction pour déplacer le drone dans une direction sur une distance précise en mètre
	fonctionne avec un système de setpoint qui est la position que va tenter d'atteindre le drone
	"""
	def move(self,direction,distance):
		pos_init = PoseStamped()
		pos_init = self.last_pos

		cmd = Twist()
		cmd.linear.x = 0
		cmd.linear.y = 0
		cmd.linear.z = 0
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = 0

		if direction == "forward":
			
			print("forward : "+str(distance))

			pos_dest = PoseStamped()
			pos_dest = pos_init
			pos_dest.pose.position.y = pos_init.pose.position.y + distance

			self.offboard_mode()
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()

			while abs(pos_dest.pose.position.y - self.last_pos.pose.position.y) > precision_lin:

				#correction des perturbations
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
					cmd.linear.x = self.speed_corr
				else:
					cmd.linear.x = -self.speed_corr

				if pos_dest.pose.position.z > self.last_pos.pose.position.z:
					cmd.linear.z = self.speed_corr
				else:
					cmd.linear.z = -self.speed_corr
				#commande de la direction principale
				if pos_dest.pose.position.y > self.last_pos.pose.position.y:
					cmd.linear.y = self.speed
				else:
					cmd.linear.y = -self.speed	
				self.offboard_mode()
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()
				print("y axis : "+ str(self.last_pos.pose.position.y))

		if direction == "back":
			
			print("back : "+str(distance))

			pos_dest = PoseStamped()
			pos_dest = pos_init
			pos_dest.pose.position.y = pos_init.pose.position.y - distance

			self.offboard_mode()
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()

			while abs(pos_dest.pose.position.y - self.last_pos.pose.position.y) > precision_lin:

				#correction des perturbations
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
					cmd.linear.x = self.speed_corr
				else:
					cmd.linear.x = -self.speed_corr

				if pos_dest.pose.position.z > self.last_pos.pose.position.z:
					cmd.linear.z = self.speed_corr
				else:
					cmd.linear.z = -self.speed_corr

				#commande de la direction principale
				if pos_dest.pose.position.y < self.last_pos.pose.position.y:
					cmd.linear.y = -self.speed
				else:
					cmd.linear.y = self.speed
				self.offboard_mode()
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()
				print("y axis : "+ str(self.last_pos.pose.position.y))

		if direction == "right":
			
			print("right : "+str(distance))

			pos_dest = PoseStamped()
			pos_dest = pos_init
			pos_dest.pose.position.y = pos_init.pose.position.x + distance

			self.offboard_mode()
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()

			while abs(pos_dest.pose.position.x - self.last_pos.pose.position.x) > precision_lin:

				#correction des perturbations
				if pos_dest.pose.position.y > self.last_pos.pose.position.y:
					cmd.linear.y = self.speed_corr
				else:
					cmd.linear.y = -self.speed_corr

				if pos_dest.pose.position.z > self.last_pos.pose.position.z:
					cmd.linear.z = self.speed_corr
				else:
					cmd.linear.z = -self.speed_corr

				#commande de la direction principale
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
					cmd.linear.x = self.speed
				else:
					cmd.linear.x = -self.speed
				print("x axis : "+ str(self.last_pos.pose.position.x))	
				self.offboard_mode()
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()

		if direction == "left":
			
			print("left : "+str(distance))

			pos_dest = PoseStamped()
			pos_dest = pos_init
			pos_dest.pose.position.x = pos_init.pose.position.x - distance

			self.offboard_mode()
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()

			while abs(pos_dest.pose.position.x - self.last_pos.pose.position.x) > precision_lin:

				#correction des perturbations
				if pos_dest.pose.position.y > self.last_pos.pose.position.y:
					cmd.linear.y = self.speed_corr
				else:
					cmd.linear.y = -self.speed_corr

				if pos_dest.pose.position.z > self.last_pos.pose.position.z:
					cmd.linear.z = self.speed_corr
				else:
					cmd.linear.z = -self.speed_corr

				#commande de la direction principale
				if pos_dest.pose.position.x < self.last_pos.pose.position.x:
					cmd.linear.x = -self.speed
				else:
					cmd.linear.x = self.speed
				print("y axis : "+ str(self.last_pos.pose.position.x))
				self.offboard_mode()
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()

		if direction == "top":
			
			print("top :"+str(distance))

			pos_dest = PoseStamped()
			pos_dest = pos_init
			pos_dest.pose.position.z = pos_init.pose.position.z + distance

			self.offboard_mode()
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()

			while abs(self.last_pos.pose.position.z - pos_dest.pose.position.z) > precision_alt:

				#correction des perturbations
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
					cmd.linear.x = self.speed_corr
				else:
					cmd.linear.x = -self.speed_corr

				if pos_dest.pose.position.y > self.last_pos.pose.position.y:
					cmd.linear.y = self.speed_corr
				else:
					cmd.linear.y = -self.speed_corr

				#commande de la direction principale
				if pos_dest.pose.position.z > self.last_pos.pose.position.z:
					cmd.linear.z = self.speed_alt
				else:
					cmd.linear.z = -self.speed_alt
				print("z axis : "+ str(self.last_pos.pose.position.z))
				self.offboard_mode()
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()

		if direction == "bottom":
			
			print("bottom :"+str(distance))

			pos_dest = PoseStamped()
			pos_dest = pos_init
			pos_dest.pose.position.z = pos_init.pose.position.z - distance

			self.offboard_mode()
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()

			while abs(self.last_pos.pose.position.z - pos_dest.pose.position.z) > precision_alt:

				#correction des perturbations
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
					cmd.linear.x = self.speed_corr
				else:
					cmd.linear.x = -self.speed_corr
				
				if pos_dest.pose.position.y > self.last_pos.pose.position.y:
					cmd.linear.y = self.speed_corr
				else:
					cmd.linear.y = -self.speed_corr

				#commande de la direction principale
				if pos_dest.pose.position.z < self.last_pos.pose.position.z:
					cmd.linear.z = -self.speed_alt
				else:
					cmd.linear.z = self.speed_alt
				print("z axis : "+ str(self.last_pos.pose.position.z))
				self.offboard_mode()
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()

	"""	
	fonction pour faire tourner le drone sur lui même par la droite ou par la gauche avec un angle donné
	fonctionne avec une orientation en quaternion que le drone va tenter d'atteindre
	on transforme donc la valeur à atteindre de degré à radian puis en quaternion qu'on met comme objectif
	puis dans l'affichage on met l'évolution de la valeur d'angle
	"""
	def rotate(self,sens,angle):
		pos_init = PoseStamped()
		pos_init = self.last_pos

		cmd = Twist()
		cmd.linear.x = 0
		cmd.linear.y = 0
		cmd.linear.z = 0
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = 0

		deg_x,deg_y,deg_z = quaternion_to_euler(pos_init.pose.orientation)
		if deg_z > 0:
			deg_z = deg_z
		else:
			deg_z = deg_z+360

		if sens == "right":
			print("rota droite :"+str(angle))
			pos_dest = PoseStamped()
			pos_dest = pos_init
			deg_z_obj = (deg_z-angle)%360
			
			print("angle objectif : "+ str(deg_z_obj))

			while deg_z > deg_z_obj + precision_ang or deg_z < deg_z_obj - precision_ang :

				#correction des perturbations
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
					cmd.linear.x = self.speed_corr
				else:
					cmd.linear.x = -self.speed_corr
				
				if pos_dest.pose.position.y > self.last_pos.pose.position.y:
					cmd.linear.y = self.speed_corr
				else:
					cmd.linear.y = -self.speed_corr

				if pos_dest.pose.position.z > self.last_pos.pose.position.z:
					cmd.linear.z = self.speed_corr
				else:
					cmd.linear.z = -self.speed_corr

				#commande de l'angle
				self.offboard_mode()
				cmd.angular.z = -self.speed_ang
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()

				deg_x,deg_y,deg_z = quaternion_to_euler(self.last_pos.pose.orientation)
				if deg_z > 0:
					deg_z = deg_z
				else:
					deg_z = deg_z + 360
				print("angle z : "+str(deg_z))
				

		if sens == "left":
			print("rota gauche :"+str(angle))
			pos_dest = PoseStamped()
			pos_dest = pos_init
			deg_z_obj = (deg_z+angle)%360
			
			print("angle objectif : "+ str(deg_z_obj))

			while deg_z > deg_z_obj + precision_ang or deg_z < deg_z_obj - precision_ang :

				#correction des perturbations
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
					cmd.linear.x = self.speed_corr
				else:
					cmd.linear.x = -self.speed_corr
				
				if pos_dest.pose.position.y > self.last_pos.pose.position.y:
					cmd.linear.y = self.speed_corr
				else:
					cmd.linear.y = -self.speed_corr

				if pos_dest.pose.position.z > self.last_pos.pose.position.z:
					cmd.linear.z = self.speed_corr
				else:
					cmd.linear.z = -self.speed_corr

				#commande de l'angle
				self.offboard_mode()
				cmd.angular.z = self.speed_ang
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()

				deg_x,deg_y,deg_z = quaternion_to_euler(self.last_pos.pose.orientation)
				if deg_z > 0:
					deg_z = deg_z
				else:
					deg_z = deg_z + 360
				print("angle z : "+str(deg_z))

	######################EXAMPLES TRAJET#####################

	#deplacement préprogrammé qui fait un carré
	def square(self,distance):
		#add fonction decollage atterissage
		self.move("top",1)

		self.move("forward",distance)
		self.rotate("right",90)
		self.move("right",distance)
		self.rotate("right",90)
		self.move("back",distance)
		self.rotate("right",90)
		self.move("left",distance)
		self.rotate("right",90)

		#add fonction decollage atterissage
		self.move("bottom",1)


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

		node = AutopilotOffboardNode()
		node.offboard_mode()
		
		#example de déplacement
		node.square(1)

	except rospy.ROSInterruptException:
		pass