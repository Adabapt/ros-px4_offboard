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

"""version 2 avec des zones pour chaque drone et une fonction home"""
""" note : se déplace relativement au repere du drone pas comme l'autre avec la position absolu"""

################VAR GLOBALE########################

precision_lin = 0.05 #5 cm d'écart par rapport à la distance voulue
precision_alt = 0.1 #10 cm d'écart par rapport à la distance voulue
precision_ang = 2 #2 degre d'ecart par rapport à l'angle voulu

marge_zone = 0.05 #5 cm de marge sur les bords du drone en bas à gauche de la zone

#recup les arguments du launch
topic_name = str(rospy.get_param('topic_name'))
node_name = str(rospy.get_param('node_name'))
longueur_zone = float(rospy.get_param('longueur_zone')) #axe y
largeur_zone = float(rospy.get_param('largeur_zone')) #axe x
hauteur_zone = float(rospy.get_param('hauteur_zone')) #axe z

################AUTOPILOT CLASS####################
"""
node pour l'autopilotage du drone en utilisant l'antenne 4333MHz pour la télémetrie
"""
class AutopilotOffboardNode:
	"""
	initialisation de la node de pilotage automatique du drone à partir de fonction de déplacement élementaire en utilisant des objectifs de position
	"""
	def __init__(self):

		#init pos
		self.flag_init_pos = True #pour récupérer une fois la position initiale
		self.init_pos = PoseStamped()

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
	fonction de callback pour raffraichir la variable de position du drone*
	recupère la position initiale au debut
	"""
	def pose_cb(self,pose):

		if self.flag_init_pos == True:
			self.flag_init_pos = False
			self.init_pos = pose

		self.last_pos = pose

	################METHODES####################

	"""fonctions pour armer et changer le mode du drone"""


	def offboard_mode(self):
		print("offboard mode")
		self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
		self.rate.sleep()

	def arm(self):
		print("arming drone")
		self.arming_client(True)
		self.rate.sleep()	

	def land_mode(self):
		print("land mode")
		self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
		self.rate.sleep()

	def disarm(self):
		print("disarming drone")
		#fonctionne à tous les coups
		self.cmd(broadcast=False, command=400, confirmation=0, 
				param1=0, param2=21196, param3=0,param4=0, param5=0, param6=0, param7=0)
		self.rate.sleep()

	"""
	fonction pour déplacer le drone dans une direction sur une distance précise en mètre
	fonctionne avec un système de setpoint qui est la position que va tenter d'atteindre le drone
	"""
	def move(self,direction,distance):
		pos_start = PoseStamped()
		pos_start = self.last_pos

		flag_depassement = False
		flag_condition = True

		pos_dest = PoseStamped()
		pos_dest = pos_start
		cmd = Twist()

		if direction == "forward":
			
			print("forward : "+str(distance))
			pos_dest.pose.position.y = pos_start.pose.position.y + distance

			#condition pour pas dépasser la zone limite du drone
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0:
				if pos_dest.pose.position.y > self.init_pos.pose.position.y + longueur_zone :
					pos_dest.pose.position.y = self.init_pos.pose.position.y + longueur_zone

		if direction == "back":
			
			print("back : "+str(distance))
			pos_dest.pose.position.y = pos_start.pose.position.y - distance

			#condition pour pas dépasser la zone limite du drone
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0 :
				if pos_dest.pose.position.y < self.init_pos.pose.position.y - marge_zone:
					pos_dest.pose.position.y = self.init_pos.pose.position.y

		if direction == "right":
			
			print("right : "+str(distance))
			pos_dest.pose.position.x = pos_start.pose.position.x + distance

			#condition pour pas dépasser la zone limite du drone
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0 :
				if pos_dest.pose.position.x > self.init_pos.pose.position.x + largeur_zone :
					pos_dest.pose.position.x = self.init_pos.pose.position.x + largeur_zone

		if direction == "left":
			
			print("left : "+str(distance))
			pos_dest.pose.position.x = pos_start.pose.position.x - distance

			#condition pour pas dépasser la zone limite du drone
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0 :
				if pos_dest.pose.position.x < self.init_pos.pose.position.x - marge_zone :
					pos_dest.pose.position.x = self.init_pos.pose.position.x

		if direction == "top":
			
			print("top :"+str(distance))
			pos_dest.pose.position.z = pos_start.pose.position.z + distance

			#pour limiter l'altitude
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0 :
				if pos_dest.pose.position.z > self.init_pos.pose.position.z + hauteur_zone :
					pos_dest.pose.position.z = self.init_pos.pose.position.z + hauteur_zone

		if direction == "bottom":
			
			print("bottom :"+str(distance))
			pos_dest.pose.position.z = pos_start.pose.position.z - distance

			#pour limiter l'altitude
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0 :
				if pos_dest.pose.position.z < self.init_pos.pose.position.z :
					pos_dest.pose.position.z = self.init_pos.pose.position.z

		self.cmd_vel_pub.publish(cmd)
		self.rate.sleep()


		while flag_condition or flag_depassement :

			if direction == "forward" or direction == "back" :

				if pos_dest.pose.position.y > self.last_pos.pose.position.y:
					cmd.linear.y = self.speed
				else:
					cmd.linear.y = -self.speed

				#correction des erreurs
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
						cmd.linear.x = self.speed_corr
				elif pos_dest.pose.position.x < self.last_pos.pose.position.x :
					cmd.linear.x = -self.speed_corr

				if pos_dest.pose.position.z > self.last_pos.pose.position.z:
					cmd.linear.z = self.speed_corr
				elif pos_dest.pose.position.z < self.last_pos.pose.position.z:
					cmd.linear.z = -self.speed_corr

			if direction == "right" or direction == "left" :

				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
					cmd.linear.x = self.speed
				else:
					cmd.linear.x = -self.speed
					
				#correction des erreurs
				if pos_dest.pose.position.y > self.last_pos.pose.position.y :
						cmd.linear.y = self.speed_corr
				elif pos_dest.pose.position.y < self.last_pos.pose.position.y :
					cmd.linear.y = -self.speed_corr

				if pos_dest.pose.position.z > self.last_pos.pose.position.z :
					cmd.linear.z = self.speed_corr
				elif pos_dest.pose.position.z < self.last_pos.pose.position.z :
					cmd.linear.z = -self.speed_corr

			if direction == "top" or direction == "bottom" :

				if pos_dest.pose.position.z > self.last_pos.pose.position.z :
					cmd.linear.z = self.speed
				else:
					cmd.linear.z = -self.speed

				#correction des erreurs
				if pos_dest.pose.position.x > self.last_pos.pose.position.x:
						cmd.linear.x = self.speed_corr
				elif pos_dest.pose.position.x < self.last_pos.pose.position.x :
					cmd.linear.x = -self.speed_corr
				if pos_dest.pose.position.y > self.last_pos.pose.position.y :
						cmd.linear.y = self.speed_corr
				elif pos_dest.pose.position.y < self.last_pos.pose.position.y :
					cmd.linear.y = -self.speed_corr

			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()
			cmd = Twist()

			#verifie que la consigne est valide
			if (direction == "forward" or direction == "back") and abs(pos_dest.pose.position.y - self.last_pos.pose.position.y) <= precision_lin :
				flag_condition = False

			elif (direction == "left" or direction == "right") and abs(pos_dest.pose.position.x - self.last_pos.pose.position.x) <= precision_lin :
				flag_condition = False

			elif (direction == "top" or direction == "bottom") and abs(pos_dest.pose.position.z - self.last_pos.pose.position.z) <= precision_alt :
				flag_condition = False

			else :
				flag_condition = True

			#verifie que l'on depasse pas la zone
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0 :
				if self.last_pos.pose.position.y > self.init_pos.pose.position.y + longueur_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.x > self.init_pos.pose.position.x + largeur_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.y < self.init_pos.pose.position.y - marge_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.x < self.init_pos.pose.position.x - marge_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.z > self.init_pos.pose.position.z + hauteur_zone :
					flag_depassement = True
				else : 
					flag_depassement = False


	"""	
	fonction pour faire tourner le drone sur lui même par la droite ou par la gauche avec un angle donné
	fonctionne avec une orientation en quaternion que le drone va tenter d'atteindre
	on transforme donc la valeur à atteindre de degré à radian puis en quaternion qu'on met comme objectif
	puis dans l'affichage on met l'évolution de la valeur d'angle
	"""
	def rotate(self,sens,angle):
		pos_start = PoseStamped()
		pos_start = self.last_pos

		pos_dest = PoseStamped()
		pos_dest = pos_start

		cmd = Twist()

		deg_x,deg_y,deg_z = quaternion_to_euler(pos_start.pose.orientation)
		if deg_z > 0:
			deg_z = deg_z
		else:
			deg_z = deg_z+360

		if sens == "right":
			print("rota droite :"+str(angle))
			deg_z_obj = (deg_z-angle)%360
		
		if sens == "left":
			print("rota gauche :"+str(angle))
			deg_z_obj = (deg_z+angle)%360	


		while deg_z > deg_z_obj + precision_ang or deg_z < deg_z_obj - precision_ang or flag_depassement :

			#correction des perturbations
			if pos_dest.pose.position.x > self.last_pos.pose.position.x:
				cmd.linear.x = self.speed_corr
			elif pos_dest.pose.position.x < self.last_pos.pose.position.x:
				cmd.linear.x = -self.speed_corr
			
			if pos_dest.pose.position.y > self.last_pos.pose.position.y:
				cmd.linear.y = self.speed_corr
			elif pos_dest.pose.position.y < self.last_pos.pose.position.y:
				cmd.linear.y = -self.speed_corr

			if pos_dest.pose.position.z > self.last_pos.pose.position.z:
				cmd.linear.z = self.speed_corr
			elif pos_dest.pose.position.z < self.last_pos.pose.position.z:
				cmd.linear.z = -self.speed_corr


			if sens == "right" :
				cmd.angular.z = -self.speed_ang
			else :
				cmd.angular.z = self.speed_ang

			#commande de l'angle
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()
			cmd = Twist()

			deg_x,deg_y,deg_z = quaternion_to_euler(self.last_pos.pose.orientation)
			if deg_z > 0:
				deg_z = deg_z
			else:
				deg_z = deg_z + 360

			#verifie que l'on depasse pas la zone
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0 :
				if self.last_pos.pose.position.y > self.init_pos.pose.position.y + longueur_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.x > self.init_pos.pose.position.x + largeur_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.y < self.init_pos.pose.position.y - marge_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.x < self.init_pos.pose.position.x - marge_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.z > self.init_pos.pose.position.z + hauteur_zone :
					flag_depassement = True
				else : 
					flag_depassement = False
				

	"""
	fonction pour attendre a une position donnée pendant un temps donné
	"""
	def wait(self,temps):

		print("wait "+str(temps)+"s")

		cmd = Twist()

		pos_dest = self.last_pos

		time_start = time.time()
		time_fin = time_start + temps

		while time_fin > time.time():

			#commande
			if pos_dest.pose.position.y > self.last_pos.pose.position.y:
				cmd.linear.y = self.speed_corr
			elif pos_dest.pose.position.y < self.last_pos.pose.position.y:
				cmd.linear.y = -self.speed_corr
			if pos_dest.pose.position.x > self.last_pos.pose.position.x:
				cmd.linear.x = self.speed_corr
			elif pos_dest.pose.position.x < self.last_pos.pose.position.x :
				cmd.linear.x = -self.speed_corr
			if pos_dest.pose.position.z > self.last_pos.pose.position.z:
				cmd.linear.z = self.speed_corr
			elif pos_dest.pose.position.z < self.last_pos.pose.position.z :
				cmd.linear.z = -self.speed_corr

			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()
			cmd = Twist()

	"""	
	fonction qui ramène à la position initiale
	"""
	def home(self):

		print("home")

		cmd = Twist()

		flag_depassement = False
		pos_dest = PoseStamped()
		pos_dest = self.init_pos
		#regle le x et le y à une altitude de 50cm
		pos_dest.pose.position.z = pos_dest.pose.position.z + 0.5

		deg_x,deg_y,deg_z = quaternion_to_euler(self.last_pos.pose.orientation)
		if deg_z > 0:
			deg_z = deg_z
		else:
			deg_z = deg_z+360

		deg_x,deg_y,deg_z_obj = quaternion_to_euler(pos_dest.pose.orientation)
		if deg_z_obj > 0:
			deg_z_obj = deg_z_obj
		else:
			deg_z_obj = deg_z_obj+360

		while abs(pos_dest.pose.position.x - self.last_pos.pose.position.x) >= precision_lin or abs(pos_dest.pose.position.y - self.last_pos.pose.position.y) >= precision_lin or abs(pos_dest.pose.position.z - self.last_pos.pose.position.z) >= precision_alt or (deg_z >= deg_z_obj + precision_ang or deg_z <= deg_z_obj - precision_ang) or flag_depassement:

			#commande
			if pos_dest.pose.position.y > self.last_pos.pose.position.y:
				cmd.linear.y = self.speed_corr
			elif pos_dest.pose.position.y < self.last_pos.pose.position.y:
				cmd.linear.y = -self.speed_corr
			if pos_dest.pose.position.x > self.last_pos.pose.position.x:
				cmd.linear.x = self.speed_corr
			elif pos_dest.pose.position.x < self.last_pos.pose.position.x:
				cmd.linear.x = -self.speed_corr
			if pos_dest.pose.position.z > self.last_pos.pose.position.z:
				cmd.linear.z = self.speed_corr
			elif pos_dest.pose.position.z < self.last_pos.pose.position.z:
				cmd.linear.z = -self.speed_corr
			if deg_z > deg_z_obj :
				cmd.angular.z = -self.speed_corr
			elif deg_z < deg_z_obj :
				cmd.angular.z = self.speed_corr

			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()
			cmd = Twist()

			deg_x,deg_y,deg_z = quaternion_to_euler(self.last_pos.pose.orientation)
			if deg_z > 0:
				deg_z = deg_z
			else:
				deg_z = deg_z + 360

			#verifie que l'on depasse pas la zone
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0 :
				if self.last_pos.pose.position.y > self.init_pos.pose.position.y + longueur_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.x > self.init_pos.pose.position.x + largeur_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.y < self.init_pos.pose.position.y - marge_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.x < self.init_pos.pose.position.x - marge_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.z > self.init_pos.pose.position.z + hauteur_zone :
						flag_depassement = True
				else :
					flag_depassement = False

		pos_dest.pose.position.z = self.init_pos.pose.position.z

		while abs(pos_dest.pose.position.z - self.last_pos.pose.position.z) >= precision_alt or flag_depassement :

			#correction des perturbations
			if pos_dest.pose.position.x > self.last_pos.pose.position.x:
				cmd.linear.x = self.speed_corr
			elif pos_dest.pose.position.x < self.last_pos.pose.position.x:
				cmd.linear.x = -self.speed_corr
			if pos_dest.pose.position.y > self.last_pos.pose.position.y:
				cmd.linear.y = self.speed_corr
			elif pos_dest.pose.position.y < self.last_pos.pose.position.y:
				cmd.linear.y = -self.speed_corr

			#commande de la direction principale
			if pos_dest.pose.position.z < self.last_pos.pose.position.z:
				cmd.linear.z = -self.speed_corr
			else:
				cmd.linear.z = self.speed_corr

			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()
			cmd = Twist()

			#verifie que l'on depasse pas la zone
			if largeur_zone > 0 and longueur_zone > 0 and hauteur_zone > 0:
				if self.last_pos.pose.position.y > self.init_pos.pose.position.y + longueur_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.x > self.init_pos.pose.position.x + largeur_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.y < self.init_pos.pose.position.y - marge_zone :
					flag_depassement = True
				elif self.last_pos.pose.position.x < self.init_pos.pose.position.x - marge_zone :
					flag_depassement = True
				else :
					flag_depassement = False


	######################EXAMPLES TRAJET#####################

	#deplacement préprogrammé qui fait un carré
	def square(self,distance):

		self.move("forward",distance)
		self.rotate("right",90)
		self.move("right",distance)
		self.rotate("right",90)
		self.move("back",distance)
		self.rotate("right",90)
		self.move("left",distance)
		self.rotate("right",90)


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
		
		node.arm()

		node.offboard_mode()
		
		node.move("top",2)

		#example de déplacement
		for i in range(3) :
			node.square(2)
			node.wait(1)

		node.home()

		node.disarm()

	except rospy.ROSInterruptException:
		pass