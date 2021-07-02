#!/usr/bin/env python3

import rospy
import mavros
import roslib
import argparse
import math
import numpy as np
import time
import sys, select, termios, tty
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, Thrust 
from mavros_msgs.srv import CommandBool, SetMode


"""version 2 avec des zones pour chaque drone et une fonction home"""
""" note : se déplace relativement au repere du drone pas comme l'autre avec la position absolu"""

################VAR GLOBALE########################

todo = """
Reading from the keyboard
-------------------------
Moving around:
  z    
q   d
  s

w : up (+z)
x : down (-z)

a : rota gauche
e : rota droite

+ : speed +
- : speed -

c : speed angular +
v : speed angular -

& : home

anything else : stabilize


CTRL-C to quit
"""


#recupere les arguments du launch
topic_name = str(rospy.get_param('topic_name'))
node_name = str(rospy.get_param('node_name'))
longueur_zone = float(rospy.get_param('longueur_zone')) #axe y
largeur_zone = float(rospy.get_param('largeur_zone')) #axe x
hauteur_zone = float(rospy.get_param('hauteur_zone')) #axe z


precision_lin = 0.05 #5 cm d'écart par rapport à la distance voulue
precision_alt = 0.1 #10 cm d'écart par rapport à la distance voulue
precision_ang = 2 #2 degre d'ecart par rapport à l'angle voulu

marge_zone = 0.05 #5 cm de marge sur les bords du drone en bas à gauche de la zone
alt_minimum = 0.2 #20 cm d'altitude minimum avant de faire les contraintes de zones sinon trop dur de décoller

################TELEOP CLASS########################
"""
node pour le contrôle clavier du drone avec l'antenne pour la télémetrie.
"""
class TeleopOffboardNode:

	"""
	initialisation de la node de contrôle à distance du drone depuis le clavier
	"""
	def __init__(self):

		#init pos
		self.flag_init_pos = True #pour récupérer une fois la position initiale
		self.init_pos = PoseStamped()

		#last pos
		self.last_pos = PoseStamped()

		#speed
		self.speed = 0.2 # en m/s
		self.speed_ang = 0.5 # en rad/s
		self.speed_alt = 0.5 # en m/s
		self.speed_corr = 0.1 # en m/s
		self.speed_corr_ang = 0.2 # en rad/s


		# Start ROS node
		rospy.init_node(node_name)

		# Load subscribers, publishers, services

		self.cmd_vel_pub = rospy.Publisher("/"+topic_name+"/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
	
		self.state_sub = rospy.Subscriber("/"+topic_name+"/state", State, self.state_cb)
		self.pose_sub = rospy.Subscriber("/"+topic_name+"/local_position/pose", PoseStamped, self.pose_cb)
	
		self.arming_client = rospy.ServiceProxy("/"+topic_name+"/cmd/arming", CommandBool)
		self.set_mode_client = rospy.ServiceProxy("/"+topic_name+"/set_mode", SetMode)


		self.current_state = State()
		self.prev_state = self.current_state

		self.rate = rospy.Rate(50.0) # MUST be more then 2Hz

		self.last_request = 0
		self.now = 0

		rospy.loginfo('Teleop offboard node initialized')

		cmd = Twist()

		# send a few setpoints before starting
		for i in range(100):
			self.cmd_vel_pub.publish(cmd)
			self.rate.sleep()
		
		rospy.loginfo('Teleop offboard overflow cmd')

		# wait for FCU connection
		while not self.current_state.connected:
			self.rate.sleep()

		rospy.loginfo('Teleop offboard node connected')
		self.last_request = rospy.get_rostime()

	################CALLBACK FUNCTION####################

	"""
	fonction de callback pour raffraichir la variable état du drone pour le conserver en offboard et armé
	"""
	def state_cb(self,state):
		self.current_state = state

	"""
	fonction de callback pour raffraichir la variable de position du drone
	recupère la position initiale au debut
	"""
	def pose_cb(self,pose):

		if self.flag_init_pos == True:
			self.flag_init_pos = False
			self.init_pos = pose

		self.last_pos = pose

	################METHODES####################

	"""
	fonction qui raffraichi l'état de pilotage du drone sur offboard pour pouvoir lui envoyer des instructions depuis l'ordinateur
	elle sert aussi à armer le drone (peut être à séparer dans une autre fonction)
	"""
	def offboard_mode(self):
		self.now = rospy.get_rostime()
		if self.current_state.mode != "OFFBOARD" and (self.now - self.last_request > rospy.Duration(5.)):
			self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
			self.last_request = self.now 
		else:
			if not self.current_state.armed and (self.now - self.last_request > rospy.Duration(5.)):
				self.arming_client(True)
				self.last_request = self.now 

		# older versions of PX4 always return success==True, so better to check Status instead
		if self.prev_state.armed != self.current_state.armed:
			rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
		if self.prev_state.mode != self.current_state.mode: 
			rospy.loginfo("Current mode: %s" % self.current_state.mode)
		self.prev_state = self.current_state
		self.rate.sleep()


	"""
	fonction qui en fonction de l'entrée clavier envoie une consigne de vitesse différente que le drone va essayer d'atteindre pour contrôler la trajectoire du drone
	depuis un clavier à distance
	"""
	def key_action(self):
		cmd = Twist()

		#aquisition de la touche clavier
		key = ''
	
		tty.setraw(sys.stdin.fileno())

		rlist, _, _ = select.select([sys.stdin], [], [], 0.5)
		if rlist:
			key = sys.stdin.read(1)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

		#arrêt de la node si controle + c
		if(key == '\x03'):
			#plus disarm
			self.arming_client(False)
			self.rate.sleep()
			return 0

		if(key == '&'):
			self.home()
			self.rate.sleep()
			return 0	
				
		#envoie de la bonne consigne pour se déplacer dans la direction voulu
		if key == 'z':
			cmd.linear.y = self.speed
			print("speed forward (y): ")
			print(cmd.linear.y)
		if key == 's':
			cmd.linear.y = -self.speed
			print("speed back (y) : ")
			print(cmd.linear.y)
		if key == 'd':
			cmd.linear.x = self.speed
			print("speed right (x): ")
			print(cmd.linear.x)
		if key == 'q':
			cmd.linear.x = -self.speed
			print("speed left (x): ")
			print(cmd.linear.x)
		if key == 'w':
			cmd.linear.z = self.speed_alt
			print("height + (z): ")
			print(cmd.linear.z)
		if key == 'x':
			cmd.linear.z = -self.speed_alt
			print("height - (z): ")
			print(cmd.linear.z)
		if key == 'a':
			cmd.angular.z = self.speed_ang
			print("rotation left (z): ")
			print(cmd.angular.z)
		if key == 'e':
			cmd.angular.z = -self.speed_ang
			print("rotation right (z): ")
			print(cmd.angular.z)
	
		#gestion de la vitesse
		if key == '+':
			self.speed = self.speed + 0.1
			print("speed linear: ")
			print(self.speed)
		if key == '-':
			self.speed = self.speed - 0.1
			if self.speed < 0:
				self.speed = 0
			print("speed linear: ")
			print(self.speed)
			
		if key == 'c':
			self.speed_ang = self.speed_ang + 0.1
			print("speed angular: ")
			print(self.speed_ang)
		if key == 'v':
			self.speed_ang = self.speed_ang - 0.1
			if self.speed_ang < 0:
				self.speed_ang = 0
			print("speed angular: ")
			print(self.speed_ang)

		#envoie de l'objectif de vitesse
		self.cmd_vel_pub.publish(cmd)
		self.rate.sleep()
		return 1

	"""
	marche pas pour l'instant
	fonction pour verif qu'on est bien dans la zone et corriger si ce n'est pas le cas
	"""
	def verif_zone(self):
		cmd = Twist()
		flag_depassement = False

		#sinon on arrive jamais à décoller
		if longueur_zone > 0 and largeur_zone > 0 and hauteur_zone > 0 :

			if self.last_pos.pose.position.y > self.init_pos.pose.position.y + longueur_zone :
				flag_depassement = True
			if self.last_pos.pose.position.x > self.init_pos.pose.position.x + largeur_zone :
				flag_depassement = True
			if self.last_pos.pose.position.y < self.init_pos.pose.position.y - marge_zone :
				flag_depassement = True
			if self.last_pos.pose.position.x < self.init_pos.pose.position.x - marge_zone :
				flag_depassement = True
			if self.last_pos.pose.position.z > self.init_pos.pose.position.z + hauteur_zone :
				flag_depassement = True
			#sinon on decolle jamais a cause de la derive
			if self.last_pos.pose.position.z < self.init_pos.pose.position.z + alt_minimum :
				flag_depassement = False

			while flag_depassement :

				print("#####")
				print("init")
				print(self.init_pos.pose.position)
				print("actual")
				print(self.last_pos.pose.position)

				if self.last_pos.pose.position.y > self.init_pos.pose.position.y + longueur_zone :
					cmd.linear.y = -self.speed_corr
				if self.last_pos.pose.position.y < self.init_pos.pose.position.y :
					cmd.linear.y = self.speed_corr
				if self.last_pos.pose.position.x > self.init_pos.pose.position.x + largeur_zone :
					cmd.linear.x = -self.speed_corr
				if self.last_pos.pose.position.x < self.init_pos.pose.position.y :
					cmd.linear.x = self.speed_corr
				if self.last_pos.pose.position.z > self.init_pos.pose.position.z + hauteur_zone :
					cmd.linear.z = -self.speed_corr
				
				self.cmd_vel_pub.publish(cmd)
				self.rate.sleep()
				cmd = Twist()

				#condition de fin
				flag_depassement = False

				if self.last_pos.pose.position.y > self.init_pos.pose.position.y + longueur_zone :
					flag_depassement = True
				if self.last_pos.pose.position.y < self.init_pos.pose.position.y :
					flag_depassement = True
				if self.last_pos.pose.position.x > self.init_pos.pose.position.x + largeur_zone :
					flag_depassement = True
				if self.last_pos.pose.position.x < self.init_pos.pose.position.x :
					flag_depassement = True
				if self.last_pos.pose.position.z > self.init_pos.pose.position.z + hauteur_zone :
					flag_depassement = True
				if self.last_pos.pose.position.z < self.init_pos.pose.position.z + alt_minimum :
					flag_depassement = False

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
				cmd.angular.z = -self.speed_corr_ang
			elif deg_z < deg_z_obj :
				cmd.angular.z = self.speed_corr_ang

			self.offboard_mode()
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

			self.offboard_mode()
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

		res = 1
		settings = termios.tcgetattr(sys.stdin)
		node = TeleopOffboardNode()
		print(todo)

		while res :
			node.offboard_mode()
			res = node.key_action()
			node.verif_zone()

	except rospy.ROSInterruptException:
		pass