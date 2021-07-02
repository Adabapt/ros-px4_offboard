#!/usr/bin/env python3

import rospy
import mavros
import roslib
import argparse
import sys, select, termios, tty
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, Thrust 
from mavros_msgs.srv import CommandBool, SetMode

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

anything else : stabilize


CTRL-C to quit
"""


#recupère les arguments du launch
topic_name = str(rospy.get_param('topic_name'))
node_name = str(rospy.get_param('node_name'))

################TELEOP CLASS########################
"""
node pour le contrôle clavier du drone avec l'antenne pour la télémetrie.
"""
class TeleopOffboardNode:

	"""
	initialisation de la node de contrôle à distance du drone depuis le clavier
	"""
	def __init__(self):

		#speed
		self.speed = 0.5 # en m/s
		self.speed_ang = 0.2 # en rad/s
		self.speed_alt = 0.5 # en m/s

		# Start ROS node
		rospy.init_node(node_name)

		# Load subscribers, publishers, services

		self.cmd_vel_pub = rospy.Publisher("/"+topic_name+"/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
	
		self.state_sub = rospy.Subscriber("/"+topic_name+"/state", State, self.state_cb)
	
		self.arming_client = rospy.ServiceProxy("/"+topic_name+"/cmd/arming", CommandBool)
		self.set_mode_client = rospy.ServiceProxy("/"+topic_name+"/set_mode", SetMode)


		self.current_state = State()
		self.prev_state = self.current_state

		self.rate = rospy.Rate(50.0) # MUST be more then 2Hz

		self.last_request = 0
		self.now = 0

		rospy.loginfo('Teleop offboard node initialized')

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
		cmd.linear.x = 0
		cmd.linear.y = 0
		cmd.linear.z = 0
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = 0

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

####################MAIN###############################

if __name__ == '__main__':
	try:

		res = 1
		settings = termios.tcgetattr(sys.stdin)
		node = TeleopOffboardNode()
		print(todo)

		while(res):
			node.offboard_mode()
			res = node.key_action()

	except rospy.ROSInterruptException:
		pass