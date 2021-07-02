# PX4 Offboard PACKAGE

## Nécessite:

ROS
mavros

## Installation:

Télecharger le dossier
Mettre le dossier dans le source de catkin workspace
Faire un catkin_make
Faire source devel/setup.bash

## Arborescence:

launch -> contient les fichiers launch pour lancer les nodes
nodes -> contient les fichiers éxecutables pour intéragir avec les drones

## Fonctionnements:

Le package contient plusieurs nodes pour le contrôle à distance du drone tournant sous Mavros à distance

### Nodes de Contrôle à distance:

Il y a 3 types de nodes dans le package :

teleop_offb : permet de contrôler le drone en envoyant des entrées clavier pour diriger le drone en les traduisant en vitesse
autopilot_pose_offb : permet de contrôler le drone automatiquement en lui envoyant les positions qu'il doit atteindre
autopilot_vel_offb :  permet de contrôler le drone automatiquement en lui envoyant des vitesses jusqu'à ce qu'il atteigne la position voulue

### Nodes de Communication réseau:

Il y a aussi un node pour communiquer en udp depuis l'extérieur et renvoyer les positions du drone
udp_client_offb

### Paramètres des launch:

Chaque node à un fichier launch qui possède plusieurs paramètres : 

node_name : pour changer le nom de la node et pouvoir en lancer plusieurs
topic_name : la racine des topics mavros du drone

Besoin des 3 pour que la zone soit active

longueur_zone : pour limiter la zone dans laquelle le drone évolue
largeur_zone : pour limiter la zone dans laquelle le drone évolue
hauteur_zone : pour limiter la zone dans laquelle le drone évolue