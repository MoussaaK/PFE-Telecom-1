#-*- coding: utf-8 -*-
from __future__ import print_function # pour compatibilte avec python 2
from __future__ import division       #    

import brickpi3
import time
from threading import Thread    
import sys		 #systeme d’exploitation 
import cv2		 #opencv
import numpy as np	 #permettra l’affichage du contenu de la camera 


# A assurer avec Python2 ou Python3
if sys.version_info[0] < 3:
    input = raw_input


#Initialisation des ports
def iniatialise_ports():
    BP = brickpi3.BrickPi3() #Instanciation de l'objet Bp
    
    #Premier capteur-avant port S1
    BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    #Second capteur-avant port S2   
    BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    #Troisieme capteur-avant port S3     
    BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.TOUCH)
    #bouton arret d'urgence port S4             
    BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.TOUCH)             

class SecondCounter(Thread):
    '''
    On cree un objet qui comptera les 90 secondes en arrière plan
    En utilisant le multithreading

    Cet objet contient les méthodes :
    -   run     --- lance le thread
    -   peek    --- donne le temps actuel
    -   finish  --- arrete le thread
    '''
    def __init__(self, interval=1):
        # initialisation de la thread
        Thread.__init__(self)
        self.interval = interval  # 1 seconde
        # initialisation du compteur
        self.value = 0
        # Condition de sortie de la boucle while du runner
        self.alive = False

    def run(self):
        '''
        Quand on instancie un objet, la méthode run lance le chrono 
        '''
        self.alive = True
        while self.alive:
            time.sleep(self.interval)
            # update count value
            self.value += self.interval
            
    def peek(self):
        '''
        retourne la temps actuel
        '''
        return self.value

    def finish(self):
        '''
        ferme la thread et retourne la valeur finale
        '''
        # On remet a False self.alive pour quitter la boiucle du runner
        self.alive = False
        return self.value


class Deplacements:
    """ 
    Class qui assure les deplacements contient :
        - avancer
        - reculer
        - arret
        - tourner_a_gauche
        - tourner_a_droite
        - rotation_90_a_gauche
        - rotation_90_a_droite
        - rotation_360_a_gauche
        - rotation_360_a_droite
        - detection_obstacle
        - arret_durgence
        - demarrage
        - fin_du_match
    """
    
    def __init__(self):
        # initialisation de la thread
        Thread.__init__(self)
        self.BP = brickpi3.BrickPi3()     #Instanciation de l'objet BP
        self.interval = interval  # 1 seconde
        # initialisation du compteur
        self.value = 0
        # Condition de sortie de la boucle while du runner
        self.alive = False
        
    def avancer(self, SPEED, DISTANCE):
        #iniatialise_ports
        TARGET = 1800*DISTANCE    # 10 * 180 degrees correspond a un placement de 25 cm
        Robot = Actions()
        arret_durgence =  Robot.demarrage()
        # On remet a zeros la position des moteurs
        self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
        # on fixe la vitesse a ne pas depasser ppar le moteur
        # 1.1 represente le coeffecient pour faire tourner les deux moteurs
        # a la meme vitesse vu que celui de droite allait plus vite 
        self.BP.set_motor_limits(self.BP.PORT_A, 0.5*SPEED*1.1)
        self.BP.set_motor_limits(self.BP.PORT_B, 0.5*SPEED)
        # on fixe la distance a atteindre par le moteur
        self.BP.set_motor_position(self.BP.PORT_A, TARGET)   #Moteur gauche
        self.BP.set_motor_position(self.BP.PORT_B, TARGET)
        
        while (self.BP.get_motor_encoder(self.BP.PORT_A)[0] < TARGET-90) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] < TARGET-90) and (arret_durgence != 1) :
            obstacle = Robot.detection_obstacle()
            arret_durgence =  Robot.demarrage()
            # Phase dacceleration - deceleration :
            # En fonction de la distance fournie en parametre, on va subdiviser cette distance 
            # en 10 intervalles sur lesquelles le robot se deplacera a vitesse non constante
            if(obstacle != True):                
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.1*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.2*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.1*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.2*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.6*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.6*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.2*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.3*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.2*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.3*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.7*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.7*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.3*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.4*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.3*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.4*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.8*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.8*SPEED) 
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.4*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.5*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.4*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.5*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.9*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.9*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.5*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.6*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.5*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.6*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.6*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.7*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.6*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.7*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.9*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.9*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.7*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.8*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.7*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.8*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.8*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.8*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.8*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.9*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.8*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.9*TARGET): 
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.7*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.7*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.9*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.9*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.6*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.6*SPEED)
            else:
                while (obstacle == True):
                    # s.il ya un obstacle on met la vitesse a zero
                    self.BP.set_motor_speed(self.BP.PORT_A, 0)
                    self.BP.set_motor_speed(self.BP.PORT_B, 0)
                    #on update la valeur de obstacle
                    obstacle = Robot.detection_obstacle()
                # On remet a zeros la position des moteurs
                self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
                self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
                # on remet la vitesse du debut du trajet
                self.BP.set_motor_speed(self.BP.PORT_A, 0.5*SPEED*1.1)
                self.BP.set_motor_speed(self.BP.PORT_B, 0.5*SPEED)
                # Et on limite cette vitesse 
                self.BP.set_motor_limits(self.BP.PORT_A, 0.5*SPEED*1.1)
                self.BP.set_motor_limits(self.BP.PORT_B, 0.5*SPEED)
                # vu qu.on a pas atteint la position qu.on devrait du a lobstacle on remet a jour
                # la distance suivante a atteindre en enlevant la distance deja parcourrue
                TARGET = TARGET - self.BP.get_motor_encoder(self.BP.PORT_B)[0]
                self.BP.set_motor_position(self.BP.PORT_A, TARGET)   #Moteur gauche
                self.BP.set_motor_position(self.BP.PORT_B, TARGET) 
                #print("Arret obstacle")
                
        if(arret_durgence == 1):
            Robot.BP.reset_all()            #remet tout a zeros
            print("Arret Durgence")
        print("Avancer")
        
    def reculer(self, SPEED, DISTANCE):
        #iniatialise_ports
        TARGET = -1800*DISTANCE    # 10 * 180 degrees correspond a un placement de 25 cm
        Robot = Actions()
        self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
        self.BP.set_motor_limits(self.BP.PORT_A, 0.5*SPEED*1.1)
        self.BP.set_motor_limits(self.BP.PORT_B, 0.5*SPEED)
        self.BP.set_motor_position(self.BP.PORT_A, TARGET)            #Moteur gauche
        self.BP.set_motor_position(self.BP.PORT_B, TARGET) 
        
        while (self.BP.get_motor_encoder(self.BP.PORT_A)[0] > TARGET+90) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] > TARGET+90):  
            obstacle = Robot.detection_obstacle_arriere()
            if(obstacle != True):  
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.1*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.2*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.1*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.2*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.6*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.6*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.2*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.3*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.2*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.3*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.7*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.7*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.3*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.4*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.3*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.4*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.8*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.8*SPEED) 
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.4*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.5*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.4*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.5*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.9*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.9*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.5*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.6*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.5*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.6*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.6*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.7*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.6*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.7*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.9*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.9*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.7*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.8*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.7*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.8*TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.8*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.8*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.8*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= 0.9*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.8*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= 0.9*TARGET): 
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.7*SPEED*1.1)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.7*SPEED)
                if (self.BP.get_motor_encoder(self.BP.PORT_A)[0] <= 0.9*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_A)[0] >= TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] <= 0.9*TARGET) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] >= TARGET):
                    self.BP.set_motor_limits(self.BP.PORT_A, 0.6*SPEED*1.11)
                    self.BP.set_motor_limits(self.BP.PORT_B, 0.6*SPEED)
            else:
                while (obstacle == True):
                    self.BP.set_motor_speed(self.BP.PORT_A, 0)
                    self.BP.set_motor_speed(self.BP.PORT_B, 0)
                    obstacle = Robot.detection_obstacle_arriere()
                TARGET = -(TARGET - self.BP.get_motor_encoder(self.BP.PORT_B)[0])
                self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
                self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
                self.BP.set_motor_speed(self.BP.PORT_A, 0.5*SPEED*1.15)
                self.BP.set_motor_speed(self.BP.PORT_B, 0.5*SPEED)
                self.BP.set_motor_limits(self.BP.PORT_A, 0.5*SPEED*1.15)
                self.BP.set_motor_limits(self.BP.PORT_B, 0.5*SPEED)
                self.BP.set_motor_position(self.BP.PORT_A, TARGET)            #Moteur gauche
                self.BP.set_motor_position(self.BP.PORT_B, TARGET) 
                #print("Arret obstacle")

            arret_durgence = Robot.demarrage()
            if(arret_durgence == 1):
                Robot.BP.reset_all()            #remet tout a zeros       
                print("Arret Durgence")
        print("Reculer")
        
    def arret(self):
        #iniatialise_ports()
        self.BP.set_motor_speed(self.BP.PORT_A, 0)                  #Moteur gauche
        self.BP.set_motor_speed(self.BP.PORT_B, 0)                  #Moteur droite
        self.BP.set_motor_speed(self.BP.PORT_C, 0)
        self.BP.set_motor_speed(self.BP.PORT_D, 0)
        print("Arret")
        #self.BP.reset_all()                    # arrete tout meme les capteurs

    def tourner_a_gauche(self):
        #iniatialise_ports()
        TARGET = 250
        Robot = Actions()
        obstacle = Robot.detection_obstacle()
        time.sleep(0.5) # on laisse les deux roues se mettrent au meme niveau
        self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
        self.BP.set_motor_position(self.BP.PORT_A, -TARGET)         #Moteur gauche
        self.BP.set_motor_position(self.BP.PORT_B, TARGET)          
        
        while(self.BP.get_motor_encoder(self.BP.PORT_A)[0] < TARGET*0.9) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] < TARGET*0.9):  
            obstacle = Robot.detection_obstacle()
            if(obstacle == True):
                while (obstacle == True):
                    self.BP.set_motor_speed(self.BP.PORT_A, 0)
                    self.BP.set_motor_speed(self.BP.PORT_B, 0)
                    obstacle = Robot.detection_obstacle()
                    print("Arret obstacle")
                TARGET = TARGET - self.BP.get_motor_encoder(self.BP.PORT_B)[0]
                self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
                self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
                self.BP.set_motor_position(self.BP.PORT_A, -TARGET)            #Moteur gauche
                self.BP.set_motor_position(self.BP.PORT_B, TARGET)
            print("Tourner a gauche")
            arret_durgence = Robot.demarrage()
            if(arret_durgence == 1):
                Robot.BP.reset_all()            #remet tout a zeros
                print("Arret Durgence")

    def tourner_a_droite(self):
        #iniatialise_ports()
        TARGET = 250
        Robot = Actions()
        obstacle = Robot.detection_obstacle()
        time.sleep(0.5) # on laisse les deux roues se mettrent au meme niveau
        self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
        self.BP.set_motor_position(self.BP.PORT_A, TARGET)           #Moteur gauche
        self.BP.set_motor_position(self.BP.PORT_B, -TARGET)          #Moteur droite
        
        while(self.BP.get_motor_encoder(self.BP.PORT_A)[0] < TARGET*0.9) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] < TARGET*0.9):
            obstacle = Robot.detection_obstacle()
            if(obstacle == True):
                while (obstacle == True):
                    self.BP.set_motor_speed(self.BP.PORT_A, 0)
                    self.BP.set_motor_speed(self.BP.PORT_B, 0)
                    obstacle = Robot.detection_obstacle()
                    print("Arret obstacle")
                TARGET = TARGET - self.BP.get_motor_encoder(self.BP.PORT_B)[0]
                self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
                self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
                self.BP.set_motor_position(self.BP.PORT_A, TARGET)            #Moteur gauche
                self.BP.set_motor_position(self.BP.PORT_B, -TARGET)
            print("Tourner a droite")
            arret_durgence = Robot.demarrage()
            if(arret_durgence == 1):
                Robot.BP.reset_all()            #remet tout a zeros
                print("Arret Durgence")
        
    def rotation(self, ANGLE):
        '''
        methode qui assure les rotations dans les deux sens (gauche, droite)
        '''
        TARGET = 250*ANGLE
        Robot = Actions()
        obstacle = Robot.detection_obstacle()
        time.sleep(0.5) # on laisse les deux roues se mettrent au meme niveau
        self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
        self.BP.set_motor_limits(self.BP.PORT_A, 50)
        self.BP.set_motor_limits(self.BP.PORT_B, 50)
        self.BP.set_motor_position(self.BP.PORT_A, TARGET)            #Moteur gauche
        self.BP.set_motor_position(self.BP.PORT_B, -TARGET)
        
        while(self.BP.get_motor_encoder(self.BP.PORT_A)[0] < TARGET*0.9) and (self.BP.get_motor_encoder(self.BP.PORT_B)[0] < TARGET*0.9):
            obstacle = Robot.detection_obstacle()
            if(obstacle == True):
                while (obstacle == True):
                    self.BP.set_motor_speed(self.BP.PORT_A, 0)
                    self.BP.set_motor_speed(self.BP.PORT_B, 0)
                    obstacle = Robot.detection_obstacle()
                    print("Arret obstacle")
                TARGET = TARGET - self.BP.get_motor_encoder(self.BP.PORT_B)[0]
                self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
                self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
                self.BP.set_motor_position(self.BP.PORT_A, TARGET)            #Moteur gauche
                self.BP.set_motor_position(self.BP.PORT_B, -TARGET) 
                print("Rotation de", TARGET-160, " Degrees")

            arret_durgence = Robot.demarrage()
            if(arret_durgence == 1):
                Robot.BP.reset_all()            #remet tout a zeros
                print("Arret Durgence")
                

    def rotation_gauche(self, ANGLE):
        '''
        methode qui assure les rotations dans le sens gauche
        '''
        TARGET = 250*ANGLE
        time.sleep(0.5) # on laisse les deux roue se mettre au meme niveau
        Robot = Actions()
        obstacle = Robot.detection_obstacle_proche()
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
        self.BP.set_motor_position(self.BP.PORT_B, TARGET)
        self.BP.set_motor_limits(self.BP.PORT_B, 50)
        while(self.BP.get_motor_encoder(self.BP.PORT_B)[0] < TARGET*0.9):  
            obstacle = Robot.detection_obstacle_proche()
            if(obstacle == True):
                while (obstacle == True):
                    self.BP.set_motor_speed(self.BP.PORT_B, 0)
                    obstacle = Robot.detection_obstacle_proche()
                    print("Arret obstacle")
                TARGET = TARGET - self.BP.get_motor_encoder(self.BP.PORT_B)[0]
                self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)[0])
                self.BP.set_motor_position(self.BP.PORT_B, TARGET)
                
            arret_durgence = Robot.demarrage()
            if(arret_durgence == 1):
                Robot.BP.reset_all()            #remet tout a zeros
                print("Arret Durgence")
        print("Rotation a gauche ", TARGET-160, "Degrees")
        
    def rotation_droite(self, ANGLE):
        '''
        methode qui assure les rotations dans le sens droite
        '''
        TARGET = 250*ANGLE
        time.sleep(0.5) # on laisse les deux roue se mettre au meme niveau
        Robot = Actions()
        obstacle = Robot.detection_obstacle()
        self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
        self.BP.set_motor_position(self.BP.PORT_A, TARGET)
        self.BP.set_motor_limits(self.BP.PORT_B, 50)
        while(self.BP.get_motor_encoder(self.BP.PORT_A)[0] < TARGET*0.9):
            obstacle = Robot.detection_obstacle()
            if(obstacle == True):
                while (obstacle == True):
                    self.BP.set_motor_speed(self.BP.PORT_A, 0)
                    obstacle = Robot.detection_obstacle()
                    print("Arret obstacle")
                TARGET = TARGET - self.BP.get_motor_encoder(self.BP.PORT_A)[0]
                self.BP.offset_motor_encoder(self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A)[0])
                self.BP.set_motor_position(self.BP.PORT_A, TARGET)

            arret_durgence = Robot.demarrage()
            if(arret_durgence == 1):
                Robot.BP.reset_all()            #remet tout a zeros
                print("Arret Durgence")
        print("Rotation a droite", TARGET-160, " Degrees")

    
    def detection_obstacle(self):
        '''
        methode qui assure la detection d'obstacles en avant du robot
        -   Retourne True s'il y'a un obstacle
        -   False sinon
        '''
        obstacle2 = self.BP.get_sensor(self.BP.PORT_2)[0]
        #obstacle3 = self.BP.get_sensor(self.BP.PORT_3)[0]
        if(obstacle2 > 8):
            return False
        else:
            return True

    def detection_obstacle_proche(self):
        '''
        methode qui assure la detection d'obstacles tres tres proches en avant du robot
        -   Retourne True s'il y'a un obstacle
        -   False sinon
        '''
        obstacle2 = self.BP.get_sensor(self.BP.PORT_2)[0]
        #obstacle3 = self.BP.get_sensor(self.BP.PORT_3)[0]
        if(obstacle2 > 2):
            return False
        else:
            #return True
            return False
        
    #detection d'obstacle à l'arriere
    def detection_obstacle_arriere(self):
        '''
        methode qui assure la detection d'obstacles a l'arriere du robot
        -   Retourne True s'il y'a un obstacle
        -   False sinon
        '''
        obstacle1 = self.BP.get_sensor(self.BP.PORT_1)[0]
        #obstacle4 = self.BP.get_sensor(self.BP.PORT_4)[0]
        if(obstacle1 > 2):
            return False
        else:
            return True

    def arret_durgence(self):
        '''
        methode qui assure l'arret d'urgence du robot
        '''
        try:
            arret = self.BP.get_sensor(self.BP.PORT_2)[0]
            time.sleep(0.02)                                   #on le laisse lire la valeur pendant 20ms
        except brickpi3.SensorError as error:
                print(error)
        if (arret == 1):
            arret()
            print("Arret d'urgence")
            return True
        else:
            return False
             
    def demarrage(self):
        '''
        methode qui assure le demarrage en appuyant sur le bouton demarrage
        '''
        try:    #on capture la valeur du capteur si on narrive pas, on renvoie un code d'erreur 
                bouton_demarrage = self.BP.get_sensor(self.BP.PORT_4)[0]
                #time.sleep(0.02) #on le laisse lire la valeur pendant 20ms
        except brickpi3.SensorError as error:
                print(error)
        return bouton_demarrage #if bouton_demarrage = 1 on demarre sinon rien developper dans le main

    
    def tirette(self):
        #tirette port 3 
        tirette = self.BP.get_sensor(self.BP.PORT_3)[0]
        time.sleep(0.02)                                   #on le laisse lire la valeur pendant 20ms
        if (tirette == 1):
            return True
        else:
            return False




#Class qui herite de deplacements            
class Actions(Deplacements):
    """ Class qui assure les actions contient:
    ------- ouvrir_main
    ------------ fermer_main
    ----------------- funny_action
    ----------------------renverser_cylindre
    -------------------------retour_renverser_cylindre
    ------------------------------------------attrape_lache                          """
    
    def __init__(self):
        self.BP = brickpi3.BrickPi3()      #Instanciation de l'objet BP

    def ouvrir_main(self):
        '''
        methode qui assure l'ouverture de la pince pour attraper un module lunaire
        '''
        
        arret_durgence = Robot.demarrage()
        if(arret_durgence == 1):
            Robot.BP.reset_all()            #remet tout a zeros
            print("Arret Durgence")
        DEGREES = 90
        self.BP.set_motor_limits(self.BP.PORT_C, 20)
        self.BP.set_motor_position(self.BP.PORT_C, DEGREES)
        self.BP.offset_motor_encoder(self.BP.PORT_C, self.BP.get_motor_encoder(self.BP.PORT_C)[0])
        print("Lacher Cylindre")
        
    def fermer_main(self):
        #iniatialise_ports()

        arret_durgence = Robot.demarrage()
        if(arret_durgence == 1):
            Robot.BP.reset_all()            #remet tout a zeros
            print("Arret Durgence")
        
        DEGREES = -100
        self.BP.set_motor_limits(self.BP.PORT_C, 70)
        self.BP.set_motor_position(self.BP.PORT_C, DEGREES)
        position = self.BP.get_motor_encoder(self.BP.PORT_C)[0]
        #self.BP.offset_motor_encoder(self.BP.PORT_C, position)
        print("Attraper Cylindre")
        
    def funny_action(self):
        arret_durgence = Robot.demarrage()
        if(arret_durgence == 1):
            Robot.BP.reset_all()            #remet tout a zeros
            print("Arret Durgence")
            
        self.BP.set_motor_position(self.BP.PORT_D, 90)
        position = self.BP.get_motor_encoder(self.BP.PORT_D)[0]
        self.BP.offset_motor_encoder(self.BP.PORT_D, position)
        #position = self.BP.get_motor_encoder(self.BP.PORT_C)[0]


    def renverser_cylindre(self):
        #iniatialise_ports()
        DEGREES = -180
        arret_durgence = Robot.demarrage()
        if(arret_durgence == 1):
            Robot.BP.reset_all()            #remet tout a zeros
            print("Arret Durgence")
            
        self.BP.set_motor_position(self.BP.PORT_D, DEGREES)
        position = self.BP.get_motor_encoder(self.BP.PORT_D)[0]
        self.BP.offset_motor_encoder(self.BP.PORT_D, position)
        print("renverser Cylindre")


    def retour_renverser_cylindre(self):
        #iniatialise_ports()
        DEGREES = 180
        arret_durgence = Robot.demarrage()
        if(arret_durgence == 1):
            Robot.BP.reset_all()            #remet tout a zeros
            print("Arret Durgence")
            
        self.BP.set_motor_position(self.BP.PORT_D, DEGREES)
        position = self.BP.get_motor_encoder(self.BP.PORT_D)[0]
        self.BP.offset_motor_encoder(self.BP.PORT_D, position)
        print("Retour renverser Cylindre")
        

    def attrape_lache(self):
        
        arret_durgence = Robot.demarrage()
        if(arret_durgence == 1):
            Robot.BP.reset_all()            #remet tout a zeros
            print("Arret Durgence")
            
        Robot = Actions()
        Robot.fermer_main()
        #Robot.arret()
        time.sleep(5)
        #Robot.retour_renverser_cylindre()
        Robot.renverser_cylindre()
        time.sleep(2)
        Robot.retour_renverser_cylindre()
        #time.sleep(5)
        Robot.ouvrir_main()

class DetectionObjets(Deplacements, Actions):
    def ouvertureCamera(self):
        #ouverture de la camera en utilisant opencv2
        #On a laisser tomber la detection d'objet en utilisant la camera par soucis de temps
        # vu qu'on etait en retard par rapport a la competition
        # on s'est plus base sur les donnees de l'encodeur des moteurs en estimant le trajet 
        # necessaire pour aller d'objet en objet afin de gagner des points

        Webcam = VideoCapture(1)
        while True:
            ret, frame = webcam.read()
            cv2.imshow('frame', frame)

            if cv2.waitKey(0):
                break

        Webcam.release()
        cv2.destroyAllWindows

def GOmatch_bleue():
    #methode fait avancer jusqu'au premier module, le recupere
    #et retourne jusqu'a la base et fait dix points
     
    try:
        Robot = Actions()
 
        arret_durgence = 1
        obstacle1 = Robot.detection_obstacle()
        
        #Boucle infinie pour le demarrage
        booll = True
        while booll:
            arret_durgence =  Robot.demarrage()
            print("on fait rien, on attend le demarrage")
            if arret_durgence == 0:
                booll = False
        print("Couleur bleue")
        count = SecondCounter()
        #debut du chrono
        count.start()
        # pour prendre en compte larret durgence
        obstacle1 = Robot.detection_obstacle()
        temps_initial = count.peek()  
        temps = count.peek() 
        boolle = True
        while boolle and temps <= 76:
            arret_durgence =  Robot.demarrage()
            temps = count.peek()

            Robot.ouvrir_main()
            time.sleep(0.5)
            Robot.avancer(70, 0.45)
            time.sleep(0.5)
            Robot.fermer_main()
            time.sleep(0.5)
            Robot.reculer(50,0.45)

            time.sleep(3)
            Robot.rotation(-1.2)
            time.sleep(2)
            Robot.ouvrir_main()
            time.sleep(0.5)
            Robot.reculer(50,0.3)
            time.sleep(2)
            Robot.rotation(-0.5)
            time.sleep(2)
           

            #Rajout
            #time.sleep(1.5)
            #Robot.rotation_gauche(2)
            #time.sleep(0.5)
            #Robot.avancer(50, 0.1)
            #time.sleep(0.5)
            #Robot.reculer(50, 0.1)
            #time.sleep(2)
            #Robot.rotation_gauche(3.6)
            
            Robot.arret()
            temps = count.peek()
            if (arret_durgence == 1) or(temps >76):
                boolle = False
            break
        Robot.BP.reset_all()
        count.finish()
        print("la fonction a pris {} seconds pour terminer".format(temps))
    except KeyboardInterrupt: # exception pour gerer l'appui sur Ctrl+C.
        Robot.BP.reset_all()

def homologuation():
    try:
        Robot = Actions()
        count = SecondCounter()
        #debut du chrono
        count.start()
        arret_durgence = 1
        obstacle = Robot.detection_obstacle()
        
        #pour le demarrage
        temps_initial = count.peek()  
        booll = True
        while booll:
            arret_durgence =  Robot.demarrage()
            print("on fait rien, on attend le demarrage")
            if arret_durgence == 0:
                booll = False
                
        # pour prendre en compte larret durgence
        temps = count.peek() 
        boolle = True
        obstacle = Robot.detection_obstacle()
        while boolle:
            arret_durgence =  Robot.demarrage()
            obstacle = Robot.detection_obstacle()
            temps = count.peek()
            
            while (temps - temps_initial < 90)and (obstacle != True):
                #temps = count.peek()
                if(temps - temps_initial < 8):
                    Robot.avancer(200)
                    
                if (temps - temps_initial >= 8) and (temps - temps_initial < 9):
                    Robot.tourner_a_droite(200)
                    Robot.fermer_main()
                    time.sleep(2)
                    Robot.ouvrir_main()
                #else:
                    #time.sleep(2)
                    
                temps = count.peek()
                obstacle = Robot.detection_obstacle()
                arret_durgence =  Robot.demarrage()

                if (arret_durgence == 1):
                    Robot.arret()
                    break

            temps = count.peek()
            #time.sleep()
            if (temps + temps_initial >= 91) and (temps + temps_initial <= 101):
                Robot.BP.set_motor_speed(Robot.BP.PORT_A, 0)      #Moteur gauche
                Robot.BP.set_motor_speed(Robot.BP.PORT_B, 0)
                Robot.funny_action()
                arret_durgence =  Robot.demarrage()
                if arret_durgence == 1:
                    boolle = False
                    
            temps_funny = temps + temps_initial
            if (arret_durgence == 1) or temps_funny > 101:
                boolle = False
            while (obstacle == True):
                Robot.arret()
                obstacle = Robot.detection_obstacle()
            temps = count.peek()
        Robot.arret()
        
    except KeyboardInterrupt:
        Robot.BP.reset_all()

def GOmatch_jaune():
    #methode pour la couleur jaune fait avancer, attrape 
    #le premier cylindre polychrome et retourne jusqu'à la base puis
    #fait une rotation et depose le cylindre ensuite revient a sa position initiale,
    #recupere un module dans la fusee puis va jusqu'a la base lunaire pour le deposer
 
    try:
        Robot = Actions()
        #debut du chrono
        arret_durgence = 1
        obstacle1 = Robot.detection_obstacle()
        #Boucle infinie pour le demarrage
        booll = True
        while booll:
            arret_durgence =  Robot.demarrage()
            print("on fait rien, on attend le demarrage")
            if arret_durgence == 0:
                booll = False
        print("Couleur Jaune") 
        count = SecondCounter()
        count.start()
        # pour prendre en compte larret durgence
        temps_initial = count.peek()  
        temps = count.peek() 
        boolle = True
        obstacle1 = Robot.detection_obstacle()
        while boolle and temps <= 76:
            arret_durgence =  Robot.demarrage()
            print(temps)
            temps = count.peek()
            
            #Marche, recupere le premiier cylindre polychrome et le met dans la base
            Robot.ouvrir_main()
            time.sleep(0.5)
            Robot.avancer(70, 0.45)
            time.sleep(0.5)
            Robot.fermer_main()
            time.sleep(0.5)
            Robot.reculer(50,0.45)

            time.sleep(3)
            Robot.rotation(-1.2)
            time.sleep(2)
            Robot.ouvrir_main()
            time.sleep(0.5)
            Robot.reculer(50,0.3)
            time.sleep(2)
            Robot.rotation(-0.5)
            time.sleep(2)
            Robot.avancer(70, 0.1)
            time.sleep(0.5)
            Robot.reculer(50, 0.1)
            time.sleep(2)
            Robot.rotation(0.6)
            time.sleep(2)
            Robot.avancer(70, 0.45)
            time.sleep(0.5)
            Robot.rotation(-0.88)
            Robot.renverser_cylindre()
            time.sleep(0.5)
            Robot.avancer(70, 0.25)
            time.sleep(0.5)
            Robot.ouvrir_main()
            Robot.retour_renverser_cylindre()

            #Rajout
            #time.sleep(1.5)
            #Robot.rotation_gauche(2)
            #time.sleep(0.5)
            #Robot.avancer(50, 0.1)
            #time.sleep(0.5)
            #Robot.reculer(50, 0.1)
            #time.sleep(2)
            #Robot.rotation_gauche(3.6)
            
            Robot.arret()
            if (arret_durgence == 1) or (temps >76):
                boolle = False
            temps = count.peek()
            break
        Robot.BP.reset_all()
        count.finish()
        print("la fonction a pris {} seconds pour terminer".format(temps))
    except KeyboardInterrupt:                                                   
        Robot.BP.reset_all()        # exception pour gerer l'appui sur Ctrl+C auquel le programme s'arrete.
try:
    iniatialise_ports()
    Robot = Actions()
    count1 = SecondCounter()
    count1.start()
    temps= count1.peek()
    
    #Loop pour ne rien faire jusqua ce quon appui une fois nous permet de marcher jusquau tribune
    #En laissant le code tourner sur le robot
    tirette_ = Robot.tirette()
    while(tirette_ == False):
        tirette_ = Robot.tirette()
        print("On fait rien on attend le premier appui sur la tirette")
    
    #choix couleur
    # quand on est jaune on met la tirette, on appui sur le bouton demarrage et on attend 
    # le signal de l'arbitre
    # quand on est bleue, on appui le bouton demarrage, on attend exactement 10s ni moins ni plus
    # puis on met la tirette et on attend le signal de l'arbitre
    choix_compteur = 0
    compteur = 0
    while (temps <=5):
        print("Choix de couleur")
        temps = count1.peek()
        time.sleep(2)
        choix_compteur = Robot.demarrage()
        while(choix_compteur == 1):
            compteur += 1
            break
    count1.finish()
    print(compteur)
    time.sleep(10)
    
    if(compteur < 1):
        GOmatch_bleue()
    else:
       GOmatch_jaune()
    count1.finish()

except KeyboardInterrupt:       # exception pour gerer l'appui sur Ctrl+C auquel le programme s'arrete.
    Robot.BP.reset_all()        # desactive les capteurs, les moteurs, et remet le control de la LED au firmwre de BrickPi3.
