
"""
    On fait du threading ici pour pouvoir compter en arrière plan
"""
from threading import Thread
import time
import sys

# A assurer avec Python2 ou Python3
if sys.version_info[0] < 3:
    input = raw_input

class SecondCounter(Thread):
    '''
    On crée un objet qui comptera les 90 secondes en arrière plan
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
        Quand on instancie un objet 
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

    
