import cv2
import numpy as np

#ouverture de la camera

webcam = VideoCapture(1)

while True:
    ret, frame = webcam.read()
    cv2.imshow('frame', frame)

    if cv2.waitKey(0):
        break
webcam.release()
cv2.destroyAllWindows


    time.sleep(20)
    time.sleep(15)
    Robot.avancer(80, 0.50)
    Robot.ouvrir_main()
    time.sleep(2)
    Robot.fermer_main()
    time.sleep(2)
    Robot.tourner_a_droite()
    time.sleep(2)
    Robot.avancer(80, 0.60)
    time.sleep(2)
    Robot.ouvrir_main()
    time.sleep(2)
    Robot.fermer_main()
    time.sleep(2)
    Robot.reculer(80, 0.60)
    time.sleep(2)
    Robot.tourner_a_gauche()
    time.sleep(2)
    Robot.reculer(80, 0.50)

    time.sleep(20)
    
    Robot.avancer(80, 0.50)
    Robot.ouvrir_main()
    time.sleep(2)
    Robot.fermer_main()
    time.sleep(2)
    Robot.tourner_a_droite()
    time.sleep(2)
    Robot.avancer(80, 0.60)
    time.sleep(2)
    Robot.ouvrir_main()
    time.sleep(2)
    Robot.fermer_main()
    time.sleep(2)
    Robot.reculer(80, 0.60)
    time.sleep(5)
    Robot.tourner_a_gauche()
    time.sleep(2)
    Robot.reculer(80, 0.50)
