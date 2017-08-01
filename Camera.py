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