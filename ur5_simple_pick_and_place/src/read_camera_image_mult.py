#!/usr/bin/env python
"""
Python node for reading camera data
"""

import sys, time
import numpy as np
import cv2 as cv
import roslib
import rospy
from sensor_msgs.msg import CompressedImage  # ROS messages

print('opencv version: ', cv.__version__)

class image_read:
    def __init__(self):
        # Define the subscriber topic
        self.subscriber = rospy.Subscriber(("/myur5/camera1/image_raw/compressed"), 
                                           CompressedImage, self.callback, queue_size=1)

    def callback(self, ros_data):
        global coordenates, n_b, n_g, n_r
        coordenates = []
        def dibujar(mask, color):
            n = 0
            coord = []
            contours, hiterachy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

            for c in contours:
                area = cv.contourArea(c)
                if area > 100:
                    n = n + 1
                    m = cv.moments(c)
                    if m["m00"] == 0:
                        m["m00"] = 1
                    x = int(m["m10"] / m["m00"])
                    y = int(m["m01"] / m["m00"])
                    cv.circle(frame, (x, y), 3, color, -1)
                    font = cv.FONT_HERSHEY_SIMPLEX
                    cv.putText(frame, "(" + str(x) + ", " + str(y) + ")", (x + 28, y), font, 0.5, color, 1, cv.LINE_AA)
                    convexhull = cv.convexHull(c)
                    cv.drawContours(frame, [convexhull], 0, color, 3)
                    coord.append(x)
                    coord.append(y)
            return n, coord


        """Here images are read and processed"""
        # ROJO
        redBajo1 = np.array([0, 100, 20], np.uint8)
        redAlto1 = np.array([8, 255, 255], np.uint8)
        redBajo2 = np.array([175, 100, 20], np.uint8)
        redAlto2 = np.array([179, 255, 255], np.uint8)

        # AZUL
        blueBajo = np.array([100, 100, 20], np.uint8)
        blueAlto = np.array([125, 255, 255], np.uint8)

        # VERDE
        greenBajo = np.array([45, 100, 20], np.uint8)
        greenAlto = np.array([95, 255, 255], np.uint8)

        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        maskRed1 = cv.inRange(hsv, redBajo1, redAlto1)
        maskRed2 = cv.inRange(hsv, redBajo2, redAlto2)
        maskRed = cv.bitwise_or(maskRed1, maskRed2)
        maskBlue = cv.inRange(hsv, blueBajo, blueAlto)
        maskGreen = cv.inRange(hsv, greenBajo, greenAlto)

        piec_blue = []
        piec_green = []  # Vectores que almacenan la posicion en pixels de cada una de las piezas
        piec_red = []

        n_b, piec_blue = dibujar(maskBlue, (255, 0, 0))
        coordenates.append(piec_blue)
        n_g, piec_green = dibujar(maskGreen, (0, 255, 0))
        coordenates.append(piec_green)
        n_r, piec_red = dibujar(maskRed, (0, 0, 255))
        coordenates.append(piec_red)

        cv.imshow('frame', frame)
        cv.waitKey(2)
    
    def rev_coord(self):
        global coordenates, n_b, n_g, n_r
        return n_b, n_g, n_r, coordenates
        
    
def main(args):
    """Initializes and clearup ros node"""
    ic = image_read()
    rospy.init_node('image_read', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down the ROS Image Reader Node')
        cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)