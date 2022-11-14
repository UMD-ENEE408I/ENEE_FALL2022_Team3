import socket
import time
import struct
import cam_loc
import cv2
import numpy as np
import argparse

meas_x = 10
meas_y = 10
heading = 0

#setup

#update x,y, heading
vid = cv2.VideoCapture(0) #cam feed into jetson, need to adjust for multiple streams

while meas_x == 10 or meas_y == 10:
#image section
    
    ret, frame = vid.read()
    coords = cam_loc.read_tag(frame)
    if coords:
        meas_x = coords[0]
        meas_y = coords[1]
    
    
# After the loop release the cap object
vid.release()
print("x = ")
print(meas_x)
print("y = ")
print(meas_y)
print("\n")
print("orientation complete \n")

vid = cv2.VideoCapture(0)
#loop for dance routine
while(True):

    #image section
    
    ret, frame = vid.read()
    coords = cam_loc.read_tag(frame)
    print("Coords ")
    print(coords)
    time.sleep(1)
# After the loop release the cap object
vid.release()
