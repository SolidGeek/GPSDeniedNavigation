import cv2
import numpy as np

import time
import math

from opticalflow import OpticalFlow

cam_num = 1 # Id in the list of camera sources
cap = cv2.VideoCapture(cam_num)

width    = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

scaledown = 2 # Scale down the video stream before processing
movestep  = 16 # Perform the opticalflow at each 16 pixel

# Initiate flow object
flow = OpticalFlow(width, height, scaledown=scaledown, move_step=movestep) 

time_step = 0
last_time = 0

x = 0
y = 0

while True:
    success, frame = cap.read()
        
    if not success:
        break
    
    time_step = time.time() - last_time
    last_time = time.time()

    result = flow.processFrame(frame, distance=0.7, timestep=time_step)

    if result:
    
        # Unpack results
        vx, vy, frame = result
        
        # Show results
        cv2.imshow('Camera Flow Sensor', frame)
        
        # Calcualte moved distance
        x = x + vx * time_step
        y = y + vy * time_step

        print('x{:+.3f} y{:+.3f} m/s'.format(vx, vy))
        
        print('x{:+.3f} y{:+.3f} m'.format(x,y))

    # Break the loop
    if cv2.waitKey(1) & 0x000000FF== 27: # ESC
        break
    