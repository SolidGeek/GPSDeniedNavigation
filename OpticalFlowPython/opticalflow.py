'''
optical_flow.py - Optical-flow velocity calculation and display using OpenCV

    Copyright (C) 2014 Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
'''

import cv2
import numpy as np

import time
import math

class OpticalFlow:
    '''
    A class for optical flow calculations using OpenCV
    '''
    
    def __init__(self, frame_width, frame_height, scaledown=1, move_step=16):
        '''
        Creates an OpticalFlow object for images with specified width and height.
        '''
        
        self.move_step = move_step
        self.mv_color_bgr = (0,255,0)
        
        self.size = (int(frame_width/scaledown), int(frame_height/scaledown))

        self.prev_gray = None
        self.prev_time = None

    def processFrame(self, frame, distance, timestep):
        '''
        Processes one image frame, returning summed X,Y flow and frame.
        '''
        frame2 = cv2.resize(frame, self.size)
 
        gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        xsum, ysum = 0,0
        xvel, yvel = 0,0

        flow = None

        if not self.prev_gray is None:

            flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, flow, pyr_scale=0.5, levels=5, winsize=13, iterations=10, poly_n=5, poly_sigma=1.1, flags=0) 

            for y in range(0, flow.shape[0], self.move_step):

                for x in range(0, flow.shape[1], self.move_step):
                    
                    # Vector describing the flow in y,x
                    fx, fy = flow[y, x]
                    xsum += fx
                    ysum += fy

                    cv2.line(frame2, (x,y), (int(x+fx),int(y+fy)), self.mv_color_bgr)
                    cv2.circle(frame2, (x,y), 1, self.mv_color_bgr, -1)

            xvel = self._get_velocity(flow, xsum, flow.shape[1], distance, timestep)
            yvel = self._get_velocity(flow, ysum, flow.shape[0], distance, timestep)

        self.prev_gray = gray
        
        # Return x,y velocities and new image with flow lines
        return  xvel, yvel, frame2

    def _get_velocity(self, flow, sum_velocity_pixels, dimsize_pixels, distance_meters, timestep_seconds):

        # Number of steps in the image 
        count =  (flow.shape[0] * flow.shape[1]) / self.move_step**2

        # Average velocity for all steps
        average_velocity_pixels_per_second = sum_velocity_pixels / count / timestep_seconds

        # print(average_velocity_pixels_per_second)

        return self._velocity_meters_per_second(average_velocity_pixels_per_second, dimsize_pixels, distance_meters)

    def _velocity_meters_per_second(self, velocity_pixels_per_second, dimsize_pixels, distance_meters):

        distance_pixels = (dimsize_pixels/2)         

        pixels_per_meter = distance_pixels / distance_meters
         
        return velocity_pixels_per_second / pixels_per_meter