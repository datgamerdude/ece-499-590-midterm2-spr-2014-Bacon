#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import math

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

curr_err = 0
prev_err = 0
err_sum = 0

err_arr = [0,0,0,0,0]

dist = 0
speed = 0
x = 0
y = 0

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    imgL = np.zeros((newx,newy,3), np.uint8)
    imgR = np.zeros((newx,newy,3), np.uint8)
    c_image = imgL.copy()
    c_image = imgR.copy()
    vidL = cv2.resize(c_image,(newx,newy))
    vidR = cv2.resize(c_image,(newx,newy))
    [status, framesize] = vl.get(vidL, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidL,(nx,ny))
        imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_L", imgL)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )
    [status, framesize] = vr.get(vidR, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidR,(nx,ny))
        imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_R", imgR)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # imgL       = cv image in BGR format (Left Camera)
    # imgR       = cv image in BGR format (Right Camera)
    
        # Get start time of the sim
    otime2=tim.sim[0]

# Create a frame for finding green colors
    green = cv2.inRange(vid2, np.array([0,0,0], dtype = np.uint8), np.array([0,255,0], dtype = np.uint8));

# Finds the blue box
    cntRGB, h = cv2.findContours(green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    area = cv2.countNonZero(green)
    
    radius =math.sqrt(area/math.pi)
    diameter = 2*radius
    
    per_circle = diameter/320
    circle_fov = 60 * per_circle

    angle = circle_fov/2

# 1m = diameter so diameter pixel width = 1m

    if (diameter == 0):
    	dist = 0
    else:
	dist = (radius/math.tan(angle*math.pi/180)) - radius
	dist = dist/diameter   

#    print dist

# Finds the center of the box and calculates its center
    for cnt in cntRGB:
	(x, y), radius = cv2.minEnclosingCircle(cnt)
	center = (int(x), int(y))
	radius = int(radius)
	check=1
#	print 'x, y ', x, y

# Get the error of the robot center from the box center
    err = 0
    err_per = 0
    err_sum = 0
    u_in = 0
    u_in_per = 0

    err = (nx/2) - x
#    print 'Error (pixels) = ',err
    err_per = abs(err/newx);
#    print 'Error (%) = ',err_per*100,' %'

# Integral Control
    for num in range(0,3):
	err_arr[num] = err_arr[num+1]
	err_sum = err_sum + err_arr[num]

    err_arr[4] = err
    err_sum = err_sum + err_arr[4]

# Porportional and Derivative Control with summation
    curr_err = err
    u_in = 2*err + 2*(curr_err-prev_err) + 0.2*err_sum
    prev_err = curr_err

# Input used to set motor speed
    u_in_per = abs(u_in/newx);
#    print'Motor Vel. input(%) = ',u_in_per*100,'%'

    print'Distance = ',dist, '| Speed = ',speed
    
# Motor speed inputs

#    if(dist < 3.5) and (speed > 0):
#	speed = speed - 0.05
#    elif(dist < 3.75) and (speed > 0):
#	speed = speed - 0.01
#    elif(dist > 3.75) and (dist < 3.95) and (speed > 0):
#	speed = speed - 0.05
#    elif(dist > 4.05) and (dist < 4.25):
#	speed = speed + 0.01
#    elif(dist < 4.5):
#	speed = speed + 0.05
#    elif(dist > 4.5):
#	speed = speed + 0.1

    if(dist < 3.75) and (speed > 0):
	speed = speed - 0.025
    elif(dist < 4.5):
	speed = speed + 0.05
    elif(dist > 4.5):
	speed = speed + 0.1

    if(u_in > 0.5):
	ref.ref[0] = speed+speed*u_in_per
	ref.ref[1] = speed-speed*u_in_per
    elif(u_in < -0.5):
	ref.ref[0] = speed-speed*u_in_per
	ref.ref[1] = speed+speed*u_in_per
    
#    if(u_in > 0.5):
#	ref.ref[0] = 2+1*u_in_per
#	ref.ref[1] = 2-1*u_in_per
#    elif(u_in < -0.5):
#	ref.ref[0] = 2-1*u_in_per
#	ref.ref[1] = 2+1*u_in_per

    r.put(ref)

# Have the simulation wait
    while((tim.sim[0]-otime2)<.1):
	[status, framesize] = t.get(tim, wait=False, last=True)

    # Sleeps
    time.sleep(0.1)   
#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
