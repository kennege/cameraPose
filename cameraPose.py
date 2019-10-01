#!/usr/bin/env python

import random as rand
from math import sin, cos, pi
import numpy as np

def cameraPose(center, radius):
    # given: 
    #   centre - center of rotation (R3 tuple)
    #   radius - radius of rotation (meters)
    # return:
    #   cameraPose (SE3)

    R = np.array([[0,-1, 0],
        [0, 0, -1],
        [1, 0, 0]])# Rotate from cv convention to robotic

    # calculate camera location 
    phi =rand.random()*2*pi # azimuthal angle
    r = radius + 0.1*radius*rand.uniform(-1,1) # 10% noise in radius
    theta = pi + rand.uniform(-pi/2,pi/2) # polar angle
    x = r*sin(theta)*cos(phi)
    y = r*sin(theta)*sin(phi)
    z = -r*cos(theta)
    camLoc = np.asarray([(x,y,z)])

    # calculate camera rotation 
    target_position = np.asarray([center])
    camZ = -(camLoc - np.matmul(R.T,target_position.T).T) 
    camZ = camZ / np.linalg.norm(camZ) # normalise
    camX = np.cross(np.array([0.0, 1.0, 0.0]), camZ)
    camX = camX / np.linalg.norm(camX) # normalise
    camY = np.cross(camZ, camX)
    camY = camY / np.linalg.norm(camY) # normalise
    
    # construct camera pose matrix
    T = np.eye(4)
    T[0, :3] = camX
    T[1, :3] = camY
    T[2, :3] = camZ  
    T[:3, -1] =  camLoc   

    return T


print(cameraPose((0,0,0),5.0))
