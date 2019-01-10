"""
Parts of the code in this file are from https://github.com/yrouben/Sampling-Based-Path-Planning-Library. Thanks for the coder yrouben.
"""

from __future__ import division
from shapely.geometry import Polygon
from environment import Environment
from RRTs import RRTPlanner
from matplotlib import pyplot as plt

import vrep
import time
import math
import numpy as np
import sys
import os
from PIL import Image


def hhh():
    environment = Environment('bugtrap.yaml')
    bounds = (-2, -3, 12, 8)
    start_pose = (2, 2.5)
    goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])
    object_radius = 0.3
    steer_distance = 0.3
    num_iterations = 10000
    resolution = 3
    drawResults = True
    runForFullIterations = False


    sbpp = RRTPlanner()
    path= sbpp.RRT(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, drawResults, runForFullIterations)
    print (path)
    plt.show()


if __name__=="__main__":
    print ("Program Started")
    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, -500000, 5)
    if clientID != -1:
        print ("Connected to remote API server")
        emptyBuff = bytearray()
        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        _, bias = vrep.simxGetObjectHandle(clientID, "Bias", vrep.simx_opmode_blocking)
        _, obstacle1 = vrep.simxGetObjectHandle(clientID, "obsWall1", vrep.simx_opmode_blocking)
        _, obstacle2 = vrep.simxGetObjectHandle(clientID, "obsWall2", vrep.simx_opmode_blocking)
        _, startNode = vrep.simxGetObjectHandle(clientID, "DummyStart", vrep.simx_opmode_blocking)
        _, goalNode = vrep.simxGetObjectHandle(clientID, "DummyGoal", vrep.simx_opmode_blocking)
        _, origin = vrep.simxGetObjectHandle(clientID, "Origin", vrep.simx_opmode_blocking)
        #vrep.simxGetObjectPosition(clientID, obstacle1, bias, vrep.simx_opmode_oneshot_wait)"""

        print ("Finding Path...")
        environment = Environment('bugtrap.yaml')
        bounds = (-2, -2, 2, 2)
        start_pose = (-0.8, -0.275)
        goal_region = Polygon([(0.015, 0.825), (0.015,0.875), (0.035, 0.875), (0.035, 0.825)])
        object_radius = 0.005
        steer_distance = 0.015
        num_iterations = 10000
        resolution = 3
        drawResults = True
        runForFullIterations= False
        sbpp = RRTPlanner()
        path= sbpp.RRT(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, drawResults, runForFullIterations)
        #print (path)
        path = path[0]
        plt.show()
        print ("Path Found!")
        print ("Start Simulating...")
        _, dummyPosition = vrep.simxGetObjectPosition(clientID, objectHandle=startNode, relativeToObjectHandle=origin, operationMode=vrep.simx_opmode_blocking)
        print(dummyPosition)
        for pos in path:
            x, y = pos
            move = (x, y, 0)
            _ = vrep.simxSetObjectPosition(clientID, startNode, origin, move, vrep.simx_opmode_oneshot)
            time.sleep(0.1)

    else:
        print ("Failed connecting to the API server")
    print ("Program Ended")
