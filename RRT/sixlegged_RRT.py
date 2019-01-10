from __future__ import division
from shapely.geometry import Polygon
from environment import Environment
from RRTs import RRTPlanner

import vrep
import sys
import os
from PIL import Image

import math
import numpy as np
import time
from intervals import Interval
import array
import matplotlib.pyplot as plt
import cv2

N = 75
A = 0.10 #length
B = 0.05 #height
theta = np.arange(0, math.pi+0.001, math.pi/N)
theta_cos = np.cos(theta)
theta_sin = np.sin(theta)
theta_sin[-1] = 0

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

def GetPee():
    Pee = []
    for i in range(0,6):
        res, S1_pos = vrep.simxGetObjectPosition(clientID, S1[i], BCS,
                                                                vrep.simx_opmode_oneshot_wait)
        Pee.append(S1_pos[0])
        Pee.append(S1_pos[1])
        Pee.append(S1_pos[2])
    return Pee

def SetPee(Pee):
    Coor = np.array(Pee)
    Coordinate = np.reshape(Coor, (6, -1))
    for i in range(Coordinate.shape[0]):
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSetObjectPosition(clientID, Tip_target[i], BCS, [Coordinate[i][0], Coordinate[i][1], Coordinate[i][2]],
                                   vrep.simx_opmode_oneshot_wait)

def recover(n=N):
    Lz = np.zeros(n+1)
    init_position = np.zeros((6, 3))
    time.sleep(3)
    for i in range(6):
        res, init_position[i] = vrep.simxGetObjectPosition(clientID, S1[i], BCS, vrep.simx_opmode_oneshot_wait)
    for i in range(1,n+1):
        Lz[i] = init_position[0][2] - i * 0.18/n

    for i in range(1,n+1):
        for j in range(0,6,2):
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxSetObjectPosition(clientID, Tip_target[j], BCS, [init_position[j][0], init_position[j][1], Lz[i]],
                               vrep.simx_opmode_oneshot_wait)
    for i in range(1,n+1):
        for j in range(1,6,2):
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxSetObjectPosition(clientID, Tip_target[j], BCS, [init_position[j][0], init_position[j][1], Lz[i]],
                               vrep.simx_opmode_oneshot_wait)

def SetTheFoot(target_point, foot, target_position, n = N):
    res, init_position = vrep.simxGetObjectPosition(clientID, foot, BCS, vrep.simx_opmode_oneshot_wait)
    Lx = [0 for x in range(0, n)]
    Ly = [0 for x in range(0, n)]
    Lz = [0 for x in range(0, n)]
    #Define the trajectory
    for i in range(n):
        Lx[i] = init_position[0] + i * (target_position[0] - init_position[0]) / n
        Ly[i] = init_position[1] + i * (target_position[1] - init_position[1]) / n
        Lz[i] = init_position[2] + i * (target_position[2] - init_position[2]) / n
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSetObjectPosition(clientID, target_point, BCS, [Lx[i], Ly[i], Lz[i]], vrep.simx_opmode_oneshot_wait)

def three_first_step(Lx, Ly, Lz, n = N):
    for i in range(1,n+1):
        vrep.simxSynchronousTrigger(clientID)
        for j in range(0, 6, 2):
            vrep.simxSetObjectPosition(clientID, Tip_target[j], BCS, [Lx[j][i], Ly[j][i], Lz[j][i]],vrep.simx_opmode_oneshot_wait)

def three_end_step(Lx, Ly, Lz, n = N):
    for i in range(1,n+1):
        vrep.simxSynchronousTrigger(clientID)
        for j in range(1, 6, 2):
            vrep.simxSetObjectPosition(clientID, Tip_target[j], BCS, [Lx[j][i], Ly[j][i], Lz[j][i]],vrep.simx_opmode_oneshot_wait)

def three_one_loop(Lx, Ly, Lz, n = N):
    for i in range(1,n+1):
        vrep.simxSynchronousTrigger(clientID)
        for j in range(6):
            vrep.simxSetObjectPosition(clientID, Tip_target[j], BCS, [Lx[j][i], Ly[j][i], Lz[j][i]],vrep.simx_opmode_oneshot_wait)

def three_step_loops(Lx, Ly, Lz, init_position, n = N, a = A, b = B):
    mid_position = np.zeros((6, 3))
    for i in range(6):
        mid_position[i] = init_position[i]
    for j in range(0, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] =  mid_position[j][0] + i * (INIT_POSITION[j][0] -  mid_position[j][0]) / n
            Ly[j][i] =  mid_position[j][1] + i * (INIT_POSITION[j][1] -  mid_position[j][1]) / n
            Lz[j][i] =  mid_position[j][2] + i * (INIT_POSITION[j][2] -  mid_position[j][2]) / n
    for j in range(1, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] = mid_position[j][0] + a + a * theta_cos[N-i]
            Ly[j][i] = mid_position[j][1]
            Lz[j][i] = mid_position[j][2] + b * theta_sin[N-i]
    three_one_loop(Lx, Ly, Lz)
    for i in range(6):
        init_position[i][0] = Lx[i][-1]
        init_position[i][1] = Ly[i][-1]
        init_position[i][2] = Lz[i][-1]
    # time.sleep(3)
    # for i in range(6):
    #     res, mid_position[i] = vrep.simxGetObjectPosition(clientID, S1[i], BCS, vrep.simx_opmode_oneshot_wait)
    for i in range(6):
        mid_position[i] = init_position[i]
    for j in range(0, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] =  mid_position[j][0] + a + a * theta_cos[N-i]
            Ly[j][i] =  mid_position[j][1]
            Lz[j][i] =  mid_position[j][2] + b * theta_sin[N-i]
    for j in range(1, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] = mid_position[j][0] + i * (INIT_POSITION[j][0] -  mid_position[j][0]) / n
            Ly[j][i] = mid_position[j][1] + i * (INIT_POSITION[j][1] -  mid_position[j][1]) / n
            Lz[j][i] = mid_position[j][2] + i * (INIT_POSITION[j][2] -  mid_position[j][2]) / n
    three_one_loop(Lx, Ly, Lz)
    for i in range(6):
        init_position[i][0] = Lx[i][-1]
        init_position[i][1] = Ly[i][-1]
        init_position[i][2] = Lz[i][-1]
    return init_position

def three_three_gait(INIT_POSITION, step = 1, a = A, b = B, n = N ):
    init_position = np.zeros((6, 3))
    Lx = np.zeros((6, n+1))
    Ly = np.zeros((6, n+1))
    Lz = np.zeros((6, n+1))
    for i in range(6):
        # res, init_position[i] = vrep.simxGetObjectPosition(clientID, S1[i], BCS, vrep.simx_opmode_oneshot_wait)
        init_position[i] = INIT_POSITION[i]
        print(init_position[i])
    for j in range(0, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] = init_position[j][0] + a + a * theta_cos[N-i]
            Ly[j][i] = init_position[j][1]
            Lz[j][i] = init_position[j][2] + b * theta_sin[N-i]
    for i in range(0, 6, 2):
        init_position[i][0] = Lx[i][-1]
        init_position[i][1] = Ly[i][-1]
        init_position[i][2] = Lz[i][-1]
    three_first_step(Lx, Ly, Lz)
    for m in range(step-1):
        init_position = three_step_loops(Lx, Ly, Lz, init_position = init_position)
    for j in range(1, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] = init_position[j][0] + a + a * theta_cos[N-i]
            Ly[j][i] = init_position[j][1]
            Lz[j][i] = init_position[j][2] + b * theta_sin[N-i]
    three_end_step(Lx, Ly, Lz)
    for i in range(1, 6, 2):
        init_position[i][0] = Lx[i][-1]
        init_position[i][1] = Ly[i][-1]
        init_position[i][2] = Lz[i][-1]
    for j in range(6):
        for i in range(1,n+1):
            Lx[j][i] = init_position[j][0] + i * (INIT_POSITION[j][0] -  init_position[j][0]) / n
            Ly[j][i] = init_position[j][1] + i * (INIT_POSITION[j][1] -  init_position[j][1]) / n
            Lz[j][i] = init_position[j][2] + i * (INIT_POSITION[j][2] -  init_position[j][2]) / n
    three_one_loop(Lx, Ly, Lz)
    for i in range(6):
        init_position[i][0] = Lx[i][-1]
        init_position[i][1] = Ly[i][-1]
        init_position[i][2] = Lz[i][-1]
    return init_position

def turn_around(aim_theta, INIT_position , b = B, n=N):
    init_position = INIT_position.copy()
    flag = True
    if aim_theta<0:
        aim_theta *= (-1)
        flag = False

    aim_theta_list = np.arange(0, aim_theta+0.001, aim_theta/n)
    aim_theta_cos = np.cos(aim_theta_list)
    aim_theta_sin = np.sin(aim_theta_list)
    if flag==False:
        aim_theta_sin *= (-1)
    Lx = np.zeros((6, n+1))
    Ly = np.zeros((6, n+1))
    Lz = np.zeros((6, n+1))
    for j in range(0, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] = init_position[j][0] * aim_theta_cos[i] - init_position[j][1] * aim_theta_sin[i]
            Ly[j][i] = init_position[j][0] * aim_theta_sin[i] + init_position[j][1] * aim_theta_cos[i]
            Lz[j][i] = init_position[j][2] + b * theta_sin[N-i]
    for j in range(0, 6, 2):
        init_position[j][0] = Lx[j][-1]
        init_position[j][1] = Ly[j][-1]
        init_position[j][2] = Lz[j][-1]
    for i in range(1,n+1):
        vrep.simxSynchronousTrigger(clientID)
        for j in range(0, 6, 2):
            vrep.simxSetObjectPosition(clientID, Tip_target[j], BCS, [Lx[j][i], Ly[j][i], Lz[j][i]],vrep.simx_opmode_oneshot_wait)
    for j in range(1, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] = init_position[j][0]
            Ly[j][i] = init_position[j][1]
            Lz[j][i] = init_position[j][2] + b * theta_sin[N-i]
    for j in range(0, 6, 2):
        for i in range(1,n+1):
            Lx[j][i] = init_position[j][0] * aim_theta_cos[i] + init_position[j][1] * aim_theta_sin[i]
            Ly[j][i] = init_position[j][1] * aim_theta_cos[i] - init_position[j][0] * aim_theta_sin[i]
            Lz[j][i] = init_position[j][2]
    for i in range(1,n+1):
        vrep.simxSynchronousTrigger(clientID)
        for j in range(6):
            vrep.simxSetObjectPosition(clientID, Tip_target[j], BCS, [Lx[j][i], Ly[j][i], Lz[j][i]],vrep.simx_opmode_oneshot_wait)
    for i in range(6):
        init_position[i][0] = Lx[i][-1]
        init_position[i][1] = Ly[i][-1]
        init_position[i][2] = Lz[i][-1]
    return init_position

def two_legs_recovery(n=N):
    print("Start recovering legs!")
    init_position = np.zeros((2, 3))
    Lx = np.zeros((2, n+1))
    Ly = np.zeros((2, n+1))
    Lz = np.zeros((2, n+1))
    res, init_position[0] = vrep.simxGetObjectPosition(clientID, S1[1], BCS, vrep.simx_opmode_oneshot_wait)
    res, init_position[1] = vrep.simxGetObjectPosition(clientID, S1[4], BCS, vrep.simx_opmode_oneshot_wait)

    for i in range(n+1):
            Lx[0][i] = init_position[0][0] + i * (INIT_POSITION[1][0] - init_position[0][0]) / n
            Ly[0][i] = init_position[0][1] + i * (INIT_POSITION[1][1] - 0.2 - init_position[0][1]) / n
            Lz[0][i] = init_position[0][2] + i * (INIT_POSITION[1][2] - init_position[0][2]) / n
    for i in range(n+1):
            Lx[1][i] = init_position[1][0] + i * (INIT_POSITION[4][0] - init_position[1][0]) / n
            Ly[1][i] = init_position[1][1] + i * (INIT_POSITION[4][1] + 0.2 - init_position[1][1]) / n
            Lz[1][i] = init_position[1][2] + i * (INIT_POSITION[4][2] - init_position[1][2]) / n

    for i in range(n):
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSetObjectPosition(clientID, Tip_target[1], BCS, [Lx[0][i], Ly[0][i], Lz[0][i]],vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(clientID, Tip_target[4], BCS, [Lx[1][i], Ly[1][i], Lz[1][i]],vrep.simx_opmode_oneshot_wait)

def two_one_step(Lx, Ly, Lz, j, n=N):
    k=0
    if j==0: k=0
    elif j==1: k=2
    elif j==2: k=3
    else: k=5

    for i in range(n):
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSetObjectPosition(clientID, Tip_target[k], BCS, [Lx[j][i], Ly[j][i], Lz[j][i]],vrep.simx_opmode_oneshot_wait)


def four_legs(a = A, b = B, n = N):
    init_position = np.zeros((4, 3))
    mid_position = np.zeros((4, 3))
    Lx = np.zeros((4, n))
    Ly = np.zeros((4, n))
    Lz = np.zeros((4, n))

    res, init_position[0] = vrep.simxGetObjectPosition(clientID, S1[0], BCS, vrep.simx_opmode_oneshot_wait)
    res, init_position[1] = vrep.simxGetObjectPosition(clientID, S1[2], BCS, vrep.simx_opmode_oneshot_wait)
    res, init_position[2] = vrep.simxGetObjectPosition(clientID, S1[3], BCS, vrep.simx_opmode_oneshot_wait)
    res, init_position[3] = vrep.simxGetObjectPosition(clientID, S1[5], BCS, vrep.simx_opmode_oneshot_wait)

    for j in range(4):
        for i in range(n):
            Lx[j][i] = init_position[j][0] + a + a * math.cos(theta[N - 1 - i])
            Ly[j][i] = init_position[j][1]
            Lz[j][i] = init_position[j][2] + b * math.sin(theta[N - 1 - i])
    two_one_step(Lx, Ly, Lz, 0)
    two_one_step(Lx, Ly, Lz, 3)
    two_one_step(Lx, Ly, Lz, 1)

    res, mid_position[0] = vrep.simxGetObjectPosition(clientID, S1[0], BCS, vrep.simx_opmode_oneshot_wait)
    res, mid_position[1] = vrep.simxGetObjectPosition(clientID, S1[2], BCS, vrep.simx_opmode_oneshot_wait)
    res, mid_position[2] = vrep.simxGetObjectPosition(clientID, S1[3], BCS, vrep.simx_opmode_oneshot_wait)
    res, mid_position[3] = vrep.simxGetObjectPosition(clientID, S1[5], BCS, vrep.simx_opmode_oneshot_wait)
    for j in range(3):
        for i in range(n):
            Lx[j][i] = mid_position[j][0] + i * (TWO_INIT_POSITION[j][0] -  mid_position[j][0]) / n
            Ly[j][i] = mid_position[j][1] + i * (TWO_INIT_POSITION[j][1] -  mid_position[j][1]) / n
            Lz[j][i] = mid_position[j][2] + i * (TWO_INIT_POSITION[j][2] -  mid_position[j][2]) / n
    for i in range(n):
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSetObjectPosition(clientID, Tip_target[0], BCS, [Lx[0][i], Ly[0][i], Lz[0][i]],vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(clientID, Tip_target[2], BCS, [Lx[1][i], Ly[1][i], Lz[1][i]],vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(clientID, Tip_target[3], BCS, [Lx[2][i], Ly[2][i], Lz[2][i]],vrep.simx_opmode_oneshot_wait)


def write_depth_data(depth_buffer):
    depthArray = np.array(depth_buffer)
    depthArray_reshaped = np.reshape(depthArray, (480, -1))
    with open("depth_data_test","w") as f1:
        for i in range(depthArray_reshaped.shape[0]):
            for j in range(depthArray_reshaped.shape[1]):
                f1.write(str(depthArray_reshaped[i][j])+"\t")
            f1.write("\n")

def show_depth_img(depth_buffer):
    depthArray = np.array(depth_buffer)
    depthArray_reshaped = np.reshape(depthArray, (480, -1))
    depth_im = Image.fromarray(depthArray_reshaped * 255)
    depth_im_ini = depth_im.transpose(Image.FLIP_TOP_BOTTOM)
    depth_im_ini.show()

# Get the depth information from the sensor
def get_depth_info(depth_buffer):
    depthArray = np.array(depth_buffer)
    depthArray_reshaped = np.reshape(depthArray, (480, -1))
    depth_im = Image.fromarray(depthArray_reshaped * 255)
    depth_im_ini = depth_im.transpose(Image.FLIP_TOP_BOTTOM)
    img = np.asarray(depth_im_ini, dtype=np.uint8)
    t, binary = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU, cv2.THRESH_BINARY)

    # image = cv2.Canny(binary, 20, 90)
    # print(binary)
    # print(binary[80, 360])
    # print(binary[0, 360])
    # print(binary[400, 360])

    """
    1 for Left
    0 for Right
    -1 for Ahead
    2 for Trying
    """
    if binary[80, 120] == 0:
        if binary[80, 0] == 0:
            return 0 # Turn Right
        else:
            if binary[80, 639] == 0:
                return 1 # Turn Left
    elif binary[80, 520] == 0:
        if binary[80, 639] == 0:
            return 1
        else:
            if binary[80, 0] == 0:
                return 0
    else:
        return -1

    cv2.imshow("depthArray", binary)
    cv2.waitKey(0)

def write_rgb_data(rgb_buffer):
    rgbArray = np.array(rgb_buffer)
    rgbArray_reshaped = np.reshape(rgbArray,(480, 640, -1))
    with open("rgb_data_test","w") as f2:
        for i in range(rgbArray_reshaped.shape[0]):
            for j in range(rgbArray_reshaped.shape[1]):
                f2.write(str(rgbArray_reshaped[i,j]) + "\t")
            f2.write("\n")

def show_rgb_img(rgb_buffer):
    rgbArray = np.array(rgb_buffer)
    rgbArray_reshaped = np.reshape(rgbArray, (480, 640, -1))
    rgb_im = Image.fromarray(np.uint8(rgbArray_reshaped))
    rgb_im_ini = rgb_im.transpose(Image.FLIP_TOP_BOTTOM)
    rgb_im_ini.show()

COUNT = 0
def generate_XYZ(depthBuffer, resolution, u, v, Kin_in_GCS):
    n_p, f_p = 0.01, 5.0
    xAngle_half = 53.13 * math.pi / 360
    yAngle_half = math.atan(math.tan(xAngle_half) * resolution[1] / resolution[0])
    z = n_p + (f_p - n_p) * depthBuffer[(resolution[1] - 1 - v) * resolution[0] + u]
    x = z * math.tan(xAngle_half) * (resolution[0] - 2 * u) / resolution[0]
    y = z * math.tan(yAngle_half) * (resolution[1] - 2 * v) / resolution[1]
    # transform
    if abs(z-5.0)>=0.1:
        Bx = Kin_in_GCS[0] + z
        By = Kin_in_GCS[1] + x
        Bz = Kin_in_GCS[2] + y
    else:
        Bx, By, Bz = 0, 0, 0
    Bx = ("%.3f" % Bx)
    By = ("%.3f" % By)
    Bz = ("%.3f" % Bz)
    x = ("%.3f" % x)
    y = ("%.3f" % y)
    z = ("%.3f" % z)
    return Bx, By, Bz


def averagenum(num):
    nsum = 0
    for i in range(len(num)):
        nsum += float(num[i])
    if len(num)!= 0:
        ave = nsum / len(num)
    else: ave = 0
    return ave

def write_XYZ(flag):
    n = 0
    with open("XYZ_DATA", "w") as f3:
        for i in range(48):
            for j in range(64):
                f3.write(str(flag[n]) + "   \t")
                n += 1
            f3.write("\n")

def write_grid_map(grid_map):
    n = 0
    with open("grid_map", "w") as f4:
        for i in range(25):
            for j in range(25):
                f4.write(str(grid_map[25-1-i][j]) + "\t     ")
                n += 1
            f4.write("\n")


def generate_grid_map(flag, Kin_in_GCS):
    grid_map=np.zeros((25, 25))
    #grid map origin
    x_0 = -Kin_in_GCS[0]
    y_0 = -2.5 - Kin_in_GCS[1]

    for i in range(25):
        for j in range(25):
            sum_height = []
            for k in range(48*64):
                if ((x_0 + i*0.2)<=float(flag[k][0])<(x_0 + (i+1)*0.2))  and ((y_0 + j*0.2)<=float(flag[k][1])<(y_0 + (j+1)*0.2)):
                    sum_height.append(flag[k][2])
            grid_map[i][j] = averagenum(sum_height)
            grid_map[i][j] = ("%.3f" % grid_map[i][j])
            if grid_map[i][j]==0:
                grid_map[i][j] += 10

    return grid_map



def setGBS():
    res, BCS_position = vrep.simxGetObjectPosition(clientID, BCS, -1,
                                                          vrep.simx_opmode_oneshot_wait)
    [x, y, z] = [BCS_position[0], BCS_position[1], BCS_position[2]]
    vrep.simxSetObjectPosition(clientID, GCS, -1, [x, y, 0], vrep.simx_opmode_oneshot_wait)

def goTo(x, y, clientID, obj, origin, INIT_POSITION):
    """
    Go to a place (x, y) relative to the absolute coordinate
    Input: (x, y) - a point in the absolute coordinate
           obj - the target robotics
           origin - the origin of the whole map of absolute coordinate
    Output: The robot moves to (x, y)
    """
    _, objPos = vrep.simxGetObjectPosition(clientID, obj, origin, vrep.simx_opmode_blocking)
    tangent = (y - objPos[1]) / (x - objPos[0])
    theta = math.atan(tangent)

    times = int(abs(theta) / 0.2)
    for i in range(times):
        print("Turnning to Goal")
        if theta < 0:
            turn_around(0.2, INIT_POSITION)
        else:
            turn_around(-0.2, INIT_POSITION)
        time.sleep(1)

    print("Moving")
    three_three_gait(INIT_POSITION = INIT_POSITION, step = 1)
    time.sleep(1)
    three_three_gait(INIT_POSITION = INIT_POSITION, step = 1)

    for i in range(times):
        if theta < 0:
            turn_around(-0.2, INIT_POSITION)
        else:
            turn_around(0.2, INIT_POSITION)
        time.sleep(1)

print('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5) # Connect to V-REP, set a very large time-out for blocking commands
if clientID!=-1:
    print ('Connected to remote API server')
    res = vrep.simxSynchronous(clientID, True)

    emptyBuff = bytearray()

    # Start the simulation:
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    # ???
    res, U4 = vrep.simxGetObjectHandle(clientID, 'J11R', vrep.simx_opmode_blocking)
    res, U5 = vrep.simxGetObjectHandle(clientID, 'shaft19', vrep.simx_opmode_blocking)
    res, robotHandle = vrep.simxGetObjectHandle(clientID, 'bodyDyn', vrep.simx_opmode_blocking)
    res, endHandle = vrep.simxGetObjectHandle(clientID, 'Destination', vrep.simx_opmode_blocking)

    res, robotPosition = vrep.simxGetObjectPosition(clientID=clientID, objectHandle=robotHandle, relativeToObjectHandle=endHandle, operationMode=vrep.simx_opmode_oneshot_wait)


    # camera
    res, kinect_depth_camera = vrep.simxGetObjectHandle(clientID, 'kinect_depth', vrep.simx_opmode_blocking)
    res, kinect_rgb_camera = vrep.simxGetObjectHandle(clientID, 'kinect_rgb', vrep.simx_opmode_blocking)
    res, kinect_joint = vrep.simxGetObjectHandle(clientID, 'kinect_joint', vrep.simx_opmode_blocking)

    #Retrive the body coordinate system
    # coordinate ????
    res, GCS = vrep.simxGetObjectHandle(clientID, 'GCS', vrep.simx_opmode_blocking)
    res, BCS = vrep.simxGetObjectHandle(clientID, 'BCS', vrep.simx_opmode_blocking)
    # Retrive the poles
    # 18 legs
    pole = np.zeros(19, dtype='int32')
    for i in range(1, 19):
        res, pole[i] = vrep.simxGetObjectHandle(clientID, 'P' + str(i), vrep.simx_opmode_blocking)
    #Retrive the U1
    U1 = np.zeros(6, dtype='int32')
    for i in range(0, 6):
        res, U1[i] = vrep.simxGetObjectHandle(clientID, 'Hip' + str(i + 1), vrep.simx_opmode_blocking)
    # Retrive the U2
    # first layer
    U2 = np.zeros(6, dtype='int32')
    res, U2[0] = vrep.simxGetObjectHandle(clientID, 'J21R', vrep.simx_opmode_blocking)
    for i in range(1, 6):
        res, U2[i] = vrep.simxGetObjectHandle(clientID, 'J21R' + str(i - 1), vrep.simx_opmode_blocking)
    # Retrive the U3
    # first layer
    U3 = np.zeros(6, dtype='int32')
    res, U3[0] = vrep.simxGetObjectHandle(clientID, 'J31R', vrep.simx_opmode_blocking)
    for i in range(1, 6):
        res, U3[i] = vrep.simxGetObjectHandle(clientID, 'J31R' + str(i - 1), vrep.simx_opmode_blocking)
    # Retrive the target Dummy
    target_Dummy = np.zeros(6, dtype='int32')
    for i in range(0, 6):
        res, target_Dummy[i] = vrep.simxGetObjectHandle(clientID, 'target_Dummy' + str(i + 1),
                                                        vrep.simx_opmode_blocking)

    # Retrive the S1
    # Tip
    S1 = np.zeros(6, dtype='int32')
    for i in range(0, 6):
        res, S1[i] = vrep.simxGetObjectHandle(clientID, 'Tip' + str(i + 1), vrep.simx_opmode_blocking)
    Tip_target = np.zeros(6, dtype='int32')
    for i in range(0, 6):
        res, Tip_target[i] = vrep.simxGetObjectHandle(clientID, 'TipTarget' + str(i + 1), vrep.simx_opmode_blocking)


    INIT_POSITION = np.zeros((6,3))
    TWO_INIT_POSITION = np.zeros((4,3))
    # setGBS()

    res, kinect_rgb_resoluton, kinect_rgb_buffer = vrep.simxGetVisionSensorImage(clientID, kinect_rgb_camera, 0,
                                                                                 vrep.simx_opmode_oneshot_wait)
    # write_rgb_data(kinect_rgb_buffer)
    show_rgb_img(kinect_rgb_buffer)


    res, resolution, kinect_depth_buffer = vrep.simxGetVisionSensorDepthBuffer(clientID,
                                                                                            kinect_depth_camera,
                                                                                            vrep.simx_opmode_oneshot_wait)
    print(kinect_depth_buffer)
    print(len(kinect_depth_buffer))
    print('begin')
    # write_depth_data(kinect_depth_buffer) #write the depth data in the text file
    show_depth_img(kinect_depth_buffer)
    print('end')


    recover(n=30)


    for i in range(6):
        res, INIT_POSITION[i] = vrep.simxGetObjectPosition(clientID, S1[i], BCS, vrep.simx_opmode_oneshot_wait)
    TWO_INIT_POSITION[0] = INIT_POSITION[0]
    TWO_INIT_POSITION[1] = INIT_POSITION[2]
    TWO_INIT_POSITION[2] = INIT_POSITION[3]
    TWO_INIT_POSITION[3] = INIT_POSITION[5]
    #two_legs_recovery(n=30)
    #four_legs()
    # time.sleep(10)

    ini = INIT_POSITION
    print('start turn_around')
    # turn_around(0.2, INIT_POSITION)
    print('over')


    # three_three_gait(INIT_POSITION = INIT_POSITION, step = 1)

    # res, retInts, retVertices, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'RobotServer',
    #                                                                           vrep.sim_scripttype_childscript,
    #                                                                           'GetMesh', [], [], [], emptyBuff,
    #                                                                           vrep.simx_opmode_oneshot_wait)
    # print(retVertices)

    #Get Matrix Kinect to Body system
    res, retInts, Mat_K_to_B, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'RobotServer',
                                                                              vrep.sim_scripttype_childscript,
                                                                              'GetMatrix', [], [], [], emptyBuff,
                                                                              vrep.simx_opmode_oneshot_wait)
    '''print("MAT Kin to Body:",Mat_K_to_B)
    Mat1 = np.array(Mat_K_to_B)
    print(Mat1)
    Mat1 = np.reshape(Mat1, (3, 4))
    Mat2 = [[0, 0, 0, 1]]
    Matrix_KB = np.row_stack((Mat1, Mat2))
    print(Matrix_KB)'''

    #Get Kinect Info
    res, resolution, kinect_depth_buffer = vrep.simxGetVisionSensorDepthBuffer(clientID,
                                                                                            kinect_depth_camera,
                                                                                            vrep.simx_opmode_oneshot_wait)

    #print(resolution)
    flag = []
    res, Kin_in_GCS = vrep.simxGetObjectPosition(clientID, kinect_depth_camera, GCS, vrep.simx_opmode_oneshot_wait)
    print(Kin_in_GCS)
    for v in range(resolution[1]):
        for u in range(resolution[0]):
            x, y, z = generate_XYZ(kinect_depth_buffer, resolution, u, v, Kin_in_GCS)
            if u%10==0 and v%10==0:
                flag.append([x, y, z])
            if u==320 and v==240:
                print('x,y,z:',x,y,z)

    grid_map = generate_grid_map(flag, Kin_in_GCS)
    # write_grid_map(grid_map)
    # write_XYZ(flag)
    res, kinect_rgb_resoluton, kinect_rgb_buffer = vrep.simxGetVisionSensorImage(clientID, kinect_rgb_camera, 0,
                                                                                 vrep.simx_opmode_oneshot_wait)

    _, origin = vrep.simxGetObjectHandle(clientID, "Origin", vrep.simx_opmode_blocking)
    #vrep.simxGetObjectPosition(clientID, obstacle1, bias, vrep.simx_opmode_oneshot_wait)"""

    print ("Finding Path...")
    environment = Environment('bugtrap.yaml')
    bounds = (-9, -9, 9, 9)
    start_pose = (6.7, 1.34)
    goal_region = Polygon([(-7.24, 6.6), (-7.24, 7.1), (-7.21, 7.1), (-7.21, 6.6)])
    object_radius = 0.005
    steer_distance = 0.015
    num_iterations = 10000000
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

    for pos in path:
        x, y = pos
        goTo(x, y, clientID, robotHandle, origin, INIT_POSITION)
        time.sleep(0.1)


    #vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(clientID)

else:
    print ('Failed connecting to remote API server')
print ('Program ended')
