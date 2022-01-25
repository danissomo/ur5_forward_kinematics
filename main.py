
from ntpath import join
from matplotlib.pyplot import sci
import numpy as np
import math


import rtde_control, rtde_receive

def RotMat(phi, axis):
    if axis == "x":
        return np.array([
            [1,             0,              0, 0], 
            [0, math.cos(phi), -math.sin(phi), 0], 
            [0, math.sin(phi),  math.cos(phi), 0], 
            [0,             0,              0, 1]])
    elif axis =="y":
        return np.array([
            [ math.cos(phi), 0, math.sin(phi), 0], 
            [             0, 1,             0, 0], 
            [-math.sin(phi), 0, math.cos(phi), 0], 
            [             0, 0,             0, 1]])
    elif axis == "z":
        return np.array([
            [math.cos(phi), -math.sin(phi), 0, 0],
            [math.sin(phi),  math.cos(phi), 0, 0], 
            [            0,              0, 1, 0], 
            [            0,              0, 0, 1]])

def TransMat(xyz):
    return np.array([
        [1, 0, 0, xyz[0]],
        [0, 1, 0, xyz[1]],
        [0, 0, 1, xyz[2]],
        [0, 0, 0,     1]])

def AddMat(xyz):
    return np.array([
        [0, 0, 0, xyz[0]],
        [0, 0, 0, xyz[1]],
        [0, 0, 0, xyz[2]],
        [0, 0, 0,     0]])

class UR5Model:
    j0_height = 0.1625000043102569
    j0_to_j1  = 0

    j1_height = 0
    j1_to_j2  = -0.425

    j2_height = 0
    j2_to_j3  = -0.3922

    j3_height = 0.13329999581393556
    j3_to_j4  = 0

    j4_height = 0.09970004005417395
    j4_to_j5  = 0
    
    j5_height = 0.09959
    j5_offset = 0

def NormalJointToXYZ(jointArray):
    jointHeights = [
        UR5Model.j0_height,
        UR5Model.j1_height,
        UR5Model.j2_height,
        UR5Model.j3_height,
        UR5Model.j4_height,
        UR5Model.j5_height
    ]
    jointLenght = [
        UR5Model.j0_to_j1,
        UR5Model.j1_to_j2,
        UR5Model.j2_to_j3,
        UR5Model.j3_to_j4,
        UR5Model.j4_to_j5,
        UR5Model.j5_offset
    ]
    jointRot = [
        math.pi/2,
        0,
        0,
        math.pi/2,
        -math.pi/2,
        0
    ]
    xyz = np.eye(4)
    result_rot = np.eye(4)
    for jH, jL, jR, jQ in zip(jointHeights, jointLenght, jointRot, jointArray):
        rm = np.matmul(RotMat(jQ, "z"),  RotMat(jR, "x"))
        tr = np.matmul(rm, TransMat([jL, 0, 0])) + AddMat([0, 0, jH])
        result_rot = np.matmul(result_rot, rm)
        xyz = np.matmul(xyz, tr)

    p = [0] *3
    return np.matmul(xyz, np.array([0, 0, 0, 1]))[0:3]

def JointsToXYZ(jointArray):

    rm1 = RotMat(math.pi/2, "x")
    rm1 = np.matmul(RotMat(jointArray[0], "z"), rm1)

    rm2 =RotMat(0, "x")
    rm2 = np.matmul(RotMat(jointArray[1], "z"), rm2)

    rm3 = RotMat(0, "x")
    rm3 = np.matmul(RotMat(jointArray[2], "z"), rm3)

    rm4 = RotMat(math.pi/2, "x")
    rm4 = np.matmul(RotMat(jointArray[3], "z"), rm4)

    rm5 = RotMat(-math.pi/2, "x")
    rm5 = np.matmul(RotMat(jointArray[4], "z"), rm5)

    rm6 = RotMat(0, "x")
    rm6 = np.matmul(RotMat(jointArray[5], "z"), rm6)
    
    tr1 = np.matmul(rm1, TransMat([ UR5Model.j0_to_j1, 0, 0])) + AddMat([0, 0, UR5Model.j0_height]) 
    tr2 = np.matmul(rm2, TransMat([ UR5Model.j1_to_j2, 0, 0])) + AddMat([0, 0, UR5Model.j1_height])
    tr3 = np.matmul(rm3, TransMat([ UR5Model.j2_to_j3, 0, 0])) + AddMat([0, 0, UR5Model.j2_height])
    tr4 = np.matmul(rm4, TransMat([ UR5Model.j3_to_j4, 0, 0])) + AddMat([0, 0, UR5Model.j3_height])
    tr5 = np.matmul(rm5, TransMat([ UR5Model.j4_to_j5, 0, 0])) + AddMat([0, 0, UR5Model.j4_height])
    tr6 = np.matmul(rm6, TransMat([UR5Model.j5_offset, 0, 0])) + AddMat([0, 0, UR5Model.j5_height]) 
    xyz = np.matmul(tr1, tr2)
    xyz = np.matmul(xyz, tr3)
    xyz = np.matmul(xyz, tr4)
    xyz = np.matmul(xyz, tr5)
    xyz = np.matmul(xyz, tr6)

    
    return np.matmul(xyz, np.array([0, 0, 0, 1]))
test_Q = [0.5, 1.2, 2.12, 1, 1, 0]

rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")
rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
actual_q = rtde_r.getActualQ()

rtde_c.moveJ(test_Q)
#print(JointsToXYZ([-1.6006999999999998, -1.7271, -2.2029999999999994, -0.8079999999999998, 1.5951, -0.030999999999999694]))
print("my")
print(JointsToXYZ(test_Q))
print(NormalJointToXYZ(test_Q))
print("rtde")
print(rtde_r.getActualTCPPose())