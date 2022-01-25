import numpy as np
import math
from scipy.spatial.transform import Rotation as R
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

    jointHeights = [
        j0_height,
        j1_height,
        j2_height,
        j3_height,
        j4_height,
        j5_height
    ]
    jointLenght = [
        j0_to_j1,
        j1_to_j2,
        j2_to_j3,
        j3_to_j4,
        j4_to_j5,
        j5_offset
    ]
    jointRot = [
        math.pi/2,
        0,
        0,
        math.pi/2,
        -math.pi/2,
        0
    ]


def NormalJointToXYZ(jointArray):
    xyz = np.eye(4)
    result_rot = np.eye(4)
    for jH, jL, jR, jQ in zip(UR5Model.jointHeights,UR5Model.jointLenght,UR5Model.jointRot, jointArray):
        rm = np.matmul(RotMat(jQ, "z"),  RotMat(jR, "x"))
        tr = np.matmul(rm, TransMat([jL, 0, 0]))
        tr = np.matmul(TransMat([0, 0, jH]), tr)
        result_rot = np.matmul(result_rot, rm)
        xyz = np.matmul(xyz, tr)

    r = R.from_matrix([result_rot[i][0:3] for i in range(3)])
    return np.concatenate([np.matmul(xyz, np.array([0, 0, 0, 1]))[0:3], r.as_rotvec()])

def main():
    test_Q = [3, 1, 2.12, 3, 1, 3]

    rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")
    rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
    
    rtde_c.moveJ(test_Q)
    myxyz = NormalJointToXYZ(test_Q)
    rtdexyz = np.array(rtde_r.getActualTCPPose())
    print("my")
    print(myxyz)
    print("rtde")
    print(rtdexyz)
    print("error")
    print(myxyz - rtdexyz)
    rtde_c.moveL(myxyz)




if __name__ == "__main__":
    main()