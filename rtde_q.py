import math
import rtde_control, rtde_receive

rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")
rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
actual_q = rtde_r.getActualQ()
rtde_c.moveJ([0,1.6,1.5,0,0,0])
print("Q ", actual_q)
print("L ", rtde_r.getActualTCPPose()[0:3])