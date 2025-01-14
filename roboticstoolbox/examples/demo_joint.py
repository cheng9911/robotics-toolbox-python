'''
测试代码：

'''
import roboticstoolbox as rtb
import numpy as np
import rtde_receive
import rtde_control
from scipy.signal import butter, lfilter
from scipy.signal import butter, filtfilt
import sys
import time
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
# rtde_c = rtde_control.RTDEControlInterface("192.168.3.101")
robot=rtb.models.DH.sun_white()
urdf_sun=rtb.models.URDF.UR5()
while True:
    # Joint_tor=rtde_c.getJointTorques()
    # print("补偿后的机械臂力矩值",Joint_tor)
    q = np.array(rtde_r.getActualQ())
    # print("实际关节角q", q)
    TcpPose = rtde_r.getActualTCPPose()
    # print("实际TcpPose", TcpPose)
    TheoryTcp = robot.fkine(q)

    # print("理论TcpPose", TheoryTcp)
    qd = np.array(rtde_r.getActualQd())
    # print("实际关节角q速度", qd)
    qdd = np.array(rtde_r.getTargetQdd())
    # print("目标关节角加速度", qdd)

    TheoryTorques = robot.rne(q, qd, qdd)
    # print("理论计算的机械臂力矩值", TheoryTorques)
    # Joint_tor = rtde_c.getJointTorques()
    # print("补偿后的机械臂力矩值", Joint_tor)
    # # Assuming you have a list of current values called 'current_data'
    # filter = RealTimeButterworthFilter(order=4, cutoff_freq=2.0, sampling_freq=100.0)
    # filtered_current = filter.add_new_data(Jointcurrent)
    # if filtered_current is not None:
    #     print("滤波后的机械臂电流",filtered_current)
    # else:
    #     print("error")

    # 关节摩擦力矩
    # print("关节摩擦力矩", urdf_sun.friction(qd))

    K = [0, 9.83, 12.93327, 3.7047, 0, 0]
    ##机械臂原点的电流初始值
    Initcurrent = [-0.20817123, 0.2088255, -0.09637149, 0.13434261, -0.47637963, 0.0412327]

    # Jointcurrent=Jointcurrent/1000
    # print("实际读取的机械臂电流",Jointcurrent)
    compensate_torque = [0, -5.147, -3.673, 0.55, 0, 0]
    # 电流环估计输出力矩
    Estimated_Current = np.zeros(6)
    Estimated_Torq = np.zeros(6)
    # while True:
    Jointcurrent = np.array(rtde_r.getActualCurrent())
    print("实际读取的机械臂电流", Jointcurrent)
    for i in range(6):
        Estimated_Current[i] = (Jointcurrent[i] - Initcurrent[i])
        Estimated_Torq[i] = (Jointcurrent[i] - Initcurrent[i]) * K[i] - TheoryTorques[i]
        # if abs(Estimated_Torq[i]) > 10:
        #     print("碰撞的关节", i)
        #     print("碰撞的力矩", Estimated_Torq[i])
        #     print("**********collision*************")
        #     print("**********collision*************")
        #     sys.exit("Collision detected! Exiting the program.")

            # print("i",Jointcurrent[i]-Initcurrent[i])
    # print("估计电流值",Estimated_Current)
    # print("估计输出力矩", Estimated_Torq)
    # time.sleep(0.5)