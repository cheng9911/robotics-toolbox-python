"""
[sun_action]

功能：
[实现运动状态下UR5E机械臂的外力估计。主要考虑误差项：自身重力，摩擦力模型]

作者：
[sun,https://github.com/cheng9911]

存在的问题：
[代码目前处于开发阶段，对于外力的估计尚不完善。]

注意事项：
[请务必一定要先读取数据，看实际估计的力矩和理论力矩的差值。]

许可证：
[使用UR_rtde，故许可和UR_rtde一致。]

"""
import roboticstoolbox as rtb
import numpy as np
import rtde_receive
import rtde_control
from scipy.signal import butter, lfilter
from scipy.signal import butter, filtfilt
import sys
class RealTimeButterworthFilter:
    def __init__(self, order=4, cutoff_freq=1.0, sampling_freq=100.0):
        self.order = order
        self.cutoff_freq = cutoff_freq
        self.sampling_freq = sampling_freq
        self.nyquist_freq = 0.5 * sampling_freq
        self.normal_cutoff = cutoff_freq / self.nyquist_freq
        self.b, self.a = butter(order, self.normal_cutoff, btype='low', analog=False)
        self.prev_data = []

    def add_new_data(self, new_data):
        # Append new data to the buffer
        self.prev_data.append(new_data)

        # If the buffer size exceeds the filter order, apply the filter and return the result
        if len(self.prev_data) >= self.order:
            filtered_data = self.apply_filter()
            return filtered_data

        # Return None if there are not enough data points to apply the filter
        return None

    def apply_filter(self):
        # Apply the Butterworth filter to the current buffer of data
        filtered_data = lfilter(self.b, self.a, self.prev_data)
        # Discard the oldest data point to slide the window
        self.prev_data.pop(0)
        return filtered_data[-1]

j=0

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
# rtde_c = rtde_control.RTDEControlInterface("192.168.3.101")
robot=rtb.models.DH.sun_white()
urdf_sun=rtb.models.URDF.UR5()
print(robot)
q=np.array(rtde_r.getTargetQ())
print("实际关节角q",q)
qd=np.array(rtde_r.getTargetQd())
print("实际关节角q速度",qd)
qdd=np.array(rtde_r.getTargetQdd())
print("目标关节角加速度",qdd)

TheoryTorques=robot.rne(q,qd,qdd)
print("理论计算的机械臂力矩值",TheoryTorques)


# # Assuming you have a list of current values called 'current_data'
# filter = RealTimeButterworthFilter(order=4, cutoff_freq=2.0, sampling_freq=100.0)
# filtered_current = filter.add_new_data(Jointcurrent)
# if filtered_current is not None:
#     print("滤波后的机械臂电流",filtered_current)
# else:
#     print("error")




#关节摩擦力矩
print("关节摩擦力矩",urdf_sun.friction(qd))

K=[0,10.43,12.93327,3.7047,0,0]
##机械臂原点的电流初始值
Initcurrent= [-0.20817123 , 0.2088255  ,-0.09637149,  0.13434261 ,-0.47637963 , 0.0412327 ]

# Jointcurrent=Jointcurrent/1000
# print("实际读取的机械臂电流",Jointcurrent)
compensate_torque=[0,-5.147,-3.673,0.55,0,0]
friction_torque=[0,243,0,0,0,0]
#电流环估计输出力矩
Estimated_Current=np.zeros(6)
Estimated_Torq=np.zeros(6)
while True:
    q = np.array(rtde_r.getTargetQ())
    print("实际关节角q", q)
    qd = np.array(rtde_r.getTargetQd())
    print("实际关节角q速度", qd)
    qdd = np.array(rtde_r.getTargetQdd())
    print("目标关节角加速度", qdd)
    TheoryTorques = robot.rne(q, qd, qdd)
    print("理论计算的机械臂力矩值", TheoryTorques)
    print("关节摩擦力矩", urdf_sun.friction(qd))
    Jointcurrent=np.array(rtde_r.getActualCurrent())
    print("实际读取的机械臂电流",Jointcurrent)
    for i in range(6):
        Estimated_Current[i]=(Jointcurrent[i]-Initcurrent[i])
        Estimated_Torq[i]=(Jointcurrent[i]-Initcurrent[i])*K[i]-TheoryTorques[i]-friction_torque[i]*qd

    print("估计电流值",Estimated_Current)
    print("估计输出力矩",Estimated_Torq)

# while True:
#     Jointcurrent=np.array(rtde_r.getActualCurrent())
#     print("实际读取的机械臂电流",Jointcurrent)
#     for i in range(6):
#         Estimated_Current[i]=(Jointcurrent[i]-Initcurrent[i])
#         Estimated_Torq[i]=(Jointcurrent[i]-Initcurrent[i])*K[i]-TheoryTorques[i]
#         if abs(Estimated_Torq[i])>10:
#             print("碰撞的力矩",Estimated_Torq[i])
#             print("**********collision*************")
#             print("**********collision*************")
#             sys.exit("Collision detected! Exiting the program.")
#
#         # print("i",Jointcurrent[i]-Initcurrent[i])
#     print("估计电流值",Estimated_Current)
#     print("估计输出力矩",Estimated_Torq)

##补偿输出力矩


# print(urdf_panda.friction(qd))
#
# a=np.array([-2.8973, 2.8973])*180/np.pi
