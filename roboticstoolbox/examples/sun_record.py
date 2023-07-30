"""
[sun_action]

功能：
[实现静止状态下UR5E机械臂的数据采集。以而关节为例，每次运动0.01rad随后采集电流和计算理论力矩，通过matlab的数据拟合工具cftool（终端输入即可）进行数据拟合求取拟合参数]

作者：
[sun,https://github.com/cheng9911]

存在的问题：
[代码目前处于开发阶段，对于外力的采集尚不完善。]

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
import csv
import time
# 创建机械臂模型和连接到机械臂控制器
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
rtde_c = rtde_control.RTDEControlInterface("192.168.3.101")
robot = rtb.models.DH.sun_white()

# 创建空列表来存储数据
current_data = []
torque_data = []
Estimated_Torq=np.zeros(6)
# 每次运动的关节角度增量（弧度）
joint_increment = 0.1
K=[0,9.83,12.93327,3.7047,0,0]
##机械臂原点的电流初始值
Initcurrent= [-0.20817123 , 0.2088255  ,-0.09637149,  0.13434261 ,-0.47637963 , 0.0412327 ]

# 运动二关节从0到-π，每次增加0.01弧度
joint_angle_range = np.arange(np.pi/2, -np.pi/2 - joint_increment, -joint_increment)
print(joint_angle_range)
##初始姿态
InitJoint=np.array([0,-1.5707,np.pi/2,-1.5707,0,0])
rtde_c.moveJ(InitJoint,0.1,0.1)
time.sleep(2.0)
# 运动每次只涉及二关节，并静止3秒，重复采集数据多次
num_samples = len(joint_angle_range)  # 采样次数

for i in range(num_samples):
    # 获取当前关节角
    q = np.array([0,-1.5707,joint_angle_range[i],-1.5707,0,0])


    # 控制机械臂运动到新的关节角
    rtde_c.moveJ(q,0.1,0.1)  # 取消注释这行，如果你需要使用控制接口来控制机械臂运动

    # 静止3秒，等待电流稳定
    time.sleep(2.0)

    # 读取实际电流和计算理论力矩值
    joint_Torque = np.array(rtde_c.getJointTorques())
    Jointcurrent = np.array(rtde_r.getActualCurrent())
    TheoryTorques=TheoryTorques=robot.rne(q,np.zeros(6),np.zeros(6))
    for i in range(6):

        Estimated_Torq[i] = (Jointcurrent[i] - Initcurrent[i]) * K[i] - TheoryTorques[i]
    theory_torque = Estimated_Torq

    print("测量次数",i)
    # 存储数据到列表中
    current_data.append(joint_Torque)
    torque_data.append(theory_torque)

# 将数据保存到CSV文件
with open('joint_torque_data2.csv', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    # 写入表头
    csv_writer.writerow(['Joint1 Torque', 'Joint2 Torque', 'Joint3 Torque', 'Joint4 Torque', 'Joint5 Torque', 'Joint6 Torque',
                         'Theory Torque Joint1', 'Theory Torque Joint2', 'Theory Torque Joint3', 'Theory Torque Joint4', 'Theory Torque Joint5', 'Theory Torque Joint6'])

    # 写入数据
    for i in range(num_samples):
        csv_writer.writerow(current_data[i].tolist() + torque_data[i].tolist())























