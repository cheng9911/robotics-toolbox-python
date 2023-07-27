"""
[sun_Teach_Pendant]

功能：
[机械臂的拖动示教的尝试，利用UR自身获取关节力矩函数getJointTorques()]
作者：
[sun,https://github.com/cheng9911]

存在的问题：
[代码目前处于开发阶段，拖动示教功能不完善，请小心使用。]

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
import time
import signal
import keyboard
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
rtde_c = rtde_control.RTDEControlInterface("192.168.3.101")
joint_num=1
class JointControl:
    def __init__(self, joint_mass, joint_damping, joint_stiffness, max_velocity, max_acceleration,initpose):
        self.joint_mass = joint_mass
        self.joint_damping = joint_damping
        self.joint_stiffness = joint_stiffness
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

        self.current_position = initpose
        self.current_velocity = [0,0,0,0,0,0]

    def update_par(self,velocity,num):

        self.current_velocity[num]=velocity
        # self.current_acceleration=acceleration
    def update(self, external_torque, dt):
        i=joint_num

        while i<6:
            #计算当前关节位置的加速度
            current_acceleration= (external_torque[i] - self.joint_damping[i] * self.current_velocity[i]) / self.joint_mass[i]

            # 限制加速度
            current_acceleration = max(-self.max_acceleration[i], min(self.max_acceleration[i], current_acceleration))
            print("current_acceleration",current_acceleration)
            # 计算当前关节位置的速度
            self.current_velocity[i] += current_acceleration * dt

            # 限制速度
            self.current_velocity[i] = max(-self.max_velocity[i], min(self.max_velocity[i], self.current_velocity[i]))
            print("self.current_velocity",self.current_velocity[i])
            # 计算当前关节位置
            self.current_position[i] += self.current_velocity[i] * dt
            if self.current_position[i]<-180*np.pi/180 or self.current_position[i]>180*np.pi/180:
                sys.exit("关节角超限")
                rtde_c.servoStop()
                rtde_c.stopScript()
                sys.exit("关节角超限")
            else:
                i+=1

        return self.current_position
def signal_handler(signal, frame):
    print("\n程序退出")
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
# 测试代码
if __name__ == "__main__":

    ##关节num{0,1,2,3,4,5]
    # joint_num=1

    robot = rtb.models.DH.sun_white()
    urdf_sun = rtb.models.URDF.UR5()
    ##假设UR的自身getJointTorques()函数对机械臂的重力和摩擦力进行了补偿
    Joint_tor = rtde_c.getJointTorques()
    print("补偿后的机械臂力矩值",Joint_tor)
    K = [0, 0, 0, 0, 0, 1.2]
    M = [0, 0, 0, 0, 0, 2]
    B = [0, 0, 0, 0, 0, 3]
    # while True:
    #     # 使用select实现非阻塞等待用户输入
    #     sys.stdout.flush()
    #     # 使用msvcrt模块实现非阻塞等待用户输入
    #     if keyboard.is_pressed("esc"):
    #         sys.exit("ESC退出程序.")
    #     Joint_tor = rtde_c.getJointTorques()
    #     print("补偿后的机械臂力矩值", Joint_tor)
    #
    #     time.sleep(0.5)

    # except KeyboardInterrupt:
    #     sys.exit("Ctrl+C退出程序.")
    # 设置关节的物理参数和约束
    ##拖动
    mass = [3.761, 8.058, 2.846, 1.37, 1.3, 0.365]
    ##关节参数：关节阻尼【0，2，1，0.5，0.1，0.1】
    #关节力的阈值【0，10，7，5，3，3】
    #减去【0，10，7，5，0，0】
    #关节质量【】
    joint_dec=[0,15,12,4.5,2.5,1.5]
    joint_mass = [10,20,6,3,2,1]  # 关节质量
    joint_damping = [2,15 ,10,5,1.5,1] # 关节阻尼
    joint_stiffness = [10.0 ,10,10,10,10,10] # 关节刚度
    max_velocity = [0.1,0.1,0.1,0.2,0.2,0.2]  # 最大速度限制
    max_acceleration = [0.1 ,0.1,0.1,0.2,0.2,0.2] # 最大加速度限制
    ##可更改为任意一个初始值
    # initpose=[0,-90*np.pi/180,-90*np.pi/180,0,0,0]
    initpose=rtde_r.getTargetQ()
    # rtde_c.moveJ(initpose, 0.5, 0.5)
    ##
    dt_ctr = 0.01
    ##servoj
    velocity = 0.2
    acceleration = 0.2
    dt = 1.0 / 500  # 2ms
    lookahead_time = 0.15
    gain = 300
    joint_q = rtde_r.getActualQ()


    # 创建关节控制器

    joint_ctrl = JointControl(joint_mass, joint_damping, joint_stiffness, max_velocity, max_acceleration,initpose)
    # 模拟时间步长和外部力矩
    while True:
        # 使用select实现非阻塞等待用户输入
        sys.stdout.flush()
        # 使用msvcrt模块实现非阻塞等待用户输入
        if keyboard.is_pressed("esc"):
            rtde_c.servoStop()
            rtde_c.stopScript()
            sys.exit("ESC退出程序.")
        Joint_tor = rtde_c.getJointTorques()
        print("补偿后的机械臂力矩值", Joint_tor)
        external_torque = Joint_tor
        # external_torque =16
        # print("外力",external_torque)
        j=joint_num
        while j <6:
            print("j",j)
            if abs(external_torque[j]) - joint_dec[j]>0:
            #模拟控制过程
            # 获取目标关节位置
                if external_torque[j] < 0:
                    external_torque[j]=external_torque[j] +joint_dec[j]
                    print("j", j)
                    print("力矩值为负", external_torque[j])
                elif external_torque[j] > 0:
                    print("j",j)
                    external_torque[j] = external_torque[j] - joint_dec[j]
                    print("力矩值为正", external_torque[j])
            else:
                external_torque[j]=0
                # update_velocity= rtde_r.getActualQd()[j]
                update_velocity = 0
                joint_ctrl.update_par(update_velocity,j)
            j+=1
        print("下发的力矩数据",external_torque)
        target_position = joint_ctrl.update(external_torque , dt_ctr)
        # print("补偿后的机械臂力矩值", Joint_tor)
        print("target_position",target_position)
        t_start = rtde_c.initPeriod()
        rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
        joint_q= target_position
        rtde_c.waitPeriod(t_start)
                # elif external_torque >0:
                #     print("力矩值为正", external_torque)
                #     print("补偿后的机械臂力矩值", Joint_tor)
                #     target_position = joint_ctrl.update(external_torque - 15, dt_ctr)
                #     print("target_position", target_position)
                #     t_start = rtde_c.initPeriod()
                #     rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
                #     joint_q[joint_num] = target_position
                #     rtde_c.waitPeriod(t_start)
                #
            # else:
            #     # print("力矩值不够",external_torque)
            #
            #     update_velocity=rtde_r.getActualQd()[joint_num]
            #     joint_ctrl.update_par(update_velocity)
            #     pass
    #
    #
    #     # 打印当前关节位置
    #     print(f"Current joint position: {joint_ctrl.current_position:.3f}")






