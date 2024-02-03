import numpy as np
import casadi as ca
from matplotlib import pyplot as plt
import matplotlib as mpl
import matplotlib.patches as mpatches


def plot_results(self):
    # 读取yawangle.txt文件
    self.fig, self.axes = plt.subplots(nrows=2, ncols=1, figsize=(7, 5))
    program_directory = '/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM'
    with open('/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM/yawangle.txt', 'r') as file:
        yaw_angles = [float(line.strip().replace('[', '').replace(']', '')) for line in file.readlines()]

    # 读取steeringangle.txt文件
    with open('/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM/steeringangle.txt', 'r') as file:
        steering_angles = [float(line.strip().replace('[', '').replace(']', '')) for line in file.readlines()]

    with open('/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM/yawanglewoB.txt', 'r') as file:
        yaw_angles_oB = [float(line.strip().replace('[', '').replace(']', '')) for line in file.readlines()]

    # 读取steeringangle.txt文件
    with open('/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM/steeringanglewoB.txt', 'r') as file:
        steering_angles_oB = [float(line.strip().replace('[', '').replace(']', '')) for line in file.readlines()]

    iter_history = list(range(0, self.mpc_sim_iter ))
    color1 = (251 / 255, 132 / 255, 2 / 255)  # 蓝色系
    color2 = (1 / 255, 53 / 255, 101 / 255)  # 红色系

    # 绘制Yaw角度
    self.axes[0].plot(iter_history, np.rad2deg(yaw_angles), color=color1,linewidth= 3.0, label = 'With B-spline')
    self.axes[0].plot(iter_history, np.rad2deg(yaw_angles_oB), color=color2, linestyle='-.',linewidth= 2.0,label = 'Without B-spline')

    self.axes[0].set_xticks(range(0, self.sim_step, 50))
    self.axes[0].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
    self.axes[0].set_ylim(np.rad2deg(self.omega_min), np.rad2deg(self.omega_max))
    self.axes[0].set_xlim(-10, self.sim_iter_sum + 10)
    self.axes[0].grid(linestyle="--")
    self.axes[0].legend()
    self.axes[0].set_title('Yaw Angle (°)')
    self.axes[0].set_xlabel('Prediction Time Step (k)')
    self.axes[0].set_ylabel('Angle (°)')
    self.axes[0].spines['top'].set_visible(False)
    self.axes[0].spines['right'].set_visible(False)

    # 绘制Steering角度
    self.axes[1].plot(iter_history, np.rad2deg(steering_angles), color=color1 ,linewidth= 3.0,label = 'With B-spline')
    self.axes[1].plot(iter_history, np.rad2deg(steering_angles_oB), color=color2 ,linestyle='-.',linewidth= 1.5, label = 'Without B-spline')

    self.axes[1].set_xticks(range(0, self.sim_step, 50))
    self.axes[1].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
    self.axes[1].set_ylim(np.rad2deg(self.omega_min), np.rad2deg(self.omega_max))
    self.axes[1].set_xlim(-10, self.sim_iter_sum + 10)
    self.axes[1].grid(linestyle="--")
    self.axes[1].legend()
    self.axes[1].set_title('Steering Angle (°)')
    self.axes[1].set_xlabel('Prediction Time Step (k)')
    self.axes[1].set_ylabel('Angle (°)')
    self.axes[1].spines['top'].set_visible(False)
    self.axes[1].spines['right'].set_visible(False)

    plt.tight_layout()  # 调整子图间距
    plt.show()

def plot_results1(self):
    # 读取yawangle.txt文件
    self.fig, self.axes = plt.subplots(nrows=2, ncols=1, figsize=(7, 5))
    program_directory = '/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM'
    with open('/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM/yawanglew3.txt', 'r') as file:
        yaw_angles3 = [float(line.strip().replace('[', '').replace(']', '')) for line in file.readlines()]

    # 读取steeringangle.txt文件
    with open('/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM/steeringanglew3.txt', 'r') as file:
        steering_angles3 = [float(line.strip().replace('[', '').replace(']', '')) for line in file.readlines()]

    with open('/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM/yawanglew4.txt', 'r') as file:
        yaw_angles4 = [float(line.strip().replace('[', '').replace(']', '')) for line in file.readlines()]

    # 读取steeringangle.txt文件
    with open('/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM/steeringanglew4.txt', 'r') as file:
        steering_angles4= [float(line.strip().replace('[', '').replace(']', '')) for line in file.readlines()]

    iter_history = list(range(0, self.mpc_sim_iter))
    color1 = (251 / 255, 132 / 255, 2 / 255)  # 蓝色系
    color2 = (1 / 255, 53 / 255, 101 / 255)  # 红色系

    # 绘制Yaw角度
    self.axes[0].plot(iter_history, np.rad2deg(yaw_angles4), color=color1, linewidth=3.0, label='With FSM')
    self.axes[0].plot(iter_history, np.rad2deg(yaw_angles3), color=color2, linestyle='-.', linewidth=2.0,
                      label='Without FSM')

    self.axes[0].set_xticks(range(0, self.sim_step, 50))
    self.axes[0].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
    self.axes[0].set_ylim(np.rad2deg(self.omega_min), np.rad2deg(self.omega_max))
    self.axes[0].set_xlim(-10, self.sim_iter_sum + 10)
    self.axes[0].grid(linestyle="--")
    self.axes[0].legend(loc='lower left')
    self.axes[0].set_title('Yaw Angle (°)')
    self.axes[0].set_xlabel('Prediction Time Step (k)')
    self.axes[0].set_ylabel('Angle (°)')
    self.axes[0].spines['top'].set_visible(False)
    self.axes[0].spines['right'].set_visible(False)

    # 绘制Steering角度
    self.axes[1].plot(iter_history, np.rad2deg(steering_angles4), color=color1, linewidth=3.0, label='With FSM')
    self.axes[1].plot(iter_history, np.rad2deg(steering_angles3), color=color2, linestyle='-.', linewidth=2.0,
                      label='Without FSM')

    self.axes[1].set_xticks(range(0, self.sim_step, 50))
    self.axes[1].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
    self.axes[1].set_ylim(np.rad2deg(self.omega_min) , np.rad2deg(self.omega_max) )
    self.axes[1].set_xlim(-10, self.sim_iter_sum + 10)
    self.axes[1].grid(linestyle="--")
    self.axes[1].legend(loc='lower left')
    self.axes[1].set_title('Steering Angle (°)')
    self.axes[1].set_xlabel('Prediction Time Step (k)')
    self.axes[1].set_ylabel('Angle (°)')
    self.axes[1].spines['top'].set_visible(False)
    self.axes[1].spines['right'].set_visible(False)

    plt.tight_layout()  # 调整子图间距
    plt.show()

