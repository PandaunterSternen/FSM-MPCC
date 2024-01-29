import casadi as ca

import numpy as np

def mpcmodel(self):
    # 根据数学模型建模
    # 系统状态
    x = ca.SX.sym('x')  # x轴状态x 的符号基元 未知数
    y = ca.SX.sym('y')  # y轴状态
    # yaw = ca.SX.sym
    phi = ca.SX.sym('phi')  # z轴转角　('yaw')
    v = ca.SX.sym('v')
    s = ca.SX.sym('s')  # 前向距离

    states = ca.vertcat(x, y, phi, v, s)  # 构建小车状态向量 \bm{x} = [x, y, omega].T
    # states = ca.vertcat(states, omega)  # 实际上也可以通过 states = ca.vertcat(*[x, y, omega])一步实现
    self.n_states = states.size()[0]  # 在列上排列，获得系统状态的尺寸，向量以（n_states, 1）的格式呈现 【这点很重要】
    # [x, y, omega]
    ## 控制输入
    a = ca.SX.sym('a')
    omega = ca.SX.sym('omega')  # 转动速度 这应该是delta阿
    p = ca.SX.sym('p')
    controls = ca.vertcat(a, omega, p)  # 控制向量
    self.n_controls = controls.size()[0]  # 控制向量尺寸
    # v, omega)

    # 运动学模型
    beta = np.arctan(self.l_r * np.tan(omega) / (self.l_f + self.l_r))
    # rhs = ca.vertcat(v * np.cos(omega + beta), v * np.sin(omega + beta), (v * np.sin(beta) / self.l_r),p)
    # rhs = ca.vertcat(v * np.cos(phi), v * np.sin(phi), (v / self.L) * np.tan(omega), p)
    rhs = ca.vertcat(v * np.cos(phi + beta), v * np.sin(phi + beta), (v * np.sin(beta) / self.l_r), a, p)

    # 利用CasADi构建一个函数
    self.f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])
    # def f(states, controls):
    # 	return rhs

    # 开始构建MPC
    ## 相关变量，格式(状态长度， 步长)
    self.U = ca.SX.sym('U', self.n_controls, self.N)  # N步内的控制输出
    self.X = ca.SX.sym('X', self.n_states, self.N + 1)  # N+1步的系统状态，通常长度比控制多1

    # parameters (which include the nth degree polynomial coefficients,
    # initial state and the reference along the predicted trajectory (reference states and reference controls))

    self.P = ca.SX.sym('P',
                       self.n_states + self.n_states + 3 * self.Observe_num_cars + 6 * self.N + self.N * self.n_controls)  # 构建问题的相关参数 包含初始状态和目标状态的向量
    # 在这里 p = (x0, xt( traget ), obstacle xy xy xy , 2N , N * num of control)+ self.n_states * self.N
    # # Single Shooting 约束条件
    # # NLP问题
    # ## 惩罚矩阵