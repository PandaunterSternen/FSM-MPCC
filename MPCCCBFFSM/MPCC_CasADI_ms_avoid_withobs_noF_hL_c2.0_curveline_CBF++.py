import casadi as ca

import numpy as np
import time
import os

from MPCCCBFFSM.env.OtherVehicle_curve_fest import OtherVehicleCurve_fest
from matplotlib import pyplot as plt


from MPCCCBFFSM.env.road_config import Road
from MPCCCBFFSM.env.sim_parameters import Params
from MPCCCBFFSM.env.road_search import *
from MPCCCBFFSM.Plotting.plot_dashbroad import dashboard
from MPCCCBFFSM.Plotting.result_ploting import plot_results
from MPCCCBFFSM.Plotting.result_ploting import plot_results1
from MPCCCBFFSM.MPCC_set.mpcc_model import mpcmodel
from MPCCCBFFSM.MPCC_set.mpcc_optimazation_fuction import mpc_optimazation_init
from MPCCCBFFSM.high_level_control.FSM import FSM
from MPCCCBFFSM.high_level_control.tracking_line import tracking_line_setting







class MPC:
    def __init__(self):
        Params.params(self)

        self.Q = np.zeros((2, 2))
        self.Q[0, 0] = 1000  # self.param['mpc_w_cte']  # cross track error
        self.Q[1, 1] = 400  # 700   # self.param['mpc_w_lag']  # lag error

        self.R = np.zeros((3, 3))
        self.R[0, 0] = 0.3  # self.param['mpc_w_vel']  # use of velocity control
        self.R[1, 1] = 200  # self.param['mpc_w_delta']  # use of steering actuator.  weighing matrices (controls)
        self.R[2, 2] = 0  # self.param['mpc_w_p']  # projected velocity input for progress along the track

        self.S = np.zeros((3, 3))
        self.S[0, 0] = 0.2  # self.param['mpc_w_accel']  # change in velocity i.e, acceleration
        self.S[1, 1] = 1200#500 # self.param['mpc_w_delta_d']  # change in steering angle. weighing matrices (change in controls)
        self.S[2, 2] = 0  # self.param['mpc_w_delta_p']


    ## todo
    def shift_movement(self, T, t0, x0, u, x_predict_current_state, f):
        f_value = f(x0, u[0, :])  # u 不一样[obstacle[i]['States_X'][], obstacle[i]['States_Y']]
        st = x0 + T * f_value.full()
        t = t0 + T
        u_end = np.concatenate((u[1:], u[-1:]))  # 串联矩阵
        # 去掉第一个，最后一个复制一下
        x_predict_next_state = np.concatenate((x_predict_current_state[1:], x_predict_current_state[-1:]), axis=0)
        return t, st, u_end.T, x_predict_next_state

    def get_path_constraints_points(self, X0):
        right_points = np.zeros((self.N, 2))
        left_points = np.zeros((self.N, 2))

        for k in range(1, self.N + 1):
            X_track = X0[k, :]
            index = state_to_index(self, X_track)

            right_points[k - 1, :] = np.array([self.right_x[0][index],
                                               self.right_y[0][index]]).squeeze()  # Right boundary
            left_points[k - 1, :] = np.array([self.left_x[self.n_road - 1][index],
                                              self.left_y[self.n_road - 1][index]]).squeeze()  # Left boundary
        return right_points, left_points

    # for k in range(1, self.N + 1):
    #     index = self.state_to_index(X0[k - 1, :])
    #
    #     right_points[k - 1, :] = np.array([self.right_x[0][index],
    #                                        self.right_y[0][index]]).squeeze()  # Right boundary
    #     left_points[k - 1, :] = np.array([self.left_x[self.n_road - 1][index],
    #                                       self.left_y[self.n_road - 1][index]]).squeeze()  # Left boundary

    def set_road_env(self):
        road_init = Road(self.n_road, self.n_interp, self.road_type)
        self.road_params = road_init.road
        for road_id in range(self.n_road):
            self.center_x[road_id], self.center_y[road_id] = (self.road_params[road_id]['Center_line']['x'],
                                                              self.road_params[road_id]['Center_line'][
                                                                  'y'])  # c_x、c_y：轨道中心线的x和y坐标。
            self.center_dx[road_id], self.center_dy[road_id] = (self.road_params[road_id]['Center_line']['dx'],
                                                                self.road_params[road_id]['Center_line']['dy'])
            # c_dx, c_dy  # c_dx、c_dy：轨道中心线的x和y方向的变化率（斜率）
            self.right_x[road_id], self.right_y[road_id] = (self.road_params[road_id]['Right_line']['x'],
                                                            self.road_params[road_id]['Right_line']['y'])
            # r_x、r_y：右侧轨道边界的x和y坐标。 不一定能用到
            self.left_x[road_id], self.left_y[road_id] = (self.road_params[road_id]['Left_line']['x'],
                                                          self.road_params[road_id]['Left_line']['y'])
            # l_x、l_y：左侧轨道边界的x和y坐标
            self.road_center_arc_seg[road_id] = self.road_params[road_id]['Center_line']['road_arc_seg']
            # element_arc_lengths：每个元素（段）的弧长。
            self.road_center_total_arc[road_id] = self.road_params[road_id]['Center_line']['road_total_arc']  # 原始轨道的总弧长
            self.center_s[road_id] = self.road_params[road_id]['Center_line']['s']







    def mpc_simulation(self):
        # intialisation of simulation
        t0 = 0.0  # 仿真时间
        states_history = []
        self.set_road_env()


        init_id = 0 #
        init_index = 5 #
        self.ego_states_current = np.array([self.center_x[init_id][init_index], self.center_y[init_id][init_index],
                                            np.arctan(self.center_dy[init_id][init_index], self.center_dx[init_id][init_index]),
                                            [self.v_ave], self.center_s[init_id][init_index]]).reshape(-1, 1)
        STATE_INIT = self.ego_states_current.copy()
        self.ego_id = init_id

        self.index_in_curve = state_to_index(self, self.ego_states_current)
        target_index = int(self.index_in_curve + self.index_target)

        self.states_target = [self.road_params[self.ego_id]['Center_line']['x'][target_index],
                              self.road_params[self.ego_id]['Center_line']['y'][target_index],
                              [0],
                              [self.v_ave],
                              self.road_params[self.ego_id]['Center_line']['s'][target_index]]
        self.states_target = np.array(self.states_target).reshape(-1, 1)
        # todo dx dy 可能是不对的
        self.tracking_line_x, self.tracking_line_y, self.tracking_line_dx, self.tracking_line_dy, self.tracking_line_s = tracking_line_setting(self)
        x_predict_current_state = np.zeros((self.N + 1, self.n_states))
        x_predict_current_state[0, :] = self.ego_states_current.reshape(-1)
        x_predict_current_state[1:, 0] = np.array(self.tracking_line_x).reshape(-1)
        x_predict_current_state[1:, 1] = np.array(self.tracking_line_y).reshape(-1)
        x_predict_current_state[1:, 2] = np.array(
            np.arctan(np.array(self.tracking_line_dy), np.array(self.tracking_line_dx))).reshape(-1)
        x_predict_current_state[1:, 3] = self.v_ave * np.ones([self.N, 1]).reshape(-1)
        x_predict_current_state[1:, 4] = np.array(self.tracking_line_s.reshape(-1)).reshape(-1)

        next_states = x_predict_current_state.copy()
        # xs = np.array([25, 1.5, 0.0, 1.0]).reshape(-1, 1)  # 小车末了状态

        x_predict_history = []  # 存储系统的状态
        u_c = []  # 存储控制全部计算后的控制指令
        t_c = []  # 保存时间
        x_history = []  # 存储每一步车辆位置

        #  # 开始仿真
        self.mpc_sim_iter = 0  # 迭代计数器
        start_time = time.time()  # 获取开始仿真时间
        #  ## 终止条件为小车和目标的欧式距离小于0.01或者仿真超时
        index_t = []
        sim_iter = self.sim_step  # 仿真时长 逐次递减

        self.OtherVehicle = OtherVehicleCurve_fest(num_cars=self.num_cars, sim_step=self.sim_step, T=self.T, v=self.v_ave,
                                         n_road=self.n_road, N=self.N, road_params=self.road_params)

        self.state_other_car = self.OtherVehicle.get_all_states()

        # obs_xy = []  # car0 x0 y0 x1 y1
        # obs_phi = []
        # for car_id in range(self.num_cars):
        #     obs_xy.append(self.state_other_car[car_id]['States_X'])
        #     obs_xy.append(self.state_other_car[car_id]['States_Y'])
        #     obs_phi.append(self.state_other_car[car_id]['States_psi'])
        # obs_xy = np.array(obs_xy).reshape(-1, 1)
        # ocs_phi = np.array(obs_phi).reshape(-1, 1)
        self.fig, self.axes = plt.subplots(nrows=2, ncols=4, figsize=(16, 12))
        last_road_ID = 0


        while self.sim_iter_rest > 0:
            #  ## 初始化优化参数


            self.ego_id = find_ego_id(self)
            obs_obs_xyphi = []  # the N coordinates of obstacles within the observed are
            obs_i_s = []  # The coordinates (x) of all obstacles/vehicles
            observe_car_current_state = []  # the current coordinates xy xy of obstacles within the observed are
            variations_states = self.OtherVehicle.get_variations_states(self.mpc_sim_iter)
             # 计算一下ego车在哪条道路上
            # 对是否撞车和是否左转，右转进行一次更新
            self.crash_warning = 0  # Is there any obstacle ahead of the vehicle
            self.ego_command = 0 # 0 -1 1
            # 参数化 X*N+1
            # Y*N 2*N+1 obs_x_predict = obs_x[mpciter:mpciter+N]

            for car_id in range(self.num_cars):  # Retrieve the positions x of all vehicles
                obs_i_s.append(variations_states[car_id]['States_s'][0])
            differences = abs(obs_i_s - self.ego_states_current[4])
            closed_car_id = np.argsort(np.array(differences).flatten())[:self.Observe_num_cars]  # 获取离ego车 最近的self.Observe_num_cars 的ID
            # 这个要改
            # 防止 min函数出错的初始化，不影响逻辑
            ego_left_car_s = [self.ego_states_current[4] + 50]
            ego_right_car_s = [self.ego_states_current[4] + 60]
            ego_ahead_car_s = [self.ego_states_current[4] + 100]
            #
            # 从最近的五辆车中，找在行驶方向前的，不在前边的，设到不干扰地点
            # 并当前边有车的时候发出warning
            min_as = 1000 # ahead s
            min_rs = 1000
            min_ls = 1000
            lv = 1000 # left velocity
            av = 1000
            rv = 1000
            d_1 = 0
            for car_id in closed_car_id:
                car_id = int(car_id)
                obs_i_s = variations_states[car_id]['States_s'][0]
                if 20 >= obs_i_s - self.ego_states_current[4] >= -2:
                    obs_obs_xyphi.append(variations_states[car_id]['States_X'][0])
                    observe_car_current_state.append(variations_states[car_id]['States_X'][0])
                    obs_obs_xyphi.append(variations_states[car_id]['States_Y'][0])
                    observe_car_current_state.append(variations_states[car_id]['States_Y'][0])
                    obs_obs_xyphi.append(variations_states[car_id]['States_psi'][0])
                    observe_car_current_state.append(variations_states[car_id]['States_psi'][0])
                    d_1 = (obs_i_s - self.ego_states_current[4])

                    if variations_states[car_id]['road_ID'] == self.ego_id and -3 < (obs_i_s - self.ego_states_current[
                        4] )<= self.command_range:
                        self.crash_warning = 1
                        self.crash_warning_car_no = car_id

                        ego_ahead_car_s.append(variations_states[car_id]['States_s'][0])
                        if variations_states[car_id]['States_s'][0] < min_as:
                            av = variations_states[car_id]['States_v'][0]
                            min_as = variations_states[car_id]['States_s'][0]
                            self.crash_warning_car_s = min_as

                    elif variations_states[car_id]['road_ID'] == self.ego_id + 1 and \
                            obs_i_s - self.ego_states_current[4] >= -1:
                        ego_left_car_s.append(variations_states[car_id]['States_s'][0])
                        if variations_states[car_id]['States_s'][0] < min_ls:
                            lv = variations_states[car_id]['States_v'][0]
                            min_ls = variations_states[car_id]['States_s'][0]

                    elif variations_states[car_id]['road_ID'] == self.ego_id - 1 and \
                            obs_i_s - self.ego_states_current[4] >= -1:
                        ego_right_car_s.append(variations_states[car_id]['States_s'][0])
                        if variations_states[car_id]['States_s'][0] < min_rs:
                            rv = variations_states[car_id]['States_v'][0]
                            min_rs = variations_states[car_id]['States_s'][0]

                else:
                    obs_obs_xyphi.append([-50])
                    obs_obs_xyphi.append([-50])
                    obs_obs_xyphi.append([0])
                    observe_car_current_state.append([-50])
                    observe_car_current_state.append([-50])
                    observe_car_current_state.append([0])

            obs_obs_xyphi_new = np.array(obs_obs_xyphi).reshape(-1, 1)  # 5*3 （x,y,psi,x,y,psi）
            obs_obs_xyphi = obs_obs_xyphi_new
            ego_left_ahead_right_car_v = [lv, av, rv]
            # 要是不转向就是原方向
            ego_s = self.ego_states_current[4]  # 1.429

            if self.crash_warning:


                FSM(self, ego_left_car_s, ego_right_car_s, ego_ahead_car_s, ego_s, ego_left_ahead_right_car_v)
            # self.ego_target_id = self.ego_id #代替 fsm



            self.index_in_curve = state_to_index(self, self.ego_states_current)
            target_index = int(self.index_in_curve + self.index_target)
            # if self.mpc_sim_iter == 50:
            #     self.ego_target_id = 1


            self.states_target = [self.road_params[self.ego_target_id]['Center_line']['x'][target_index],
                                  self.road_params[self.ego_target_id]['Center_line']['y'][target_index],
                                  np.arctan(self.road_params[self.ego_target_id]['Center_line']['dy'][target_index],
                                            self.road_params[self.ego_target_id]['Center_line']['dx'][target_index]),
                                  [self.v_ave],
                                  self.road_params[self.ego_target_id]['Center_line']['s'][target_index]]
            self.states_target = np.array(self.states_target).reshape(-1, 1)

            self.ego_id = find_ego_id(self)

            # if self.ego_id != last_road_ID:
            #     self.reset_road_s()
            #     last_road_ID = copy.deepcopy(self.ego_id)

            self.tracking_line_x, self.tracking_line_y, self.tracking_line_dx, self.tracking_line_dy, self.tracking_line_s = tracking_line_setting(self)  # 4*30
            self.tracking_line_k = np.arctan(np.array(self.tracking_line_dy), np.array(self.tracking_line_dx))
            # self.tracking_line_x = np.array(self.tracking_line_x).reshape(-1, 1)
            # self.tracking_line_y = np.array(self.tracking_line_y).reshape(-1, 1)
            # c_p = np.concatenate((self.ego_states_current, self.states_target, obs_obs_xy, self.tracking_line_current))

            c_p = np.concatenate((self.ego_states_current, self.states_target, obs_obs_xyphi,
                                  self.tracking_line_k, self.tracking_line_dy,
                                  self.tracking_line_x, self.tracking_line_y,
                                  np.zeros([2 * self.N + self.N * self.n_controls, 1])))
            # P #for i in range(N):
            # 函数将传递的数组序列沿指定的轴进行连接，返回连接后的新数组
            #  ## 初始化优化目标变量
            # init_control = ca.reshape(u0, -1, 1)  # ?
            # 在这地方 warm start
            self.tracking_line_current = np.zeros([self.N, self.n_states])
            self.tracking_line_current[:, 0] = np.array(self.tracking_line_x).squeeze()
            self.tracking_line_current[:, 1] = np.array(self.tracking_line_y).squeeze()
            self.tracking_line_current[:, 2] = self.tracking_line_k.squeeze()
            self.tracking_line_current[:, 3] = (self.v_ave * np.ones([self.N, 1])).squeeze()
            # todo
            if self.crash_warning:#self.solver_succes == False:
                init_control = np.concatenate((self.controls_predict_current_state.reshape(-1, 1),
                                               self.ego_states_current, self.tracking_line_current.reshape(-1,1)))
                #    (self.controls_predict_current_state.reshape(-1, 1), next_states.reshape(-1, 1)))
            else:
                # warm start
                init_control = np.concatenate(
                    (self.controls_predict_current_state.reshape(-1, 1), next_states.reshape(-1, 1)))
            # 这地方应该放
            self.right_points, self.left_points = self.get_path_constraints_points(x_predict_current_state)
            index_g = 0
            # 4* 21
            for k in range(self.N):  # set the reference controls and path boundary conditions to track
                delta_x_path = self.right_points[k, 0] - self.left_points[k, 0]
                delta_y_path = self.right_points[k, 1] - self.left_points[k, 1]
                c_p[self.n_states + self.n_states + 3 * self.Observe_num_cars + self.N * 4 + k] = -delta_x_path
                c_p[self.n_states + self.n_states + 3 * self.Observe_num_cars + self.N * 5 + k] = delta_y_path

                up_bound = max(-delta_x_path * self.right_points[k, 0] - delta_y_path * self.right_points[k, 1],
                               -delta_x_path * self.left_points[k, 0] - delta_y_path * self.left_points[k, 1])
                # 它表示在当前路径点处，控制变量的值不能超过一定的上限。这个上限通过考虑右边点和左边点的影响，计算出右边点和左边点中哪一个会对控制变量产生更大的影响，并且使用较大的那个值来限制上界
                low_bound = min(-delta_x_path * self.right_points[k, 0] - delta_y_path * self.right_points[k, 1],
                                -delta_x_path * self.left_points[k, 0] - delta_y_path * self.left_points[k, 1])
                index_g = self.n_states-1 + (k + 1) * (self.n_states+1) + k * 9 * self.Observe_num_cars
                self.lbg[index_g] = low_bound
                self.ubg[index_g] = up_bound

                # obstacle parameters

                # Control parameters
                a_ref = 0
                p_ref = self.v_ave
                omega_ref = 0

                control_ref_index_start = self.n_states + self.n_states + 3 * self.Observe_num_cars + 6 * self.N + self.n_controls * k
                control_ref_index_end = control_ref_index_start + self.n_controls

                c_p[control_ref_index_start: control_ref_index_end] = np.array([a_ref, omega_ref, p_ref]).reshape(-1, 1)
            #  ## 计算结果并且
            t_ = time.time()
            res = self.solver(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx, ubg=self.ubg,
                              ubx=self.ubx)  # 怎么做到的
            self.solver_success = self.solver.stats()['success']
            # res 有f:目标函数的最小化结果或系统的性能指标
            # g 约束函数向量的最优结果
            # lam_g 约束函数向量的拉格朗日乘子，用于表示约束的敏感度
            # lam_p 问题参数的拉格朗日乘子
            # lam_x 目标函数或性能指标的拉格朗日乘子
            # x 最优的控制变量值 184&1 前边的 2*30（N）为控制 后边的 4 * N+1 为状态
            index_t.append(time.time() - t_)

            estimated_opt = res['x'].full()  # 先转换为numpy 提取 变量

            self.controls_predict_current_state = estimated_opt[:self.n_controls * self.N].reshape(self.N,
                                                                                                   self.n_controls)
            # self.controls_predict_current_state = np.array([10, 0.2, 0]*self.N).reshape(-1, 3)
            # (N,n_controls) 已知前2*（N）为U的结果，转换成
            x_predict_current_state = estimated_opt[self.n_controls * self.N:].reshape(self.N + 1, self.n_states)
            # (N+1, n_states) 剩余部分为MPC计算出之后N步可能的状态
            x_predict_history.append(x_predict_current_state.T)
            u_c.append(self.controls_predict_current_state[0, :])
            t_c.append(t0)

            dashboard.draw_dashboard(self, x_predict_current_state, observe_car_current_state)

            #  1, n_states*1, n_controls * N, n_states*(N+1)
            t0, self.ego_states_current, self.controls_predict_current_state, next_states \
                = self.shift_movement(self.T, t0,
                                      self.ego_states_current,
                                      self.controls_predict_current_state,
                                      x_predict_current_state,
                                      self.f)
            self.ego_states_current = ca.reshape(self.ego_states_current, -1, 1)
            self.ego_states_current = self.ego_states_current.full()
            x_history.append(self.ego_states_current)

            # self.ego_states_current = np.array(x_predict_current_state[1, :]).reshape(-1,1)

            self.sim_iter_rest = self.sim_iter_rest - 1
            self.mpc_sim_iter = self.mpc_sim_iter + 1
        # 输出文件
        # program_directory = '/home/xinwensu/桌面/mpc_cbf/MPCC/MPCCCBFFSM'
        # os.makedirs(program_directory, exist_ok=True)
        # # 使用os.path.join来构建文件路径，确保路径在不同操作系统上都能正确工作
        # yawangle_file_path = os.path.join(program_directory, 'yawanglew3.txt')
        # steeringangle_file_path = os.path.join(program_directory, 'steeringanglew3.txt')
        # phi_history_strings = [f"{item}\n" for item in self.phi_history]
        # omega_history_strings = [f"{item}\n" for item in self.omega_history]
        #
        #
        # with open(yawangle_file_path, 'w') as file:
        #     file.writelines(phi_history_strings)
        #
        # with open(steeringangle_file_path, 'w') as file:
        #     file.writelines(omega_history_strings)

        plt.show()


        plot_results(self)

        # t_v = np.array(index_t)
        # print(t_v.mean())
        # print((time.time() - start_time) / (300))
        # print("the distance is %s" % (np.linalg.norm(self.ego_states_current - self.states_target)))
        # draw_result = Draw_MPC_Obstacle(rob_diam=0.3, L=self.L, W=self.W, N_Road=self.n_road, init_state=x0_,
        #                                 target_state=self.states_target,
        #                                 ego_states=x_history, x_c=x_predict_history, step=self.N,
        #                                 obstacle=self.state_other_car, export_fig=True,
        #                                 num_cars=self.num_cars)


if __name__ == '__main__':
    mpc = MPC()
    mpcmodel(mpc)
    mpc_optimazation_init(mpc)

    mpc.mpc_simulation()
