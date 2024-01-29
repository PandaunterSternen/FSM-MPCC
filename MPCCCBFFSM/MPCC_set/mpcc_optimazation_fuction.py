import casadi as ca
import numpy as np




def mpc_optimazation_init(self):
    mpc_optimazation_model(self)
    mpc_slover_init(self)
    mpc_constraints_init(self)
def mpc_optimazation_model(self):
    #  ## 优化目标
    self.obj = 0
    #  ## 剩余N状态约束条件
    self.g = []  # equal constrains
    self.g.append(self.X[:, 0] - self.P[:self.n_states])  # 初始状态希望相等 (TODO ) 4

    # self.ego_states_current, 4 self.states_target 4 , obs_obs_xy, 10
    # self.tracking_line_current 80

    for i in range(self.N):  #
        # 通过前述函数获得下个时刻系统状态变化。
        # 这里需要注意引用的index为[:, i]，因为X为(n_states, N+1)矩阵

        x = self.X[:, i]
        x_next = self.X[:, i + 1]
        u = self.U[:, i]
        # dx = self.P[self.n_states + self.n_states + 2 * self.Observe_num_cars + i]
        # dy = self.P[self.n_states + self.n_states + 2 * self.Observe_num_cars + self.N + i]
        # t_angle = np.arctan(dy, dx)
        t_angle = self.P[self.n_states + self.n_states + 3 * self.Observe_num_cars + i]
        # 其实就等于零 t_angle = ca.atan2(dy, dx)  # 轨道中心线在当前位置处的切线角度。
        # 对应的 sin angle = 0 cos = 1
        ref_x = self.P[self.n_states + self.n_states + 3 * self.Observe_num_cars + self.N * 2 + i]
        ref_y = self.P[self.n_states + self.n_states + 3 * self.Observe_num_cars + self.N * 3 + i]

        # Contouring error
        error_con = (np.sin(t_angle) * (x_next[0] - ref_x) - np.cos(t_angle) * (
                x_next[1] - ref_y))# * (1 / (1 + np.exp(0.1 * i)))
        error_lag = (-np.cos(t_angle) * (x_next[0] - ref_x) - np.sin(t_angle) * (x_next[1] - ref_y))# * (1 / (1 + np.exp(0.1 * i)))
        error = ca.vertcat(error_con, error_lag)

        # 约束条件的起始索引 con_start_idx 和结束索引 con_end_idx。
        #
        con_start_idx = self.n_states + self.n_states + 3 * self.Observe_num_cars + 6 * self.N + self.n_controls * i
        con_end_idx = self.n_states + self.n_states + 3 * self.Observe_num_cars + 6 * self.N + self.n_controls * (
                i + 1)

        self.obj = (self.obj + ca.mtimes(error.T, ca.mtimes(self.Q, error)) + (x[3] - u[2]) ** 2 * 100
                    + ca.mtimes(ca.mtimes((u - self.P[con_start_idx:con_end_idx]).T, self.R),
                                (u - self.P[con_start_idx:con_end_idx])))

        # 据当前时间步和下一个时间步的控制输入计算控制差的平方，并将其添加到目标函数中。这个目标函数的计算是为了最小化连续时间内控制输入的变化。
        if i < self.N - 1:
            u_next = self.U[:, i + 1]
            self.obj += ca.mtimes((u_next - u).T, ca.mtimes(self.S, (u_next - u)))

        k1 = self.f(x, u)
        x_next_euler = 0
        if self.INTEGRATION_MODE == "Euler":  # Euler
            x_next_euler = x + (self.T * k1)
        elif self.INTEGRATION_MODE == "RK3":  # 采用三阶 Runge-Kutta 法进行积分
            k2 = self.f(x + self.T / 2 * k1, u)
            k3 = self.f(x + self.T * (2 * k2 - k1), u)
            x_next_euler = x + self.T / 6 * (k1 + 4 * k2 + k3)
        elif self.INTEGRATION_MODE == "RK4":  # 四阶 Runge-Kutta 法进行积分
            k2 = self.f(x + self.T / 2 * k1, u)
            k3 = self.f(x + self.T / 2 * k2, u)
            k4 = self.f(x + self.T * k3, u)
            x_next_euler = x + self.T / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        self.g.append(x_next - x_next_euler)
        #    g.append(ca.sqrt((X[0, i]-P[6*N+4, i])**2+(X[1, i]-P[6*N+5, i])**2))

        # path boundary constraints
        self.g.append(
            self.P[self.n_states + self.n_states + 3 * self.Observe_num_cars + self.N * 4 + i] * x_next[0] - self.P[
                self.n_states + self.n_states + 3 * self.Observe_num_cars + self.N * 5 + i] * x_next[
                1])  # LB<=ax-by<=UB  --represents half space planes

        r = self.W * 1.1
        for near_car in range(self.Observe_num_cars):  # 5
            gap = 0.4
            # ego_car_front_point_x, ego_car_front_point_y
            efx = self.X[0, i] + gap * np.cos(self.X[2, i])
            efy = self.X[1, i] + gap * np.sin(self.X[2, i])
            # ego_car_back_point_x, ego_car_back_point_y
            ebx = self.X[0, i] - gap * np.cos(self.X[2, i])
            eby = self.X[1, i] - gap * np.sin(self.X[2, i])
            # obs_car_front_point_x, obs_car_front_point_y
            ofx = self.P[self.n_states + self.n_states + 3 * near_car + 0] + gap * np.cos(
                self.P[self.n_states + self.n_states + 3 * near_car + 2])
            ofy = self.P[self.n_states + self.n_states + near_car * 3 + 1] + gap * np.sin(
                self.P[self.n_states + self.n_states + 3 * near_car + 2])
            # obs_car_back_point_x, obs_car_back_point_y
            obx = self.P[self.n_states + self.n_states + 3 * near_car + 0] - gap * np.cos(
                self.P[self.n_states + self.n_states + 3 * near_car + 2])
            oby = self.P[self.n_states + self.n_states + near_car * 3 + 1] - gap * np.sin(
                self.P[self.n_states + self.n_states + 3 * near_car + 2])
            # 对中心点
            # 自车前端点和观察车辆前端点
            # self.g.append(ca.sqrt((self.X[0, i] - self.P[7 + 3 * near_car + 1]) ** 2 + (
            #         self.X[1, i] - self.P[7 + near_car * 3 + 2]) ** 2))
            # self.g.append(ca.sqrt((ebx - self.P[7 + 3 * near_car + 1]) ** 2 + (
            #         eby - self.P[7 + near_car * 3 + 2]) ** 2))
            # self.g.append(ca.sqrt((efx - self.P[7 + 3 * near_car + 1]) ** 2 + (
            #         efy - self.P[7 + near_car * 3 + 2]) ** 2))
            b_11 = ca.sqrt((self.X[0, i] - self.P[self.n_states + self.n_states + 3 * near_car + 0]) ** 2 + (
                    self.X[1, i] - self.P[self.n_states + self.n_states + near_car * 3 + 2]) ** 1) - r
            b_next_11 = ca.sqrt((self.X[0, i + 1] - self.P[self.n_states + self.n_states + 3 * near_car + 0]) ** 2 +
                                (self.X[1, i + 1] - self.P[self.n_states + self.n_states + 3 * near_car + 1]) ** 2) - r
            self.g.append(b_next_11 - b_11 + self.cbf_gamma * b_11)
            # 自车后端点和观察车辆后端点
            b_12 = ca.sqrt((ebx - self.P[self.n_states + self.n_states + 3 * near_car + 0]) ** 2 + (
                    eby - self.P[self.n_states + self.n_states + near_car * 3 + 1]) ** 2) - r
            b_next_12 = ca.sqrt((self.X[0, i + 1] - gap * np.cos(self.X[2, i + 1]) - self.P[
                self.n_states + self.n_states + 3 * near_car + 0]) ** 2 +
                                (self.X[1, i + 1] - gap * np.sin(self.X[2, i + 1]) - self.P[
                                    self.n_states + self.n_states + near_car * 3 + 1]) ** 2) - r
            self.g.append(b_next_12 - b_12 + self.cbf_gamma * b_12)

            # 自车前端点和观察车辆后端点的 CBF
            b_13 = ca.sqrt((efx - self.P[self.n_states + self.n_states + 3 * near_car + 0]) ** 2 + (
                    efy - self.P[self.n_states + self.n_states + 3 * near_car + 1]) ** 2) - r
            b_next_13 = ca.sqrt((self.X[0, i + 1] + gap * np.cos(self.X[2, i + 1]) - self.P[
                self.n_states + self.n_states + 3 * near_car + 0]) ** 2 +
                                (self.X[1, i + 1] + gap * np.sin(self.X[2, i + 1]) - self.P[
                                    self.n_states + self.n_states + 3 * near_car + 1]) ** 2) - r
            self.g.append(b_next_13 - b_13 + self.cbf_gamma * b_13)

            #        self.g.append(ca.sqrt((self.X[0, i] - obx) ** 2 + (
            #         self.X[1, i] - oby) ** 2))
            # self.g.append(ca.sqrt((ebx - obx) ** 2 + (eby - oby) ** 2))
            # self.g.append(ca.sqrt((efx - obx) ** 2 + (efy - oby) ** 2))
            # 自车后端点和观察车辆前端点的 CBF
            b_21 = ca.sqrt((self.X[0, i] - obx) ** 2 + (
                    self.X[1, i] - oby) ** 2) - r
            b_next_21 = ca.sqrt((self.X[0, i + 1] - obx) ** 2 +
                                (self.X[1, i + 1] - oby) ** 2) - r
            self.g.append(b_next_21 - b_21 + self.cbf_gamma * b_21)

            b_22 = ca.sqrt((ebx - obx) ** 2 + (eby - oby) ** 2) - r
            b_next_22 = ca.sqrt((self.X[0, i + 1] - gap * np.cos(self.X[2, i + 1]) - obx) ** 2 +
                                (self.X[1, i + 1] - gap * np.sin(self.X[2, i + 1]) - oby) ** 2) - r
            self.g.append(b_next_22 - b_22 + self.cbf_gamma * b_22)

            b_23 = ca.sqrt((efx - obx) ** 2 + (efy - oby) ** 2) - r
            b_next_23 = ca.sqrt(
                (self.X[0, i + 1] + gap * np.cos(self.X[2, i + 1]) - obx) ** 2 +
                (self.X[1, i + 1] + gap * np.sin(self.X[2, i + 1]) - oby) ** 2) - r
            self.g.append(b_next_23 - b_23 + self.cbf_gamma * b_23)

            # self.g.append(ca.sqrt((self.X[0, i] - ofx) ** 2 + (self.X[1, i] - ofy) ** 2))
            # self.g.append(ca.sqrt((ebx - ofx) ** 2 + (eby - ofy) ** 2))
            # self.g.append(ca.sqrt((efx - ofx) ** 2 + (efy - ofy) ** 2))
            b_31 = ca.sqrt((self.X[0, i] - ofx) ** 2 + (self.X[1, i] - ofy) ** 2) - r
            b_next_31 = ca.sqrt((self.X[0, i + 1] - ofx) ** 2 +
                                (self.X[1, i + 1] - ofy) ** 2) - r
            self.g.append(b_next_31 - b_31 + self.cbf_gamma * b_31)

            b_32 = ca.sqrt((ebx - ofx) ** 2 + (eby - ofy) ** 2) - r
            b_next_32 = ca.sqrt((self.X[0, i + 1] - gap * np.cos(self.X[2, i + 1]) - ofx) ** 2 +
                                (self.X[1, i + 1] - gap * np.sin(self.X[2, i + 1]) - ofy) ** 2) - r
            self.g.append(b_next_32 - b_32 + self.cbf_gamma * b_32)

            b_33 = ca.sqrt((efx - ofx) ** 2 + (efy - ofy) ** 2) - r
            b_next_33 = ca.sqrt(
                (self.X[0, i + 1] + gap * np.cos(self.X[2, i + 1]) - ofx) ** 2 +
                (self.X[1, i + 1] + gap * np.sin(self.X[2, i + 1]) - ofy) ** 2) - r
            self.g.append(b_next_33 - b_33 + self.cbf_gamma * b_33)

def mpc_slover_init(self):
    # 定义优化问题
    #  # 输入变量。和之前不同的是，这里需要同时将系统状态X也作为优化变量输入，
    #  # 根据CasADi要求，必须将它们都变形为一维向量

    opt_variables = ca.vertcat(ca.reshape(self.U, -1, 1),
                               ca.reshape(self.X, -1, 1))
    ## 和前例相同的定义方式和设置

    # self.opts["ipopt"] = {}
    # self.opts["ipopt"]["max_iter"] = 2000
    # self.opts["ipopt"]["print_level"] = 0
    # self.opts["verbose"] = self.param['ipopt_verbose']
    # self.opts["jit"] = True
    # self.opts["print_time"] = 0
    # self.opts["ipopt"]["acceptable_tol"] = 1e-8
    # self.opts["ipopt"]["acceptable_obj_change_tol"] = 1e-6
    # self.opts["ipopt"]["fixed_variable_treatment"] = "make_parameter"
    # self.opts["ipopt"]["linear_solver"] = "ma57" （todo 参数参考纠正）

    nlp_prob = {'f': self.obj, 'x': opt_variables, 'p': self.P, 'g': ca.vertcat(*self.g)}
    opts_setting = {'ipopt.max_iter': 200, 'ipopt.print_level': 5, 'print_time': 0,
                    'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6}

    self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

# todo
# def search_id(self):
#     ego_id = self.n_road + 1
#
#     ego_x = self.ego_states_current[0]
#     ego_y = self.ego_states_current[1]
#     dist_comp = 20
#     for road_id in range(self.n_road):
#         x = self.road_params[road_id]['Center_line']['x']
#         y = self.road_params[road_id]['Center_line']['y']
#         for interpoint in range(self.n_interp):
#             dist = (x[interpoint] - ego_x) ** 2 + (y[interpoint] - ego_y) ** 2
#             if dist_comp > dist:
#                 ego_id = road_id
#                 dist_comp = dist
#     return ego_id

def mpc_constraints_init(self):

    for _ in range(self.N):  # 之后每一段开始也是得相等
        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.lbg.append(0.0)

        self.lbg.append(10.0)

        self.ubg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)

        self.ubg.append(5.0)

        for _ in range(self.Observe_num_cars):
            self.lbg.append(0.05)
            self.ubg.append(np.inf)
            self.lbg.append(0.05)
            self.ubg.append(np.inf)
            self.lbg.append(0.05)
            self.ubg.append(np.inf)

            self.lbg.append(0.05)
            self.ubg.append(np.inf)
            self.lbg.append(0.05)
            self.ubg.append(np.inf)
            self.lbg.append(0.05)
            self.ubg.append(np.inf)

            self.lbg.append(0.05)
            self.ubg.append(np.inf)
            self.lbg.append(0.05)
            self.ubg.append(np.inf)
            self.lbg.append(0.05)
            self.ubg.append(np.inf)

    # for car_id in range(1, num_cars + 1):
    #     for _ in range(3):
    #         for _ in range(N + 1):
    #             lbg.append(W) #最小距离0.3
    #             ubg.append(np.inf)

    #  ### 因为之前定义的时候是先U再X所以这里定义的时候也需要先定义U的约束后再添加X约束

    for _ in range(self.N):
        self.lbx.append(self.a_min)
        self.lbx.append(self.omega_min)
        self.lbx.append(self.p_min)

        self.ubx.append(self.a_max)
        self.ubx.append(self.omega_max)
        self.ubx.append(self.p_max)

    for _ in range(self.N + 1):
        self.lbx.append(self.x_min)
        self.lbx.append(self.y_min)
        self.lbx.append(self.phi_min)
        self.lbx.append(self.v_min)
        self.lbx.append(self.s_min)

        self.ubx.append(self.x_max)
        self.ubx.append(self.y_max)
        self.ubx.append(self.phi_max)
        self.ubx.append(self.v_max)
        self.ubx.append(self.s_max)
