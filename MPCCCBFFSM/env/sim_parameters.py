
import numpy as np

class Params:


    def params(self):
        # 比例  1：3.5m
        self.road_type = 'curve'

        self.follow_road_id = None
        self.index_in_curve = None

        self.arr_size = 0.3  #
        self.l_r = 0.25
        self.l_f = 0.30
        self.L = 1.3
        self.W = 0.6

        self.n_road = 3
        self.road_params = None

        self.num_cars = 11  # car 0 1 2....

        self.x_min = -10
        self.x_max = 500
        self.y_min = -10
        self.y_max = 500
        self.phi_min = -np.pi
        self.phi_max = np.pi

        self.v_ave = 10
        self.v_max = 12
        self.v_min = 6

        self.s_min, self.s_max = 0, 1000

        self.a_max = 2  # 最大前向速度【物理约束】
        self.a_min = -2

        self.omega_max = np.pi / 4  # 比我的小
        self.omega_min = -self.omega_max
        self.p_min = 0
        self.p_max = 12

        self.n_states = None
        self.n_controls = None

        self.T = 0.04#0.04  # （模拟的）系统采样时间【秒】0.04 0.07

        self.N = 20  # 需要预测的步长【超参数】
        self.sim_step = 500  # 仿真时长

        #  ##状态约束
        self.lbg = [0.0, 0.0, 0.0, 0.0, 0.0]  # 一开始状态初始状态等于X0
        self.ubg = [0.0, 0.0, 0.0, 0.0, 0.0]  # x，y不得大于2
        self.g = None
        #  ## 控制约束
        self.lbx = []  # 最低约束条件
        self.ubx = []  # 最高约束条件
        ##
        self.f = None
        self.obj = None
        self.X = None
        self.P = None
        self.U = None
        self.S = None
        self.solver = None
        self.Q = None  # np.array([[0.2, 0.0, 0.0, 0.0], [0.0, 3.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
        self.R = None  # np.array([[0.5, 0.0], [0.0, 0.3]])

        self.ego_states_current = None  # 小车当前
        self.ego_id = None
        self.controls_predict_current_state = np.array([0, 0, 10] * self.N).reshape(-1, 3)  # np.ones((N, 2)) # controls 2N
        self.Observe_num_cars = 5
        self.mpc_sim_iter = 0
        self.index_target = 50
        self.s_target = 11#12
        self.ego_target_id = 0

        self.state_other_car = None
        self.ego_command = 0  # turn left 1 ， keep 0，turn right -1
        self.crash_warning = 0  #
        self.command_range = 8
        self.crash_warning_car_no = None
        self.crash_warning_car_s = 60
        self.sim_iter_rest = 380
        # draw

        self.axes = None
        self.fig = None
        self.x_history = []
        self.y_history = []
        self.phi_history = []
        self.v_history = []
        self.a_history = []
        self.sim_iter_sum = self.sim_iter_rest

        self.omega_history = []
        self.n_interp = 1000
        self.tracking_line_x = None
        self.tracking_line_y = None
        self.tracking_line_dx = None
        self.tracking_line_dy = None
        self.tracking_line_s = None
        self.right_points, self.left_points = None, None
        self.states_target = None
        self.solver_success = True

        # mpcc

        self.center_x, self.center_y = {}, {}
        self.center_dx, self.center_dy = {}, {}
        self.right_x, self.right_y = {}, {}
        self.left_x, self.left_y = {}, {}
        self.road_center_arc_seg = {}
        self.road_center_total_arc = {}
        self.center_s = {}

        self.INTEGRATION_MODE = "Euler"  # RK4 and RK3 method are the other two choices
        self.p_initial = 2.0  # projected centerline vel can set to desired value for initial estimation
        self.boundary_pub = None

        # quxian
        self.n_interp = 800
        # cbf
        self.cbf_gamma = 0.95 # 0.95
        self.fsmpra = 0.1

        self.solver_succes = True