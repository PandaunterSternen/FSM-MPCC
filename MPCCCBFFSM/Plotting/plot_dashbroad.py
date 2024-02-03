import numpy as np
import casadi as ca
from matplotlib import pyplot as plt
import matplotlib as mpl
import matplotlib.patches as mpatches

class dashboard:
    def draw_dashboard(self, x_predict_current_state, observe_car_current_state):
        # 创建一个包含多个子图的图形对象，同时指定每个子图的大小
        alpha = np.linspace(1, 0, self.N)
        #a = self.ego_states_current
        ego_states_current = ca.reshape(self.ego_states_current, -1, 1)
        ego_states_current = ego_states_current.full()

        Diagonale_line = np.sqrt(self.L ** 2 + self.W ** 2) * (1 / 2)
        Diagonale_angle = np.arctan(self.W / self.L)

        # init for size
        for ax in self.fig.axes:
            ax.cla()

        self.axes[0, 0].set_position([0.1, 0.25, 0.37, 0.6])  # 调整位置和大小
        self.axes[0, 1].set_position([0.545, 0.25, 0.425, 0.2])
        self.axes[0, 2].set_position([0.545, 0.55, 0.425, 0.2])
        # [left, bottom, width, height]
        self.axes[1, 0].set_position([0.075, 0.05, 0.175, 0.14])
        self.axes[1, 1].set_position([0.290, 0.05, 0.175, 0.14])
        self.axes[1, 2].set_position([0.575, 0.05, 0.175, 0.14])


        # plot for main

        self.axes[0, 0].set_title('Lane Change')
        #
        # road

        # self.ax.set_xlim(-20, 200)
        # self.ax.set_ylim(-5, 50)
        # self.fig.set_dpi(400)
        # init for plot

        self.axes[0, 0].set_facecolor('#333333')
        self.axes[0, 0].plot(self.road_params[0]['Right_line']['x'], self.road_params[0]['Right_line']['y'],
                             linestyle='-', color='white',
                             linewidth=3)
        self.axes[0, 0].plot(self.road_params[self.n_road - 1]['Left_line']['x'],
                             self.road_params[self.n_road - 1]['Left_line']['y'],
                             linestyle='-',
                             color='white', linewidth=3)

        for i in range(self.n_road):
            self.axes[0, 0].plot(self.road_params[i]['Right_line']['x'], self.road_params[i]['Right_line']['y'],
                                 linestyle='--',
                                 color='white',
                                 linewidth=3, dashes=(12, 18))
            self.axes[0, 0].plot(self.road_params[i]['Left_line']['x'], self.road_params[i]['Left_line']['y'],
                                 linestyle='--',
                                 color='white', linewidth=3, dashes=(12, 18))
            self.axes[0, 0].plot(self.road_params[i]['Center_line']['x'], self.road_params[i]['Center_line']['y'],
                                 linestyle='--',
                                 color='white', linewidth=0.25, dashes=(4, 3))

        # 高速公路、一级公路和城市快速路，车道分界线的尺寸为划600间隔900 ；划400间隔600。其他道路，划200间隔400。（单位cm）。
        # 高速路虚线和虚线之间的距离为9米左右，虚线6米左右
        # ego state and target state
        # prediction of states
        prediction_ego_car_color = "#0099FF"
        if not self.solver_success:
            prediction_ego_car_color = "#e84545"
        for i in range(0, self.N, 2):  # 1，2，3，4....100
            ego_pre = [[x_predict_current_state[i][0]], [x_predict_current_state[i][1]]]
            ego_pre_orientation = x_predict_current_state[i][2]
            distance_pre = [[Diagonale_line * np.cos(ego_pre_orientation + Diagonale_angle)],
                            [Diagonale_line * np.sin(ego_pre_orientation + Diagonale_angle)]]
            rect1 = mpatches.Rectangle(ego_pre - np.array(distance_pre), self.L, self.W,
                                       angle=np.rad2deg(ego_pre_orientation), alpha=alpha[i - 1],
                                       edgecolor='white', facecolor=prediction_ego_car_color)

            self.axes[0, 0].add_patch(rect1)

        position = ego_states_current[:2]
        orientation = ego_states_current[2]

        # draw ego car

        distance_ego = [Diagonale_line * np.cos(orientation + Diagonale_angle),
                        Diagonale_line * np.sin(orientation + Diagonale_angle)]
        rect2 = mpatches.Rectangle(position - np.array(distance_ego), self.L, self.W,
                                   angle=np.rad2deg(orientation), edgecolor='black', facecolor=(0.20, 1.00, 0.20))

        self.axes[0, 0].scatter(ego_states_current[0], ego_states_current[1])
        self.axes[0, 0].add_patch(rect2)

        circle1 = mpatches.Circle((position[0], position[1]),
                                  radius=self.W * 1.2 / 2, color='green', fill=False)
        circle2 = mpatches.Circle((position[0] - 0.4 * np.cos(orientation), position[1] - 0.4 * np.sin(orientation)),
                                  radius=self.W * 1.2 / 2, color='blue', fill=False, alpha=0.5)
        circle3 = mpatches.Circle((position[0] + 0.4 * np.cos(orientation), position[1] + 0.4 * np.sin(orientation)),
                                  radius=self.W * 1.2 / 2, color='blue', fill=False, alpha=0.5)
        self.axes[0, 0].add_patch(circle1)
        self.axes[0, 0].add_patch(circle2)
        self.axes[0, 0].add_patch(circle3)

        # obstacle
        other_states = []
        if self.state_other_car is not None:
            for car_id in range(self.num_cars):
                x = self.state_other_car[car_id]['States_X'][self.mpc_sim_iter]
                y = self.state_other_car[car_id]['States_Y'][self.mpc_sim_iter]
                phi = self.state_other_car[car_id]['States_psi'][self.mpc_sim_iter]
                obs = np.array([x, y])
                distance_oth = [Diagonale_line * np.cos(phi + Diagonale_angle),
                                Diagonale_line * np.sin(phi + Diagonale_angle)]
                rect3 = mpatches.Rectangle(obs - np.array(distance_oth), self.L,
                                           self.W, angle=np.rad2deg(phi), edgecolor='black', facecolor='#FFCC00')

                self.axes[0, 0].add_patch(rect3)
                self.axes[0, 0].text(x, y, str(car_id), fontsize=12, ha='center', va='center', color='white')
                self.axes[0, 0].scatter(x, y)
            # for i in range(self.Observe_num_cars):
            #     circle1 = mpatches.Circle((observe_car_current_state[2 * i], observe_car_current_state[1 + 2 * i]),
            #                               radius=self.W * 1.2 / 2, color='red', fill=False)
            #     circle2 = mpatches.Circle((observe_car_current_state[2 * i] - 0.4, observe_car_current_state[1 + 2 * i]),
            #                               radius=self.W * 1.2 / 2, color='orange', fill=False, alpha=0.5)
            #     circle3 = mpatches.Circle((observe_car_current_state[2 * i] + 0.4, observe_car_current_state[1 + 2 * i]),
            #                               radius=self.W * 1.2 / 2, color='orange', fill=False, alpha=0.5)
            #     self.axes[0, 0].add_patch(circle1)
            #     self.axes[0, 0].add_patch(circle2)
            #     self.axes[0, 0].add_patch(circle3)

            # high lever command
            #
            # arrow = mpatches.Arrow(position[0], position[1],0, self.W * self.ego_command, width=0.2, color='yellow')
            # 警告块
            # arr_length = 0
            # if self.crash_warning == 0:
            #     arr_length = 1
            #     self.axes[0, 0].plot([self.command_range + position[0], self.command_range + position[0]],
            #                          [0, self.n_road], color='#62d2a2', linewidth=10, alpha=0.5)
            # else:
            #     self.axes[0, 0].plot([self.command_range + position[0], self.command_range + position[0]],
            #                          [0, self.n_road], color='#FF521D', linewidth=10, alpha=0.5)
            #     self.axes[0, 0].text(self.command_range + position[0], self.n_road / 2, str(self.crash_warning_car_no),
            #                          fontsize=12, ha='center', va='center', color='white')
            #
            # arrow = mpatches.Arrow(position[0], position[1], arr_length, self.W * self.ego_command, width=self.W * 2,
            #                        color='#AAFFC3')
            # self.axes[0, 0].add_patch(arrow)
        else:
            print('no obstacle given, break')

        # tracking_line
        self.axes[0, 0].plot(self.tracking_line_x, self.tracking_line_y, linestyle='--', color='pink')  ##
        # constraint
        self.axes[0, 0].scatter(self.left_points[:, 0], self.left_points[:, 1], color='blue', marker='o')
        self.axes[0, 0].scatter(self.right_points[:, 0], self.right_points[:, 1], color='blue', marker='o')
        # # controls 随着N变化
        # u1 = []
        # u2 = []
        # v = []
        # for i in range(self.N):
        #     u1.append(self.controls_predict_current_state[i][0])
        #     u2.append(self.controls_predict_current_state[i][1])
        #
        # u_x = list(range(self.N))
        #
        # x_ticks = range(0, self.N, 5)  # 刻度位置
        # x_tick_labels = [str(x) for x in x_ticks]  # 刻度标签
        # # a
        # self.axes[1, 0].plot(u_x, u1)
        # self.axes[1, 0].set_xticks(x_ticks)
        # self.axes[1, 0].set_xticklabels(x_tick_labels)
        #
        # # omega
        # self.axes[1, 1].plot(u_x, u1)
        # self.axes[1, 1].set_xticks(x_ticks)
        # self.axes[1, 1].set_xticklabels(x_tick_labels)

        ## overview

        self.axes[0, 1].set_facecolor('#333333')
        self.axes[0, 1].plot(self.road_params[0]['Right_line']['x'], self.road_params[0]['Right_line']['y'],
                             linestyle='-', color='white',
                             linewidth=0.5)
        self.axes[0, 1].plot(self.road_params[self.n_road - 1]['Left_line']['x'],
                             self.road_params[self.n_road - 1]['Left_line']['y'],
                             linestyle='-',
                             color='white', linewidth=0.5)

        for i in range(self.n_road):
            self.axes[0, 1].plot(self.road_params[i]['Right_line']['x'], self.road_params[i]['Right_line']['y'],
                                 linestyle='--',
                                 color='white',
                                 linewidth=0.5, dashes=(12, 18))
            self.axes[0, 1].plot(self.road_params[i]['Left_line']['x'], self.road_params[i]['Left_line']['y'],
                                 linestyle='--',
                                 color='white', linewidth=0.5, dashes=(12, 18))
            self.axes[0, 1].plot(self.road_params[i]['Center_line']['x'], self.road_params[i]['Center_line']['y'],
                                 linestyle='--',
                                 color='white', linewidth=0.25, dashes=(4, 3))

        self.axes[0, 1].scatter(ego_states_current[0], ego_states_current[1], color='green')
        # debug target point
        # self.axes[0, 0].scatter(self.states_target[0], self.states_target[1],color = 'white')

        # controls
        iter_history = list(range(0, self.mpc_sim_iter + 1))

        self.a_history.append(self.controls_predict_current_state[0][0])
        self.omega_history.append(self.controls_predict_current_state[0][1])
        iter_history = list(range(0, self.mpc_sim_iter + 1))

        # v
        self.axes[1, 0].plot(iter_history, self.a_history)
        self.axes[1, 0].set_xticks(range(0, self.sim_step, 50))
        self.axes[1, 0].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
        self.axes[1, 0].set_xlim(self.mpc_sim_iter - 20, self.mpc_sim_iter + 5)
        self.axes[1, 0].set_ylim(self.a_min, self.a_max + 1)  # (self.v_min, self.v_max)

        # omega
        self.axes[1, 1].plot(iter_history, np.rad2deg(self.omega_history))
        self.axes[1, 1].set_xticks(range(0, self.sim_step, 50))
        self.axes[1, 1].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
        self.axes[1, 1].set_xlim(self.mpc_sim_iter - 20, self.mpc_sim_iter + 5)
        self.axes[1, 1].set_ylim(np.rad2deg(self.omega_min), np.rad2deg(self.omega_max))

        self.x_history.append(ego_states_current[0])
        self.y_history.append(ego_states_current[1])
        self.phi_history.append(ego_states_current[2])
        self.v_history.append(ego_states_current[3])

        # x y
        # v
        # self.axes[1, 2].plot(self.x_history, self.y_history)
        # self.axes[1, 2].set_xticks(range(0, self.sim_step, 50))
        # self.axes[1, 2].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
        # self.axes[1, 2].set_ylim(-10, 60)
        # self.axes[1, 2].set_xlim(-10, 200)

        self.axes[1, 2].plot(iter_history, self.v_history)
        self.axes[1, 2].set_xticks(range(0, self.sim_step, 50))
        self.axes[1, 2].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
        self.axes[1, 2].set_ylim(self.v_min, self.v_max+1)
        self.axes[1, 2].set_xlim(-10, 200)

        self.axes[0, 1].plot(self.x_history, self.y_history)

        # # phi
        # phi_history_deg = np.rad2deg(self.phi_history)
        # # Initialize empty lists for peaks and troughs
        # peak_locations = []
        # trough_locations = []
        #
        # # Identify peak and trough locations using simple comparison
        # for i in range(1, len(phi_history_deg) - 1):
        #     if phi_history_deg[i] > phi_history_deg[i - 1] and phi_history_deg[i] > phi_history_deg[i + 1]:
        #         peak_locations.append(i)
        #     elif phi_history_deg[i] < phi_history_deg[i - 1] and phi_history_deg[i] < phi_history_deg[i + 1]:
        #         trough_locations.append(i)
        #
        #
        #
        # # Plotting peaks if they exist
        # if peak_locations:
        #     self.axes[0, 2].scatter([iter_history[i] for i in peak_locations],
        #                             [phi_history_deg[i] for i in peak_locations], marker=mpl.markers.CARETUPBASE,
        #                             color='green', label='Peaks', zorder=5)
        #
        # # Plotting troughs if they exist
        # if trough_locations:
        #     self.axes[0, 2].scatter([iter_history[i] for i in trough_locations],
        #                             [phi_history_deg[i] for i in trough_locations], marker=mpl.markers.CARETDOWNBASE,
        #                             color='red', label='Troughs', zorder=5)
        # self.axes[0, 2].legend()

        self.axes[0, 2].plot(iter_history, np.rad2deg(self.phi_history))
        # Scatter plot for peaks
        # self.axes[0, 2].plot(iter_history, np.rad2deg(self.omega_history), color = 'red')


        self.axes[0, 2].set_xticks(range(0, self.sim_step, 50))
        self.axes[0, 2].set_xticklabels([str(x) for x in range(0, self.sim_step, 50)])
        self.axes[0, 2].set_ylim(np.rad2deg(self.omega_min), np.rad2deg(self.omega_max))
        self.axes[0, 2].set_xlim(-10, self.sim_iter_sum+10)
        self.axes[0, 2].grid(linestyle = "--")
        self.axes[0, 2].spines['top'].set_visible(False)
        self.axes[0, 2].spines['right'].set_visible(False)


        self.axes[1, 0].set_title('Acceleration (m/s²)')
        self.axes[1, 0].set_xlabel('Prediction Time Step (k)')
        self.axes[1, 0].set_ylabel('Acceleration (m/s²)')

        self.axes[1, 1].set_title('Angular Velocity (°/s)')
        self.axes[1, 1].set_xlabel('Prediction Time Step (k)')
        self.axes[1, 1].set_ylabel('Angular Velocity (°/s)')

        self.axes[1, 2].set_title('Velocity (m/s)')
        self.axes[1, 2].set_xlabel('Prediction Time Step (k)')
        self.axes[1, 2].set_ylabel('Velocity (m/s)')

        self.axes[0, 2].set_title('Yaw Angle (°)')
        self.axes[0, 2].set_xlabel('Prediction Time Step (k)')
        self.axes[0, 2].set_ylabel('Yaw Angle (°)')

        self.axes[0, 0].set_xlim(position[0] - 8, position[0] + 16)
        self.axes[0, 0].set_ylim(position[1] - 7, position[1] + 14)

        # if self.axes[0, 1] in self.fig.axes:
        #     self.fig.delaxes(self.axes[0, 1])
        if self.axes[1, 3] in self.fig.axes:
            self.fig.delaxes(self.axes[1, 3])
        if self.axes[0, 3] in self.fig.axes:
            self.fig.delaxes(self.axes[0, 3])

        # control yaw and arr
        # plt.tight_layout()
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        plt.pause(0.005)
