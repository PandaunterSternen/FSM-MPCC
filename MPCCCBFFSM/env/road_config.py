import numpy as np
import matplotlib.pyplot as plt
import copy
from scipy.interpolate import CubicSpline


class Road:
    def __init__(self, n_road , n_interp, road_type):#
        self.n_interp = n_interp
        # 原始数据点
        self.road = {}
        self.n_road = n_road


        for road_id in range(self.n_road):
            self.road[road_id] = {
                'Right_line': {'x': None,
                               'y': None
                               },

                'Center_line': {'x': None,
                                'y': None,
                                'dx': None,
                                'dy': None,
                                'road_arc_seg': None,
                                'road_total_arc': None,
                                's': None,
                                'psi': None
                                },
                'Left_line': {'x': None,
                              'y': None
                              },
            }
        self.ax = None
        if road_type == 'curve':
            self.x_data = np.array([0, 25, 40, 80, 120, 140, 170, 200])

            self.y_data = np.array([0, 0, 0, 10, 14, 12, 20, 22])
            self.set_curve_road()
        # self.y_data = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        elif road_type == 'straight':
            self.set_straight_road()


    # 使用多项式拟合得到平滑曲线
    def set_straight_road(self):
        x_stop = 300
        points = np.linspace(0, x_stop, self.n_interp)
        y_start = 0



        for i in range(self.n_road):
            self.road[i]['Right_line']['x'] = points.reshape(-1, 1)
            self.road[i]['Right_line']['y'] = y_start * np.ones((self.n_interp, 1))
            y_start += 0.5

            self.road[i]['Center_line']['x'] = points.reshape(-1, 1)
            self.road[i]['Center_line']['y'] = y_start * np.ones((self.n_interp, 1))
            self.road[i]['Center_line']['dx'] = np.ones((self.n_interp, 1))
            self.road[i]['Center_line']['dy'] = y_start * np.zeros((self.n_interp, 1))
            self.road[i]['Center_line']['road_arc_seg'] = x_stop/self.n_interp
            self.road[i]['Center_line']['road_total_arc'] = x_stop
            self.road[i]['Center_line']['s'] = points.reshape(-1, 1)
            self.road[i]['Center_line']['psi'] = np.zeros((self.n_interp, 1))

            y_start += 0.5

            self.road[i]['Left_line']['x'] = points.reshape(-1, 1)
            self.road[i]['Left_line']['y'] = y_start * np.ones((self.n_interp, 1))





    def set_curve_road(self):
        # cs = CubicSpline(self.x_data, self.y_data)
        # 使用三次样条插值生成平滑曲线
        cs = CubicSpline(self.x_data, self.y_data, bc_type='clamped')
        # 在给定的范围内生成插值曲线上的点
        x_start = np.linspace(min(self.x_data), max(self.x_data), self.n_interp)
        y_start = cs(x_start)

        # 初始化第一条公路
        self.road[0]['Right_line']['x'] = x_start.reshape(-1, 1)
        self.road[0]['Right_line']['y'] = y_start.reshape(-1, 1)

        for road_id in range(self.n_road):
            # 中轴线
            x_right = self.road[road_id]['Right_line']['x']
            y_right = self.road[road_id]['Right_line']['y']

            k_right = np.zeros((self.n_interp, 1))
            for i in range(self.n_interp):
                if i == self.n_interp - 1:
                    k_right[i] = k_right[i - 1]
                elif x_right[i + 1] - x_right[i] == 0:
                    k_right[i] = np.inf
                else:
                    k_right[i] = (y_right[i + 1] - y_right[i]) / (x_right[i + 1] - x_right[i])
            # y_interp = poly(x_interp)
            # 选择一个点作为参考点
            # coefficients = np.polyfit(x_data, y_data, 4)
            # poly = np.poly1d(coefficients)

            k_vertically = - 1 / k_right
            coefficient = np.where(k_vertically > 0, 1, -1)
            dx = coefficient * 1 / (k_vertically ** 2 + 1) ** (1 / 2)
            # dx = 1 / (k_vertically ** 2 + 1) ** (1 / 2)
            dy = np.abs(k_vertically * dx)

            x_center = x_right + 0.5 * dx
            y_center = y_right + 0.5 * dy
            x_left = x_right + dx
            y_left = y_right + dy

            self.road[road_id]['Center_line']['x'] = x_center
            self.road[road_id]['Center_line']['y'] = y_center

            self.road[road_id]['Left_line']['x'] = x_left
            self.road[road_id]['Left_line']['y'] = y_left
            if road_id < self.n_road - 1:
                self.road[road_id + 1]['Right_line']['x'] = x_left
                self.road[road_id + 1]['Right_line']['y'] = y_left

            k_center = np.zeros((self.n_interp, 1))
            for i in range(self.n_interp):
                if i == self.n_interp - 1:
                    k_center[i] = k_center[i - 1]
                elif x_center[i + 1] - x_center[i] == 0:
                    k_center[i] = np.inf
                else:
                    k_center[i] = (y_center[i + 1] - y_center[i]) / (x_center[i + 1] - x_center[i])

            x_center_after = copy.deepcopy(x_center)
            y_center_after = copy.deepcopy(y_center)

            x_center_after[:-1] = x_center[1:]
            y_center_after[:-1] = y_center[1:]
            dx = x_center_after - x_center
            dy = y_center_after - y_center

            self.road[road_id]['Center_line']['dx'] = dx
            self.road[road_id]['Center_line']['dy'] = dy
            self.road[road_id]['Center_line']['psi'] = np.arctan(k_center)

            road_center_element_length = np.zeros((self.n_interp, 1))
            s = np.zeros([self.n_interp, 1])
            current_sum = 0
            for i in range(self.n_interp):
                road_center_element_length[i] = (dx[i] ** 2 + dy[i] ** 2) ** (1 / 2)
                current_sum += road_center_element_length[i]
                s[i] = current_sum

            self.road[road_id]['Center_line']['road_arc_seg'] = road_center_element_length
            self.road[road_id]['Center_line']['road_total_arc'] = current_sum
            self.road[road_id]['Center_line']['s'] = s

            # 绘制原始曲线、参考点和平行线

    def draw_road(self):
        plt.figure(figsize=(40, 10))
        self.ax = plt.axes()
        # self.ax.set_xlim(-20, 200)
        # self.ax.set_ylim(-5, 50)
        # self.fig.set_dpi(400)
        # init for plot

        self.ax.set_facecolor('#333333')
        self.ax.plot(self.road[0]['Right_line']['x'], self.road[0]['Right_line']['y'], linestyle='-', color='white',
                     linewidth=3)
        self.ax.plot(self.road[self.n_road - 1]['Left_line']['x'], self.road[self.n_road - 1]['Left_line']['y'],
                     linestyle='-',
                     color='white', linewidth=3)

        for i in range(self.n_road):
            self.ax.plot(self.road[i]['Right_line']['x'], self.road[i]['Right_line']['y'], linestyle='--',
                         color='white',
                         linewidth=3, dashes=(12, 18))
            self.ax.plot(self.road[i]['Left_line']['x'], self.road[i]['Left_line']['y'], linestyle='--',
                         color='white', linewidth=3, dashes=(12, 18))

        plt.show()


if __name__ == '__main__':
    road = Curve_road(3,1000)
    road.draw_road()
