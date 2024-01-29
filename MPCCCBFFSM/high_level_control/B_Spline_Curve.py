

from geomdl import BSpline
from geomdl import utilities
import numpy as np
import matplotlib.pyplot as plt
import math

class b_spline:
    def __init__(self, current_position, project_point, start_angle, end_angle):
        self.start_point = current_position
        self.end_point = project_point
        self.start_angle = start_angle  # 转换为弧度
        self.end_angle = end_angle  # 转换为弧度
        self.curve = BSpline.Curve()
        self.path_set()




    def path_set(self):
        # 设置度数
        self.curve.degree = 4
        dist = (self.start_point[0]-self.end_point[0])**2 + (self.start_point[1]-self.end_point[1])**2
        dist = np.sqrt(dist)

        # 计算控制点
        point02 = [self.start_point[0] + dist*0.30*np.cos(self.start_angle), self.start_point[1] + dist*0.30*np.sin(self.start_angle)]
        point_02 = [self.end_point[0] - dist*0.20*np.cos(self.end_angle), self.end_point[1] - dist*0.20*np.sin(self.end_angle)]
        point_middle1 = [self.start_point[0]+(self.end_point[0] - self.start_point[0])/2, self.start_point[1]+(self.end_point[1] - self.start_point[1])/2]
        #point_middle2 = [self.start_point[0]+(self.end_point[0] - self.start_point[0])*2/3, self.start_point[1]+(self.end_point[1] - self.start_point[1])*2/3]
        self.curve.ctrlpts = [self.start_point, point02, point_middle1, point_02, self.end_point]

        # 设置节点向量
        self.curve.knotvector = utilities.generate_knot_vector(self.curve.degree, len(self.curve.ctrlpts))
        # 通过utilities.generate_knot_vector自动生成适合的节点向量，这通常会保证至少C1的连续性（连续的一阶导数）
        # 生成曲线
        self.curve.evaluate()


    def path_get(self, N):
        # 计算均匀分布的参数值
        param_values = np.linspace(0, 1, N)

        # 计算曲线上的点和斜率
        curve_points = [self.curve.evaluate_single(p) for p in param_values]
        curve_tangents = [self.curve.derivatives(p, order=1) for p in param_values]  # 获取一阶导数

        # 提取斜率（切线方向）是弧度值
        curve_slopes = [np.arctan2(tangent[1], tangent[0]) for tangent in curve_tangents]
        return np.array(curve_points), np.array(curve_slopes)

    def road_plot(self):

        # 曲线上的点

        curve_points = self.curve.evalpts

        # 提取曲线上的点
        x_vals, y_vals = zip(*curve_points)

        # 绘制控制点和曲线
        plt.plot(x_vals, y_vals, label="B-Spline Curve")
        plt.scatter(*zip(*self.curve.ctrlpts), color='red', marker='x', label="Control Points")
        plt.legend()
        plt.show()

if __name__ == '__main__':
    current_position = [1, 2]
    project_point = [10, 1]
    start_angle = 40
    end_angle = 0
    b = b_spline(current_position, project_point, start_angle, end_angle)
    b.path_get(10)
    b.road_plot()
