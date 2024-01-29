import numpy as np
from MPCCCBFFSM.env.road_search import *
from MPCCCBFFSM.high_level_control.B_Spline_Curve import b_spline
import copy
def tracking_line_setting(self):
    # states_target = self.states_target
    # states_target[1] = self.
    #
    # X_target = self.states_target
    #
    # s_target = X_target[3]

    s_current = self.ego_states_current[4]
    s_delta = self.s_target / self.N

    s_project = np.linspace(s_current, s_current + self.s_target + s_delta, self.N + 1, endpoint=False)[1:]
    tracking_line_x = []
    tracking_line_y = []
    tracking_line_dx = []
    tracking_line_dy = []
    line = self.road_params[self.ego_target_id]['Center_line']
    for i in range(self.N):
        x, y = s_to_xy(self, line, s_project[i])
        tracking_line_x.append(x)
        tracking_line_y.append(y)
        dx, dy = s_to_dxdy(self, line, s_project[i])
        tracking_line_dx.append(dx)
        tracking_line_dy.append(dy)
    distence =np.sqrt( (self.ego_states_current[0]+0.5-tracking_line_x[0])**2 + (self.ego_states_current[1]-tracking_line_y[0])**2)

    if distence > 0.2:#0.2: #self.crash_warning:#d

        differences = [abs(s_pro - self.crash_warning_car_s) for s_pro in s_project]

        # 找到最小差值的索引
        min_index = differences.index(min(differences))
        start_xy = [self.ego_states_current[0], self.ego_states_current[1]]
        end_xy = [tracking_line_x[min_index], tracking_line_y[min_index]]
        start_angle = self.ego_states_current[2]
        end_angle = np.arctan2(tracking_line_dy[min_index], tracking_line_dx[min_index])
        b = b_spline(start_xy, end_xy, start_angle, end_angle)
        curve_points, curve_slopes = b.path_get(min_index+1)
        for i in range(min_index):
            tracking_line_x[i] = np.array([curve_points[i][0]])
            tracking_line_y[i] = np.array([curve_points[i][1]])
            tracking_line_dx[i] = np.array([curve_slopes[i][0]])
            tracking_line_dy[i] = np.array([curve_slopes[i][1]])




    return tracking_line_x, tracking_line_y, tracking_line_dx, tracking_line_dy, s_project