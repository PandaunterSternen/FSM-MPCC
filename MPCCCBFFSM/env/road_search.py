import numpy as np
def s_to_dxdy(self, line, s):
    s_diff = abs(line['s'] - s)
    index = np.argmin(s_diff)
    dx = line['dx'][index]
    dy = line['dy'][index]
    return dx, dy


def s_to_xy(self, line, s):
    s_diff = abs(line['s'] - s)
    index = np.argmin(s_diff)
    x = line['x'][index]
    y = line['y'][index]
    return x, y

import copy
def state_to_index(self, state):
    s_diff = abs(self.center_s[self.ego_id] - state[4])
    index_in_curve_loca = np.argmin(s_diff)
    return index_in_curve_loca


def find_ego_id(self):
    search_range = 30
    index = copy.deepcopy(self.index_in_curve)
    index_area = list(range(int(index - search_range / 2), int(index + search_range / 2)))
    roads_arround = np.zeros([3, self.n_road, search_range])  # 行是路的序号，
    distence = np.zeros([self.n_road, search_range])
    for road_id in range(self.n_road):
        road_center = self.road_params[road_id]['Center_line']
        every_road = 0
        for i in index_area:
            # if i >= search_range:
            roads_arround[0, road_id, every_road] = road_center['x'][i]  # 第一行存x
            roads_arround[1, road_id, every_road] = road_center['y'][i]  # 第二行存Y
            roads_arround[2, road_id, every_road] = i  # 第三行存行
            distence[road_id, every_road] = np.sqrt((self.ego_states_current[0] - road_center['x'][i]) ** 2 + (
                        self.ego_states_current[1] - road_center['y'][i]) ** 2)
            every_road = every_road + 1
        # else:
        #    road_number = 0 #defult

    road_number, index_number = np.unravel_index(np.argmin(distence), distence.shape)
    return road_number


def reset_road_s(self):
    search_range = 50
    index = copy.deepcopy(self.index_in_curve)
    index_area = list(range(int(index - search_range / 2), int(index + search_range / 2)))
    roads_arround = np.zeros([3, self.n_road, search_range])  # 行是路的序号，
    distence = np.zeros([self.n_road, search_range])
    for road_id in range(self.n_road):
        road_center = self.road_params[road_id]['Center_line']
        every_road = 0
        for i in index_area:
            if i >= search_range:
                roads_arround[0, road_id, every_road] = road_center['x'][i]
                roads_arround[1, road_id, every_road] = road_center['y'][i]
                roads_arround[2, road_id, every_road] = i
                distence[road_id, every_road] = np.sqrt(
                    (self.ego_states_current[0] - road_center['x'][i]) ** 2 + (
                            self.ego_states_current[1] - road_center['y'][i]) ** 2)
                every_road = every_road + 1
            else:
                road_number = 0  # defult

    road_number, index_number = np.unravel_index(np.argmin(distence), distence.shape)
    self.ego_states_current[4] = self.road_params[road_number]['Center_line']['s'][index_area[index_number]]