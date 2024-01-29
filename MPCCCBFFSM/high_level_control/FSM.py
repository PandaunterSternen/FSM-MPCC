

def FSM_CBF_SEFTY(self, ego_s, min_ahead_s, FSM_state, ego_left_ahead_right_car_v, ):
    ego_v = self.ego_states_current[3]
    min_ahead_v = ego_left_ahead_right_car_v[-FSM_state + 1]
    if min_ahead_v <= ego_v:
        h = (min_ahead_s - ego_s - self.L - (1 + self.fsmpra) * ego_v - 0.5 * (
                min_ahead_v - ego_v) ** 2 / self.a_max)
        h_dot = min_ahead_v - (0.5 / self.a_max) * 2 * (min_ahead_v - ego_v) * 2
    else:
        h = (min_ahead_s - ego_s - self.L - (1 + self.fsmpra) * ego_v)
        h_dot = min_ahead_v

    if h >= 0:
        self.ego_command = FSM_state

    else:
        self.ego_command = 0

    if h_dot <= 0.1 * h:
        self.a_ref = -self.a_demax
    elif 0.1 * h < h_dot <= 0.2 * h:
        self.a_ref = -self.a_demax / 2
    else:
        self.a_ref = 0
    self.hfsm = -(h - min_ahead_s)
    self.h_dot = h_dot
    self.ego_target_id = self.ego_id + self.ego_command


def FSM(self, ego_left_car_s, ego_right_car_s, ego_ahead_car_s, ego_s, ego_left_ahead_right_car_v):
    # return self.ego_command
    FSM_state = 0
    min_ahead_s = min(ego_ahead_car_s)

    if min(ego_left_car_s) > min(ego_right_car_s) > ego_s and min(
            ego_left_car_s) > ego_s and self.ego_id != self.n_road-1:  # turn left
        FSM_state = 1
        min_ahead_s = min(ego_left_car_s)
        # self.ego_command = 1
        # self.ego_target_id = self.ego_id + 1

    elif min(ego_right_car_s) > min(ego_left_car_s) > ego_s and \
            min(ego_right_car_s) > ego_s and self.ego_id != 0:  # turn right
        FSM_state = -1
        min_ahead_s = min(ego_right_car_s)

        # self.ego_command = -1
        # self.ego_target_id = self.ego_id - 1

    elif self.ego_id == 0 and ego_s < min(ego_ahead_car_s) < ego_s + 40:  # 有的时候还在向左转 warning还没解除
        FSM_state = 1
        min_ahead_s = min(ego_left_car_s)
        # self.ego_command = 1
        # self.ego_target_id = self.ego_id + 1

    elif self.ego_id == self.n_road - 1 and ego_s < min(ego_ahead_car_s) < ego_s + 40:
        FSM_state = -1
        min_ahead_s = min(ego_right_car_s)
        # self.ego_command = -1
        # self.ego_target_id = self.ego_id - 1

    else:
        FSM_state = 0

    FSM_CBF_SEFTY(self, ego_s, min_ahead_s, FSM_state, ego_left_ahead_right_car_v)
    # self.ego_command = FSM_state
    # self.ego_target_id = self.ego_id + self.ego_command
