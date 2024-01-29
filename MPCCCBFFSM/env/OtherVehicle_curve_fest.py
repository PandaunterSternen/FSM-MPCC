import numpy as np
import random


class OtherVehicleCurve_fest:
    def __init__(self, num_cars, sim_step, T, v, n_road, N, road_params):
        self.num_cars = num_cars
        self.sim_step = sim_step
        self.T = T
        self.v_ave = v
        self.N = N
        self.cars = {}
        self.cars_all_states = {}
        self.cars_N1_states = {}
        self.a = 0.05
        self.road_params = road_params

        for car_id in range(num_cars):
            all_init_ID = [ 0,2,1,0,1,0,2,1,2,1,0,1,2,1,0]
            random_ID = all_init_ID[car_id]
            self.cars[car_id] = {
                'States_X': [],
                'States_Y': [],
                'States_v': [],
                'States_psi': [],
                'States_s': [],
                'road_ID': random_ID
            }
            self.cars_N1_states[car_id] = {
                'States_X': [],
                'States_Y': [],
                'States_v': [],
                'States_psi': [],
                'States_s': [],
                'road_ID': random_ID 
            }
        self.set_state()

    def constantspeed(self, v):
        v_state = [v] * (self.sim_step + 1)
        return v_state

    def varyspeed(self, v):
        v_state = []
        init_v = v
        if random.randint(1, 10) >= 8:
            for t in range(self.sim_step + 1):
                if 1.05*init_v > v > 0.05*init_v:
                    v = v - self.a * (t * self.T)
                    v_state.append(v)
                else:
                    v_state.append(v)
        else:
            for t in range(self.sim_step + 1):
                if 1.1*init_v > v > 0.9*init_v:
                    v = v - self.a * (t * self.T)
                    v_state.append(v)
                else:
                    v_state.append(v)

        return v_state

    def set_state(self):
        v = self.v_ave
        all_init_index = [20,40,100,140,190,240,270,330,380,400,480]
        other_cars_init_index = all_init_index[:self.num_cars]


        for car_id in range(self.num_cars):
            other_cars_index = []
            road_number = int(self.cars[car_id]['road_ID'])
            car_this_road = self.road_params[road_number]['Center_line']
            index_init = int(other_cars_init_index[car_id])
            other_cars_index.append(index_init)

            v = (self.cars[car_id]['road_ID'] - 2) * 0.1 * v + v
            x_line = car_this_road['x']
            y_line = car_this_road['y']
            psi_line = car_this_road['psi']
            s_line = car_this_road['s']
            s = car_this_road['s'][other_cars_init_index[car_id]]

            if random.randint(1, 10) >= 0:
                state_v = self.constantspeed(v)
            else:
                state_v = self.varyspeed(v)
            other_cars_index = []
            state_x = np.zeros((self.sim_step, 1))
            state_y = np.zeros((self.sim_step, 1))
            state_psi = np.zeros((self.sim_step, 1))
            state_s = np.zeros((self.sim_step, 1))
            for i in range(self.sim_step):
                index = self.s_to_index(s, self.cars[car_id]['road_ID'])
                if index is not None:
                    other_cars_index.append(index)
                    state_x[i] = x_line[index]
                    state_y[i] = y_line[index]
                    state_psi[i] = psi_line[index]
                    state_s[i] = s_line[index]
                    s = s + state_v[i] * self.T
                else:
                    state_x[i] = state_x[i - 1]
                    state_y[i] = state_y[i - 1]
                    state_psi[i] = state_psi[i - 1]
                    state_s[i] = state_s[i - 1]
                    s = s + state_v[i] * self.T

            self.cars[car_id]['States_X'] = state_x
            self.cars[car_id]['States_Y'] = state_y
            self.cars[car_id]['States_psi'] = state_psi
            self.cars[car_id]['States_v'] = state_v
            self.cars[car_id]['States_s'] = state_s

        else:
            print(f"Car {car_id} does not exist.")

    def get_all_states(self):

        self.cars_all_states = self.cars
        return self.cars_all_states

    def s_to_index(self, s, road_id):
        s_diff = abs(self.road_params[road_id]['Center_line']['s'] - s)
        index_in_curve = np.argmin(s_diff)
        return index_in_curve

    def get_variations_states(self, mpciter):

        for car_id in range(self.num_cars):
            self.cars_N1_states[car_id]['States_X'] = self.cars[car_id]['States_X'][
                                                      mpciter: mpciter + self.N]
            self.cars_N1_states[car_id]['States_Y'] = self.cars[car_id]['States_Y'][
                                                      mpciter: mpciter + self.N]
            self.cars_N1_states[car_id]['States_psi'] = self.cars[car_id]['States_psi'][
                                                      mpciter: mpciter + self.N]
            self.cars_N1_states[car_id]['States_s'] = self.cars[car_id]['States_s'][
                                                        mpciter: mpciter + self.N]
            self.cars_N1_states[car_id]['States_v'] = self.cars[car_id]['States_v'][
                                                        mpciter: mpciter + self.N]

        return self.cars_N1_states

