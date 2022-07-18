
import copy
import os
import platform
import sys

import matplotlib.patches as patch
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

if platform.system() == 'Darwin':
    sys.path.append(os.path.abspath(os.path.join(os.getcwd(), "..")))
else:
    sys.path.append('.')

from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import \
    fot_wrapper


class FOT():
    def __init__(self, obs, goal, dt=0.1):

        self.goal = goal
        self.goal_reached = 0
        self.result_x = []
        self.result_y = []

        self.hyperparameters = {
            "max_speed": 50.0,
            "max_accel": 15.0,
            "max_curvature": 20.0,
            "max_road_width_l": 5.0,
            "max_road_width_r": 5.0,
            "max_steering": 2,
            "d_road_w": 0.5,
            "dt": dt,
            "maxt": 2.0,
            "mint": 0.5,
            "d_t_s": 0.3,
            "n_s_sample": 5.0,
            "obstacle_clearance": 0.1,
            "kd": 2,
            "kv": 0.1,
            "ka": 0.1,
            "kj": 0.1,
            "kt": 0.1,
            "ko": 0.1,
            "klat": 1.0,
            "klon": 1.0,
            "num_threads": 0,  # set 0 to avoid using threaded algorithm
            "wp": np.concatenate(([[obs[0, 0], obs[0, 1]], ],
                                  [[np.mean(self.goal[0, :]), np.mean(self.goal[1, :])], ]), axis=0)
        }

    def prepare_data(self, raw_data):

        row = np.size(raw_data, 0) - 1
        obs_center = np.concatenate(
            (raw_data[1:, 0:2], raw_data[1:, 0:2]), axis=1)
        obs_size = np.c_[
            -0.5*raw_data[1:, 4],
            -0.5*raw_data[1:, 5],
            0.5*raw_data[1:, 4],
            0.5*raw_data[1:, 5]]
        obs_cornerpoint = obs_center + obs_size

        heading_agnle = raw_data[0, 3]
        vx = raw_data[0, 2] * np.cos(heading_agnle)
        vy = raw_data[0, 2] * np.sin(heading_agnle)

        self.data4fot = {
            's0': 0,
            'target_speed': 40,
            'wp': self.hyperparameters['wp'],
            'obs': obs_cornerpoint,
            'pos': raw_data[0, 0:2],
            'vel': [vx, vy],
            'heading': heading_agnle,
            'vel_l': raw_data[0, 4],
        }

    def planning(self):

        conds = self.data4fot

        self.initial_conditions = {
            'ps': conds['s0'],
            'target_speed': conds['target_speed'],
            'pos': conds['pos'],
            'vel': np.array(conds['vel']),
            'wp': conds['wp'],
            'obs': np.array(conds['obs'])
        }

        self.result_x, self.result_y, self.speeds, ix, iy, iyaw, d, s, self.speeds_x, \
            self.speeds_y, misc, costs, success = \
            fot_wrapper.run_fot(self.initial_conditions, self.hyperparameters)

        if success:
            self.initial_conditions['ps'] = misc['s']
            self.plan_collection = [copy.deepcopy(self.result_x), copy.deepcopy(
                self.result_y), copy.deepcopy(self.speeds), copy.deepcopy(self.speeds_x), copy.deepcopy(self.speeds_y)]
        # self.result_x = self.plan_collection[0][1:]
        # self.result_y = self.plan_collection[1][1:]
        # self.speeds = self.plan_collection[2][1:]

        # acc = (self.speeds[1] - np.linalg.norm(self.initial_conditions['vel']))/self.hyperparameters['dt']
        return success

    def generate_control(self, flag=False):

        if flag == True:
            self.result_x = self.plan_collection[0][1:]
            self.result_y = self.plan_collection[1][1:]
            self.speeds = self.plan_collection[2][1:]
            self.speeds_x = self.plan_collection[3][1:]
            self.speeds_y = self.plan_collection[4][1:]

            self.plan_collection[0] = self.plan_collection[0][1:]
            self.plan_collection[1] = self.plan_collection[1][1:]
            self.plan_collection[2] = self.plan_collection[2][1:]
            self.plan_collection[3] = self.plan_collection[3][1:]
            self.plan_collection[4] = self.plan_collection[4][1:]
        
        # break if near goal
        if self.goal_reached:
            return 0,0
        if np.hypot(self.result_x[1] - self.hyperparameters['wp'][-1, 0], self.result_y[1] - self.hyperparameters['wp'][-1, 1]) <= 10.0:
            print("Goal")
            self.goal_reached = 1
            return 0,0 

        init_state = {
            'px': self.initial_conditions['pos'][0],
            'py': self.initial_conditions['pos'][1],
            'v': np.linalg.norm(self.initial_conditions['vel']),
            'heading': self.data4fot['heading'],
        }

        target_state = {
            'px': self.result_x[1],
            'py': self.result_y[1],
            'vx': self.speeds_x[1],
            'vy': self.speeds_y[1],
        }

        vel_len = self.data4fot['vel_l']
        dt = self.hyperparameters['dt']

        x0_vals = np.array([init_state['px'], init_state['py'],
                           init_state['v'], init_state['heading']])
        x1_vals = np.array(
            [target_state['px'], target_state['py'], target_state['vx'], target_state['vy']])
        u0 = np.array([0, 0])
        bnds = ((-self.hyperparameters['max_accel'], self.hyperparameters['max_accel']),
                (-self.hyperparameters['max_steering'], self.hyperparameters['max_steering']))

        # Minimize difference between simulated state and next state by varying input u
        u0 = minimize(position_orientation_objective, u0, args=(x0_vals, x1_vals, vel_len, dt),
                      options={'disp': False, 'maxiter': 100, 'ftol': 1e-9},
                      method='SLSQP', bounds=bnds).x

        # Get simulated state using the found inputs

        x1_sim_array = vehicle_dynamic(init_state, u0, vel_len, dt)
        x1_sim = np.array([x1_sim_array['px'], x1_sim_array['py'],
                           x1_sim_array['v']*np.cos(x1_sim_array['heading']),
                           x1_sim_array['v']*np.sin(x1_sim_array['heading'])])

        x_delta = np.linalg.norm(x1_sim-x1_vals)
        # print('error:', x_delta)
        # x1_sim = vehicle_dynamics.forward_simulation(x0_vals, u0, dt, throw=False)

        # print('action:', u0)
        return u0

    def show_frame(self):
        # if show_animation:  # pragma: no cover
        area = 40
        wx = self.initial_conditions['wp'][:, 0]
        wy = self.initial_conditions['wp'][:, 1]
        obs = self.data4fot['obs']

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None])
        plt.plot(wx, wy)
        if obs.shape[0] == 0:
            obs = np.empty((0, 4))
        ax = plt.gca()
        for o in obs:
            rect = patch.Rectangle((o[0], o[1]), o[2] - o[0], o[3] - o[1])
            ax.add_patch(rect)

        goal_region = patch.Rectangle((self.goal[0, 0], self.goal[1, 0]),
                                      self.goal[0, 1] - self.goal[0, 0],
                                      self.goal[1, 1] - self.goal[1, 0])
        ax.add_patch(goal_region)

        if np.size(self.result_x, 0) > 1:
            plt.plot(self.result_x[1:], self.result_y[1:], "-or")
            plt.plot(self.result_x[1], self.result_y[1], "vc")
            plt.xlim(self.result_x[1] - area, self.result_x[1] + area)
            plt.ylim(self.result_y[1] - area, self.result_y[1] + area)
        else:
            plt.xlim(self.initial_conditions['pos'][0] -
                     area, self.initial_conditions['pos'][0] + area)
            plt.ylim(self.initial_conditions['pos'][1] -
                     area, self.initial_conditions['pos'][1] + area)

        plt.xlabel("X axis")
        plt.ylabel("Y axis")
        plt.title("v[m/s]:" +
                  str(np.linalg.norm(self.initial_conditions['vel']))[0:4])
        plt.grid(True)
        plt.pause(0.1)

        return 0


def vehicle_dynamic(state, action, vel_len, dt):
    init_state = state
    a, rot = action

    final_state = {
        'px': 0,
        'py': 0,
        'v': 0,
        'heading': 0
    }
    # 首先根据旧速度更新本车位置
    # 更新本车转向角
    final_state['heading'] = init_state['heading'] + \
        init_state['v'] / vel_len * np.tan(rot) * dt

    # 更新本车速度
    final_state['v'] = init_state['v'] + a * dt

    # 更新X坐标
    final_state['px'] = init_state['px'] + init_state['v'] * \
        dt * np.cos(init_state['heading'])  # *np.pi/180

    # 更新Y坐标
    final_state['py'] = init_state['py'] + init_state['v'] * \
        dt * np.sin(init_state['heading'])  # *np.pi/180

    return final_state


def position_orientation_objective(u: np.array, x0_array: np.array, x1_array: np.array, vel_l: float, dt: float, e: np.array = np.array([2e-3, 2e-3, 3e-3])) -> float:
    """
    Position-Orientation objective function to be minimized for the state transition feasibility.

    Simulates the next state using the inputs and calculates the norm of the difference between the
    simulated next state and actual next state. Position, velocity and orientation state fields will
    be used for calculation of the norm.

    :param u: input values
    :param x0: initial state values
    :param x1: next state values
    :param dt: delta time
    :param vehicle_dynamics: the vehicle dynamics model to be used for forward simulation
    :param ftol: ftol parameter used by the optimizer
    :param e: error margin, function will return norm of the error vector multiplied with 100 as cost
        if the input violates the friction circle constraint or input bounds.
    :return: cost
    """
    x0 = {
        'px': x0_array[0],
        'py': x0_array[1],
        'v': x0_array[2],
        'heading': x0_array[3],
    }

    x1_target = x1_array
    x1_sim = vehicle_dynamic(x0, u, vel_l, dt)
    x1_sim_array = np.array([x1_sim['px'], x1_sim['py'], x1_sim['v'] *
                            np.cos(x1_sim['heading']), x1_sim['v']*np.sin(x1_sim['heading'])])

    # if the input violates the constraints
    if x1_sim is None:
        return np.linalg.norm(e * 100)

    else:
        diff = np.subtract(x1_target, x1_sim_array)
        cost = np.linalg.norm(diff)
        return cost
