#!/usr/bin/python


import numpy as np
from scipy import interpolate
import time


def interpolate_my(x_queue, x, y, interpolate_kind):
# if interpolate_kind == 'traj'
# interpolate the desired trajectory by time value
# x_queue is the interpolated point/points
# if x_queue is 1-D numpy array, then the output is 2-D numpy array
# if x_queue is a float, then the output is 2-D numpy array, but only has one column
# x is 1-D numpy array (1 by n), y is 2-D numpy array (m by n)
# default is linear interpolation
# when x_queue exceeds the range of x, function returns the boundary value of y

# if interpolate_kind == 'cmd'
# interpolate the commands by the machine time by Zero-Order Hold
# x_queue is the interpolated machine time
# assume x_queue is always 1-D numpy array, and the output is 2-D numpy array
# time before the first timestamp, commands are zeros
# time after the last timestamp, commands are the last ones.

    if interpolate_kind == 'traj':
        boundary = (y[:, 0], y[:, -1])
        f = interpolate.interp1d(x, y, kind='linear', bounds_error=False, fill_value=boundary)
        y_raw = f(x_queue)
        if isinstance(x_queue, float):
            y_queue = y_raw.reshape(y_raw.shape[0], -1)
        else:
            y_queue = y_raw

    elif interpolate_kind == 'cmd':
        boundary = (np.zeros(4), y[:, -1])
        f = interpolate.interp1d(x, y, kind='zero', bounds_error=False, fill_value=boundary)
        y_queue = f(x_queue)

    else:
        print("The interpolation type is wrong!")
        y_queue = []

    return y_queue


if __name__ == '__main__':
    t_start = 0.0
    t_end = 6.0
    dt_planner = 0.01

    t_traj_known = np.linspace(t_start, t_end, (t_end-t_start)/dt_planner+1)

    a = np.linspace(0.0, 180.0, t_traj_known.shape[0])
    print(a)

    # a = np.zeros((1, 6))

    # b = np.array([[0.0, 1.0, 2.0, 3.0, 4.0, 5.0]])

    # q_traj_known = np.concatenate((b,a,a,a,a,a,a), axis=0)



    # t_queue = np.linspace(0.0, 5.0, 5/0.2+1)



    # print(interpolate_my(t_queue, t_traj_known, q_traj_known, interpolate_kind='traj'))