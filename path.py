from math import tan, radians, ceil
import matplotlib.pyplot as plt
import numpy as np
import Spline


def create_path(start, end, curr_pos, scan_height):
    path = []
    x_arr = []
    y_arr = []
    w = 2 * scan_height * tan(radians(45)) * 0.5
    x_arr.append(curr_pos[0])
    x_arr.append(start[0])
    y_arr.append(curr_pos[1])
    y_arr.append(start[1])
    path.append(curr_pos)
    path.append(start)
    k = 0
    for i in range(0, ceil((end[0] - start[0]) / (w / 2))):

        k += 1

        if k <= 2:
            y = end[1]
        else:
            y = start[1]

        if k == 4:
            k = 0

        if i % 2 == 1:

            x = path[i + 1][0] + w
        else:

            x = path[i + 1][0]

        x_arr.append(x)
        y_arr.append(y)
        point = [x, y]
        path.append(point)
    return path


