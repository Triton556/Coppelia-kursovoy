import numpy as np


def Find_spline(x_arr, y_arr, step_size = 0.01):
    dotAmount = len(x_arr)
    splineAmount = dotAmount - 1
    ermVectScale = 1.7  # scale factor of Ermits vectors
    rx = [(x_arr[i + 1] - x_arr[i - 1]) / ermVectScale for i in range(1, dotAmount - 1)]  # Ermits vectors for x_arr
    rx.insert(0, 0)
    rx.append(0)
    ry = [(y_arr[i + 1] - y_arr[i - 1]) / ermVectScale for i in range(1, dotAmount - 1)]  # Ermits vectors for y_arr
    ry.insert(0, 0)
    ry.append(0)


    newX = np.zeros(int(splineAmount / step_size))
    newY = np.zeros(int(splineAmount / step_size))
    l = 0

    spline_points = []
    for i in range(splineAmount):
        k = 0
        for j in range(int(1 / step_size)):
            newX[l] = x_arr[i] * (2 * k ** 3 - 3 * k ** 2 + 1) + x_arr[i + 1] * (-2 * k ** 3 + 3 * k ** 2) + rx[i] * (
                    k ** 3 - 2 * k ** 2 + k) + rx[i + 1] * (k ** 3 - 1 * k ** 2)
            newY[l] = y_arr[i] * (2 * k ** 3 - 3 * k ** 2 + 1) + y_arr[i + 1] * (-2 * k ** 3 + 3 * k ** 2) + ry[i] * (
                    k ** 3 - 2 * k ** 2 + k) + ry[i + 1] * (k ** 3 - 1 * k ** 2)
            spline_points.append([newX[l], newY[l]])
            k += step_size
            l += 1
    return spline_points