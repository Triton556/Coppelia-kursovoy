import csv
import sys

import sim
from math import cos, sin, sqrt, fabs, atan2, floor,radians
import time
import threading
import path
import numpy as np


def PIDregKurs(h, e_last, e, Ui, maxU=3.0):
    P = 1.5
    I = 0.00004
    D = 0.03
    Up = P * e
    Ui = Ui + I * e * h

    Ud = D * (e - e_last) / h

    U = Up + Ui + Ud

    if U > maxU:

        U = maxU
        Ui = Ui - I * e * h

    elif U < -maxU:
        U = -maxU
        Ui = Ui - I * e * h

    return U, Ui


def PIDregVel(h, e_last, e, Ui, maxU=11.0):
    P = 4  # *10
    I = 0.001 * 100
    D = 1000 * 500
    Up = P * e
    Ui = Ui + I * e * h
    Ud = D * (e - e_last) / h

    U = Up + Ui + Ud

    if U > maxU:
        U = maxU
        Ui = Ui - I * e * h

    elif U < -maxU:
        U = -maxU
        Ui = Ui - I * e * h

    return U, Ui


def PIDregAngVel(h, e_last, e, Ui, maxU=5.0):
    P = 0.3  # *10
    I = 0 * 500
    D = 50 * 1000
    Up = P * e
    Ui = Ui + I * e * h
    Ud = D * (e - e_last) / h

    U = Up + Ui + Ud

    if U > maxU:
        U = maxU
        Ui = Ui - I * e * h

    elif U < -maxU:
        U = -maxU
        Ui = Ui - I * e * h

    return U, Ui


def PIDregHeight(h, e_last, e, Ui, maxU=8.0):
    P = 8  # *10
    I = 0.05
    D = 0.5
    Up = P * e
    Ui = Ui + I * e * h
    Ud = D * (e - e_last) / h

    U = Up + Ui + Ud

    if U > maxU:
        U = maxU
        Ui = Ui - I * e * h

    elif U < -maxU:
        U = -maxU
        Ui = Ui - I * e * h

    return U, Ui


def Vel_rotate(a, b, g, vx, vy, vz):
    va_x = vx * (cos(b) * cos(g) - sin(a) * sin(b) * sin(g)) - vy * (cos(a) * sin(g)) + vz * (
            cos(g) * sin(b) + cos(b) * sin(a) * sin(g))
    va_y = vx * (cos(b) * sin(g) + cos(g) * sin(a) * sin(b)) + vy * (cos(a) * cos(g)) + vz * (
            sin(b) * sin(g) - cos(b) * cos(g) * sin(a))
    va_z = -vx * (cos(a) * sin(b)) + vy * (sin(a)) + vz * (cos(a) * cos(b))

    return [va_x, va_y, va_z]


def integrate(sig, last_sig, dt):
    integ = ((sig + last_sig) / 2) * dt

    # integ = sig * dt

    return integ


def set_pos():
    global target_pos
    while True:
        target_pos_string = input('pos: ')
        target_pos_arr = target_pos_string.split(' ')
        for k in range(len(target_pos)):
            target_pos[k] = float(target_pos_arr[k])


sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 15000, 5)
if clientID != -1:
    print('Connected to remote API server')

res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

res, robot = sim.simxGetObjectHandle(clientID, 'robot', sim.simx_opmode_oneshot_wait)
# res, target = sim.simxGetObjectHandle(clientID, 'target_object', sim.simx_opmode_oneshot)

simStarted = False

last_time = 0.0
last_gamma_err = 0.0
last_dist = 0.0
last_velocity_err = 0.0
wi_move = 0.0
wi_kurs = 0.0
ang_i_vel_z = 0.0
last_ang_vel_z = 0.0
ang_vel_z = 0.0
ang_vel_x = 0.0
ang_vel_y = 0.0
alpha = 0
beta = 0
gamma = 0

scanned_points = []

target_height = 2
last_height_err = 0.0

target_velocity = 0.8
point_num = 0
start = [-5, -5]
end = [5, 5]

route = path.create_path(start, end, start, target_height)
route = np.asarray(route)

target_pos = route[point_num]
sleep_time = 0.5
sleep = 0

integrated_pos = [0.0, 0.0, 0.0]
last_velocity = [0.0, 0.0, 0.0]
new_velocity = [0.0, 0.0, 0.0]
ock_velocity = [0.0, 0.0, 0.0]
angles = [0.0, 0.0, 0.0]
target_relative_pos = [0, 0, 0]
dt = 0.0000001
height_err = 0

lidar_end_time = 0
lidar_check_time = 0.2

velocity_err = 0.0

threading.Thread(target=set_pos, daemon=True).start()

with open('points.csv', 'w', newline='') as csvfile:
    cloud_writer = csv.writer(csvfile)
    cloud_writer.writerow(['x', 'y', 'z'])
time.sleep(1)
while True:

    start_time = time.time()

    # print(dt)

    res = sim.simxSetFloatSignal(clientID, 'target_velocity', target_velocity, sim.simx_opmode_oneshot)

    res, robotPos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)

    res, target_handle = sim.simxGetIntegerSignal(clientID, 'target_handle', sim.simx_opmode_oneshot)

    res, velocity_string = sim.simxGetStringSignal(clientID, 'velocity', sim.simx_opmode_oneshot)

    res, ang_velocity_string = sim.simxGetStringSignal(clientID, 'ang_velocity', sim.simx_opmode_oneshot)

    res, angles_string = sim.simxGetStringSignal(clientID, 'angles', sim.simx_opmode_oneshot)

    res, curr_height = sim.simxGetFloatSignal(clientID, 'height', sim.simx_opmode_oneshot)



    height_err = target_height - curr_height

    if start_time - lidar_end_time > lidar_check_time:

        res, lidar_points_string = sim.simxGetStringSignal(clientID, 'lidar_points', sim.simx_opmode_oneshot)

        if lidar_points_string != []:
            lidar_points = sim.simxUnpackFloats(lidar_points_string)
            # print(lidar_points)

            for i in range(0, len(lidar_points), 3):
                point = [0, 0, 0]
                point_in_lidar = [lidar_points[i], lidar_points[i + 1], lidar_points[i + 2]]
                point_in_glob = Vel_rotate(alpha-radians(90), beta-radians(90), gamma-radians(90), point_in_lidar[0], point_in_lidar[1], point_in_lidar[2])

                with open('points.csv', 'a', newline='') as csvfile:
                    cloud_writer = csv.writer(csvfile)
                    cloud_writer.writerow([point_in_glob[0] + integrated_pos[0], point_in_glob[1] + integrated_pos[1],
                                           point_in_glob[2] - height_err + integrated_pos[2]])
                # cloud_writer.writerow(scanned_points[i])
                # print(f'i: {i} pt: {scanned_points[i]}')

        lidar_end_time = time.time()

    target_pos = route[point_num]

    if target_handle != 0:
        # res = sim.simxSetObjectPosition(clientID, target_handle, -1, target_pos, sim.simx_opmode_oneshot)

        res, target_relative_pos = sim.simxGetObjectPosition(clientID, target_handle, robot, sim.simx_opmode_oneshot)

        dist = sqrt((target_relative_pos[0]) ** 2 + (target_relative_pos[1]) ** 2)

        if dist < 0.3:

            if (point_num < len(route[:, 0]) - 1) and simStarted and start_time - sleep >= sleep_time:
                point_num += 1

                target_pos = route[point_num]
                set_point = [target_pos[0],target_pos[1],1]


                res = sim.simxSetObjectPosition(clientID, target_handle, -1, set_point,
                                                sim.simx_opmode_oneshot)

                res, target_relative_pos = sim.simxGetObjectPosition(clientID, target_handle, robot,
                                                                     sim.simx_opmode_oneshot)
                dist = sqrt((target_relative_pos[0]) ** 2 + (target_relative_pos[1]) ** 2)

                sleep = time.time()
            elif (point_num >= len(route[:, 0]) - 1):
                sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
                sys.exit()

    velocity = sim.simxUnpackFloats(velocity_string)

    ang_velocity = sim.simxUnpackFloats(ang_velocity_string)

    angles = sim.simxUnpackFloats(angles_string)

    if angles != []:
        simStarted = True
        alpha = angles[0]
        beta = angles[1]
        gamma = angles[2]

    if ang_velocity != []:
        ang_vel_x = ang_velocity[0]
        ang_vel_y = ang_velocity[1]
        ang_vel_z = ang_velocity[2]

    if (velocity != []):
        ock_velocity = Vel_rotate(alpha, beta, gamma, velocity[0], velocity[1], velocity[2])

        new_velocity = ock_velocity
        velocity_err = target_velocity - sqrt(ock_velocity[0] ** 2 + ock_velocity[1] ** 2 + ock_velocity[2] ** 2)

    for i in range(len(last_velocity)):
        integrated_pos[i] += integrate(new_velocity[i], last_velocity[i], dt)

    #integrated_pos[2] = curr_height
    # print(f'pos {integrated_pos}')

    last_velocity = new_velocity

    gamma_err = atan2(target_relative_pos[1], target_relative_pos[0])

    dist = sqrt((target_relative_pos[0]) ** 2 + (target_relative_pos[1]) ** 2)

    w_kurs, wi_kurs = PIDregKurs(dt, last_gamma_err, gamma_err, wi_kurs)

    w_move, wi_move = PIDregVel(dt, last_velocity_err, velocity_err, wi_move)

    ang_corr_z, ang_i_vel_z = PIDregAngVel(dt, last_ang_vel_z, ang_vel_z, ang_i_vel_z)

    w_move_h = w_move

    if dist < 0.8:
        ang_corr_z = 0

    if dist < target_velocity:
        w_move /= 2

    if dist < 1:
        w_move = dist * 5

    if simStarted:
        if dist < 0.25:
            w_move = 0
            w_kurs = 0

    # print(f'\rdt = {dt} Err_vel = {velocity_err} Err_h = {height_err}', end='')

    # beta_cor = beta/3
    # alpha_cor = alpha/3

    if alpha > radians(30) or alpha < radians(-30) or ang_vel_x > 2 or ang_vel_y > 2 or beta > radians(30) or beta < radians(-30):
        w_move = 0
        ang_corr_z = 0
        w_kurs = 0
        w_move_h = 0

    # w_move = 0
    # ang_corr_z = 0

    w1 = (w_kurs - w_move - ang_corr_z) * fabs(w_kurs - w_move - ang_corr_z)
    w2 = (w_kurs + w_move - ang_corr_z) * fabs(w_kurs + w_move - ang_corr_z)
    w3 = (w_kurs + w_move - ang_corr_z) * fabs(w_kurs + w_move - ang_corr_z)
    w4 = (w_kurs - w_move - ang_corr_z) * fabs(w_kurs - w_move - ang_corr_z)



    height_w, height_wi = PIDregHeight(dt, last_height_err, height_err, 0)

    w5 = (height_w + beta / 2 - alpha / 2) * fabs(height_w + beta / 2 - alpha / 2)
    w6 = (height_w + beta / 2 + alpha / 2) * fabs(height_w + beta / 2 + alpha / 2)
    w7 = (height_w - beta / 2 - alpha / 2) * fabs(height_w - beta / 2 - alpha / 2)
    w8 = (height_w - beta / 2 + alpha / 2) * fabs(height_w - beta / 2 + alpha / 2)

    res = sim.simxSetFloatSignal(clientID, 'w1', w1, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w2', w2, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w3', w3, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w4', w4, sim.simx_opmode_oneshot)

    res = sim.simxSetFloatSignal(clientID, 'w5', w5, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w6', w6, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w7', w7, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w8', w8, sim.simx_opmode_oneshot)

    # print(f'\rdt = {dt} Err_vel = {velocity_err} move_h = {height_w} a = {alpha/3}', end='')

    print(f'\rpos = {integrated_pos} | {height_w}', end='')

    # print(f'\rg_err {gamma_err} w_kurs {w_kurs} dist{dist}',end='')

    last_gamma_err = gamma_err
    last_velocity_err = velocity_err
    last_ang_vel_z = ang_vel_z
    last_height_err = height_err

    end_time = time.time()
    dt = end_time - start_time

    if dt == 0:
        dt = 0.00000001
