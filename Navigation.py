import socket
import threading
import Spline
import sim
from math import cos, sin, sqrt, fabs, atan2, degrees, radians
import time
import path
import numpy as np
import os

UDP_MAX_SIZE = 65535

mapping_manager = ('127.0.0.1', 3001)
interface_manager = ('127.0.0.1', 3002)


def connect(host: str = '127.0.0.1', port: int = 3003):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))

    msg = "Navigation connected!".encode()
    send_to_interface(s, msg)

    threading.Thread(target=listen, args=(s,), daemon=True).start()

    pathToDirectory = os.path.dirname(os.path.realpath(__file__))

    os.startfile(fr'{pathToDirectory}\Mapping.py')

    time.sleep(2)


    threading.Thread(target=work_with_coppelia, args=(1,'[[0.0, 0.0], [-5.0, -15.0], [-5.0, 5.0], [-4.0, 5.0], [-4.0, -15.0], [-3.0, -15.0], [-3.0, '
                       '5.0], [-2.0, 5.0], [-2.0, -15.0], [-1.0, -15.0], [-1.0, 5.0], [-1.1102230246251565e-16, 5.0], '
                       '[-1.1102230246251565e-16, -15.0], [0.9999999999999998, -15.0], [0.9999999999999998, 5.0], '
                       '[1.9999999999999996, 5.0], [1.9999999999999996, -15.0], [2.9999999999999996, -15.0], '
                       '[2.9999999999999996, 5.0], [3.9999999999999996, 5.0], [3.9999999999999996, -15.0], '
                       '[4.999999999999999, -15.0], [4.999999999999999, 5.0], [5.999999999999999, 5.0], '
                       '[5.999999999999999, -15.0], [6.999999999999999, -15.0], [6.999999999999999, 5.0], '
                       '[7.999999999999999, 5.0], [7.999999999999999, -15.0], [8.999999999999998, -15.0], '
                       '[8.999999999999998, 5.0], [9.999999999999998, 5.0], [9.999999999999998, -15.0], '
                       '[10.999999999999998, -15.0], [10.999999999999998, 5.0], [11.999999999999998, 5.0], '
                       '[11.999999999999998, -15.0], [12.999999999999998, -15.0], [12.999999999999998, 5.0], '
                       '[13.999999999999998, 5.0], [13.999999999999998, -15.0], [14.999999999999998, -15.0], '
                       '[14.999999999999998, 5.0]]',
                       s,), daemon=True).start()

    while True:
        input('working...')




def listen(s: socket.socket):
    while True:
        try:
            msg = s.recv(UDP_MAX_SIZE)
            print(msg)
        except ConnectionResetError:
            print('no data')
            continue

        data = msg.decode().split('|')


def send_to_interface(s: socket.socket, msg):
    s.sendto(msg, interface_manager)


def send_to_mapping(s: socket.socket, msg):

    s.sendto(msg, mapping_manager)


def Vel_rotate(a, b, g, vx, vy, vz):
    va_x = vx * (cos(b) * cos(g) - sin(a) * sin(b) * sin(g)) - vy * (cos(a) * sin(g)) + vz * (
            cos(g) * sin(b) + cos(b) * sin(a) * sin(g))
    va_y = vx * (cos(b) * sin(g) + cos(g) * sin(a) * sin(b)) + vy * (cos(a) * cos(g)) + vz * (
            sin(b) * sin(g) - cos(b) * cos(g) * sin(a))
    va_z = -vx * (cos(a) * sin(b)) + vy * (sin(a)) + vz * (cos(a) * cos(b))
    return [va_x, va_y, va_z]


def work_with_coppelia(walk_mode: int, mission: str, s: socket.socket, target_velocity=0.8, target_height=2):
    simStarted = False
    target_handle = 0

    angles = [0, 0, 0]
    alpha = 0
    beta = 0
    gamma = 0
    gamma_err = 0
    last_gamma_err = 0

    ang_velocity = [0, 0, 0]
    ang_vel_x = 0
    ang_vel_y = 0
    ang_vel_z = 0
    velocity = [0, 0, 0]
    last_velocity = [0, 0, 0]
    new_velocity = [0, 0, 0]
    velocity_err = 0
    last_velocity_err = 0
    ock_velocity = [0, 0, 0]

    integrated_pos = [0, 0, 0]
    target_relative_pos = [0, 0, 0]
    dist = 0
    target_pos = [0, 0]

    curr_height = 0
    height_err = 0
    last_height_err = 0

    w_move = 0
    wi_move = 0
    w_kurs = 0
    wi_kurs = 0
    ang_corr_z = 0
    ang_i_vel_z = 0
    last_ang_vel_z = 0.0
    height_w = 0
    height_wi = 0
    side_force = 0

    dt = 0.0000001
    sleep_time = 0.1
    sleep = 0

    point_num = 0
    route = []

    def parce_route(mission):
        pointStr = str.strip(mission, '[]')
        points = pointStr.split('], [')
        route = []
        for i in range(len(points)):
            point_str_arr = points[i].split(', ')
            point = [float(point_str_arr[0]), float(point_str_arr[1])]
            route.append(point)
        route = np.asarray(route)
        return route

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

    def integrate(sig, last_sig, dt):
        integ = ((sig + last_sig) / 2) * dt

        return integ

    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 15000, 5)
    if clientID != -1:
        print('Connected to remote API server')

    res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print('sim started with res', res)
    res, robot = sim.simxGetObjectHandle(clientID, 'robot', sim.simx_opmode_oneshot_wait)

    route = parce_route(mission)

    time.sleep(1)
    while True:





        start_time = time.time()

        res, target_handle = sim.simxGetIntegerSignal(clientID, 'target_handle', sim.simx_opmode_oneshot)

        res, velocity_string = sim.simxGetStringSignal(clientID, 'velocity', sim.simx_opmode_oneshot)

        res, ang_velocity_string = sim.simxGetStringSignal(clientID, 'ang_velocity', sim.simx_opmode_oneshot)

        res, angles_string = sim.simxGetStringSignal(clientID, 'angles', sim.simx_opmode_oneshot)

        res, curr_height = sim.simxGetFloatSignal(clientID, 'height', sim.simx_opmode_oneshot)

        res, lidar_points_string = sim.simxGetStringSignal(clientID, 'lidar_points', sim.simx_opmode_oneshot)

        send_to_mapping(s,lidar_points_string)

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

        if target_handle != 0:

            res, target_relative_pos = sim.simxGetObjectPosition(clientID, target_handle, robot,
                                                                 sim.simx_opmode_oneshot)
            dist = sqrt((target_relative_pos[0]) ** 2 + (target_relative_pos[1]) ** 2)

            if walk_mode == 0 or 1:

                if dist < 0.3:

                    if (point_num < len(route[:, 0]) - 1) and simStarted and start_time - sleep >= sleep_time:
                        point_num += 1

                        target_pos = route[point_num]
                        set_point = [target_pos[0], target_pos[1], 1]

                        res = sim.simxSetObjectPosition(clientID, target_handle, -1, set_point,
                                                        sim.simx_opmode_oneshot)

                        res, target_relative_pos = sim.simxGetObjectPosition(clientID, target_handle, robot,
                                                                             sim.simx_opmode_oneshot)
                        dist = sqrt((target_relative_pos[0]) ** 2 + (target_relative_pos[1]) ** 2)

                        sleep = time.time()

        for i in range(len(last_velocity)):
            integrated_pos[i] += integrate(new_velocity[i], last_velocity[i], dt)

        last_velocity = new_velocity

        gamma_err = atan2(target_relative_pos[1], target_relative_pos[0])

        dist = sqrt((target_relative_pos[0]) ** 2 + (target_relative_pos[1]) ** 2)

        height_err = target_height - curr_height



        msg = f'nav|{alpha}|{beta}|{gamma}|{integrated_pos[0]}|{integrated_pos[1]}|{integrated_pos[2]}|{clientID}|{lidar_points_string}'.encode()

        send_to_mapping(s, msg)



        w_kurs, wi_kurs = PIDregKurs(dt, last_gamma_err, gamma_err, wi_kurs)

        w_move, wi_move = PIDregVel(dt, last_velocity_err, velocity_err, wi_move)

        ang_corr_z, ang_i_vel_z = PIDregAngVel(dt, last_ang_vel_z, ang_vel_z, ang_i_vel_z)

        height_w, height_wi = PIDregHeight(dt, last_height_err, height_err, 0)

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

        if walk_mode == 1:
            if 5 < degrees(gamma_err) < -5:
                w_move = 0
            if 5 > degrees(gamma_err) > -5:
                w_kurs = 0
                side_force = 1

        if alpha > radians(30) or alpha < radians(-30) or ang_vel_x > 2 or ang_vel_y > 2 or beta > radians(
                30) or beta < radians(-30):
            w_move = 0
            ang_corr_z = 0
            w_kurs = 0

        w1 = (w_kurs - w_move - ang_corr_z) * fabs(w_kurs - w_move - ang_corr_z)
        w2 = (w_kurs + w_move - ang_corr_z) * fabs(w_kurs + w_move - ang_corr_z)
        w3 = (w_kurs + w_move - ang_corr_z) * fabs(w_kurs + w_move - ang_corr_z)
        w4 = (w_kurs - w_move - ang_corr_z) * fabs(w_kurs - w_move - ang_corr_z)

        w5 = (height_w + beta / 2 - alpha / 2) * fabs(height_w + beta / 2 - alpha / 2)
        w6 = (height_w + beta / 2 + alpha / 2) * fabs(height_w + beta / 2 + alpha / 2)
        w7 = (height_w - beta / 2 - alpha / 2) * fabs(height_w - beta / 2 - alpha / 2)
        w8 = (height_w - beta / 2 + alpha / 2) * fabs(height_w - beta / 2 + alpha / 2)

        w9 = side_force
        w10 = (-side_force)

        res = sim.simxSetFloatSignal(clientID, 'w1', w1, sim.simx_opmode_oneshot)
        res = sim.simxSetFloatSignal(clientID, 'w2', w2, sim.simx_opmode_oneshot)
        res = sim.simxSetFloatSignal(clientID, 'w3', w3, sim.simx_opmode_oneshot)
        res = sim.simxSetFloatSignal(clientID, 'w4', w4, sim.simx_opmode_oneshot)

        res = sim.simxSetFloatSignal(clientID, 'w5', w5, sim.simx_opmode_oneshot)
        res = sim.simxSetFloatSignal(clientID, 'w6', w6, sim.simx_opmode_oneshot)
        res = sim.simxSetFloatSignal(clientID, 'w7', w7, sim.simx_opmode_oneshot)
        res = sim.simxSetFloatSignal(clientID, 'w8', w8, sim.simx_opmode_oneshot)

        res = sim.simxSetFloatSignal(clientID, 'w9', w9, sim.simx_opmode_oneshot)
        res = sim.simxSetFloatSignal(clientID, 'w10', w10, sim.simx_opmode_oneshot)

        print(f'\rN {point_num} target{target_pos} dist {dist}', end='')

        last_gamma_err = gamma_err
        last_velocity_err = velocity_err
        last_ang_vel_z = ang_vel_z
        last_height_err = height_err

        end_time = time.time()
        dt = end_time - start_time

        if dt == 0:
            dt = 0.00000001


if __name__ == "__main__":
    connect()
    #work_with_coppelia(1,
    #                   '[[0.0, 0.0], [-5.0, -15.0], [-5.0, 5.0], [-4.0, 5.0], [-4.0, -15.0], [-3.0, -15.0], [-3.0, 5.0], [-2.0, 5.0], [-2.0, -15.0], [-1.0, -15.0], [-1.0, 5.0], [-1.1102230246251565e-16, 5.0], [-1.1102230246251565e-16, -15.0], [0.9999999999999998, -15.0], [0.9999999999999998, 5.0], [1.9999999999999996, 5.0], [1.9999999999999996, -15.0], [2.9999999999999996, -15.0], [2.9999999999999996, 5.0], [3.9999999999999996, 5.0], [3.9999999999999996, -15.0], [4.999999999999999, -15.0], [4.999999999999999, 5.0], [5.999999999999999, 5.0], [5.999999999999999, -15.0], [6.999999999999999, -15.0], [6.999999999999999, 5.0], [7.999999999999999, 5.0], [7.999999999999999, -15.0], [8.999999999999998, -15.0], [8.999999999999998, 5.0], [9.999999999999998, 5.0], [9.999999999999998, -15.0], [10.999999999999998, -15.0], [10.999999999999998, 5.0], [11.999999999999998, 5.0], [11.999999999999998, -15.0], [12.999999999999998, -15.0], [12.999999999999998, 5.0], [13.999999999999998, 5.0], [13.999999999999998, -15.0], [14.999999999999998, -15.0], [14.999999999999998, 5.0]]',
    #                   s)
