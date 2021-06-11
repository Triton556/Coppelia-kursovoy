import sim
import math
import time


def PIDregKurs(h, e_last, e, Ui, maxU=3.0):
    P = 1.5  # 2.1
    I = 0.00004  # 0.00001
    D = 0.03  # 0.2
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


def PIDregVel(h, e_last, e, Ui, maxU=13.0):
    P = 0.5
    I = 0.001
    D = 1000
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
    P = 0.3
    I = 0
    D = 50
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

def PIDregHeight(h, e_last, e, Ui, maxU=1.0):
    P = 0.5
    I = 0
    D = 50
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

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 15000, 5)
if clientID != -1:
    print('Connected to remote API server')

res, robot = sim.simxGetObjectHandle(clientID, 'robot', sim.simx_opmode_oneshot_wait)

res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

t = 0.1
last_time = 0.0
last_gamma = 0.0
last_dist = 0.0
last_velocity_err = 0.0
wi_move = 0.0
wi_kurs = 0.0
ang_i_vel_z = 0.0
last_ang_vel_z = 0.0
ang_vel_z = 0.0
ang_vel_x = 0.0
ang_vel_y = 0.0


last_height_err = 0.0

target_velocity = 3
target_height = 2.0

while True:
    # last_time = t
    t = time.time()
    dt = t - last_time

    res = sim.simxSetFloatSignal(clientID, 'target_velocity', target_velocity, sim.simx_opmode_oneshot)

    res, robotPos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)

    res, gamma = sim.simxGetFloatSignal(clientID, 'gamma', sim.simx_opmode_oneshot)

    res, dist = sim.simxGetFloatSignal(clientID, 'dist', sim.simx_opmode_oneshot)

    res, velocity_err = sim.simxGetFloatSignal(clientID, 'velocity_err', sim.simx_opmode_oneshot)

    res, ang_vel_z = sim.simxGetFloatSignal(clientID, 'ang_vel_z', sim.simx_opmode_oneshot)

    res, ang_vel_x = sim.simxGetFloatSignal(clientID, 'ang_vel_x', sim.simx_opmode_oneshot)

    res, ang_vel_y = sim.simxGetFloatSignal(clientID, 'ang_vel_y', sim.simx_opmode_oneshot)

    res, height = sim.simxGetFloatSignal(clientID, 'height', sim.simx_opmode_oneshot)

    #res, robotPos_z = sim.simxGetFloatSignal(clientID, 'height', sim.simx_opmode_oneshot)

    res, alpha = sim.simxGetFloatSignal(clientID, 'alpha', sim.simx_opmode_oneshot)

    res, beta = sim.simxGetFloatSignal(clientID, 'beta', sim.simx_opmode_oneshot)

    if dt == 0:
        dt = 0.1

    w_kurs, wi_kurs = PIDregKurs(dt, last_gamma, gamma, wi_kurs)

    w_move, wi_move = PIDregVel(dt, last_velocity_err, velocity_err, wi_move)

    ang_corr_z, ang_i_vel_z = PIDregAngVel(dt, last_ang_vel_z, ang_vel_z, ang_i_vel_z)


    w_move_h = w_move

    if dist < 0.8:
        ang_corr_z = 0

    if dist < target_velocity / 2:
        w_move /= 4

    if dist < target_velocity / 4:
        w_move = dist * 3

    if dist < 0.1:
        w_move = 0
        w_kurs = 0

    #print(f'\rdt = {dt} Err_vel = {velocity_err} Err_h = {height_err}', end='')

    #beta_cor = beta/3
    #alpha_cor = alpha/3

    if alpha > 30 or alpha < -30 or ang_vel_x > 2 or ang_vel_y > 2 or beta > 30 or beta < -30:
        w_move = 0
        ang_corr_z = 0
        w_kurs = 0
        w_move_h = 0

    w1 = (w_kurs - w_move - ang_corr_z) * math.fabs(w_kurs - w_move - ang_corr_z)
    w2 = (w_kurs + w_move - ang_corr_z) * math.fabs(w_kurs + w_move - ang_corr_z)
    w3 = (w_kurs + w_move - ang_corr_z) * math.fabs(w_kurs + w_move - ang_corr_z)
    w4 = (w_kurs - w_move - ang_corr_z) * math.fabs(w_kurs - w_move - ang_corr_z)

    height_err = height - robotPos[2]


    #if beta < 0.5:
    #    beta = 0
    #if alpha < 0.5:
    #    alpha = 0



    #if height_err > 2:
    #    height_err = 2

    height_w, height_wi = PIDregHeight(dt, last_height_err, height_err, 0)

    #alpha, beta = 0, 0

    w5 = (height_w * w_move_h + beta/2 - alpha/2) * math.fabs(height_w * w_move_h + beta/2 - alpha/2)
    w6 = (height_w * w_move_h + beta/2 + alpha/2) * math.fabs(height_w * w_move_h + beta/2 + alpha/2)
    w7 = (height_w * w_move_h - beta/2 - alpha/2) * math.fabs(height_w * w_move_h - beta/2 - alpha/2)
    w8 = (height_w * w_move_h - beta/2 + alpha/2) * math.fabs(height_w * w_move_h - beta/2 + alpha/2)


    res = sim.simxSetFloatSignal(clientID, 'w1', w1, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w2', w2, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w3', w3, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w4', w4, sim.simx_opmode_oneshot)

    res = sim.simxSetFloatSignal(clientID, 'w5', w5, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w6', w6, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w7', w7, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w8', w8, sim.simx_opmode_oneshot)

    print(f'\rdt = {dt} Err_vel = {velocity_err} move_h = {height_w} a = {alpha/3}', end='')

    last_gamma = gamma
    last_velocity_err = velocity_err
    last_ang_vel_z = ang_vel_z
    last_height_err = height_err

    last_time = t
