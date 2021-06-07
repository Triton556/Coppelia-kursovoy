import sim
import math


def PIDregKurs(h, e_last, e, Ui, maxU=3.0):
    P = 1.5 #2.1
    I =  0.00004#0.00001
    D =  0.03#0.2
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

def PIDregVel(h, e_last, e, Ui, maxU=10.0):

    P = 1.3
    I = 0.003
    D = 100
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

last_gamma = 0.0
last_dist = 0.0
last_velocity_err = 0.0
wi_move = 0.0
wi_kurs = 0.0



target_velocity = 5

while True:
    last_time = t

    res = sim.simxSetFloatSignal(clientID, 'target_velocity', target_velocity, sim.simx_opmode_oneshot)

    res, robotPos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)



    res, gamma = sim.simxGetFloatSignal(clientID, 'gamma', sim.simx_opmode_oneshot)

    res, dist = sim.simxGetFloatSignal(clientID, 'dist', sim.simx_opmode_oneshot)

    res, velocity_err = sim.simxGetFloatSignal(clientID, 'velocity_err', sim.simx_opmode_oneshot)

    res, ang_vel = sim.simxGetFloatSignal(clientID, 'ang_vel', sim.simx_opmode_oneshot)

    #if math.degrees(gamma) > 45 or math.degrees(gamma) < -45:
    #velocity_err = velocity_err - velocity_err * math.sin(gamma)

    #if ang_vel > 1:
    #    velocity_err -= math.sqrt(ang_vel)

    dt = t - last_time

    if dt == 0:
        dt = 0.1

    w_kurs, wi_kurs = PIDregKurs(dt, last_gamma, gamma, wi_kurs)



    w_move, wi_move = PIDregVel(dt, last_velocity_err, velocity_err, wi_move)

    if dist < 1.7:
        w_move /= 3

    if dist < 1.2:
        w_move = dist*3




    if dist < 0.1:

        w_move = 0
        w_kurs = 0

    print(f'\rErr_kurs = {gamma} Err_vel = {velocity_err} w_move = {w_move}', end='')

    res = sim.simxSetFloatSignal(clientID, 'w_kurs', w_kurs, sim.simx_opmode_oneshot)
    res = sim.simxSetFloatSignal(clientID, 'w_move', w_move, sim.simx_opmode_oneshot)

    last_gamma = gamma
    last_velocity_err = velocity_err