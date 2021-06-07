import sim
import math


def PIDregKurs(h, e_last, e, Ui, maxU=15.0):
    P = 2.2
    I = 0.00001
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

def PIDregVel(h, e_last, e, Ui, maxU=15.0):
    P = 0
    I = 0
    D = 0
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

print(clientID)

res, robot = sim.simxGetObjectHandle(clientID, 'robot', sim.simx_opmode_oneshot_wait)

res, flaHandle = sim.simxGetObjectHandle(clientID, 'fla', sim.simx_opmode_oneshot_wait)

res, fluHandle = sim.simxGetObjectHandle(clientID, 'flu', sim.simx_opmode_oneshot_wait)

res, fraHandle = sim.simxGetObjectHandle(clientID, 'fra', sim.simx_opmode_oneshot_wait)

res, fruHandle = sim.simxGetObjectHandle(clientID, 'fru', sim.simx_opmode_oneshot_wait)

res, rlaHandle = sim.simxGetObjectHandle(clientID, 'rla', sim.simx_opmode_oneshot_wait)

res, rluHandle = sim.simxGetObjectHandle(clientID, 'rlu', sim.simx_opmode_oneshot_wait)

res, rraHandle = sim.simxGetObjectHandle(clientID, 'rra', sim.simx_opmode_oneshot_wait)

res, rruHandle = sim.simxGetObjectHandle(clientID, 'rru', sim.simx_opmode_oneshot_wait)

# res, flaPosition = sim.simxGetObjectPosition(clientID, flaHandle, robot, sim.simx_opmode_oneshot_wait)
#
# res, fluPosition = sim.simxGetObjectPosition(clientID, fluHandle, robot, sim.simx_opmode_oneshot_wait)
#
# res, fraPosition = sim.simxGetObjectPosition(clientID, fraHandle, robot, sim.simx_opmode_oneshot_wait)
#
# res, fruPosition = sim.simxGetObjectPosition(clientID, fruHandle, robot, sim.simx_opmode_oneshot_wait)
#
# res, rlaPosition = sim.simxGetObjectPosition(clientID, rlaHandle, robot, sim.simx_opmode_oneshot_wait)
#
# res, rluPosition = sim.simxGetObjectPosition(clientID, rluHandle, robot, sim.simx_opmode_oneshot_wait)
#
# res, rraPosition = sim.simxGetObjectPosition(clientID, rraHandle, robot, sim.simx_opmode_oneshot_wait)
#
# res, rruPosition = sim.simxGetObjectPosition(clientID, rruHandle, robot, sim.simx_opmode_oneshot_wait)


print(res)

res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

w = 2.0

last_gamma = 0.0
last_dist = 0.0
UiFLA = 0.0
UiFRA = 0.0
UiRLA = 0.0
UiRRA = 0.0
UiKurs = 0.0
UiVel = 0.0
uVel = 0.0
t = 0.1


print(res)
while True:
    last_time = t
    res, t = sim.simxGetFloatSignal(clientID, 'time', sim.simx_opmode_oneshot)

    res, robotPos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)

    res, gamma = sim.simxGetFloatSignal(clientID, 'gamma', sim.simx_opmode_oneshot)

    res, dist = sim.simxGetFloatSignal(clientID, 'dist', sim.simx_opmode_oneshot)


    #dist = math.sin(gamma)*dist
    # if (gamma - max_gamma > 0):
    #     max_time = t
    #     max_gamma = gamma
    # if (gamma - min_gamma < 0):
    #     min_time = t
    #     min_gamma = gamma

    #gamma += math.sin(gamma) * dist * 10

    dt = t - last_time

    if dt == 0:
        dt = 0.1

    uKurs, UiKurs = PIDregKurs(dt, last_gamma, gamma, UiKurs)

    uVel, UiVel = PIDregVel(dt, last_dist, dist, UiVel)

    u = uKurs + uVel

    # uFLA, UiFLA = PIDreg(0.1, last_gamma - dist, gamma - dist, UiFLA)
    # uFRA, UiFRA = PIDreg(0.1, last_gamma + dist, gamma + dist, UiFRA)
    # uRLA, UiRLA = PIDreg(0.1, last_gamma - dist, gamma - dist, UiRLA)
    # uRRA, UiRRA = PIDreg(0.1, last_gamma + dist, gamma + dist, UiRRA)

    print(f"\rErr kurs = {gamma} Err dist = {dist} u = {u}", end='')
    # print("U = ", u)

    res = sim.simxSetFloatSignal(clientID, "w_kurs", u, sim.simx_opmode_oneshot)

    last_gamma = gamma
    last_dist = dist
