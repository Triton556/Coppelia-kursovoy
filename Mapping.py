import csv
import socket
import threading

import sim
from math import cos, sin, sqrt, fabs, atan2, degrees, radians
import time

UDP_MAX_SIZE = 65535

navigation_manager = ('127.0.0.1', 3003)
interface_manager = ('127.0.0.1', 3002)

integrated_pos = [0.0, 0.0, 0.0]
alpha, beta, gamma, height_err, clientID = 0.0, 0.0, 0.0, 0.0, -1
lidar_check_time = 0.15
lidar_points =[]

def Vel_rotate(a, b, g, vx, vy, vz):
    va_x = vx * (cos(b) * cos(g) - sin(a) * sin(b) * sin(g)) - vy * (cos(a) * sin(g)) + vz * (
            cos(g) * sin(b) + cos(b) * sin(a) * sin(g))
    va_y = vx * (cos(b) * sin(g) + cos(g) * sin(a) * sin(b)) + vy * (cos(a) * cos(g)) + vz * (
            sin(b) * sin(g) - cos(b) * cos(g) * sin(a))
    va_z = -vx * (cos(a) * sin(b)) + vy * (sin(a)) + vz * (cos(a) * cos(b))
    return [va_x, va_y, va_z]


def connect(host: str = '127.0.0.1', port: int = 3001):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))

    msg = "Mapping connected!".encode()
    send_to_interface(s, msg)

    threading.Thread(target=listen, args=(s,), daemon=True).start()

    threading.Thread(target=map_making, args=(s,), daemon=True).start()
    print('thread started')

    #map_making(s)

    while True:
        input('working...')




def listen(s: socket.socket):
    global integrated_pos, alpha, beta, gamma, height_err, clientID, lidar_points
    while True:

        try:
            msg = s.recv(UDP_MAX_SIZE)

        except ConnectionResetError:
            print('no data')
            continue

        if not msg:
            continue

        try:
            data = msg.decode().split('|')
            if data[0] == 'nav':
                alpha = float(data[1])
                beta = float(data[2])
                gamma = float(data[3])
                integrated_pos[0] = float(data[4])
                integrated_pos[1] = float(data[5])
                integrated_pos[2] = float(data[6])
                clientID = int(data[7])
                lidar_points_string = data[8]
        except:
            lidar_points_string = msg
            lidar_points = sim.simxUnpackFloats(lidar_points_string)


def send_to_interface(s: socket.socket, msg):
    s.sendto(msg, interface_manager)


def map_making(s: socket.socket):
    global integrated_pos, alpha, beta, gamma, height_err, lidar_check_time, clientID

    if clientID != -1:
       print('Connected to remote API server')

    lidar_end_time = 0

    while True:
        if clientID == -1:
            print('not connected to coppelia!')

            continue

        start_time = time.time()
        #print(start_time - lidar_end_time)
        if start_time - lidar_end_time > lidar_check_time:

            #res, lidar_points_string = sim.simxGetStringSignal(clientID, 'lidar_points', sim.simx_opmode_oneshot)
            #print(lidar_points_string)
            if lidar_points != []:
                #lidar_points = sim.simxUnpackFloats(lidar_points_string)

                for i in range(0, len(lidar_points), 3):
                    point = [0, 0, 0]
                    point_in_lidar = [lidar_points[i], lidar_points[i + 1], lidar_points[i + 2]]
                    point_in_glob = Vel_rotate(alpha - radians(90), beta - radians(90), gamma - radians(90),
                                               point_in_lidar[0], point_in_lidar[1], point_in_lidar[2])

                    with open('points.csv', 'a', newline='') as csvfile:
                        cloud_writer = csv.writer(csvfile)
                        point = [point_in_glob[0] + integrated_pos[0], point_in_glob[1] + integrated_pos[1],
                                 point_in_glob[2] - height_err + integrated_pos[2]]
                        cloud_writer.writerow(
                            [point_in_glob[0] + integrated_pos[0], point_in_glob[1] + integrated_pos[1],
                             point_in_glob[2] - height_err + integrated_pos[2]])
                        print(f'\r{point}', end='')
            lidar_end_time = time.time()


if __name__ == "__main__":
    connect()
