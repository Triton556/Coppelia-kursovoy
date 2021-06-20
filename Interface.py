import os
import socket
import threading
import numpy as np
from path import create_path
from Spline import Find_spline

from PyQt5 import uic, QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

UDP_MAX_SIZE = 65535

mapping_manager = ('127.0.0.1', 3001)
interface_manager = ('127.0.0.1', 3002)
navigation_manager = ('127.0.0.1', 3003)




mapping_on = False
navigation_on = False
listening = False
has_graph = False
sim_started = False
send_start = False





robot_pos = [0.0, 0.0, 0.0]
traveled_dist = 0

mission = []
move_mode = 0
target_speed = 0
target_height = 0
scan_freq = 1


Form, Window = uic.loadUiType("interface1.ui")


class Server():

    def start_listening(self, form):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(interface_manager)
        self.s.settimeout(0.5)
        self.listen(form)

    def listen(self, form):
        global navigation_on, mapping_on, robot_pos, traveled_dist
        print("start listening")
        # listening = False
        while listening:


            msg = None
            try:
                msg = self.s.recv(UDP_MAX_SIZE)
            except socket.timeout:
                # print('no data')
                pass

            # if not msg:
            #    pass

            try:
                if not navigation_on or mapping_on:
                    data = msg.decode()
                    if data == 'map':
                        form.mappingStatus.setText('ON')
                        mapping_on = True

                    elif data == 'nav':
                        form.navigationStatus.setText('ON')
                        navigation_on = True
                else:
                    data = msg.decode().split('|')
                    if data[0] == 'pos':
                        robot_pos[0] = float(data[1])
                        robot_pos[1] = float(data[2])
                        robot_pos[2] = float(data[3])
                    if data[0] == 'trav_dist':
                        traveled_dist = data[1]
            except:
                pass

            if not sim_started and send_start:
                start_mes =
                self.s.sendto()
        # self.s.close()
        print('server closed!')


class Menu(QMainWindow):

    def __init__(self):
        QMainWindow.__init__(self)
        self.app = QApplication([])
        self.window = Window()
        self.form = Form()
        self.form.setupUi(self)

        self.form.startMapping.clicked.connect(self.start_mapping_module)
        self.form.startServerButton.clicked.connect(self.start_listening_from_button)
        self.form.startNavigation.clicked.connect(self.start_navigation_module)
        self.form.createPathButton.clicked.connect(self.calculate_path)
        self.form.startButton.clicked.connect(self.start_mission)


    def start_listening_from_button(self):
        global listening
        if not listening:
            self.form.startServerButton.setText('Stop server')
            listening = True
            server = Server()
            listenThread = threading.Thread(target=server.start_listening, args=(self.form,))
            listenThread.start()
        else:
            self.form.startServerButton.setText('Start server')
            listening = False

    def start_navigation_module(self):
        pathToDirectory = os.path.dirname(os.path.realpath(__file__))
        os.startfile(fr'{pathToDirectory}\Navigation.py')

    def start_mapping_module(self):
        pathToDirectory = os.path.dirname(os.path.realpath(__file__))
        os.startfile(fr'{pathToDirectory}\Mapping.py')

    def calculate_path(self):
        global mission, has_graph
        spline_mode = self.form.moveSpline.isChecked()

        x_start = self.form.xStart.text()
        y_start = self.form.yStart.text()
        x_finish = self.form.xFinish.text()
        y_finish = self.form.yFinish.text()

        try:
            x_start = float(x_start)
            y_start = float(y_start)
            x_finish = float(x_finish)
            y_finish = float(y_finish)

        except:
            print('except')
            x_start = -10
            y_start = -10
            x_finish = 10
            y_finish = 10

        target_height = self.form.height_slider.value() / 2

        start_point = [float(x_start), float(y_start)]
        finish_point = [float(x_finish), float(y_finish)]

        route = create_path(start_point, finish_point, [robot_pos[0], robot_pos[1]], target_height)

        mission = route

        if spline_mode:
            route = np.asarray(route)
            spline_route = Find_spline(route[:, 0], route[:, 1])
            mission = spline_route

        l = self.form.horizontalLayout
        graph = MyDynamicMplCanvas(dpi=100)

        if not has_graph:
            l.addWidget(graph)

        has_graph = True

    def start_mission(self):
        global move_mode, scan_freq, mission, target_speed, target_height
        if mapping_on and navigation_on:
            move_mode = self.form.speedSlider.value()/10
        else:
            msgBx = QMessageBox()
            msgBx.setWindowTitle("Ошибка")
            msgBx.setText("Убедитесь, что модули подключены!")
            msgBx.exec()


class MyMplCanvas(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)

        self.compute_initial_figure()

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def compute_initial_figure(self):
        pass


class MyDynamicMplCanvas(MyMplCanvas):
    """A canvas that updates itself every second with a new plot."""

    def __init__(self, *args, **kwargs):
        MyMplCanvas.__init__(self, *args, **kwargs)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(1000)

    def compute_initial_figure(self):
        array = np.asarray(mission)

        self.axes.plot(array[:, 0], array[:, 1], 'r.--')

        self.axes.plot(robot_pos[0], robot_pos[1], 'gx')

    def update_figure(self):
        # Build a list of 4 random integers between 0 and 10 (both inclusive)
        array = np.asarray(mission)
        self.axes.cla()

        self.axes.plot(array[:, 0], array[:, 1], 'r.--')

        self.axes.plot(robot_pos[0], robot_pos[1], 'gx')

        self.draw()


if __name__ == '__main__':
    import sys

    app = QApplication([])
    window = Menu()

    window.show()

    sys.exit(app.exec())
