# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'C:\Users\Danil\Desktop\FEFE\1Kursovoi_Uxa\Coppelia-kursovoy\interface1.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(720, 627)
        self.moveSpline = QtWidgets.QRadioButton(Dialog)
        self.moveSpline.setGeometry(QtCore.QRect(40, 450, 161, 16))
        self.moveSpline.setChecked(False)
        self.moveSpline.setObjectName("moveSpline")
        self.moveForward = QtWidgets.QRadioButton(Dialog)
        self.moveForward.setEnabled(True)
        self.moveForward.setGeometry(QtCore.QRect(40, 490, 151, 16))
        self.moveForward.setChecked(True)
        self.moveForward.setObjectName("moveForward")
        self.speedSlider = QtWidgets.QSlider(Dialog)
        self.speedSlider.setGeometry(QtCore.QRect(40, 590, 160, 22))
        self.speedSlider.setMaximum(10)
        self.speedSlider.setSingleStep(1)
        self.speedSlider.setPageStep(1)
        self.speedSlider.setProperty("value", 5)
        self.speedSlider.setSliderPosition(5)
        self.speedSlider.setOrientation(QtCore.Qt.Horizontal)
        self.speedSlider.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.speedSlider.setObjectName("speedSlider")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(60, 530, 121, 31))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setGeometry(QtCore.QRect(40, 570, 16, 16))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setGeometry(QtCore.QRect(190, 570, 16, 16))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(Dialog)
        self.label_4.setGeometry(QtCore.QRect(240, 20, 251, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_4.setFont(font)
        self.label_4.setWordWrap(True)
        self.label_4.setObjectName("label_4")
        self.xStart = QtWidgets.QLineEdit(Dialog)
        self.xStart.setGeometry(QtCore.QRect(30, 270, 31, 21))
        self.xStart.setObjectName("xStart")
        self.yStart = QtWidgets.QLineEdit(Dialog)
        self.yStart.setGeometry(QtCore.QRect(70, 270, 31, 21))
        self.yStart.setObjectName("yStart")
        self.xFinish = QtWidgets.QLineEdit(Dialog)
        self.xFinish.setGeometry(QtCore.QRect(550, 50, 31, 21))
        self.xFinish.setObjectName("xFinish")
        self.yFinish = QtWidgets.QLineEdit(Dialog)
        self.yFinish.setGeometry(QtCore.QRect(590, 50, 31, 21))
        self.yFinish.setObjectName("yFinish")
        self.label_5 = QtWidgets.QLabel(Dialog)
        self.label_5.setGeometry(QtCore.QRect(40, 290, 16, 31))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(Dialog)
        self.label_6.setGeometry(QtCore.QRect(560, 70, 16, 21))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(Dialog)
        self.label_7.setGeometry(QtCore.QRect(80, 290, 16, 31))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(Dialog)
        self.label_8.setGeometry(QtCore.QRect(600, 70, 16, 21))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(Dialog)
        self.label_9.setGeometry(QtCore.QRect(560, 20, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(Dialog)
        self.label_10.setGeometry(QtCore.QRect(40, 230, 51, 31))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(Dialog)
        self.label_11.setGeometry(QtCore.QRect(90, 360, 181, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_11.setFont(font)
        self.label_11.setWordWrap(True)
        self.label_11.setObjectName("label_11")
        self.line = QtWidgets.QFrame(Dialog)
        self.line.setGeometry(QtCore.QRect(0, 330, 781, 16))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.line_2 = QtWidgets.QFrame(Dialog)
        self.line_2.setGeometry(QtCore.QRect(350, 340, 16, 291))
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.label_12 = QtWidgets.QLabel(Dialog)
        self.label_12.setGeometry(QtCore.QRect(110, 570, 20, 20))
        self.label_12.setObjectName("label_12")
        self.freqSlider = QtWidgets.QSlider(Dialog)
        self.freqSlider.setGeometry(QtCore.QRect(450, 380, 160, 22))
        self.freqSlider.setMaximum(10)
        self.freqSlider.setSingleStep(1)
        self.freqSlider.setPageStep(1)
        self.freqSlider.setProperty("value", 5)
        self.freqSlider.setSliderPosition(5)
        self.freqSlider.setOrientation(QtCore.Qt.Horizontal)
        self.freqSlider.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.freqSlider.setObjectName("freqSlider")
        self.label_13 = QtWidgets.QLabel(Dialog)
        self.label_13.setGeometry(QtCore.QRect(450, 360, 21, 16))
        self.label_13.setObjectName("label_13")
        self.label_14 = QtWidgets.QLabel(Dialog)
        self.label_14.setGeometry(QtCore.QRect(595, 360, 21, 20))
        self.label_14.setObjectName("label_14")
        self.label_15 = QtWidgets.QLabel(Dialog)
        self.label_15.setGeometry(QtCore.QRect(520, 360, 31, 21))
        self.label_15.setObjectName("label_15")
        self.label_16 = QtWidgets.QLabel(Dialog)
        self.label_16.setGeometry(QtCore.QRect(460, 330, 141, 31))
        self.label_16.setObjectName("label_16")
        self.missionProgress = QtWidgets.QProgressBar(Dialog)
        self.missionProgress.setGeometry(QtCore.QRect(410, 550, 291, 51))
        self.missionProgress.setProperty("value", 0)
        self.missionProgress.setObjectName("missionProgress")
        self.label_17 = QtWidgets.QLabel(Dialog)
        self.label_17.setGeometry(QtCore.QRect(490, 510, 141, 31))
        self.label_17.setObjectName("label_17")
        self.startMapping = QtWidgets.QPushButton(Dialog)
        self.startMapping.setGeometry(QtCore.QRect(550, 420, 93, 28))
        self.startMapping.setObjectName("startMapping")
        self.startNavigation = QtWidgets.QPushButton(Dialog)
        self.startNavigation.setGeometry(QtCore.QRect(550, 460, 93, 28))
        self.startNavigation.setObjectName("startNavigation")
        self.startButton = QtWidgets.QPushButton(Dialog)
        self.startButton.setGeometry(QtCore.QRect(570, 160, 131, 61))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.startButton.setFont(font)
        self.startButton.setObjectName("startButton")
        self.stopButton = QtWidgets.QPushButton(Dialog)
        self.stopButton.setGeometry(QtCore.QRect(570, 240, 131, 61))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.stopButton.setFont(font)
        self.stopButton.setObjectName("stopButton")
        self.mappingStatus = QtWidgets.QLabel(Dialog)
        self.mappingStatus.setGeometry(QtCore.QRect(650, 430, 31, 16))
        self.mappingStatus.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.mappingStatus.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.mappingStatus.setTextFormat(QtCore.Qt.AutoText)
        self.mappingStatus.setAlignment(QtCore.Qt.AlignCenter)
        self.mappingStatus.setObjectName("mappingStatus")
        self.label_19 = QtWidgets.QLabel(Dialog)
        self.label_19.setGeometry(QtCore.QRect(460, 410, 81, 41))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_19.setFont(font)
        self.label_19.setObjectName("label_19")
        self.label_20 = QtWidgets.QLabel(Dialog)
        self.label_20.setGeometry(QtCore.QRect(460, 450, 81, 41))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_20.setFont(font)
        self.label_20.setObjectName("label_20")
        self.navigationStatus = QtWidgets.QLabel(Dialog)
        self.navigationStatus.setGeometry(QtCore.QRect(650, 460, 31, 16))
        self.navigationStatus.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.navigationStatus.setAlignment(QtCore.Qt.AlignCenter)
        self.navigationStatus.setObjectName("navigationStatus")
        self.createPathButton = QtWidgets.QPushButton(Dialog)
        self.createPathButton.setGeometry(QtCore.QRect(570, 100, 131, 41))
        self.createPathButton.setObjectName("createPathButton")
        self.height_slider = QtWidgets.QSlider(Dialog)
        self.height_slider.setGeometry(QtCore.QRect(260, 450, 22, 160))
        self.height_slider.setMinimum(3)
        self.height_slider.setMaximum(6)
        self.height_slider.setPageStep(1)
        self.height_slider.setProperty("value", 4)
        self.height_slider.setOrientation(QtCore.Qt.Vertical)
        self.height_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.height_slider.setObjectName("height_slider")
        self.label_18 = QtWidgets.QLabel(Dialog)
        self.label_18.setGeometry(QtCore.QRect(290, 440, 21, 16))
        self.label_18.setObjectName("label_18")
        self.label_21 = QtWidgets.QLabel(Dialog)
        self.label_21.setGeometry(QtCore.QRect(290, 490, 21, 16))
        self.label_21.setObjectName("label_21")
        self.label_22 = QtWidgets.QLabel(Dialog)
        self.label_22.setGeometry(QtCore.QRect(290, 540, 21, 16))
        self.label_22.setObjectName("label_22")
        self.label_23 = QtWidgets.QLabel(Dialog)
        self.label_23.setGeometry(QtCore.QRect(290, 590, 21, 16))
        self.label_23.setObjectName("label_23")
        self.label_24 = QtWidgets.QLabel(Dialog)
        self.label_24.setGeometry(QtCore.QRect(230, 400, 91, 31))
        self.label_24.setObjectName("label_24")
        self.startServerButton = QtWidgets.QPushButton(Dialog)
        self.startServerButton.setGeometry(QtCore.QRect(10, 10, 101, 41))
        self.startServerButton.setObjectName("startServerButton")
        self.horizontalLayoutWidget = QtWidgets.QWidget(Dialog)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(119, 49, 421, 241))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Lightning McQueen"))
        self.moveSpline.setText(_translate("Dialog", "Двигаться по сплайну"))
        self.moveForward.setText(_translate("Dialog", "Двигаться по прямой"))
        self.label.setText(_translate("Dialog", "Скорость Движения"))
        self.label_2.setText(_translate("Dialog", "0"))
        self.label_3.setText(_translate("Dialog", "1"))
        self.label_4.setText(_translate("Dialog", "Область сканирования"))
        self.label_5.setText(_translate("Dialog", "Х"))
        self.label_6.setText(_translate("Dialog", "Х"))
        self.label_7.setText(_translate("Dialog", "Y"))
        self.label_8.setText(_translate("Dialog", "Y"))
        self.label_9.setText(_translate("Dialog", "FINISH"))
        self.label_10.setText(_translate("Dialog", "START"))
        self.label_11.setText(_translate("Dialog", "Параметры движения"))
        self.label_12.setText(_translate("Dialog", "0.5"))
        self.label_13.setText(_translate("Dialog", "0.1"))
        self.label_14.setText(_translate("Dialog", "0.5"))
        self.label_15.setText(_translate("Dialog", "0.25"))
        self.label_16.setText(_translate("Dialog", "Частота сканирования"))
        self.label_17.setText(_translate("Dialog", "Прогресс миссии"))
        self.startMapping.setText(_translate("Dialog", "Запуск"))
        self.startNavigation.setText(_translate("Dialog", "Запуск"))
        self.startButton.setText(_translate("Dialog", "Старт миссии"))
        self.stopButton.setText(_translate("Dialog", "СТОП"))
        self.mappingStatus.setText(_translate("Dialog", "OFF"))
        self.label_19.setText(_translate("Dialog", "Картограф"))
        self.label_20.setText(_translate("Dialog", "Навигация"))
        self.navigationStatus.setText(_translate("Dialog", "OFF"))
        self.createPathButton.setText(_translate("Dialog", "Построить маршрут"))
        self.label_18.setText(_translate("Dialog", "3"))
        self.label_21.setText(_translate("Dialog", "2.5"))
        self.label_22.setText(_translate("Dialog", "2"))
        self.label_23.setText(_translate("Dialog", "1.5"))
        self.label_24.setText(_translate("Dialog", "Высота от дна"))
        self.startServerButton.setText(_translate("Dialog", "Start server"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())