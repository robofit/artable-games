#!/usr/bin/env python
# -*- coding: utf-8 -*-

##################################################################################
##
## @file       rescuers_gui.py
## @date       12. 03. 2018
## @brief      Main source file inspired by art_projected_gui package
## @author     Juraj Bačovčin (xbacov04@stud.fit.vutbr.cz)
##
##################################################################################

import os
import sys
import signal
import rospy
from PyQt5 import QtGui, QtWidgets, QtCore, QtNetwork, QtQml, QtTest
from xbacov04.helpers import ProjectorCalibrator, TouchCalibrator
from art_msgs.msg import Touch
from OpenGL import GL

## Setting up path to images and QML files
scriptDir = os.path.dirname(os.path.realpath(__file__))

## Class representing application engine
class RescuersGui(QtQml.QQmlApplicationEngine):

	## RescuersGui constructor
    def __init__(self, scene_server_port):

        super(RescuersGui, self).__init__()

        self.port = scene_server_port
        self.rpm = rospy.get_param("/art/interface/projected_gui/rpm")
        self.xinput = "USBest Technology SiS HID Touch Controller"

        # Setting up TCP connection for scene server and client
        self.tcpServer = QtNetwork.QTcpServer(self)
        if not self.tcpServer.listen(port=self.port):
            rospy.logerr(
                'Failed to start scene TCP server on port ' + str(self.port))

        self.tcpServer.newConnection.connect(self.new_connection)
        self.connections = []

        self.scene_timer = QtCore.QTimer()
        self.scene_timer.timeout.connect(self.send_to_clients_evt)
        self.scene_timer.start(1.0 / 15 * 1000)

        # Setting up projector
        self.projectors = [ProjectorCalibrator("localhost")]

		# Setting up projector calibration
        rospy.loginfo("Waiting for projector nodes...")
        for proj in self.projectors:
            proj.wait_until_available()
            if not proj.is_calibrated():
                rospy.loginfo("Starting calibration of projector: " + proj.proj_id)
                proj.calib_pub.publish(False)
                proj.calibrate(self.calibrated_cb)
            else:
                rospy.loginfo("Projector " + proj.proj_id + " already calibrated.")
                proj.calib_pub.publish(True)

	## Checks whether projector is calibrated
    def calibrated_cb(self, proj):

        proj.calib_pub.publish(True)
        rospy.loginfo("Projector " + proj.proj_id + " calibrated: " + str(proj.is_calibrated()))

	## Touchpad interaction handler
    def handle_touch(self, touching):

		# Disabling default touchpad input method
        os.system("xinput disable '" + self.xinput + "'")

		# Simulation of mouse press and release events
        if touching.touch:
            point = QtCore.QPoint(touching.point.point.x*self.rpm, self.rootObjects()[0].height() - touching.point.point.y*self.rpm)
            event = QtGui.QMouseEvent(QtCore.QEvent.MouseButtonPress, point, QtCore.Qt.LeftButton, QtCore.Qt.NoButton, QtCore.Qt.NoModifier)
            QtWidgets.QApplication.postEvent(self.rootObjects()[0], event)
        else:
            point = QtCore.QPoint(touching.point.point.x*self.rpm, self.rootObjects()[0].height() - touching.point.point.y*self.rpm)
            event = QtGui.QMouseEvent(QtCore.QEvent.MouseButtonRelease, point, QtCore.Qt.LeftButton, QtCore.Qt.NoButton, QtCore.Qt.NoModifier)
            QtWidgets.QApplication.postEvent(self.rootObjects()[0], event)

	## Touchpad calibration
    def continue_calibration(self, scene):

        self.scene = scene
        self.touches = [TouchCalibrator(self.scene, self.rpm)]

		# Setting up touchpad calibration
        rospy.loginfo("Waiting for touch nodes...")
        for touch in self.touches:
            touch.wait_until_available()
            rospy.loginfo("Starting calibration of touch.")
            touch.calibrate()
            rospy.Subscriber(touch.touch_ns + "touch", Touch, self.handle_touch)

	## Handler of connections to scene server
	# @todo Dealing with disconnected clients
    def new_connection(self):

        rospy.loginfo("Some projector node just connected.")
        self.connections.append(self.tcpServer.nextPendingConnection())
        self.connections[-1].setSocketOption(QtNetwork.QAbstractSocket.LowDelayOption, 1)
        self.continue_calibration(self.rootObjects()[0])

        # self.connections[-1].disconnected.connect(clientConnection.deleteLater)

	## Event sending output image to projector
    def send_to_clients_evt(self):

        if len(self.connections) == 0:
            return

		# Setting up output method
        block = QtCore.QByteArray()
        out = QtCore.QDataStream(block, QtCore.QIODevice.WriteOnly)
        out.setVersion(QtCore.QDataStream.Qt_4_0)
        out.writeUInt32(0)

		# Preparing output by grabbing screen image
        pix = self.rootObjects()[0].screen().grabWindow(self.rootObjects()[0].winId()).toImage()
        pix = pix.mirrored()
        img = QtCore.QByteArray()
        buffer = QtCore.QBuffer(img)
        buffer.open(QtCore.QIODevice.WriteOnly)
        pix.save(buffer, "JPG", 95)
        out << img

        out.device().seek(0)
        out.writeUInt32(block.size() - 4)

        for con in self.connections:
            con.write(block)

	## Displaying application window for debugging
    def debug_show(self):
      
        self.load(QtCore.QUrl(scriptDir + os.path.sep + 'qml/TitleScreen.qml'))

## Signal handler
def sigint_handler(*args):

    sys.stderr.write('\r')
    QtWidgets.QApplication.quit()

## Main function:
# Initialization of rescuers_gui node, setting up signals, application, engine and timer
def main(args):

    rospy.init_node('rescuers_gui', anonymous=True, log_level=rospy.DEBUG)

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtWidgets.QApplication(sys.argv)
    app.setWindowIcon(QtGui.QIcon(scriptDir + os.path.sep + 'img/icon.ico'))

    engine = RescuersGui(1234)
    engine.debug_show()
    engine.quit.connect(app.quit)

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    sys.exit(app.exec_())

## Executing main function with keyboard interrupt exception
if __name__ == '__main__':

    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")

##################################################################################
## End of file rescuers_gui.py
##################################################################################
