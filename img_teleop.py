import logging
import sys
import time
import math
import argparse
import socket, os, struct
from threading import Thread
from time import sleep

import numpy as np
from vispy import scene
from vispy.scene import visuals
from vispy.scene.cameras import TurntableCamera

import cv2

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
# from cflib.utils.multiranger import Multiranger

from PyQt5 import QtCore, QtWidgets

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Enable Crazyflie position plotting
PLOT_CF = False
# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.3

# AI deck port id
# deck_port = 5000
# Ai deck IP addr
# deck_ip = "192.168.4.1"

if len(sys.argv) > 1:
    URI = sys.argv[1]

class Camera:
    def __init__(self, deck_ip, deck_port):
        # Start as seperate thread
        # Thread.__init__(self)

        # Connect to AI deck
        print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        print("Socket connected")

        self.imgdata = None
        self.data_buffer = bytearray()

    def streamImages(self, save_image):
        start = time.time()
        counter = 0

        while(True):
            # Grab packet
            packetInfoRaw = self.rx_bytes(4)
            [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

            imgHeader = self.rx_bytes(length - 2)
            [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

            # Check if magic is valid
            if magic == 0xBC:
                imgStream = bytearray()
                # Stream img data into img buffer
                while len(imgStream) < size:
                    packetInfoRaw = self.rx_bytes(4)
                    [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                    #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                    chunk = self.rx_bytes(length - 2)
                    imgStream.extend(chunk)
                # Save raw grayscale img as np array
                if format == 0:
                    bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
                    bayer_img.shape = (244, 324)
                    self.imgdata = bayer_img    # raw bayer img
                    cv2.imshow('Camera', bayer_img)
                    cv2.waitKey(1)
                    if save_image:
                        cv2.imwrite(f"stream/raw/img_{count:06d}.png", bayer_img)
                else:
                    with open("img.jpeg", "wb") as f:
                        f.write(imgStream)
                    nparr = np.frombuffer(imgStream, np.uint8)
                    self.imgdata = nparr
                    # decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)

                # Count the img stream fps
                count  = count + 1
                meanTimePerImage = (time.time()-start) / count
                print("{}".format(meanTimePerImage))

    
    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size-len(data)))
        return data


class MainWindow(QtWidgets.QMainWindow, Thread):

    def __init__(self, URI):
        QtWidgets.QMainWindow.__init__(self)

        self.resize(700, 500)
        self.setWindowTitle('FPV teleop')

        self.canvas = Canvas(self.updateHover)
        self.canvas.create_native()
        self.canvas.native.setParent(self)

        self.setCentralWidget(self.canvas.native)

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Connect callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)

        # Connect to the Crazyflie
        self.cf.open_link(URI)

        self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}

        self.hoverTimer = QtCore.QTimer()
        self.hoverTimer.timeout.connect(self.sendHoverCommand)
        self.hoverTimer.setInterval(100)
        self.hoverTimer.start()

    def sendHoverCommand(self):
        self.cf.commander.send_hover_setpoint(
            self.hover['x'], self.hover['y'], self.hover['yaw'],
            self.hover['height'])

    def updateHover(self, k, v):
        if (k != 'height'):
            self.hover[k] = v * SPEED_FACTOR
        else:
            self.hover[k] += v

    def disconnected(self, URI):
        print('Disconnected')

    def connected(self, URI):
        print('We are now connected to {}'.format(URI))

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')

        try:
            self.cf.log.add_config(lpos)
            lpos.data_received_cb.add_callback(self.pos_data)
            lpos.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

        # lmeas = LogConfig(name='Meas', period_in_ms=100)
        # lmeas.add_variable('range.front')
        # lmeas.add_variable('range.back')
        # lmeas.add_variable('range.up')
        # lmeas.add_variable('range.left')
        # lmeas.add_variable('range.right')
        # lmeas.add_variable('range.zrange')
        # lmeas.add_variable('stabilizer.roll')
        # lmeas.add_variable('stabilizer.pitch')
        # lmeas.add_variable('stabilizer.yaw')

        # try:
        #     self.cf.log.add_config(lmeas)
        #     lmeas.data_received_cb.add_callback(self.meas_data)
        #     lmeas.start()
        # except KeyError as e:
        #     print('Could not start log configuration,'
        #           '{} not found in TOC'.format(str(e)))
        # except AttributeError:
        #     print('Could not add Measurement log config, bad configuration.')

    def pos_data(self, timestamp, data, logconf):
        position = [
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z']
        ]
        self.canvas.set_position(position)

    # def meas_data(self, timestamp, data, logconf):
    #     measurement = {
    #         'roll': data['stabilizer.roll'],
    #         'pitch': data['stabilizer.pitch'],
    #         'yaw': data['stabilizer.yaw'],
    #         'front': data['range.front'],
    #         'back': data['range.back'],
    #         'up': data['range.up'],
    #         'down': data['range.zrange'],
    #         'left': data['range.left'],
    #         'right': data['range.right']
    #     }
    #     self.canvas.set_measurement(measurement)

    def closeEvent(self, event):
        if (self.cf is not None):
            self.cf.close_link()

class Canvas(scene.SceneCanvas):
    def __init__(self, keyupdateCB):
        scene.SceneCanvas.__init__(self, keys=None)
        self.size = 800, 600
        self.unfreeze()
        self.view = self.central_widget.add_view()
        self.view.bgcolor = '#ffffff'
        self.view.camera = TurntableCamera(
            fov=10.0, distance=30.0, up='+z', center=(0.0, 0.0, 0.0))
        self.last_pos = [0, 0, 0]
        self.pos_markers = visuals.Markers()
        self.meas_markers = visuals.Markers()
        self.pos_data = np.array([0, 0, 0], ndmin=2)
        self.meas_data = np.array([0, 0, 0], ndmin=2)
        self.lines = []

        self.view.add(self.pos_markers)
        self.view.add(self.meas_markers)
        for i in range(6):
            line = visuals.Line()
            self.lines.append(line)
            self.view.add(line)

        self.keyCB = keyupdateCB

        self.freeze()

        scene.visuals.XYZAxis(parent=self.view.scene)

    def on_key_press(self, event):
        if (not event.native.isAutoRepeat()):
            if (event.native.key() == QtCore.Qt.Key_Left):
                self.keyCB('y', 1)
            if (event.native.key() == QtCore.Qt.Key_Right):
                self.keyCB('y', -1)
            if (event.native.key() == QtCore.Qt.Key_Up):
                self.keyCB('x', 1)
            if (event.native.key() == QtCore.Qt.Key_Down):
                self.keyCB('x', -1)
            if (event.native.key() == QtCore.Qt.Key_A):
                self.keyCB('yaw', -70)
            if (event.native.key() == QtCore.Qt.Key_D):
                self.keyCB('yaw', 70)
            if (event.native.key() == QtCore.Qt.Key_Z):
                self.keyCB('yaw', -200)
            if (event.native.key() == QtCore.Qt.Key_X):
                self.keyCB('yaw', 200)
            if (event.native.key() == QtCore.Qt.Key_W):
                self.keyCB('height', 0.1)
            if (event.native.key() == QtCore.Qt.Key_S):
                self.keyCB('height', -0.1)

    def on_key_release(self, event):
        if (not event.native.isAutoRepeat()):
            if (event.native.key() == QtCore.Qt.Key_Left):
                self.keyCB('y', 0)
            if (event.native.key() == QtCore.Qt.Key_Right):
                self.keyCB('y', 0)
            if (event.native.key() == QtCore.Qt.Key_Up):
                self.keyCB('x', 0)
            if (event.native.key() == QtCore.Qt.Key_Down):
                self.keyCB('x', 0)
            if (event.native.key() == QtCore.Qt.Key_A):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_D):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_W):
                self.keyCB('height', 0)
            if (event.native.key() == QtCore.Qt.Key_S):
                self.keyCB('height', 0)
            if (event.native.key() == QtCore.Qt.Key_Z):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_X):
                self.keyCB('yaw', 0)

    def set_position(self, pos):
        self.last_pos = pos
        if (PLOT_CF):
            self.pos_data = np.append(self.pos_data, [pos], axis=0)
            self.pos_markers.set_data(self.pos_data, face_color='red', size=5)

    def rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos(math.radians(roll))
        cosp = math.cos(math.radians(pitch))
        cosy = math.cos(math.radians(yaw))

        sinr = math.sin(math.radians(roll))
        sinp = math.sin(math.radians(pitch))
        siny = math.sin(math.radians(yaw))

        roty = np.array([[cosy, -siny, 0],
                         [siny, cosy, 0],
                         [0, 0,    1]])

        rotp = np.array([[cosp, 0, sinp],
                         [0, 1, 0],
                         [-sinp, 0, cosp]])

        rotr = np.array([[1, 0,   0],
                         [0, cosr, -sinr],
                         [0, sinr,  cosr]])

        rotFirst = np.dot(rotr, rotp)

        rot = np.array(np.dot(rotFirst, roty))

        tmp = np.subtract(point, origin)
        tmp2 = np.dot(rot, tmp)
        return np.add(tmp2, origin)

    # def rotate_and_create_points(self, m):
    #     data = []
    #     o = self.last_pos
    #     roll = m['roll']
    #     pitch = -m['pitch']
    #     yaw = m['yaw']

    #     if (m['up'] < SENSOR_TH):
    #         up = [o[0], o[1], o[2] + m['up'] / 1000.0]
    #         data.append(self.rot(roll, pitch, yaw, o, up))

    #     if (m['down'] < SENSOR_TH and PLOT_SENSOR_DOWN):
    #         down = [o[0], o[1], o[2] - m['down'] / 1000.0]
    #         data.append(self.rot(roll, pitch, yaw, o, down))

    #     if (m['left'] < SENSOR_TH):
    #         left = [o[0], o[1] + m['left'] / 1000.0, o[2]]
    #         data.append(self.rot(roll, pitch, yaw, o, left))

    #     if (m['right'] < SENSOR_TH):
    #         right = [o[0], o[1] - m['right'] / 1000.0, o[2]]
    #         data.append(self.rot(roll, pitch, yaw, o, right))

    #     if (m['front'] < SENSOR_TH):
    #         front = [o[0] + m['front'] / 1000.0, o[1], o[2]]
    #         data.append(self.rot(roll, pitch, yaw, o, front))

    #     if (m['back'] < SENSOR_TH):
    #         back = [o[0] - m['back'] / 1000.0, o[1], o[2]]
    #         data.append(self.rot(roll, pitch, yaw, o, back))

    #     return data

    # def set_measurement(self, measurements):
    #     data = self.rotate_and_create_points(measurements)
    #     o = self.last_pos
    #     for i in range(6):
    #         if (i < len(data)):
    #             o = self.last_pos
    #             self.lines[i].set_data(np.array([o, data[i]]))
    #         else:
    #             self.lines[i].set_data(np.array([o, o]))

    #     if (len(data) > 0):
    #         self.meas_data = np.append(self.meas_data, data, axis=0)
    #     self.meas_markers.set_data(self.meas_data, face_color='blue', size=5)



if __name__ == '__main__':
    # Args for setting IP/port of AI-deck. Default settings are for when
    # AI-deck is in AP mode.
    parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
    parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
    parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
    parser.add_argument('--save', action='store_true', help="Save streamed images")
    args = parser.parse_args()


    # Initialize the low-level drivers (don't list the debug drivers)
    # cflib.crtp.init_drivers(enable_debug_driver=False)

    # cf = Crazyflie(rw_cache='./cache')
    appQt = QtWidgets.QApplication(sys.argv)
    win = MainWindow(URI)
    camera = Camera(args.n, args.p)     # n - camera ip address / p - camera port address
    
    win.start()
    # camera.start()

    win.show()
    appQt.exec_()
    
    camera_t = Thread(target = camera.streamImages, args = (False, ))      # stream images no saving in seperate thread
    camera_t.start()

    win.join()
    camera_t.join()
    print("Img_teleop finished...exiting")