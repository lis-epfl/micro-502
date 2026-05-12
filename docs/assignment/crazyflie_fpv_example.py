#!/usr/bin/env python3
"""
Crazyflie FPV example.
  Video:   UDP stream from the AI-deck (laptop must be on the AI-deck WiFi)
  Control: cflib over Crazyradio

Keys:  arrows = pitch/roll,  A/D = yaw,  W/S = up/down,  Space = stop.
"""
import contextlib
import os
import socket
import struct
import sys

import cflib.crtp
import cv2
import numpy as np
from cflib.crazyflie import Crazyflie
from PyQt6 import QtCore, QtGui, QtWidgets


@contextlib.contextmanager
def _muted_stderr():
    saved = os.dup(2)
    null = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(null, 2)
        yield
    finally:
        os.dup2(saved, 2)
        os.close(null)
        os.close(saved)

# --- Configure these for your setup ---
URI         = 'radio://0/10/2M/E7E7E7E701'
AIDECK_IP   = '192.168.4.1'
AIDECK_PORT = 5000
LOCAL_PORT  = 5001
START_MAGIC = b'FER'
SPEED       = 0.6

CPX_HEADER_SIZE  = 4
IMG_HEADER_MAGIC = 0xBC
IMG_HEADER_SIZE  = 11
IMG_WIDTH        = 324
IMG_HEIGHT       = 244
MIN_JPEG_BYTES   = 5000


class UdpVideoThread(QtCore.QThread):
    frame_ready = QtCore.pyqtSignal(np.ndarray)

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        sock.bind(('0.0.0.0', LOCAL_PORT))
        sock.sendto(START_MAGIC, (AIDECK_IP, AIDECK_PORT))

        buffer = bytearray()
        expected_size = 0
        receiving = False

        while True:
            data, _ = sock.recvfrom(2048)
            if len(data) < CPX_HEADER_SIZE:
                continue
            payload = data[CPX_HEADER_SIZE:]

            if len(payload) >= IMG_HEADER_SIZE and payload[0] == IMG_HEADER_MAGIC:
                _, w, h, _, _, size = struct.unpack(
                    '<BHHBBI', payload[:IMG_HEADER_SIZE])
                if w == IMG_WIDTH and h == IMG_HEIGHT and 0 < size < 65536:
                    expected_size = size
                    buffer = bytearray()
                    receiving = True
                    continue

            if not receiving:
                continue

            buffer.extend(payload)

            if len(buffer) >= expected_size:
                self._decode_and_emit(buffer)
                receiving = False

    def _decode_and_emit(self, buffer):
        soi = buffer.find(b'\xff\xd8')
        eoi = buffer.rfind(b'\xff\xd9')
        if soi < 0 or eoi <= soi:
            return
        jpeg_len = eoi + 2 - soi
        if jpeg_len < MIN_JPEG_BYTES:
            return
        jpeg = np.frombuffer(buffer, np.uint8, count=jpeg_len, offset=soi)
        with _muted_stderr():
            img = cv2.imdecode(jpeg, cv2.IMREAD_UNCHANGED)
        if img is None or img.shape[:2] != (IMG_HEIGHT, IMG_WIDTH):
            return
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.frame_ready.emit(img)


class FPVWindow(QtWidgets.QWidget):
    _connected_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Crazyflie FPV')

        self.image_label = QtWidgets.QLabel('Waiting for video...')
        self.status_label = QtWidgets.QLabel(f'Connecting to {URI}...')
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.image_label)
        layout.addWidget(self.status_label)

        self.hover = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'height': 0.3}
        self._held = set()

        self.video = UdpVideoThread(self)
        self.video.frame_ready.connect(self._show_frame)
        self.video.start()

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(rw_cache='cache')
        self._connected_signal.connect(self._on_connected)
        self.cf.connected.add_callback(self._connected_signal.emit)
        self.cf.open_link(URI)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._send_setpoint)
        self._timer.setInterval(100)

    def _on_connected(self, uri):
        self.status_label.setText(f'Connected to {uri}')
        self.cf.supervisor.send_arming_request(True)
        self._timer.start()

    def _show_frame(self, img):
        if img.ndim == 2:
            h, w = img.shape
            qimg = QtGui.QImage(img.data, w, h, w, QtGui.QImage.Format.Format_Grayscale8)
        else:
            h, w, _ = img.shape
            qimg = QtGui.QImage(img.data, w, h, w * 3, QtGui.QImage.Format.Format_RGB888)
        self.image_label.setPixmap(QtGui.QPixmap.fromImage(qimg.scaled(w * 2, h * 2)))

    def _send_setpoint(self):
        self.cf.commander.send_hover_setpoint(
            self.hover['x'], self.hover['y'],
            self.hover['yaw'], self.hover['height'])

    def _update_velocity(self):
        K = QtCore.Qt.Key
        vx = (K.Key_Up in self._held) * SPEED - (K.Key_Down in self._held) * SPEED
        vy = (K.Key_Left in self._held) * SPEED - (K.Key_Right in self._held) * SPEED
        yaw = (K.Key_D in self._held) * 70.0 - (K.Key_A in self._held) * 70.0
        self.hover['x'], self.hover['y'], self.hover['yaw'] = vx, vy, yaw

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        k = event.key()
        if k == QtCore.Qt.Key.Key_Space:
            self.cf.commander.send_stop_setpoint()
            self._timer.stop()
        elif k == QtCore.Qt.Key.Key_W:
            self.hover['height'] += 0.1
        elif k == QtCore.Qt.Key.Key_S:
            self.hover['height'] -= 0.1
        else:
            self._held.add(k)
            self._update_velocity()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        self._held.discard(event.key())
        self._update_velocity()

    def closeEvent(self, event):
        self._timer.stop()
        self.cf.commander.send_stop_setpoint()
        self.cf.close_link()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = FPVWindow()
    win.show()
    sys.exit(app.exec())