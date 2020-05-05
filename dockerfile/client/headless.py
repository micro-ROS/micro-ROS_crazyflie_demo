"""
Headless client for the Crazyflie.
"""
import logging
import os
import signal
import sys
import serial
import threading
import time
import subprocess

import cfclient.utils
import cflib.crtp
from cfclient.utils.input import JoystickReader
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crazyflie import Crazyflie

os.environ["SDL_VIDEODRIVER"] = "dummy"

CRTP_PORT_MICROROS = 9

class CrazyflieBridgedController():

    def __init__(self, link_uri, serialport, jr, xmode=True):
        """Initialize the headless client and libraries"""
        self._cf = Crazyflie()
        self._jr = jr
        self._serial = serialport

        self._cf.commander.set_client_xmode(xmode)
        self._cf.open_link(link_uri)
        self.is_connected = True
        self._console = ""

        # Callbacks
        self._cf.connected.add_callback(lambda msg: print("Connected to " + msg))
        self._cf.disconnected.add_callback(self._disconnected)  
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.console.receivedChar.add_callback(self._console_char)
        self._cf.packet_received.add_callback(self._data_received)
        
        # Controller callback
        if self._jr:
            self._jr.input_updated.add_callback(self._cf.commander.send_setpoint)

        self.serialthread = threading.Thread(target=self._serial_listener)
        self.serialthread.start()

    def _console_char(self, pk):
        self._console = self._console + str(pk)
        if "\n" in self._console:
            print("Console: " + str(self._console ), end="")
            self._console = ""

    def _serial_listener(self):
        while self.is_connected:
            data = self._serial.read(size=30)
            if(data):
                pk = CRTPPacket()
                pk.port = CRTP_PORT_MICROROS
                pk.data = data
                self._cf.send_packet(pk)

    def _data_received(self, pk):
        if pk.port == CRTP_PORT_MICROROS:
            self._serial.write(pk.data)
            self._serial.flush()

    def _disconnected(self, link_uri):
        print("Disconnected. Reconnecting...")
        # self._cf.open_link(link_uri)
        self.is_connected = False


    def _connection_failed(self, link_uri, msg):
        print("Conection failed. Reconnecting...")
        # self._cf.open_link(link_uri)
        self.is_connected = False


def main():
    # Creating a soca bridge
    serial_dev = '/dev/ttyS11'
    serial_dev_agent = '/dev/ttyS10'
    socat_process = subprocess.Popen(["socat", "PTY,link=/dev/ttyS10", "PTY,link=" + serial_dev])
    time.sleep(1)
    print("Use " + serial_dev_agent + " for micro-ROS agent connection")

    # Set logging level
    logging.basicConfig(level=logging.INFO)

    # Open serial port
    serialport = serial.Serial(serial_dev, baudrate=115200, timeout=0.1)

    # Retrieve controller
    jr = JoystickReader(do_device_discovery=False)
    available_devices = jr.available_devices()
    if len(available_devices):
        used_device = available_devices[0]
        print("Using input device: " + str(used_device))
        jr.device_error.add_callback(lambda msg: print("Input device error: " + str(msg)))
        jr.start_input(used_device.name)
        jr.set_input_map(used_device.name, "PS3_Mode_1")
    else:
        used_device = None
        print("Using no input device")

    # Configure radio link
    cflib.crtp.radiodriver.set_retries_before_disconnect(-1)
    cflib.crtp.radiodriver.set_retries(100)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    link_uri = "radio://0/65/2M"

    try:
        while True:
            cf = CrazyflieBridgedController(link_uri, serialport, jr if used_device else None)
            while cf.is_connected:
                time.sleep(0.1)
            print("Disconected from: " + str(link_uri))
    except KeyboardInterrupt:
        pass

    os.killpg(os.getpgid(socat_process.pid), signal.SIGTERM)


if __name__ == "__main__":
    main()
