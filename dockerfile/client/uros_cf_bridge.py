"""
Headless client for the Crazyflie.
"""
import logging
import os
import signal
import sys
import serial
import pty
import threading
import time
import subprocess

import cflib.crtp
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crazyflie import Crazyflie

import argparse

os.environ["SDL_VIDEODRIVER"] = "dummy"

class CrazyflieBridgedController():

    def __init__(self, link_uri, radio_port, serialport, jr, xmode=True):
        """Initialize the headless client and libraries"""
        self._cf = Crazyflie()
        self._jr = jr
        self._serial = serialport

        self._cf.commander.set_client_xmode(xmode)
        self._cf.open_link(link_uri)
        self._port = radio_port
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
                pk.port = self._port
                pk.data = data
                self._cf.send_packet(pk)

    def _data_received(self, pk):
        if pk.port == self._port:
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
    parser = argparse.ArgumentParser("")
    parser.add_argument("--channel", help="Crazyradio configured channel", type=str, default="65")
    parser.add_argument("--port", help="Crazyradio configured port", type=int, default=10)
    parser.add_argument("--controller", help="Use controller to command crazyflie", action='store_true')
    args = parser.parse_args()

    master, slave = pty.openpty()
    m_name = os.ttyname(master)
    s_name = os.ttyname(slave)

    # Save serial port name to file
    with open("/tmp/uros_port.log", 'w+') as file:
        file.write(s_name)
        print("Using " + s_name + " for micro-ROS agent connection")

    # Set logging level
    logging.basicConfig(level=logging.INFO)

    # Open serial port
    serialport = serial.Serial(s_name, baudrate=115200, timeout=1)
    serialport.fd = master

    # Retrieve controller if configured
    if args.controller:
        from cfclient.utils.input import JoystickReader

        jr = JoystickReader(do_device_discovery=False)
        available_devices = jr.available_devices()
        if len(available_devices):
            used_device = available_devices[0]
            print("Using input device: " + str(used_device))
            jr.device_error.add_callback(lambda msg: print("Input device error: " + str(msg)))
            jr.start_input(used_device.name)
            jr.set_input_map(used_device.name, "PS3_Mode_1")
        else:
            print("No input device detected")
            jr = None
    else:
        jr = None

    # Configure radio link
    cflib.crtp.radiodriver.set_retries_before_disconnect(-1)
    cflib.crtp.radiodriver.set_retries(100)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    link_uri = "radio://0/{}/2M".format(args.channel)
    radio_port = args.port

    print("Connecting to Crazyflie using {}, with port {}".format(link_uri, radio_port))

    try:
        while True:
            cf = CrazyflieBridgedController(link_uri, radio_port, serialport, jr)
            while cf.is_connected:
                time.sleep(0.1)
            print("Disconected from: " + str(link_uri))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
