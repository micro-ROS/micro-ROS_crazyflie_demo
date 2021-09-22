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

class CrazyflieController():

    def __init__(self, link_uri, radio_port, serialport, xmode=True):
        """Initialize the headless client and libraries"""
        from cfclient.utils.input import JoystickReader
        self._cf = Crazyflie()
        self._serial = serialport

        self._cf.commander.set_client_xmode(xmode)
        self._cf.open_link(link_uri)
        self._port = radio_port
        self.is_connected = True
        self.running = True
        self._console = ""

        # Callbacks
        self._cf.connected.add_callback(lambda msg: print("Connected to " + msg))
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.console.receivedChar.add_callback(self._console_char)
        self._cf.packet_received.add_callback(self._data_received)

        # Search for controllers
        self._jr = JoystickReader(do_device_discovery=False)
        available_devices = self._jr.available_devices()
        if len(available_devices):
            used_device = available_devices[0]
            print("Using input device: " + str(used_device))
            self._jr.device_error.add_callback(lambda msg: print("Input device error: " + str(msg)))
            self._jr.start_input(used_device.name)
            self._jr.set_input_map(used_device.name, "xbox360_mode1")

            # Controller callback
            self._jr.input_updated.add_callback(self._cf.commander.send_setpoint)
        else:
            print("No input device detected")

        self.serialthread = threading.Thread(target=self._serial_listener)
        self.serialthread.start()

    def stop(self):
        self.running = False
        self.serialthread.join()
        self._cf.close_link()

    def _console_char(self, pk):
        self._console = self._console + str(pk)
        if "\n" in self._console:
            print("Console: " + str(self._console ), end="")
            self._console = ""

    def _disconnected(self, link_uri):
        print("Disconnected...")
        if self.running:
            self.stop()

    def _connection_failed(self, link_uri, msg):
        print("Conection failed...")
        if self.running:
            self.stop()

    def _serial_listener(self):
        while self.running and self.is_connected:
            data = self._serial.read_until(size=30)
            if(data):
                pk = CRTPPacket()
                pk.port = self._port
                pk.data = data
                pk.size = len(data)
                self._cf.send_packet(pk)
                #print("Got data from agent: len {}, data {}".format(pk.size, pk.data))

    def _data_received(self, pk):
        if pk and pk.port == self._port:
            self._serial.write(pk.data)
            self._serial.flush()
            #print("Got data from crazyflie: len {}, data {}".format(len(pk.data), pk.data))

class CrazyflieBridge():
    def __init__(self, link_uri, radio_port, serialport):
        """Initialize the headless client and libraries"""
        self._serial = serialport
        self._port = radio_port
        self.is_connected = True
        self.running = True

        self.link = cflib.crtp.get_link_driver(link_uri, None, None)

        self.serialthread = threading.Thread(target=self._serial_listener)
        self.serialthread.start()

        self.cfthread = threading.Thread(target=self._cf_listener)
        self.cfthread.start()

    def stop(self):
        self.running = False
        self.serialthread.join()
        self.cfthread.join()
        self.link.close()

    def _serial_listener(self):
        while self.running and self.is_connected:
            data = self._serial.read_until(size=30)
            if(data):
                pk = CRTPPacket()
                pk.port = self._port
                pk.data = data
                pk.size = len(data)
                self.link.send_packet(pk)
                #print("Got data from agent: len {}, data {}".format(pk.size, pk.data))

    def _cf_listener(self):
        while self.running and self.is_connected:
            pk = self.link.receive_packet(1)
            if pk and pk.port == self._port:
                self._serial.write(pk.data)
                self._serial.flush()
                #print("Got data from crazyflie: len {}, data {}".format(len(pk.data), pk.data))
    
def main():
    parser = argparse.ArgumentParser("")
    parser.add_argument("--channel", help="Crazyradio configured channel", type=str, default="65")
    parser.add_argument("--port", help="Crazyradio configured port", type=int, default=9)
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
    serialport = serial.Serial(s_name, baudrate=115200, timeout=0.01)
    serialport.fd = master

    # Configure radio link
    cflib.crtp.radiodriver.set_retries_before_disconnect(-1)
    cflib.crtp.radiodriver.set_retries(100)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    link_uri = "radio://0/{}/2M".format(args.channel)
    radio_port = args.port

    print("Connecting to Crazyflie using {}, with port {}".format(link_uri, radio_port))

    try:
        while True:
            if args.controller:
                cf = CrazyflieController(link_uri, radio_port, serialport)
            else:
                cf = CrazyflieBridge(link_uri, radio_port, serialport)
            while cf.is_connected:
                time.sleep(0.1)
            print("Disconected from: " + str(link_uri))
    except KeyboardInterrupt:
        pass
    finally:
        print("Exiting")
        cf.stop()

if __name__ == "__main__":
    main()
