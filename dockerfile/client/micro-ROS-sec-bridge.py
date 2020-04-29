import logging
import socket
import sys
import threading
import time
import serial

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.drivers.crazyradio import Crazyradio
from cflib.crazyflie.log import LogConfig

CRTP_PORT_MICROROS = 10

# logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.ERROR)

class RadioBridge:
    def __init__(self, link_uri, serial_dev):
        self.link_uri = link_uri
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.link_quality_updated.add_callback(self._update_quality)
        self._cf.console.receivedChar.add_callback(self._console_char)
        self._cf.packet_received.add_callback(self._data_received)


        self._link_quality = 0
        self._link_quality_n = 0

        self._serial_dev = serial_dev
        self._serial = serial.Serial(serial_dev)

        self._console = ""

        # Try to connect to the Crazyflie
        self._cf.open_link(self.link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        threading.Thread(target=self._agentlistener).start()
        threading.Thread(target=self._print_quality).start()
    
    def _agentlistener(self):
        while self.is_connected:
            data = self._serial.read(size=30)
            pk = CRTPPacket()
            pk.port = CRTP_PORT_MICROROS
            pk.data = data
            self._cf.send_packet(pk)

    def _connected(self, link_uri):
        # print('Connected to %s' % link_uri)
        pass
    
    def _console_char(self, pk):
        self._console = self._console + str(pk)
        if "\n" in self._console:
            print("Console: " + str(self._console ), end="")
            self._console = ""

    def _update_quality(self, pk):
        self._link_quality = self._link_quality + pk
        self._link_quality_n = self._link_quality_n + 1

    def _print_quality(self):
        i = 0
        while self.is_connected:
            if self._link_quality_n != 0:
                
                link_average = self._link_quality/self._link_quality_n
                # print("Link quality:" + str(link_average))
                # if(link_average < 90):
                #     self._cf.close_link()
                #     self.is_connected = False
                self._link_quality = 0
                self._link_quality_n = 0
            time.sleep(1)

    def _data_received(self, pk):
        if pk.port == CRTP_PORT_MICROROS:
            if pk.channel == 0:
                self._serial.write(pk.data)

    def _connection_failed(self, link_uri, msg):
        pass

    def _connection_lost(self, link_uri, msg):
        pass

    def _disconnected(self, link_uri):
        print("Disconnected. Reconnecting...")
        self.is_connected = True
        self._serial.close()
        self._serial = serial.Serial(self._serial_dev)
        self._cf.open_link(self.link_uri)



if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.radiodriver.set_retries_before_disconnect(-1)
    cflib.crtp.radiodriver.set_retries(100)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cr = Crazyradio()
    cr.set_power(0)

    ser = '/dev/ttyS11'

    try:
        f = open("/.env/variables.env", "w+")
        f.write("SERIAL_DEV=%s" % slave_name)
        f.close()
    except:
        print("No /.env/variables.env wrote")

    while True:
        available = cflib.crtp.scan_interfaces()
        available = [x for x in available if x[0].startswith("radio")]

        if len(available) > 0:
            link_uri = available[0][0]
            link_uri = "radio://0/30/2M"
            print('Connecting to: ' + str(link_uri))
            le = RadioBridge(link_uri, ser)

            try:
                while le.is_connected:
                    time.sleep(1)
                print("Disconected from: " + str(link_uri))
            except KeyboardInterrupt:
                sys.exit(1)
        else: 
            print("No Crazyflies found by radio")
            time.sleep(1)
