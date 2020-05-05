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

import cfclient.utils
import cflib.crtp
from cfclient.utils.input import JoystickReader
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crazyflie import Crazyflie

if os.name == 'posix':
    print('Disabling standard output for libraries!')
    stdout = os.dup(1)
    os.dup2(os.open('/dev/null', os.O_WRONLY), 1)
    sys.stdout = os.fdopen(stdout, 'w')

# set SDL to use the dummy NULL video driver,
#   so it doesn't need a windowing system.
os.environ["SDL_VIDEODRIVER"] = "dummy"

CRTP_PORT_MICROROS = 9

class HeadlessClient():
    """Crazyflie headless client"""

    def __init__(self, serial_dev):
        """Initialize the headless client and libraries"""
        self._jr = JoystickReader(do_device_discovery=False)

        self._cf = Crazyflie()
        self._cf.packet_received.add_callback(self._data_received)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.disconnected.remove_callback(self._cf._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        # self._cf.link_quality_updated.add_callback(lambda x: [])
        self._cf.console.receivedChar.add_callback(self._console_char)
        self._jr.input_updated.add_callback(self._cf.commander.send_setpoint)

        self._cf.param.add_update_callback(
            group="imu_sensors", name="HMC5883L", cb=(
                lambda name, found: self._jr.set_alt_hold_available(
                    eval(found))))
        self._jr.assisted_control_updated.add_callback(
            lambda enabled: self._cf.param.set_value("flightmode.althold",
                                                     enabled))
        
        self._serial_dev = serial_dev
        self._serial = serial.Serial(serial_dev, baudrate=115200)
        self.is_connected = False

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self._devs = []

        for d in self._jr.available_devices():
            self._devs.append(d.name)
        
        threading.Thread(target=self._serial_listener).start()

    def setup_controller(self, input_config, input_device=0, xmode=False):
        """Set up the device reader"""
        # Set up the joystick reader
        self._jr.device_error.add_callback(self._input_dev_error)
        print("Client side X-mode: %s" % xmode)
        if (xmode):
            self._cf.commander.set_client_xmode(xmode)

        devs = self._jr.available_devices()  # noqa, is this a bug?
        print("Will use [%s] for input" % self._devs[input_device])
        self._jr.start_input(self._devs[input_device])
        self._jr.set_input_map(self._devs[input_device], input_config)

    def controller_connected(self):
        """ Return True if a controller is connected"""
        return True if (len(self._jr.available_devices()) > 0) else False

    def list_controllers(self):
        """List the available controllers and input mapping"""
        print("\nAvailable controllers:")
        for i, dev in enumerate(self._devs):
            print(" - Controller #{}: {}".format(i, dev))
        print("\nAvailable input mapping:")
        for map in os.listdir(cfclient.config_path + '/input'):
            print(" - " + map.split(".json")[0])

    def connect_crazyflie(self, link_uri):
        """Connect to a Crazyflie on the given link uri"""
        self.link_uri = link_uri
        self._console = ""
        self.is_connected = True
        self._cf.open_link(link_uri)
    
    def _console_char(self, pk):
        self._console = self._console + str(pk)
        if "\n" in self._console:
            print("Console: " + str(self._console ), end="")
            self._console = ""

    def _serial_listener(self):
        while True:
            try:
                if self._serial and self._serial.is_open and self.is_connected:
                    data = self._serial.read(size=30)
                    if(data):
                        pk = CRTPPacket()
                        pk.port = CRTP_PORT_MICROROS
                        pk.data = data
                        self._cf.send_packet(pk)
                else:
                    time.sleep(0.1)
            except:
                pass

    def _data_received(self, pk):
        if pk.port == CRTP_PORT_MICROROS:
            self._serial.write(pk.data)
            self._serial.flush()

    def _input_dev_error(self, message):
        """Callback for an input device error"""
        print("Error when reading device: {}".format(message))
        # sys.exit(-1)
        self.is_connected = True

    def _connection_failed(self, link_uri, msg):
        pass

    def _connection_lost(self, link_uri, error):
        pass

    def _disconnected(self, link_uri):
        pass



def main():
    """Main Crazyflie headless application"""
    import argparse

    parser = argparse.ArgumentParser(prog="cfheadless")
    parser.add_argument("-u", "--uri", action="store", dest="uri", type=str,
                        default="radio://0/65/2M",
                        help="URI to use for connection to the Crazyradio"
                             " dongle, defaults to radio://0/65/2M")
    parser.add_argument("-i", "--input", action="store", dest="input",
                        type=str, default="PS3_Mode_1",
                        help="Input mapping to use for the controller,"
                             "defaults to PS3_Mode_1")
    parser.add_argument("-d", "--debug", action="store_true", dest="debug",
                        help="Enable debug output")
    parser.add_argument("-c", "--controller", action="store", type=int,
                        dest="controller", default=0,
                        help="Use controller with specified id,"
                             " id defaults to 0")
    parser.add_argument("--controllers", action="store_true",
                        dest="list_controllers",
                        help="Only display available controllers and exit")
    parser.add_argument("-x", "--x-mode", action="store_true",
                        dest="xmode",
                        help="Enable client-side X-mode")
    (args, unused) = parser.parse_known_args()

    cflib.crtp.radiodriver.set_retries_before_disconnect(-1)
    cflib.crtp.radiodriver.set_retries(100)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    ser = '/dev/ttyS11'

    try:
        f = open("/.env/variables.env", "w+")
        f.write("SERIAL_DEV=%s" % slave_name)
        f.close()
    except:
        print("No /.env/variables.env wrote")

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    headless = HeadlessClient(ser)

    if (args.list_controllers):
        headless.list_controllers()
    else:
        if headless.controller_connected():
            headless.setup_controller(input_config=args.input,
                                    input_device=args.controller,
                                    xmode=args.xmode)

            while True:
                headless.connect_crazyflie(link_uri=args.uri)
                while True:
                    time.sleep(1)
        else:
            print("No input-device connected, exiting!")


if __name__ == "__main__":
    main()
