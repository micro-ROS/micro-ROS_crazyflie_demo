# micro-ROS Crazyflie demo

This demos aims to show the benefits of micro-ROS regarding its low resource consumption and its extensible and modular communication system.
In particular, it is focused on the micro-ROS's middleware layer where [eProsima Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS) is the default implementation.
This software, base on the [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/About-DDS-XRCE/) wire protocol, offers to micro-ROS client-server communication with the following characteristics:

* Multi-transport protocol support (UDP, TCP and Serial).
* Peer-to-peer communication.
* Server discovery.
* Best-effort and reliable communication.
* Message fragmentation.

Each one of the aforementioned characteristics will be used around this demo.

## Scenario

An *MAV (Micro Aerial Vehicle)* overflies a given area commanded by a *Flight Operator* through a *GCS (Ground Control Station)*.
*Remote Sensors*, distributed over the area, takes environmental measures (temperature, pressure and humidity).
The *Flight Operator* shall command the *MAV* toward the *Remote Sensors* and once positioned over them, the *MAV* shall establish a connection with the *Remote Sensors* in order to gather its data.
Finally, the *Fligh Operator* shall command the *MAV* toward the home position.

![](https://raw.githubusercontent.com/micro-ROS/micro-ROS_crazyflie_demo/dashing/assets/images/drone_use-case_diagram.png)

## Actors & Topics

In this demo there are three different micro-ROS actors (*MAV*, *GCS* and *Remote Sensors*) which publish/subscribe to/from six topics:

* `/drone/odometry`: *MAV*'s odometry.
* `/drone/attitude`: *MAV*'s attitude.
* `/flight/cmd`: flight commands.
* `/sensor/temperature`: *Remote Sensors*' temperature.
* `/sensor/pressure`: *Remote Sensors*' pressure.
* `/sensor/humidity`: *Remote Sensors*' humidity.

### MAV

A [Crazyflie 2.1](https://www.bitcraze.io/crazyflie-2-1/) running a micro-ROS-Client application which is in charge of publishing its attitude and odometry, and subscribing to flight commands and *Remote Sensor*' data.

### GCS

A general-purpose computer running a micro-ROS-Agent application which is in charge of publishing flight commands, and subscribing to *MAV*'s attitude and odometry data.

### Remote Sensors

A [SparkFun Wheather Station](https://www.sparkfun.com/products/13956) connecting to a [Raspberry Pi 3A+](https://www.raspberrypi.org/products/raspberry-pi-3-model-a-plus/) which is running a micro-ROS-Agent and a micro-ROS-Client application in charge of publishing the *Remote Sensors*' data.

The figure below shows the current status of this demo.

![](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/micro-ROS/micro-ROS_crazyflie_demo/master/assets/diagrams/architecture.puml)

## Communications

In this demo there are two different kind of communication between its actors.

On the one hand, the link between the *GCS* and the *MAV* follows a client-server communication pattern.
The *GCS* works as a server using a **micro-ROS-Agent** application, while the *MAV* works as a client through a **micro-ROS-Client** application.

On the other hand, the link between the *MAV* and the *Remote Sensors* follows a peer-to-peer pattern.
Both actors works as clients communicating through a micro-ROS-Agent application running on the *Remote Sensors* side.
In that case, the micro-ROS-Agent application works as a centralized broker where the *MAV* and the *Remote Sensors* exchange its topics without ROS 2 output.
This application also allows the *MAV* to discover the *Remote Sensors* dynamically.

## Hardware

The following is a list of the hardware needed to reproduce this demo:

* 1 x [Crazyflie 2.1](https://www.bitcraze.io/crazyflie-2-1/),
* 1 x [Crazyradio PA](https://www.bitcraze.io/crazyradio-pa/),
* 1 x [Flow deck v2](https://www.bitcraze.io/flow-deck-v2/),
* 1 x [Crazyflie-compatible remote controller](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/inputdevices/).

## How to build and flash the firmware?

*NOTE: Take into account that Crazyflie firmware of NRF51 radio device should be updated to this [commit](https://github.com/bitcraze/crazyflie2-nrf-firmware/commit/b1420de7511d5a9a79f989b1f142593da3c51e22)*

1. Run the builder Docker:
```bash
docker-compose run --rm cf_builder
```

2. Build the micro-ROS firmware inside the Docker:
```bash
ros2 run micro_ros_setup configure_firmware.sh crazyflie_demo
ros2 run micro_ros_setup build_firmware.sh
```

3. Put the Crazyflie in DFU mode following the [official instructions](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/dfu/).

4. Flash the micro-ROS firmware:
```bash
ros2 run micro_ros_setup flash_firmware.sh
```

## How to prepare the Raspberry Pi applications?

To compile the required applications, so to say the specific Micro-ROS agent and client applications to publish the sensors data:

1. Launch an instance of the *micro-ROS/base* docker container:
```bash
docker run -ti --rm microros/base:foxy
```

2. Create a *raspbian* workspace inside the container:
```bash
root@microros:/uros_ws# ros2 run micro_ros_setup create_firmware_ws.sh raspbian buster_v8
```

3. On a first stage, build the `weather_agent` executable:
```bash
root@microros:/uros_ws# ros2 run micro_ros_setup configure_firmware.sh weather_agent
root@microros:/uros_ws# ros2 run micro_ros_setup build_firmware.sh
```

4. The resulting binary file is located under `/uros_ws/firmware/bin`. Copy it to the RPi via ssh's `scp`.

5. Perform steps 3 and 4, but replacing *weather_agent* application for *weather_publisher*.

6. Copy the Python 3 script `uros_cf_bridge_no_joystick.py` into the RPi, located in this repository under `dockerfile/client` folder.

7. If your Raspberry Pi does not have the I2C port enabled, you will need to do it before connecting the Sparkfun weather station.
   This can be easily achieved executing `sudo raspi-config` and opening up the I2C port.
   After that, connect the weather station board, taking into consideration the pin layout:

```
        3.3Vcc  [1][2]
        I2C SDA [3][4]
        I2C SCL [5][6] GND
```

## How to use?

To start the application just three steps are needed:

1. Install and connect the Crazyradio PA (it require [setting](https://github.com/bitcraze/crazyflie-lib-python#platform-notes) udev permissions).

2. Connect a Crazyradio antenna to the RPi. On three different ssh terminals attached to the Raspberry Pi, execute the following commands:

```bash
pi@raspberry:~/$ python3 uros_cf_bridge_joystick.py
pi@raspberry:~/$ sudo ./weather_agent /dev/ttyS10
pi@raspberry:~/$ ./weather_publisher
```

3. Up the Docker Compose. Remember to give permissions to Docker to access the X display server:

```bash
xhost +
docker-compose up -d
```
4. [Connect](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#config-client) to the Crazyflie.

To stop the application just down the Docker Compose:

```bash
docker-compose down
```

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

There are no known limitations.