# The micro-ROS_crazyflie_demo repository

Coming soon...

![](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/micro-ROS/micro-ROS_crazyflie_demo/master/assets/diagrams/architecture.puml)

## Purpose of the project

The software is not ready for a production use.
It has neither been developed nor tested for a specific use case.
However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust ot according to any applicable safety standards (e.g. ISO 26262).

## Hardware

The following is a list of the hardware needed to reproduce this demo:

* 1 x [Crazyflie 2.1](https://www.bitcraze.io/crazyflie-2-1/),
* 1 x [Crazyradio PA](https://www.bitcraze.io/crazyradio-pa/),
* 1 x [Flow deck v2](https://www.bitcraze.io/flow-deck-v2/),
* 1 x [Crazyflie-compatible remote controller](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/inputdevices/).

## How to build and flash the firmware?

1. Run the builder Docker:
```bash
docker-compose run cf_builder
```

2. Build the micro-ROS firmware inside the Docker:
```bash
ros2 run micro_ros_setup configure_firmware.sh crazyflie_position_publisher
ros2 run micro_ros_setup build_firmware.sh
```

3. Put the Crazyflie in DFU mode following the [official instructions](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/dfu/).

4. Flash the micro-ROS firmware:
```bash
ros2 run micro_ros_setup flash_firmware.sh
```

## How to use?

To start the application just three step are needed:

1. Install and connect the Crazyradio PA (it require [setting](https://github.com/bitcraze/crazyflie-lib-python#platform-notes) udev permissions).

2. Up the Docker Compose:

```bash
docker-compose up -d
```
3. [Connect](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#config-client) to the Crazyflie.

To stop the application just down the Docker Compose:

```bash
docker-compose down
```
