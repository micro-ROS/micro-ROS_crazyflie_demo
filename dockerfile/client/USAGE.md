# GCS

Connect the Joystick controller and the Crazyradio dongle, and execute:

```bash
sudo python3 dockerfile/client/uros_cf_bridge_joystick.py
sudo ./MicroXRCEAgent serial --dev /dev/ttyS10 -v6 --refs agent.refs 
```


# RPi

Connect the Crazyradio dongle, and execute:

```bash
sudo python3 micro-ROS_crazyflie_demo/dockerfile/client/headlessbridge.py 
sudo ./weather_agent 8888 /dev/ttyS10 
./weather_publisher 
```