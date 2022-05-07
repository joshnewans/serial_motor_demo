# ROS/Arduino Serial Motor Demo

This is demonstration of a ROS 2 interface to an Arduino running differential-drive motor control code.

The corresponding Arduino code can be found [here](https://github.com/joshnewans/ros_arduino_bridge), which is itself a fork of [this repo](https://github.com/hbrobotics/ros_arduino_bridge), which also contains a similar implementation for the ROS/Python/Client side (ROS 1 though).

## Components

The `serial_motor_demo` package consists of two nodes, `driver.py` and `gui.py`. The idea is that the driver can be run on an onboard PC inside a robot (e.g. a Raspberry Pi), interfacing with the lower-level hardware. The driver exposes motor control through ROS topics (see below), which are to be published by the user's software.

The GUI provides a simple interface for development and testing of such a system. It publishes and subscribes to the appropriate topics.


## Driver configuration & usage

The driver has a few parameters:

- `encoder_cpr` - Encoder counts per revolution
- `loop_rate` - Execution rate of the *Arduino* code (see Arduino side documentation for details)
- `serial_port` - Serial port to connect to (default `/dev/ttyUSB0`)
- `baud_rate` - Serial baud rate (default `57600`)
- `serial_debug` - Enables debugging of serial commands (default `false`)

To run, e.g.
```
ros2 run serial_motor_demo driver --ros-args -p encoder_cpr:=3440 -p loop_rate:=30 -p serial_port:=/dev/ttyUSB0 -p baud_rate:=57600
```

It makes use of the following topics
- `motor_command` - Subscribes a `MotorCommand`, in rads/sec for each of the two motors
- `motor_vels` - Publishes a `MotorVels`, motor velocities in rads/sec
- `encoder_vals` - Publishes an `EncoderVals`, raw encoder counts for each motor



## GUI Usage

Has two modes, one for raw PWM input (-255 to 255) and one for closed-loop control. In this mode you must first set the limits for the sliders.


## TODO

- Add service for encoder reset
- Add service for updating PID parameters
- Stability improvements
- More parameterisation



