# The Arduino Repo #

This repo is the home for any code that can be loaded on to an Arduino.
For complete documentation see the [Implementation Guide](https://www.overleaf.com/read/bhqnrsqxtznt).

### Prototype Sketches: ###

* remote_control_roboclaw_1_0_hard_limit
* remote_control_ros_turtlebot
* ros_joystick_roboclaw_1_1_hard_limit

### Final Arduino Network: ###

* ros_node_teensy
        * ROS node for Teensy 3.2
        * Publishes to:
                * arduino_error
        * Subscribes to:
                * kinect_servo
                * camera_servo_pan
                * camera_servo_tilt
* serial_to_drivetrain
* serial_to_excavation
* serial_to_vision