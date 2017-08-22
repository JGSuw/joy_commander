# Joy Commander
Joy Commander is a ROS node for doing manual control of flying vehicles and robots. It is designed to be flexible and compatible with a wide range of controllers. It includes a feature where other nodes can register callbacks that are called when a requested button is pressed, which allows many nodes on the ROS network to respond to button presses in their own context.

The node has two modes for convenience which change how the stick information is interpreted. The default mode is attitude; in this mode the sticks are interpreted as roll, pitch, yaw rate and thrust based on a user defined stick mapping. The second mode is x-y-z (velocity), where the sticks are interpreted as the x, y, z velocities, and yaw rate.
## Requirements and Dependencies
All of the dependencies of Joy Commander come with standard ROS distributions. The Joy Commander package relies on the [ROS Joy package](http://wiki.ros.org/joy) to interface with the controller, and requires [sensor_msgs](http://wiki.ros.org/sensor_msgs) and [geoemetry_msgs](http://wiki.ros.org/geometry_msgs) for communication. Hence this node can be used to interface with any joystick that is supported by Linux.
## Installation
Clone this package into your catkin workspace source directory, and build.
## Usage
The Joy Commander receives input by subscribing to a sensor_msgs/Joy topic, and publishes the stick information to a geoemetry_msgs/Twist topic. The input topic by default is */joy* and can be changed via a parameter called *joy_sub_name*. The output topic by default is */joy_twist* and can be changed via a parameter called *twist_pub_name*. In order to use the buttons, a node needs to implement a button service (see srv/button_srv.srv), and registers that service with the Joy Commander node by calling its add button service (srv/add_button_srv.srv), which is advertised as */add_button_srv*.
### Launch File
The default joy_commander.launch launches two nodes, a Joy node, and a Joy Commander node. You can launch it this way: `roslaunch joy_commander joy_commander.launch joy_dev:=<file descriptor>` where the *joy_dev* argument references the file descriptor (without square brackets) of the joystick. This argument has a default value of */dev/input/js0*.
### Parameters
The Joy Commander uses a variety of parameters which determine the stick mapping (of the sensor_msgs/Joy/buttons array), the interpretation of stick data (attitude vs velocity), associated sign conventions, and extreme values. These are loaded from a yaml file specified in the launch file, located in /config.
