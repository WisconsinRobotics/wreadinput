# WReadInput

This is the ROS input device library for Wisconsin Robotics.
The nodes in this package read from specific input devices (e.g. xbox controllers) using the Linux evdev interface and relay the data to ROS topics.
This is done with the help of the [python-evdev](https://github.com/gvalkov/python-evdev) package, which can be installed using `pip`.
Controller initialization is handled by [WReady](https://github.com/WisconsinRobotics/wready), so some WReady server implementation must be present in the robot system where WReadInput is used.
This software is designed for ROS Noetic and Python 3.8.x.

## Usage

WReadInput comes with various pre-packaged nodes that handle specific input devices.
For example, the `input_logx3dpro.py` node handles the Logitech Extreme 3D Pro joystick.
Nodes for other devices with evdev support can easily be implemented; see the sections further below for more details.

These nodes take three parameters, all of which are optional.
These are:

* `name : str`: A human-readable name identifying the device. By default, uses the node name.
* `namespace : str`: The ROS namespace to publish device data to. By default, uses the node's private namespace.
* `wready_ns : str`: The ROS namespace for the WReady server to use for device binding. By default, uses `/wready`.

When one of these node runs, it will first request an initialization task with WReady in order to bind to a specific input device.
During this init task, a human operator should press any button on the input device that the node should bind to.
Once this is done, the node will immediately begin publishing the device state.

## Package Structure

WReadInput is approximately divided into two layers of abstraction: the evdev wrapper and the ROS node.
For more details, check out the source code for the modules mentioned below.

### Evdev Wrapper

The evdev wrapper is responsible for handling input devices through the evdev interface.
The main class representing a device is `InputDevice`, in the `device` module.
Instances of this class maintain a polling thread for consuming evdev events and tracking device state.
Code from other threads can then retrieve the device state using the methods exposed by `InputDevice`.
Various useful evdev constants representing events and inputs are available in the `util.evdev_const` module.

### ROS Node

The ROS node is responsible for relaying data from the input device to ROS.
The main class representing a node is `WReadInput`, in the `wreadinput` module.
Instances of this class maintain an `InputDevice` instance, polling it for data and publishing to ROS topics.
Devices are defined using a `DeviceShape`, from the `shape` module, which represents a set of axes and buttons on a particular device.

Furthermore, a simple full node implementation exists in the `default_node` module, which defines a `main` method that performs all the necessary operations for a minimal input device node.
A node using this implmentation needs only to pass a node name and the shape of the desired device.
The pre-packaged nodes shipped with WReadInput use this default implementation.
