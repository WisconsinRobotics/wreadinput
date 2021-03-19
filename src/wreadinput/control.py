from abc import ABC, abstractmethod

import rospy
from std_msgs.msg import Bool, Float32

from .device import InputDevice
from .util.evdev_const import DeviceAxis, DeviceKey

class ControlPublisher(ABC):
    @abstractmethod
    def publish(self):
        raise NotImplementedError()

class AxisPublisher(ControlPublisher):
    def __init__(self, device: InputDevice, axis: DeviceAxis, topic: str):
        self._device = device
        self._axis = axis
        self._pub = rospy.Publisher(str, Float32, queue_size=10)
    
    def publish(self):
        state = self._device.get_axis(self._axis)
        if state is not None:
            self._pub.publish(Float32(state))

class KeyPublisher(ControlPublisher):
    def __init__(self, device: InputDevice, key: DeviceKey, topic: str):
        self._device = device
        self._key = key
        self._pub = rospy.Publisher(str, Bool, queue_size=10)
    
    def publish(self):
        state = self._device.get_key(self._key)
        if state is not None:
            self._pub.publish(Bool(state))
