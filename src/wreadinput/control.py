from abc import ABC, abstractmethod

import rospy
from std_msgs.msg import Bool, Float32

from .device import InputDevice
from .util.evdev_const import DeviceAxis, DeviceKey

class ControlPublisher(ABC):
    """Publishes info about a particular control to a ROS topic.
    
    Used by WReadInput's default node implementation.
    """
    
    @abstractmethod
    def publish(self):
        """Publishes the control state to ROS."""
        raise NotImplementedError()

class AxisPublisher(ControlPublisher):
    """Publishes data about an absolute axis to a ROS topic."""
    
    def __init__(self, device: InputDevice, axis: DeviceAxis, topic: str):
        """Creates a new axis publisher for the given absolute axis.

        Parameters
        ----------
        device : InputDevice
            The device to get axis data from.
        axis : DeviceAxis
            The absolute axis to publish data for.
        topic : str
            The topic to publish axis data to.
        """
        self._device = device
        self._axis = axis
        self._pub = rospy.Publisher(topic, Float32, queue_size=10)
    
    def publish(self):
        state = self._device.get_axis(self._axis)
        if state is not None:
            self._pub.publish(Float32(state))

class KeyPublisher(ControlPublisher):
    """Publishes data about a button to a ROS topic."""
    
    def __init__(self, device: InputDevice, key: DeviceKey, topic: str):
        """Creates a new key publisher for the given button.

        Parameters
        ----------
        device : InputDevice
            The device to get button data from.
        key : DeviceKey
            The button to publish data for.
        topic : str
            The topic to publish button data to.
        """
        self._device = device
        self._key = key
        self._pub = rospy.Publisher(topic, Bool, queue_size=10)
    
    def publish(self):
        state = self._device.get_key(self._key)
        if state is not None:
            self._pub.publish(Bool(state))
