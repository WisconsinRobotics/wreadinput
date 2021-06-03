from typing import ContextManager, List, Optional, cast

import evdev
import rospy
from wready import WReadyClient

from .control import AxisPublisher, ControlPublisher, KeyPublisher
from .device import InputDevice
from .finder import DeviceFinder
from .shape import DeviceShape
from .util.evdev_const import DeviceCaps

class WReadInput(ContextManager['WReadInput']):
    """The WReadInput main object.
    
    An instance of WReadInput wraps an input device and publishes data about its
    controls to ROS based on the control names as defined in the device's shape.

    The static `find` function automatically finds a device by asking a human operator
    to press a button on the desired input device, synchronizing with WReady to avoid
    conflicts with other WReadInput instances.

    WReadInput instances should be used as context managers in order to clean up the
    input device when done, as in::

        with WReadInput.find(dev_name, dev_shape, wready_ns) as wreadinput:
            wreadinput.spin(dev_ns)
    """
    
    def __init__(self, device: InputDevice):
        """Constructs a new WReadInput system wrapping the given input device.

        The names for the controls given by the device shape determine the names of
        the topics to which control data is published.

        Parameters
        ----------
        device : InputDevice
            The input device to publish control data for.
        """
        self._device = device
        self._shape = device.shape

    @staticmethod
    def find(name: str, shape: DeviceShape, wready_server_ns: str) -> 'WReadInput':
        """Submits a WReady task that finds an input device by operator intervention.

        This function will first synchronize with WReady to avoid conflicts with other
        WReadInput instances. Once the task is scheduled, this function will search
        for all devices on the system matching the given device shape, then prompt for
        a human operator to press a button on the desired device. Once a button is
        pressed, the device is wrapped in a WReadInput instance, which is returned.

        Parameters
        ----------
        name : str
            The name of the device. Only used for the prompt in order to differentiate
            between multiple WReadInput instances that may be simultaneously searching
            for devices.
        shape : DeviceShape
            The shape of the device to search for.
        wready_server_ns : str
            The ROS namespace of the WReady server to synchronize with.

        Returns
        -------
        WReadInput
            A WReadInput instance wrapping the discovered input device.

        Raises
        ------
        ValueError
            If no accessible input devices with the correct shape exist on the system.
            It is possible that some matching devices exist, but that the WReadInput
            process does not have permission to access them.
        """
        wready = WReadyClient(wready_server_ns)
        try:
            with wready.request_sync(f'Controller Configuration') as task:
                rospy.loginfo(f'Searching for devices...')
                task.report_progress(f'Polling for controllers...')
                devs = [evdev.InputDevice(dev_file) for dev_file in evdev.list_devices()]
                rospy.loginfo(f'Discovered {len(devs)} devices')
                
                found_device: Optional[evdev.InputDevice] = None
                try:
                    devs = list(filter(lambda d: shape.match(cast(DeviceCaps, d.capabilities())), devs))
                    rospy.loginfo(f'Matched {len(devs)} correctly-shaped devices')
                    if len(devs) == 0:
                        task.report_progress('No correctly-shaped devices could be found!', 1)
                        raise ValueError('No correctly-shaped devices could be found!')
                    
                    rospy.loginfo(f'Waiting for an input...')
                    with DeviceFinder(devs) as dev_finder:
                        task.report_progress(f'Press any button on controller "{name}"!', 0.5)
                        found_device = dev_finder.get()
                        rospy.loginfo(f'Acquired device: {found_device.name}')
                        return WReadInput(InputDevice(found_device, shape))
                finally:
                    for dev in devs:
                        if dev is not found_device:
                            dev.close()
        finally:
            wready.kill()

    def spin(self, ns: Optional[str] = None, spin_rate: float = 60):
        """Spins, publishing device control data to ROS.

        This method will not terminate until the ROS node has shut down.

        Parameters
        ----------
        ns : Optional[str], optional
            The ROS namespace to publish data to. By default, the node's private
            namespace.
        spin_rate : float, optional
            The frequency, in Hz, at which control data should be published. By
            default, 60 Hz.
        """
        namespace = '~' if ns is None else f'{ns}/'

        rospy.loginfo('Initializing ROS transports...')
        controls: List[ControlPublisher] = []
        for axis, name in self._shape.axes.items():
            controls.append(AxisPublisher(self._device, axis, f'{namespace}axis/{name}'))
        for key, name in self._shape.keys.items():
            controls.append(KeyPublisher(self._device, key, f'{namespace}button/{name}'))

        rospy.loginfo('Starting evdev thread...')
        self._device.start()

        rospy.loginfo('Spinning...')
        sleeper = rospy.Rate(spin_rate)
        while not rospy.is_shutdown():
            for publisher in controls:
                publisher.publish()
            sleeper.sleep()
        rospy.loginfo('Cleaning up...')

    def __enter__(self) -> 'WReadInput':
        return self

    def __exit__(self, *exc):
        self._device.kill()
