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
    def __init__(self, device: InputDevice, shape: DeviceShape):
        self._device = device
        self._shape = shape

    @staticmethod
    def find(name: str, shape: DeviceShape, wready_server_ns: str) -> 'WReadInput':
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
                        return WReadInput(InputDevice(found_device), shape)
                finally:
                    for dev in devs:
                        if dev is not found_device:
                            dev.close()
        finally:
            wready.kill()

    def spin(self, ns: Optional[str] = None, spin_rate: float = 60):
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
