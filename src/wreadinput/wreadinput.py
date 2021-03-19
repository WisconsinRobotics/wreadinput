from typing import List, cast

import evdev
import rospy
from wready import WReadyClient

from wreadinput import AxisPublisher, ControlPublisher, DeviceCaps, DeviceFinder, DeviceShape, InputDevice, KeyPublisher

class WReadInput:
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
                devs = list(filter(lambda d: shape.match(cast(DeviceCaps, d.capabilities())), devs))
                rospy.loginfo(f'Matched {len(devs)} correctly-shaped devices')
                
                rospy.loginfo(f'Waiting for an input...')
                with DeviceFinder(devs) as dev_finder:
                    task.report_progress(f'Press any button on controller "{name}"!', 0.5)
                    device = dev_finder.get()
                    rospy.loginfo(f'Acquired device: {device.name}')
                    return WReadInput(InputDevice(device), shape)
        finally:
            wready.kill()

    def spin(self, ns: str, spin_rate: float = 60):
        rospy.loginfo('Initializing ROS transports...')
        controls: List[ControlPublisher] = []
        for axis, name in self._shape.axes.items():
            controls.append(AxisPublisher(self._device, axis, name))
        for key, name in self._shape.keys.items():
            controls.append(KeyPublisher(self._device, key, name))

        rospy.loginfo('Starting evdev thread...')
        self._device.start()

        rospy.loginfo('Spinning...')
        sleeper = rospy.Rate(spin_rate)
        while not rospy.is_shutdown():
            for publisher in controls:
                publisher.publish()
            sleeper.sleep()
