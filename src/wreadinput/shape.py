from typing import Collection, Dict, Set

from .util import evdev_util
from .util.evdev_const import DeviceAxis, DeviceCaps, DeviceEventType, DeviceKey

class DeviceShape:
    def __init__(self):
        self.axes: Dict[DeviceAxis, str] = {}
        self.keys: Dict[DeviceKey, str] = {}
        self._names: Set[str] = set()

    def with_axis(self, axis: DeviceAxis, name: str) -> 'DeviceShape':
        if name in self._names:
            raise ValueError(f'A control named "{name}" already exists!')
        self.axes[axis] = name
        self._names.add(name)
        return self

    def with_key(self, key: DeviceKey, name: str) -> 'DeviceShape':
        if name in self._names:
            raise ValueError(f'A control named "{name}" already exists!')
        self.keys[key] = name
        self._names.add(name)
        return self

    def match(self, caps: DeviceCaps) -> bool:
        return check_caps(caps, DeviceEventType.EV_ABS, self.axes.keys())\
            and check_caps(caps, DeviceEventType.EV_KEY, self.keys.keys())

def check_caps(caps: DeviceCaps, ev_type: DeviceEventType, expected: Collection[int]) -> bool:
    if len(expected) > 0:
        if ev_type not in caps:
            return False
        supported_codes = set(evdev_util.get_capability_codes(caps[ev_type]))
        for exp_code in expected:
            if exp_code not in supported_codes:
                return False
    return True
