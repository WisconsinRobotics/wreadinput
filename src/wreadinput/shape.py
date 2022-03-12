from typing import Collection, Dict, Set

from .util import evdev_util
from .util.evdev_const import DeviceAxis, DeviceCaps, DeviceEventType, DeviceKey

class AxisDefinition:
    """Represents an absolute axis declaration in a device shape."""
    
    def __init__(self, name: str, min_val: float, max_val: float, deadband: float):
        """Creates a new axis definition with the given properties.

        Parameters
        ----------
        name : str
            The name of the axis.
        min_val : float
            The minimum value to output for the axis.
        max_val : float
            The maximum value to output for the axis.
        """
        self.name = name
        self.min_val = min_val
        self.max_val = max_val
        self.deadband = deadband

class DeviceShape:
    """Represents the set of controls that are characteristic of an input device.
    
    Each control has an associated name, which identifies the control in a human-
    readable way. The control's name is also used by WReadInput's default node
    implementation to generate the name of the topic on which the control's state
    is published.
    """
    
    def __init__(self):
        """Constructs a new device shape that initially has no controls specified."""
        self.axes: Dict[DeviceAxis, AxisDefinition] = {}
        self.keys: Dict[DeviceKey, str] = {}
        self._names: Set[str] = set() # track the names that have already been used

    def with_axis(self, axis: DeviceAxis, name: str, min_val: float = -1, max_val: float = 1, deadband: float = 0) -> 'DeviceShape':
        """Specifies an absolute axis for the device shape.

        Parameters
        ----------
        axis : DeviceAxis
            The absolute axis to specify.
        min_val : float
            The minimum value to output for the axis.
        max_val : float
            The maximum value to output for the axis.

        Returns
        -------
        DeviceShape
            This same device shape instance, for chaining.

        Raises
        ------
        ValueError
            If a control with the same name is already defined.
        """
        if name in self._names:
            raise ValueError(f'A control named "{name}" already exists!')
        self.axes[axis] = AxisDefinition(name, min_val, max_val, deadband)
        self._names.add(name)
        return self

    def with_key(self, key: DeviceKey, name: str) -> 'DeviceShape':
        """Specifies a button for the device shape.

        Parameters
        ----------
        key : DeviceKey
            The button to specify.
        name : str
            The name of the button.

        Returns
        -------
        DeviceShape
            This same device shape instance, for chaining.

        Raises
        ------
        ValueError
            If a control with the same name is already defined.
        """
        if name in self._names:
            raise ValueError(f'A control named "{name}" already exists!')
        self.keys[key] = name
        self._names.add(name)
        return self

    def match(self, caps: DeviceCaps) -> bool:
        """Checks if a device has this shape or not based on its capabilities.

        Note that this will match all devices whose shapes are a superset of this shape.
        This means that if a device contains all the controls specified in this shape,
        but also contains additional controls that are not specified, it will still be
        matched, and so this method will return `True` for such a device.

        Parameters
        ----------
        caps : DeviceCaps
            The capabilities of the device whose shape should be checked.

        Returns
        -------
        bool
            Whether the device has this shape or not.
        """
        return check_caps(caps, DeviceEventType.EV_ABS, self.axes.keys())\
            and check_caps(caps, DeviceEventType.EV_KEY, self.keys.keys())

def check_caps(caps: DeviceCaps, ev_type: DeviceEventType, expected: Collection[int]) -> bool:
    """Checks that a capability set advertises a particular set of event codes.

    Parameters
    ----------
    caps : DeviceCaps
        The device capability set.
    ev_type : DeviceEventType
        The event type whose codes should be checked.
    expected : Collection[int]
        The set of event codes to check for.

    Returns
    -------
    bool
        Whether the capabilities advertise the given event codes or not.
    """
    if len(expected) > 0:
        if ev_type not in caps:
            return False
        supported_codes = set(evdev_util.get_capability_codes(caps[ev_type]))
        for exp_code in expected:
            if exp_code not in supported_codes:
                return False
    return True
