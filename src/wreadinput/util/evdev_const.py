from enum import IntEnum
from struct import Struct
from typing import Dict, List

DeviceCaps = Dict[int, List[int]]

# https://www.kernel.org/doc/Documentation/input/input.txt
EVENT_STRUCT = Struct('llHHI')

class DeviceEventType(IntEnum):
    """Constants for evdev event types.

    See [1]_ for a list of event types and [2]_ for their
    declarations in the Linux evdev source code.
    
    References
    ----------
    .. [1] https://www.kernel.org/doc/Documentation/input/event-codes.txt
    .. [2] https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h#L34
    """
    
    EV_SYN = 0x00
    EV_KEY = 0x01
    EV_REL = 0x02
    EV_ABS = 0x03
    EV_MSC = 0x04
    EV_SW = 0x05
    EV_LED = 0x11
    EV_SND = 0x12
    EV_REP = 0x14
    EV_FF = 0x15
    EV_PWR = 0x16
    EV_FF_STATUS = 0x17
    EV_MAX = 0x1F

class DeviceKey(IntEnum):
    """Constants for evdev keys.

    See [1]_ for the constant declarations in the Linux evdev
    source code.

    References
    ----------
    .. [1] https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h#L64
    """
    BTN_0 = 0x100
    BTN_1 = 0x101
    BTN_2 = 0x102
    BTN_3 = 0x103
    BTN_4 = 0x104
    BTN_5 = 0x105
    BTN_6 = 0x106
    BTN_7 = 0x107
    BTN_8 = 0x108
    BTN_9 = 0x109
    BTN_LEFT = 0x110
    BTN_RIGHT = 0x111
    BTN_MIDDLE = 0x112
    BTN_SIDE = 0x113
    BTN_EXTRA = 0x114
    BTN_FORWARD = 0x115
    BTN_BACK = 0x116
    BTN_TASK = 0x117

    BTN_TRIGGER = 0x120
    BTN_THUMB = 0x121
    BTN_THUMB2 = 0x122
    BTN_TOP = 0x123
    BTN_TOP2 = 0x124
    BTN_PINKIE = 0x125
    BTN_BASE = 0x126
    BTN_BASE2 = 0x127
    BTN_BASE3 = 0x128
    BTN_BASE4 = 0x129
    BTN_BASE5 = 0x12A
    BTN_BASE6 = 0x12B
    BTN_DEAD = 0x12F

    BTN_A = 0x130
    BTN_B = 0x131
    BTN_C = 0x132
    BTN_X = 0x133
    BTN_Y = 0x134
    BTN_Z = 0x135
    BTN_TL = 0x136
    BTN_TR = 0x137
    BTN_TL2 = 0x138
    BTN_TR2 = 0x139
    BTN_SELECT = 0x13A
    BTN_START = 0x13B
    BTN_MODE = 0x13C
    BTN_THUMBL = 0x13D
    BTN_THUMBR = 0x13E

    BTN_GEAR_DOWN = 0x150
    BTN_GEAR_UP = 0x151

class DeviceAxis(IntEnum):
    """Constants for evdev absolute axes.

    See [1]_ for the constant declarations in the Linux evdev
    source code.

    References
    ----------
    .. [1] https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h#L811
    """
    ABS_X = 0x00
    ABS_Y = 0x01
    ABS_Z = 0x02
    ABS_RX = 0x03
    ABS_RY = 0x04
    ABS_RZ = 0x05
    ABS_THROTTLE = 0x06
    ABS_RUDDER = 0x07
    ABS_WHEEL = 0x08
    ABS_GAS = 0x09
    ABS_BRAKE = 0x0A
    ABS_HAT0X = 0x10
    ABS_HAT0Y = 0x11
    ABS_HAT1X = 0x12
    ABS_HAT1Y = 0x13
    ABS_HAT2X = 0x14
    ABS_HAT2Y = 0x15
    ABS_HAT3X = 0x16
    ABS_HAT3Y = 0x17
    ABS_PRESSURE = 0x18
    ABS_DISTANCE = 0x19
    ABS_TILT_X = 0x1A
    ABS_TILT_Y = 0x1B
    ABS_TOOL_WIDTH = 0x1C

    ABS_VOLUME = 0x20

    ABS_MISC = 0x28

class SyncEvent(IntEnum):
    """Constants for evdev device sync event types.

    See [1]_ for the constant declarations in the Linux evdev
    source code.

    References
    ----------
    .. [1] https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h#L53
    """
    SYN_REPORT = 0
    SYN_CONFIG = 1
    SYN_MT_REPORT = 2
    SYN_DROPPED = 3
