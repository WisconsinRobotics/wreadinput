#!/usr/bin/env python3

from wreadinput import DeviceAxis, DeviceShape, DeviceKey, default_node

# TODO check to make sure these are what actually get mapped by evdev
SHAPE_LOGX3DPRO = DeviceShape()\
    .with_axis(DeviceAxis.ABS_X, 'stick_x')\
    .with_axis(DeviceAxis.ABS_Y, 'stick_y')\
    .with_axis(DeviceAxis.ABS_RZ, 'stick_twist')\
    .with_axis(DeviceAxis.ABS_THROTTLE, 'throttle')\
    .with_axis(DeviceAxis.ABS_HAT0X, 'pov_x')\
    .with_axis(DeviceAxis.ABS_HAT0Y, 'pov_y')\
    .with_key(DeviceKey.BTN_TRIGGER, 'trigger')\
    .with_key(DeviceKey.BTN_THUMB, 'thumb')\
    .with_key(DeviceKey.BTN_THUMB2, '3')\
    .with_key(DeviceKey.BTN_TOP, '4')\
    .with_key(DeviceKey.BTN_TOP2, '5')\
    .with_key(DeviceKey.BTN_PINKIE, '6')\
    .with_key(DeviceKey.BTN_BASE, '7')\
    .with_key(DeviceKey.BTN_BASE2, '8')\
    .with_key(DeviceKey.BTN_BASE3, '9')\
    .with_key(DeviceKey.BTN_BASE4, '10')\
    .with_key(DeviceKey.BTN_BASE5, '11')\
    .with_key(DeviceKey.BTN_BASE6, '12')

# see `wreadinput.default_node` for implementation details
if __name__ == '__main__':
    default_node.main('input_logx3dpro', SHAPE_LOGX3DPRO)
