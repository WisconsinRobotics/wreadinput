#!/usr/bin/env python3

from wreadinput import DeviceAxis, DeviceShape, DeviceKey, default_node

# TODO check to make sure these are what actually get mapped by evdev
SHAPE_XBOX360 = DeviceShape()\
    .with_axis(DeviceAxis.ABS_X, 'stick_left_x')\
    .with_axis(DeviceAxis.ABS_Y, 'stick_left_y')\
    .with_axis(DeviceAxis.ABS_RX, 'stick_right_x')\
    .with_axis(DeviceAxis.ABS_RY, 'stick_right_y')\
    .with_axis(DeviceAxis.ABS_Z, 'trigger_left')\
    .with_axis(DeviceAxis.ABS_RZ, 'trigger_right')\
    .with_axis(DeviceAxis.ABS_HAT0X, 'pov_x')\
    .with_axis(DeviceAxis.ABS_HAT0Y, 'pov_y')\
    .with_key(DeviceKey.BTN_A, 'a')\
    .with_key(DeviceKey.BTN_B, 'b')\
    .with_key(DeviceKey.BTN_X, 'x')\
    .with_key(DeviceKey.BTN_Y, 'y')\
    .with_key(DeviceKey.BTN_TL, 'shoulder_l')\
    .with_key(DeviceKey.BTN_TR, 'shoulder_r')\
    .with_key(DeviceKey.BTN_THUMBL, 'stick_left')\
    .with_key(DeviceKey.BTN_THUMBR, 'stick_right')\
    .with_key(DeviceKey.BTN_START, 'start')\
    .with_key(DeviceKey.BTN_SELECT, 'select')

# see `wreadinput.default_node` for implementation details
if __name__ == '__main__':
    default_node.main('input_xbox360', SHAPE_XBOX360)
