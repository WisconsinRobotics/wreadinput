import selectors
from typing import ContextManager, Iterable, Iterator, List, Optional, cast
from wready import SignalInterruptHandler

import evdev

from .util.evdev_const import DeviceEventType

class DeviceFinderEnumerator:
    def __init__(self, devs: List[evdev.InputDevice]):
        self._sel = selectors.DefaultSelector()
        for dev in devs:
            self._sel.register(dev, selectors.EVENT_READ)
    
    def get(self) -> evdev.InputDevice:
        while True:
            for key, _ in self._sel.select():
                device = cast(evdev.InputDevice, key.fileobj)
                for event in cast(Iterator[evdev.InputEvent], device.read()):
                    if event.type == DeviceEventType.EV_KEY and event.value != 0:
                        return device

    def _kill(self):
        self._sel.close()

class DeviceFinder(ContextManager[DeviceFinderEnumerator]):
    def __init__(self, devs: Iterable[evdev.InputDevice]):
        self._devs = list(devs)
        self._enum: Optional[DeviceFinderEnumerator] = None
        self._sig_int_handler = SignalInterruptHandler(self.__exit__)

    def __enter__(self) -> DeviceFinderEnumerator:
        if self._enum is not None:
            raise ValueError('Enumerator is already active!')
        self._enum = DeviceFinderEnumerator(self._devs)
        self._sig_int_handler.inject()
        return self._enum
    
    def __exit__(self, *exc):
        self._sig_int_handler.restore()
        if self._enum is not None:
            self._enum._kill()
            self._enum = None
