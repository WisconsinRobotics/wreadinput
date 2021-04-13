import selectors
import signal
import threading
from typing import Any, ContextManager, Iterable, Iterator, List, Optional, Tuple, cast

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
        self._hijacked_signals: Optional[Tuple[Any, Any, Any]] # not clear what type a signal handler is

    def __enter__(self) -> DeviceFinderEnumerator:
        if self._enum is not None:
            raise ValueError('Enumerator is already active!')
        self._enum = DeviceFinderEnumerator(self._devs)
        if threading.current_thread() == threading.main_thread():
            def interrupt(*sig):
                self._enum._kill()
                raise KeyboardInterrupt()
            self._prev_sig_handlers = signal.signal(signal.SIGINT, interrupt),\
                                      signal.signal(signal.SIGHUP, interrupt),\
                                      signal.signal(signal.SIGTERM, interrupt)
        return self._enum
    
    def __exit__(self, *exc):
        if self._enum is not None:
            self._enum._kill()
            self._enum = None
        if self._prev_sig_handlers is not None:
            signal.signal(signal.SIGINT, self._prev_sig_handlers[0])
            signal.signal(signal.SIGHUP, self._prev_sig_handlers[1])
            signal.signal(signal.SIGTERM, self._prev_sig_handlers[2])
            self._prev_sig_handlers = None
