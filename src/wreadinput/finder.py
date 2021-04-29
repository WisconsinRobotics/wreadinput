import selectors
from typing import ContextManager, Iterable, Iterator, List, Optional, cast
from wready import SignalInterruptHandler

import evdev

from .util.evdev_const import DeviceEventType

class DeviceFinderEnumerator:
    """Finds the first device in a set that emits any button press event.
    
    This is done by listening for an event from all the devices using a selector.
    """
    
    def __init__(self, devs: List[evdev.InputDevice]):
        """Creates a new enumerator that consumes events from the given devices.

        Parameters
        ----------
        devs : List[evdev.InputDevice]
            The list of devices to consume events from.
        """
        self._sel = selectors.DefaultSelector()
        for dev in devs:
            self._sel.register(dev, selectors.EVENT_READ)
    
    def get(self) -> evdev.InputDevice:
        """Gets the first device that emits any button press event.

        This call will block until a button press event is consumed.

        Returns
        -------
        evdev.InputDevice
            The first input device to emit a button press event.
        """
        while True:
            for key, _ in self._sel.select():
                device = cast(evdev.InputDevice, key.fileobj)
                for event in cast(Iterator[evdev.InputEvent], device.read()):
                    if event.type == DeviceEventType.EV_KEY and event.value != 0:
                        return device

    def _kill(self):
        """Shuts down the enumerator, closing the selector."""
        self._sel.close()

class DeviceFinder(ContextManager[DeviceFinderEnumerator]):
    """Acquires an input device by waiting for a button press."""
    
    def __init__(self, devs: Iterable[evdev.InputDevice]):
        """Constructs a new device finder for the given search set.

        This should probably be used as a context manager in order to manage the
        enumerator more easily, like so::

            device: evdev.InputDevice
            with DeviceFinder(devs) as finder:
                device = finder.get()

        This will automatically close the enumerator after the `with`-block ends.

        Parameters
        ----------
        devs : Iterable[evdev.InputDevice]
            The search set of devices to wait for an input from.
        """
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
