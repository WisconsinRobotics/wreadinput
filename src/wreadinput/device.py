from io import BytesIO
import os
import selectors
from threading import Lock, Thread
from typing import Dict, Iterator, Optional, Tuple, cast

import evdev

from .util import evdev_util
from .util.evdev_const import DeviceAxis, DeviceEventType, DeviceKey, SyncEvent

class AxisBuf:
    def __init__(self, init_value: float, unmapped_min: float, unmapped_max: float):
        self._offset: float
        self._scale: float
        if unmapped_min == 0 and unmapped_max > 0:
            self._offset = 0
            self._scale = 1 / unmapped_max
        elif unmapped_max == 0 and unmapped_min < 0:
            self._offset = 0
            self._scale = 1 / abs(unmapped_min)
        else:
            self._offset = -(unmapped_min + unmapped_max) / 2
            self._scale = 2 / (unmapped_max - unmapped_min)
        self._value = self._remap(init_value)
    
    @property
    def value(self) -> float:
        return self._value

    def update(self, unmapped_value: float):
        self._value = self._remap(unmapped_value)

    def _remap(self, unmapped_value: float) -> float:
        return (unmapped_value + self._offset) * self._scale

class KeyBuf:
    def __init__(self, init_value: bool):
        self.value = init_value

class InputDevice:
    def __init__(self, dev_file: str):
        self._dev = evdev.InputDevice(dev_file)

        self._poll_thread_ctx: Optional[Tuple[Thread, int]] = None # thread and notify pipe
        self._thread_lock = Lock()
        
        self._axis_cache: Dict[DeviceAxis, AxisBuf] = {}
        self._key_cache: Dict[DeviceKey, KeyBuf] = {}
        self._data_lock = Lock()

        for ev_type, ev_caps in self._dev.capabilities().items():
            ev_codes = evdev_util.get_capability_codes(ev_caps)
            if ev_type == DeviceEventType.EV_ABS:
                for code in ev_codes:
                    axis_info = self._dev.absinfo(code)
                    axis_buf = AxisBuf(axis_info.value, axis_info.min, axis_info.max)
                    try:
                        self._axis_cache[DeviceAxis(code)] = axis_buf
                    except ValueError:
                        pass
            elif ev_type == DeviceEventType.EV_KEY:
                init_key_states = set(self._dev.active_keys())
                for code in ev_codes:
                    try:
                        self._key_cache[DeviceKey(code)] = KeyBuf(code in init_key_states)
                    except ValueError:
                        pass
        
    def start(self):
        with self._thread_lock:
            if self._dev.fd == -1:
                raise ValueError('Device is already closed!')
            elif self._poll_thread_ctx is not None:
                raise ValueError('Poll thread already exists!')

        # may deadlock if the device is lost, since the selector will never receive an event
        # so we add a virtual pipe for the selector to read from that we can use to "break out"
        notify_pipe_r, notify_pipe_w = os.pipe2(os.O_NONBLOCK)

        def poll():
            axis_temp: Dict[int, int] = dict()
            key_temp: Dict[int, int] = dict()
            syn_okay = True
            
            def consume_event(event: evdev.InputEvent):
                nonlocal syn_okay
                if event.type == DeviceEventType.EV_ABS: # axis state event
                    if syn_okay:
                        try:
                            axis_temp[DeviceAxis(event.code)] = event.value
                        except ValueError:
                            pass
                elif event.type == DeviceEventType.EV_KEY: # key state event
                    if syn_okay:
                        try:
                            key_temp[DeviceKey(event.code)] = event.value
                            evdev.categorize
                        except ValueError:
                            pass
                elif event.type == DeviceEventType.EV_SYN: # synchronization event
                    if event.code == SyncEvent.SYN_REPORT: # end of a sync frame
                        if syn_okay: # sync frame was okay; copy data for frame to state caches
                            for axis_code, state in axis_temp.items(): # copy axis state
                                try:
                                    axis_buf = self._axis_cache.get(DeviceAxis(axis_code))
                                    if axis_buf is not None:
                                        axis_buf.update(state)
                                except ValueError:
                                    pass
                            axis_temp.clear()
                            for key_code, state in key_temp.items(): # copy key state
                                try:
                                    key_buf = self._key_cache.get(DeviceKey(key_code))
                                    if key_buf is not None:
                                        key_buf.value = state != 0 # 0 => release; 1 => press; 2 => hold
                                except ValueError:
                                    pass
                            key_temp.clear()
                        else: # sync frame was bad; retrieve actual state using ioctl, then return to normal
                            syn_okay = True
                            for abs_code, abs_buf in self._axis_cache.items(): # resync axis states
                                abs_buf.update(self._dev.absinfo(abs_code.value).value) # shouldn't need to update other axis props... probably
                            for key_buf in self._key_cache.values(): # resync key states
                                key_buf.value = False
                            for key_code in self._dev.active_keys():
                                try:
                                    key_buf = self._key_cache.get(DeviceKey(key_code))
                                    if key_buf is not None:
                                        key_buf.value = True
                                except ValueError:
                                    pass
                    elif event.code == SyncEvent.SYN_DROPPED: # sync was lost; drop the sync frame and wait for the next one
                        axis_temp.clear()
                        key_temp.clear()
                        syn_okay = False

            # check to ensure that the device is still there; better safe than sorry
            with self._thread_lock:
                if self._dev.fd == -1:
                    return

            with open(notify_pipe_r, 'rb') as notify_pipe_file:
                # use selector to join the device and the virtual "break-out" pipe
                sel = selectors.DefaultSelector()
                sel.register(self._dev, selectors.EVENT_READ)
                sel.register(notify_pipe_file, selectors.EVENT_READ)
                
                while True:
                    # read events
                    for key, _ in sel.select():
                        if key.fileobj == self._dev:
                            for event in cast(Iterator[evdev.InputEvent], self._dev.read()):
                                consume_event(event)
                        else: # must be the virtual pipe
                            cast(BytesIO, key.fileobj).read()
                    
                    # terminate if the device is closed
                    with self._thread_lock:
                        if self._dev.fd == -1:                            
                            break
        
        poll_thread = Thread(target=poll)
        poll_thread.daemon = True
        poll_thread.start()
        self._poll_thread_ctx = poll_thread, notify_pipe_w

    def get_axis(self, axis: DeviceAxis) -> Optional[float]:
        with self._data_lock:
            axis_buf = self._axis_cache.get(axis)
            return axis_buf.value if axis_buf is not None else None

    def get_key(self, key: DeviceKey) -> Optional[bool]:
        with self._data_lock:
            key_buf = self._key_cache.get(key)
            return key_buf.value if key_buf is not None else None

    def kill(self):
        with self._thread_lock:
            self._dev.close()
        if self._poll_thread_ctx is not None:
            os.write(self._poll_thread_ctx[1], b'\0') # write some random byte to break out of the selector read
            os.close(self._poll_thread_ctx[1])
            self._poll_thread_ctx[0].join()
            self._poll_thread_ctx = None
