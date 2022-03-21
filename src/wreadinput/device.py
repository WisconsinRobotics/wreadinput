from io import BytesIO
import os
import selectors
from threading import Lock, Thread
from typing import Dict, Iterator, Optional, Tuple, Union, cast

import evdev
import rospy

from .shape import DeviceShape
from .util import evdev_util
from .util.evdev_const import DeviceAxis, DeviceEventType, DeviceKey, SyncEvent

class AxisBuf:
    """A buffer storing the state of a single absolute axis on an input device.
    
    Axis values are remapped from a source interval [a, b] to a used-defined target
    interval [a', b'], which allows for the normalization of joystick values.
    """

    def __init__(self, init_value: float, from_min: float, from_max: float, to_min: float, to_max: float, deadband: float):
        """Creates a new buffer for an absolute axis with the given properties.

        Parameters
        ----------
        init_value : float
            The initial axis value.
        from_min : float
            The minimum value for the source interval.
        from_max : float
            The maximum value for the source interval.
        to_min : float
            The minimum value for the target interval.
        to_max : float
            The maximum value for the target interval.
        deadband : float
            The ratio of input area on both sides of the input center that should be centered in the output.
        """

        # Set up slopes and output offsets, as well as the bounds for the 3 regions introduced by the deadband
        self._scale = (to_max - to_min) / ((1 - deadband) * (from_max - from_min))
        self._offset = (to_max + to_min) / 2
        input_center = (from_max + from_min) / 2
        self._deadband_high = deadband * (from_max - input_center) + input_center
        self._deadband_low = deadband * (from_min - input_center) + input_center
        
        # Sort the regions in the event of inversion maps
        self._low_region_lower_bound = min(self._deadband_low, from_min)
        self._low_region_upper_bound = max(self._deadband_low, from_min)
        self._high_region_lower_bound = min(self._deadband_high, from_max)
        self._high_region_upper_bound = max(self._deadband_high, from_max)
        self._center_region_lower_bound = min(self._low_region_upper_bound, self._high_region_lower_bound)
        self._center_region_upper_bound = max(self._low_region_upper_bound, self._high_region_lower_bound)

        # Update the output value
        self._value = self._remap(init_value)
    
    @property
    def value(self) -> float:
        """The current remapped value of the axis.

        Returns
        -------
        float
            The remapped axis value.
        """
        return self._value

    def update(self, unmapped_value: float):
        """Writes a new value to the buffer.

        Parameters
        ----------
        unmapped_value : float
            The unmapped axis value.
        """
        self._value = self._remap(unmapped_value)

    def _remap(self, unmapped_value: float) -> float:
        """Remaps an axis value to the target interval.

        Parameters
        ----------
        unmapped_value : float
            The unmapped axis value.

        Returns
        -------
        float
            The remapped axis value.
        """
        # TODO: This is an opportunity to improve code in the future if we migrate the wrover and base station to Python 3.10
        if self._low_region_lower_bound <= unmapped_value <= self._low_region_upper_bound:
            return self._scale * (unmapped_value - self._deadband_low) + self._offset
        elif self._center_region_lower_bound <= unmapped_value <= self._center_region_upper_bound:
            return self._offset
        elif self._high_region_lower_bound <= unmapped_value <= self._high_region_upper_bound:
            return self._scale * (unmapped_value - self._deadband_high) + self._offset
        else:
            raise ValueError(f"Value {unmapped_value} is not in the desginated input region of {(self._low_region_lower_bound, self._high_region_upper_bound)}")

class KeyBuf:
    """A buffer storing the state of a single button on an input device.
    
    Considerably simpler than the axis buffer.
    """
    
    def __init__(self, init_value: bool):
        """Creates a new buffer for a button.

        Parameters
        ----------
        init_value : bool
            The initial state of the button.
        """
        self.value = init_value

class InputDevice:
    """Represents a single input device and all of its state.
    
    Instances of this class maintain a polling thread that consumes evdev events.
    To ensure that the thread is cleaned up and to prevent deadlocks, users of this
    class should make sure to call `kill` on an instance when it is no longer needed.
    """
    
    def __init__(self, device: Union[str, evdev.InputDevice], shape: DeviceShape):
        """Constructs a new `InputDevice` instance for the given device.

        The device will be polled for capabilities, which will allow for the creation
        of state buffers for each axis and button on the device. To start the evdev
        polling thread, call `start`; the `InputDevice` will not be able to track the
        device's state until then.

        Parameters
        ----------
        device : Union[str, evdev.InputDevice]
            The device, given either as a path to a device file or as an instance of
            `evdev.InputDevice`.
        shape : DeviceShape
            The shape of the device.
        
        See Also
        --------
        start : Initializes evdev polling.
        """
        self.shape = shape
        self._dev = device if isinstance(device, evdev.InputDevice) else evdev.InputDevice(device)

        self._poll_thread_ctx: Optional[Tuple[Thread, int]] = None # thread and notify pipe
        self._thread_lock = Lock()
        
        self._axis_cache: Dict[DeviceAxis, AxisBuf] = {}
        self._key_cache: Dict[DeviceKey, KeyBuf] = {}
        self._data_lock = Lock()

        # construct axis and key buffers based on the device's advertised capabilities
        for ev_type, ev_caps in self._dev.capabilities().items():
            ev_codes = evdev_util.get_capability_codes(ev_caps)
            if ev_type == DeviceEventType.EV_ABS:
                for code in ev_codes:
                    axis: DeviceAxis
                    try:
                        axis = DeviceAxis(code)
                    except ValueError:
                        continue
                    axis_def = shape.axes.get(axis)
                    if axis_def is None:
                        continue
                    axis_info = self._dev.absinfo(code)
                    self._axis_cache[axis] = AxisBuf(
                        axis_info.value, axis_info.min, axis_info.max, axis_def.min_val, axis_def.max_val, axis_def.deadband)
            elif ev_type == DeviceEventType.EV_KEY:
                init_key_states = set(self._dev.active_keys())
                for code in ev_codes:
                    try:
                        self._key_cache[DeviceKey(code)] = KeyBuf(code in init_key_states)
                    except ValueError:
                        pass
        
    def start(self):
        """Initializes the evdev polling thread.

        This is what allows for the tracking of the device's state. Once this device
        is no longer needed, the `kill` method should be called to ensure that the
        polling thread is cleaned up properly in order to prevent resource leaks and
        deadlocks.

        Raises
        ------
        ValueError
            If the polling thread has already been started, or if the device has
            already been shut down.
        
        Notes
        -----
        Evdev events are organized into "frames", each of which is separated by an
        EV_SYN event of code SYN_REPORT. Inputs should only be considered committed
        when a whole frame has been sent. In the case where the event buffer overflows,
        events will be lost, in which case the frame may be incomplete. This is
        indicated by an EV_SYN event of code SYN_DROPPED, which signals to us that we
        need to resynchronize with the frames. See [1]_ for more details.

        References
        ----------
        .. [1] https://www.freedesktop.org/software/libevdev/doc/latest/syn_dropped.html
        """
        with self._thread_lock:
            if self._dev.fd == -1:
                raise ValueError('Device is already closed!')
            elif self._poll_thread_ctx is not None:
                raise ValueError('Poll thread already exists!')

        # may deadlock if the device is lost, since the selector will never receive an event
        # so we add a virtual pipe for the selector to read from that we can use to "break out"
        notify_pipe_r, notify_pipe_w = os.pipe2(os.O_NONBLOCK)

        def poll():
            rospy.loginfo('Initializing evdev thread state...')
            
            axis_temp: Dict[int, int] = dict() # temp buffers for the current incomplete frame
            key_temp: Dict[int, int] = dict()
            syn_okay = True # if SYN_DROPPED, this becomes false to indicate that the frame is fragmented
            
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
                    rospy.loginfo('Device was dead before the evdev thread was ready!')
                    return

            with open(notify_pipe_r, 'rb') as notify_pipe_file:
                # use selector to conjoin the device and the virtual "break-out" pipe
                sel = selectors.DefaultSelector()
                sel.register(self._dev, selectors.EVENT_READ)
                sel.register(notify_pipe_file, selectors.EVENT_READ)
                
                rospy.loginfo('Entering evdev polling loop...')
                while True:
                    # read events
                    for key, _ in sel.select():
                        if key.fileobj == self._dev: # it's from evdev
                            for event in cast(Iterator[evdev.InputEvent], self._dev.read()):
                                consume_event(event)
                        else: # must be the virtual pipe
                            rospy.loginfo('Received notification from virtual pipe!')
                            cast(BytesIO, key.fileobj).read()

                    # terminate if the device is closed
                    with self._thread_lock:
                        if self._dev.fd == -1:
                            rospy.loginfo('The device was closed! Terminating the evdev thread...')
                            break
        
        poll_thread = Thread(target=poll)
        poll_thread.start()
        self._poll_thread_ctx = poll_thread, notify_pipe_w

    def get_axis(self, axis: DeviceAxis) -> Optional[float]:
        """Retrieves the state of an absolute axis.

        The axis value will be normalized. See the `AxisBuf` class for more details.

        Parameters
        ----------
        axis : DeviceAxis
            The axis whose state should be queried.

        Returns
        -------
        Optional[float]
            The axis' state, or `None` if there is no data available for it.
        """
        with self._data_lock:
            axis_buf = self._axis_cache.get(axis)
            return axis_buf.value if axis_buf is not None else None

    def get_key(self, key: DeviceKey) -> Optional[bool]:
        """Retrieves the state of a button.

        Parameters
        ----------
        key : DeviceKey
            The button whose state should be queried.

        Returns
        -------
        Optional[bool]
            The button's state, or `None` if there is no data available for it.
        """
        with self._data_lock:
            key_buf = self._key_cache.get(key)
            return key_buf.value if key_buf is not None else None

    def kill(self):
        """Shuts down the device.
        
        This closes any relevant file handles and terminates the polling thread.
        The `InputDevice` instance can no longer be used once this is done.
        """
        with self._thread_lock:
            self._dev.close()
        if self._poll_thread_ctx is not None:
            os.write(self._poll_thread_ctx[1], b'\0') # write some random byte to break out of the selector read
            os.close(self._poll_thread_ctx[1])
            self._poll_thread_ctx[0].join()
            self._poll_thread_ctx = None
