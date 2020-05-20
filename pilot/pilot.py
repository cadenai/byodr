import logging
import multiprocessing
import threading
import traceback
from abc import ABCMeta, abstractmethod

import cachetools
import pid_controller.pid as pic

from byodr.utils import timestamp
from byodr.utils.option import parse_option, hash_dict

logger = logging.getLogger(__name__)


class ClassificationType(object):
    def __init__(self):
        pass

    UNDETERMINED = 'null'


class OriginType(ClassificationType):
    def __init__(self):
        ClassificationType.__init__(self)

    HUMAN = 'src.human'  # Human annotated (regular driving).
    CONSOLE = 'src.console'  # Recorded (raw) console value (interventions).
    DAGGER = 'src.dagger'  # Human expert recorded under dropout dagger.
    CRUISE_CONTROL = 'src.cruise'  # Determined in an automatic fashion.
    DNN = 'src.dnn'  # Determined by an automated model.
    DNN_PRE_INTERVENTION = 'src.dnn.pre-intervention'
    ANALYSED = 'src.analysed'
    AUGMENTED = 'src.augmented'
    OPEN_AI_DEEPDRIVE = 'src.open_ai.deepdrive'
    BACKEND_AUTOPILOT = 'src.backend.autopilot'


class AttrDict(dict):
    """ Access dictionary items through instance attributes."""

    def __init__(self, **kwargs):
        super(AttrDict, self).__init__(**kwargs)
        self.__dict__ = self


class Blob(AttrDict):
    def __init__(self, **kwargs):
        super(Blob, self).__init__(**kwargs)
        self.time = timestamp()
        self.cruise_speed = kwargs.get('cruise_speed')
        self.desired_speed = kwargs.get('desired_speed')
        self.driver = kwargs.get('driver')
        self.forced_acceleration = kwargs.get('forced_acceleration')
        self.forced_deceleration = kwargs.get('forced_deceleration')
        self.forced_steering = kwargs.get('forced_steering')
        self.forced_throttle = kwargs.get('forced_throttle')
        self.instruction = kwargs.get('instruction')
        self.save_event = kwargs.get('save_event')
        self.speed_driver = kwargs.get('speed_driver')
        self.steering = kwargs.get('steering', 0)
        self.steering_driver = kwargs.get('steering_driver')
        self.throttle = kwargs.get('throttle', 0)
        if self.forced_steering is None:
            self.forced_steering = abs(self.steering) > 0
        if self.forced_throttle is None:
            self.forced_throttle = abs(self.throttle) > 0
        if self.forced_acceleration is None:
            self.forced_acceleration = self.forced_throttle and self.throttle > 0
        if self.forced_deceleration is None:
            self.forced_deceleration = self.forced_throttle and self.throttle < 0


class IgnoreDifferences(object):
    """Use previous value if the new value did not change enough."""

    def __init__(self, init_value=0., threshold=0.):
        self._value = init_value
        self._threshold = threshold

    def calculate(self, x):
        if abs(x - self._value) > self._threshold:
            self._value = x
        return self._value


class DynamicMomentum(object):
    """Low-pass filter with separate acceleration and deceleration momentum."""

    def __init__(self, up=.1, down=.9, ceiling=1.):
        self._previous_value = 0
        self._acceleration = up
        self._deceleration = down
        self._ceiling = ceiling

    def calculate(self, value):
        _momentum = self._acceleration if value > self._previous_value else self._deceleration
        _new_value = min(self._ceiling, _momentum * value + (1. - _momentum) * self._previous_value)
        self._previous_value = _new_value
        return _new_value


class AbstractDriverBase(object):
    __metaclass__ = ABCMeta

    def __init__(self, control):
        super(AbstractDriverBase, self).__init__()
        self._control = control
        self._lock = multiprocessing.Lock()
        self._active = False

    @staticmethod
    def _apply_dead_zone(value, dead_zone=0.):
        if value > 0:
            value += dead_zone
        elif value < 0:
            value -= dead_zone
        return value

    @staticmethod
    def noop(blob):
        blob.steering = 0
        blob.throttle = 0
        blob.cruise_speed = 0
        blob.desired_speed = 0
        blob.instruction = 'intersection.ahead'
        blob.steering_driver = OriginType.UNDETERMINED
        blob.speed_driver = OriginType.UNDETERMINED
        blob.save_event = False

    def next_action(self, *args):
        blob = args[0]
        self.get_action(*args) if self._active else self.noop(blob)

    @abstractmethod
    def get_action(self, *args):
        raise NotImplementedError()

    def _activate(self):
        pass

    def _deactivate(self):
        pass

    def activate(self):
        with self._lock:
            self._activate()
            self._active = True

    def deactivate(self):
        with self._lock:
            self._deactivate()
            self._active = False

    def quit(self):
        pass


class AbstractThrottleControl(object):
    __metaclass__ = ABCMeta

    def __init__(self, min_desired_speed, max_desired_speed):
        self._min_desired_speed = min_desired_speed
        self._max_desired_speed = max_desired_speed

    def calculate_desired_speed(self, desired_speed, throttle, forced_acceleration, forced_deceleration, maximum=None):
        maximum = self._max_desired_speed if maximum is None else maximum
        ds = desired_speed
        if forced_acceleration:
            # Scale to the remainder of the max.
            ds += (maximum - ds) * throttle
        elif forced_deceleration:
            # Brake with constant power on the throttle scale.
            ds = max(0, ds + throttle * maximum)
            ds = 0 if ds <= self._min_desired_speed else ds
        return min(maximum, ds)

    @abstractmethod
    def calculate_throttle(self, desired_speed, current_speed):
        raise NotImplementedError()


class PidThrottleControl(AbstractThrottleControl):
    def __init__(self, (p, i, d), stop_p, min_desired_speed, max_desired_speed):
        super(PidThrottleControl, self).__init__(min_desired_speed, max_desired_speed)
        self._pid_throttle = None
        self._pid_stop = None
        self._min_desired_speed = min_desired_speed
        self._max_desired_speed = max_desired_speed
        self._pid_throttle = self._create_pid_ctl(p=p, i=i, d=d)
        self._pid_stop = self._create_pid_ctl(p=stop_p)

    @staticmethod
    def _create_pid_ctl(p, i=0., d=0.):
        _ctl = pic.PID(p=p, i=i, d=d)
        _ctl.target = 0.
        return _ctl

    def calculate_throttle(self, desired_speed, current_speed):
        # Keep both controllers up to date.
        feedback = desired_speed - current_speed
        stop_throttle_ = self._pid_stop(feedback=feedback)
        throttle_ = self._pid_throttle(feedback=feedback)
        # A separate pid controller is engaged when the desired speed drops below a threshold.
        # This could be used for emergency braking e.g. when desired speed is zero.
        return min(0, stop_throttle_) if desired_speed < self._min_desired_speed else max(-1, min(1, throttle_))


class DirectThrottleControl(AbstractThrottleControl):
    def __init__(self, min_desired_speed, max_desired_speed, throttle_up_momentum, throttle_down_momentum, throttle_cutoff):
        super(DirectThrottleControl, self).__init__(min_desired_speed, max_desired_speed)
        self._moment = DynamicMomentum(up=throttle_up_momentum, down=throttle_down_momentum)
        self._throttle_cut = throttle_cutoff

    def calculate_throttle(self, desired_speed, current_speed):
        _throttle = desired_speed
        _throttle = self._moment.calculate(_throttle)
        return _throttle if abs(_throttle) > self._throttle_cut else 0


class AbstractCruiseControl(AbstractDriverBase):
    __metaclass__ = ABCMeta

    def __init__(self, control, **kwargs):
        super(AbstractCruiseControl, self).__init__(control)
        _errors = []
        self._min_desired_speed = 0
        self._max_desired_speed = 0
        self._min_desired_speed = parse_option('driver.cc.static.speed.min', float, 0, _errors, **kwargs)
        self._max_desired_speed = parse_option('driver.cc.static.speed.max', float, 0, _errors, **kwargs)
        #
        _control_type = parse_option('driver.cc.control.type', str, 'direct', _errors, **kwargs)
        if _control_type == 'pid':
            p = (parse_option('driver.cc.throttle.pid_controller.p', float, 0, _errors, **kwargs))
            i = (parse_option('driver.cc.throttle.pid_controller.i', float, 0, _errors, **kwargs))
            d = (parse_option('driver.cc.throttle.pid_controller.d', float, 0, _errors, **kwargs))
            stop_p = (parse_option('driver.cc.stop.pid_controller.p', float, 1, _errors, **kwargs))
            self._throttle_control = PidThrottleControl(
                (p, i, d),
                stop_p=stop_p,
                min_desired_speed=self._min_desired_speed,
                max_desired_speed=self._max_desired_speed
            )
        else:
            _throttle_up_momentum = parse_option('driver.throttle.direct.up.momentum', float, 0, _errors, **kwargs)
            _throttle_down_momentum = parse_option('driver.throttle.direct.down.momentum', float, 0, _errors, **kwargs)
            _throttle_cutoff = parse_option('driver.throttle.direct.minimum', float, 0, _errors, **kwargs)
            self._throttle_control = DirectThrottleControl(
                min_desired_speed=self._min_desired_speed,
                max_desired_speed=self._max_desired_speed,
                throttle_up_momentum=_throttle_up_momentum,
                throttle_down_momentum=_throttle_down_momentum,
                throttle_cutoff=_throttle_cutoff
            )
        self._errors = _errors

    def get_errors(self):
        return self._errors

    def calculate_desired_speed(self, desired_speed, throttle, forced_acceleration, forced_deceleration, maximum=None):
        return self._throttle_control.calculate_desired_speed(desired_speed, throttle, forced_acceleration, forced_deceleration, maximum)

    def calculate_throttle(self, desired_speed, current_speed):
        return 0 if None in (desired_speed, current_speed) else self._throttle_control.calculate_throttle(desired_speed, current_speed)


class RawConsoleDriver(AbstractCruiseControl):
    def __init__(self, **kwargs):
        super(RawConsoleDriver, self).__init__('driver_mode.teleop.direct', **kwargs)

    def get_action(self, *args):
        blob = args[0]
        blob.steering = self._apply_dead_zone(blob.steering, dead_zone=0)
        blob.desired_speed = blob.throttle * self._max_desired_speed
        blob.steering_driver = OriginType.UNDETERMINED
        blob.speed_driver = OriginType.UNDETERMINED
        # No recording in this mode.
        blob.save_event = False


class StaticCruiseDriver(AbstractCruiseControl):
    def __init__(self, **kwargs):
        super(StaticCruiseDriver, self).__init__('driver_mode.teleop.cruise', **kwargs)

    def get_action(self, *args):
        blob, vehicle = args[:2]
        blob.desired_speed = self.calculate_desired_speed(desired_speed=blob.cruise_speed,
                                                          throttle=blob.throttle,
                                                          forced_acceleration=blob.forced_acceleration,
                                                          forced_deceleration=blob.forced_deceleration)
        blob.steering = self._apply_dead_zone(blob.steering, dead_zone=0)
        blob.throttle = self.calculate_throttle(blob.desired_speed, current_speed=(vehicle.get('velocity', 0)))
        blob.steering_driver = OriginType.HUMAN
        blob.speed_driver = OriginType.HUMAN
        blob.save_event = True


class BackendAutopilotDriver(AbstractCruiseControl):
    def __init__(self, **kwargs):
        super(BackendAutopilotDriver, self).__init__('driver_mode.automatic.backend', **kwargs)
        self._corridor_threshold = .99

    def get_action(self, *args):
        blob, vehicle, inference = args
        blob.desired_speed = vehicle.get('velocity', 0)
        blob.steering = vehicle.get('auto_steering', 0)
        blob.throttle = vehicle.get('auto_throttle', 0)
        blob.steering_driver = OriginType.BACKEND_AUTOPILOT
        blob.speed_driver = OriginType.BACKEND_AUTOPILOT
        blob.save_event = inference is None or inference.get('corridor') > self._corridor_threshold


class DeepNetworkDriver(AbstractCruiseControl):

    def __init__(self, **kwargs):
        super(DeepNetworkDriver, self).__init__('driver_mode.inference.dnn', **kwargs)
        self._piv_count = 0

    def _activate(self):
        pass

    def _deactivate(self):
        pass

    def get_action(self, *args):
        blob, vehicle, inference = args
        if None in (vehicle, inference):
            self.noop(blob)
            return

        action_out = inference.get('action')
        _dagger = inference.get('dagger', 0) == 1
        _total_penalty = inference.get('penalty')
        # Handle speed.
        desired_speed = blob.cruise_speed if _dagger else blob.cruise_speed * (1 - _total_penalty)
        blob.desired_speed = max(0, self.calculate_desired_speed(desired_speed=desired_speed,
                                                                 throttle=blob.throttle,
                                                                 forced_acceleration=blob.forced_acceleration,
                                                                 forced_deceleration=blob.forced_deceleration,
                                                                 maximum=self._max_desired_speed))
        _speed_intervention = blob.forced_acceleration or blob.forced_deceleration
        blob.speed_driver = OriginType.CONSOLE if _speed_intervention else OriginType.DNN
        blob.throttle = self.calculate_throttle(desired_speed=blob.desired_speed, current_speed=(vehicle.get('velocity', 0)))
        # Handle steering.
        if blob.forced_steering or _speed_intervention:
            # Any intervention type is sufficient to set the steering coming in from the user.
            steering = blob.steering
            blob.save_event = blob.desired_speed > 1e-3 if blob.forced_steering else True
            # Mark the first few interventions as such.
            if self._piv_count > 2:
                steering_driver = (OriginType.DAGGER if _dagger else OriginType.CONSOLE)
            else:
                self._piv_count += 1
                steering_driver = OriginType.DNN_PRE_INTERVENTION
        else:
            blob.save_event = False
            steering, steering_driver = action_out, OriginType.DNN
            self._piv_count = 0
        # Finalize.
        blob.steering = self._apply_dead_zone(steering, dead_zone=0)
        blob.steering_driver = steering_driver


class PilotState(object):
    def __init__(self):
        self.instruction = 'intersection.ahead'
        self.cruise_speed = 0


class DriverManager(object):
    def __init__(self, **kwargs):
        self._errors = []
        self._principal_steer_scale = parse_option('driver.steering.teleop.scale', float, 0, self._errors, **kwargs)
        self._cruise_speed_step = parse_option('driver.cc.static.gear.step', float, 0, self._errors, **kwargs)
        _steer_threshold = parse_option('driver.handler.steering.diff.threshold', float, 0, self._errors, **kwargs)
        self._steering_stabilizer = IgnoreDifferences(threshold=_steer_threshold)
        self._pilot_state = PilotState()
        self._driver_cache = {}
        self._errors.extend(self._fill_driver_cache(**kwargs))
        self._lock = multiprocessing.RLock()
        self._driver = None
        self._driver_ctl = None
        self.switch_ctl()

    def _fill_driver_cache(self, **kwargs):
        _errors = []
        _names = ('default', 'driver_mode.inference.dnn', 'driver_mode.teleop.cruise', 'driver_mode.automatic.backend')
        _methods = ((lambda: RawConsoleDriver(**kwargs)),
                    (lambda: DeepNetworkDriver(**kwargs)),
                    (lambda: StaticCruiseDriver(**kwargs)),
                    (lambda: BackendAutopilotDriver(**kwargs))
                    )
        for n, m in zip(_names, _methods):
            _driver = m()
            self._driver_cache[n] = _driver
            _errors.extend(_driver.get_errors())
        return _errors

    def _get_driver(self, control=None):
        return self._driver_cache[control] if control in self._driver_cache else self._driver_cache['default']

    def _activate(self):
        try:
            self._lock.acquire()
            if self._driver is not None:
                self._driver.activate()
        except Exception as e:
            logger.error("Driver activation: {}".format(traceback.format_exc(e)))
        finally:
            self._lock.release()

    def get_errors(self):
        return self._errors

    def increase_cruise_speed(self):
        with self._lock:
            if self.get_driver_ctl() != 'driver_mode.teleop.direct':
                self._pilot_state.cruise_speed += self._cruise_speed_step
                logger.info("Cruise speed set to '{}'.".format(self._pilot_state.cruise_speed))

    def decrease_cruise_speed(self):
        with self._lock:
            if self.get_driver_ctl() != 'driver_mode.teleop.direct':
                self._pilot_state.cruise_speed = max(0., self._pilot_state.cruise_speed - self._cruise_speed_step)
                logger.info("Cruise speed set to '{}'.".format(self._pilot_state.cruise_speed))

    def turn_instruction(self, turn):
        with self._lock:
            _is_ahead = self._pilot_state.instruction == turn and turn == 'intersection.ahead'
            intention = 'general.fallback' if _is_ahead else turn
            self._pilot_state.instruction = intention

    def get_driver_ctl(self):
        return self._driver_ctl

    def switch_ctl(self, control='driver_mode.teleop.direct'):
        if control is not None and control.lower() not in ('none', 'null', 'ignore', '0', 'false'):
            with self._lock:
                self._pilot_state.cruise_speed = 0
                # The switch must be immediate. Do not force wait on the previous driver to deactivate.
                if control != self._driver_ctl:
                    if self._driver is not None:
                        threading.Thread(target=self._driver.deactivate).start()
                    self._driver_ctl = control
                    self._driver = self._get_driver(control=control)
                    threading.Thread(target=self._activate).start()
                    logger.info("Pilot switch control to '{}'.".format(control))

    def noop(self):
        blob = Blob()
        self._driver.noop(blob)
        return blob

    def next_action(self, command, vehicle, inference):
        with self._lock:
            # The blob time is now.
            # If downstream processes need the teleop time then use an extra attribute.
            blob = Blob(driver=self._driver_ctl,
                        cruise_speed=self._pilot_state.cruise_speed,
                        instruction=self._pilot_state.instruction,
                        **command)
            # Scale teleop before interpretation by the driver.
            blob.steering = self._principal_steer_scale * blob.steering
            self._driver.next_action(blob, vehicle, inference)
            blob.steering = self._steering_stabilizer.calculate(blob.steering)
            return blob

    def quit(self):
        for driver in self._driver_cache.values():
            driver.deactivate()
            driver.quit()


class CommandProcessor(object):
    def __init__(self, **kwargs):
        self._hash = hash_dict(**kwargs)
        self._driver = DriverManager(**kwargs)
        self._errors = [] + self._driver.get_errors()
        self._process_frequency = parse_option('clock.hz', int, 10, self._errors, **kwargs)
        self._patience_ms = parse_option('patience.ms', int, 100, self._errors, **kwargs)
        self._button_north_ctl = parse_option('controller.button.north.mode', str, 0, self._errors, **kwargs)
        self._button_west_ctl = parse_option('controller.button.west.mode', str, 0, self._errors, **kwargs)
        self._patience_micro = self._patience_ms * 1000.
        # Avoid processing the same command more than once.
        # TTL is specified in seconds.
        self._cache = cachetools.TTLCache(maxsize=100, ttl=(self._patience_ms * 1e-3))

    def _cache_safe(self, key, func, *arguments):
        if self._cache.get(key) is None:
            try:
                func(*arguments)
            finally:
                self._cache[key] = 1

    def _process(self, command):
        if 'quit' in command.keys():
            self._driver.switch_ctl()
        #
        # Buttons clockwise: N, E, S, W
        # N
        elif command.get('button_y', 0) == 1:
            self._cache_safe('button north ctl', lambda: self._driver.switch_ctl(self._button_north_ctl))
        # E
        elif command.get('button_b', 0) == 1:
            self._cache_safe('teleop driver', lambda: self._driver.switch_ctl('driver_mode.teleop.direct'))
        # S
        elif command.get('button_x', 0) == 1:
            self._cache_safe('button west ctl', lambda: self._driver.switch_ctl(self._button_west_ctl))
        #
        elif command.get('arrow_up', 0) == 1:
            self._cache_safe('increase cruise speed', lambda: self._driver.increase_cruise_speed())
        elif command.get('arrow_down', 0) == 1:
            self._cache_safe('decrease cruise speed', lambda: self._driver.decrease_cruise_speed())
        elif command.get('button_left', 0) == 1:
            self._cache_safe('turn left', lambda: self._driver.turn_instruction('intersection.left'))
        elif command.get('button_center', 0) == 1:
            self._cache_safe('turn ahead', lambda: self._driver.turn_instruction('intersection.ahead'))
        elif command.get('button_right', 0) == 1:
            self._cache_safe('turn right', lambda: self._driver.turn_instruction('intersection.right'))

    def get_patience_ms(self):
        return self._patience_ms

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def get_frequency(self):
        return self._process_frequency

    def get_errors(self):
        return self._errors

    def quit(self):
        self._driver.quit()

    def next_action(self, *args):
        _patience, _ts = self._patience_micro, timestamp()
        # Any of these can be None, too old or repetitive.
        times = [None if arg is None else _ts - arg.get('time') for arg in args]
        commands = [None if arg is None else arg if (times[i] < _patience) else None for i, arg in enumerate(args)]
        # What to do on message timeout depends on which driver is active.
        _ctl = self._driver.get_driver_ctl()
        teleop, vehicle, inference = commands
        # First process teleop commands.
        if teleop is not None:
            self._process(teleop)
        # Switch off autopilot on internal errors.
        if _ctl == 'driver_mode.inference.dnn' and None in (vehicle, inference):
            self._cache_safe('teleop driver', lambda: self._driver.switch_ctl('driver_mode.teleop.direct'))
            return self._driver.noop()
        # Everything normal or there is no autopilot process but teleop should function normally.
        if None not in commands or (None not in (teleop, vehicle) and _ctl == 'driver_mode.teleop.direct'):
            return self._driver.next_action(teleop, vehicle, inference)
        # Autopilot drives without teleop commands.
        if None not in (vehicle, inference) and _ctl == 'driver_mode.inference.dnn':
            return self._driver.next_action(dict(), vehicle, inference)
        # Vehicle control by backend.
        if vehicle is not None and _ctl == 'driver_mode.automatic.backend':
            return self._driver.next_action(dict(), vehicle, inference)
        # Ignore old or repetitive teleop commands.
        return None
