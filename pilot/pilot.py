import logging
import multiprocessing
import threading
import traceback
from abc import ABCMeta, abstractmethod

import cachetools
import numpy as np
import pid_controller.pid as pic

from byodr.utils import timestamp, Configurable
from byodr.utils.navigate import NavigationCommand
from byodr.utils.option import parse_option

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
        self.arrow_up = kwargs.get('arrow_up')
        self.arrow_down = kwargs.get('arrow_down')
        if self.forced_steering is None:
            self.forced_steering = abs(self.steering) > 0
        if self.forced_throttle is None:
            self.forced_throttle = abs(self.throttle) > 0
        if self.forced_acceleration is None:
            self.forced_acceleration = self.forced_throttle and self.throttle > 0
        if self.forced_deceleration is None:
            self.forced_deceleration = self.forced_throttle and self.throttle < 0


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


class LowPassFilter(object):
    def __init__(self, alpha=0.90):
        self._alpha = alpha
        self._value = 0

    def calculate(self, x):
        self._value = (self._alpha * x) + (1. - self._alpha) * self._value
        return self._value


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
        blob.instruction = 'general.fallback'
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
        self._version = 'v2'
        self._desired_speed = 2.0

    def get_action(self, *args):
        blob, vehicle, inference = args
        _do_v1 = inference is None or self._version == 'v1'
        return self._action_v1(*args) if _do_v1 else self._action_v2(*args) if self._version == 'v2' else self._action_v3(*args)

    @staticmethod
    def _action_v1(*args):
        blob, vehicle = args[:2]
        blob.desired_speed = vehicle.get('velocity')
        blob.steering = vehicle.get('auto_steering')
        blob.throttle = vehicle.get('auto_throttle')
        blob.steering_driver = OriginType.BACKEND_AUTOPILOT
        blob.speed_driver = OriginType.BACKEND_AUTOPILOT
        blob.save_event = True

    @staticmethod
    def _action_v2(*args):
        blob, vehicle, inference = args
        blob.desired_speed = vehicle.get('velocity')
        blob.steering = vehicle.get('auto_steering')
        blob.throttle = vehicle.get('auto_throttle')
        blob.steering_driver = OriginType.BACKEND_AUTOPILOT
        blob.speed_driver = OriginType.BACKEND_AUTOPILOT
        blob.save_event = inference.get('corridor') > .95 or inference.get('surprise') > .95

    def _action_v3(self, *args):
        blob, vehicle, inference = args
        self._desired_speed = min(12., max(2., self._desired_speed * 1.0001))
        dnn_desired_speed = self._desired_speed
        dnn_throttle = self.calculate_throttle(desired_speed=dnn_desired_speed, current_speed=vehicle.get('velocity'))
        if inference.get('critic') < 1:
            blob.driver = 'driver_mode.inference.dnn'
            if inference.get('internal') < .050:
                blob.instruction = 'general.fallback'
            elif blob.instruction == 'general.fallback' and inference.get('internal') >= .050:
                blob.instruction = np.random.choice(['intersection.left', 'intersection.ahead', 'intersection.right'])
            blob.cruise_speed = dnn_desired_speed
            blob.desired_speed = dnn_desired_speed
            blob.steering = inference.get('action')
            blob.throttle = dnn_throttle
            blob.steering_driver = OriginType.DNN
            blob.speed_driver = OriginType.DNN
            blob.save_event = False
        else:
            backend_active = vehicle.get('auto_active')
            self._desired_speed = vehicle.get('velocity') if backend_active else self._desired_speed
            blob.driver = 'driver_mode.automatic.backend'
            blob.desired_speed = vehicle.get('velocity') if backend_active else dnn_desired_speed
            blob.steering = vehicle.get('auto_steering') if backend_active else inference.get('action')
            blob.throttle = vehicle.get('auto_throttle') if backend_active else dnn_throttle
            blob.steering_driver = OriginType.BACKEND_AUTOPILOT if backend_active else OriginType.DNN
            blob.speed_driver = OriginType.BACKEND_AUTOPILOT if backend_active else OriginType.DNN
            blob.save_event = backend_active


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
        self.instruction = 'general.fallback'
        self.cruise_speed = 0


class DriverManager(Configurable):
    def __init__(self):
        super(DriverManager, self).__init__()
        self._principal_steer_scale = 0
        self._cruise_speed_step = 0
        self._steering_stabilizer = None
        self._pilot_state = PilotState()
        self._driver_cache = {}
        self._lock = multiprocessing.RLock()
        self._driver = None
        self._driver_ctl = None

    def internal_quit(self, restarting=False):
        for driver in self._driver_cache.values():
            driver.deactivate()
            driver.quit()

    def internal_start(self, **kwargs):
        _errors = []
        self._principal_steer_scale = parse_option('driver.steering.teleop.scale', float, 0, _errors, **kwargs)
        self._cruise_speed_step = parse_option('driver.cc.static.gear.step', float, 0, _errors, **kwargs)
        _steer_low_momentum = parse_option('driver.handler.steering.low_pass.momentum', float, 0, _errors, **kwargs)
        self._steering_stabilizer = LowPassFilter(alpha=_steer_low_momentum)
        self._driver_cache.clear()
        _errors.extend(self._fill_driver_cache(**kwargs))
        self._driver = None
        self._driver_ctl = None
        self.switch_ctl()
        return _errors

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

    def set_cruise_speed(self, value):
        with self._lock:
            self._pilot_state.cruise_speed = max(0., value)
            logger.info("Cruise speed set to '{}'.".format(self._pilot_state.cruise_speed))

    def turn_instruction(self, turn='general.fallback'):
        with self._lock:
            self._pilot_state.instruction = 'general.fallback' if self._pilot_state.instruction == turn else turn

    def get_driver_ctl(self):
        return self._driver_ctl

    def switch_ctl(self, control='driver_mode.teleop.direct'):
        # self.turn_instruction()
        if control is not None and control.lower() not in ('none', 'null', 'ignore', '0', 'false'):
            with self._lock:
                # There is not feedback of this speed in teleoperation mode. Reset at driver changes.
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

    def next_action(self, teleop, vehicle, inference):
        with self._lock:
            # The blob time is now.
            # If downstream processes need the teleop time then use an extra attribute.
            blob = Blob(driver=self._driver_ctl,
                        cruise_speed=self._pilot_state.cruise_speed,
                        instruction=self._pilot_state.instruction,
                        **teleop)
            # Scale teleop before interpretation by the driver.
            blob.steering = self._principal_steer_scale * blob.steering
            self._driver.next_action(blob, vehicle, inference)
            blob.steering = self._steering_stabilizer.calculate(blob.steering)
            return blob


class Navigator(object):
    def __init__(self, route_store):
        self._store = route_store
        self._point = None

    def load_routes(self):
        self._store.load_routes()

    def open(self, route):
        self._store.open(route)

    def update(self, c_inference):
        navigation_point = self._store.get_image_navigation_point(c_inference.get('navigation_image'))
        if self._point == navigation_point:
            return None
        self._point = navigation_point
        return self._store.get_instructions(navigation_point)

    @staticmethod
    def translate_direction(direction):
        if direction == NavigationCommand.LEFT:
            return 'intersection.left'
        elif direction == NavigationCommand.AHEAD:
            return 'intersection.ahead'
        elif direction == NavigationCommand.RIGHT:
            return 'intersection.right'
        return 'general.fallback'


class CommandProcessor(Configurable):
    def __init__(self, route_store):
        super(CommandProcessor, self).__init__()
        self._navigator = Navigator(route_store)
        self._driver = DriverManager()
        self._process_frequency = 10
        self._patience_ms = 10
        self._button_north_ctl = None
        self._button_west_ctl = None
        self._patience_micro = 1000.
        self._navigation_recognition_threshold = 0.
        self._cache = None

    def get_patience_ms(self):
        return self._patience_ms

    def get_frequency(self):
        return self._process_frequency

    def start_route(self, route):
        self._navigator.open(route)

    def internal_quit(self, restarting=False):
        if not restarting:
            self._driver.quit()

    def internal_start(self, **kwargs):
        _errors = []
        self._driver.restart(**kwargs)
        self._process_frequency = parse_option('clock.hz', int, 10, _errors, **kwargs)
        self._patience_ms = parse_option('patience.ms', int, 100, _errors, **kwargs)
        self._button_north_ctl = parse_option('controller.button.north.mode', str, 0, _errors, **kwargs)
        self._button_west_ctl = parse_option('controller.button.west.mode', str, 0, _errors, **kwargs)
        self._navigation_recognition_threshold = parse_option('navigation.point.recognition.threshold', float, 0., _errors, **kwargs)
        self._patience_micro = self._patience_ms * 1000.
        # Avoid processing the same command more than once.
        # TTL is specified in seconds.
        self._cache = cachetools.TTLCache(maxsize=100, ttl=(self._patience_ms * 1e-3))
        self._navigator.load_routes()
        return _errors + self._driver.get_errors()

    def _cache_safe(self, key, func, *arguments):
        if self._cache.get(key) is None:
            try:
                func(*arguments)
            finally:
                self._cache[key] = 1

    def _process(self, c_teleop, c_ros, c_inference):
        c_inference = {} if c_inference is None else c_inference
        if c_inference.get('navigation_distance', 1e9) < self._navigation_recognition_threshold:
            navigation_instruction = self._navigator.update(c_inference)
            if navigation_instruction is not None:
                if navigation_instruction.get_direction() is not None:
                    self._driver.turn_instruction(self._navigator.translate_direction(navigation_instruction.get_direction()))
                if navigation_instruction.get_speed() is not None:
                    self._driver.set_cruise_speed(navigation_instruction.get_speed())

        # Continue with teleop instructions which take precedence over a route.
        c_ros = {} if c_ros is None else c_ros
        if 'pilot.driver.set' in c_ros:
            self._cache_safe('ros switch driver', lambda: self._driver.switch_ctl(c_ros.get('pilot.driver.set')))
        if 'pilot.maximum.speed' in c_ros:
            self._cache_safe('ros set cruise speed', lambda: self._driver.set_cruise_speed(c_ros.get('pilot.maximum.speed')))

        # Continue with teleop instructions which take precedence over the rest.
        c_teleop = {} if c_teleop is None else c_teleop
        if 'quit' in c_teleop:
            self._driver.switch_ctl()
        #
        # Buttons clockwise: N, E, S, W
        # N
        elif c_teleop.get('button_y', 0) == 1:
            self._cache_safe('button north ctl', lambda: self._driver.switch_ctl(self._button_north_ctl))
        # E
        elif c_teleop.get('button_b', 0) == 1:
            self._cache_safe('teleop driver', lambda: self._driver.switch_ctl('driver_mode.teleop.direct'))
        # S
        elif c_teleop.get('button_x', 0) == 1:
            self._cache_safe('button west ctl', lambda: self._driver.switch_ctl(self._button_west_ctl))
        #
        elif c_teleop.get('arrow_up', 0) == 1:
            self._cache_safe('increase cruise speed', lambda: self._driver.increase_cruise_speed())
        elif c_teleop.get('arrow_down', 0) == 1:
            self._cache_safe('decrease cruise speed', lambda: self._driver.decrease_cruise_speed())
        elif c_teleop.get('button_left', 0) == 1:
            self._cache_safe('turn left', lambda: self._driver.turn_instruction('intersection.left'))
        elif c_teleop.get('button_center', 0) == 1:
            self._cache_safe('turn ahead', lambda: self._driver.turn_instruction('intersection.ahead'))
        elif c_teleop.get('button_right', 0) == 1:
            self._cache_safe('turn right', lambda: self._driver.turn_instruction('intersection.right'))

    def next_action(self, *args):
        # A higher patience for teleop commands allows to switch driver etc on slow connections.
        _patience, _ts = self._patience_micro, timestamp()
        # Any of these can be None, too old or repetitive.
        times = [None if arg is None else _ts - arg.get('time') for arg in args]
        arguments = [None if arg is None else arg if (times[i] < _patience) else None for i, arg in enumerate(args)]
        teleop, ros, vehicle, inference = arguments
        # What to do on message timeout depends on which driver is active.
        _ctl = self._driver.get_driver_ctl()
        # Handle instructions.
        self._process(teleop, ros, inference)
        # Switch off autopilot on internal errors.
        if _ctl == 'driver_mode.inference.dnn' and None in (vehicle, inference):
            self._cache_safe('teleop driver', lambda: self._driver.switch_ctl('driver_mode.teleop.direct'))
            return self._driver.noop()
        # Everything normal or there is no autopilot process but teleop should function normally.
        if None not in (teleop, vehicle, inference) or (None not in (teleop, vehicle) and _ctl == 'driver_mode.teleop.direct'):
            return self._driver.next_action(teleop, vehicle, inference)
        # Autopilot drives without teleop commands.
        if None not in (vehicle, inference) and _ctl == 'driver_mode.inference.dnn':
            return self._driver.next_action(dict(), vehicle, inference)
        # Vehicle control by backend.
        if vehicle is not None and _ctl == 'driver_mode.automatic.backend':
            return self._driver.next_action(dict(), vehicle, inference)
        # Ignore old or repetitive teleop commands.
        return self._driver.noop()
