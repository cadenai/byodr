import collections
import logging
import multiprocessing
import threading
import traceback
from abc import ABCMeta, abstractmethod
from collections import deque

import cachetools
import numpy as np
import pid_controller.pid as pic

from byodr.utils import timestamp, Configurable
from byodr.utils.navigate import NavigationCommand, NavigationInstructions
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


def _is_forced_value(value):
    return abs(0 if value is None else value) > 0


class Blob(AttrDict):
    def __init__(self, **kwargs):
        super(Blob, self).__init__(**kwargs)
        self.time = timestamp()
        self.speed_scale = kwargs.get('speed_scale')
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
        self.button_left = kwargs.get('button_left')
        self.button_right = kwargs.get('button_right')
        self.navigation_active = kwargs.get('navigation_active')
        self.navigation_route = kwargs.get('navigation_route')
        self.navigation_match_image = kwargs.get('navigation_match_image', -1)
        self.navigation_match_distance = kwargs.get('navigation_match_distance', 1)
        self.navigation_match_point = kwargs.get('navigation_match_point')
        self.inference_brake = kwargs.get('inference_brake', 0)
        if self.forced_steering is None:
            self.forced_steering = _is_forced_value(self.steering)
        if self.forced_throttle is None:
            self.forced_throttle = _is_forced_value(self.throttle)
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
    def __init__(self, initial_value=0., alpha=0.90):
        self._value = initial_value
        self._alpha = alpha

    def get(self):
        return self._value

    def update(self, x):
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
        self._min_desired_speed = parse_option('driver.cc.static.speed.min', float, 0.1, _errors, **kwargs)
        self._max_desired_speed = parse_option('driver.cc.static.speed.max', float, 5.0, _errors, **kwargs)
        #
        _control_type = parse_option('driver.cc.control.type', str, 'pid', _errors, **kwargs)
        if _control_type == 'pid':
            p = (parse_option('driver.cc.throttle.pid_controller.p', float, 0.6, _errors, **kwargs))
            i = (parse_option('driver.cc.throttle.pid_controller.i', float, 2.0, _errors, **kwargs))
            d = (parse_option('driver.cc.throttle.pid_controller.d', float, 0, _errors, **kwargs))
            stop_p = (parse_option('driver.cc.stop.pid_controller.p', float, 1.0, _errors, **kwargs))
            self._throttle_control = PidThrottleControl(
                (p, i, d),
                stop_p=stop_p,
                min_desired_speed=self._min_desired_speed,
                max_desired_speed=self._max_desired_speed
            )
        else:
            _throttle_up_momentum = parse_option('driver.throttle.direct.up.momentum', float, 0.05, _errors, **kwargs)
            _throttle_down_momentum = parse_option('driver.throttle.direct.down.momentum', float, 1.0, _errors, **kwargs)
            _throttle_cutoff = parse_option('driver.throttle.direct.minimum', float, 0.002, _errors, **kwargs)
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


def _translate_navigation_direction(direction):
    if direction == NavigationCommand.LEFT:
        return 'intersection.left'
    elif direction == NavigationCommand.AHEAD:
        return 'intersection.ahead'
    elif direction == NavigationCommand.RIGHT:
        return 'intersection.right'
    return 'general.fallback'


class Navigator(object):
    def __init__(self, route_store):
        self._store = route_store
        self._open_lock = threading.Lock()
        self._override_requests = collections.deque(maxlen=1)
        self._match_point = None
        self._match_image = None
        self._match_distance = None

    def initialize(self):
        self.reload()

    def reload(self):
        self._store.load_routes()

    def _open_store(self, route):
        # Even though store is thread safe prevent concurrent store open processes which are started by threads in this class.
        _acquired = self._open_lock.acquire(False)
        try:
            if _acquired:
                self._store.open(route)
        finally:
            if _acquired:
                self._open_lock.release()

    def close(self):
        self._store.close()
        self._match_point = None
        self._match_image = None
        self._match_distance = None

    def is_active(self):
        return len(self._store) > 0

    def get_navigation_route(self):
        return self._store.get_selected_route()

    def get_match_image_id(self):
        return self._match_image

    def get_match_point(self):
        return self._match_point

    def get_match_distance(self):
        return self._match_distance

    def handle_teleop_state(self, c_teleop):
        # This must run quickly.
        # The teleop service is the authority on route state.
        route = c_teleop.get('navigator').get('route', None)
        if route is None:
            self.close()
        elif route not in self._store.list_routes():
            threading.Thread(target=self.reload).start()
        elif route != self._store.get_selected_route():
            threading.Thread(target=self._open_store, args=(route,)).start()

    def set_override_request(self, request):
        self._override_requests.append(request)

    def _get_override_request(self):
        try:
            return self._override_requests[-1]
        except IndexError:
            return None

    def update(self, c_inference):
        # This runs at the service process frequency.
        #   /navigate?action='halt'
        #   /navigate?action='halt'&route=''&point=''
        #   /navigate?action='resume'&route=''&speed=1
        # Some overrides are independent of a matched point.
        instructions = None
        _override = self._get_override_request()
        r_action = None if _override is None else _override.get('action', None)
        r_point = None if _override is None else _override.get('point', None)
        r_speed = None if _override is None else _override.get('speed', None)
        if r_action == 'halt' and r_point is None:
            instructions = NavigationInstructions(commands=NavigationCommand(speed=0))
            self._override_requests.clear()
        elif r_action == 'resume' and r_speed is not None:
            instructions = NavigationInstructions(commands=NavigationCommand(speed=float(r_speed)))
            self._override_requests.clear()
        elif self.is_active():
            try:
                _point_id = c_inference.get('navigation_point')
                _image_id = c_inference.get('navigation_image')
                _distance = c_inference.get('navigation_distance', 1.)
                if _point_id >= 0:
                    _point_name = self._store.list_navigation_points()[_point_id]
                    if _point_name != self._match_point:
                        self._match_point = _point_name
                        self._match_image = _image_id
                        self._match_distance = _distance
                        if r_action == 'halt' and r_point == _point_name:
                            instructions = NavigationInstructions(commands=NavigationCommand(speed=0))
                            self._override_requests.clear()
                        else:
                            instructions = self._store.get_instructions(_point_name)
            except LookupError:
                pass
        # No new match.
        return instructions


class DriverManager(Configurable):
    def __init__(self, route_store):
        super(DriverManager, self).__init__()
        self._navigator = Navigator(route_store)
        self._navigation_queue = deque(maxlen=10)
        self._principal_steer_scale = 0
        # Static conversion factor of m/s to km/h.
        self._speed_scale = 3.6
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
        _steer_low_momentum = parse_option('driver.handler.steering.low_pass.momentum', float, 0.40, _errors, **kwargs)
        self._principal_steer_scale = parse_option('driver.steering.teleop.scale', float, 0.45, _errors, **kwargs)
        self._cruise_speed_step = parse_option('driver.cc.static.gear.step', float, 0.50, _errors, **kwargs)
        self._steering_stabilizer = LowPassFilter(alpha=_steer_low_momentum)
        self._navigator.initialize()
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

    def process_override(self, c_external):
        r_action = None if c_external is None else c_external.get('action', None)
        if r_action in ('halt', 'resume'):
            self._navigator.set_override_request(c_external)
        elif r_action is not None:
            logger.warning("Illegal override request {}".format(c_external))

    def process_navigation(self, c_teleop, c_inference):
        # This runs at the service process frequency.
        # Leave the state as is on empty teleop state.
        if c_teleop is not None:
            self._navigator.handle_teleop_state(c_teleop)
        try:
            # Peek the first command in execution order.
            command = self._navigation_queue[0]
            if command.get_sleep() is None or timestamp() > command.get_time() + command.get_sleep() * 1e6:
                # Execute the command now.
                command = self._navigation_queue.popleft()
                if command.get_direction() is not None:
                    self._set_direction(_translate_navigation_direction(command.get_direction()))
                if command.get_speed() is not None:
                    self.set_cruise_speed(command.get_speed())
        except LookupError:
            pass
        # Fill the queue with the next instructions in order.
        c_inference = {} if c_inference is None else c_inference
        navigation_instructions = self._navigator.update(c_inference)
        if navigation_instructions is not None:
            self._navigation_queue.extend([c.set_time(timestamp()) for c in navigation_instructions.get_commands()])

    def increase_cruise_speed(self):
        if self.get_driver_ctl() != 'driver_mode.teleop.direct':
            self.set_cruise_speed(self._cruise_speed_step, relative=True)

    def decrease_cruise_speed(self):
        if self.get_driver_ctl() != 'driver_mode.teleop.direct':
            self.set_cruise_speed(-self._cruise_speed_step, relative=True)

    def set_cruise_speed(self, value, relative=False):
        with self._lock:
            new_val = (value / self._speed_scale)
            new_val = max(0., self._pilot_state.cruise_speed + new_val if relative else new_val)
            self._pilot_state.cruise_speed = new_val

    def _set_direction(self, turn='general.fallback'):
        pass
        # with self._lock:
        #     self._pilot_state.instruction = turn

    def teleop_direction(self, turn='general.fallback'):
        pass
        # with self._lock:
        #     self._pilot_state.instruction = 'general.fallback' if self._pilot_state.instruction == turn else turn

    def get_driver_ctl(self):
        return self._driver_ctl

    def switch_ctl(self, control='driver_mode.teleop.direct'):
        # self.turn_instruction()
        if control is not None and control.lower() not in ('none', 'null', 'ignore', '0', 'false'):
            with self._lock:
                # There is not feedback of this speed in teleop mode. Reset at driver changes.
                self._pilot_state.cruise_speed = 0
                # The switch must be immediate. Do not force wait on the previous driver to deactivate.
                if control != self._driver_ctl:
                    if self._driver is not None:
                        threading.Thread(target=self._driver.deactivate).start()
                    self._driver_ctl = control
                    self._navigation_queue.clear()
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
            _nav_active = self._navigator.is_active()
            _nav_route = self._navigator.get_navigation_route() if _nav_active else None
            _nav_match_image = self._navigator.get_match_image_id() if _nav_active else None
            _nav_match_distance = self._navigator.get_match_distance() if _nav_active else None
            _nav_match_point = self._navigator.get_match_point() if _nav_active else None
            _inference_brake = 0. if inference is None else inference.get('obstacle', 0.)
            blob = Blob(driver=self._driver_ctl,
                        speed_scale=self._speed_scale,
                        cruise_speed=self._pilot_state.cruise_speed,
                        instruction=self._pilot_state.instruction,
                        navigation_active=_nav_active,
                        navigation_route=_nav_route,
                        navigation_match_image=_nav_match_image,
                        navigation_match_distance=_nav_match_distance,
                        navigation_match_point=_nav_match_point,
                        inference_brake=_inference_brake,
                        **teleop)
            # Scale teleop before interpretation by the driver.
            blob.steering = self._principal_steer_scale * blob.steering
            self._driver.next_action(blob, vehicle, inference)
            blob.steering = self._steering_stabilizer.update(blob.steering)
            return blob


class CommandProcessor(Configurable):
    def __init__(self, route_store):
        super(CommandProcessor, self).__init__()
        self._driver = DriverManager(route_store)
        self._process_frequency = 10
        self._patience_ms = 10
        self._button_north_ctl = None
        self._button_west_ctl = None
        self._patience_micro = 1000.
        self._cache = None

    def get_patience_ms(self):
        return self._patience_ms

    def get_frequency(self):
        return self._process_frequency

    def internal_quit(self, restarting=False):
        if not restarting:
            self._driver.quit()

    def internal_start(self, **kwargs):
        _errors = []
        self._driver.restart(**kwargs)
        self._process_frequency = parse_option('clock.hz', int, 100, _errors, **kwargs)
        self._patience_ms = parse_option('patience.ms', int, 500, _errors, **kwargs)
        self._button_north_ctl = parse_option('controller.button.north.mode', str, 'driver_mode.inference.dnn', _errors, **kwargs)
        self._button_west_ctl = parse_option('controller.button.west.mode', str, 'ignore', _errors, **kwargs)
        self._patience_micro = self._patience_ms * 1000.
        # Avoid processing the same command more than once.
        # TTL is specified in seconds.
        self._cache = cachetools.TTLCache(maxsize=100, ttl=(self._patience_ms * 1e-3))
        return _errors + self._driver.get_errors()

    def _cache_safe(self, key, func, *arguments):
        if self._cache.get(key) is None:
            try:
                func(*arguments)
            finally:
                self._cache[key] = 1

    def _process(self, c_teleop, c_external, c_ros, c_inference):
        r_action = None if c_external is None else c_external.get('action', None)
        if r_action == 'resume':
            self._cache_safe('external api call', lambda: self._driver.switch_ctl('driver_mode.inference.dnn'))

        self._driver.process_navigation(c_teleop, c_inference)
        self._driver.process_override(c_external)

        # Continue with ros instructions which take precedence over a route.
        c_ros = {} if c_ros is None else c_ros
        if 'pilot.driver.set' in c_ros:
            self._cache_safe('ros switch driver', lambda: self._driver.switch_ctl(c_ros.get('pilot.driver.set')))
        if 'pilot.maximum.speed' in c_ros:
            self._cache_safe('ros set cruise speed', lambda: self._driver.set_cruise_speed(c_ros.get('pilot.maximum.speed')))

        c_teleop = {} if c_teleop is None else c_teleop
        # Steering interventions must be accompanied with throttle.
        if _is_forced_value(c_teleop.get('steering')) or _is_forced_value(c_teleop.get('throttle')):
            self._driver.set_cruise_speed(0)

        # Continue with teleop instructions which take precedence over the rest.
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
            self._cache_safe('turn left', lambda: self._driver.teleop_direction('intersection.left'))
        elif c_teleop.get('button_center', 0) == 1:
            self._cache_safe('turn ahead', lambda: self._driver.teleop_direction('intersection.ahead'))
        elif c_teleop.get('button_right', 0) == 1:
            self._cache_safe('turn right', lambda: self._driver.teleop_direction('intersection.right'))

    def _unpack_commands(self, teleop, external, ros, vehicle, inference):
        _patience, _ts = self._patience_micro, timestamp()
        # The teleop, vehicle or inference commands could be none, old or repeated.
        teleop = teleop if teleop is not None and (_ts - teleop.get('time') < _patience) else None
        vehicle = vehicle if vehicle is not None and (_ts - vehicle.get('time') < _patience) else None
        inference = inference if inference is not None and (_ts - inference.get('time') < _patience) else None
        # The external and ros commands need to be handled each occurrence.
        return teleop, external, ros, vehicle, inference

    def next_action(self, *args):
        teleop, external, ros, vehicle, inference = self._unpack_commands(*args)
        # Handle instructions first.
        self._process(teleop, external, ros, inference)
        # What to do on message timeout depends on which driver is active.
        _ctl = self._driver.get_driver_ctl()
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
