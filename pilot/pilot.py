import logging
import multiprocessing
import threading
import traceback
from abc import ABCMeta, abstractmethod

import cachetools
import pid_controller.pid as pic

from byodr.utils import timestamp

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
        self.time = kwargs.get('time', timestamp())
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
        self._previous_config = None

    def set_config(self, **kwargs):
        # Print the changes in configuration when the previous configuration is known.
        entries = []
        if self._previous_config is not None:
            entries = filter(lambda k: kwargs.get(k, None) != self._previous_config.get(k, None), kwargs.keys())
        for key in sorted(entries):
            logger.info("{} = {}".format(key, kwargs[key]))
        self._previous_config = kwargs

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
        blob.desired_speed = 0
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


class NoThrottleControl(AbstractThrottleControl):
    def __init__(self):
        super(NoThrottleControl, self).__init__(min_desired_speed=0, max_desired_speed=0)

    def calculate_throttle(self, desired_speed, current_speed):
        return 0


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
    def __init__(self, min_desired_speed, max_desired_speed, throttle_up_momentum, throttle_down_momentum):
        super(DirectThrottleControl, self).__init__(min_desired_speed, max_desired_speed)
        self._moment = DynamicMomentum(up=throttle_up_momentum, down=throttle_down_momentum)

    def calculate_throttle(self, desired_speed, current_speed):
        _throttle = desired_speed
        _throttle = self._moment.calculate(_throttle)
        return _throttle


class AbstractCruiseControl(AbstractDriverBase):
    __metaclass__ = ABCMeta

    def __init__(self, control):
        super(AbstractCruiseControl, self).__init__(control)
        self._min_desired_speed = 0
        self._max_desired_speed = 0
        self._throttle_control = NoThrottleControl()

    def set_config(self, **kwargs):
        super(AbstractCruiseControl, self).set_config(**kwargs)
        self._min_desired_speed = float(kwargs['driver.cc.static.speed.min'])
        self._max_desired_speed = float(kwargs['driver.cc.static.speed.max'])
        #
        _control_type = kwargs['driver.cc.control.type']
        #
        if _control_type == 'pid':
            p = (float(kwargs['driver.cc.throttle.pid_controller.p']))
            i = (float(kwargs['driver.cc.throttle.pid_controller.i']))
            d = (float(kwargs['driver.cc.throttle.pid_controller.d']))
            stop_p = (float(kwargs['driver.cc.stop.pid_controller.p']))
            self._throttle_control = PidThrottleControl(
                (p, i, d),
                stop_p=stop_p,
                min_desired_speed=self._min_desired_speed,
                max_desired_speed=self._max_desired_speed
            )
        else:
            _throttle_up_momentum = float(kwargs['driver.throttle.direct.up.momentum'])
            _throttle_down_momentum = float(kwargs['driver.throttle.direct.down.momentum'])
            self._throttle_control = DirectThrottleControl(
                min_desired_speed=self._min_desired_speed,
                max_desired_speed=self._max_desired_speed,
                throttle_up_momentum=_throttle_up_momentum,
                throttle_down_momentum=_throttle_down_momentum
            )

    def calculate_desired_speed(self, desired_speed, throttle, forced_acceleration, forced_deceleration, maximum=None):
        return self._throttle_control.calculate_desired_speed(desired_speed, throttle, forced_acceleration, forced_deceleration, maximum)

    def calculate_throttle(self, desired_speed, current_speed):
        return 0 if None in (desired_speed, current_speed) else self._throttle_control.calculate_throttle(desired_speed, current_speed)


class RawConsoleDriver(AbstractCruiseControl):
    def __init__(self):
        super(RawConsoleDriver, self).__init__('driver_mode.teleop.direct')

    def set_config(self, **kwargs):
        super(RawConsoleDriver, self).set_config(**kwargs)

    def get_action(self, *args):
        blob = args[0]
        blob.steering = self._apply_dead_zone(blob.steering, dead_zone=0)
        blob.desired_speed = blob.throttle * self._max_desired_speed
        blob.steering_driver = OriginType.UNDETERMINED
        blob.speed_driver = OriginType.UNDETERMINED
        # No recording in this mode.
        blob.save_event = False


class BackendAutopilotDriver(AbstractCruiseControl):
    def __init__(self):
        super(BackendAutopilotDriver, self).__init__('driver_mode.automatic.backend')

    def get_action(self, *args):
        blob = args[0]
        blob.desired_speed = blob.throttle * self._max_desired_speed
        blob.steering_driver = OriginType.BACKEND_AUTOPILOT
        blob.speed_driver = OriginType.BACKEND_AUTOPILOT
        blob.save_event = True


class StaticCruiseDriver(AbstractCruiseControl):
    def __init__(self):
        super(StaticCruiseDriver, self).__init__('driver_mode.teleop.cruise')

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


class DeepNetworkDriver(AbstractCruiseControl):

    def __init__(self):
        super(DeepNetworkDriver, self).__init__('driver_mode.inference.dnn')
        self._piv_count = 0

    def set_config(self, **kwargs):
        super(DeepNetworkDriver, self).set_config(**kwargs)

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
        _use_expert_steering = blob.forced_steering
        if _use_expert_steering or blob.forced_acceleration:
            # Any intervention type is sufficient to set the steering coming in from the user.
            steering = blob.steering
            # Mark the first few interventions as such.
            if self._piv_count > 2:
                steering_driver = (OriginType.DAGGER if _dagger else OriginType.CONSOLE)
            else:
                self._piv_count += 1
                steering_driver = OriginType.DNN_PRE_INTERVENTION
        else:
            steering, steering_driver = action_out, OriginType.DNN
            self._piv_count = 0
        blob.steering = self._apply_dead_zone(steering, dead_zone=0)
        blob.steering_driver = steering_driver
        blob.save_event = _use_expert_steering or _speed_intervention


class PilotState(object):
    def __init__(self):
        self.instruction = 'intersection.ahead'
        self.cruise_speed = 0


class DriverManager(object):
    def __init__(self, **kwargs):
        self._settings = kwargs
        self._principal_steer_scale = float(kwargs['driver.steering.teleop.scale'])
        self._cruise_speed_step = float(kwargs['driver.cc.static.gear.step'])
        self._steering_stabilizer = IgnoreDifferences(threshold=float(kwargs['driver.handler.steering.diff.threshold']))
        self._pilot_state = PilotState()
        self._driver_cache = {}
        self._lock = multiprocessing.RLock()
        self._driver = None
        self._driver_ctl = None
        self.switch_ctl()

    def _get_driver(self, control=None):
        if control in self._driver_cache:
            return self._driver_cache[control]
        # Create a new instance of the driver.
        if control == 'driver_mode.inference.dnn':
            driver = DeepNetworkDriver()
        elif control == 'driver_mode.teleop.cruise':
            driver = StaticCruiseDriver()
        elif control == 'driver_mode.automatic.backend':
            driver = BackendAutopilotDriver()
        else:
            driver = RawConsoleDriver()
        # Cache the new instance.
        self._driver_cache[control] = driver
        return driver

    def _activate(self):
        try:
            self._lock.acquire()
            if self._driver is not None:
                self._driver.set_config(**self._settings)
                self._driver.activate()
        except Exception as e:
            logger.error("Driver activation: {}".format(traceback.format_exc(e)))
        finally:
            self._lock.release()

    def increase_cruise_speed(self):
        with self._lock:
            self._pilot_state.cruise_speed += self._cruise_speed_step
            logger.info("Cruise speed set to '{}'.".format(self._pilot_state.cruise_speed))

    def decrease_cruise_speed(self):
        with self._lock:
            self._pilot_state.cruise_speed = max(0., self._pilot_state.cruise_speed - self._cruise_speed_step)
            logger.info("Cruise speed set to '{}'.".format(self._pilot_state.cruise_speed))

    def turn_instruction(self, turn):
        with self._lock:
            self._pilot_state.instruction = turn
            logger.info("Instruction set to '{}'.".format(turn))

    def switch_ctl(self, control='driver_mode.teleop.direct'):
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
            # The blob time must be based on the command teleop time to allow downstream processes to react.
            blob = Blob(driver=self._driver_ctl,
                        cruise_speed=self._pilot_state.cruise_speed,
                        instruction=self._pilot_state.instruction,
                        **command)
            blob.steering = self._principal_steer_scale * blob.steering
            self._driver.next_action(blob, vehicle, inference)
            blob.steering = self._steering_stabilizer.calculate(blob.steering)
        return blob

    def quit(self):
        for driver in self._driver_cache.values():
            driver.deactivate()
            driver.quit()


class CommandProcessor(object):
    def __init__(self, driver, patience_ms=100.):
        self._driver = driver
        self._patience_micro = patience_ms * 1000
        # Avoid processing the same command more than once.
        # TTL is specified in seconds.
        self._cache = cachetools.TTLCache(maxsize=100, ttl=(patience_ms * 1e-3))

    def _cache_safe(self, key, func, *arguments):
        if self._cache.get(key) is None:
            try:
                func(*arguments)
            finally:
                self._cache[key] = 1

    def _process(self, command):
        keys = {} if command is None else command.keys()
        if 'quit' in keys:
            self._driver.switch_ctl()
        #
        # Buttons clockwise: N, E, S, W
        # N
        elif command.get('button_y', 0) == 1:
            self._cache_safe('dnn driver', lambda: self._driver.switch_ctl('driver_mode.inference.dnn'))
        # E
        elif command.get('button_b', 0) == 1:
            self._cache_safe('teleop driver', lambda: self._driver.switch_ctl('driver_mode.teleop.direct'))
        # S
        elif command.get('button_a', 0) == 1:
            self._cache_safe('cruise driver', lambda: self._driver.switch_ctl('driver_mode.teleop.cruise'))
        # W
        elif command.get('button_x', 0) == 1:
            self._cache_safe('dagger driver', lambda: self._driver.switch_ctl('driver_mode.inference.dnn'))
        #
        elif command.get('arrow_up', 0) == 1:
            self._cache_safe('increase cruise speed', lambda: self._driver.increase_cruise_speed())
        elif command.get('arrow_down', 0) == 1:
            self._cache_safe('decrease cruise speed', lambda: self._driver.decrease_cruise_speed())
        elif command.get('button_left', 0) == 1:
            self._cache_safe('turn left', lambda: self._driver.turn_instruction('intersection.left'))
        elif command.get('button_back', 0) == 1:
            self._cache_safe('turn ahead', lambda: self._driver.turn_instruction('intersection.ahead'))
        elif command.get('button_right', 0) == 1:
            self._cache_safe('turn right', lambda: self._driver.turn_instruction('intersection.right'))

    def quit(self):
        self._driver.quit()

    def next_action(self, *args):
        _ts = timestamp()
        _patience = self._patience_micro
        times = [None if arg is None else _ts - arg.get('time') for arg in args]
        commands = [None if arg is None else arg if (times[i] < _patience) else None for i, arg in enumerate(args)]
        if None in commands:
            self._cache_safe('teleop driver', lambda: self._driver.switch_ctl('driver_mode.teleop.direct'))
        teleop, vehicle, inference = commands
        if teleop is None:
            return None, False
        else:
            self._process(teleop)
            return self._driver.next_action(teleop, vehicle, inference), True
