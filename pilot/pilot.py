import json
import logging
import multiprocessing
import threading
import time
import traceback
from abc import ABCMeta, abstractmethod

import cachetools
import pid_controller.pid as pic
from jsoncomment import JsonComment

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
        self.forced_steering = kwargs.get('forced_steering')
        self.forced_throttle = kwargs.get('forced_throttle')
        self.instruction = kwargs.get('instruction')
        self.save_event = kwargs.get('save_event')
        self.speed_driver = kwargs.get('speed_driver')
        self.steering = kwargs.get('steering')
        self.steering_driver = kwargs.get('steering_driver')
        self.throttle = kwargs.get('throttle')
        self.time = kwargs.get('time', time.time())
        self.velocity_y = kwargs.get('velocity_y')
        if self.forced_steering is None and self.steering is not None:
            self.forced_steering = abs(self.steering) > 0
        if self.forced_throttle is None and self.throttle is not None:
            self.forced_throttle = abs(self.throttle) > 0


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
        # Print the changes in configuration or all when not previous configuration is known.
        entries = kwargs.keys()
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
    def get_noop(blob):
        blob.steering = 0
        blob.throttle = 0
        blob.desired_speed = 0
        blob.steering_driver = OriginType.UNDETERMINED
        blob.speed_driver = OriginType.UNDETERMINED
        blob.save_event = False

    def get_next_action(self, blob):
        self.get_action(blob) if self._active else self.get_noop(blob)

    @abstractmethod
    def next_recorder(self, mode):
        raise NotImplementedError()

    @abstractmethod
    def get_action(self, blob):
        raise NotImplementedError()

    def get_navigation_settings(self):
        """Use navigation direction y/n and speed y/n"""
        return False, False

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
    def __init__(self, (p, i, d), stop_p, pid_feedback_cap, throttle_dead_zone, throttle_scale, min_desired_speed, max_desired_speed):
        super(PidThrottleControl, self).__init__(min_desired_speed, max_desired_speed)
        self._pid_throttle = None
        self._pid_stop = None
        self._pid_feedback_cap = pid_feedback_cap
        self._throttle_dead_zone = throttle_dead_zone
        self._throttle_scale = throttle_scale
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
        stop_throttle_ = self._pid_stop(feedback=feedback) * self._throttle_scale
        # Cap the feedback when desired and the current speed is higher.
        feedback = feedback if feedback > 0 else min(0, feedback + self._pid_feedback_cap)
        throttle_ = self._pid_throttle(feedback=feedback) * self._throttle_scale
        # A separate pid controller is engaged when the desired speed drops below a threshold.
        # This could be used for emergency braking e.g. when desired speed is zero.
        if desired_speed <= self._min_desired_speed:
            return min(0, stop_throttle_)
        throttle_ += self._throttle_dead_zone
        return max(-1., min(1., throttle_))


class DirectThrottleControl(AbstractThrottleControl):
    def __init__(self, min_desired_speed, max_desired_speed,
                 throttle_dead_zone, throttle_scale_forwards, throttle_scale_backwards,
                 throttle_up_momentum, throttle_down_momentum):
        super(DirectThrottleControl, self).__init__(min_desired_speed, max_desired_speed)
        self._throttle_scale_forwards = throttle_scale_forwards
        self._throttle_scale_backwards = throttle_scale_backwards
        self._throttle_dead_zone = throttle_dead_zone
        self._moment = DynamicMomentum(up=throttle_up_momentum, down=throttle_down_momentum)

    def calculate_throttle(self, desired_speed, current_speed):
        _scale = self._throttle_scale_backwards if desired_speed < 0 else self._throttle_scale_forwards
        _throttle = desired_speed * _scale
        _throttle = self._moment.calculate(_throttle)
        if _throttle > 1e-3:
            _throttle /= (1 + self._throttle_dead_zone)
            _throttle += self._throttle_dead_zone
        return _throttle


class AbstractCruiseControl(AbstractDriverBase):
    __metaclass__ = ABCMeta

    def __init__(self, control):
        super(AbstractCruiseControl, self).__init__(control)
        self._console_steering_dead_zone = 0
        self._min_desired_speed = 0
        self._max_desired_speed = 0
        self._throttle_control = NoThrottleControl()

    def set_config(self, **kwargs):
        super(AbstractCruiseControl, self).set_config(**kwargs)
        self._console_steering_dead_zone = float(kwargs['driver.console.steering.dead-zone'])
        self._min_desired_speed = float(kwargs['driver.cc.static.speed.min'])
        self._max_desired_speed = float(kwargs['driver.cc.static.speed.max'])
        #
        _control_type = kwargs['driver.cc.control.type']
        _throttle_dead_zone = float(kwargs['driver.cc.throttle.dead-zone'])
        _throttle_scale_forwards = float(kwargs['driver.cc.throttle.scale.forwards'])
        _throttle_scale_backwards = float(kwargs['driver.cc.throttle.scale.backwards'])
        #
        if _control_type == 'pid':
            p = (float(kwargs['driver.cc.throttle.pid_controller.p']))
            i = (float(kwargs['driver.cc.throttle.pid_controller.i']))
            d = (float(kwargs['driver.cc.throttle.pid_controller.d']))
            stop_p = (float(kwargs['driver.cc.stop.pid_controller.p']))
            _pid_feedback_cap = float(kwargs['driver.cc.throttle.pid.cap.feedback'])
            self._throttle_control = PidThrottleControl(
                (p, i, d),
                stop_p=stop_p,
                pid_feedback_cap=_pid_feedback_cap,
                throttle_dead_zone=_throttle_dead_zone,
                throttle_scale=_throttle_scale_forwards,
                min_desired_speed=self._min_desired_speed,
                max_desired_speed=self._max_desired_speed
            )
        else:
            _throttle_up_momentum = float(kwargs['driver.throttle.direct.up.momentum'])
            _throttle_down_momentum = float(kwargs['driver.throttle.direct.down.momentum'])
            self._throttle_control = DirectThrottleControl(
                min_desired_speed=self._min_desired_speed,
                max_desired_speed=self._max_desired_speed,
                throttle_dead_zone=_throttle_dead_zone,
                throttle_scale_forwards=_throttle_scale_forwards,
                throttle_scale_backwards=_throttle_scale_backwards,
                throttle_up_momentum=_throttle_up_momentum,
                throttle_down_momentum=_throttle_down_momentum
            )

    def calculate_desired_speed(self, desired_speed, throttle, forced_acceleration, forced_deceleration, maximum=None):
        return self._throttle_control.calculate_desired_speed(desired_speed, throttle, forced_acceleration, forced_deceleration, maximum)

    def calculate_throttle(self, desired_speed, current_speed):
        return 0 if None in (desired_speed, current_speed) else self._throttle_control.calculate_throttle(desired_speed, current_speed)


class RawConsoleDriver(AbstractCruiseControl):
    def __init__(self):
        super(RawConsoleDriver, self).__init__('driver.mode.console')
        self._throttle_dead_zone = 0.
        self._throttle_scale_forwards = 0.
        self._throttle_scale_backwards = 0.

    def set_config(self, **kwargs):
        super(RawConsoleDriver, self).set_config(**kwargs)
        self._throttle_dead_zone = float(kwargs['driver.cc.throttle.dead-zone'])
        self._throttle_scale_forwards = float(kwargs['driver.cc.throttle.scale.forwards'])
        self._throttle_scale_backwards = float(kwargs['driver.cc.throttle.scale.backwards'])

    def next_recorder(self, mode=None):
        return None

    def get_navigation_settings(self):
        return False, False

    def _raw_throttle(self, value):
        # Leave throttle zero when not set.
        _min_throttle = 1e-4
        # Shift the scaled throttle by dead zone.
        _scale = self._throttle_scale_backwards if value < 0 else self._throttle_scale_forwards
        _throttle = value * _scale
        if _throttle > _min_throttle:
            _throttle /= (1 + self._throttle_dead_zone)
            _throttle += self._throttle_dead_zone
        return _throttle

    def get_action(self, blob):
        blob.steering = self._apply_dead_zone(blob.steering, dead_zone=self._console_steering_dead_zone)
        blob.throttle = self._raw_throttle(value=blob.throttle)
        blob.desired_speed = blob.throttle * self._max_desired_speed
        # No recording in this mode.
        blob.steering_driver = OriginType.UNDETERMINED
        blob.speed_driver = OriginType.UNDETERMINED
        blob.save_event = False
        # blob.publish = blob.client_command is not None


class BackendAutopilotDriver(AbstractCruiseControl):
    def __init__(self):
        super(BackendAutopilotDriver, self).__init__('driver.mode.automatic')

    def next_recorder(self, mode=None):
        return 'record.mode.driving'

    def get_navigation_settings(self):
        return False, False

    def get_action(self, blob):
        blob.desired_speed = blob.throttle * self._max_desired_speed
        blob.steering_driver = OriginType.BACKEND_AUTOPILOT
        blob.speed_driver = OriginType.BACKEND_AUTOPILOT
        blob.save_event = True
        # blob.publish = True


class StaticCruiseDriver(AbstractCruiseControl):
    def __init__(self):
        super(StaticCruiseDriver, self).__init__('driver.mode.cruise')

    def next_recorder(self, mode=None):
        return 'record.mode.driving'

    def get_navigation_settings(self):
        return False, False

    def get_action(self, blob):
        blob.desired_speed = self.calculate_desired_speed(desired_speed=blob.cruise_speed,
                                                          throttle=blob.throttle,
                                                          forced_acceleration=(blob.forced_throttle and blob.throttle > 0),
                                                          forced_deceleration=(blob.forced_throttle and blob.throttle < 0))
        blob.steering = self._apply_dead_zone(blob.steering, dead_zone=self._console_steering_dead_zone)
        blob.throttle = self.calculate_throttle(blob.desired_speed, blob.velocity_y)
        blob.steering_driver = OriginType.HUMAN
        blob.speed_driver = OriginType.HUMAN
        blob.save_event = True
        # blob.publish = blob.client_command is not None


class PilotState(object):
    def __init__(self):
        self.instruction = 'intersection.ahead'
        self.cruise_speed = 0


class DriverManager(object):
    def __init__(self, config_file):
        self._config_file = config_file
        self._pilot_state = PilotState()
        self._driver_cache = {}
        self._driver_lock = multiprocessing.RLock()
        self._driver = None
        self._driver_ctl = 'driver.mode.console'
        self.switch_ctl(self._driver_ctl)

    def _get_driver(self, control=None):
        if control in self._driver_cache:
            return self._driver_cache[control]
        # Create a new instance of the driver.
        # if control == 'driver.mode.dager':
        #     driver = DeepNetworkDriver(self._tf_driver, dagger=True)
        # elif control == 'driver.mode.dnn':
        #     driver = DeepNetworkDriver(self._tf_driver, dagger=False)
        if control == 'driver.mode.cruise':
            driver = StaticCruiseDriver()
        elif control == 'driver.mode.automatic':
            driver = BackendAutopilotDriver()
        else:
            driver = RawConsoleDriver()
        # Cache the new instance.
        self._driver_cache[control] = driver
        return driver

    def _activate(self):
        try:
            self._driver_lock.acquire()
            if self._driver is not None:
                with open(self._config_file, 'r') as cfg_file:
                    cfg = JsonComment(json).loads(cfg_file.read())
                self._driver.set_config(**cfg)
                self._driver.activate()
        except Exception as e:
            logger.error("Driver activation: {}".format(traceback.format_exc(e)))
        finally:
            self._driver_lock.release()

    def switch_ctl(self, control=None):
        # The switch must be immediate. Do not force wait on the previous driver to deactivate.
        if self._driver is not None:
            threading.Thread(target=self._driver.deactivate).start()
        self._driver_ctl = control
        self._driver = self._get_driver(control=control)
        threading.Thread(target=self._activate).start()

    def get_next_action(self, command, velocity_y):
        # Say 50 ms is the ipc connectivity minimum.
        if command is None or time.time() - command.get('time') > .049999:
            blob = Blob()
            self._driver.get_noop(blob)
        else:
            blob = Blob(driver=self._driver_ctl,
                        cruise_speed=self._pilot_state.cruise_speed,
                        velocity_y=velocity_y,
                        **command)
            self._driver.get_next_action(blob)
        return blob

    def quit(self):
        for driver in self._driver_cache.values():
            driver.deactivate()
            driver.quit()


class CommandProcessor(object):
    def __init__(self, driver):
        self._driver = driver
        # Avoid processing the same command more than once.
        # TTL is specified in seconds.
        self._cache = cachetools.TTLCache(maxsize=100, ttl=.35)

    def _cache_safe(self, key, func, *arguments):
        if self._cache.get(key) is None:
            try:
                func(*arguments)
            finally:
                self._cache[key] = 1

    def process(self, command):
        keys = {} if command is None else command.keys()
        if 'quit' in keys:
            self._driver.switch_ctl()
        #
        # Buttons clockwise: N, E, S, W
        # N
        elif 'button_y' in keys and command['button_y'] == 1:
            self._cache_safe('dnn driver', lambda: self._driver.switch_ctl('driver.mode.dnn'))
        # E
        elif 'button_b' in keys and command['button_b'] == 1:
            self._cache_safe('console driver', lambda: self._driver.switch_ctl('driver.mode.console'))
        # S
        elif 'button_a' in keys and command['button_a'] == 1:
            self._cache_safe('cruise driver', lambda: self._driver.switch_ctl('driver.mode.cruise'))
        # W
        elif 'button_x' in keys and command['button_x'] == 1:
            self._cache_safe('dagger driver', lambda: self._driver.switch_ctl('driver.mode.dagger'))
