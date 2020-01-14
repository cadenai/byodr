import argparse
import collections
import json
import logging
import multiprocessing
import sys
import threading
import time
import traceback
from abc import ABCMeta, abstractmethod

import pid_controller.pid as pic
import rospy
from std_msgs.msg import String as RosString

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()


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

    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self


class Blob(AttrDict):
    def __init__(self, *args, **kwargs):
        super(Blob, self).__init__(*args, **kwargs)
        self.desired_speed = kwargs.get('desired_speed')
        self.driver = kwargs.get('driver')
        self.save_event = kwargs.get('save_event')
        self.speed_driver = kwargs.get('speed_driver')
        self.steering = kwargs.get('steering')
        self.steering_driver = kwargs.get('steering_driver')
        self.throttle = kwargs.get('throttle')


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

    def log_config(self):
        return False

    def set_config(self, **kwargs):
        # Print the changes in configuration.
        if self.log_config() and self._previous_config is not None:
            for key in sorted(filter(lambda k: kwargs.get(k, None) != self._previous_config.get(k, None), kwargs.keys())):
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
        return self._throttle_control.calculate_throttle(desired_speed, current_speed)


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

    def log_config(self):
        return True

    def next_recorder(self, mode=None):
        return 'record.mode.driving'

    def get_navigation_settings(self):
        return False, False

    def get_action(self, blob):
        blob.desired_speed = self.calculate_desired_speed(desired_speed=blob.road_speed,
                                                          throttle=blob.throttle,
                                                          forced_acceleration=blob.forced_acceleration,
                                                          forced_deceleration=blob.forced_deceleration)
        blob.steering = self._apply_dead_zone(blob.steering, dead_zone=self._console_steering_dead_zone)
        blob.throttle = self.calculate_throttle(blob.desired_speed, blob.velocity_y)
        blob.steering_driver = OriginType.HUMAN
        blob.speed_driver = OriginType.HUMAN
        blob.save_event = True
        # blob.publish = blob.client_command is not None


class DriverManager(object):
    def __init__(self):
        self._driver_cache = {}
        self._driver_lock = multiprocessing.RLock()
        self._driver_ctl = 'driver.mode.console'
        self._driver = self._get_driver(self._driver_ctl)
        self._driver.activate()

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

    def _activate(self, cfg):
        try:
            self._driver_lock.acquire()
            if self._driver is not None:
                self._driver.set_config(**cfg)
                self._driver.activate()
        except Exception as e:
            logger.error("Driver activation: {}".format(traceback.format_exc(e)))
        finally:
            self._driver_lock.release()

    def on_command(self, cmd):
        pass

    def _switch_ctl(self, control, cfg):
        # The switch must be immediate. Do not force wait on the previous driver to deactivate.
        if self._driver is not None:
            threading.Thread(target=self._driver.deactivate).start()
        self._driver_ctl = control
        self._driver = self._get_driver(control=control)
        threading.Thread(target=self._activate, args=(cfg,)).start()

    def close_sessions(self):
        for _c in ('driver.mode.dnn', 'driver.mode.dagger'):
            if _c in self._driver_cache:
                self._driver_cache[_c].deactivate()

    def get_control(self):
        return self._driver_ctl

    def get_navigation_settings(self):
        return self._driver.get_navigation_settings()

    def next_recorder(self):
        return self._driver.next_recorder()

    def create_blob(self):
        blob = Blob()
        self._driver.get_noop(blob)
        return blob

    def get_next_action(self, blob):
        return self._driver.get_next_action(blob)

    def quit(self):
        for driver in self._driver_cache.values():
            driver.deactivate()
            driver.quit()


def _ros_init():
    # Ros replaces the root logger - add a new handler after ros initialisation.
    rospy.init_node('pilot', disable_signals=False, anonymous=True, log_level=rospy.DEBUG)
    console_handler = logging.StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    console_handler.setLevel(logging.DEBUG)
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.DEBUG)
    rospy.on_shutdown(lambda: quit_event.set())


def main():
    parser = argparse.ArgumentParser(description='Pilot.')
    parser.add_argument('--clock', type=int, default=50, help='Clock frequency in hz.')
    args = parser.parse_args()

    driver = DriverManager()

    _ros_init()
    drive_queue = collections.deque(maxlen=1)
    rospy.Subscriber('aav/teleop/input/drive', RosString, lambda x: drive_queue.appendleft(json.loads(x.data)))
    rospy.Subscriber('aav/teleop/input/control', RosString, lambda x: driver.on_command(json.loads(x.data)))
    state_topic = rospy.Publisher('aav/pilot/state/blob', RosString, queue_size=1)
    output_topic = rospy.Publisher('aav/pilot/command/drive', RosString, queue_size=1)

    # Determine the process frequency.
    _process_frequency = args.clock
    logger.info("Processing at {} Hz.".format(_process_frequency))
    max_duration = 1. / _process_frequency
    _num_violations = 0

    while not quit_event.is_set():
        try:
            proc_start = time.time()
            # Run the main step.
            blob = driver.create_blob()
            drive = drive_queue[0] if bool(drive_queue) else None
            # Say 250 ms is the connectivity minimum.
            if drive is not None and time.time() - drive.get('time') < .25:
                blob.driver = driver.get_control()
                blob.steering = drive.get('steering')
                blob.throttle = drive.get('throttle')
                driver.get_next_action(blob)
            # Otherwise steering and throttle are set to zero - per the noop command that created the blob.
            output_topic.publish(json.dumps(dict(steering=blob.steering, throttle=blob.throttle)))
            state_topic.publish(json.dumps(blob))
            # Synchronize per clock rate.
            _proc_sleep = max_duration - (time.time() - proc_start)
            _num_violations = max(0, _num_violations + (1 if _proc_sleep < 0 else -1))
            # Once is a fluke from three it's a pattern.
            if _num_violations > 2:
                logger.warning("Cannot maintain {} Hz frequency.".format(_process_frequency))
            time.sleep(max(0, _proc_sleep))
        except IOError as e:
            logger.warning(e)
            time.sleep(2 * max_duration)
        except KeyboardInterrupt:
            quit_event.set()

    logger.info("Waiting on handler to quit.")
    driver.quit()


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.DEBUG)
    main()
