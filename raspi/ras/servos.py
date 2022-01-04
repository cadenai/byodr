from __future__ import absolute_import

import argparse
import collections
import logging
import os
from abc import ABC, abstractmethod
from configparser import ConfigParser as SafeConfigParser

import numpy as np
from gpiozero import AngularServo

from byodr.utils import timestamp, Application
from byodr.utils.ipc import JSONPublisher, JSONServerThread
from byodr.utils.option import parse_option
from byodr.utils.protocol import MessageStreamProtocol
from byodr.utils.usbrelay import SearchUsbRelayFactory, StaticRelayHolder
from .core import CommandHistory, HallOdometer, VESCDrive

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(asctime)s %(filename)s %(funcName)s %(message)s'


class AbstractDriver(ABC):
    def __init__(self, relay):
        self._relay = relay

    @staticmethod
    def configuration_check(config):
        version_ok = config is not None and config.get('app_version', -1) == 2
        if config is not None and not version_ok:
            logger.warning("Received incompatible application version - configuration aborted.")
        return version_ok

    @abstractmethod
    def has_sensors(self):
        raise NotImplementedError()

    @abstractmethod
    def relay_ok(self):
        raise NotImplementedError()

    @abstractmethod
    def relay_violated(self, on_integrity=True):
        raise NotImplementedError()

    @abstractmethod
    def set_configuration(self, config):
        raise NotImplementedError()

    @abstractmethod
    def is_configured(self):
        raise NotImplementedError()

    @abstractmethod
    def velocity(self):
        raise NotImplementedError()

    @abstractmethod
    def drive(self, steering, throttle):
        raise NotImplementedError()

    @abstractmethod
    def quit(self):
        raise NotImplementedError()


class NoopDriver(AbstractDriver):
    def __init__(self, relay):
        super().__init__(relay)
        self._relay.close()

    def has_sensors(self):
        return False

    def relay_ok(self):
        pass

    def relay_violated(self, on_integrity=True):
        pass

    def set_configuration(self, config):
        pass

    def is_configured(self):
        return True

    def velocity(self):
        return 0

    def drive(self, steering, throttle):
        return 0

    def quit(self):
        pass


class AbstractSteerServoDriver(AbstractDriver, ABC):
    def __init__(self, relay, **kwargs):
        super().__init__(relay)
        self._steer_servo_config = dict(pin=parse_option('servo.steering.pin.nr', int, 13, **kwargs),
                                        min_pw=parse_option('servo.steering.min_pulse_width.ms', float, 0.5, **kwargs),
                                        max_pw=parse_option('servo.steering.max_pulse_width.ms', float, 2.5, **kwargs),
                                        frame=parse_option('servo.steering.frame_width.ms', float, 20.0, **kwargs))
        self._steering_config = dict(scale=parse_option('steering.domain.scale', float, 1.0, **kwargs))
        self._steer_servo = None

    @staticmethod
    def _angular_servo(message):
        fields = ('pin', 'min_pw', 'max_pw', 'frame')
        m_config = [message.get(f) for f in fields]
        pin, min_pw, max_pw, frame = [m_config[0]] + [1e-3 * x for x in m_config[1:]]
        return AngularServo(pin=pin, min_pulse_width=min_pw, max_pulse_width=max_pw, frame_width=frame)

    def _create_servo(self, servo, name, message):
        logger.info("Creating servo {} with config {}".format(name, message))
        if servo is not None:
            servo.close()
        servo = self._angular_servo(message=message)
        return servo

    def _apply_steering(self, steering):
        if self._steer_servo is not None:
            config = self._steering_config
            scale = config.get('scale')
            self._steer_servo.angle = scale * 90. * min(1, max(-1, steering))

    def set_configuration(self, config):
        # Translate the values into our domain.
        _steer_servo_config = self._steer_servo_config
        _steer_servo_config['min_pw'] = 0.5 + .5 * max(-2, min(2, config.get('steering_offset')))
        self._steer_servo = self._create_servo(self._steer_servo, 'steering', _steer_servo_config)

    def is_configured(self):
        return self._steer_servo is not None

    def quit(self):
        if self._steer_servo is not None:
            self._steer_servo.close()


class GPIODriver(AbstractSteerServoDriver):
    def __init__(self, relay, **kwargs):
        super().__init__(relay)
        # Our relay is expected to be wired on the motor power line.
        self._relay.open()
        self._motor_servo_config = dict(pin=parse_option('servo.motor.pin.nr', int, 12, **kwargs),
                                        min_pw=parse_option('servo.motor.min_pulse_width.ms', float, 0.5, **kwargs),
                                        max_pw=parse_option('servo.motor.max_pulse_width.ms', float, 2.5, **kwargs),
                                        frame=parse_option('servo.motor.frame_width.ms', float, 20.0, **kwargs))
        self._throttle_config = dict(reverse=parse_option('throttle.reverse.gear', int, 0.0, **kwargs),
                                     forward_shift=parse_option('throttle.domain.forward.shift', float, 0.0, **kwargs),
                                     backward_shift=parse_option('throttle.domain.backward.shift', float, 0.0, **kwargs),
                                     scale=parse_option('throttle.domain.scale', float, 2.0, **kwargs))
        self._motor_servo = None

    def has_sensors(self):
        return False

    def relay_ok(self):
        self._relay.close()

    def relay_violated(self, on_integrity=True):
        self._relay.open()

    def set_configuration(self, config):
        if self.configuration_check(config):
            logger.info("Received configuration {}.".format(config))
            super().set_configuration(config)
            self._throttle_config['scale'] = max(0, config.get('motor_scale'))
            self._motor_servo = self._create_servo(self._motor_servo, 'motor', self._motor_servo_config)

    def is_configured(self):
        return super().is_configured() and self._motor_servo is not None

    def velocity(self):
        raise NotImplementedError()

    def drive(self, steering, throttle):
        self._apply_steering(steering)
        _motor_effort = 0
        if self._motor_servo is not None:
            config = self._throttle_config
            _motor_effort = config.get('scale') * throttle
            _motor_shift = config.get('forward_shift') if throttle > 0 else config.get('backward_shift')
            _motor_angle = min(90, max(-90, _motor_shift + _motor_effort))
            _reverse_boost = config.get('reverse')
            if throttle < -.990 and _reverse_boost < _motor_angle:
                _motor_effort = _reverse_boost / 90.
                _motor_angle = _reverse_boost
            self._motor_servo.angle = _motor_angle
        return _motor_effort

    def quit(self):
        self._relay.open()
        super().quit()
        if self._motor_servo is not None:
            self._motor_servo.close()


class SingularVescDriver(AbstractSteerServoDriver):
    def __init__(self, relay, **kwargs):
        super().__init__(relay)
        self._relay.close()
        self._drive = VESCDrive(serial_port=parse_option('drive.serial.port', str, '/dev/ttyACM0', **kwargs),
                                cm_per_pole_pair=parse_option('drive.distance.cm_per_pole_pair', float, 1, **kwargs))
        self._throttle_config = dict(scale=parse_option('throttle.domain.scale', float, 2.0, **kwargs))

    def has_sensors(self):
        return self.is_configured()

    def relay_ok(self):
        self._relay.close()

    def relay_violated(self, on_integrity=True):
        if on_integrity:
            self.drive(0, 0)

    def set_configuration(self, config):
        if self.configuration_check(config):
            logger.info("Received configuration {}.".format(config))
            super().set_configuration(config)
            self._throttle_config['scale'] = max(0, config.get('motor_scale'))

    def is_configured(self):
        return super().is_configured() and self._drive.is_open()

    def velocity(self):
        try:
            return self._drive.get_velocity()
        except Exception as e:
            logger.warning(e)
            return 0

    def drive(self, steering, throttle):
        self._apply_steering(steering)
        _motor_effort = self._throttle_config.get('scale') * throttle
        self._drive.set_effort(_motor_effort)
        return _motor_effort

    def quit(self):
        self._relay.open()
        super().quit()
        self._drive.close()


class DualVescDriver(AbstractDriver):
    def __init__(self, relay, **kwargs):
        super().__init__(relay)
        self._relay.close()
        _pp_cm = parse_option('drive.distance.cm_per_pole_pair', float, 2.3, **kwargs)
        self._drive1 = VESCDrive(serial_port=parse_option('drive.0.serial.port', str, '/dev/ttyACM0', **kwargs), cm_per_pole_pair=_pp_cm)
        self._drive2 = VESCDrive(serial_port=parse_option('drive.1.serial.port', str, '/dev/ttyACM1', **kwargs), cm_per_pole_pair=_pp_cm)
        self._steering_offset = 0
        self._steering_effect = max(0., float(kwargs.get('drive.steering.effect', 1.8)))
        self._throttle_config = dict(scale=parse_option('throttle.domain.scale', float, 2.0, **kwargs))
        self._axes_ordered = kwargs.get('drive.axes.mount.order', 'normal') == 'normal'
        self._axis0_multiplier = 1 if kwargs.get('drive.axis0.mount.direction', 'forward') == 'forward' else -1
        self._axis1_multiplier = 1 if kwargs.get('drive.axis1.mount.direction', 'forward') == 'forward' else -1

    def has_sensors(self):
        return self.is_configured()

    def relay_ok(self):
        self._relay.close()

    def relay_violated(self, on_integrity=True):
        if on_integrity:
            self.drive(0, 0)

    def set_configuration(self, config):
        if self.configuration_check(config):
            logger.info("Received configuration {}.".format(config))
            self._steering_offset = max(-1., min(1., config.get('steering_offset')))
            self._throttle_config['scale'] = max(0, config.get('motor_scale'))

    def is_configured(self):
        return self._drive1.is_open() and self._drive2.is_open()

    def velocity(self):
        try:
            return (self._drive1.get_velocity() + self._drive2.get_velocity()) / 2.
        except Exception as e:
            logger.warning(e)
            return 0

    def drive(self, steering, throttle):
        _motor_scale = self._throttle_config.get('scale')
        # Scale down throttle for one wheel, the other retains its value.
        steering = min(1., max(-1., steering + self._steering_offset))
        throttle = min(1., max(-1., throttle))
        effect = 1 - min(1., abs(steering) * self._steering_effect)
        left = throttle if steering >= 0 else throttle * effect
        right = throttle if steering < 0 else throttle * effect
        a = (right if self._axes_ordered else left) * self._axis0_multiplier * _motor_scale
        b = (left if self._axes_ordered else right) * self._axis1_multiplier * _motor_scale
        self._drive1.set_effort(a)
        self._drive2.set_effort(b)
        return np.mean([a, b])

    def quit(self):
        self._relay.open()
        self._drive1.close()
        self._drive2.close()


class MainApplication(Application):
    def __init__(self, relay, hz=50, **kwargs):
        super(MainApplication, self).__init__(run_hz=hz)
        self._integrity = MessageStreamProtocol(max_age_ms=100, max_delay_ms=100)
        self._cmd_history = CommandHistory(hz=hz)
        self._config_queue = collections.deque(maxlen=1)
        self._drive_queue = collections.deque(maxlen=1)
        self._odometer = HallOdometer(**kwargs)
        self._chassis = None
        self.platform = None
        self.publisher = None
        # Setup the chassis.
        _drive_type = parse_option('drive.type', str, **kwargs)
        if _drive_type in ('gpio', 'gpio_with_hall'):
            self._chassis = GPIODriver(relay, **kwargs)
        elif _drive_type == 'vesc_single':
            self._chassis = SingularVescDriver(relay, **kwargs)
        elif _drive_type == 'vesc_dual':
            self._chassis = DualVescDriver(relay, **kwargs)
        else:
            raise AssertionError("Unknown drive type '{}'.".format(_drive_type))

    def _pop_config(self):
        return self._config_queue.popleft() if bool(self._config_queue) else None

    def _pop_drive(self):
        return self._drive_queue.popleft() if bool(self._drive_queue) else None

    def _on_message(self, message):
        self._integrity.on_message(message.get('time'))
        if message.get('method') == 'ras/driver/config':
            self._config_queue.appendleft(message.get('data'))
        else:
            self._drive_queue.appendleft(message.get('data'))

    def setup(self):
        self.platform.add_listener(self._on_message)
        self._integrity.reset()
        self._cmd_history.reset()
        self._odometer.setup()

    def finish(self):
        self._chassis.quit()
        self._odometer.quit()

    def step(self):
        n_violations = self._integrity.check()
        if n_violations > 5:
            self._chassis.relay_violated(on_integrity=True)
            self._integrity.reset()
            return

        c_config, c_drive = self._pop_config(), self._pop_drive()
        self._chassis.set_configuration(c_config)

        v_steering = 0 if c_drive is None else c_drive.get('steering', 0)
        v_throttle = 0 if c_drive is None else c_drive.get('throttle', 0)
        v_wakeup = False if c_drive is None else bool(c_drive.get('wakeup'))

        self._cmd_history.touch(steering=v_steering, throttle=v_throttle, wakeup=v_wakeup)
        if self._cmd_history.is_missing():
            self._chassis.relay_violated(on_integrity=False)
        elif n_violations < -5:
            self._chassis.relay_ok()

        # Immediately zero out throttle when violations start occurring.
        v_throttle = 0 if n_violations > 0 else v_throttle
        _effort = self._chassis.drive(v_steering, v_throttle)
        _data = dict(time=timestamp(), configured=int(self._chassis.is_configured()), motor_effort=_effort)
        if self._chassis.has_sensors():
            _data.update(dict(velocity=self._chassis.velocity()))
        elif self._odometer.is_enabled():
            _data.update(dict(velocity=self._odometer.velocity()))

        # Let the communication partner know we are operational.
        self.publisher.publish(data=_data)


def main():
    parser = argparse.ArgumentParser(description='Steering and throttle driver.')
    parser.add_argument('--config', type=str, default='/config/driver.ini', help='Configuration file.')
    args = parser.parse_args()

    config_file = args.config
    assert os.path.exists(config_file) and os.path.isfile(config_file)

    parser = SafeConfigParser()
    parser.read(config_file)
    kwargs = dict(parser.items('driver')) if parser.has_section('driver') else {}
    kwargs.update(dict(parser.items('odometer')) if parser.has_section('odometer') else {})

    _relay = SearchUsbRelayFactory().get_relay()
    assert _relay.is_attached(), "The relay device is not attached."

    holder = StaticRelayHolder(relay=_relay, channels=(0, 1))
    try:
        application = MainApplication(relay=holder, hz=50, **kwargs)
        quit_event = application.quit_event

        application.publisher = JSONPublisher(url='tcp://0.0.0.0:5555', topic='ras/drive/status')
        application.platform = JSONServerThread(url='tcp://0.0.0.0:5550', event=quit_event, receive_timeout_ms=50)

        threads = [application.platform]
        if quit_event.is_set():
            return 0

        [t.start() for t in threads]
        application.run()

        logger.info("Waiting on threads to stop.")
        [t.join() for t in threads]
    finally:
        holder.open()


if __name__ == "__main__":
    logging.basicConfig(format=log_format, datefmt='%Y%m%d:%H:%M:%S %p %Z')
    logging.getLogger().setLevel(logging.INFO)
    main()
