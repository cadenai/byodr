import collections
import logging

from gpiozero import AngularServo

from byodr.utils import timestamp, Application
from byodr.utils.ipc import JSONPublisher, JSONServerThread
from byodr.utils.protocol import MessageStreamProtocol
from byodr.utils.usbrelay import SearchUsbRelayFactory

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'


class Chassis(object):
    def __init__(self):
        self._steer_servo = None
        self._motor_servo = None
        self._throttle_config = dict(reverse=0, forward_shift=0, backward_shift=0, scale=0)

    @staticmethod
    def _angular_servo(message):
        fields = ('pin', 'min_pw', 'max_pw', 'frame')
        m_config = [message.get(f) for f in fields]
        pin, min_pw, max_pw, frame = [m_config[0]] + [1e-3 * x for x in m_config[1:]]
        return AngularServo(pin=pin, min_pulse_width=min_pw, max_pulse_width=max_pw, frame_width=frame)

    def _create_servo(self, servo, message):
        if servo is not None:
            servo.close()
        servo = self._angular_servo(message=message)
        logger.info("Created servo with config {}".format(message))
        return servo

    def set_configuration(self, config):
        if config is not None:
            self._steer_servo = self._create_servo(self._steer_servo, config.get('steering'))
            self._motor_servo = self._create_servo(self._motor_servo, config.get('motor'))
            self._throttle_config = config.get('throttle')

    def is_configured(self):
        return None not in (self._steer_servo, self._motor_servo)

    def apply_steering(self, value):
        if self._steer_servo is not None:
            self._steer_servo.angle = 90. * min(1, max(-1, value))

    def apply_throttle(self, throttle, in_reverse):
        if self._motor_servo is not None:
            config = self._throttle_config
            if throttle < -.95 and in_reverse:
                _angle = config.get('reverse')
            else:
                _angle = config.get('forward_shift') if throttle > 0 else config.get('backward_shift')
                _angle = min(90, max(-90, _angle + config.get('scale') * throttle))
            self._motor_servo.angle = _angle

    def quit(self):
        if self._steer_servo is not None:
            self._steer_servo.close()
        if self._motor_servo is not None:
            self._motor_servo.close()


class CommandHistory(object):
    def __init__(self, timeout_seconds=180, hz=25):
        self._threshold = timeout_seconds * hz
        self._num_missing = None
        self.reset()

    def touch(self, steering, throttle):
        no_steering = steering is None or abs(steering) < 1e-3
        no_throttle = throttle is None or abs(throttle) < 1e-3
        if no_steering and no_throttle:
            self._num_missing += 1
        else:
            self._num_missing = 0

    def reset(self):
        self._num_missing = self._threshold * 2

    def is_missing(self):
        return self._num_missing > self._threshold


class ChassisApplication(Application):
    def __init__(self, relay, chassis=None, hz=10):
        super(ChassisApplication, self).__init__(run_hz=hz)
        self._relay = relay
        self._chassis = Chassis() if chassis is None else chassis
        self._integrity = MessageStreamProtocol()
        self._cmd_history = CommandHistory(hz=hz)
        self._config_queue = collections.deque(maxlen=1)
        self._drive_queue = collections.deque(maxlen=1)
        self.platform = None
        self.publisher = None

    def _pop_config(self):
        return self._config_queue.popleft() if bool(self._config_queue) else None

    def _pop_drive(self):
        return self._drive_queue.popleft() if bool(self._drive_queue) else None

    def _on_message(self, message):
        self._integrity.on_message(message.get('time'))
        if message.get('method') == 'ras/servo/config':
            self._config_queue.appendleft(message.get('data'))
        else:
            self._drive_queue.appendleft(message.get('data'))

    def setup(self):
        self.platform.add_listener(self._on_message)
        self._integrity.reset()
        self._cmd_history.reset()

    def finish(self):
        self._relay.open()
        self._chassis.quit()

    def step(self):
        # At startup the relay is open untill non empty commands while the integrity requirements are met.
        # After a number of missing commands open the relay and close it again when commands resume.
        n_violations = self._integrity.check()
        if n_violations > 5:
            self._relay.open()
            self._integrity.reset()
            return

        c_config, c_drive = self._pop_config(), self._pop_drive()
        self._chassis.set_configuration(c_config)

        v_steering = 0 if c_drive is None else c_drive.get('steering', 0)
        v_throttle = 0 if c_drive is None else c_drive.get('throttle', 0)
        v_reverse = False if c_drive is None else bool(c_drive.get('reverse'))
        self._cmd_history.touch(steering=v_steering, throttle=v_throttle)
        if self._cmd_history.is_missing():
            self._relay.open()
        elif n_violations < -5:
            self._relay.close()

        self._chassis.apply_steering(v_steering)
        # Immediately zero out throttle when violations start occuring.
        self._chassis.apply_throttle(0 if n_violations > 0 else v_throttle, v_reverse)
        # Let the communication partner know we are operational.
        self.publisher.publish(data=dict(time=timestamp(), configured=int(self._chassis.is_configured())))


def main():
    relay = SearchUsbRelayFactory().get_relay()
    assert relay.is_attached(), "The device is not attached."

    try:
        application = ChassisApplication(relay=relay, hz=25)
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
        relay.open()


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
