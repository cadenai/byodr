import collections
import logging
import multiprocessing
import signal
import time

from gpiozero import AngularServo, DigitalOutputDevice

from byodr.utils import timestamp
from byodr.utils.ipc import JSONPublisher, JSONServerThread
from byodr.utils.protocol import MessageStreamProtocol

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class ReceivingServer(JSONServerThread):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(ReceivingServer, self).__init__(url, event, receive_timeout_ms=receive_timeout_ms)
        self._integrity = MessageStreamProtocol()
        self._config_queue = collections.deque(maxlen=1)
        self._drive_queue = collections.deque(maxlen=1)

    def pop_config(self):
        return self._config_queue.popleft() if bool(self._config_queue) else None

    def pop_drive(self):
        return self._drive_queue.popleft() if bool(self._drive_queue) else None

    def check_integrity_violations(self):
        return self._integrity.check()

    def reset_integrity(self):
        self._integrity.reset()

    def on_message(self, message):
        super(ReceivingServer, self).on_message(message)
        self._integrity.on_message(message.get('time'))
        if message.get('method') == 'ras/servo/config':
            self._config_queue.appendleft(message.get('data'))
        else:
            self._drive_queue.appendleft(message.get('data'))


class SingleChannelRelay(object):
    def __init__(self, pin):
        self._device = DigitalOutputDevice(pin=pin)

    def on(self):
        self._device.on()

    def off(self):
        self._device.off()

    def toggle(self):
        self.off()
        self.on()


def _angular_servo(message):
    fields = ('pin', 'min_pw', 'max_pw', 'frame')
    m_config = [message.get(f) for f in fields]
    pin, min_pw, max_pw, frame = [m_config[0]] + [1e-3 * x for x in m_config[1:]]
    return AngularServo(pin=pin, min_pulse_width=min_pw, max_pulse_width=max_pw, frame_width=frame)


def _create_servo(servo, message):
    if servo is not None:
        servo.close()
    servo = _angular_servo(message=message)
    logger.info("Created servo with config {}".format(message))
    return servo


def _steer(servo, value):
    servo.angle = 90. * min(1, max(-1, value))


def _throttle(config, servo, throttle, in_reverse):
    if throttle < -.95 and in_reverse:
        _angle = config.get('reverse')
    else:
        _angle = config.get('forward_shift') if throttle > 0 else config.get('backward_shift')
        _angle = min(90, max(-90, _angle + config.get('scale') * throttle))
    servo.angle = _angle


def main():
    p_status = JSONPublisher(url='tcp://0.0.0.0:5555', topic='ras/drive/status')
    e_server = ReceivingServer(url='tcp://0.0.0.0:5550', event=quit_event, receive_timeout_ms=50)

    threads = [e_server]
    [t.start() for t in threads]

    steer_servo, motor_servo, throttle_config = None, None, dict(reverse=0, forward_shift=0, backward_shift=0, scale=0)
    relay = SingleChannelRelay(pin=18)
    try:
        rate = 0.04  # 25 Hz.
        relay.on()
        while not quit_event.is_set():
            n_violations = e_server.check_integrity_violations()
            if n_violations > 2:
                relay.off()
                e_server.reset_integrity()
                logger.warning("Motor relay OFF")
            c_config, c_drive = e_server.pop_config(), e_server.pop_drive()
            if c_config is not None:
                steer_servo = _create_servo(steer_servo, c_config.get('steering'))
                motor_servo = _create_servo(motor_servo, c_config.get('motor'))
                throttle_config = c_config.get('throttle')
            if steer_servo is not None:
                _steer(steer_servo, 0 if c_drive is None else c_drive.get('steering', 0))
            if motor_servo is not None:
                _zero_throttle = c_drive is None or n_violations > 2
                throttle = 0 if _zero_throttle else c_drive.get('throttle', 0)
                in_reverse = False if c_drive is None else bool(c_drive.get('reverse'))
                _throttle(throttle_config, motor_servo, throttle, in_reverse)
            _configured = None not in (steer_servo, motor_servo)
            p_status.publish(data=dict(time=timestamp(), configured=int(_configured)))
            time.sleep(rate)
    except Exception as e:
        logger.error(e)
        raise e
    finally:
        relay.off()
        if steer_servo is not None:
            steer_servo.close()
        if motor_servo is not None:
            motor_servo.close()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
