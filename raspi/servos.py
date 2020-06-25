import logging
import multiprocessing
import signal
import time

from gpiozero import AngularServo

from byodr.utils import timestamp
from byodr.utils.ipc import ReceiverThread, JSONPublisher, JSONServerThread

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class LocalIPCServer(JSONServerThread):
    """
    Receive the rover hostname/ip.
    """

    def __init__(self, url, event, fn_callback, receive_timeout_ms=50):
        super(LocalIPCServer, self).__init__(url, event, receive_timeout_ms)
        self._callback = fn_callback

    def serve(self, message):
        try:
            self._callback(message.get('master'))
        except Exception as e:
            logger.warn(e)
        return {}


class Receiver(object):
    def __init__(self):
        self._config = None
        self._drive = None

    def start(self, master_uri):
        logger.info("Connecting to uri '{}'.".format(master_uri))
        self._config = ReceiverThread(url=master_uri, topic=b'ras/servo/config', event=quit_event)
        self._drive = ReceiverThread(url=master_uri, topic=b'ras/servo/drive', event=quit_event)
        self._config.start()
        self._drive.start()

    def is_started(self):
        return self._config is not None and self._drive is not None

    def pop_config(self):
        return self._config.pop_latest() if self.is_started() else None

    def pop_drive(self):
        return self._drive.pop_latest() if self.is_started() else None

    def join(self):
        if self.is_started():
            self._config.join()
            self._drive.join()


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
    _reverse, _shift, _scale = config.get('reverse'), config.get('shift'), config.get('scale')
    if throttle < -.95 and in_reverse:
        _angle = _reverse
    else:
        _angle = min(90, max(-90, _shift + _scale * throttle))
    servo.angle = _angle


def main():
    receiver = Receiver()

    def _on_host(_uri):
        if not receiver.is_started():
            receiver.start(_uri)

    ipc_server = LocalIPCServer(url='tcp://0.0.0.0:5550', event=quit_event, fn_callback=_on_host)
    p_status = JSONPublisher(url='tcp://0.0.0.0:5555', topic='ras/drive/status')

    threads = [ipc_server, p_status]
    [t.start() for t in threads]

    steer_servo, motor_servo, throttle_config = None, None, dict(reverse=0, shift=0, scale=0)
    try:
        rate = 0.04  # 25 Hz.
        while not quit_event.is_set():
            command = receiver.pop_config()
            if command is not None:
                steer_servo = _create_servo(steer_servo, command.get('steering'))
                motor_servo = _create_servo(motor_servo, command.get('motor'))
                throttle_config = command.get('throttle')
            command = receiver.pop_drive()
            if steer_servo is not None:
                _steer(steer_servo, 0 if command is None else command.get('steering', 0))
            if motor_servo is not None:
                throttle = 0 if command is None else command.get('throttle', 0)
                in_reverse = False if command is None else bool(command.get('reverse'))
                _throttle(throttle_config, motor_servo, throttle, in_reverse)
            _configured = None not in (steer_servo, motor_servo)
            p_status.publish(data=dict(time=timestamp(), connected=int(receiver.is_started()), configured=int(_configured)))
            time.sleep(rate)
    except Exception as e:
        logger.error(e)
        raise e
    finally:
        if steer_servo is not None:
            steer_servo.close()
        if motor_servo is not None:
            motor_servo.close()

    logger.info("Waiting on receiver to stop.")
    receiver.join()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
