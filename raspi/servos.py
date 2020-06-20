import logging
import multiprocessing
import os
import signal
import time

from gpiozero import AngularServo

from byodr.utils.ipc import ReceiverThread

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def _angular_servo(message):
    fields = ('pin', 'min_pw', 'max_pw', 'frame')
    m_config = [message.get(f) for f in fields]
    pin, min_pw, max_pw, frame = [m_config[0]] + [1e-3 * x for x in m_config[1:]]
    return AngularServo(pin=pin, min_pulse_width=min_pw, max_pulse_width=max_pw, frame_width=frame)


def _create_servo(servo, message):
    if message is not None:
        if servo is not None:
            servo.close()
        logger.info("Creating servo with config {}".format(message))
        servo = _angular_servo(message=message)
    return servo


def _steer(servo, value):
    if servo is not None:
        servo.angle = 90. * min(1, max(-1, value))


def _throttle(servo, value):
    if servo is not None:
        servo.angle = 90. * min(1, max(-1, value))


def main():
    servo_master_uri = os.environ['DRIVE_MASTER_URI']
    threads = []
    r_config = ReceiverThread(url=servo_master_uri, topic=b'ras/servo/config', event=quit_event)
    r_drive = ReceiverThread(url=servo_master_uri, topic=b'ras/servo/drive', event=quit_event)
    threads.append(r_config)
    threads.append(r_drive)
    [t.start() for t in threads]

    steer_servo, motor_servo = None, None
    rate = 1000 / 25 * 1e-3
    while not quit_event.is_set():
        command = r_config.pop_latest()
        steer_servo = _create_servo(steer_servo, None if command is None else command.get('steering'))
        motor_servo = _create_servo(motor_servo, None if command is None else command.get('motor'))
        command = r_drive.pop_latest()
        _steer(steer_servo, 0 if command is None else command.get('steering', 0))
        _throttle(motor_servo, 0 if command is None else command.get('throttle', 0))
        time.sleep(rate)

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
