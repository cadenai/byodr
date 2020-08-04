import argparse
import glob
import logging
import math
import multiprocessing
import os
import signal
import time
from ConfigParser import SafeConfigParser

from byodr.utils import timestamp, Configurable
from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher, LocalIPCServer, JSONZmqClient
from byodr.utils.option import parse_option
from byodr.utils.protocol import MessageStreamProtocol
from core import PTZCamera, GstSource, GpsPollerThread
from relay import SingleChannelUsbRelay

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def _latest_or_none(receiver, patience):
    candidate = receiver.get_latest()
    _time = 0 if candidate is None else candidate.get('time')
    _on_time = (timestamp() - _time) < patience
    return candidate if _on_time else None


class IPCServer(LocalIPCServer):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(IPCServer, self).__init__('platform', url, event, receive_timeout_ms)

    def serve_local(self, message):
        return {}


def get_parser(config_dir):
    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(config_dir, '*.ini'))]
    return parser


class PiProtocol(Configurable):
    def __init__(self):
        super(PiProtocol, self).__init__()
        self._event = multiprocessing.Event()
        self._integrity = MessageStreamProtocol()
        self._master_uri = None
        self._pi_client = None
        self._status = None
        self._odometer = None

    def pop_status(self):
        return None if self._status is None else self._status.pop_latest()

    def pop_odometer(self):
        return None if self._odometer is None else self._odometer.pop_latest()

    def send_config(self, data):
        if self._pi_client is not None and data is not None:
            self._pi_client.call(dict(time=timestamp(), method='ras/servo/config', data=data))

    def send_drive(self, throttle=0., steering=0., reverse_gear=False):
        if self._pi_client is not None:
            throttle = max(-1., min(1., throttle))
            steering = max(-1., min(1., steering))
            _reverse = 1 if reverse_gear else 0
            self._pi_client.call(dict(time=timestamp(),
                                      method='ras/servo/drive',
                                      data=dict(steering=steering, throttle=throttle, reverse=_reverse)))

    def is_reconfigured(self, **kwargs):
        return self._master_uri != kwargs.get('ras.master.uri')

    def internal_quit(self, restarting=False):
        self._event.set()
        self.reset_integrity()
        if self._pi_client is not None:
            self._pi_client.quit()

    def _status_message_received(self, msg):
        self._integrity.on_message(msg.get('time'))

    def check_integrity_violations(self):
        return self._integrity.check()

    def reset_integrity(self):
        self._integrity.reset()

    def internal_start(self, **kwargs):
        errors = []
        _master_uri = parse_option('ras.master.uri', str, 'none', errors, **kwargs)
        logger.info("Using pi master uri '{}'.".format(_master_uri))
        self.reset_integrity()
        self._event = multiprocessing.Event()
        self._pi_client = JSONZmqClient(urls='{}:5550'.format(_master_uri))
        self._status = ReceiverThread(url='{}:5555'.format(_master_uri),
                                      topic=b'ras/drive/status',
                                      event=self._event,
                                      on_message=self._status_message_received)
        self._odometer = ReceiverThread(url='{}:5560'.format(_master_uri), topic=b'ras/sensor/odometer', event=self._event)
        self._status.start()
        self._odometer.start()
        self._master_uri = _master_uri
        return errors


class Platform(Configurable):
    def __init__(self):
        super(Platform, self).__init__()
        self._protocol = PiProtocol()
        self._gps_poller = GpsPollerThread()
        self._servo_config = None
        self._rps = 0  # Hall sensor revolutions per second.
        self._circum_m = 1
        self._gear_ratio = 1

    def state(self):
        # Convert to travel speed in meters per second.
        velocity = (self._rps / self._gear_ratio) * self._circum_m
        return dict(x_coordinate=self._gps_poller.get_latitude(),
                    y_coordinate=self._gps_poller.get_longitude(),
                    heading=0,
                    velocity=velocity,
                    time=timestamp())

    def check_integrity_violations(self):
        return self._protocol.check_integrity_violations()

    def reset_integrity(self):
        self._protocol.reset_integrity()

    def drive(self, pilot, teleop):
        pi_status = self._protocol.pop_status()
        pi_odo = self._protocol.pop_odometer()
        if pi_status is not None and not bool(pi_status.get('configured')):
            self._protocol.send_config(self._servo_config)
        if pi_odo is not None:
            self._rps = pi_odo.get('rps')
        if pilot is None:
            self._protocol.send_drive()
        else:
            _reverse = teleop and teleop.get('arrow_down', 0)
            self._protocol.send_drive(steering=pilot.get('steering'), throttle=pilot.get('throttle'), reverse_gear=_reverse)

    def internal_quit(self, restarting=False):
        if not restarting:
            self._protocol.quit()
            self._gps_poller.quit()

    def internal_start(self, **kwargs):
        self._protocol.restart(**kwargs)
        if not self._gps_poller.is_alive():
            self._gps_poller.start()
        errors = []
        self._circum_m = 2 * math.pi * parse_option('chassis.wheel.radius.meter', float, 0, errors, **kwargs)
        self._gear_ratio = parse_option('chassis.hall.ticks.per.rotation', int, 0, errors, **kwargs)
        c_steer = dict(pin=parse_option('ras.servo.steering.pin.nr', int, 0, errors, **kwargs),
                       min_pw=parse_option('ras.servo.steering.min_pulse_width.ms', float, 0, errors, **kwargs),
                       max_pw=parse_option('ras.servo.steering.max_pulse_width.ms', float, 0, errors, **kwargs),
                       frame=parse_option('ras.servo.steering.frame_width.ms', float, 0, errors, **kwargs))
        c_motor = dict(pin=parse_option('ras.servo.motor.pin.nr', int, 0, errors, **kwargs),
                       min_pw=parse_option('ras.servo.motor.min_pulse_width.ms', float, 0, errors, **kwargs),
                       max_pw=parse_option('ras.servo.motor.max_pulse_width.ms', float, 0, errors, **kwargs),
                       frame=parse_option('ras.servo.motor.frame_width.ms', float, 0, errors, **kwargs))
        c_throttle = dict(reverse=parse_option('ras.throttle.reverse.gear', int, 0, errors, **kwargs),
                          forward_shift=parse_option('ras.throttle.domain.forward.shift', float, 0, errors, **kwargs),
                          backward_shift=parse_option('ras.throttle.domain.backward.shift', float, 0, errors, **kwargs),
                          scale=parse_option('ras.throttle.domain.scale', float, 0, errors, **kwargs))
        self._servo_config = dict(steering=c_steer, motor=c_motor, throttle=c_throttle)
        self._protocol.send_config(self._servo_config)
        return errors + self._protocol.get_errors()


class Rover(Configurable):

    def __init__(self, config_dir, image_publisher, ipc_server):
        super(Rover, self).__init__()
        self._config_dir = config_dir
        self._ipc_server = ipc_server
        self._process_frequency = 10
        self._patience_micro = 100.
        self._vehicle = Platform()
        self._camera = PTZCamera()
        self._gst_source = GstSource(image_publisher)
        self._relay = SingleChannelUsbRelay()
        self._relay.attach()
        self._relay.close()

    def get_process_frequency(self):
        return self._process_frequency

    def get_patience_micro(self):
        return self._patience_micro

    def is_reconfigured(self, **kwargs):
        return True

    def internal_quit(self, restarting=False):
        if not restarting:
            self._vehicle.quit()
            self._camera.quit()
            self._gst_source.quit()
            self._relay.open()

    def internal_start(self):
        errors = []
        parser = get_parser(self._config_dir)
        vehicle_cfg = dict(parser.items('vehicle'))
        camera_cfg = dict(parser.items('camera'))
        self._process_frequency = parse_option('clock.hz', int, 10, errors, **vehicle_cfg)
        self._patience_micro = parse_option('patience.ms', int, 200, errors, **vehicle_cfg) * 1000.
        _parties = [self._vehicle, self._camera, self._gst_source]
        _starts = sum([c.get_num_starts() for c in _parties])
        self._vehicle.restart(**vehicle_cfg)
        self._camera.restart(**camera_cfg)
        self._gst_source.restart(**camera_cfg)
        _restarted = sum([c.get_num_starts() for c in _parties]) > _starts
        if _restarted:
            [errors.extend(x.get_errors()) for x in _parties]
            self._ipc_server.register_start(errors)
            logger.info("Processing at {} Hz and a patience of {} ms.".format(self._process_frequency, self._patience_micro / 1000))
        return errors + self._vehicle.get_errors() + self._camera.get_errors() + self._gst_source.get_errors()

    def cycle(self, c_pilot, c_teleop):
        if self._vehicle.check_integrity_violations() > 2:
            self._relay.open()
            # A reset can be too fast for the dependant circuit to reboot.
            # self._relay.close()
            # self._vehicle.reset_integrity()
            logger.warn("Relay Open")
        else:
            self._vehicle.drive(c_pilot, c_teleop)
            self._camera.add(c_pilot, c_teleop)
            self._gst_source.check()
        return self._vehicle.state()


def main():
    parser = argparse.ArgumentParser(description='Rover main.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    ipc_server = IPCServer(url='ipc:///byodr/vehicle_c.sock', event=quit_event)
    rover = Rover(args.config, image_publisher, ipc_server)

    threads = [pilot, teleop, ipc_chatter, ipc_server, rover]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    _period = 1. / rover.get_process_frequency()
    while not quit_event.is_set():
        c_pilot = _latest_or_none(pilot, patience=rover.get_patience_micro())
        c_teleop = _latest_or_none(teleop, patience=rover.get_patience_micro())
        _state = rover.cycle(c_pilot, c_teleop)
        state_publisher.publish(_state)
        chat = ipc_chatter.pop_latest()
        if chat and chat.get('command') == 'restart':
            rover.restart()
            _period = 1. / rover.get_process_frequency()
        else:
            time.sleep(_period)

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
