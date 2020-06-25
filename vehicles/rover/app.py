import argparse
import glob
import logging
import multiprocessing
import os
import signal
import socket
import time
from ConfigParser import SafeConfigParser

from byodr.utils import timestamp
from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher, LocalIPCServer, JSONZmqClient
from byodr.utils.option import parse_option
from core import TwistHandler, PTZCamera, GstSource, ZMQGate

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


class RasPi(object):
    def __init__(self, config_dir):
        self._config_dir = config_dir
        self._event = multiprocessing.Event()
        self._pi_client = None
        self._master_uri = None
        self._status = None
        self._odometer = None

    def start(self):
        self.restart()

    def quit(self):
        if self._pi_client is not None:
            self._pi_client.quit()
        self._event.set()

    def join(self):
        self._status.join()
        self._odometer.join()

    def restart(self):
        parser = get_parser(self._config_dir)
        vehicle_cfg = dict(parser.items('vehicle'))
        _master_uri = parse_option('ras.master.uri', str, 'none', [], **vehicle_cfg)
        if self._master_uri != _master_uri:
            logger.info("Using pi master uri '{}'.".format(_master_uri))
            self.quit()
            self.join()
            self._pi_client = JSONZmqClient(urls='{}:5550'.format(_master_uri))
            self._status = ReceiverThread(url='{}:5555'.format(_master_uri), topic=b'ras/drive/status', event=self._event)
            self._odometer = ReceiverThread(url='{}:5560'.format(_master_uri), topic=b'ras/sensor/odometer', event=self._event)
            self._status.start()
            self._odometer.start()
            self._master_uri = _master_uri

    def update(self, status_msg):
        if not bool(status_msg.get('connected')):
            self._pi_client.call(dict(master='tcp://{}:5555'.format(socket.gethostname())))

    def pop_odometer(self):
        return self._odometer.pop_latest()

    def pop_status(self):
        return self._status.pop_latest()


class Rover(object):
    def __init__(self, config_dir, drive_publisher, image_publisher, ipc_server):
        self._config_dir = config_dir
        self._drive_publisher = drive_publisher
        self._image_publisher = image_publisher
        self._ipc_server = ipc_server
        self.ras_pi = RasPi(config_dir)
        self.vehicle = None
        self.camera = None
        self.gst_source = None

    def get_process_frequency(self):
        return self.vehicle.get_process_frequency()

    def get_patience_micro(self):
        return self.vehicle.get_patience_micro()

    def get_vehicle_state(self):
        return self.vehicle.state()

    def start(self):
        self.restart()

    def quit(self):
        self.ras_pi.quit()
        if self.vehicle is not None:
            self.vehicle.quit()
        if self.camera is not None:
            self.camera.quit()
        if self.gst_source is not None:
            self.gst_source.quit()

    def join(self):
        self.quit()

    def restart(self):
        self.ras_pi.restart()
        parser = get_parser(self._config_dir)
        vehicle_cfg = dict(parser.items('vehicle'))
        camera_cfg = dict(parser.items('camera'))
        previous = self.vehicle, self.camera, self.gst_source
        vehicle, ptz_camera, gst_source = previous
        _configured = any(map(lambda _x: _x is None, previous))
        if vehicle is None:
            gate = ZMQGate(self._drive_publisher)
            vehicle = TwistHandler(gate, **vehicle_cfg)
        elif vehicle.is_reconfigured(**vehicle_cfg):
            vehicle.restart(**vehicle_cfg)
            _configured = True
        if ptz_camera is None:
            ptz_camera = PTZCamera(**camera_cfg)
        elif ptz_camera.is_reconfigured(**camera_cfg):
            ptz_camera.restart(**camera_cfg)
            _configured = True
        if gst_source is None:
            gst_source = GstSource(self._image_publisher, **camera_cfg)
        elif gst_source.is_reconfigured(**camera_cfg):
            gst_source.restart(**camera_cfg)
            _configured = True
        if _configured:
            errors = []
            [errors.extend(x.get_errors()) for x in (vehicle, ptz_camera, gst_source)]
            self._ipc_server.register_start(errors)
            _process_frequency = vehicle.get_process_frequency()
            _patience_micro = vehicle.get_patience_micro()
            logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))
        self.vehicle, self.camera, self.gst_source = vehicle, ptz_camera, gst_source

    def update(self, c_pilot, c_teleop):
        self.vehicle.drive(c_pilot, c_teleop)
        self.camera.add(c_pilot, c_teleop)
        self.gst_source.check()
        pi_status = self.ras_pi.pop_status()
        if pi_status is not None:
            self.ras_pi.update(pi_status)
            if not bool(pi_status.get('configured')):
                self.vehicle.send_config()
        pi_odo = self.ras_pi.pop_odometer()
        if pi_odo is not None:
            self.vehicle.update_rps(pi_odo.get('rps'))


def main():
    parser = argparse.ArgumentParser(description='Rover main.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')
    drive_publisher = JSONPublisher(url='tcp://0.0.0.0:5555', topic='ras/servo/drive')

    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    ipc_server = IPCServer(url='ipc:///byodr/vehicle_c.sock', event=quit_event)

    rover = Rover(args.config, drive_publisher, image_publisher, ipc_server)

    threads = [pilot, teleop, ipc_chatter, ipc_server, rover]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    _period = 1. / rover.get_process_frequency()
    while not quit_event.is_set():
        state_publisher.publish(rover.get_vehicle_state())
        c_pilot = _latest_or_none(pilot, patience=rover.get_patience_micro())
        c_teleop = _latest_or_none(teleop, patience=rover.get_patience_micro())
        rover.update(c_pilot, c_teleop)
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
