import argparse
import glob
import logging
import os
import shutil
from ConfigParser import SafeConfigParser

from byodr.utils import Application
from byodr.utils import timestamp, Configurable
from byodr.utils.ipc import JSONPublisher, ImagePublisher, LocalIPCServer, CollectorThread, JSONReceiver
from byodr.utils.option import parse_option, hash_dict
from core import GpsPollerThread, GstSource, PTZCamera

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'


class Platform(Configurable):
    def __init__(self):
        super(Platform, self).__init__()
        self._gps_poller = GpsPollerThread()
        self._velocity = 0

    def state(self, c_teleop):
        # The platform currently does not contain a speedometer.
        # Use throttle as the proxy.
        y_vel = self._velocity
        # The teleop command can be none sometimes on slow connections.
        if c_teleop:
            y_vel = c_teleop.get('throttle', 0)
            self._velocity = y_vel
        return dict(x_coordinate=self._gps_poller.get_latitude(),
                    y_coordinate=self._gps_poller.get_longitude(),
                    heading=0,  # Tbd - get this from gps.
                    velocity=y_vel,
                    time=timestamp())

    def internal_quit(self, restarting=False):
        if not restarting:
            self._gps_poller.quit()

    def internal_start(self, **kwargs):
        if not self._gps_poller.is_alive():
            self._gps_poller.start()
        return []


class RoverHandler(Configurable):
    def __init__(self):
        super(RoverHandler, self).__init__()
        self._vehicle = Platform()
        self._process_frequency = 10
        self._patience_micro = 100.
        self._gst_sources = []
        self._ptz_cameras = []

    def get_process_frequency(self):
        return self._process_frequency

    def get_patience_micro(self):
        return self._patience_micro

    def is_rear_camera_enabled(self):
        return self._gst_sources and self._gst_sources[-1].is_enabled()

    def is_reconfigured(self, **kwargs):
        return True

    def internal_quit(self, restarting=False):
        if not restarting:
            self._vehicle.quit()
            map(lambda x: x.quit(), self._ptz_cameras)
            map(lambda x: x.quit(), self._gst_sources)

    def internal_start(self, **kwargs):
        errors = []
        self._process_frequency = parse_option('clock.hz', int, 10, errors, **kwargs)
        self._patience_micro = parse_option('patience.ms', int, 200, errors, **kwargs) * 1000.
        self._vehicle.restart(**kwargs)
        errors.extend(self._vehicle.get_errors())
        if not self._gst_sources:
            front_camera = ImagePublisher(url='ipc:///byodr/camera_0.sock', topic='aav/camera/0')
            rear_camera = ImagePublisher(url='ipc:///byodr/camera_1.sock', topic='aav/camera/1')
            self._gst_sources.append(GstSource(position='front', image_publisher=front_camera))
            self._gst_sources.append(GstSource(position='rear', image_publisher=rear_camera))
        if not self._ptz_cameras:
            self._ptz_cameras.append(PTZCamera(position='front'))
            self._ptz_cameras.append(PTZCamera(position='rear'))
        for item in self._gst_sources + self._ptz_cameras:
            item.restart(**kwargs)
            errors.extend(item.get_errors())
        return errors

    def _cycle_ptz_cameras(self, c_pilot, c_teleop):
        # The front camera ptz function is enabled for teleop direct driving only.
        # Set the front camera to the home position anytime the autopilot is switched on.
        if self._ptz_cameras and c_teleop is not None:
            c_camera = c_teleop.get('camera_id', -1)
            button_north_pressed = bool(c_teleop.get('button_y', 0))
            if button_north_pressed:
                self._ptz_cameras[0].add({'goto_home': 1})
            elif c_camera in (0, 1) and (c_camera == 1 or (c_pilot is not None and c_pilot.get('driver') == 'driver_mode.teleop.direct')):
                button_south_pressed = bool(c_teleop.get('button_a', 0))
                button_west_pressed = bool(c_teleop.get('button_x', 0))
                command = {'pan': c_teleop.get('pan', 0),
                           'tilt': c_teleop.get('tilt', 0),
                           'set_home': 1 if button_west_pressed else 0,
                           'goto_home': 1 if button_south_pressed else 0
                           }
                self._ptz_cameras[c_camera].add(command)

    def cycle(self, c_pilot, c_teleop):
        self._cycle_ptz_cameras(c_pilot, c_teleop)
        map(lambda x: x.check(), self._gst_sources)
        return self._vehicle.state(c_teleop)


class RoverApplication(Application):
    def __init__(self, handler=None, config_dir=os.getcwd()):
        super(RoverApplication, self).__init__()
        self._config_dir = config_dir
        self._handler = RoverHandler() if handler is None else handler
        self._config_hash = -1
        self.state_publisher = None
        self.ipc_server = None
        self.pilot = None
        self.teleop = None
        self.ipc_chatter = None

    def _check_user_file(self):
        # One user configuration file is optional and can be used to persist settings.
        _candidates = glob.glob(os.path.join(self._config_dir, '*.ini'))
        if len(_candidates) == 0:
            shutil.copyfile('user.template.ini', os.path.join(self._config_dir, 'config.ini'))
            logger.info("Create a new user configuration file from template.")

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        cfg = dict(parser.items('vehicle'))
        cfg.update(dict(parser.items('camera')))
        return cfg

    def _capabilities(self):
        return {
            'rear_camera_enabled': 1 if self._handler.is_rear_camera_enabled() else 0
        }

    def setup(self):
        if self.active():
            _config = self._config()
            _hash = hash_dict(**_config)
            if _hash != self._config_hash:
                self._config_hash = _hash
                self._check_user_file()
                _restarted = self._handler.restart(**_config)
                if _restarted:
                    self.ipc_server.register_start(self._handler.get_errors(), self._capabilities())
                    _frequency = self._handler.get_process_frequency()
                    self.set_hz(_frequency)
                    self.logger.info("Processing at {} Hz.".format(_frequency))

    def finish(self):
        self._handler.quit()

    def step(self):
        rover, pilot, teleop, publisher = self._handler, self.pilot, self.teleop, self.state_publisher
        c_pilot = self._latest_or_none(pilot, patience=rover.get_patience_micro())
        c_teleop = self._latest_or_none(teleop, patience=rover.get_patience_micro())
        _state = rover.cycle(c_pilot, c_teleop)
        publisher.publish(_state)
        chat = self.ipc_chatter()
        if chat and chat.get('command') == 'restart':
            self.setup()


def main():
    parser = argparse.ArgumentParser(description='Rover main.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    application = RoverApplication(config_dir=args.config)
    quit_event = application.quit_event

    pilot = JSONReceiver(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output')
    teleop = JSONReceiver(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input')
    ipc_chatter = JSONReceiver(url='ipc:///byodr/teleop_c.sock', topic=b'aav/teleop/chatter', pop=True)
    collector = CollectorThread(receivers=(pilot, teleop, ipc_chatter), event=quit_event)

    application.state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/vehicle_c.sock', name='platform', event=quit_event)
    application.pilot = lambda: collector.get(0)
    application.teleop = lambda: collector.get(1)
    application.ipc_chatter = lambda: collector.get(2)

    threads = [collector, application.ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    application.run()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
