import argparse
import glob
import logging
import os
import shutil
from ConfigParser import SafeConfigParser

from byodr.utils import Application
from byodr.utils import Configurable
from byodr.utils.ipc import JSONPublisher, ImagePublisher, LocalIPCServer, JSONReceiver, CollectorThread
from byodr.utils.option import parse_option
from vehicle import CarlaHandler

logger = logging.getLogger(__name__)


class CarlaRunner(Configurable):
    def __init__(self, image_publisher):
        super(CarlaRunner, self).__init__()
        self._process_frequency = 10
        self._patience_micro = 1000.
        self._vehicle = CarlaHandler((lambda img, camera: image_publisher.publish(img, camera=camera)))

    def internal_quit(self, restarting=False):
        if not restarting:
            self._vehicle.quit()

    def internal_start(self, **kwargs):
        self._vehicle.restart(**kwargs)
        _errors = []
        self._process_frequency = parse_option('clock.hz', int, 10, _errors, **kwargs)
        self._patience_micro = parse_option('patience.ms', int, 200, _errors, **kwargs) * 1000.
        return _errors + self._vehicle.get_errors()

    def reset_agent(self):
        self._vehicle.reset()

    def get_process_frequency(self):
        return self._process_frequency

    def get_patience_micro(self):
        return self._patience_micro

    def state(self):
        return self._vehicle.state()

    def noop(self):
        self._vehicle.noop()

    def drive(self, cmd):
        self._vehicle.drive(cmd)


class CarlaApplication(Application):
    def __init__(self, image_publisher, config_dir=os.getcwd()):
        super(CarlaApplication, self).__init__()
        self._runner = CarlaRunner(image_publisher)
        self._config_dir = config_dir
        self.publisher = None
        self.ipc_chatter = None
        self.pilot = None
        self.teleop = None
        self.ipc_server = None

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
        cfg.update(dict(parser.items('platform')))
        return cfg

    def setup(self):
        if self.active():
            self._check_user_file()
            _restarted = self._runner.restart(**self._config())
            if _restarted:
                self.ipc_server.register_start(self._runner.get_errors())
                _process_frequency = self._runner.get_process_frequency()
                _patience_micro = self._runner.get_patience_micro()
                self.set_hz(_process_frequency)
                self.logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))

    def finish(self):
        self._runner.quit()

    def step(self):
        runner, pilot, teleop, ipc_chatter, ipc_server = self._runner, self.pilot, self.teleop, self.ipc_chatter, self.ipc_server
        c_pilot = self._latest_or_none(pilot, patience=(runner.get_patience_micro()))
        c_teleop = self._latest_or_none(teleop, patience=(runner.get_patience_micro()))
        if c_teleop is not None and c_teleop.get('button_a', 0):
            runner.reset_agent()
        if c_pilot is not None:
            runner.drive(c_pilot)
        else:
            runner.noop()
        self.publisher.publish(runner.state())
        chat = self.ipc_chatter()
        if chat and chat.get('command') == 'restart':
            self.setup()


class RoutingImagePublisher(object):
    def __init__(self, front_camera, rear_camera):
        self.front_camera = front_camera
        self.rear_camera = rear_camera

    def publish(self, _img, camera=0):
        if camera == 0:
            self.front_camera.publish(_img)
        else:
            self.rear_camera.publish(_img)


def main():
    parser = argparse.ArgumentParser(description='Carla vehicle client.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    front_camera = ImagePublisher(url='ipc:///byodr/camera_0.sock', topic='aav/camera/0')
    rear_camera = ImagePublisher(url='ipc:///byodr/camera_1.sock', topic='aav/camera/1')

    application = CarlaApplication(image_publisher=(RoutingImagePublisher(front_camera, rear_camera)), config_dir=args.config)
    quit_event = application.quit_event

    pilot = JSONReceiver(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output')
    teleop = JSONReceiver(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input')
    ipc_chatter = JSONReceiver(url='ipc:///byodr/teleop_c.sock', topic=b'aav/teleop/chatter', pop=True)
    collector = CollectorThread(receivers=(pilot, teleop, ipc_chatter), event=quit_event)

    application.publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
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
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
