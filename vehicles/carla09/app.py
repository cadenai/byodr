import argparse
import glob
import logging
import multiprocessing
import os
import signal
import time
import traceback
from ConfigParser import SafeConfigParser

from byodr.utils import timestamp, Configurable
from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher, LocalIPCServer
from byodr.utils.option import parse_option
from vehicle import CarlaHandler

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class IPCServer(LocalIPCServer):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(IPCServer, self).__init__('platform', url, event, receive_timeout_ms)

    def serve_local(self, message):
        return {}


def _latest_or_none(receiver, patience):
    candidate = receiver.get_latest()
    _time = 0 if candidate is None else candidate.get('time')
    _on_time = (timestamp() - _time) < patience
    return candidate if _on_time else None


class CarlaRunner(Configurable):
    def __init__(self, image_publisher):
        super(CarlaRunner, self).__init__()
        self._process_frequency = 10
        self._patience_micro = 1000.
        self._vehicle = CarlaHandler((lambda x: image_publisher.publish(x)))

    def internal_quit(self, restarting=False):
        if not restarting:
            self._vehicle.quit()

    def internal_start(self, **kwargs):
        self._vehicle.restart(**kwargs)
        _errors = []
        self._process_frequency = parse_option('clock.hz', int, 10, _errors, **kwargs)
        self._patience_micro = parse_option('patience.ms', int, 200, _errors, **kwargs) * 1000.
        return _errors + self._vehicle.get_errors()

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


def create_runner(ipc_server, config_dir, image_publisher, previous=None):
    # The end-user config overrides come last so all settings are modifiable.
    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(config_dir, '*.ini'))]
    cfg = dict(parser.items('vehicle'))
    cfg.update(dict(parser.items('platform')))
    _starts = 0 if previous is None else previous.get_num_starts()
    if previous is None:
        previous = CarlaRunner(image_publisher)
        previous.start(**cfg)
    else:
        previous.restart(**cfg)
    if previous.get_num_starts() > _starts:
        ipc_server.register_start(previous.get_errors())
        _process_frequency = previous.get_process_frequency()
        _patience_micro = previous.get_patience_micro()
        logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))
    return previous


def main():
    parser = argparse.ArgumentParser(description='Carla vehicle client.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    ipc_server = IPCServer(url='ipc:///byodr/vehicle_c.sock', event=quit_event)
    threads = [teleop, pilot, ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    try:
        runner = create_runner(ipc_server, args.config, image_publisher)
        _period = 1. / runner.get_process_frequency()
        _patience_micro = runner.get_patience_micro()
        while not quit_event.is_set():
            c_pilot = _latest_or_none(pilot, patience=_patience_micro)
            c_teleop = _latest_or_none(teleop, patience=_patience_micro)
            if c_teleop is not None and c_teleop.get('button_a', 0):
                runner.quit()
                runner.start()
            if c_pilot is not None:
                runner.drive(c_pilot)
            else:
                runner.noop()
            chat = ipc_chatter.pop_latest()
            if chat and chat.get('command') == 'restart':
                runner = create_runner(ipc_server, args.config, image_publisher, previous=runner)
                _period = 1. / runner.get_process_frequency()
                _patience_micro = runner.get_patience_micro()
            state_publisher.publish(runner.state())
            time.sleep(_period)
        logger.info("Waiting on carla to quit.")
        runner.quit()
    except KeyboardInterrupt:
        quit_event.set()
    except Exception as e:
        logger.error("{}".format(traceback.format_exc(e)))
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
