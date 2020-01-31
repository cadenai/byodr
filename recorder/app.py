import argparse
import json
import logging
import multiprocessing
import os
import signal
import time
import traceback

from jsoncomment import JsonComment

from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher
from recorder import get_or_create_recorder
from store import Event

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def to_event(blob, vehicle, image):
    event = Event(
        timestamp=blob.get('time'),
        image=image,
        steer_src=blob.get('steering_driver'),
        speed_src=blob.get('speed_driver'),
        command_src=blob.get('steering_driver'),
        steering=blob.get('steering'),
        desired_speed=blob.get('desired_speed'),
        actual_speed=vehicle.get('velocity'),
        heading=vehicle.get('heading'),
        throttle=blob.get('throttle'),
        command=blob.get('instruction'),
        x_coordinate=vehicle.get('x_coordinate'),
        y_coordinate=vehicle.get('y_coordinate')
    )
    event.save_event = blob.get('save_event')
    return event


class EventHandler(object):
    def __init__(self, directory, **kwargs):
        self._directory = directory
        self._vehicle = kwargs.get('recorder.constant.vehicle.type')
        self._config = kwargs.get('recorder.constant.vehicle.config')
        self._active = False
        self._driver = None
        self._recorder = get_or_create_recorder(directory=directory, mode=None)

    def _instance(self, driver):
        mode = None
        if driver in ('driver_mode.teleop.cruise', 'driver_mode.automatic.backend'):
            mode = 'record.mode.driving'
        elif driver == 'driver_mode.inference.dnn':
            mode = 'record.mode.interventions'
        return get_or_create_recorder(mode=mode,
                                      directory=self._directory,
                                      vehicle_type=self._vehicle,
                                      vehicle_config=self._config)

    def state(self):
        return dict(active=self._active, mode=self._recorder.get_mode())

    def record(self, blob, vehicle, image):
        # Switch on automatically and then off only when the driver changes.
        if not self._active and blob.get('cruise_speed') > 1e-3:
            self._recorder.start()
            self._active = True
        _driver = blob.get('driver')
        if self._active and self._driver != _driver:
            self._driver = _driver
            self._recorder.stop()
            self._active = False
            self._recorder = self._instance(_driver)
            self._recorder.start()
        if self._active and blob.get('save_event', False):
            self._recorder.do_record(to_event(blob, vehicle, image))


def main():
    parser = argparse.ArgumentParser(description='Recorder.')
    parser.add_argument('--sessions', type=str, required=True, help='Sessions directory.')
    parser.add_argument('--config', type=str, required=True, help='Config file location.')
    parser.add_argument('--clock', type=int, required=True, help='Clock frequency in hz.')
    parser.add_argument('--patience', type=float, default=.050, help='Maximum age of an event before it is considered stale.')
    args = parser.parse_args()

    sessions_dir = os.path.expanduser(args.sessions)
    assert os.path.exists(sessions_dir), "Cannot use sessions directory '{}'".format(sessions_dir)

    with open(args.config, 'r') as cfg_file:
        cfg = JsonComment(json).loads(cfg_file.read())
    for key in sorted(cfg):
        logger.info("{} = {}".format(key, cfg[key]))

    _process_frequency = args.clock
    _patience = args.patience
    logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience * 1000))
    max_duration = 1. / _process_frequency

    handler = EventHandler(directory=sessions_dir, **cfg)
    state_publisher = JSONPublisher(url='ipc:///byodr/recorder.sock', topic='aav/recorder/state')

    threads = []
    camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    threads.append(camera)
    threads.append(pilot)
    threads.append(vehicle)
    [t.start() for t in threads]

    try:
        while not quit_event.is_set():
            proc_start = time.time()
            blob = pilot.get_latest()
            image_md, image = camera.capture()
            if None not in (blob, image_md):
                blob_time, image_time = blob.get('time'), image_md.get('time')
                _now = time.time()
                if (_now - blob_time) < _patience and (_now - image_time) < _patience:
                    handler.record(blob, vehicle.get_latest(), image)
            state_publisher.publish(handler.state())
            _proc_sleep = max_duration - (time.time() - proc_start)
            if _proc_sleep < 0:
                logger.warning("Cannot maintain {} Hz.".format(_process_frequency))
            time.sleep(max(0, _proc_sleep))
    except KeyboardInterrupt:
        quit_event.set()
    except StandardError as e:
        logger.error("{}".format(traceback.format_exc(e)))
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
