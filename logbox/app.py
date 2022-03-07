from __future__ import absolute_import

import argparse
import logging
import os
import sys

from pymongo import MongoClient

from byodr.utils import Application, timestamp
from byodr.utils.ipc import CameraThread, json_collector

if sys.version_info > (3,):
    from configparser import ConfigParser as SafeConfigParser
else:
    from six.moves.configparser import SafeConfigParser

logger = logging.getLogger(__name__)

TRIGGER_SERVICE_START = 2 ** 0


class LogApplication(Application):
    def __init__(self, hz, config_dir=os.getcwd()):
        super(LogApplication, self).__init__(run_hz=hz)
        self._config_dir = config_dir
        self._mongo = None
        self._database = None
        self.camera = None
        self.pilot = None
        self.vehicle = None
        self.inference = None

    def _config(self):
        parser = SafeConfigParser()

    def setup(self):
        self._mongo = MongoClient()
        self._database = self._mongo.blackbox
        self._database.events.insert_one({'time': timestamp(), 'trigger': TRIGGER_SERVICE_START})
        # pass

    def finish(self):
        if self._mongo is not None:
            self._mongo.close()

    def step(self):
        pass


def main():
    parser = argparse.ArgumentParser(description='Logger black box service.')
    parser.add_argument('--name', type=str, default='none', help='Process name.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    application = LogApplication(hz=20, config_dir=args.config)
    quit_event = application.quit_event

    pilot = json_collector(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = json_collector(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = json_collector(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)

    application.camera = CameraThread(url='ipc:///byodr/camera_0.sock', topic=b'aav/camera/0', event=quit_event)
    application.pilot = lambda: pilot.get()
    application.vehicle = lambda: vehicle.get()
    application.inference = lambda: inference.get()

    threads = [application.camera, pilot, vehicle, inference]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    application.run()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(asctime)s %(filename)s %(funcName)s %(message)s', datefmt='%Y%m%d:%H:%M:%S %p %Z')
    logging.getLogger().setLevel(logging.INFO)
    main()
