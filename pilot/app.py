import argparse
import glob
import logging
import os
from ConfigParser import SafeConfigParser

from byodr.utils import Application
from byodr.utils.ipc import ReceiverThread, JSONPublisher, LocalIPCServer
from pilot import CommandProcessor


class PilotApplication(Application):
    def __init__(self, config_dir=os.getcwd()):
        super(PilotApplication, self).__init__()
        self._config_dir = config_dir
        self._processor = CommandProcessor()
        self.publisher = None
        self.ipc_server = None
        self.ipc_chatter = None
        self.teleop = None
        self.vehicle = None
        self.inference = None

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        return dict(parser.items('pilot'))

    def setup(self):
        if self.active():
            _restarted = self._processor.restart(**self._config())
            if _restarted:
                self.ipc_server.register_start(self._processor.get_errors())
                _frequency = self._processor.get_frequency()
                self.set_hz(_frequency)
                self.logger.info("Processing at {} Hz - patience is {:2.2f} ms.".format(_frequency, self._processor.get_patience_ms()))

    def finish(self):
        self._processor.quit()

    def step(self):
        commands = (self.teleop.get_latest(), self.vehicle.get_latest(), self.inference.get_latest())
        action = self._processor.next_action(*commands)
        if action:
            self.publisher.publish(action)
        chat = self.ipc_chatter.pop_latest()
        if chat and chat.get('command') == 'restart':
            self.setup()


def main():
    parser = argparse.ArgumentParser(description='Pilot.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    application = PilotApplication(config_dir=args.config)
    quit_event = application.quit_event
    logger = application.logger

    application.publisher = JSONPublisher(url='ipc:///byodr/pilot.sock', topic='aav/pilot/output')
    application.teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    application.vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    application.inference = ReceiverThread(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)
    application.ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/pilot_c.sock', name='pilot', event=quit_event)
    threads = [application.teleop, application.vehicle, application.inference, application.ipc_chatter, application.ipc_server]
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
