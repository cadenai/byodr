import argparse
import glob
import logging
import os
from ConfigParser import SafeConfigParser

from byodr.utils import Application
from byodr.utils.ipc import JSONPublisher, LocalIPCServer, json_collector
from byodr.utils.navigate import FileSystemRouteDataSource, ReloadableDataSource
from pilot import CommandProcessor


class PilotApplication(Application):
    def __init__(self, processor, config_dir=os.getcwd()):
        super(PilotApplication, self).__init__()
        self._config_dir = config_dir
        self._processor = processor
        self.publisher = None
        self.ipc_server = None
        self.ipc_chatter = None
        self.teleop = None
        self.external = None
        self.ros = None
        self.vehicle = None
        self.inference = None

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        cfg = dict(parser.items('pilot'))
        self.logger.info(cfg)
        return cfg

    def get_process_frequency(self):
        return self._processor.get_frequency()

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
        commands = (self.teleop(), self.external(), self.ros(), self.vehicle(), self.inference())
        action = self._processor.next_action(*commands)
        if action:
            self.publisher.publish(action)
        chat = self.ipc_chatter()
        if chat is not None:
            if chat.get('command') == 'restart':
                self.setup()


def main():
    parser = argparse.ArgumentParser(description='Pilot.')
    parser.add_argument('--name', type=str, default='none', help='Process name.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    parser.add_argument('--routes', type=str, default='/routes', help='Directory with the navigation routes.')
    args = parser.parse_args()

    route_store = ReloadableDataSource(FileSystemRouteDataSource(directory=args.routes, load_instructions=True))
    application = PilotApplication(processor=CommandProcessor(route_store), config_dir=args.config)
    quit_event = application.quit_event
    logger = application.logger

    teleop = json_collector(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    external = json_collector(url='ipc:///byodr/external.sock', topic=b'aav/external/input', hwm=10, pop=True, event=quit_event)
    ros = json_collector(url='ipc:///byodr/ros.sock', topic=b'aav/ros/input', hwm=10, pop=True, event=quit_event)
    vehicle = json_collector(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = json_collector(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)
    ipc_chatter = json_collector(url='ipc:///byodr/teleop_c.sock', topic=b'aav/teleop/chatter', pop=True, event=quit_event)

    application.teleop = lambda: teleop.get()
    application.external = lambda: external.get()
    application.ros = lambda: ros.get()
    application.vehicle = lambda: vehicle.get()
    application.inference = lambda: inference.get()
    application.ipc_chatter = lambda: ipc_chatter.get()
    application.publisher = JSONPublisher(url='ipc:///byodr/pilot.sock', topic='aav/pilot/output')
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/pilot_c.sock', name='pilot', event=quit_event)
    threads = [teleop, external, ros, vehicle, inference, ipc_chatter, application.ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    application.run()

    route_store.quit()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
