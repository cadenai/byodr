from __future__ import absolute_import
import argparse
import glob
import logging
import os

from six.moves.configparser import SafeConfigParser
from core import CommandProcessor
from relay import NoopRelay, RealMonitoringRelay

from byodr.utils import Application
from byodr.utils.ipc import JSONPublisher, LocalIPCServer, json_collector
from byodr.utils.navigate import FileSystemRouteDataSource, ReloadableDataSource
from byodr.utils.usbrelay import StaticRelayHolder, SearchUsbRelayFactory

logger = logging.getLogger(__name__)


class PilotApplication(Application):
    def __init__(self, processor, relay, config_dir=os.getcwd()):
        super(PilotApplication, self).__init__()
        self._config_dir = config_dir
        self._processor = processor
        self._relay = relay
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
        [parser.read(_f) for _f in glob.glob(os.path.join(self._config_dir, '*.ini'))]
        cfg = dict(parser.items('pilot')) if parser.has_section('pilot') else {}
        self.logger.info(cfg)
        return cfg

    def get_process_frequency(self):
        return self._processor.get_frequency()

    def setup(self):
        if self.active():
            _relay_errors = self._relay.setup()
            _restarted = self._processor.restart(**self._config())
            if _restarted:
                self.ipc_server.register_start(_relay_errors + self._processor.get_errors())
                _frequency = self._processor.get_frequency()
                self.set_hz(_frequency)
                self.logger.info("Processing at {} Hz - patience is {:2.2f} ms.".format(_frequency, self._processor.get_patience_ms()))

    def finish(self):
        self._relay.quit()
        self._processor.quit()

    def step(self):
        teleop = self.teleop()
        commands = (teleop, self.external(), self.ros(), self.vehicle(), self.inference())
        pilot = self._processor.next_action(*commands)
        self._relay.step(pilot, teleop)
        if pilot is not None:
            self.publisher.publish(pilot)
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

    _relay = SearchUsbRelayFactory().get_relay()
    logger.info("The USB Relay is {} attached.".format('well' if _relay.is_attached() else 'not'))

    if _relay.is_attached():
        _holder = StaticRelayHolder(relay=_relay, channels=(0, 1))
        monitoring_relay = RealMonitoringRelay(relay=_holder, config_dir=args.config)
    else:
        monitoring_relay = NoopRelay()

    try:
        route_store = ReloadableDataSource(FileSystemRouteDataSource(directory=args.routes, load_instructions=True))
        application = PilotApplication(processor=CommandProcessor(route_store), relay=monitoring_relay, config_dir=args.config)
        quit_event = application.quit_event

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
    finally:
        monitoring_relay.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(asctime)s %(filename)s %(funcName)s %(message)s', datefmt='%Y%m%d:%H:%M:%S %p %Z')
    logging.getLogger().setLevel(logging.INFO)
    main()
