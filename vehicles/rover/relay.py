import argparse
import glob
import logging
import multiprocessing
import os
import signal
from ConfigParser import SafeConfigParser

from byodr.utils import Application
from byodr.utils.ipc import ReceiverThread, LocalIPCServer
from byodr.utils.option import parse_option
from byodr.utils.protocol import MessageStreamProtocol
from byodr.utils.usbrelay import SingleChannelUsbRelay, StaticChannelRelayHolder, SearchUsbRelayFactory

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def execute(arguments):
    _device = SingleChannelUsbRelay()
    _device.attach()

    _cmd = arguments.cmd
    if _cmd == 'open':
        _device.open()
    elif _cmd == 'close':
        _device.close()
    else:
        raise AssertionError("Invalid command string '{}'.".format(_cmd))


class MonitorReceiverThreadFactory(object):
    def __init__(self, topic=''):
        self._topic = topic

    def create(self, **kwargs):
        errors = []
        _master_uri = parse_option('ras.master.uri', str, 'none', errors, **kwargs)
        return errors, ReceiverThread(url=('{}:5555'.format(_master_uri)), topic=b'' + self._topic)


class MonitorApplication(Application):
    def __init__(self, relay, receiver_factory, hz=10, config_dir=os.getcwd()):
        super(MonitorApplication, self).__init__(run_hz=hz)
        self._config_dir = config_dir
        self._process_frequency = hz
        self._relay = relay
        self._integrity = MessageStreamProtocol()
        self._receiver_factory = receiver_factory
        self._receiver = None
        self.ipc_chatter = None
        self.ipc_server = None

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        return dict(parser.items('vehicle'))

    def _on_receive(self, msg):
        self._integrity.on_message(msg.get('time'))

    def setup(self):
        if self._receiver is not None:
            self._receiver.quit()
        if self.active():
            _errors, self._receiver = self._receiver_factory.create(**self._config())
            self._receiver.add_listener(self._on_receive)
            self._receiver.start()
            self._integrity.reset()
            self.ipc_server.register_start(_errors)
            self.logger.info("Processing at {} Hz.".format(self._process_frequency))

    def finish(self):
        self._relay.open()
        if self._receiver is not None:
            self._receiver.quit()

    def step(self):
        n_violations = self._integrity.check()
        if n_violations < -5:
            self._relay.close()
        elif n_violations > 200:
            # ZeroMQ ipc over tcp does not allow connection timeouts to be set - while the timeout is too high.
            self.setup()  # Resets the protocol.
        elif n_violations > 5:
            self._relay.open()
        chat = self.ipc_chatter.pop_latest()
        if chat and chat.get('command') == 'restart':
            self.setup()


def monitor(arguments):
    _relay = SearchUsbRelayFactory().get_relay()
    assert _relay.is_attached(), "The device is not attached."

    try:
        _holder = StaticChannelRelayHolder(relay=_relay, channel=arguments.channel)
        _receiver_factory = MonitorReceiverThreadFactory(topic=arguments.topic)

        application = MonitorApplication(relay=_holder, receiver_factory=_receiver_factory, hz=arguments.hz, config_dir=arguments.config)
        application.ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
        application.ipc_server = LocalIPCServer(url='ipc:///byodr/vehicle_c.sock', name='relay', event=quit_event)

        threads = [application.ipc_chatter, application.ipc_server]
        if quit_event.is_set():
            return 0

        [t.start() for t in threads]
        application.run()

        logger.info("Waiting on threads to stop.")
        [t.join() for t in threads]
    finally:
        _relay.open()


def main():
    parser = argparse.ArgumentParser(description='Spdt usb relay.')
    subparsers = parser.add_subparsers(help='Methods.')

    parser_a = subparsers.add_parser('exec', help='Open or close the relay.')
    parser_a.add_argument('--cmd', type=str, required=True, help='Open or close the relay.')
    parser_a.set_defaults(func=execute)

    parser_b = subparsers.add_parser('monitor', help='Open or close the relay depending on the availability of a service and topic.')
    parser_b.add_argument('--config', type=str, default='/config', help='Config directory path.')
    parser_b.add_argument('--channel', type=int, default=0, help='Relay channel id.')
    parser_b.add_argument('--topic', type=str, default='ras/drive/status', help='Topic to monitor.')
    parser_b.add_argument('--hz', type=int, default=20, help='Check frequency.')
    parser_b.set_defaults(func=monitor)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
