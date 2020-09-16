import argparse
import glob
import logging
import os
from ConfigParser import SafeConfigParser

from byodr.utils import Application, timestamp
from byodr.utils.ipc import ReceiverThread, LocalIPCServer, JSONZmqClient, JSONReceiver, CollectorThread
from byodr.utils.option import parse_option
from byodr.utils.protocol import MessageStreamProtocol
from byodr.utils.usbrelay import SingleChannelUsbRelay, StaticChannelRelayHolder, SearchUsbRelayFactory

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'


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


class StatusReceiverThreadFactory(object):
    def __init__(self, topic='ras/drive/status'):
        self._topic = topic

    def create(self, master_uri):
        return ReceiverThread(url=('{}:5555'.format(master_uri)), topic=b'' + self._topic)


class PiClientFactory(object):
    def __init__(self):
        pass

    @staticmethod
    def create(master_uri):
        return JSONZmqClient(urls='{}:5550'.format(master_uri))


class MonitorApplication(Application):
    def __init__(self, relay, client_factory=None, status_factory=None, config_dir=os.getcwd()):
        super(MonitorApplication, self).__init__()
        self._relay = relay
        self._config_dir = config_dir
        self._integrity = MessageStreamProtocol()
        self._status_factory = StatusReceiverThreadFactory() if status_factory is None else status_factory
        self._client_factory = PiClientFactory() if client_factory is None else client_factory
        self._patience_micro = 100.
        self._pi_client = None
        self._status = None
        self._servo_config = None
        self.ipc_server = None
        self.pilot = None
        self.teleop = None
        self.ipc_chatter = None

    def _send_config(self, data):
        if self._pi_client is not None and data is not None:
            self._pi_client.call(dict(time=timestamp(),
                                      method='ras/servo/config',
                                      data=data))

    def _send_drive(self, throttle=0., steering=0., reverse_gear=False, wakeup=False):
        if self._pi_client is not None:
            throttle = max(-1., min(1., throttle))
            steering = max(-1., min(1., steering))
            _reverse = 1 if reverse_gear else 0
            _wakeup = 1 if wakeup else 0
            self._pi_client.call(dict(time=timestamp(),
                                      method='ras/servo/drive',
                                      data=dict(steering=steering, throttle=throttle, reverse=_reverse, wakeup=_wakeup)))

    def _drive(self, pilot, teleop):
        pi_status = None if self._status is None else self._status.pop_latest()
        if pi_status is not None and not bool(pi_status.get('configured')):
            self._send_config(self._servo_config)
        if pilot is None:
            self._send_drive()
        else:
            _reverse = teleop and teleop.get('arrow_down', 0)
            _wakeup = teleop and teleop.get('button_b', 0)
            self._send_drive(steering=pilot.get('steering'), throttle=pilot.get('throttle'), reverse_gear=_reverse, wakeup=_wakeup)

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        return dict(parser.items('vehicle'))

    def _on_receive(self, msg):
        self._integrity.on_message(msg.get('time'))

    def setup(self):
        if self._pi_client is not None:
            self._pi_client.quit()
        if self._status is not None:
            self._status.quit()
        if self.active():
            errors = []
            _config = self._config()
            _process_frequency = parse_option('clock.hz', int, 10, errors, **_config)
            _master_uri = parse_option('ras.master.uri', str, 'none', errors, **_config)
            c_steer = dict(pin=parse_option('ras.servo.steering.pin.nr', int, 0, errors, **_config),
                           min_pw=parse_option('ras.servo.steering.min_pulse_width.ms', float, 0, errors, **_config),
                           max_pw=parse_option('ras.servo.steering.max_pulse_width.ms', float, 0, errors, **_config),
                           frame=parse_option('ras.servo.steering.frame_width.ms', float, 0, errors, **_config))
            c_motor = dict(pin=parse_option('ras.servo.motor.pin.nr', int, 0, errors, **_config),
                           min_pw=parse_option('ras.servo.motor.min_pulse_width.ms', float, 0, errors, **_config),
                           max_pw=parse_option('ras.servo.motor.max_pulse_width.ms', float, 0, errors, **_config),
                           frame=parse_option('ras.servo.motor.frame_width.ms', float, 0, errors, **_config))
            c_throttle = dict(reverse=parse_option('ras.throttle.reverse.gear', int, 0, errors, **_config),
                              forward_shift=parse_option('ras.throttle.domain.forward.shift', float, 0, errors, **_config),
                              backward_shift=parse_option('ras.throttle.domain.backward.shift', float, 0, errors, **_config),
                              scale=parse_option('ras.throttle.domain.scale', float, 0, errors, **_config))
            self.set_hz(_process_frequency)
            self._patience_micro = parse_option('patience.ms', int, 200, errors, **_config) * 1000.
            self._servo_config = dict(steering=c_steer, motor=c_motor, throttle=c_throttle)
            self._pi_client = self._client_factory.create(_master_uri)
            self._status = self._status_factory.create(_master_uri)
            self._status.add_listener(self._on_receive)
            self._status.start()
            self._integrity.reset()
            self.ipc_server.register_start(errors)
            self._send_config(self._servo_config)
            self.logger.info("Processing master uri '{}' at {} Hz.".format(_master_uri, _process_frequency))

    def finish(self):
        self._relay.open()
        if self._pi_client is not None:
            self._pi_client.quit()
        if self._status is not None:
            self._status.quit()

    def step(self):
        # Always consume the latest commands.
        pilot, teleop, _patience_micro = self.pilot, self.teleop, self._patience_micro
        c_pilot = self._latest_or_none(pilot, patience=_patience_micro)
        c_teleop = self._latest_or_none(teleop, patience=_patience_micro)
        n_violations = self._integrity.check()
        if n_violations < -5:
            self._relay.close()
            self._drive(c_pilot, c_teleop)
        elif n_violations > 200:
            # ZeroMQ ipc over tcp does not allow connection timeouts to be set - while the timeout is too high.
            self.setup()  # Resets the protocol.
        elif n_violations > 5:
            self._relay.open()
            self._drive(None, None)
        else:
            self._drive(None, None)
        chat = self.ipc_chatter()
        if chat and chat.get('command') == 'restart':
            self.setup()


def monitor(arguments):
    _relay = SearchUsbRelayFactory().get_relay()
    assert _relay.is_attached(), "The device is not attached."

    try:
        _holder = StaticChannelRelayHolder(relay=_relay, channel=arguments.channel)
        application = MonitorApplication(relay=_holder, config_dir=arguments.config)
        quit_event = application.quit_event

        pilot = JSONReceiver(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output')
        teleop = JSONReceiver(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input')
        ipc_chatter = JSONReceiver(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', pop=True)
        collector = CollectorThread(receivers=(pilot, teleop, ipc_chatter), event=quit_event)

        application.ipc_server = LocalIPCServer(url='ipc:///byodr/vehicle_c.sock', name='relay', event=quit_event)
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
    parser_b.set_defaults(func=monitor)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
