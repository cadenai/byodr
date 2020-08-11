import glob
import logging
import multiprocessing
import os
import signal
from ConfigParser import SafeConfigParser

import argparse
import time

from byodr.utils.ipc import ReceiverThread
from byodr.utils.option import parse_option
from byodr.utils.protocol import MessageStreamProtocol
from byodr.utils.usbrelay import SingleChannelUsbRelay, DoubleChannelUsbRelay

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


def monitor(arguments):
    cfg_parser = SafeConfigParser()
    config_dir = arguments.config
    [cfg_parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(config_dir, '*.ini'))]
    vehicle_cfg = dict(cfg_parser.items('vehicle'))

    _channel = arguments.channel
    _topic = arguments.topic
    _master_uri = parse_option('ras.master.uri', str, 'none', [], **vehicle_cfg)
    _url = '{}:5555'.format(_master_uri)
    logger.info("Using status url '{}' and topic '{}'.".format(_url, _topic))

    _relay = DoubleChannelUsbRelay()
    _relay.attach()
    _integrity = MessageStreamProtocol()

    def _receive(msg):
        _integrity.on_message(msg.get('time'))

    _status = ReceiverThread(url=_url, topic=b'' + _topic, event=quit_event, on_message=_receive)
    _status.start()

    _relay.close(channel=_channel)
    _period = 1. / arguments.hz
    while not quit_event.is_set():
        if _integrity.check() > 2:
            _relay.open(channel=_channel)
            # A reset could be too fast for the dependant circuit to reboot.
            # self._relay.close()
            _integrity.reset()
            logger.warning("Relay Open")
        time.sleep(_period)

    # Leave the relay state as is - if open rely on the connected component to check its own integrity.
    # _relay.open()
    _status.join()


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
