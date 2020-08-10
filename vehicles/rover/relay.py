import glob
import logging
import multiprocessing
import os
import signal
from ConfigParser import SafeConfigParser

import argparse
import time
import usb.core
import usb.util

from byodr.utils.ipc import ReceiverThread
from byodr.utils.option import parse_option
from byodr.utils.protocol import MessageStreamProtocol

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class SingleChannelUsbRelay(object):
    def __init__(self, vendor=0x1a86, product=0x7523):
        self._vendor = vendor
        self._product = product
        self._device = None
        self._endpoint = None

    def attach(self):
        self._device = usb.core.find(idVendor=self._vendor, idProduct=self._product)
        if self._device is None:
            logger.error("Device vendor={} product={} not found.".format(self._vendor, self._product))
            return

        try:
            if self._device.is_kernel_driver_active(0):
                self._device.detach_kernel_driver(0)

            _config = self._device.get_active_configuration()
            _intf = _config[(0, 0)]

            self._endpoint = usb.util.find_descriptor(
                _intf,
                # match the first OUT endpoint
                custom_match=(lambda _e: usb.util.endpoint_direction(_e.bEndpointAddress) == usb.util.ENDPOINT_OUT))

            if self._endpoint is None:
                logger.error("Endpoint not found.")
        except Exception as e:
            logger.error(e)

    def open(self):
        if self._endpoint is not None:
            self._endpoint.write([0xA0, 0x01, 0x00, 0xA1])

    def close(self):
        if self._endpoint is not None:
            self._endpoint.write([0xA0, 0x01, 0x01, 0xA2])


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

    _topic = arguments.topic
    _master_uri = parse_option('ras.master.uri', str, 'none', [], **vehicle_cfg)
    _url = '{}:5555'.format(_master_uri)
    logger.info("Using status url '{}' and topic '{}'.".format(_url, _topic))

    _relay = SingleChannelUsbRelay()
    _relay.attach()
    _integrity = MessageStreamProtocol()

    def _receive(msg):
        _integrity.on_message(msg.get('time'))

    _status = ReceiverThread(url=_url, topic=b'' + _topic, event=quit_event, on_message=_receive)
    _status.start()

    _relay.close()
    _period = 1. / arguments.hz
    while not quit_event.is_set():
        if _integrity.check() > 2:
            _relay.open()
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
    parser_b.add_argument('--topic', type=str, default='ras/drive/status', help='Topic to monitor.')
    parser_b.add_argument('--hz', type=int, default=20, help='Check frequency.')
    parser_b.set_defaults(func=monitor)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
