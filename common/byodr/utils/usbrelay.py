from __future__ import absolute_import

import logging
import multiprocessing
import time

import usb.core
import usb.util
from usb.util import CTRL_IN, CTRL_OUT, CTRL_TYPE_VENDOR

logger = logging.getLogger(__name__)


class SingleChannelUsbRelay(object):
    """
    HALJIA USB-relaismodule USB Smart Control Switch Intelligent Switch Control USB Relais module
    """

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


class DoubleChannelUsbRelay(object):
    """
    ICQUANZX SRD-05VDC-SL-C 2-way
    """

    def __init__(self, vendor=0x16c0, product=0x05df):
        self._vendor = vendor
        self._product = product
        self._device_on = [
            [0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]
        self._device_off = [
            [0xFC, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0xFC, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]
        self._device = None

    def find(self):
        return usb.core.find(idVendor=self._vendor, idProduct=self._product)

    def poll(self):
        return self.find() is not None

    def attach(self):
        self._device = self.find()
        self._device = usb.core.find(idVendor=self._vendor, idProduct=self._product)
        if self._device is None:
            logger.error("Device vendor={} product={} not found.".format(self._vendor, self._product))
            return

        try:
            if self._device.is_kernel_driver_active(0):
                self._device.detach_kernel_driver(0)
            self._device.set_configuration()
        except Exception as e:
            logger.error(e)

    def is_attached(self):
        return self._device is not None

    def open(self, channel=0):
        assert self.is_attached(), "The device is not attached."
        self._device.ctrl_transfer(0x21, 0x09, 0x0300, 0x0000, "".join(chr(n) for n in self._device_off[channel]), 1000)

    def close(self, channel=0):
        assert self.is_attached(), "The device is not attached."
        self._device.ctrl_transfer(0x21, 0x09, 0x0300, 0x0000, "".join(chr(n) for n in self._device_on[channel]), 1000)


class TransientMemoryRelay(object):
    def __init__(self, num_channels=4):
        self._state = [0] * num_channels

    def open(self, channel=0):
        self._state[channel] = 0

    def close(self, channel=0):
        self._state[channel] = 1

    def states(self):
        return [bool(x) for x in self._state]


class FourChannelUsbRelay(object):
    """
    Conrad Components 393905 Relay Module 5 V/DC
    Conrad article 393905
    Conrad supplier 393905
    EAN: 4016138810585
    Type: CP210x
    """

    MAX_GPIO_INDEX = 4

    CP210X_VENDOR_ID = 0x10c4
    CP210X_PRODUCT_ID = 0xea60

    CP210X_REQUEST_TYPE_READ = CTRL_IN | CTRL_TYPE_VENDOR
    CP210X_REQUEST_TYPE_WRITE = CTRL_OUT | CTRL_TYPE_VENDOR

    CP210X_REQUEST_VENDOR = 0xFF

    CP210X_VALUE_READ_LATCH = 0x00C2
    CP210X_VALUE_WRITE_LATCH = 0x37E1

    def __init__(self, vendor=CP210X_VENDOR_ID, product=CP210X_PRODUCT_ID):
        """
        Adapted from https://github.com/jjongbloets/CP210xControl/blob/master/CP210xControl/model.py.
        """
        self._vendor = vendor
        self._product = product
        self._device = None

    def find(self):
        return usb.core.find(idVendor=self._vendor, idProduct=self._product)

    def poll(self):
        return self.find() is not None

    def attach(self):
        self._device = self.find()
        if self._device is None:
            logger.error("Device vendor={} product={} not found.".format(self._vendor, self._product))
            return

        try:
            if self._device.is_kernel_driver_active(0):
                self._device.detach_kernel_driver(0)
            self._device.set_configuration()
        except Exception as e:
            logger.error(e)

    def is_attached(self):
        return self._device is not None

    def _query(self, request, value, index, length):
        assert self.is_attached(), "The device is not attached."
        return self._device.ctrl_transfer(self.CP210X_REQUEST_TYPE_READ, request, value, index, length)

    def _write(self, request, value, index, data):
        assert self.is_attached(), "The device is not attached."
        return self._device.ctrl_transfer(self.CP210X_REQUEST_TYPE_WRITE, request, value, index, data)

    def _set_gpio(self, index, value):
        mask = 1 << index
        values = (0 if value else 1) << index
        msg = (values << 8) | mask
        return self._write(self.CP210X_REQUEST_VENDOR, self.CP210X_VALUE_WRITE_LATCH, msg, 0)

    def _get_gpio_states(self):
        results = []
        response = self._query(self.CP210X_REQUEST_VENDOR, self.CP210X_VALUE_READ_LATCH, 0, 1)
        if len(response) > 0:
            response = response[0]
        for idx in range(self.MAX_GPIO_INDEX):
            results.append((response & (1 << idx)) == 0)
        return results

    def open(self, channel=0):
        self._set_gpio(channel, 0)

    def close(self, channel=0):
        self._set_gpio(channel, 1)

    def states(self):
        return self._get_gpio_states()


class SearchUsbRelayFactory(object):
    def __init__(self):
        _relay = FourChannelUsbRelay()
        # The others are not supported until they expose a read state method.
        # if not _relay.poll():
        #     _relay = DoubleChannelUsbRelay()
        _relay.attach()
        self._relay = _relay

    def get_relay(self):
        return self._relay


class StaticRelayHolder(object):
    def __init__(self, relay, default_channels=(0,)):
        self._relay = relay
        self._default_channels = self._tup_or_li(default_channels)
        self._pulse_channels = ()
        self._lock = multiprocessing.Lock()

    @staticmethod
    def _tup_or_li(arg):
        return arg if isinstance(arg, tuple) or isinstance(arg, list) else (arg,)

    def _arg_(self, ch=None):
        return self._default_channels if ch is None else self._tup_or_li(ch)

    def set_pulse_channels(self, channels):
        with self._lock:
            self._pulse_channels = self._tup_or_li(channels)

    def open(self, channels=None):
        with self._lock:
            [self._relay.open(ch) for ch in self._arg_(channels)]

    def close(self, channels=None):
        with self._lock:
            for ch in self._arg_(channels):
                self._relay.close(ch)
                if ch in self._pulse_channels:
                    time.sleep(0.100)
                    self._relay.open(ch)

    def states(self):
        with self._lock:
            return self._relay.states()

    def pulse_config(self):
        with self._lock:
            return [i in self._pulse_channels for i in range(len(self._relay.states()))]
