from __future__ import absolute_import
import logging

import usb.core
import usb.util

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


class FourChannelUsbRelay(object):
    """
    Conrad Components 393905 Relay Module 5 V/DC
    Conrad article 393905
    Conrad supplier 393905
    EAN: 4016138810585
    """

    def __init__(self, vendor=0x10c4, product=0xea60):
        """
        Note - this class currently only supports the first two channels.
        device.ctrl_transfer(reqType, bReq, wVal, 0xFF, [])   # all on
        device.ctrl_transfer(reqType, bReq, wVal, 0xFCFF, []) # 1 and 2 on
        device.ctrl_transfer(reqType, bReq, wVal, 0xFC, [])   # 3 and 4 on
        - find bitmasks for individual control of channels 3 and 4
        """
        self._vendor = vendor
        self._product = product
        self._device_on = [0xFC01, 0xFC02]
        self._device_off = [0xFF01, 0xFF02]
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

    def open(self, channel=0):
        assert self.is_attached(), "The device is not attached."
        self._device.ctrl_transfer(0x41, 0xFF, 0x37E1, self._device_off[channel], [])

    def close(self, channel=0):
        assert self.is_attached(), "The device is not attached."
        self._device.ctrl_transfer(0x41, 0xFF, 0x37E1, self._device_on[channel], [])


class SearchUsbRelayFactory(object):
    def __init__(self):
        _relay = FourChannelUsbRelay()
        if not _relay.poll():
            _relay = DoubleChannelUsbRelay()
        _relay.attach()
        self._relay = _relay

    def get_relay(self):
        return self._relay


class StaticChannelRelayHolder(object):
    def __init__(self, relay, channel=0):
        self._relay = relay
        self._channel = channel

    def open(self):
        self._relay.open(self._channel)

    def close(self):
        self._relay.close(self._channel)
