import logging

import usb.core
import usb.util

logger = logging.getLogger(__name__)


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


class DoubleChannelUsbRelay(object):
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
        self._channel0_on = "".join(chr(n) for n in [0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._channel1_on = "".join(chr(n) for n in [0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._channel0_off = "".join(chr(n) for n in [0xFC, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._channel1_off = "".join(chr(n) for n in [0xFC, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._device = None

    def attach(self):
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


class StaticChannelRelayHolder(object):
    def __init__(self, relay, channel=0):
        self._relay = relay
        self._channel = channel

    def open(self):
        self._relay.open(self._channel)

    def close(self):
        self._relay.close(self._channel)
