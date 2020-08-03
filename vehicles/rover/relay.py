import argparse

import usb.core
import usb.util


class SingleChannelUsbRelay(object):
    def __init__(self, vendor=0x1a86, product=0x7523):
        self._vendor = vendor
        self._product = product
        self._device = None
        self._endpoint = None

    def attach(self):
        self._device = usb.core.find(idVendor=self._vendor, idProduct=self._product)
        assert self._device is not None, "Device vendor={} product={} not found.".format(self._vendor, self._product)

        if self._device.is_kernel_driver_active(0):
            self._device.detach_kernel_driver(0)

        _config = self._device.get_active_configuration()
        _intf = _config[(0, 0)]

        self._endpoint = usb.util.find_descriptor(
            _intf,
            # match the first OUT endpoint
            custom_match=(lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT))
        assert self._endpoint is not None, "Endpoint not found."

    def open(self):
        assert self._device is not None, "The instance device is not attached."
        self._endpoint.write([0xA0, 0x01, 0x00, 0xA1])

    def close(self):
        assert self._device is not None, "The instance device is not attached."
        self._endpoint.write([0xA0, 0x01, 0x01, 0xA2])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Open or close the spdt usb relay.')
    parser.add_argument('--cmd', type=str, required=True, help='Open or close the relay.')
    args = parser.parse_args()

    _device = SingleChannelUsbRelay()
    _device.attach()

    if args.cmd == 'open':
        _device.open()
    elif args.cmd == 'close':
        _device.close()
    else:
        raise AssertionError("Invalid command string '{}'.".format(args.cmd))
