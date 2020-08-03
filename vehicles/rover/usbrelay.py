import argparse

import usb.core
import usb.util

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Open or close the spdt usb relay.')
    parser.add_argument('--cmd', type=str, required=True, help='Open or close the relay.')
    args = parser.parse_args()

    dev = usb.core.find(idVendor=0x1a86, idProduct=0x7523)

    assert dev is not None, "Device not found"

    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)

    cfg = dev.get_active_configuration()
    intf = cfg[(0, 0)]

    ep = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match=(lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT))

    assert ep is not None, "Endpoint not found"

    open_relay_cmd = [0xA0, 0x01, 0x00, 0xA1]
    close_relay_cmd = [0xA0, 0x01, 0x01, 0xA2]

    if args.cmd == 'open':
        ep.write(open_relay_cmd)
    elif args.cmd == 'close':
        ep.write(close_relay_cmd)
    else:
        raise AssertionError("Invalid command string '{}'.".format(args.cmd))
