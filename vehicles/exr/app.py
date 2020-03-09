import argparse
import collections
import logging
import math
import multiprocessing
import signal
import struct
import threading
import time
from ConfigParser import SafeConfigParser
from functools import partial

import can
import cv2
import numpy as np
from can import CanError
from pyueye import ueye

from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher
from camera import Camera, FrameThread

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

# Shape to pull of each camera.
CAMERA_SHAPE = (240, 320, 3)

PEAK_CAN_BIT_RATE = 100000
CAN_BUS_HZ = 20

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def init_drives(bus):
    """
        Settings from 2018 RUVU Robotics Rein Appeldoorn / Rokus Ottervanger

        can0  TX - -  100   [8]  00 00 00 00 00 00 00 3F   '.......?'
        can0  TX - -  130   [8]  00 00 00 00 06 40 00 00   '.....@..'
        can0  TX - -  120   [8]  64 00 00 00 06 40 00 00   'd....@..'
        can0  TX - -  101   [8]  00 00 00 00 00 00 00 3F   '.......?'
        can0  TX - -  131   [8]  00 00 00 00 06 40 00 00   '.....@..'
        can0  TX - -  121   [8]  64 00 00 00 06 40 00 00   'd....@..'
    """
    # id=0x130,0x131 Bandwidth control last 4 bytes IQ20 [0; 100]
    # Left drive.
    bus.send(can.Message(check=True, arbitration_id=0x100, is_extended_id=False, data=([0, 0, 0, 0, 0, 0, 0, 0x3f])))
    bus.send(can.Message(check=True, arbitration_id=0x130, is_extended_id=False, data=([0, 0, 0, 0, 6, 0x40, 0, 0])))
    # Right drive.
    bus.send(can.Message(check=True, arbitration_id=0x101, is_extended_id=False, data=([0, 0, 0, 0, 0, 0, 0, 0x3f])))
    bus.send(can.Message(check=True, arbitration_id=0x131, is_extended_id=False, data=([0, 0, 0, 0, 6, 0x40, 0, 0])))


def param_drives(bus):
    # id=0x120,0x121 Max acceleration IQ24 [0; 120] and jerk IQ20 [0; 750].
    # IQ24: struct.pack('>i', int(100 * math.pow(2, 24))).encode('hex')
    # IQ20: struct.pack('>i', int(300 * math.pow(2, 20))).encode('hex')
    bus.send(can.Message(check=True, arbitration_id=0x120, is_extended_id=False, data=([0x64, 0, 0, 0, 0x06, 0x40, 0, 0])))
    bus.send(can.Message(check=True, arbitration_id=0x121, is_extended_id=False, data=([0x64, 0, 0, 0, 0x06, 0x40, 0, 0])))


def tanh(x, scale):
    return (1 - math.exp(-scale * x)) / (1 + math.exp(-scale * x))


def drive_values(steering=0., throttle=0., scale=1., is_teleop=True):
    # Disable tank drive or turn on spot unless under direct control.
    _minimum = -1. if is_teleop else 0.
    left = max(_minimum, min(1., throttle + tanh(steering, scale=scale)))
    right = max(_minimum, min(1., throttle - tanh(steering, scale=scale)))
    return left, right


def drive_bytes(x):
    """
        IQ24 use half-power = [-2; 2].
    """
    x = 2 * max(-1, min(1, x))
    return struct.pack('>i', int(x * math.pow(2, 24)))


def m_speed_ref(steering, throttle, scale, is_teleop):
    left, right = drive_values(steering, throttle, scale, is_teleop)
    _data = drive_bytes(left) + drive_bytes(-right)
    m = can.Message(check=True, arbitration_id=0x111, is_extended_id=False, data=_data)
    return m


class NoneBus(object):
    def __init__(self):
        pass

    @staticmethod
    def send(m):
        # logger.debug(m)
        pass

    @staticmethod
    def shutdown():
        pass


class CanBusThread(threading.Thread):
    def __init__(self, bus, event, scale=1., frequency=CAN_BUS_HZ):
        super(CanBusThread, self).__init__()
        self._bus_name = bus
        self._ms = 1. / frequency
        self._quit_event = event
        self._scale = scale
        self._queue = collections.deque(maxlen=1)
        self._bus = NoneBus()
        self.reset()

    def _create_bus(self):
        name = self._bus_name
        if name in (None, 'None', 'none'):
            self._bus = NoneBus()
        elif name.lower() == 'pcan':
            self._bus = can.ThreadSafeBus(bustype='pcan', bitrate=PEAK_CAN_BIT_RATE)
        else:
            self._bus = can.ThreadSafeBus(bustype='socketcan', channel=name, bitrate=PEAK_CAN_BIT_RATE)

    def _reset_once(self):
        self._bus.shutdown()
        self._create_bus()
        init_drives(self._bus)
        param_drives(self._bus)

    def reset(self, tries=0):
        self.set_command()
        while tries < 4:
            try:
                self._reset_once()
                logger.info("Reset successful.")
                break
            except CanError as e:
                logger.warn("Reset failed with '{}'.".format(e))
                tries += 1
                time.sleep(1)

    def set_command(self, steering=0., throttle=0., is_teleop=True):
        self._queue.appendleft((steering, throttle, is_teleop))

    def run(self):
        while not self._quit_event.is_set():
            try:
                steering, throttle, is_teleop = self._queue[0]
                self._bus.send(m_speed_ref(steering=steering, throttle=throttle, scale=self._scale, is_teleop=is_teleop))
                time.sleep(self._ms)
            except CanError as be:
                logger.error(be)
                self.reset()

    def quit(self):
        self._quit_event.set()
        self._queue.clear()
        try:
            self._bus.shutdown()
        except CanError:
            pass


class IDSImagingThread(FrameThread):
    def __init__(self, device_id=0, views=None, copy=True):
        self._cam = Camera(device_id=device_id)
        self._cam.init()
        self._cam.set_parameters_from_memory()
        self._cam.set_colormode(ueye.IS_CM_BGR8_PACKED)
        self._cam.alloc()
        self._cam.capture_video()
        FrameThread.__init__(self, self._cam, views=views, copy=copy)

    def quit(self):
        IDSImagingThread.stop(self)
        IDSImagingThread.join(self)
        self._cam.exit()


class TwistHandler(object):
    def __init__(self, image_shape=CAMERA_SHAPE, bus_name=None, event=quit_event, steering_scale=1., fn_callback=(lambda x: x)):
        super(TwistHandler, self).__init__()
        self._image_shape = image_shape
        self._bus_name = bus_name
        self._fn_callback = fn_callback
        self._gate = CanBusThread(bus=bus_name, event=event, scale=steering_scale)
        self._gate.start()
        self._queue1 = collections.deque(maxlen=2)
        self._queue2 = collections.deque(maxlen=2)
        self._camera1 = IDSImagingThread(device_id=1, views=partial(self._handle, _queue=self._queue1, do_call=True))
        self._camera2 = IDSImagingThread(device_id=2, views=partial(self._handle, _queue=self._queue2, do_call=False))
        self._camera1.start()
        self._camera2.start()

    def _camera(self):
        h, w, c = self._image_shape
        img = np.zeros((2 * h, w, c), dtype=np.uint8)
        try:
            img[:h, :, :] = self._queue1[0]
            img[h:, :, :] = self._queue2[0]
            return img
        except IndexError:
            return img

    def _handle(self, image_data, _queue, do_call=True):
        h, w, _ = self._image_shape
        frame = cv2.resize(image_data.as_1d_image(), (w, h))
        _queue.appendleft(frame)
        if do_call:
            self._fn_callback(self._camera())

    def quit(self):
        self._gate.quit()
        self._camera1.quit()
        self._camera2.quit()

    def _drive(self, steering, throttle, driver=None):
        try:
            is_teleop = driver == 'driver_mode.teleop.direct'
            if not is_teleop and abs(throttle) < 1e-2:
                steering, throttle = 0., 0.
            self._gate.set_command(steering=steering, throttle=throttle, is_teleop=is_teleop)
        except Exception as e:
            logger.error("{}".format(e))

    @staticmethod
    def state():
        x, y = 0, 0
        return dict(x_coordinate=x,
                    y_coordinate=y,
                    heading=0,
                    velocity=0,
                    time=time.time())

    def noop(self):
        self._drive(steering=0, throttle=0)

    def drive(self, cmd):
        if cmd is not None:
            self._drive(steering=cmd.get('steering'), throttle=cmd.get('throttle'), driver=cmd.get('driver'))


def main():
    parser = argparse.ArgumentParser(description='Exr main.')
    parser.add_argument('--config', type=str, required=True, help='Config file location.')
    args = parser.parse_args()

    parser = SafeConfigParser()
    [parser.read(_f) for _f in args.config.split(',')]
    cfg = dict(parser.items('vehicle'))
    cfg.update(dict(parser.items('platform')))
    for key in sorted(cfg):
        logger.info("{} = {}".format(key, cfg[key]))

    _process_frequency = int(cfg.get('clock.hz'))
    _patience = float(cfg.get('patience.ms')) / 1000
    logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience * 1000))

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    _interface = cfg.get('can.interface')
    _steering_scale = float(cfg.get('drive.motor.steering.scale'))
    vehicle = TwistHandler(bus_name=_interface, steering_scale=_steering_scale, fn_callback=(lambda im: image_publisher.publish(im)))
    threads = []
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    threads.append(pilot)
    [t.start() for t in threads]

    _period = 1. / _process_frequency
    while not quit_event.is_set():
        command = pilot.get_latest()
        _command_time = 0 if command is None else command.get('time')
        _command_age = time.time() - _command_time
        _on_time = _command_age < _patience
        if _on_time:
            vehicle.drive(command)
        else:
            vehicle.noop()
        state_publisher.publish(vehicle.state())
        time.sleep(_period)

    logger.info("Waiting on vehicle to quit.")
    vehicle.quit()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
