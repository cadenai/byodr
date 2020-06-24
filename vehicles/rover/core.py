import Queue
import logging
import math
import multiprocessing
import threading
import time

import cv2
import numpy as np
import requests
from requests.auth import HTTPDigestAuth

from byodr.utils import timestamp
from byodr.utils.option import hash_dict, parse_option
from video import GstRawSource

logger = logging.getLogger(__name__)

CH_NONE, CH_THROTTLE, CH_STEERING, CH_BOTH = (0, 1, 2, 3)
CTL_LAST = 0

# Safety - cap the range that is availble through user configuration.
# Ranges are from the servo domain, i.e. 1/90 - the pair is for forwards and reverse.
# The ranges do not have to be symmetrical - backwards may seem faster here but not in practice (depends on the esc settings).
D_SPEED_PROFILES = {
    'economy': (4, 8),
    'standard': (5, 8),
    'sport': (7, 10),
    'performance': (10, 10)
    # Yep, what's next?
}


class ZMQGate(object):

    def __init__(self, publisher):
        self._errors = []
        self._hash = -1
        self._rps = 0  # Hall sensor revolutions per second.
        self._publisher = None
        self._subscriber = None
        self._lock = threading.Lock()
        self._publisher = publisher

    def restart(self, **kwargs):
        with self._lock:
            _hash = hash_dict(**kwargs)
            if _hash != self._hash:
                self._hash = _hash
                self._start(**kwargs)

    def get_errors(self):
        with self._lock:
            return self._errors

    def _start(self, **kwargs):
        errors = []
        self._circum_m = 2 * math.pi * parse_option('chassis.wheel.radius.meter', float, 0, errors, **kwargs)
        self._gear_ratio = parse_option('chassis.hall.ticks.per.rotation', int, 0, errors, **kwargs)
        c_steer = dict(pin=parse_option('ras.servo.steering.pin.nr', int, 0, errors, **kwargs),
                       min_pw=parse_option('ras.servo.steering.min_pulse_width.ms', float, 0, errors, **kwargs),
                       max_pw=parse_option('ras.servo.steering.max_pulse_width.ms', float, 0, errors, **kwargs),
                       frame=parse_option('ras.servo.steering.frame_width.ms', float, 0, errors, **kwargs))
        c_motor = dict(pin=parse_option('ras.servo.motor.pin.nr', int, 0, errors, **kwargs),
                       min_pw=parse_option('ras.servo.motor.min_pulse_width.ms', float, 0, errors, **kwargs),
                       max_pw=parse_option('ras.servo.motor.max_pulse_width.ms', float, 0, errors, **kwargs),
                       frame=parse_option('ras.servo.motor.frame_width.ms', float, 0, errors, **kwargs))
        c_throttle = dict(reverse=parse_option('ras.throttle.reverse.gear', int, 0, errors, **kwargs),
                          shift=parse_option('ras.throttle.domain.shift', float, 0, errors, **kwargs),
                          scale=parse_option('ras.throttle.domain.scale', float, 0, errors, **kwargs))
        self._servo_config = dict(steering=c_steer, motor=c_motor, throttle=c_throttle)
        self._errors = errors
        if not errors:
            self.publish_config()

    def update_odometer(self, rps):
        # The odometer publishes revolutions per second.
        self._rps = rps

    def publish_config(self):
        self._publisher.publish(data=self._servo_config, topic='ras/servo/config')

    def publish(self, throttle=0., steering=0., reverse_gear=False):
        with self._lock:
            throttle = max(-1., min(1., throttle))
            steering = max(-1., min(1., steering))
            _reverse = 1 if reverse_gear else 0
            self._publisher.publish(dict(steering=steering, throttle=throttle, reverse=_reverse))

    def get_odometer_value(self):
        with self._lock:
            # Convert to travel speed in meters per second.
            return (self._rps / self._gear_ratio) * self._circum_m


class TwistHandler(object):
    def __init__(self, gate, **kwargs):
        super(TwistHandler, self).__init__()
        self._gate = gate
        self._quit_event = multiprocessing.Event()
        self._hash = -1
        self._errors = []
        self._process_frequency = 10
        self._patience_micro = 100.
        self.restart(**kwargs)

    def _configure(self, **kwargs):
        _errors = []
        self._process_frequency = parse_option('clock.hz', int, 10, _errors, **kwargs)
        self._patience_micro = parse_option('patience.ms', int, 200, _errors, **kwargs) * 1000.
        self._errors = _errors
        self._hash = hash_dict(**kwargs)

    def _drive(self, steering=0, throttle=0, reverse_gear=False):
        try:
            if not self._quit_event.is_set():
                self._gate.publish(steering=steering, throttle=throttle, reverse_gear=reverse_gear)
        except Exception as e:
            logger.error("{}".format(e))

    def get_process_frequency(self):
        return self._process_frequency

    def get_patience_micro(self):
        return self._patience_micro

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def restart(self, **kwargs):
        if not self._quit_event.is_set():
            self._configure(**kwargs)
            self._gate.restart(**kwargs)

    def get_errors(self):
        return self._errors + self._gate.get_errors()

    def state(self):
        x, y = 0, 0
        return dict(x_coordinate=x,
                    y_coordinate=y,
                    heading=0,
                    velocity=self._gate.get_odometer_value(),
                    time=timestamp())

    def quit(self):
        self._quit_event.set()

    def update_rps(self, rps):
        self._gate.update_odometer(rps)

    def send_config(self):
        self._gate.publish_config()

    def noop(self):
        self._drive(steering=0, throttle=0)

    def drive(self, pilot, teleop):
        if pilot is None:
            self.noop()
        else:
            _reverse = teleop and teleop.get('arrow_down', 0)
            self._drive(steering=pilot.get('steering'), throttle=pilot.get('throttle'), reverse_gear=_reverse)


class CameraPtzThread(threading.Thread):
    def __init__(self, url, user, password, preset_duration_sec=3.8, scale=100, speed=1., flip=(1, 1)):
        super(CameraPtzThread, self).__init__()
        self._quit_event = multiprocessing.Event()
        self._lock = threading.Lock()
        self._preset_duration = preset_duration_sec
        self._scale = scale
        self._speed = speed
        self._flip = flip
        self._auth = None
        self._url = url
        self._ptz_xml = """
        <PTZData version='2.0' xmlns='http://www.isapi.org/ver20/XMLSchema'>
            <pan>{pan}</pan>
            <tilt>{tilt}</tilt>
            <zoom>0</zoom>
        </PTZData>
        """
        self.set_auth(user, password)
        self._queue = Queue.Queue(maxsize=1)
        self._previous = (0, 0)

    def set_url(self, url):
        with self._lock:
            self._url = url

    def set_auth(self, user, password):
        with self._lock:
            self._auth = HTTPDigestAuth(user, password)

    def set_speed(self, speed):
        with self._lock:
            self._speed = speed

    def set_flip(self, flip):
        with self._lock:
            self._flip = flip

    def _norm(self, value):
        return max(-self._scale, min(self._scale, int(self._scale * value * self._speed)))

    def _perform(self, operation):
        # Goto a preset position takes time.
        prev = self._previous
        if type(prev) == tuple and prev[0] == 'goto_home' and time.time() - prev[1] < self._preset_duration:
            pass
        elif operation != prev:
            ret = self._run(operation)
            if ret and ret.status_code != 200:
                logger.warn("Got status {} on operation {}.".format(ret.status_code, operation))

    def _run(self, operation):
        ret = None
        prev = self._previous
        if type(operation) == tuple and operation[0] == 'set_home':
            x_count = prev[1] if type(prev) == tuple and prev[0] == 'set_home' else 0
            if x_count >= 100 and operation[1]:
                logger.info("Saving ptz home position.")
                self._previous = 'ptz_home_set'
                ret = requests.put(self._url + '/homeposition', auth=self._auth)
            else:
                self._previous = ('set_home', x_count + 1)
        elif operation == 'goto_home':
            self._previous = ('goto_home', time.time())
            ret = requests.put(self._url + '/homeposition/goto', auth=self._auth)
        else:
            pan, tilt = operation
            self._previous = operation
            ret = requests.put(self._url + '/continuous', data=self._ptz_xml.format(**dict(pan=pan, tilt=tilt)), auth=self._auth)
        return ret

    def add(self, pilot, teleop):
        try:
            if pilot and pilot.get('driver') == 'driver_mode.teleop.direct' and teleop:
                self._queue.put_nowait(teleop)
        except Queue.Full:
            pass

    def quit(self):
        self._quit_event.set()

    def run(self):
        while not self._quit_event.is_set():
            try:
                cmd = self._queue.get(block=True, timeout=0.050)
                with self._lock:
                    operation = (0, 0)
                    if cmd.get('button_x', 0):
                        operation = ('set_home', cmd.get('button_a', 0))
                    elif any([cmd.get(k, 0) for k in ('button_y', 'button_a')]):
                        operation = 'goto_home'
                    elif 'pan' in cmd and 'tilt' in cmd:
                        operation = (self._norm(cmd.get('pan')) * self._flip[0], self._norm(cmd.get('tilt')) * self._flip[1])
                    self._perform(operation)
            except Queue.Empty:
                pass


class PTZCamera(object):
    def __init__(self, **kwargs):
        self._errors = []
        self._hash = -1
        self._camera = None
        self._lock = threading.Lock()
        self.restart(**kwargs)

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def restart(self, **kwargs):
        with self._lock:
            _hash = hash_dict(**kwargs)
            if _hash != self._hash:
                self._hash = _hash
                self._start(**kwargs)

    def get_errors(self):
        with self._lock:
            return self._errors

    def _start(self, **kwargs):
        errors = []
        ptz_enabled = parse_option('camera.ptz.enabled', (lambda x: bool(int(x))), False, errors, **kwargs)
        if ptz_enabled:
            _server = parse_option('camera.ip', str, errors=errors, **kwargs)
            _user = parse_option('camera.user', str, errors=errors, **kwargs)
            _password = parse_option('camera.password', str, errors=errors, **kwargs)
            _protocol = parse_option('camera.ptz.protocol', str, errors=errors, **kwargs)
            _path = parse_option('camera.ptz.path', str, errors=errors, **kwargs)
            _flip = parse_option('camera.ptz.flip', str, errors=errors, **kwargs)
            _speed = parse_option('camera.ptz.speed', float, 1.0, errors=errors, **kwargs)
            _flipcode = [1, 1]
            if _flip in ('pan', 'tilt', 'both'):
                _flipcode[0] = -1 if _flip in ('pan', 'both') else 1
                _flipcode[1] = -1 if _flip in ('tilt', 'both') else 1
            _port = 80 if _protocol == 'http' else 443
            _url = '{protocol}://{server}:{port}{path}'.format(**dict(protocol=_protocol, server=_server, port=_port, path=_path))
            logger.info("PTZ camera url={}.".format(_url))
            if self._camera is None:
                self._camera = CameraPtzThread(_url, _user, _password, speed=_speed, flip=_flipcode)
                self._camera.start()
            elif len(errors) == 0:
                self._camera.set_url(_url)
                self._camera.set_auth(_user, _password)
                self._camera.set_speed(_speed)
                self._camera.set_flip(_flipcode)
        # Already under lock.
        elif self._camera:
            self._camera.quit()
        self._errors = errors

    def add(self, pilot, teleop):
        with self._lock:
            if self._camera:
                self._camera.add(pilot, teleop)

    def quit(self):
        with self._lock:
            if self._camera:
                self._camera.quit()
                self._camera.join()
                self._camera = None


class GstSource(object):
    def __init__(self, image_publisher, **kwargs):
        self._errors = []
        self._image_publisher = image_publisher
        self._camera_shape = None
        self._flipcode = None
        self._hash = -1
        self._source = None
        self._lock = threading.Lock()
        self.restart(**kwargs)

    def get_errors(self):
        return self._errors

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def restart(self, **kwargs):
        with self._lock:
            _hash = hash_dict(**kwargs)
            if _hash != self._hash:
                self._hash = _hash
                self._start(**kwargs)

    def _publish(self, _b):
        if self._camera_shape is not None:
            _img = np.fromstring(_b.extract_dup(0, _b.get_size()), dtype=np.uint8).reshape(self._camera_shape)
            self._image_publisher.publish(cv2.flip(_img, self._flipcode) if self._flipcode is not None else _img)

    def _start(self, **kwargs):
        _errors = []
        _server = parse_option('camera.ip', str, errors=_errors, **kwargs)
        _user = parse_option('camera.user', str, errors=_errors, **kwargs)
        _password = parse_option('camera.password', str, errors=_errors, **kwargs)
        _rtsp_port = parse_option('camera.rtsp.port', int, 0, errors=_errors, **kwargs)
        _rtsp_path = parse_option('camera.image.path', str, errors=_errors, **kwargs)
        _img_wh = parse_option('camera.image.shape', str, errors=_errors, **kwargs)
        _img_flip = parse_option('camera.image.flip', str, errors=_errors, **kwargs)
        _shape = [int(x) for x in _img_wh.split('x')]
        _shape = (_shape[1], _shape[0], 3)
        _rtsp_url = 'rtsp://{user}:{password}@{ip}:{port}{path}'.format(
            **dict(user=_user, password=_password, ip=_server, port=_rtsp_port, path=_rtsp_path)
        )
        _url = "rtspsrc " \
               "location={url} " \
               "latency=0 drop-on-latency=true ! queue ! " \
               "rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! " \
               "videoscale ! video/x-raw,width={width},height={height},format=BGR ! queue". \
            format(**dict(url=_rtsp_url, height=_shape[0], width=_shape[1]))
        self._camera_shape = _shape
        # flipcode = 0: flip vertically
        # flipcode > 0: flip horizontally
        # flipcode < 0: flip vertically and horizontally
        self._flipcode = None
        if _img_flip in ('both', 'vertical', 'horizontal'):
            self._flipcode = 0 if _img_flip == 'vertical' else 1 if _img_flip == 'horizontal' else -1

        self._errors = _errors
        if len(_errors) == 0:
            # Do not use our method - already under lock.
            if self._source:
                self._source.close()
            logger.info("Camera rtsp url = {}.".format(_rtsp_url))
            logger.info("Using image={} and flipcode={}".format(self._camera_shape, self._flipcode))
            self._source = GstRawSource(fn_callback=self._publish, command=_url)

    def check(self):
        with self._lock:
            if self._source:
                self._source.check()

    def quit(self):
        with self._lock:
            if self._source:
                self._source.close()
