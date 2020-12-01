import Queue
import collections
import logging
import multiprocessing
import threading
import time

import numpy as np
import requests
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from requests.auth import HTTPDigestAuth

from byodr.utils import Configurable
from byodr.utils.option import parse_option
from byodr.utils.video import GstRawSource

logger = logging.getLogger(__name__)

CH_NONE, CH_THROTTLE, CH_STEERING, CH_BOTH = (0, 1, 2, 3)
CTL_LAST = 0


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
            if x_count >= 40 and operation[1]:
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

    def add(self, command):
        try:
            self._queue.put_nowait(command)
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
                    if cmd.get('set_home', 0):
                        operation = ('set_home', cmd.get('goto_home', 0))
                    elif cmd.get('goto_home', 0):
                        operation = 'goto_home'
                    elif 'pan' in cmd and 'tilt' in cmd:
                        operation = (self._norm(cmd.get('pan')) * self._flip[0], self._norm(cmd.get('tilt')) * self._flip[1])
                    self._perform(operation)
            except Queue.Empty:
                pass


class PTZCamera(Configurable):
    def __init__(self, position):
        super(PTZCamera, self).__init__()
        self._position = position
        self._worker = None

    def add(self, command):
        with self._lock:
            if self._worker and command:
                self._worker.add(command)

    def internal_quit(self, restarting=False):
        if self._worker:
            self._worker.quit()
            self._worker.join()
            self._worker = None

    def internal_start(self, **kwargs):
        errors = []
        cam_enabled = parse_option(self._position + '.camera.enabled', (lambda x: bool(int(x))), False, errors, **kwargs)
        ptz_enabled = parse_option(self._position + '.camera.ptz.enabled', (lambda x: bool(int(x))), False, errors, **kwargs)
        if cam_enabled and ptz_enabled:
            _server = parse_option(self._position + '.camera.ip', str, errors=errors, **kwargs)
            _user = parse_option(self._position + '.camera.user', str, errors=errors, **kwargs)
            _password = parse_option(self._position + '.camera.password', str, errors=errors, **kwargs)
            _protocol = parse_option(self._position + '.camera.ptz.protocol', str, errors=errors, **kwargs)
            _path = parse_option(self._position + '.camera.ptz.path', str, errors=errors, **kwargs)
            _flip = parse_option(self._position + '.camera.ptz.flip', str, errors=errors, **kwargs)
            _speed = parse_option(self._position + '.camera.ptz.speed', float, 1.0, errors=errors, **kwargs)
            _flipcode = [1, 1]
            if _flip in ('pan', 'tilt', 'both'):
                _flipcode[0] = -1 if _flip in ('pan', 'both') else 1
                _flipcode[1] = -1 if _flip in ('tilt', 'both') else 1
            _port = 80 if _protocol == 'http' else 443
            _url = '{protocol}://{server}:{port}{path}'.format(**dict(protocol=_protocol, server=_server, port=_port, path=_path))
            logger.info("PTZ camera url={}.".format(_url))
            if self._worker is None:
                self._worker = CameraPtzThread(_url, _user, _password, speed=_speed, flip=_flipcode)
                self._worker.start()
            elif len(errors) == 0:
                self._worker.set_url(_url)
                self._worker.set_auth(_user, _password)
                self._worker.set_speed(_speed)
                self._worker.set_flip(_flipcode)
        # Already under lock.
        elif self._worker:
            self._worker.quit()
        return errors


class GstSource(Configurable):
    def __init__(self, position, image_publisher):
        super(GstSource, self).__init__()
        self._position = position
        self._image_publisher = image_publisher
        self._camera_shape = None
        self._source = None
        self._enabled = False

    def _publish(self, _b):
        if self._camera_shape is not None:
            self._image_publisher.publish(np.fromstring(_b.extract_dup(0, _b.get_size()), dtype=np.uint8).reshape(self._camera_shape))

    def is_enabled(self):
        return self._enabled

    def check(self):
        with self._lock:
            if self._source:
                self._source.check()

    def internal_quit(self, restarting=False):
        if self._source:
            self._source.close()

    def internal_start(self, **kwargs):
        _errors = []
        _server = parse_option(self._position + '.camera.ip', str, errors=_errors, **kwargs)
        _user = parse_option(self._position + '.camera.user', str, errors=_errors, **kwargs)
        _password = parse_option(self._position + '.camera.password', str, errors=_errors, **kwargs)
        _decode_rate = parse_option(self._position + '.camera.decode.rate', int, 20, errors=_errors, **kwargs)
        _rtsp_port = parse_option(self._position + '.camera.rtsp.port', int, 0, errors=_errors, **kwargs)
        _rtsp_path = parse_option(self._position + '.camera.image.path', str, errors=_errors, **kwargs)
        _img_wh = parse_option(self._position + '.camera.image.shape', str, errors=_errors, **kwargs)
        _shape = [int(x) for x in _img_wh.split('x')]
        _shape = (_shape[1], _shape[0], 3)
        self._camera_shape = _shape
        # Do not use our method - already under lock.
        if self._source:
            self._source.close()
        # The enabled property may change between restarts.
        _enabled = parse_option(self._position + '.camera.enabled', (lambda v: bool(int(v))), False, _errors, **kwargs)
        self._enabled = _enabled
        if _enabled and len(_errors) == 0:
            _rtsp_url = 'rtsp://{user}:{password}@{ip}:{port}{path}'.format(
                **dict(user=_user, password=_password, ip=_server, port=_rtsp_port, path=_rtsp_path)
            )
            _url = "rtspsrc " \
                   "location={url} " \
                   "latency=0 drop-on-latency=true do-retransmission=false ! queue ! " \
                   "rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! " \
                   "videorate ! video/x-raw,framerate={framerate}/1 ! " \
                   "videoscale ! video/x-raw,width={width},height={height},format=BGR ! queue". \
                format(**dict(url=_rtsp_url, height=_shape[0], width=_shape[1], framerate=_decode_rate))
            logger.info("Camera stream url = {} image shape = {} decode rate = {}.".format(_rtsp_url, self._camera_shape, _decode_rate))
            self._source = GstRawSource(name=self._position, fn_callback=self._publish, command=_url)
        return _errors


class GpsPollerThread(threading.Thread):
    """
    https://wiki.teltonika-networks.com/view/RUT955_Modbus
    https://wiki.teltonika-networks.com/view/RUT955_GPS_Protocols
    https://community.victronenergy.com/questions/40965/wheres-my-boat-answers-rather-than-questions-gps-m.html
    https://pymodbus.readthedocs.io/en/v1.3.2/examples/modbus-payload.html
    """

    def __init__(self, host='192.168.1.1', port='502'):
        super(GpsPollerThread, self).__init__()
        self._host = host
        self._port = port
        self._quit_event = multiprocessing.Event()
        self._queue = collections.deque(maxlen=1)

    def quit(self):
        self._quit_event.set()

    def get_latitude(self, default=0):
        return self._queue[0][0] if len(self._queue) > 0 else default

    def get_longitude(self, default=0):
        return self._queue[0][1] if len(self._queue) > 0 else default

    def run(self):
        while not self._quit_event.is_set():
            try:
                client = ModbusTcpClient(self._host, port=self._port)
                client.connect()
                response = client.read_holding_registers(143, 4, unit=1)
                if hasattr(response, 'registers'):
                    decoder = BinaryPayloadDecoder.fromRegisters(response.registers, Endian.Big)
                    latitude = decoder.decode_32bit_float()
                    longitude = decoder.decode_32bit_float()
                    self._queue.appendleft((latitude, longitude))
                    time.sleep(.100)
                else:
                    time.sleep(10)
            except Exception as e:
                logger.warning(e)
                time.sleep(10)
