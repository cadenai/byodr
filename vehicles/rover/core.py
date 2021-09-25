import collections
import logging
import multiprocessing
import threading
import time

import Queue
import requests
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from requests.auth import HTTPDigestAuth

from byodr.utils import Configurable
from byodr.utils.option import parse_option
from byodr.utils.video import create_image_source

logger = logging.getLogger(__name__)

CH_NONE, CH_THROTTLE, CH_STEERING, CH_BOTH = (0, 1, 2, 3)
CTL_LAST = 0

gst_commands = {
    'h264/rtsp':
        "rtspsrc location=rtsp://{user}:{password}@{ip}:{port}{path} latency=0 drop-on-latency=true do-retransmission=false ! "
        "queue ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! videorate ! videoscale ! "
        "video/x-raw,width={width},height={height},framerate={framerate}/1,format=BGR ! queue ! appsink",
    'h264/tcp':
        "tcpclientsrc host={ip} port={port} ! queue ! gdpdepay ! h264parse ! avdec_h264 ! videoconvert ! videorate ! videoscale ! "
        "video/x-raw,width={width},height={height},framerate={framerate}/1,format=BGR ! queue ! appsink",
    'h264/udp':
        "udpsrc port={port} ! queue ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! videorate ! videoscale ! "
        "video/x-raw,width={width},height={height},framerate={framerate}/1,format=BGR ! queue ! appsink"
}


class ConfigurableImageGstSource(Configurable):
    def __init__(self, name, image_publisher):
        super(ConfigurableImageGstSource, self).__init__()
        self._name = name
        self._image_publisher = image_publisher
        self._sink = None
        self._shape = None
        self._ptz = None

    def _close(self):
        if self._sink is not None:
            self._sink.close()
            try:
                self._sink.remove_listener(self._publish)
            except ValueError:
                # ValueError: deque.remove(x): x not in deque
                pass

    def _publish(self, image):
        self._image_publisher.publish(image)

    def get_shape(self):
        return self._shape

    def get_ptz(self):
        return self._ptz

    def check(self):
        with self._lock:
            if self._sink is not None:
                self._sink.check()

    def internal_quit(self, restarting=False):
        self._close()

    def internal_start(self, **kwargs):
        self._close()
        _errors = []
        _type = parse_option(self._name + '.camera.type', str, 'h264/rtsp', errors=_errors, **kwargs)
        assert _type in gst_commands.keys(), "Unrecognized camera type '{}'.".format(_type)
        framerate = (parse_option(self._name + '.camera.framerate', int, 25, errors=_errors, **kwargs))
        out_width, out_height = [int(x) for x in parse_option(self._name + '.camera.shape', str, '320x240',
                                                              errors=_errors, **kwargs).split('x')]
        if _type == 'h264/rtsp':
            config = {
                'ip': (parse_option(self._name + '.camera.ip', str, '192.168.1.64', errors=_errors, **kwargs)),
                'port': (parse_option(self._name + '.camera.port', int, 554, errors=_errors, **kwargs)),
                'user': (parse_option(self._name + '.camera.user', str, 'user', errors=_errors, **kwargs)),
                'password': (parse_option(self._name + '.camera.password', str, 'pass', errors=_errors, **kwargs)),
                'path': (parse_option(self._name + '.camera.path', str, '/Streaming/Channels/102', errors=_errors, **kwargs)),
                'height': out_height,
                'width': out_width,
                'framerate': framerate
            }
        else:
            _type = 'h264/udp'
            config = {
                'port': (parse_option(self._name + '.camera.port', int, 5000, errors=_errors, **kwargs)),
                'height': out_height,
                'width': out_width,
                'framerate': framerate
            }
        self._shape = (out_height, out_width, 3)
        self._ptz = parse_option(self._name + '.camera.ptz.enabled', int, 1, errors=_errors, **kwargs)
        _command = gst_commands.get(_type).format(**config)
        self._sink = create_image_source(self._name, shape=self._shape, command=_command)
        self._sink.add_listener(self._publish)
        logger.info("Gst '{}' command={}".format(self._name, _command))
        return _errors


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
                logger.warning("Got status {} on operation {}.".format(ret.status_code, operation))

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
            except IOError as e:
                # E.g. a requests ConnectionError to the ip camera.
                logger.warning("PTZ#run: {}".format(e))


class PTZCamera(Configurable):
    def __init__(self, name):
        super(PTZCamera, self).__init__()
        self._name = name
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
        ptz_enabled = parse_option(self._name + '.camera.ptz.enabled', (lambda x: bool(int(x))), True, errors, **kwargs)
        if ptz_enabled:
            _server = parse_option(self._name + '.camera.ip', str, '192.168.1.64', errors=errors, **kwargs)
            _user = parse_option(self._name + '.camera.user', str, 'user', errors=errors, **kwargs)
            _password = parse_option(self._name + '.camera.password', str, 'pass', errors=errors, **kwargs)
            _protocol = 'http'
            _path = '/ISAPI/PTZCtrl/channels/1'
            _flip = 'tilt'
            _speed = 1.9
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
