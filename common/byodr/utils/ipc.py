import collections
import json
import logging
import os

import numpy as np
import threading
import zmq

from byodr.utils import timestamp

logger = logging.getLogger(__name__)


class JSONPublisher(object):
    def __init__(self, url, topic='', clean_start=True):
        if clean_start and url.startswith('ipc://') and os.path.exists(url[6:]):
            os.remove(url[6:])
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind(url)
        self._publisher = publisher
        self._topic = topic

    def publish(self, data, topic=None):
        _topic = self._topic if topic is None else topic
        self._publisher.send('{}:{}'.format(_topic, json.dumps(data)), zmq.NOBLOCK)


class ImagePublisher(object):
    def __init__(self, url, topic='', clean_start=True):
        if clean_start and url.startswith('ipc://') and os.path.exists(url[6:]):
            os.remove(url[6:])
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind(url)
        self._publisher = publisher
        self._topic = topic

    def publish(self, _img):
        self._publisher.send_multipart([self._topic,
                                        json.dumps(dict(time=timestamp(), shape=_img.shape)),
                                        np.ascontiguousarray(_img, dtype=np.uint8)],
                                       flags=zmq.NOBLOCK)


class ReceiverThread(threading.Thread):
    def __init__(self, url, event, topic=b'', receive_timeout_ms=1, on_message=(lambda m: m)):
        super(ReceiverThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, receive_timeout_ms)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect(url)
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        self._subscriber = subscriber
        self._quit_event = event
        self._queue = collections.deque(maxlen=1)
        self._on_message = on_message

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def pop_latest(self):
        return self._queue.popleft() if bool(self._queue) else None

    def run(self):
        while not self._quit_event.is_set():
            try:
                _latest = json.loads(self._subscriber.recv().split(':', 1)[1])
                self._queue.appendleft(_latest)
                self._on_message(_latest)
            except zmq.Again:
                pass


class CameraThread(threading.Thread):
    def __init__(self, url, event, topic=b'', receive_timeout_ms=1):
        super(CameraThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, receive_timeout_ms)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect(url)
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        self._subscriber = subscriber
        self._quit_event = event
        self._images = collections.deque(maxlen=1)

    def capture(self):
        return self._images[0] if bool(self._images) else (None, None)

    def run(self):
        while not self._quit_event.is_set():
            try:
                [_, md, data] = self._subscriber.recv_multipart()
                md = json.loads(md)
                height, width, channels = md['shape']
                img = np.frombuffer(buffer(data), dtype=np.uint8)
                img = img.reshape((height, width, channels))
                self._images.appendleft((md, img))
            except ValueError as e:
                logger.warning(e)
            except zmq.Again:
                pass


class JSONServerThread(threading.Thread):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(JSONServerThread, self).__init__()
        server = zmq.Context().socket(zmq.REP)
        server.setsockopt(zmq.RCVHWM, 1)
        server.setsockopt(zmq.RCVTIMEO, receive_timeout_ms)
        server.setsockopt(zmq.LINGER, 0)
        server.bind(url)
        self._server = server
        self._quit_event = event
        self._queue = collections.deque(maxlen=1)

    def on_message(self, message):
        self._queue.appendleft(message)

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def pop_latest(self):
        return self._queue.popleft() if bool(self._queue) else None

    def serve(self, request):
        return {}

    def run(self):
        while not self._quit_event.is_set():
            try:
                message = json.loads(self._server.recv())
                self.on_message(message)
                self._server.send(json.dumps(self.serve(message)))
            except zmq.Again:
                pass


class LocalIPCServer(JSONServerThread):
    def __init__(self, name, url, event, receive_timeout_ms=50):
        super(LocalIPCServer, self).__init__(url, event, receive_timeout_ms)
        self._name = name
        self._m_startup = collections.deque(maxlen=1)

    def register_start(self, errors):
        self._m_startup.append((timestamp(), errors))

    def serve_local(self, message):
        return {}

    def serve(self, message):
        try:
            if message.get('request') == 'system/startup/list' and self._m_startup:
                ts, errors = self._m_startup[-1]
                messages = ['No errors']
                if errors:
                    d_errors = dict()  # Merge to obtain distinct keys.
                    [d_errors.update({error.key: error.message}) for error in errors]
                    messages = ['{} - {}'.format(k, d_errors[k]) for k in d_errors.keys()]
                return {self._name: {ts: messages}}
        except IndexError:
            pass
        return self.serve_local(message)


class JSONZmqClient(object):
    def __init__(self, urls, receive_timeout_ms=200):
        self._urls = urls if isinstance(urls, list) else [urls]
        self._receive_timeout = receive_timeout_ms
        self._context = None
        self._socket = None
        self._create(self._urls)

    def _create(self, locations):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt(zmq.RCVTIMEO, self._receive_timeout)
        socket.setsockopt(zmq.LINGER, 0)
        [socket.connect(location) for location in locations]
        self._context = context
        self._socket = socket

    def quit(self):
        if self._context is not None:
            self._context.destroy()

    def call(self, message):
        ret = {}
        for i in range(len(self._urls)):
            try:
                self._socket.send(json.dumps(message), zmq.NOBLOCK)
                ret.update(json.loads(self._socket.recv()))
            except zmq.ZMQError:
                j = i + 1
                self._create(self._urls[j:] + self._urls[:j])
        return ret
