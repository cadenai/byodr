import collections
import json
import threading
from abc import abstractmethod

import numpy as np
import zmq

from byodr.utils import timestamp


class JSONPublisher(object):
    def __init__(self, url, topic=''):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind(url)
        self._publisher = publisher
        self._topic = topic

    def publish(self, data, topic=None):
        _topic = self._topic if topic is None else topic
        self._publisher.send('{}:{}'.format(_topic, json.dumps(data)), zmq.NOBLOCK)


class ImagePublisher(object):
    def __init__(self, url, topic=''):
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
    def __init__(self, url, event, topic=b'', receive_timeout_ms=1):
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

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def pop_latest(self):
        return self._queue.popleft() if bool(self._queue) else None

    def run(self):
        while not self._quit_event.is_set():
            try:
                self._queue.appendleft(json.loads(self._subscriber.recv().split(':', 1)[1]))
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

    @abstractmethod
    def serve(self, request):
        raise NotImplementedError()

    def run(self):
        while not self._quit_event.is_set():
            try:
                message = json.loads(self._server.recv())
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
        raise NotImplementedError()

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
        socket = zmq.Context().socket(zmq.REQ)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt(zmq.RCVTIMEO, receive_timeout_ms)
        socket.setsockopt(zmq.LINGER, 0)
        locations = urls if isinstance(urls, list) else [urls]
        [socket.connect(location) for location in locations]
        self._num_locations = len(locations)
        self._socket = socket

    def call(self, message):
        ret = {}
        for _ in range(self._num_locations):
            self._socket.send(json.dumps(message), zmq.NOBLOCK)
            try:
                ret.update(json.loads(self._socket.recv()))
            except (zmq.Again, zmq.ZMQError):
                pass
        return ret
