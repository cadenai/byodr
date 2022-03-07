from __future__ import absolute_import

import collections
import datetime
import json
import logging
import multiprocessing
import os
import sys
import threading
import time

import numpy as np
import zmq

from byodr.utils import timestamp

if sys.version_info > (3,):
    # noinspection PyShadowingBuiltins
    buffer = memoryview


    def receive_string(subscriber):
        return subscriber.recv_string()


    def send_string(sender, val, flags=0):
        return sender.send_string(val, flags)
else:
    def receive_string(subscriber):
        return subscriber.recv()


    def send_string(sender, val, flags=0):
        return sender.send(val, flags)

logger = logging.getLogger(__name__)


class JSONPublisher(object):
    def __init__(self, url, topic='', hwm=1, clean_start=True):
        if clean_start and url.startswith('ipc://') and os.path.exists(url[6:]):
            os.remove(url[6:])
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.set_hwm(hwm)
        publisher.bind(url)
        self._publisher = publisher
        self._topic = topic

    def publish(self, data, topic=None):
        _topic = self._topic if topic is None else topic
        if data is not None:
            data = dict((k, v) for k, v in data.items() if v is not None)
            send_string(self._publisher, '{}:{}'.format(_topic, json.dumps(data)), zmq.NOBLOCK)


class ImagePublisher(object):
    def __init__(self, url, topic='', hwm=1, clean_start=True):
        if clean_start and url.startswith('ipc://') and os.path.exists(url[6:]):
            os.remove(url[6:])
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.set_hwm(hwm)
        publisher.bind(url)
        self._publisher = publisher
        self._topic = topic

    def publish(self, _img, topic=None):
        _topic = self._topic if topic is None else topic
        self._publisher.send_multipart([_topic,
                                        json.dumps(dict(time=timestamp(), shape=_img.shape)),
                                        np.ascontiguousarray(_img, dtype=np.uint8)],
                                       flags=zmq.NOBLOCK)


class JSONReceiver(object):
    def __init__(self, url, topic=b'', hwm=1, receive_timeout_ms=2, pop=False):
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.set_hwm(hwm)
        subscriber.setsockopt(zmq.RCVTIMEO, receive_timeout_ms)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect(url)
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        self._pop = pop
        self._unpack = hwm == 1
        self._subscriber = subscriber
        self._lock = threading.Lock()
        self._queue = collections.deque(maxlen=hwm)

    def consume(self):
        with self._lock:
            try:
                # Does not replace local queue messages when none are available.
                self._queue.appendleft(json.loads(receive_string(self._subscriber).split(':', 1)[1]))
            except zmq.Again:
                pass

    def get(self):
        if self._unpack and not self._pop:
            return self._queue[0] if self._queue else None
        _view = list(self._queue) if self._queue else None
        if self._pop:
            self._queue.clear()
        return _view


class CollectorThread(threading.Thread):
    def __init__(self, receivers, event=None, hz=1000):
        super(CollectorThread, self).__init__()
        _list = (isinstance(receivers, tuple) or isinstance(receivers, list))
        self._receivers = receivers if _list else [receivers]
        self._quit_event = multiprocessing.Event() if event is None else event
        self._sleep = 1. / hz

    def get(self, index=0):
        # Get the latest message without blocking.
        # _receiver.consume() -- blocks; perform at thread.run()
        return self._receivers[index].get()

    def quit(self):
        self._quit_event.set()

    def run(self):
        while not self._quit_event.is_set():
            # Empty the receiver queues to not block upstream senders.
            list(map(lambda receiver: receiver.consume(), self._receivers))
            time.sleep(self._sleep)


def json_collector(url, topic, event, receive_timeout_ms=1000, hwm=1, pop=False):
    return CollectorThread(JSONReceiver(url, topic, hwm=hwm, receive_timeout_ms=receive_timeout_ms, pop=pop), event=event)


class ReceiverThread(threading.Thread):
    def __init__(self, url, event=None, topic=b'', hwm=1, receive_timeout_ms=1):
        super(ReceiverThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.set_hwm(hwm)
        subscriber.setsockopt(zmq.RCVTIMEO, receive_timeout_ms)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect(url)
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        self._subscriber = subscriber
        self._quit_event = multiprocessing.Event() if event is None else event
        self._queue = collections.deque(maxlen=1)
        self._listeners = []

    def add_listener(self, c):
        self._listeners.append(c)

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def pop_latest(self):
        return self._queue.popleft() if bool(self._queue) else None

    def quit(self):
        self._quit_event.set()

    def run(self):
        while not self._quit_event.is_set():
            try:
                _latest = json.loads(receive_string(self._subscriber).split(':', 1)[1])
                self._queue.appendleft(_latest)
                list(map(lambda x: x(_latest), self._listeners))
            except zmq.Again:
                pass


class CameraThread(threading.Thread):
    def __init__(self, url, event, topic=b'', hwm=1, receive_timeout_ms=25):
        super(CameraThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.set_hwm(hwm)
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
    def __init__(self, url, event, hwm=1, receive_timeout_ms=50):
        super(JSONServerThread, self).__init__()
        server = zmq.Context().socket(zmq.REP)
        server.set_hwm(hwm)
        server.setsockopt(zmq.RCVTIMEO, receive_timeout_ms)
        server.setsockopt(zmq.LINGER, 0)
        server.bind(url)
        self._server = server
        self._quit_event = event
        self._queue = collections.deque(maxlen=1)
        self._listeners = []

    def add_listener(self, c):
        self._listeners.append(c)

    def on_message(self, message):
        self._queue.appendleft(message)
        list(map(lambda x: x(message), self._listeners))

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def pop_latest(self):
        return self._queue.popleft() if bool(self._queue) else None

    def serve(self, request):
        return {}

    def run(self):
        while not self._quit_event.is_set():
            try:
                message = json.loads(receive_string(self._server))
                self.on_message(message)
                send_string(self._server, json.dumps(self.serve(message)))
            except zmq.Again:
                pass


class LocalIPCServer(JSONServerThread):
    def __init__(self, name, url, event, receive_timeout_ms=50):
        super(LocalIPCServer, self).__init__(url, event, receive_timeout_ms)
        self._name = name
        self._m_startup = collections.deque(maxlen=1)
        self._m_capabilities = collections.deque(maxlen=1)

    def register_start(self, errors, capabilities=None):
        capabilities = {} if capabilities is None else capabilities
        self._m_startup.append((datetime.datetime.utcnow().strftime('%b %d %H:%M:%S.%s UTC'), errors))
        self._m_capabilities.append(capabilities)

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
            elif message.get('request') == 'system/service/capabilities' and self._m_capabilities:
                return {self._name: self._m_capabilities[-1]}
        except IndexError:
            pass
        return {}


class JSONZmqClient(object):
    def __init__(self, urls, hwm=1, receive_timeout_ms=200):
        self._urls = urls if isinstance(urls, list) else [urls]
        self._receive_timeout = receive_timeout_ms
        self._context = None
        self._socket = None
        self._hwm = hwm
        self._create(self._urls)

    def _create(self, locations):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.set_hwm(self._hwm)
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
                send_string(self._socket, json.dumps(message), zmq.NOBLOCK)
                ret.update(json.loads(receive_string(self._socket)))
            except zmq.ZMQError:
                j = i + 1
                self._create(self._urls[j:] + self._urls[:j])
        return ret
