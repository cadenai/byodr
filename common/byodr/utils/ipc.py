import collections
import json
import threading

import numpy as np
import zmq

from byodr.utils import timestamp


class JSONPublisher(object):
    def __init__(self, url, topic=''):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind(url)
        self._publisher = publisher
        self._topic = topic

    def publish(self, data):
        self._publisher.send('{}:{}'.format(self._topic, json.dumps(data)), zmq.NOBLOCK)


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
