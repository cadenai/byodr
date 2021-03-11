from __future__ import absolute_import
import collections
from six.moves import map


class QueueReceiver(object):
    def __init__(self, queue_max_size=100):
        """
        A drop-in replacement for ipc ReceiverThread.
        :param queue_max_size: Max length of the queue.
        """
        self._queue = collections.deque(maxlen=queue_max_size)
        self._listeners = []
        self._started = False

    def start(self):
        self._started = True

    def is_started(self):
        return self._started

    def add_listener(self, c):
        self._listeners.append(c)

    def add(self, m):
        self._queue.appendleft(m)
        list(map(lambda x: x(m), self._listeners))

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def pop_latest(self):
        return self._queue.popleft() if bool(self._queue) else None

    def clear(self):
        self._queue.clear()

    def quit(self):
        self.clear()
        self._listeners = []
        self._started = False


class QueueCamera(object):
    def __init__(self, queue_max_size=100):
        """
        A drop-in replacement for ipc CameraThread.
        :param queue_max_size: Max length of the queue.
        """
        self._queue = collections.deque(maxlen=queue_max_size)
        self._started = False

    def start(self):
        self._started = True

    def is_started(self):
        return self._started

    def add(self, meta_data, image):
        self._queue.appendleft((meta_data, image))

    def capture(self):
        return self._queue[0] if bool(self._queue) else (None, None)

    def clear(self):
        self._queue.clear()


class CollectPublisher(object):
    def __init__(self, topic=''):
        """
        A drop-in replacement for ipc JSONPublisher.
        :param topic: The default topic.
        """
        self._topic = topic
        self._map = dict()

    def publish(self, data, topic=None):
        _topic = self._topic if topic is None else topic
        if _topic not in self._map:
            self._map[_topic] = list()
        self._map[_topic].append(data)

    def collect(self, topic=None):
        _topic = self._topic if topic is None else topic
        return self._map.get(_topic)

    def get_latest(self, topic=None):
        return self.collect(topic=topic)[-1]

    def clear(self):
        self._map.clear()


class CollectServer(object):
    def __init__(self):
        """
        A drop-in replacement for ipc LocalIPCServer.
        """
        self._errors = []
        self._capabilities = []

    def register_start(self, errors, capabilities=None):
        capabilities = {} if capabilities is None else capabilities
        self._errors.append(errors)
        self._capabilities.append(capabilities)

    def collect(self):
        return self._errors

    def get_latest(self):
        return self._errors[-1]

    def clear(self):
        self._errors = []


class CollectJSONClient(object):
    """
    A drop-in replacement for ipc JSONZmqClient.
    """

    def __init__(self):
        self._list = []

    def call(self, message, ret=None):
        self._list.append(message)
        return ret

    def collect(self):
        return self._list

    def get_latest(self):
        return self._list[-1]

    def clear(self):
        self._list = []

    def quit(self):
        self.clear()
