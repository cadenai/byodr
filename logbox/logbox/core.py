import collections
import os

import cv2
import numpy as np
from bson.binary import Binary
from bson.objectid import ObjectId

from byodr.utils import timestamp

TRIGGER_SERVICE_START = 2 ** 0
TRIGGER_SERVICE_END = 2 ** 1
TRIGGER_SERVICE_STEP = 2 ** 2
TRIGGER_PHOTO_SNAPSHOT = 2 ** 3


def get_timestamp(c, default=-1):
    return default if c is None else c.get('time')


def jpeg_encode(image, quality=95):
    """Higher quality leads to slightly more cpu load. Default cv jpeg quality is 95. """
    return cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])[1]


def cv2_image_from_bytes(b):
    return cv2.imdecode(np.frombuffer(memoryview(b), dtype=np.uint8), cv2.IMREAD_COLOR)


def prepare_image_persist(image, persist=True):
    _shape, _num_bytes, _buffer = -1, -1, -1
    if persist and image is not None:
        if image.shape != (240, 320, 3):
            image = cv2.resize(image, (320, 240))
        _shape = image.shape
        _img_bytes = jpeg_encode(image).tobytes()
        _num_bytes = len(_img_bytes)
        _buffer = Binary(_img_bytes)
    return _shape, _num_bytes, _buffer


def get_or_create_directory(directory, mode=0o775):
    if not os.path.exists(directory):
        _mask = os.umask(000)
        os.makedirs(directory, mode=mode)
        os.umask(_mask)
    return directory


class SharedUser(object):
    def __init__(self):
        self._user_busy = collections.deque(maxlen=1)

    def touch(self):
        self._user_busy.append(timestamp())

    def get(self):
        return self._user_busy[0] if len(self._user_busy) > 0 else 0

    def is_busy(self, wait_ms=3e4):
        return timestamp() - self.get() < (wait_ms * 1e3)


class SharedState(object):
    def __init__(self, channels, hz=20):
        self._hz = hz
        self._patience = (1e6 / hz)
        # The pilot channel is set to collect and pop the others cache the most recent message.
        self._camera, self._pilot, self._vehicle, self._inference = channels
        self._pil_msg = None

    def _expire(self, cmd, stamp):
        return None if cmd is None or abs(stamp - get_timestamp(cmd)) > self._patience else cmd

    def get_hz(self):
        return self._hz

    def pull(self):
        # The channels store the most recent message at the start of a list.
        image_md, image = self._camera.capture()
        # Delete expired information.
        _time = timestamp()
        image_md = self._expire(image_md, _time)
        image = None if image_md is None else image
        p_msgs = self._pilot()
        pil = self._expire(p_msgs[0] if p_msgs is not None and len(p_msgs) > 0 else self._pil_msg, _time)
        self._pil_msg = pil
        veh = self._expire(self._vehicle(), _time)
        inf = self._expire(self._inference(), _time)
        return _time, pil, veh, inf, image_md, image, p_msgs


class MongoLogBox(object):
    def __init__(self, client):
        self._client = client
        self._database = client.logbox

    def close(self):
        self._client.close()

    def load_event_image_fields(self, object_id):
        # Images are stored as jpeg encoded bytes.
        event = self._database.events.find_one({'_id': ObjectId(object_id)})
        return None if event is None else event.get('img_shape'), event.get('img_num_bytes'), event.get('img_buffer')

    def paginate_events(self, load_image=False, **kwargs):
        _filter = {}
        _projection = {} if load_image else {'img_buffer': False}
        start = kwargs.get('start', 0)
        length = kwargs.get('length', 10)
        time_order = kwargs.get('order', -1)
        cursor = self._database.events.find(
            filter=_filter,
            projection=_projection,
            sort=[('time', time_order)],
            batch_size=length,
            limit=length,
            skip=start
        )
        return cursor.collection.count_documents(_filter), cursor

    def insert_event(self, document):
        return self._database.events.insert_one(document)

    def update_event(self, query, update):
        return self._database.events.update_one(query, update)

    def list_next_batch_of_non_packaged_save_events(self, batch_size=1000):
        # The event must have an associated image.
        _filter = {'pil_is_save_event': 1, 'lb_is_packaged': 0, 'img_num_bytes': {'$gt': 0}}
        cursor = self._database.events.find(filter=_filter, sort=[('time', -1)], batch_size=batch_size, limit=batch_size)
        return list(cursor)

    def list_all_non_packaged_photo_events(self):
        _filter = {'trigger': TRIGGER_PHOTO_SNAPSHOT, 'lb_is_packaged': 0, 'img_num_bytes': {'$gt': 0}}
        cursor = self._database.events.find(filter=_filter)
        return list(cursor)
