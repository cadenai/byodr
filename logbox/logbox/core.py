import collections
import os

import cv2
import numpy as np
import pymongo
from bson.binary import Binary
from bson.objectid import ObjectId

from byodr.utils import timestamp

TRIGGER_SERVICE_START = 2 ** 0
TRIGGER_SERVICE_END = 2 ** 1
TRIGGER_PHOTO_SNAPSHOT = 2 ** 2
TRIGGER_DRIVE_OPERATOR = 2 ** 3
TRIGGER_DRIVE_TRAINER = 2 ** 4


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

    def is_busy(self, wait_sec=30):
        return timestamp() - self.get() < (wait_sec * 1e6)


def _nearest(items, ts, default=None):
    r = default
    dt = abs(ts - get_timestamp(r))
    for item in ([] if items is None else items):
        delta = abs(ts - get_timestamp(item))
        if delta < dt:
            dt = delta
            r = item
    return r


class SharedState(object):
    def __init__(self, channels, hz=20):
        self._hz = hz
        self._patience = (1e6 / hz)
        self._camera, self._pilot, self._vehicle, self._inference = channels
        self._cached = (None, None, None)

    def _expire(self, cmd, stamp):
        return None if cmd is None or abs(stamp - get_timestamp(cmd)) > self._patience else cmd

    def get_hz(self):
        return self._hz

    def pull(self):
        # The non camera channels are set to collect and pop.
        image_md, image = self._camera.capture()
        # Start with the return values.
        pil, veh, inf = self._cached
        # The image is the primary event.
        _time = get_timestamp(image_md, default=timestamp())
        # Gather the messages around the primary time.
        pilots = self._pilot()
        pil = self._expire(_nearest(pilots, _time, pil), _time)
        veh = self._expire(_nearest(self._vehicle(), _time, veh), _time)
        inf = self._expire(_nearest(self._inference(), _time, inf), _time)
        self._cached = (pil, veh, inf)
        return _time, pil, veh, inf, image_md, image, pilots


# noinspection PyUnresolvedReferences
def _create_index_if_not_exists(collection, keys, name, unique=False, background=True):
    try:
        collection.create_index(keys, name=name, unique=unique, background=background)
    except pymongo.errors.OperationFailure:
        pass


class MongoLogBox(object):
    def __init__(self, client):
        self._client = client
        self._database = client.logbox

    def close(self):
        self._client.close()

    def ensure_indexes(self):
        _coll = self._database.events
        _create_index_if_not_exists(_coll, [('time', pymongo.DESCENDING)], name='idx_time_descending', unique=True)
        _create_index_if_not_exists(_coll, [('time', pymongo.DESCENDING),
                                            ('pil_is_save_event', pymongo.ASCENDING),
                                            ('lb_is_packaged', pymongo.ASCENDING),
                                            ('img_num_bytes', pymongo.ASCENDING)], name='idx_non_packaged_save_events')
        _create_index_if_not_exists(_coll, [('trigger', pymongo.ASCENDING),
                                            ('lb_is_packaged', pymongo.ASCENDING),
                                            ('img_num_bytes', pymongo.ASCENDING)], name='idx_non_packaged_save_events')

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

    # noinspection PyUnresolvedReferences
    def insert_event(self, document):
        try:
            # Silently handle the case where a timestamp already exists.
            return self._database.events.insert_one(document)
        except pymongo.errors.DuplicateKeyError:
            return False

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
