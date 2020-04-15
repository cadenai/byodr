import StringIO
import datetime
import io
import logging
import multiprocessing
import os
import uuid
import zipfile
from abc import ABCMeta, abstractmethod

import pandas as pd

logger = logging.getLogger(__name__)


def create_data_source(directory=os.getcwd()):
    assert os.path.exists(directory), "The directory '{}' does not exist.".format(directory)
    return ZipDataSource(directory=directory)


class Event(object):
    def __init__(self,
                 timestamp,
                 image,
                 steer_src,
                 speed_src,
                 command_src,
                 steering,
                 desired_speed,
                 actual_speed,
                 heading,
                 throttle,
                 command,
                 x_coordinate,
                 y_coordinate):
        self.timestamp = timestamp
        self.image = image
        self.steer_src = steer_src
        self.speed_src = speed_src
        self.command_src = command_src
        self.steering = steering
        self.desired_speed = desired_speed
        self.actual_speed = actual_speed
        self.heading = heading
        self.throttle = throttle
        self.command = command
        self.x_coordinate = x_coordinate
        self.y_coordinate = y_coordinate
        self.vehicle = None
        self.vehicle_config = None
        self.image_uri = None
        self.save_event = False

    def __str__(self):
        return "Event (time={}, steer_src={}, steering={}, command={}, uri={}).".format(
            self.timestamp, self.steer_src, self.steering, self.command, self.image_uri
        )


class AbstractDataSource(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def open(self, session=None):
        raise NotImplementedError()

    @abstractmethod
    def close(self):
        raise NotImplementedError()

    @abstractmethod
    def create_event(self, event):
        raise NotImplementedError()


class ZipDataSource(AbstractDataSource):
    MF_TEMPLATE = """zip-datasource-version: 0.5
zip-num-entries: {num_entries}
platform-uuid-node: "{uuid_node}"
image-shape-hwc: "{image_shape}"
"""

    def __init__(self, directory=os.getcwd()):
        assert os.path.exists(directory), "The directory '{}' does not exist.".format(directory)
        self._directory = directory
        self._lock = multiprocessing.Lock()
        self._running = False
        self._read_only = True
        self._session = None
        self._data = None
        self._image_shape = None

    def open(self, session=None):
        # Existing sessions can not be reopened.
        with self._lock:
            if self._running:
                return
            self._running = True
            self._read_only = session is not None
            session = datetime.datetime.today().strftime('%Y%b%dT%H%M_%S%s') if session is None else session
            self._session = session
            _filename = os.path.join(self._directory, session + '.zip')
            if self._read_only:
                assert os.path.exists(_filename), "The session file '{}' does not exist.".format(_filename)
                with zipfile.ZipFile(_filename, mode='r') as archive:
                    self._data = pd.read_csv(StringIO.StringIO(archive.read('{}.csv'.format(session))))
            else:
                self._data = pd.DataFrame(columns=['time',
                                                   'vehicle',
                                                   'vehicle_conf',
                                                   'image_uri',
                                                   'steer_src',
                                                   'speed_src',
                                                   'steering',
                                                   'desired_speed',
                                                   'actual_speed',
                                                   'heading',
                                                   'throttle',
                                                   'turn_src',
                                                   'turn_val',
                                                   'x_coord',
                                                   'y_coord'])

    def close(self):
        with self._lock:
            if self._running and not self._read_only and len(self._data) > 0:
                logger.info("Writing session '{}' with {} rows.".format(self._session, len(self._data)))
                with zipfile.ZipFile(os.path.join(self._directory, self._session + '.zip'), mode='a', compression=0) as archive:
                    buf = io.BytesIO()
                    self._data.to_csv(buf, index=False)
                    archive.writestr('{}.csv'.format(self._session), buf.getvalue())
                    _image_shape_str = 'null' if self._image_shape is None else 'x'.join(map(str, self._image_shape))
                    archive.writestr('meta-inf/manifest.mf', ZipDataSource.MF_TEMPLATE.format(
                        **dict(num_entries=len(self._data), uuid_node=hex(uuid.getnode()), image_shape=_image_shape_str)
                    ))
            # Reset anyway.
            self._running = False
            self._read_only = True
            self._session = None
            self._data = None

    def create_event(self, event):
        from cv2 import imencode
        with self._lock:
            # The store may have been closed while waiting on the lock.
            if self._running:
                assert not self._read_only, "This is a read-only data source."
                assert self._image_shape is None or self._image_shape == event.image.shape, "The image shape should be consistent."
                self._image_shape = event.image.shape
                timestamp = event.timestamp
                steering = float(event.steering)
                desired_speed = float(event.desired_speed)
                heading = float(event.heading)
                throttle = float(event.throttle)
                steer_src = event.steer_src
                filename = "{}__st{:+2.2f}__th{:+2.2f}__dsp{:+2.1f}__he{:+2.2f}__{}.jpg".format(
                    str(timestamp), steering, throttle, desired_speed, heading, str(steer_src)
                )
                with zipfile.ZipFile(os.path.join(self._directory, self._session + '.zip'), mode='a', compression=0) as archive:
                    _, buf = imencode(".jpg", event.image)
                    archive.writestr(filename, io.BytesIO(buf).getvalue())
                #
                self._data.loc[len(self._data)] = [timestamp,
                                                   event.vehicle,
                                                   event.vehicle_config,
                                                   filename,
                                                   event.steer_src,
                                                   event.speed_src,
                                                   event.steering,
                                                   event.desired_speed,
                                                   event.actual_speed,
                                                   event.heading,
                                                   event.throttle,
                                                   event.command_src,
                                                   event.command,
                                                   event.x_coordinate,
                                                   event.y_coordinate]
