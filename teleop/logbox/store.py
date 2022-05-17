from __future__ import absolute_import

import gc
import logging
import multiprocessing
import os
import uuid
import zipfile
from abc import ABCMeta, abstractmethod
from datetime import datetime
from io import StringIO, BytesIO

import numpy as np
import pandas as pd
import six
from six.moves import map

logger = logging.getLogger(__name__)


def create_data_source(timestamp, directory=os.getcwd()):
    assert os.path.exists(directory), "The directory '{}' does not exist.".format(directory)
    return ZipDataSource(timestamp, directory=directory)


class Event(object):
    def __init__(self,
                 timestamp,
                 image_shape,
                 jpeg_buffer,
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
                 y_coordinate,
                 inference_brake):
        self.timestamp = timestamp
        self.image_shape = image_shape
        self.jpeg_buffer = jpeg_buffer
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
        self.inference_brake = inference_brake
        self.vehicle = None
        self.vehicle_config = None

    def __str__(self):
        return "Event (time={}, steer_src={}, steering={}, command={}).".format(
            self.timestamp, self.steer_src, self.steering, self.command
        )


class AbstractDataSource(six.with_metaclass(ABCMeta, object)):
    @abstractmethod
    def is_open(self):
        raise NotImplementedError()

    @abstractmethod
    def open(self):
        raise NotImplementedError()

    @abstractmethod
    def close(self):
        raise NotImplementedError()

    @abstractmethod
    def create_event(self, event):
        raise NotImplementedError()


class ZipDataSource(AbstractDataSource):
    MF_TEMPLATE = """zip-datasource-version: 0.6
zip-num-entries: {num_entries}
platform-uuid-node: "{uuid_node}"
image-shape-hwc: "{image_shape}"
"""

    def __init__(self, timestamp, directory=os.getcwd()):
        assert os.path.exists(directory), "The directory '{}' does not exist.".format(directory)
        self._date_time = datetime.fromtimestamp(timestamp * 1e-6)
        self._directory = directory
        self._lock = multiprocessing.Lock()
        self._running = False
        self._session = None
        self._data = None
        self._image_shape = None

    def _zip_file_at_write(self):
        # Create the directories now to avoid empty listings.
        _now = self._date_time
        _directory = os.path.join(self._directory, _now.strftime('%Y'), _now.strftime('%m%B'))
        if not os.path.exists(_directory):
            _mask = os.umask(000)
            os.makedirs(_directory, mode=0o775)
            os.umask(_mask)
        return os.path.join(_directory, self._session + '.zip')

    def __len__(self):
        with self._lock:
            return 0 if not self._running else len(self._data)

    def is_open(self):
        with self._lock:
            return self._running

    def open(self):
        # Existing sessions can not be reopened.
        with self._lock:
            if self._running:
                return
            self._running = True
            self._session = self._date_time.strftime('%Y%b%dT%H%M_%S%s')
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
                                               'y_coord',
                                               'inference_brake'])

    def close(self, run_gc=True):
        with self._lock:
            if self._running and len(self._data) > 0:
                logger.info("Writing session '{}' with {} rows.".format(self._session, len(self._data)))
                with zipfile.ZipFile(self._zip_file_at_write(), mode='a', compression=0) as archive:
                    buf = StringIO()
                    self._data.to_csv(buf, index=False)
                    archive.writestr('{}.csv'.format(self._session), buf.getvalue())
                    _image_shape_str = 'null' if self._image_shape is None else 'x'.join(map(str, self._image_shape))
                    archive.writestr('meta-inf/manifest.mf', ZipDataSource.MF_TEMPLATE.format(
                        **dict(num_entries=len(self._data), uuid_node=hex(uuid.getnode()), image_shape=_image_shape_str)
                    ))
                self._running = False
                self._session = None
                self._data = None
                if run_gc:
                    gc.collect()

    def create_event(self, event):
        with self._lock:
            # The store may have been closed while waiting on the lock.
            if self._running:
                assert self._image_shape is None or self._image_shape == event.image_shape, "The image shape should be consistent."
                self._image_shape = event.image_shape
                timestamp = event.timestamp
                steering = float(event.steering)
                desired_speed = float(event.desired_speed)
                heading = float(event.heading)
                throttle = float(event.throttle)
                steer_src = event.steer_src
                filename = "{}__st{:+2.2f}__th{:+2.2f}__dsp{:+2.1f}__he{:+2.2f}__{}.jpg".format(
                    str(timestamp), steering, throttle, desired_speed, heading, str(steer_src)
                )
                with zipfile.ZipFile(self._zip_file_at_write(), mode='a', compression=0) as archive:
                    archive.writestr(filename, BytesIO(np.frombuffer(memoryview(event.jpeg_buffer), dtype=np.uint8)).getvalue())
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
                                                   event.y_coordinate,
                                                   event.inference_brake]
