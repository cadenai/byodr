import glob
import json
import logging
import multiprocessing
import os
import threading
from abc import ABCMeta, abstractmethod

from byodr.utils import timestamp

logger = logging.getLogger(__name__)


def _translate_navigation_direction(value):
    if value is not None:
        value = value.lower()
        if value == 'left':
            return NavigationCommand.LEFT
        elif value == 'right':
            return NavigationCommand.RIGHT
        elif value == 'ahead':
            return NavigationCommand.AHEAD
        elif value == 'default':
            return NavigationCommand.DEFAULT
    # No change in direction.
    return None


class NavigationCommand(object):
    DEFAULT, LEFT, AHEAD, RIGHT = (0, 1, 2, 3)

    def __init__(self, sleep=None, direction=None, speed=None):
        self._time = None
        self._sleep = sleep
        self._direction = direction
        self._speed = speed

    def get_time(self):
        return self._time

    def set_time(self, value):
        self._time = value
        return self

    def get_sleep(self):
        return self._sleep

    def get_direction(self):
        return self._direction

    def get_speed(self):
        return self._speed


class NavigationInstructions(object):
    def __init__(self, version=1, commands=None):
        self._version = version
        commands = commands or []
        if not isinstance(commands, tuple) and not isinstance(commands, list):
            commands = [commands]
        self._commands = commands

    def get_commands(self):
        return self._commands


def _parse_navigation_instructions(m):
    """
    {
        "version": 1,
        "pilot": {"direction": "ahead" }
    }

    {
        "version": 1,
        "pilot": [{"speed": 0}, {"sleep": 30, "direction": "left", "speed": 1}]
    }
    """

    version = m.get('version', 1)
    commands = []
    pilot = m.get('pilot')
    if pilot is not None:
        nodes = pilot if isinstance(pilot, list) else [pilot]
        for node in nodes:
            commands.append(NavigationCommand(
                sleep=None if node.get('sleep') is None else float(node.get('sleep')),
                direction=_translate_navigation_direction(node.get('direction')),
                speed=None if node.get('speed') is None else float(node.get('speed'))
            ))
    return NavigationInstructions(version, commands)


class AbstractRouteDataSource(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __len__(self):
        raise NotImplementedError()

    @abstractmethod
    def load_routes(self):
        raise NotImplementedError()

    @abstractmethod
    def list_routes(self):
        raise NotImplementedError()

    @abstractmethod
    def get_selected_route(self):
        raise NotImplementedError()

    @abstractmethod
    def open(self, route_name=None):
        raise NotImplementedError()

    @abstractmethod
    def is_open(self):
        raise NotImplementedError()

    @abstractmethod
    def close(self):
        raise NotImplementedError()

    @abstractmethod
    def quit(self):
        raise NotImplementedError()

    @abstractmethod
    def list_navigation_points(self):
        raise NotImplementedError()

    @abstractmethod
    def list_all_images(self):
        raise NotImplementedError()

    @abstractmethod
    def get_image(self, image_id):
        raise NotImplementedError()

    @abstractmethod
    def get_image_navigation_point(self, idx):
        raise NotImplementedError()

    @abstractmethod
    def get_image_navigation_point_id(self, idx):
        raise NotImplementedError()

    @abstractmethod
    def get_instructions(self, point):
        raise NotImplementedError()


class FileSystemRouteDataSource(AbstractRouteDataSource):

    def __init__(self, directory, fn_load_image=(lambda x: x), load_instructions=True):
        self.directory = directory
        self.fn_load_image = fn_load_image
        self.load_instructions = load_instructions
        self.quit_event = multiprocessing.Event()
        self._load_timestamp = 0
        self.routes = []
        self.selected_route = None
        # Route specific data follows.
        self.points = []
        self.all_images = []
        self.image_index_to_point = {}
        self.image_index_to_point_id = {}
        self.point_to_instructions = {}
        self._check_exists()

    def _check_exists(self):
        directory = self.directory
        self._exists = directory is not None and os.path.exists(directory) and os.path.isdir(directory)

    def _reset(self):
        self.selected_route = None
        self.points = []
        self.all_images = []
        self.image_index_to_point = {}
        self.image_index_to_point_id = {}
        self.point_to_instructions = {}
        self.quit_event.clear()

    def load_routes(self):
        self._check_exists()
        if not self._exists:
            self._reset()
        else:
            _now = timestamp()  # In micro seconds.
            if _now - self._load_timestamp > 1e6:
                # Each route is a sub-directory of the base folder.
                self.routes = [d for d in os.listdir(self.directory) if not d.startswith('.')]
                self._load_timestamp = _now
                logger.info("Directory '{}' contains the following routes {}.".format(self.directory, self.routes))

    @staticmethod
    def _get_command(fname):
        try:
            with open(fname) as f:
                return json.load(f)
        except IOError:
            return {}

    def __len__(self):
        # Zero when no route selected.
        return len(self.points)

    def list_routes(self):
        return self.routes

    def get_selected_route(self):
        return self.selected_route

    def open(self, route_name=None):
        # Reopening the selected route constitutes a reload of the disk state.
        self._reset()
        if self._exists and route_name in self.routes:
            try:
                # Load the route navigation points.
                _route_directory = os.path.join(self.directory, route_name)
                if os.path.exists(_route_directory) and os.path.isdir(_route_directory):
                    np_dirs = sorted([d for d in os.listdir(_route_directory) if not d.startswith('.')])
                    logger.info("{} -> {}".format(route_name, np_dirs))
                    # Take the existing sort-order.
                    image_index = 0
                    for point_id, point_name in enumerate(np_dirs):
                        if self.quit_event.is_set():
                            break
                        self.points.append(point_name)
                        np_dir = os.path.join(self.directory, route_name, point_name)
                        _pattern = np_dir + os.path.sep
                        im_files = [f for f_ in [glob.glob(_pattern + e) for e in ('*.jpg', '*.jpeg')] for f in f_]
                        if len(im_files) < 1:
                            logger.info("Skipping point '{}' as there are no images for it.".format(point_name))
                            continue
                        if self.load_instructions:
                            contents = self._get_command(os.path.join(np_dir, 'command.json'))
                            contents = contents if contents else self._get_command(os.path.join(np_dir, point_name + '.json'))
                            self.point_to_instructions[point_name] = _parse_navigation_instructions(contents)
                        # Collect images by navigation point.
                        for im_file in im_files:
                            self.all_images.append(self.fn_load_image(im_file))
                            self.image_index_to_point[image_index] = point_name
                            self.image_index_to_point_id[image_index] = point_id
                            image_index += 1
                    self.selected_route = route_name
            except OSError as e:
                logger.info(e)

    def is_open(self):
        return self.selected_route in self.routes

    def close(self):
        self._reset()

    def quit(self):
        self.quit_event.set()

    def list_navigation_points(self):
        return self.points

    def list_all_images(self):
        return self.all_images

    def get_image(self, image_id):
        image_id = -1 if image_id is None else image_id
        images = self.list_all_images()
        return images[image_id] if len(images) > image_id >= 0 else None

    def get_image_navigation_point(self, idx):
        return self.image_index_to_point[idx]

    def get_image_navigation_point_id(self, idx):
        return self.image_index_to_point_id[idx]

    def get_instructions(self, point):
        return self.point_to_instructions.get(point)


class ReloadableDataSource(AbstractRouteDataSource):
    def __init__(self, delegate):
        self._delegate = delegate
        self._lock = threading.Lock()
        # Cache the most recent selected route.
        self._last_listed_routes = []
        self._last_selected_route = None

    def _do_safe(self, fn):
        _acquired = self._lock.acquire(False)
        try:
            return fn(_acquired)
        finally:
            if _acquired:
                self._lock.release()

    def __len__(self):
        return self._do_safe(lambda acquired: len(self._delegate) if acquired else 0)

    def load_routes(self):
        with self._lock:
            self._delegate.load_routes()

    def list_routes(self):
        _acquired = self._lock.acquire(False)
        try:
            if _acquired:
                self._last_listed_routes = self._delegate.list_routes()
            return self._last_listed_routes
        finally:
            if _acquired:
                self._lock.release()

    def get_selected_route(self):
        _acquired = self._lock.acquire(False)
        try:
            if _acquired:
                self._last_selected_route = self._delegate.get_selected_route()
            return self._last_selected_route
        finally:
            if _acquired:
                self._lock.release()

    def open(self, route_name=None):
        with self._lock:
            self._delegate.open(route_name)

    def is_open(self):
        return self._do_safe(lambda acquired: self._delegate.is_open() if acquired else False)

    def close(self):
        with self._lock:
            self._delegate.close()

    def quit(self):
        with self._lock:
            self._delegate.quit()

    def list_navigation_points(self):
        return self._do_safe(lambda acquired: self._delegate.list_navigation_points() if acquired else [])

    def list_all_images(self):
        return self._do_safe(lambda acquired: self._delegate.list_all_images() if acquired else [])

    def get_image(self, image_id):
        return self._do_safe(lambda acquired: self._delegate.get_image(image_id) if acquired else None)

    def get_image_navigation_point(self, idx):
        return self._do_safe(lambda acquired: self._delegate.get_image_navigation_point(idx) if acquired else None)

    def get_image_navigation_point_id(self, idx):
        return self._do_safe(lambda acquired: self._delegate.get_image_navigation_point_id(idx) if acquired else None)

    def get_instructions(self, point):
        return self._do_safe(lambda acquired: self._delegate.get_instructions(point) if acquired else None)
