import glob
import json
import logging
import multiprocessing
import os
from abc import ABCMeta, abstractmethod

logger = logging.getLogger(__name__)


class NavigationCommand(object):
    DEFAULT, LEFT, AHEAD, RIGHT = (0, 1, 2, 3)

    def __init__(self, sleep=None, direction=None, speed=None):
        self._sleep = sleep
        self._direction = direction
        self._speed = speed

    def get_sleep(self):
        return self._sleep

    def get_direction(self):
        return self._direction

    def get_speed(self):
        return self._speed


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


def _parse_navigation_instructions(m):
    # version = m.get('version')
    pilot = m.get('pilot')
    if pilot is None:
        return NavigationCommand()
    return NavigationCommand(
        sleep=None if pilot.get('sleep') is None else float(pilot.get('sleep')),
        direction=_translate_navigation_direction(pilot.get('direction')),
        speed=None if pilot.get('speed') is None else float(pilot.get('speed'))
    )


class AbstractRouteDataSource(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __len__(self):
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
    def get_image_navigation_point(self, idx):
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
        self.routes = []
        self.selected_route = None
        # Route specific data follows.
        self.points = []
        self.all_images = []
        self.image_index_to_point = {}
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
        self.point_to_instructions = {}
        self.quit_event.clear()

    def load_routes(self):
        self._check_exists()
        if self._exists:
            # Each route is a sub-directory of the base folder.
            self.routes = [d for d in os.listdir(self.directory) if not d.startswith('.')]
            logger.info("Directory '{}' contains the following routes {}.".format(self.directory, self.routes))
        else:
            self._reset()

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
                np_dirs = sorted([d for d in os.listdir(os.path.join(self.directory, route_name)) if not d.startswith('.')])
                logger.info("{} -> {}".format(route_name, np_dirs))
                # Take the existing sort-order.
                image_index = 0
                for point_name in np_dirs:
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
                        command = self._get_command(os.path.join(np_dir, 'command.json'))
                        command = command if command else self._get_command(os.path.join(np_dir, point_name + '.json'))
                        self.point_to_instructions[point_name] = _parse_navigation_instructions(command)
                    # Collect images by navigation point.
                    for im_file in im_files:
                        self.all_images.append(self.fn_load_image(im_file))
                        self.image_index_to_point[image_index] = point_name
                        image_index += 1
                self.selected_route = route_name
            except OSError as e:
                logger.info(e)

    def close(self):
        self._reset()

    def quit(self):
        self.quit_event.set()

    def list_navigation_points(self):
        return self.points

    def list_all_images(self):
        return self.all_images

    def get_image_navigation_point(self, idx):
        return self.image_index_to_point[idx]

    def get_instructions(self, point):
        return self.point_to_instructions.get(point)
