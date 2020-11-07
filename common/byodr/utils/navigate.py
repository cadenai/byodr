import glob
import logging
import os
from abc import ABCMeta, abstractmethod

import cv2

logger = logging.getLogger(__name__)


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

    def __init__(self, directory, fn_load_image):
        self.directory = directory
        self.fn_load_image = fn_load_image
        self.routes = []
        self.selected_route = None
        # Route specific data follows.
        self.points = []
        self.all_images = []
        self.image_index_to_point = {}
        self.point_to_instruction = {}
        # Proceed with initial loading.
        self._load_routes()

    def _load_routes(self):
        # Each route is a sub-directory of the base folder.
        self.routes = [d for d in os.listdir(self.directory) if not d.startswith('.')]
        logger.info("Directory '{}' contains the following routes {}.".format(self.directory, self.routes))

    def _reset(self):
        self.points = []
        self.all_images = []
        self.image_index_to_point = {}
        self.point_to_instruction = {}

    def __len__(self):
        # Zero when no route selected.
        return len(self.points)

    def list_routes(self):
        return self.routes

    def get_selected_route(self):
        return self.selected_route

    def open(self, route_name=None):
        self._reset()
        if route_name in self.routes:
            self.selected_route = route_name
            # Load the route navigation points.
            np_dirs = sorted([d for d in os.listdir(os.path.join(self.directory, route_name)) if not d.startswith('.')])
            logger.info("{} -> {}".format(self.selected_route, np_dirs))
            # Take the existing sort-order.
            image_index = 0
            for point_name in np_dirs:
                self.points.append(point_name)
                np_dir = os.path.join(self.directory, route_name, point_name)
                _pattern = np_dir + os.path.sep
                im_files = [f for f_ in [glob.glob(_pattern + e) for e in ('*.jpg', '*.jpeg')] for f in f_]
                if len(im_files) < 1:
                    logger.info("Skipping point '{}' as there are no images for it.".format(point_name))
                    continue
                # instruction_file = os.path.join(np_dir, 'instructions.json')
                # with open(instruction_file) as f:
                #     cmd, speed = f.read().split(',')
                self.point_to_instruction[point_name] = 'n/a'
                # Collect images by navigation point.
                for im_file in im_files:
                    self.all_images.append(self.fn_load_image(cv2.imread(im_file)))
                    self.image_index_to_point[image_index] = point_name
                    image_index += 1

    def close(self):
        self._reset()

    def list_navigation_points(self):
        return self.points

    def list_all_images(self):
        return self.all_images

    def get_image_navigation_point(self, idx):
        return self.image_index_to_point[idx]

    def get_instructions(self, point):
        return self.point_to_instruction[point]
