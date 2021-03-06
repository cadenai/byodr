import logging
import math
import multiprocessing
import re
import time

import carla
import numpy as np
from carla import Transform, Location, Rotation

from byodr.utils import timestamp, Configurable
from byodr.utils.option import parse_option

logger = logging.getLogger(__name__)


class CarlaHandler(Configurable):

    def __init__(self, fn_on_image):
        super(CarlaHandler, self).__init__()
        self._camera_callback = fn_on_image
        self._image_shape = (600, 800, 3)
        self._tm_port = 8000
        self._rand_weather_seconds = -1
        self._world = None
        self._traffic_manager = None
        self._actor = None
        self._sensors = []
        self._actor_lock = multiprocessing.Lock()
        self._actor_last_location = None
        self._actor_distance_traveled = 0.
        self._spawn_index = 1
        self._vehicle_tick = None
        self._in_carla_autopilot = False
        self._change_weather_time = 0
        self._in_reverse = False

    def internal_quit(self, restarting=False):
        if self._vehicle_tick:
            self._world.remove_on_tick(self._vehicle_tick)
        self._destroy()

    def internal_start(self, **kwargs):
        _errors = []
        _remote = parse_option('host.location', str, 'localhost', _errors, **kwargs)
        _img_wh = parse_option('camera.image.shape', str, errors=_errors, **kwargs)
        carla_host, carla_port = _remote, 2000
        if ':' in carla_host:
            host, port = carla_host.split(':')
            carla_host, carla_port = host, int(port)
        carla_client = carla.Client(carla_host, carla_port)
        carla_client.set_timeout(2.)
        _shape = [int(x) for x in _img_wh.split('x')]
        _shape = (_shape[1], _shape[0], 3)
        self._image_shape = _shape
        self._rand_weather_seconds = parse_option('weather.random.each.seconds', int, -1, _errors, **kwargs)
        self._world = carla_client.get_world()
        self._traffic_manager = carla_client.get_trafficmanager(self._tm_port)
        self._traffic_manager.global_percentage_speed_difference(65)
        self._change_weather_time = time.time() + self._rand_weather_seconds
        self._vehicle_tick = self._world.on_tick(lambda x: self.tick(x))
        self.reset()
        return _errors

    def _reset_agent_travel(self):
        logger.info("Actor distance traveled is {:8.3f}.".format(self._actor_distance_traveled))
        self._actor_distance_traveled = 0.
        self._actor_last_location = None

    def _destroy(self):
        if self._actor is not None and self._actor.is_alive:
            self._actor.destroy()
        for sensor in self._sensors:
            if sensor.is_alive:
                sensor.destroy()

    def _create_camera(self, sensor_type='sensor.camera.rgb'):
        camera_bp = self._world.get_blueprint_library().find(sensor_type)
        # Modify the attributes of the blueprint to set image resolution and field of view.
        im_height, im_width = self._image_shape[:2]
        camera_bp.set_attribute('image_size_x', '{}'.format(im_width))
        camera_bp.set_attribute('image_size_y', '{}'.format(im_height))
        # camera_bp.set_attribute('fov', '150')
        # Set the time in seconds between sensor captures
        camera_bp.set_attribute('sensor_tick', "{:2.2f}".format(1. / 50))
        return camera_bp

    def reset(self, attempt=0):
        logger.info('Resetting ...')
        self._in_carla_autopilot = False
        self._in_reverse = False
        self._destroy()
        #
        blueprint_library = self._world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        spawn_points = self._world.get_map().get_spawn_points()
        spawn_point = np.random.choice(spawn_points)
        try:
            self._actor = self._world.spawn_actor(vehicle_bp, spawn_point)
        except RuntimeError as e:
            if attempt < 4:
                self.reset(attempt + 1)
            else:
                raise e
        logger.info("Spawn point is '{}'.".format(spawn_point))
        # Attach the camera's - defaults at https://carla.readthedocs.io/en/latest/cameras_and_sensors/.
        # Provide the position of the sensor relative to the vehicle.
        # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
        camera_front = self._world.spawn_actor(self._create_camera(),
                                               Transform(Location(x=1.25, z=1.4)),
                                               attach_to=self._actor)
        camera_rear = self._world.spawn_actor(self._create_camera(),
                                              Transform(Location(x=-1.0, z=2.0), Rotation(pitch=180, roll=180)),
                                              attach_to=self._actor)
        self._sensors.append(camera_front)
        self._sensors.append(camera_rear)
        # Subscribe to the sensor stream by providing a callback function,
        # this function is called each time a new image is generated by the sensor.
        camera_front.listen(lambda data: self._on_camera(data, camera=0))
        camera_rear.listen(lambda data: self._on_camera(data, camera=1))
        self._reset_agent_travel()
        self._traffic_manager.ignore_lights_percentage(self._actor, 100.)
        self._set_weather('ClearNoon')

    def _on_camera(self, data, camera=0):
        img = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        _height, _width = self._image_shape[:2]
        img = np.reshape(img, (_height, _width, 4))  # To bgr_a format.
        img = img[:, :, :3]  # The image standard is hwc bgr.
        self._camera_callback(img, camera)

    def _carla_vel(self):
        # Carla measures in meters and seconds.
        velocity = self._actor.get_velocity()
        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

    def _position(self):
        location = None if self._actor is None else self._actor.get_location()
        return (-1, -1) if location is None else (location.x, location.y)

    def _heading(self):
        return 0 if self._actor is None else self._actor.get_transform().rotation.yaw

    def _velocity(self):
        return 0 if self._actor is None else self._carla_vel()

    def _drive(self, steering, throttle):
        try:
            _reverse = self._in_reverse
            control = carla.VehicleControl()
            control.reverse = _reverse
            control.steer = steering
            if _reverse:
                control.throttle = abs(throttle)
            elif throttle > 0:
                control.throttle = throttle
            else:
                control.brake = abs(throttle)
            self._actor.apply_control(control)
        except Exception as e:
            logger.error("{}".format(e))

    def _track_autopilot(self, driver_mode):
        if driver_mode in ('driver_mode.inference.dnn', 'driver_mode.automatic.backend'):
            self._in_reverse = False
        _autopilot = driver_mode == 'driver_mode.automatic.backend'
        if self._in_carla_autopilot != _autopilot:
            self._in_carla_autopilot = _autopilot
            self._actor.set_autopilot(_autopilot, self._tm_port)

    def _track_reverse(self, command):
        if self._in_reverse:
            self._in_reverse = command.get('throttle') <= 0
        else:
            self._in_reverse = self._velocity() < 1e-2 and command.get('throttle') < -.99  # and command.get('arrow_down', 0) == 1

    def _set_weather(self, preset=None):
        if preset is None and self._rand_weather_seconds > 0 and time.time() > self._change_weather_time:
            self._change_weather_time = time.time() + self._rand_weather_seconds
            preset = np.random.choice([x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)])
        if preset is not None:
            logger.info("Setting the weather to '{}'.".format(preset))
            self._world.set_weather(getattr(carla.WeatherParameters, preset))

    def state(self):
        x, y = self._position()
        ap_active, ap_steering, ap_throttle = False, 0, 0
        if self._actor is not None and self._actor.is_alive and self._in_carla_autopilot:
            ap_active = True
            ap_steering = self._actor.get_control().steer
            ap_throttle = self._actor.get_control().throttle
        return dict(x_coordinate=x,
                    y_coordinate=y,
                    heading=self._heading(),
                    velocity=self._velocity(),
                    auto_active=ap_active,
                    auto_steering=ap_steering,
                    auto_throttle=ap_throttle,
                    time=timestamp())

    def tick(self, _):
        if self._actor is not None and self._actor.is_alive:
            with self._actor_lock:
                self._set_weather()
                location = self._actor.get_location()
                if self._actor_last_location is not None:
                    _x, _y = self._actor_last_location
                    self._actor_distance_traveled += math.sqrt((location.x - _x) ** 2 + (location.y - _y) ** 2)
                self._actor_last_location = (location.x, location.y)

    def noop(self):
        self._drive(steering=0, throttle=0)

    def drive(self, cmd):
        if cmd is not None and self._actor is not None:
            self._track_autopilot(cmd.get('driver'))
            self._track_reverse(cmd)
            if not self._in_carla_autopilot:
                self._drive(steering=cmd.get('steering'), throttle=cmd.get('throttle'))
