import argparse
import glob
import logging
import multiprocessing
import os
import signal
import time
import traceback
from ConfigParser import SafeConfigParser
from functools import partial

import numpy as np

from byodr.utils import timestamp
from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher, LocalIPCServer
from byodr.utils.option import hash_dict, parse_option
from image import get_registered_function
from inference import TFDriver, DynamicMomentum

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class TFRunner(object):
    def __init__(self, model_directory, **kwargs):
        self._lock = multiprocessing.Lock()
        self._hash = hash_dict(**kwargs)
        _errors = []
        self._gpu_id = parse_option('gpu.id', int, 0, _errors, **kwargs)
        self._process_frequency = parse_option('clock.hz', int, 10, _errors, **kwargs)
        self._steering_scale_left = parse_option('driver.dnn.steering.scale.left', lambda x: abs(float(x)), 0, _errors, **kwargs)
        self._steering_scale_right = parse_option('driver.dnn.steering.scale.right', float, 0, _errors, **kwargs)
        _penalty_up_momentum = parse_option('driver.autopilot.filter.momentum.up', float, 0, _errors, **kwargs)
        _penalty_down_momentum = parse_option('driver.autopilot.filter.momentum.down', float, 0, _errors, **kwargs)
        _penalty_ceiling = parse_option('driver.autopilot.filter.ceiling', float, 0, _errors, **kwargs)
        self._penalty_filter = DynamicMomentum(up=_penalty_up_momentum,
                                               down=_penalty_down_momentum,
                                               ceiling=_penalty_ceiling)
        _brake_scale_min = parse_option('driver.dnn.obstacle.scale.min', float, 0, _errors, **kwargs)
        _brake_scale_max = parse_option('driver.dnn.obstacle.scale.max', float, 1e-6, _errors, **kwargs)
        _corridor_scale_min = parse_option('driver.dnn.steer.corridor.scale.min', float, 0, _errors, **kwargs)
        _corridor_scale_max = parse_option('driver.dnn.steer.corridor.scale.max', float, 1e-6, _errors, **kwargs)
        self._fn_obstacle_norm = partial(self._norm_scale, min_=_brake_scale_min, max_=_brake_scale_max)
        self._fn_corridor_norm = partial(self._norm_scale, min_=_corridor_scale_min, max_=_corridor_scale_max)
        p_conv_dropout = parse_option('driver.dnn.dagger.conv.dropout', float, 0, _errors, **kwargs)
        self._fn_dave_image = get_registered_function('dnn.image.transform.dave', _errors, **kwargs)
        self._fn_alex_image = get_registered_function('dnn.image.transform.alex', _errors, **kwargs)
        self._driver = TFDriver(model_directory=model_directory, gpu_id=self._gpu_id, p_conv_dropout=p_conv_dropout)
        self._dagger = p_conv_dropout > 0
        self._poi_fallback = parse_option('driver.autopilot.poi.fallback', float, 0, _errors, **kwargs)
        self._errors = _errors
        self._fallback = False
        self._driver.activate()

    def get_gpu(self):
        return self._gpu_id

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def get_frequency(self):
        return self._process_frequency

    def get_errors(self):
        return self._errors

    def quit(self):
        self._driver.deactivate()

    def _dnn_steering(self, raw):
        return raw * (self._steering_scale_left if raw < 0 else self._steering_scale_right)

    @staticmethod
    def _norm_scale(v, min_=0., max_=1.):
        """Zero values below the minimum but let values larger than the maximum be scaled up. """
        return abs(max(0., v - min_) / (max_ - min_))

    def forward(self, image, intention):
        _dave_img = self._fn_dave_image(image)
        _alex_img = self._fn_alex_image(image)
        dagger = self._dagger

        action_out, critic_out, surprise_out, brake_out, internal_out = \
            self._driver.forward(dave_image=_dave_img,
                                 alex_image=_alex_img,
                                 turn=intention,
                                 fallback=self._fallback,
                                 dagger=dagger)

        critic = self._fn_corridor_norm(critic_out)
        surprise = self._fn_corridor_norm(surprise_out)

        # The critic is a good indicator at inference time which is why the difference between them does not work.
        # Using the geometric mean would lessen the impact of large differences between the values.
        _corridor = np.mean([surprise, critic])

        # The decision points were made dependant on turn marked samples during training.
        _poi = np.sum(internal_out)
        self._fallback = intention == 'general.fallback' or _poi < self._poi_fallback

        # Penalties to decrease desired speed.
        _obstacle_penalty = self._fn_obstacle_norm(brake_out)
        _total_penalty = max(0, min(1, self._penalty_filter.calculate(_corridor + _obstacle_penalty)))

        return dict(action=float(self._dnn_steering(action_out)),
                    corridor=float(_corridor),
                    surprise=float(surprise),
                    critic=float(critic),
                    fallback=int(self._fallback),
                    dagger=int(dagger),
                    obstacle=float(_obstacle_penalty),
                    penalty=float(_total_penalty),
                    internal=float(_poi),
                    time=timestamp()
                    )


def _glob(directory, pattern):
    return glob.glob(os.path.join(directory, pattern))


class IPCServer(LocalIPCServer):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(IPCServer, self).__init__('inference', url, event, receive_timeout_ms)

    def serve_local(self, message):
        return {}


def create_runner(ipc_server, config_dir, models_dir, previous=None):
    # The end-user config overrides come last so all settings are modifiable.
    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + _glob(models_dir, '*.ini') + _glob(config_dir, '*.ini')]
    cfg = dict(parser.items('inference'))
    _configured = False
    if previous is None:
        previous = TFRunner(models_dir, **cfg)
        _configured = True
    elif previous.is_reconfigured(**cfg):
        previous.quit()
        previous = TFRunner(models_dir, **cfg)
        _configured = True
    if _configured:
        ipc_server.register_start(previous.get_errors())
        logger.info("Processing at {} Hz on gpu {}.".format(previous.get_frequency(), previous.get_gpu()))
    return previous


def main():
    parser = argparse.ArgumentParser(description='Inference server.')
    parser.add_argument('--models', type=str, default='/models', help='Directory with the inference models.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
    ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    ipc_server = IPCServer(url='ipc:///byodr/inference_c.sock', event=quit_event)
    threads = [pilot, camera, ipc_chatter, ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    publisher = JSONPublisher(url='ipc:///byodr/inference.sock', topic='aav/inference/state')
    try:
        runner = create_runner(ipc_server, args.config, args.models)
        max_duration = 1. / runner.get_frequency()
        while not quit_event.is_set():
            proc_start = time.time()
            blob = pilot.get_latest()
            image = camera.capture()[-1]
            if image is not None:
                instruction = 'intersection.ahead' if blob is None else blob.get('instruction')
                publisher.publish(runner.forward(image=image, intention=instruction))
            chat = ipc_chatter.pop_latest()
            if chat and chat.get('command') == 'restart':
                runner = create_runner(ipc_server, args.config, args.models, previous=runner)
                max_duration = 1. / runner.get_frequency()
            else:
                _proc_sleep = max_duration - (time.time() - proc_start)
                time.sleep(max(0., _proc_sleep))
        # Main loop ran.
        logger.info("Waiting on runner to quit.")
        runner.quit()
    except KeyboardInterrupt:
        quit_event.set()
    except Exception as e:
        logger.error("{}".format(traceback.format_exc(e)))
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
