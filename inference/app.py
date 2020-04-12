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
from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher
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
    def __init__(self, **kwargs):
        self._lock = multiprocessing.Lock()
        self._dagger = False
        self._steering_scale_left = abs(float(kwargs['driver.dnn.steering.scale.left']))
        self._steering_scale_right = float(kwargs['driver.dnn.steering.scale.right'])
        _penalty_up_momentum = float(kwargs['driver.autopilot.filter.momentum.up'])
        _penalty_down_momentum = float(kwargs['driver.autopilot.filter.momentum.down'])
        _penalty_ceiling = float(kwargs['driver.autopilot.filter.ceiling'])
        self._penalty_filter = DynamicMomentum(up=_penalty_up_momentum,
                                               down=_penalty_down_momentum,
                                               ceiling=_penalty_ceiling)
        _brake_scale_min = float(kwargs['driver.dnn.obstacle.scale.min'])
        _brake_scale_max = float(kwargs['driver.dnn.obstacle.scale.max'])
        _corridor_scale_min = float(kwargs['driver.dnn.steer.corridor.scale.min'])
        _corridor_scale_max = float(kwargs['driver.dnn.steer.corridor.scale.max'])
        self._fn_obstacle_norm = partial(self._norm_scale, min_=_brake_scale_min, max_=_brake_scale_max)
        self._fn_corridor_norm = partial(self._norm_scale, min_=_corridor_scale_min, max_=_corridor_scale_max)
        p_conv_dropout = float(kwargs['driver.dnn.dagger.conv.dropout'])
        self._fn_dave_image = get_registered_function(kwargs['dnn.image.transform.dave'])
        self._fn_alex_image = get_registered_function(kwargs['dnn.image.transform.alex'])
        self._driver = TFDriver(model_directory=kwargs['model_directory'], gpu_id=kwargs['gpu_id'], p_conv_dropout=p_conv_dropout)
        self._driver.activate()

    def quit(self):
        self._driver.deactivate()

    def set_dagger(self, value):
        with self._lock:
            self._dagger = value

    def _dnn_steering(self, raw):
        return raw * (self._steering_scale_left if raw < 0 else self._steering_scale_right)

    @staticmethod
    def _norm_scale(v, min_=0., max_=1.):
        """Zero values below the minimum but let values larger than the maximum be scaled up. """
        return abs(max(0., v - min_) / (max_ - min_))

    def forward(self, image, turn):
        with self._lock:
            dagger = self._dagger
        _dave_img = self._fn_dave_image(image)
        _alex_img = self._fn_alex_image(image)
        action_out, brake_out, surprise_out, critic_out, entropy_out = \
            self._driver.forward(dave_image=_dave_img,
                                 alex_image=_alex_img,
                                 turn=turn,
                                 dagger=dagger)

        # The critic is a good indicator at inference time which is why the difference between the two values does not work.
        # Both surprise and critic are standard deviations.
        # Using the geometric mean would lessen the impact of large differences between the values.
        _corridor_uncertainty = np.mean([surprise_out, critic_out])
        _corridor_penalty = self._fn_corridor_norm(_corridor_uncertainty)

        # Penalties to decrease desired speed.
        _obstacle_penalty = self._fn_obstacle_norm(brake_out)
        _total_penalty = max(0, min(1, self._penalty_filter.calculate(_corridor_penalty + _obstacle_penalty)))

        return dict(action=float(self._dnn_steering(action_out)),
                    brake=float(brake_out),
                    corridor=float(_corridor_penalty),
                    critic=float(critic_out),
                    dagger=int(dagger),
                    entropy=float(entropy_out),
                    obstacle=float(_obstacle_penalty),
                    penalty=float(_total_penalty),
                    surprise=float(surprise_out),
                    time=timestamp()
                    )


def _glob(directory, pattern):
    return glob.glob(os.path.join(directory, pattern))


def main():
    parser = argparse.ArgumentParser(description='Inference server.')
    parser.add_argument('--models', type=str, default='/models', help='Directory with the inference models.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    parser = SafeConfigParser()
    # The end-user config overrides come last so all settings are modifiable.
    [parser.read(_f) for _f in ['config.ini'] + _glob(args.models, '*.ini') + _glob(args.config, '*.ini')]
    cfg = dict(parser.items('inference'))
    for key in sorted(cfg):
        logger.info("{} = {}".format(key, cfg[key]))

    _gpu_id = int(cfg.get('gpu.id'))
    _process_frequency = int(cfg.get('clock.hz'))
    logger.info("Processing at {} Hz on gpu {}.".format(_process_frequency, _gpu_id))
    max_duration = 1. / _process_frequency

    threads = []
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
    threads.append(teleop)
    threads.append(pilot)
    threads.append(camera)
    [t.start() for t in threads]

    publisher = JSONPublisher(url='ipc:///byodr/inference.sock', topic='aav/inference/state')
    runner = TFRunner(model_directory=args.models, gpu_id=_gpu_id, **cfg)
    try:
        while not quit_event.is_set():
            proc_start = time.time()
            command = teleop.get_latest()
            # Leave the value intact unless the control is activated.
            if command is not None and any([k in command.keys() for k in ('button_y', 'button_b', 'button_a', 'button_x')]):
                runner.set_dagger(command.get('button_x', 0) == 1)
            blob = pilot.get_latest()
            image = camera.capture()[-1]
            if image is not None:
                instruction = 'intersection.ahead' if blob is None else blob.get('instruction')
                publisher.publish(runner.forward(image=image, turn=instruction))
            _proc_sleep = max_duration - (time.time() - proc_start)
            if _proc_sleep < 0:
                logger.warning("Cannot maintain {} Hz.".format(_process_frequency))
            time.sleep(max(0., _proc_sleep))
    except KeyboardInterrupt:
        quit_event.set()
    except StandardError as e:
        logger.error("{}".format(traceback.format_exc(e)))
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]

    logger.info("Waiting on runner to quit.")
    runner.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
