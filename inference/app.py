import argparse
import collections
import json
import logging
import multiprocessing
import signal
import threading
import time
import traceback
from functools import partial

import numpy as np
import zmq

from image import caffe_dave_200_66, hwc_alexnet
from inference import TFDriver, DynamicMomentum

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


# noinspection PyUnresolvedReferences
class CameraThread(threading.Thread):
    def __init__(self):
        super(CameraThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, 20)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect('ipc:///tmp/byodr/camera.sock')
        subscriber.setsockopt(zmq.SUBSCRIBE, b'aav/camera/0')
        self._subscriber = subscriber
        self._images = collections.deque(maxlen=1)

    def capture(self):
        return self._images[0] if bool(self._images) else None

    def run(self):
        while not quit_event.is_set():
            try:
                [_, md, data] = self._subscriber.recv_multipart()
                md = json.loads(md)
                height, width, channels = md['shape']
                img = np.frombuffer(buffer(data), dtype=np.uint8)
                img = img.reshape((height, width, channels))
                self._images.appendleft(img)
            except zmq.Again:
                pass


# noinspection PyUnresolvedReferences
class Publisher(object):
    def __init__(self):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind('ipc:///tmp/byodr/inference.sock')
        self._publisher = publisher

    def publish(self, data):
        self._publisher.send('aav/inference/state:{}'.format(json.dumps(data)), zmq.NOBLOCK)


class TFRunner(object):
    def __init__(self, driver):
        self._driver = driver
        _penalty_up_momentum = 0.2  # float(kwargs['driver.autopilot.filter.momentum.up'])
        _penalty_down_momentum = 0.4  # float(kwargs['driver.autopilot.filter.momentum.down'])
        _penalty_ceiling = 1.5  # float(kwargs['driver.autopilot.filter.ceiling'])
        self._penalty_filter = DynamicMomentum(up=_penalty_up_momentum,
                                               down=_penalty_down_momentum,
                                               ceiling=_penalty_ceiling)
        _brake_scale_min = 0  # float(kwargs['driver.dnn.obstacle.scale.min'])
        _brake_scale_max = 9999  # float(kwargs['driver.dnn.obstacle.scale.max'])
        _corridor_scale_min = 0  # float(kwargs['driver.dnn.steer.corridor.scale.min'])
        _corridor_scale_max = 9999  # float(kwargs['driver.dnn.steer.corridor.scale.max'])
        self._fn_obstacle_norm = partial(self._norm_scale, min_=_brake_scale_min, max_=_brake_scale_max)
        self._fn_corridor_norm = partial(self._norm_scale, min_=_corridor_scale_min, max_=_corridor_scale_max)
        self._driver.activate()

    def quit(self):
        self._driver.deactivate()

    @staticmethod
    def _norm_scale(v, min_=0., max_=1.):
        """Zero values below the minimum but let values larger than the maximum be scaled up. """
        return abs(max(0., v - min_) / (max_ - min_))

    def forward(self, image, turn='intersection.ahead', dagger=False):
        _dave_img = caffe_dave_200_66(image, resize_wh=(320, 240), crop=(70, 0, 10, 0))
        _alex_img = hwc_alexnet(image)
        action_out, brake_out, surprise_out, critic_out, entropy_out, conv5_out = \
            self._driver.forward(dave_image=_dave_img,
                                 alex_image=_alex_img,
                                 posor_image=_alex_img,
                                 turn=turn,
                                 dagger=dagger)

        # The critic is a good indicator at inference time which is why the difference between the two values does not work.
        # Both surprise and critic are standard deviations.
        # Using the geometric mean would lessen the impact of large differences between the values.
        _corridor_uncertainty = np.mean([surprise_out, critic_out])
        _corridor_penalty = self._fn_corridor_norm(_corridor_uncertainty)

        # Penalties to decrease desired speed.
        _obstacle_penalty = self._fn_obstacle_norm(brake_out)
        # _obstacle_penalty += _obstacle_penalty * _norm_speed * self._brake_speed_multiplier
        _total_penalty = max(0, min(1, self._penalty_filter.calculate(_corridor_penalty + _obstacle_penalty)))

        return dict(action=float(action_out),
                    brake=float(brake_out),
                    corridor=float(_corridor_penalty),
                    critic=float(critic_out),
                    entropy=float(entropy_out),
                    obstacle=float(_obstacle_penalty),
                    penalty=float(_total_penalty),
                    surprise=float(surprise_out)
                    )


def main():
    parser = argparse.ArgumentParser(description='Inference server.')
    parser.add_argument('--gpu', type=int, default=0, help='GPU number')
    parser.add_argument('--clock', type=int, default=50, help='Clock frequency in hz.')
    args = parser.parse_args()

    _process_frequency = args.clock
    logger.info("Processing at {} Hz.".format(_process_frequency))
    max_duration = 1. / _process_frequency

    camera = CameraThread()
    camera.start()
    publisher = Publisher()
    runner = TFRunner(driver=TFDriver(gpu_id=args.gpu, p_conv_dropout=0))
    try:
        while not quit_event.is_set():
            proc_start = time.time()
            image = camera.capture()
            if image is not None:
                publisher.publish(runner.forward(image=image))
            _proc_sleep = max_duration - (time.time() - proc_start)
            if _proc_sleep < 0:
                logger.warning("Cannot maintain {} Hz.".format(_process_frequency))
            time.sleep(max(0, _proc_sleep))
    except KeyboardInterrupt:
        quit_event.set()
    except StandardError as e:
        logger.error("{}".format(traceback.format_exc(e)))
        quit_event.set()

    logger.info("Waiting on thread to stop.")
    camera.join()

    logger.info("Waiting on runner to quit.")
    runner.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
