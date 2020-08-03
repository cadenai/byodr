import argparse
import glob
import logging
import os
from ConfigParser import SafeConfigParser
from functools import partial

import numpy as np

from byodr.utils import timestamp, Configurable, Application
from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher, LocalIPCServer
from byodr.utils.option import parse_option
from image import get_registered_function
from inference import TFDriver, DynamicMomentum, maneuver_index


class TFRunner(Configurable):
    def __init__(self, model_directory):
        super(TFRunner, self).__init__()
        self._model_directory = model_directory
        self._gpu_id = 0
        self._process_frequency = 10
        self._steering_scale_left = 1
        self._steering_scale_right = 1
        self._penalty_filter = None
        self._fn_obstacle_norm = None
        self._fn_brake_critic_norm = None
        self._fn_corridor_norm = None
        self._fn_dave_image = None
        self._fn_alex_image = None
        self._driver = None
        self._dagger = False
        self._fallback = False

    def get_gpu(self):
        return self._gpu_id

    def get_frequency(self):
        return self._process_frequency

    def internal_quit(self, restarting=False):
        if self._driver is not None:
            self._driver.deactivate()

    def internal_start(self, **kwargs):
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
        _brake_scale_max = parse_option('driver.dnn.obstacle.scale.max', float, 1e-6, _errors, **kwargs)
        _brake_critic_scale_max = parse_option('driver.dnn.brake.critic.scale.max', float, 1e-6, _errors, **kwargs)
        _corridor_scale_max = parse_option('driver.dnn.steer.corridor.scale.max', float, 1e-6, _errors, **kwargs)
        self._fn_obstacle_norm = partial(self._norm_scale, min_=0, max_=_brake_scale_max)
        self._fn_brake_critic_norm = partial(self._norm_scale, min_=0, max_=_brake_critic_scale_max)
        self._fn_corridor_norm = partial(self._norm_scale, min_=0, max_=_corridor_scale_max)
        p_conv_dropout = parse_option('driver.dnn.dagger.conv.dropout', float, 0, _errors, **kwargs)
        self._fn_dave_image = get_registered_function('dnn.image.transform.dave', _errors, **kwargs)
        self._fn_alex_image = get_registered_function('dnn.image.transform.alex', _errors, **kwargs)
        self._driver = TFDriver(model_directory=self._model_directory, gpu_id=self._gpu_id, p_conv_dropout=p_conv_dropout)
        self._dagger = p_conv_dropout > 0
        self._driver.activate()
        return _errors

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

        action_out, critic_out, surprise_out, brake_out, brake_critic_out = \
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
        _intention_index = maneuver_index(intention)
        self._fallback = _intention_index == 0

        # Penalties to decrease desired speed.
        _obstacle_penalty = self._fn_obstacle_norm(brake_out) + self._fn_brake_critic_norm(brake_critic_out)
        _total_penalty = max(0, min(1, self._penalty_filter.calculate(_corridor + _obstacle_penalty)))

        return dict(action=float(self._dnn_steering(action_out)),
                    corridor=float(_corridor),
                    surprise=float(surprise),
                    critic=float(critic),
                    fallback=int(self._fallback),
                    dagger=int(dagger),
                    obstacle=float(_obstacle_penalty),
                    penalty=float(_total_penalty),
                    internal=[float(0)],
                    time=timestamp()
                    )


class InferenceApplication(Application):
    def __init__(self, config_dir=os.getcwd(), models_dir=os.getcwd()):
        super(InferenceApplication, self).__init__()
        self._config_dir = config_dir
        self._models_dir = models_dir
        self._runner = TFRunner(models_dir)
        self.publisher = None
        self.ipc_server = None
        self.ipc_chatter = None
        self.pilot = None
        self.camera = None

    @staticmethod
    def _glob(directory, pattern):
        return glob.glob(os.path.join(directory, pattern))

    def _config(self):
        parser = SafeConfigParser()
        # The end-user config overrides come last so all settings are modifiable.
        [parser.read(_f) for _f in ['config.ini'] + self._glob(self._models_dir, '*.ini') + self._glob(self._config_dir, '*.ini')]
        return dict(parser.items('inference'))

    def setup(self):
        if self.active():
            _restarted = self._runner.restart(**self._config())
            if _restarted:
                self.ipc_server.register_start(self._runner.get_errors())
                _frequency = self._runner.get_frequency()
                self.set_hz(_frequency)
                self.logger.info("Processing at {} Hz on gpu {}.".format(_frequency, self._runner.get_gpu()))

    def finish(self):
        self._runner.quit()

    def step(self):
        blob = self.pilot.get_latest()
        image = self.camera.capture()[-1]
        if image is not None:
            instruction = 'intersection.ahead' if blob is None else blob.get('instruction')
            self.publisher.publish(self._runner.forward(image=image, intention=instruction))
        chat = self.ipc_chatter.pop_latest()
        if chat and chat.get('command') == 'restart':
            self.setup()


def main():
    parser = argparse.ArgumentParser(description='Inference server.')
    parser.add_argument('--models', type=str, default='/models', help='Directory with the inference models.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    application = InferenceApplication(config_dir=args.config, models_dir=args.models)
    quit_event = application.quit_event
    logger = application.logger

    application.publisher = JSONPublisher(url='ipc:///byodr/inference.sock', topic='aav/inference/state')
    application.pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    application.camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
    application.ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/inference_c.sock', name='inference', event=quit_event)
    threads = [application.pilot, application.camera, application.ipc_chatter, application.ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    application.run()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
