import argparse
import collections
import json
import logging
import multiprocessing
import signal
import threading
import time
import traceback

import zmq

from pilot import DriverManager, CommandProcessor

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


# noinspection PyUnresolvedReferences
class ReceiverThread(threading.Thread):
    def __init__(self, url, topic=''):
        super(ReceiverThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, 10)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect(url)
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        self._subscriber = subscriber
        self._queue = collections.deque(maxlen=1)

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def run(self):
        while not quit_event.is_set():
            try:
                self._queue.appendleft(json.loads(self._subscriber.recv().split(':', 1)[1]))
            except zmq.Again:
                pass


# noinspection PyUnresolvedReferences
class PilotPublisher(object):
    def __init__(self):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind('ipc:///tmp/byodr/pilot.sock')
        self._publisher = publisher

    def publish(self, data):
        self._publisher.send('aav/pilot/output:{}'.format(json.dumps(data)), zmq.NOBLOCK)


def main():
    parser = argparse.ArgumentParser(description='Pilot.')
    parser.add_argument('--config', type=str, required=True, help='Config file location.')
    parser.add_argument('--clock', type=int, default=50, help='Clock frequency in hz.')
    args = parser.parse_args()

    threads = []
    driver_manager = DriverManager(config_file=args.config)
    controller = CommandProcessor(driver=driver_manager)
    publisher = PilotPublisher()
    teleop = ReceiverThread(url='ipc:///tmp/byodr/teleop.sock', topic=b'aav/teleop/input')
    vehicle = ReceiverThread(url='ipc:///tmp/byodr/vehicle.sock', topic=b'aav/vehicle/state')
    inference = ReceiverThread(url='ipc:///tmp/byodr/inference.sock', topic=b'aav/inference/state')
    threads.append(teleop)
    threads.append(vehicle)
    threads.append(inference)
    [t.start() for t in threads]

    # Determine the process frequency - we have no control over the frequency of the teleop inputs.
    _process_frequency = args.clock
    logger.info("Processing at {} Hz.".format(_process_frequency))
    max_duration = 1. / _process_frequency
    _num_violations = 0

    while not quit_event.is_set():
        try:
            # Synchronize per clock rate.
            proc_start = time.time()
            # Process controls if any.
            command = teleop.get_latest()
            state = vehicle.get_latest()
            controller.process(command)
            current_speed = 0 if state is None else state.get('velocity')
            publisher.publish(driver_manager.get_next_action(command, current_speed))
            # Once is a fluke from three it's a pattern.
            _proc_sleep = max_duration - (time.time() - proc_start)
            _num_violations = max(0, _num_violations + (1 if _proc_sleep < 0 else -1))
            if _num_violations > 2:
                logger.warning("Cannot maintain {} Hz.".format(_process_frequency))
            time.sleep(max(0, _proc_sleep))
        except Exception as e:
            logger.error("{}".format(traceback.format_exc(e)))
            quit_event.set()
        except KeyboardInterrupt:
            quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]

    logger.info("Waiting on driver to quit.")
    driver_manager.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
