import argparse
import json
import logging
import multiprocessing
import signal
import threading
import time

import zmq

from pilot import DriverManager

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


# noinspection PyUnresolvedReferences
class TeleopThread(threading.Thread):
    def __init__(self, driver):
        super(TeleopThread, self).__init__()
        self._driver = driver
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, 10)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect('ipc:///tmp/byodr/teleop.sock')
        subscriber.setsockopt(zmq.SUBSCRIBE, b'aav/teleop/input')
        self._subscriber = subscriber

    def run(self):
        while not quit_event.is_set():
            try:
                self._driver.on_drive(json.loads(self._subscriber.recv().split(':', 1)[1]))
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

    driver = DriverManager(config_file=args.config)
    publisher = PilotPublisher()
    teleop_thread = TeleopThread(driver=driver)
    teleop_thread.start()

    # Determine the process frequency - we have no control over the frequency of the teleop inputs.
    _process_frequency = args.clock
    logger.info("Processing at {} Hz.".format(_process_frequency))
    max_duration = 1. / _process_frequency
    _num_violations = 0

    while not quit_event.is_set():
        try:
            # Synchronize per clock rate.
            proc_start = time.time()
            publisher.publish(driver.get_next_action())
            # Once is a fluke from three it's a pattern.
            _proc_sleep = max_duration - (time.time() - proc_start)
            _num_violations = max(0, _num_violations + (1 if _proc_sleep < 0 else -1))
            if _num_violations > 2:
                logger.warning("Cannot maintain {} Hz frequency.".format(_process_frequency))
            time.sleep(max(0, _proc_sleep))
        except Exception as e:
            logger.warning(e)
            time.sleep(max_duration)
        except KeyboardInterrupt:
            quit_event.set()

    logger.info("Waiting on threads to quit.")
    teleop_thread.join()

    logger.info("Waiting on driver to quit.")
    driver.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
