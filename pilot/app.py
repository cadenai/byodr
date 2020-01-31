import argparse
import json
import logging
import multiprocessing
import signal
import time
import traceback

import zmq

from byodr.utils.ipc import ReceiverThread
from pilot import DriverManager, CommandProcessor

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


# noinspection PyUnresolvedReferences
class PilotPublisher(object):
    def __init__(self):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind('ipc:///byodr/pilot.sock')
        self._publisher = publisher

    def publish(self, data):
        self._publisher.send('aav/pilot/output:{}'.format(json.dumps(data)), zmq.NOBLOCK)


def main():
    parser = argparse.ArgumentParser(description='Pilot.')
    parser.add_argument('--config', type=str, required=True, help='Config file location.')
    parser.add_argument('--clock', type=int, default=50, help='Clock frequency in hz.')
    parser.add_argument('--patience', type=float, default=.100, help='Maximum age of a command before it is considered stale.')
    args = parser.parse_args()

    threads = []
    _patience = args.patience
    controller = CommandProcessor(driver=DriverManager(config_file=args.config), patience=_patience)
    publisher = PilotPublisher()
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = ReceiverThread(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)
    threads.append(teleop)
    threads.append(vehicle)
    threads.append(inference)
    [t.start() for t in threads]

    # Determine the process frequency - we have no control over the frequency of the teleop inputs.
    _process_frequency = args.clock
    logger.info("Processing at {} Hz - patience is {:2.2f} ms.".format(_process_frequency, _patience * 1000))
    max_duration = 1. / _process_frequency

    # Teleop commands or states can be none or stale when not connected or slow - default to noop.
    while not quit_event.is_set():
        try:
            # Synchronize per clock rate.
            _ts = time.time()
            commands = (teleop.get_latest(), vehicle.get_latest(), inference.get_latest())
            action, publish = controller.next_action(*commands)
            if publish:
                publisher.publish(action)
            # Allow our threads some cpu.
            _proc_sleep = max_duration - (time.time() - _ts)
            if _proc_sleep < 0:
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
    controller.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
