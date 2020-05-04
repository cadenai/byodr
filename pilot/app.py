import argparse
import glob
import logging
import multiprocessing
import os
import signal
import time
import traceback
from ConfigParser import SafeConfigParser

from byodr.utils.ipc import ReceiverThread, JSONPublisher, LocalIPCServer
from pilot import DriverManager, CommandProcessor

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class IPCServer(LocalIPCServer):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(IPCServer, self).__init__('pilot', url, event, receive_timeout_ms)

    def serve_local(self, message):
        return {}


def create_controller(ipc_server, config_dir, previous=None):
    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(config_dir, '*.ini'))]
    cfg = dict(parser.items('pilot'))
    if previous is None or previous.is_reconfigured(**cfg):
        driver = DriverManager(**cfg)
        controller = CommandProcessor(driver, **cfg)
        ipc_server.register_start(driver.get_errors() + controller.get_errors())
        logger.info("Processing at {} Hz - patience is {:2.2f} ms.".format(controller.get_frequency(), controller.get_patience_ms()))
        return controller
    else:
        return previous


def main():
    parser = argparse.ArgumentParser(description='Pilot.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()
    config_dir = args.config

    publisher = JSONPublisher(url='ipc:///byodr/pilot.sock', topic='aav/pilot/output')
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = ReceiverThread(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)
    ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    ipc_server = IPCServer(url='ipc:///byodr/pilot_c.sock', event=quit_event)
    threads = [teleop, vehicle, inference, ipc_chatter, ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    controller = create_controller(ipc_server, config_dir)
    max_duration = 1. / controller.get_frequency()
    # Teleop commands or states can be none or stale when not connected or slow - default to noop.
    while not quit_event.is_set():
        try:
            # Synchronize per clock rate.
            _ts = time.time()
            commands = (teleop.get_latest(), vehicle.get_latest(), inference.get_latest())
            action = controller.next_action(*commands)
            if action:
                publisher.publish(action)
            chat = ipc_chatter.pop_latest()
            if chat and chat.get('command') == 'restart':
                controller.quit()
                controller = create_controller(ipc_server, config_dir, previous=controller)
                max_duration = 1. / controller.get_frequency()
            else:
                # Allow our threads some cpu.
                _proc_sleep = max_duration - (time.time() - _ts)
                time.sleep(max(0., _proc_sleep))
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
    logging.getLogger().setLevel(logging.INFO)
    main()
