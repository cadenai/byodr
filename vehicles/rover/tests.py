import os
from ConfigParser import SafeConfigParser

from app import RoverApplication
from byodr.utils import timestamp
from byodr.utils.testing import CollectPublisher, QueueReceiver, CollectServer
from relay import MonitorApplication


def set_rover_publishers_receivers(application):
    application.image_publisher = CollectPublisher()
    application.state_publisher = CollectPublisher()
    application.pilot = QueueReceiver()
    application.teleop = QueueReceiver()
    application.ipc_chatter = QueueReceiver()
    application.ipc_server = CollectServer()
    return application


def test_rover_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    app = set_rover_publishers_receivers(RoverApplication(config_dir=directory))
    ipc_chatter, ipc_server = app.ipc_chatter, app.ipc_server
    try:
        # The default settings must result in a workable instance.
        app.setup()
        assert len(ipc_server.collect()) == 1
        assert not bool(ipc_server.get_latest())
        assert app.get_process_frequency() != 0

        #
        # Change the configuration and request a restart.
        # Write a new config file.
        previous_process_frequency = app.get_process_frequency()
        new_process_frequency = previous_process_frequency + 10
        _parser = SafeConfigParser()
        _parser.add_section('vehicle')
        _parser.set('vehicle', 'clock.hz', str(new_process_frequency))
        with open(os.path.join(directory, 'test_config.ini'), 'wb') as f:
            _parser.write(f)
        #
        # Issue the restart request.
        ipc_chatter.add(dict(command='restart'))
        app.step()
        assert len(ipc_server.collect()) == 2
        assert not bool(ipc_server.get_latest())
        assert app.get_process_frequency() == new_process_frequency
    finally:
        app.finish()


class MyMonitorReceiverThreadFactory(object):
    def __init__(self, queue):
        self._queue = queue

    def create(self, **kwargs):
        return [], self._queue


class MyMonitorRelay(object):
    def __init__(self):
        self._open = True

    def is_open(self):
        return self._open

    def open(self):
        self._open = True

    def close(self):
        self._open = False


def test_monitor_relay(tmpdir):
    config_directory = str(tmpdir.realpath())
    receiver = QueueReceiver()
    relay = MyMonitorRelay()
    assert relay.is_open(), "The test assumes a normally opened relay."

    application = MonitorApplication(relay=relay,
                                     receiver_factory=(MyMonitorReceiverThreadFactory(queue=receiver)),
                                     config_dir=config_directory)
    application.ipc_chatter = QueueReceiver()
    application.ipc_server = CollectServer()
    application.setup()
    assert receiver.is_started()

    # At the first step the relay must not be closed as there is no reliable communication with the receiver yet.
    application.step()
    assert relay.is_open()

    # Send the first messages to initiate valid communication.
    [receiver.add(dict(time=timestamp())) for _ in range(10)]
    application.step()
    assert not relay.is_open()
    receiver.clear()

    # Simulate communication violations.
    for i in range(5):
        receiver.add(dict(time=timestamp() + i * 1e6))
        application.step()
    assert relay.is_open()
    receiver.clear()

    # And resuming.
    [receiver.add(dict(time=timestamp())) for _ in range(10)]
    application.step()
    assert not relay.is_open()
    receiver.clear()

    application.finish()
    assert relay.is_open()
