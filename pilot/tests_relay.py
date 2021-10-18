import os
import random
import time
from ConfigParser import SafeConfigParser

from byodr.utils import timestamp
from byodr.utils.testing import QueueReceiver, CollectServer, CollectJSONClient
from byodr.utils.usbrelay import SearchUsbRelayFactory
from relay import MonitorApplication


class MyJSONClientFactory(object):
    def __init__(self, client):
        self._client = client

    # noinspection PyUnusedLocal
    def create(self, url):
        return self._client


class MyStatusReceiverThreadFactory(object):
    def __init__(self, queue):
        self._queue = queue
        self._n_create_calls = 0

    def get_num_create_calls(self):
        return self._n_create_calls

    # noinspection PyUnusedLocal
    def create(self, uri):
        self._n_create_calls += 1
        return self._queue


class MyMonitorRelay(object):
    def __init__(self):
        self._open = True

    def is_open(self):
        return self._open

    def open(self):
        self._open = True

    def close(self):
        self._open = False


def test_relay_factory():
    # A search should not result in a None relay - we do not check whether or not it is attached although likely not in testing.
    relay = SearchUsbRelayFactory().get_relay()
    assert relay is not None


def test_monitor_relay(tmpdir):
    config_directory = str(tmpdir.realpath())
    relay = MyMonitorRelay()
    json_client = CollectJSONClient()
    receiver = QueueReceiver()
    client_factory = MyJSONClientFactory(client=json_client)
    status_factory = MyStatusReceiverThreadFactory(queue=receiver)
    assert relay.is_open(), "The test assumes a normally opened relay."

    pilot = QueueReceiver()
    teleop = QueueReceiver()
    ipc_chatter = QueueReceiver()
    application = MonitorApplication(relay=relay,
                                     client_factory=client_factory,
                                     status_factory=status_factory,
                                     config_dir=config_directory)
    application.ipc_server = CollectServer()
    application.pilot = lambda: pilot.get_latest()
    application.teleop = lambda: teleop.get_latest()
    application.ipc_chatter = lambda: ipc_chatter.pop_latest()

    application.setup()
    assert application.get_hz() > 0
    assert receiver.is_started()
    assert status_factory.get_num_create_calls() == 1

    # Without a configuration change the application does not restart.
    application.setup()
    assert receiver.is_started()
    assert status_factory.get_num_create_calls() == 1

    # Change a relevant setting.
    _parser = SafeConfigParser()
    _parser.add_section('vehicle')
    _parser.set('vehicle', 'ras.master.uri', 'localhost-' + str(random.random()))
    with open(os.path.join(config_directory, 'test_config.ini'), 'wb') as f:
        _parser.write(f)
    application.setup()
    assert receiver.is_started()
    assert status_factory.get_num_create_calls() == 2

    # At the first step the relay must not be closed as there is no reliable communication with the receiver yet.
    application.step()
    assert relay.is_open()

    # Send the first messages to initiate valid communication.
    [receiver.add(dict(time=timestamp())) for _ in range(10)]
    application.step()
    assert not relay.is_open()
    receiver.clear()

    # Simulate communication violations.
    for i in range(10):
        receiver.add(dict(time=timestamp() + i * 1e6))
        application.step()
    assert relay.is_open()
    receiver.clear()

    # And resuming.
    [receiver.add(dict(time=timestamp())) for _ in range(20)]
    application.step()
    assert not relay.is_open()
    receiver.clear()

    # After a significant amount of communication downtime the monitor attempts to resume by issuing a setup call.
    # Simulate communication violations.
    time.sleep(1)
    [application.step() for _ in range(10)]
    assert relay.is_open()

    # And steps without any communication.
    [application.step() for _ in range(200)]
    assert relay.is_open()
    assert receiver.is_started()
    assert status_factory.get_num_create_calls() == 3
    receiver.clear()

    # Reinitiate valid communication.
    [receiver.add(dict(time=timestamp())) for _ in range(20)]
    application.step()
    assert not relay.is_open()
    receiver.clear()

    application.finish()
    assert relay.is_open()
