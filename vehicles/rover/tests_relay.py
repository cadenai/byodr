import time

from byodr.utils import timestamp
from byodr.utils.testing import QueueReceiver, CollectServer
from byodr.utils.usbrelay import SearchUsbRelayFactory
from relay import MonitorApplication


class MyMonitorReceiverThreadFactory(object):
    def __init__(self, queue):
        self._queue = queue
        self._n_create_calls = 0

    def get_num_create_calls(self):
        return self._n_create_calls

    # noinspection PyUnusedLocal
    def create(self, **kwargs):
        self._n_create_calls += 1
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


def test_relay_factory():
    # A search should not result in a None relay - we do not check whether or not it is attached although likely not in testing.
    relay = SearchUsbRelayFactory().get_relay()
    assert relay is not None


def test_monitor_relay(tmpdir):
    config_directory = str(tmpdir.realpath())
    receiver = QueueReceiver()
    relay = MyMonitorRelay()
    assert relay.is_open(), "The test assumes a normally opened relay."

    process_frequency = 8
    receiver_factory = MyMonitorReceiverThreadFactory(queue=receiver)
    application = MonitorApplication(hz=process_frequency,
                                     relay=relay,
                                     receiver_factory=receiver_factory,
                                     config_dir=config_directory)
    application.ipc_chatter = QueueReceiver()
    application.ipc_server = CollectServer()
    application.setup()
    assert application.get_hz() == process_frequency
    assert receiver.is_started()
    assert receiver_factory.get_num_create_calls() == 1

    # Setup again to simulate reconfiguration.
    application.setup()
    assert receiver.is_started()
    assert receiver_factory.get_num_create_calls() == 2

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
    assert receiver_factory.get_num_create_calls() == 3
    receiver.clear()

    # Reinitiate valid communication.
    [receiver.add(dict(time=timestamp())) for _ in range(20)]
    application.step()
    assert not relay.is_open()
    receiver.clear()

    application.finish()
    assert relay.is_open()
