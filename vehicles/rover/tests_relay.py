from byodr.utils import timestamp
from byodr.utils.testing import QueueReceiver, CollectServer
from byodr.utils.usbrelay import SearchUsbRelayFactory
from relay import MonitorApplication


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


def test_relay_factory():
    # A search should not result in a None relay - we do not check whether or not it is attached although likely not in testing.
    relay = SearchUsbRelayFactory().get_relay()
    assert relay is not None


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
