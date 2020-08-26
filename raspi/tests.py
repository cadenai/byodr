from byodr.utils import timestamp
from byodr.utils.testing import CollectPublisher
from servos import ChassisApplication


class MyRelay(object):
    def __init__(self):
        self._open = True

    def is_open(self):
        return self._open

    def open(self):
        self._open = True

    def close(self):
        self._open = False


class MyPlatform(object):
    def __init__(self):
        self._listeners = []

    def add_listener(self, c):
        self._listeners.append(c)

    def send(self, message):
        map(lambda x: x(message), self._listeners)


def test_relay():
    relay = MyRelay()
    publisher = CollectPublisher()
    platform = MyPlatform()

    application = ChassisApplication(relay=relay)
    application.platform = platform
    application.publisher = publisher
    application.setup()

    # Without reliable communication the relay is open.
    application.step()
    assert relay.is_open()

    # Send the first messages to initiate valid communication.
    drive_command = dict(steering=0, throttle=0, reverse=0)
    [platform.send(dict(time=timestamp(), method='ras/servo/drive', data=drive_command)) for _ in range(10)]
    application.step()
    assert not relay.is_open()

    # Simulate communication violations.
    for i in range(5):
        platform.send(dict(time=timestamp() + i * 1e6, method='ras/servo/drive', data=drive_command))
        application.step()
    assert relay.is_open()

    # And resuming.
    [platform.send(dict(time=timestamp(), method='ras/servo/drive', data=drive_command)) for _ in range(10)]
    application.step()
    assert not relay.is_open()

    application.finish()
    assert relay.is_open()
