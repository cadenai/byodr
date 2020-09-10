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
    publisher = CollectPublisher(topic='test/status')
    platform = MyPlatform()

    application = ChassisApplication(relay=relay)
    application.platform = platform
    application.publisher = publisher
    application.setup()

    # Without reliable communication the relay is open.
    application.step()
    assert relay.is_open()
    publisher.clear()

    # A non-zero command.
    command = dict(steering=0.1, throttle=0.2, reverse=0)

    # Send the first commands to do valid communication.
    # The integrity protocol does not assume valid by default.
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=command)), application.step()), range(10))
    assert relay.is_open()
    publisher.clear()

    # Send wakeup to close the relais after startup.
    platform.send(dict(time=timestamp(), method='ras/servo/drive', data=dict(wakeup=1)))
    application.step()
    assert not relay.is_open()
    publisher.clear()

    # Simulate communication violations.
    map(lambda i: (platform.send(dict(time=timestamp() + i * 1e6, method='ras/servo/drive', data=command)), application.step()), range(10))
    assert relay.is_open()
    publisher.clear()

    # And resume.
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=command)), application.step()), range(10))
    assert not relay.is_open()
    publisher.clear()

    # Pretend missing commands but valid communication.
    _null_command = dict(steering=0, throttle=0, reverse=0)
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=_null_command)), application.step()), range(5000))
    assert relay.is_open()
    publisher.clear()

    # The communication requirements must still be met to let the other side know we are opertional.
    platform.send(dict(time=timestamp(), method='ras/servo/drive', data=_null_command))
    application.step()
    assert len(publisher.collect()) > 0
    publisher.clear()

    # Wakeup again.
    platform.send(dict(time=timestamp(), method='ras/servo/drive', data=dict(wakeup=1)))
    application.step()
    assert not relay.is_open()

    application.finish()
    assert relay.is_open()
