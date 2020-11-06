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


def test_relay_cycle():
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

    # Send wakeup to close the relay after startup.
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

    # The communication requirements must still be met to let the other side know we are operational.
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


def test_relay_wakeup_reset():
    relay = MyRelay()
    publisher = CollectPublisher(topic='test/status')
    platform = MyPlatform()

    # With hz=1 the command history threshold is 180.
    application = ChassisApplication(relay=relay, hz=1)
    application.platform = platform
    application.publisher = publisher
    application.setup()

    # Send the first commands to do valid communication.
    command = dict(steering=0.1, throttle=0.2, reverse=0)
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=command)), application.step()), range(10))
    platform.send(dict(time=timestamp(), method='ras/servo/drive', data=dict(wakeup=1)))
    application.step()
    assert not relay.is_open()
    publisher.clear()

    # Send the zero commands and wakeup.
    zero = dict(steering=0, throttle=0, reverse=0)
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=zero)), application.step()), range(180))
    platform.send(dict(time=timestamp(), method='ras/servo/drive', data=dict(wakeup=1)))
    application.step()
    assert not relay.is_open()
    publisher.clear()

    # After wakeup the counters need to have been reset in order not to revert immediately.
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=zero)), application.step()), range(10))
    assert not relay.is_open()
    publisher.clear()

    application.finish()


def test_command_history_reset():
    relay = MyRelay()
    publisher = CollectPublisher(topic='test/status')
    platform = MyPlatform()

    # With hz=1 the command history threshold is 180.
    application = ChassisApplication(relay=relay, hz=1)
    application.platform = platform
    application.publisher = publisher
    application.setup()

    # Send the first commands to do valid communication.
    command = dict(steering=0.1, throttle=0.2, reverse=0)
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=command)), application.step()), range(10))
    platform.send(dict(time=timestamp(), method='ras/servo/drive', data=dict(wakeup=1)))
    application.step()
    assert not relay.is_open()
    publisher.clear()

    # Send zero commands until just below the threshold.
    zero = dict(steering=0, throttle=0, reverse=0)
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=zero)), application.step()), range(180))
    assert not relay.is_open()
    publisher.clear()

    # And then a good command and some zero commands again.
    # The relay should remain open.
    platform.send(dict(time=timestamp(), method='ras/servo/drive', data=command))
    application.step()
    map(lambda _: (platform.send(dict(time=timestamp(), method='ras/servo/drive', data=zero)), application.step()), range(10))
    assert not relay.is_open()
    publisher.clear()

    application.finish()
