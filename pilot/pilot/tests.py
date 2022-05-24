from __future__ import absolute_import

import os
from io import open

from six.moves import map
from six.moves.configparser import SafeConfigParser

from app import CommandProcessor
from app import PilotApplication
from byodr.utils import timestamp
from byodr.utils.navigate import ReloadableDataSource, FileSystemRouteDataSource
from byodr.utils.testing import CollectPublisher, QueueReceiver, CollectServer


def test_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    publisher = CollectPublisher()
    ipc_server = CollectServer()
    teleop = QueueReceiver()
    ros = QueueReceiver()
    vehicle = QueueReceiver()
    ipc_chatter = QueueReceiver()
    route_store = ReloadableDataSource(FileSystemRouteDataSource(directory=directory, load_instructions=True))
    app = PilotApplication(CommandProcessor(route_store), config_dir=directory)
    app.publisher = publisher
    app.ipc_server = ipc_server
    app.teleop = lambda: teleop.get_latest()
    app.ros = lambda: ros.get_latest()
    app.vehicle = lambda: vehicle.get_latest()
    app.inference = lambda: None
    app.ipc_chatter = lambda: ipc_chatter.get_latest()

    try:
        # The default settings must result in a workable instance.
        app.setup()
        assert len(ipc_server.collect()) == 1
        assert not bool(ipc_server.get_latest())

        #
        # Switch to direct driver mode.
        teleop.add(dict(time=timestamp(), navigator=dict(), button_b=1))
        app.step()
        teleop.add(dict(time=timestamp(), navigator=dict()))
        vehicle.add(dict(time=timestamp()))
        app.step()
        status = publisher.get_latest()
        assert status.get('driver') == 'driver_mode.teleop.direct'
        list(map(lambda x: x.clear(), [teleop, vehicle, publisher]))

        #
        # Change the configuration and request a restart.
        # Write a new config file.
        previous_process_frequency = app.get_process_frequency()
        new_process_frequency = previous_process_frequency + 10
        _parser = SafeConfigParser()
        _parser.add_section('pilot')
        _parser.set('pilot', 'clock.hz', str(new_process_frequency))
        with open(os.path.join(directory, 'test_config.ini'), 'w') as f:
            _parser.write(f)
        #
        # Issue the restart request.
        ipc_chatter.add(dict(command='restart'))
        app.step()
        assert len(ipc_server.collect()) == 2
        assert not bool(ipc_server.get_latest())
        assert app.get_process_frequency() == new_process_frequency

        # The driver should still be in direct mode.
        teleop.add(dict(time=timestamp(), navigator=dict()))
        vehicle.add(dict(time=timestamp()))
        app.step()
        status = publisher.get_latest()
        assert status.get('driver') == 'driver_mode.teleop.direct'
        list(map(lambda x: x.clear(), [teleop, vehicle, publisher]))
    finally:
        app.finish()
