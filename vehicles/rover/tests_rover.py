from __future__ import absolute_import
import glob
import os
from six.moves.configparser import SafeConfigParser

from app import RoverApplication
from byodr.utils.testing import CollectPublisher, QueueReceiver, CollectServer
from io import open


def test_rover_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    pilot = QueueReceiver()
    teleop = QueueReceiver()
    ipc_chatter = QueueReceiver()
    ipc_server = CollectServer()

    app = RoverApplication(config_dir=directory)
    app.image_publisher = CollectPublisher()
    app.state_publisher = CollectPublisher()
    app.ipc_server = ipc_server
    app.pilot = lambda: pilot.get_latest()
    app.teleop = lambda: teleop.get_latest()
    app.ipc_chatter = lambda: ipc_chatter.pop_latest()

    try:
        # The application writes a new user configuration file if none exists at the configuration location.
        assert len(glob.glob(os.path.join(directory, 'config.ini'))) == 0
        app.setup()
        assert len(glob.glob(os.path.join(directory, 'config.ini'))) == 1

        # The settings must result in a workable instance.
        assert len(ipc_server.collect()) == 1
        assert not bool(ipc_server.get_latest())
        assert app.get_hz() > 0

        #
        # Change the configuration and request a restart.
        # Write a new config file.
        previous_process_frequency = app.get_hz()
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
        assert app.get_hz() == new_process_frequency
    finally:
        app.finish()
