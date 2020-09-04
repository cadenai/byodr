import glob
import os
from ConfigParser import SafeConfigParser

from app import RoverApplication
from byodr.utils.testing import CollectPublisher, QueueReceiver, CollectServer


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
        # The application writes a new user configuration file if none exists at the configuration location.
        assert len(glob.glob(os.path.join(directory, 'config.ini'))) == 0
        app.setup()
        assert len(glob.glob(os.path.join(directory, 'config.ini'))) == 1

        # The settings must result in a workable instance.
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
