import os
from ConfigParser import SafeConfigParser

from app import RoverApplication, RoverHandler
from byodr.utils.testing import CollectPublisher, QueueReceiver, CollectServer
from core import GstSource


def create_application(config_dir):
    image_publisher = CollectPublisher()
    rover = RoverHandler(gst_source=GstSource(image_publisher))
    application = RoverApplication(handler=rover, config_dir=config_dir)
    application.state_publisher = CollectPublisher()
    application.pilot = QueueReceiver()
    application.teleop = QueueReceiver()
    application.ipc_chatter = QueueReceiver()
    application.ipc_server = CollectServer()
    return application


def test_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    app = create_application(directory)
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
