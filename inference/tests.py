import os
from ConfigParser import SafeConfigParser

from app import InferenceApplication
from byodr.utils.testing import CollectPublisher, QueueReceiver, CollectServer, QueueCamera


def create_application(config_dir):
    application = InferenceApplication(config_dir=config_dir)
    application.publisher = CollectPublisher()
    application.pilot = QueueReceiver()
    application.camera = QueueCamera()
    application.ipc_chatter = QueueReceiver()
    application.ipc_server = CollectServer()
    return application


def test_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    app = create_application(directory)
    publisher, camera, pilot, ipc_chatter, ipc_server = app.publisher, app.camera, app.pilot, app.ipc_chatter, app.ipc_server
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
        _parser.add_section('inference')
        _parser.set('inference', 'clock.hz', str(new_process_frequency))
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
