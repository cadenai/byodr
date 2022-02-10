from __future__ import absolute_import
import os
from six.moves.configparser import SafeConfigParser

from app import RecorderApplication
from byodr.utils.testing import CollectPublisher, QueueReceiver, CollectServer, QueueCamera
from io import open


def test_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    ipc_server = CollectServer()
    ipc_chatter = QueueReceiver()
    app = RecorderApplication(config_dir=directory)
    app.publisher = CollectPublisher()
    app.camera = QueueCamera()
    app.ipc_server = ipc_server
    app.pilot = lambda: None
    app.vehicle = lambda: None
    app.ipc_chatter = lambda: ipc_chatter.pop_latest()

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
        _parser.add_section('recorder')
        _parser.set('recorder', 'clock.hz', str(new_process_frequency))
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
