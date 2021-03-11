from __future__ import absolute_import

import os
import sys
from io import open

from byodr.utils.testing import CollectPublisher, QueueReceiver, CollectServer, QueueCamera
from .app import InferenceApplication, TFRunner

if sys.version_info > (3,):
    from configparser import ConfigParser as SafeConfigParser
else:
    from six.moves.configparser import SafeConfigParser


class FakeNavigator(object):
    def recompile(self):
        pass

    def restart(self, fn_dave_image, fn_alex_image, recognition_threshold=0, gpu_id=0, runtime_compilation=1):
        pass

    def quit(self):
        pass


def test_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    ipc_server = CollectServer()
    pilot = QueueReceiver()
    teleop = QueueReceiver()
    ipc_chatter = QueueReceiver()

    runner = TFRunner(navigator=FakeNavigator())
    app = InferenceApplication(runner=runner, config_dir=directory, internal_models=directory)
    app.publisher = CollectPublisher()
    app.camera = QueueCamera()
    app.ipc_server = ipc_server
    app.teleop = lambda: teleop.get_latest()
    app.pilot = lambda: pilot.get_latest()
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
        _parser.add_section('inference')
        _parser.set('inference', 'clock.hz', str(new_process_frequency))
        with open(os.path.join(directory, 'test_config.ini'), 'w') as f:
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
