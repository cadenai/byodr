import multiprocessing
import os
from ConfigParser import SafeConfigParser

from app import TeleopApplication


def create_application(config_dir):
    application = TeleopApplication(event=multiprocessing.Event(), config_dir=config_dir)
    return application


def test_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    app = create_application(directory)
    try:
        # The application writes a new user configuration file if none exists at the configuration location.
        app.setup()
        assert app.get_display_speed_scale() != 0
        assert os.path.exists(app.get_user_config_file())

        #
        # Change the configuration and request a restart.
        # Write a new config file.
        previous_speed_scale = app.get_display_speed_scale()
        new_speed_scale = previous_speed_scale + 10
        _parser = SafeConfigParser()
        _parser.read(app.get_user_config_file())
        _parser.set('teleop', 'display.speed.scale', str(new_speed_scale))
        with open(app.get_user_config_file(), 'wb') as f:
            _parser.write(f)
        app.setup()
        assert app.get_display_speed_scale() == new_speed_scale
    finally:
        app.finish()
