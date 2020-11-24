import multiprocessing
from ConfigParser import SafeConfigParser

from app import TeleopApplication


def create_application(config_dir):
    application = TeleopApplication(event=multiprocessing.Event(), config_dir=config_dir)
    return application


def test_create_and_setup(tmpdir):
    directory = str(tmpdir.realpath())
    app = create_application(directory)
    try:
        #
        # Change the configuration and request a restart.
        # Write a new config file.
        app.setup()
        _parser = SafeConfigParser()
        _parser.read(['config.ini'])
        with open(app.get_user_config_file(), 'wb') as f:
            _parser.write(f)
        app.setup()
    finally:
        app.finish()
