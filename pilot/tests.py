from ConfigParser import SafeConfigParser

from pilot import CommandProcessor


def test_processor():
    # Before the first start any settings should be marked as dirty.
    processor = CommandProcessor()
    assert processor.is_reconfigured(**dict(a='a'))

    # The instance should be intact without settings.
    processor.start()
    assert processor.get_num_starts() == 1

    # Load the default settings.
    parser = SafeConfigParser()
    parser.read('config.ini')
    settings = dict(parser.items('pilot'))
    assert len(settings) > 0
    processor.restart(**settings)
    assert processor.get_num_starts() == 2, "New configuration settings must result in a restart."
