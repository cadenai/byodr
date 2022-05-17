#!/usr/bin/env python
from __future__ import absolute_import

import argparse
import logging
import os
import re
import shutil
import subprocess

logger = logging.getLogger(__name__)

log_format = '%(levelname)s: %(asctime)s %(filename)s %(funcName)s %(message)s'


def _check_config(config_file):
    with open(config_file, 'r') as _file:
        contents = _file.read()
    if 'version 0.66.0' in contents:
        logger.info("The proxy configuration is up to date.")
    else:
        # Not all routers are at the default ip.
        _ip = re.findall("rover.*:9101", contents)[0][6:-5]
        _ssl = ('ssl crt' in contents)
        with open('haproxy_ssl.template' if _ssl else 'haproxy.template', 'r') as _template:
            with open(config_file, 'w') as _file:
                _file.write(_template.read().replace('192.168.1.32', _ip))
        logger.info("Updated the existing proxy configuration using ip '{}'.".format(_ip))


def main():
    parser = argparse.ArgumentParser(description='Http proxy server.')
    parser.add_argument('--config', type=str, default='/config/haproxy.conf', help='Configuration file.')
    args = parser.parse_args()

    config_file = args.config
    if os.path.exists(config_file):
        _check_config(config_file)
    else:
        shutil.copyfile('haproxy.template', config_file)
        logger.info("Created a new non ssl proxy configuration.")
    subprocess.call(['/usr/sbin/haproxy', '-f', config_file])


if __name__ == "__main__":
    logging.basicConfig(format=log_format, datefmt='%Y%m%d:%H:%M:%S %p %Z')
    logging.getLogger().setLevel(logging.INFO)
    main()
