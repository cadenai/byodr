#!/usr/bin/env python
from __future__ import absolute_import

import argparse
import logging
import os
import shutil
import subprocess

logger = logging.getLogger(__name__)

log_format = '%(levelname)s: %(asctime)s %(filename)s %(funcName)s %(message)s'


def main():
    parser = argparse.ArgumentParser(description='Http proxy server.')
    parser.add_argument('--config', type=str, default='/config/haproxy.conf', help='Configuration file.')
    args = parser.parse_args()

    config_file = args.config
    if not os.path.exists(config_file):
        shutil.copyfile('haproxy.template', config_file)
        logger.info("Created a new proxy configuration file from template.")
    subprocess.call(['/usr/sbin/haproxy', '-f', config_file])


if __name__ == "__main__":
    logging.basicConfig(format=log_format, datefmt='%Y%m%d:%H:%M:%S %p %Z')
    logging.getLogger().setLevel(logging.INFO)
    main()
