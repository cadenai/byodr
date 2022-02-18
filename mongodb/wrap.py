#!/usr/bin/env python
from __future__ import absolute_import

import logging
import os
import subprocess

logger = logging.getLogger(__name__)

log_format = '%(levelname)s: %(asctime)s %(filename)s %(funcName)s %(message)s'


def main():
    pw_file = os.path.join(os.path.sep, 'config', 'mongo-root')
    if not os.path.exists(pw_file):
        os.umask(0)
        with open(os.open(pw_file, os.O_CREAT | os.O_WRONLY, 0o600), 'w') as fh:
            fh.write('robot')
        logger.info("Created the default mongo user.")
    subprocess.call(["mongod", "--wiredTigerCacheSizeGB", "0.20", "--bind_ip", "127.0.0.1"])


if __name__ == "__main__":
    logging.basicConfig(format=log_format, datefmt='%Y%m%d:%H:%M:%S %p %Z')
    logging.getLogger().setLevel(logging.INFO)
    main()
