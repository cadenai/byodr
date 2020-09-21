import argparse
import glob
import logging
import os
import subprocess
from ConfigParser import SafeConfigParser

from byodr.utils.option import parse_option

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'


def main():
    parser = argparse.ArgumentParser(description='Camera rtsp to tcp-server.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(args.config, '*.ini'))]
    camera_cfg = dict(parser.items('camera'))

    errors = []
    _server = parse_option('camera.ip', str, errors=errors, **camera_cfg)
    _user = parse_option('camera.user', str, errors=errors, **camera_cfg)
    _password = parse_option('camera.password', str, errors=errors, **camera_cfg)
    _rtsp_port = parse_option('camera.rtsp.port', int, 0, errors=errors, **camera_cfg)
    _rtsp_path = parse_option('camera.stream.path', str, errors=errors, **camera_cfg)
    _tcp_port = 5101

    _rtsp_url = 'rtsp://{user}:{password}@{ip}:{port}{path}'.format(
        **dict(user=_user, password=_password, ip=_server, port=_rtsp_port, path=_rtsp_path)
    )
    return subprocess.call(['gst-launch-1.0',
                            'rtspsrc',
                            'location={}'.format(_rtsp_url),
                            'latency=0',
                            'drop-on-latency=true',
                            '!',
                            'queue',
                            '!',
                            'rtph264depay',
                            '!',
                            'h264parse',
                            '!',
                            'video/x-h264,stream-format="byte-stream"',
                            '!',
                            'queue',
                            '!',
                            'tcpserversink',
                            'host=0.0.0.0',
                            'port={}'.format(_tcp_port),
                            'sync=false',
                            'async=false'
                            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
