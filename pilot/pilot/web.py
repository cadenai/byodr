from __future__ import absolute_import

import json
import logging

import tornado
from tornado import web
from tornado.gen import coroutine

logger = logging.getLogger(__name__)


class RelayControlRequestHandler(web.RequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._relay_holder = kwargs.get('relay_holder')

    def data_received(self, chunk):
        pass

    @tornado.gen.coroutine
    def get(self):
        response_code = 200
        message = json.dumps(dict(states=self._relay_holder.states()))
        self.set_header('Content-Type', 'application/json')
        self.set_header('Content-Length', len(message))
        self.set_status(response_code)
        self.write(message)

    @tornado.gen.coroutine
    def post(self):
        data = json.loads(self.request.body)
        action = data.get('action')
        channel = int(data.get('channel'))
        assert channel in (3, 4), "Illegal channel requested '{}'.".format(channel)
        # Convert to zero based index.
        channel -= 1
        if action in ('on', '1', 1):
            self._relay_holder.close(channel)
        else:
            self._relay_holder.open(channel)
        states = self._relay_holder.states()
        message = json.dumps(dict(channel=channel, state=states[channel]))
        self.write(message)


class RelayConfigRequestHandler(web.RequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._relay_holder = kwargs.get('relay_holder')

    def data_received(self, chunk):
        pass

    @tornado.gen.coroutine
    def get(self):
        response_code = 200
        message = json.dumps(dict(config=self._relay_holder.pulse_config()))
        self.set_header('Content-Type', 'application/json')
        self.set_header('Content-Length', len(message))
        self.set_status(response_code)
        self.write(message)
