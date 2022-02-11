import json
import logging

import tornado
from tornado import web

logger = logging.getLogger(__name__)


class RelayControlRequestHandler(web.RequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._relay_holder = kwargs.get('relay_holder')

    def data_received(self, chunk):
        pass

    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def get(self):
        blob = dict(state=self._relay_holder.states())
        message = json.dumps(blob)
        response_code = 200
        self.set_header('Content-Type', 'application/json')
        self.set_header('Content-Length', len(message))
        self.set_status(response_code)
        self.write(message)
        yield tornado.gen.Task(self.flush)
