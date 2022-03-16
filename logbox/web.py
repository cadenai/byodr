from __future__ import absolute_import

import json
import logging

import tornado
from tornado import web

logger = logging.getLogger(__name__)


class MongoLogBox(object):
    def __init__(self, client):
        self._database = client.logbox

    def read_events(self, load_image=False, **kwargs):
        _filter = {}
        _projection = {} if load_image else {'img_buffer': False}
        start = kwargs.get('start', 0)
        length = kwargs.get('length', 10)
        cursor = self._database.events.find(
            filter=_filter,
            projection=_projection,
            sort=[('time', -1)],
            batch_size=length,
            limit=length,
            skip=(start * length)
        )
        return cursor.collection.count_documents(_filter), cursor


class EventViewer(object):
    def __init__(self):
        pass

    def __call__(self, *args, **kwargs):
        return kwargs.get('time'), kwargs.get('trigger')


class DataTableRequestHandler(web.RequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._box = MongoLogBox(kwargs.get('mongo_client'))
        self._view = EventViewer()

    def data_received(self, chunk):
        pass

    @tornado.gen.coroutine
    def get(self):
        try:
            # Parse the draw as integer for security reasons: https://datatables.net/manual/server-side.
            draw = int(self.get_query_argument('draw'))
            start = int(self.get_query_argument('start'))
            length = int(self.get_query_argument('length'))
        except ValueError:
            draw = 0
            start = 0
            length = 10

        c_total, cursor = self._box.read_events(start=start, length=length)
        data = []
        for _ in range(length):
            try:
                data.append(self._view(**cursor.next()))
            except StopIteration:
                break

        blob = dict(
            draw=draw,
            recordsTotal=c_total,
            recordsFiltered=c_total,
            data=data
        )
        # Include the error property when errors occur.
        message = json.dumps(blob)
        response_code = 200
        self.set_header('Content-Type', 'application/json')
        self.set_header('Content-Length', len(message))
        self.set_status(response_code)
        self.write(message)
