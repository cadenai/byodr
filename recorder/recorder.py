from __future__ import absolute_import
import logging
import multiprocessing
import traceback
from abc import ABCMeta, abstractmethod
from collections import deque

from store import create_data_source
from six.moves import map
import six

logger = logging.getLogger(__name__)

_recorders = {}


def get_or_create_recorder(mode=None, directory=None, vehicle_type=None, vehicle_config=None, session_max=1000):
    if mode in _recorders:
        return _recorders[mode]

    # Create and add the recorder.
    datasource = None if mode is None else create_data_source(directory=directory)
    #
    if mode is None:
        recorder = NoopRecorder()
    elif mode == 'record.mode.driving':
        recorder = DirectRecorder(datasource=datasource,
                                  mode=mode,
                                  vehicle_type=vehicle_type,
                                  vehicle_config=vehicle_config,
                                  session_max=session_max)
    elif mode == 'record.mode.interventions':
        recorder = InterventionsRecorder(datasource=datasource,
                                         mode=mode,
                                         vehicle_type=vehicle_type,
                                         vehicle_config=vehicle_config,
                                         session_max=session_max)
    else:
        raise ValueError("Unknown recorder type '{}'.".format(mode))
    #
    _recorders[mode] = recorder
    logger.info("Recorder mode '{}'.".format(mode, directory))
    return recorder


class AbstractRecorder(six.with_metaclass(ABCMeta)):
    @abstractmethod
    def flush(self):
        raise NotImplementedError()

    @abstractmethod
    def get_mode(self):
        raise NotImplementedError()

    @abstractmethod
    def do_record(self, event):
        raise NotImplementedError()


class NoopRecorder(AbstractRecorder):
    def __init__(self):
        super(NoopRecorder, self).__init__()

    def flush(self):
        pass

    def get_mode(self):
        return None

    def do_record(self, event):
        pass


class DirectRecorder(six.with_metaclass(ABCMeta, AbstractRecorder)):
    """Write each event immediately to the datasource if its save attribute is set accordingly."""

    def __init__(self, datasource, mode, vehicle_type, vehicle_config, session_max=1000):
        self._datasource = datasource
        self._mode = mode
        self._vehicle_type = vehicle_type
        self._vehicle_config = vehicle_config
        self._session_max = session_max
        self._lock = multiprocessing.Lock()

    def flush(self):
        self._datasource.close()

    def get_mode(self):
        return self._mode

    def do_record(self, event):
        # Start by checking the session.
        if not self._datasource.is_open():
            self._datasource.open()
        elif len(self._datasource) >= self._session_max:
            self._datasource.close()
            self._datasource.open()
        # Continue with recording the event.
        try:
            with self._lock:
                self._record_event(event)
        except Exception as e:
            # Overwrite the image otherwise it clutters the log.
            event.image = '_deleted_for_logging_'
            logger.error("Recorder#do_record: {}".format(traceback.format_exc(e)))
            logger.error("Event: {}".format(event))

    def _record_event(self, event):
        _save = event.save_event
        if _save:
            self._persist_event(event)
            return 1
        else:
            return 0

    def _persist_event(self, event):
        _commands = ('general.fallback', 'intersection.left', 'intersection.ahead', 'intersection.right')
        assert event.command in _commands, "Command {} not recognized.".format(event.command)
        event.vehicle = self._vehicle_type
        event.vehicle_config = self._vehicle_config
        self._datasource.create_event(event)


class InterventionsRecorder(DirectRecorder):
    def __init__(self, datasource, mode, vehicle_type, vehicle_config, session_max=1000, intervention_batch_size=15):
        """
        Persist intervention events as well as non-save events leading up to and trailing the intervention.
        """
        super(InterventionsRecorder, self).__init__(datasource=datasource,
                                                    mode=mode,
                                                    vehicle_type=vehicle_type,
                                                    vehicle_config=vehicle_config,
                                                    session_max=session_max)
        self._max_length = intervention_batch_size + 1
        self._queue = deque(maxlen=self._max_length)

    def _do_save(self):
        return len(self._queue) == self._max_length and any([e.save_event for e in self._queue])

    def _record_event(self, event):
        num_recorded = 0
        if self._do_save():
            _events = list(self._queue)
            self._queue.clear()
            list(map(lambda e: super(InterventionsRecorder, self)._persist_event(e), _events))
            num_recorded = len(_events)
        self._queue.append(event)
        return num_recorded
