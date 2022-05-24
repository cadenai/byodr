import collections

import cachetools
from geographiclib.geodesic import Geodesic


def path(current_position, previous_position):
    c_latitude, c_longitude = current_position
    p_latitude, p_longitude = previous_position
    # noinspection PyUnresolvedReferences
    _g = Geodesic.WGS84.Inverse(p_latitude, p_longitude, c_latitude, c_longitude)
    # Distance in meters.
    _distance = _g['s12']
    # The azimuth is the heading measured clockwise from north.
    # azi2 is the "forward" azimuth, i.e., the heading that takes you beyond point 2 not back to point 1.
    _bearing = _g['azi2']
    return _distance, _bearing


class GeoTracker(object):
    def __init__(self, cache_ttl=10.0, min_distance_meters=0.05):
        self._min_distance = min_distance_meters
        self._positions = collections.deque(maxlen=2)
        self._cache = cachetools.TTLCache(maxsize=100, ttl=cache_ttl)

    def _track(self, position):
        n_positions = len(self._positions)
        if n_positions == 0:
            if position is None:
                return -1, -1, 0
            else:
                self._positions.append(position)
                return position[0], position[1], 0
        if n_positions == 1:
            if position is None:
                position = self._positions[-1]
                return position[0], position[1], 0
            else:
                distance, bearing = path((position[0], position[1]), self._positions[-1])
                if distance >= self._min_distance:
                    self._positions.append(position)
                return position[0], position[1], bearing
        # There are two or more tracked positions.
        if position is None:
            position = self._positions[-1]
            distance, bearing = path((position[0], position[1]), self._positions[-2])
            return position[0], position[1], bearing
        else:
            distance, bearing = path((position[0], position[1]), self._positions[-1])
            if distance >= self._min_distance:
                self._positions.append(position)
                return position[0], position[1], bearing
            else:
                distance, bearing = path((position[0], position[1]), self._positions[-2])
                return position[0], position[1], bearing

    def clear(self):
        self._positions.clear()

    def track(self, position):
        _key = position
        res = self._cache.get(_key)
        if res is None:
            res = self._track(position)
            self._cache[_key] = res
        return res
