import collections

import cachetools
from geographiclib.geodesic import Geodesic


def _distance_bearing(from_position, to_position):
    c_latitude, c_longitude = to_position
    p_latitude, p_longitude = from_position
    # noinspection PyUnresolvedReferences
    _g = Geodesic.WGS84.Inverse(p_latitude, p_longitude, c_latitude, c_longitude)
    # Distance in meters.
    _distance = _g['s12']
    # The azimuth is the heading measured clockwise from north.
    # azi2 is the "forward" azimuth, i.e., the heading that takes you beyond point 2 not back to point 1.
    _bearing = _g['azi2']
    return _distance, _bearing


class GeoTracker(object):
    def __init__(self, cache_ttl=10.0, min_distance_meters=0.10):
        self._min_distance = min_distance_meters
        self._positions = collections.deque(maxlen=8)
        self._cache = cachetools.TTLCache(maxsize=100, ttl=cache_ttl)

    def _begin(self, current):
        n_positions = len(self._positions)
        if n_positions == 0:
            if current is None:
                return None, None, None
            else:
                self._positions.append(current)
                return current[0], current[1], None
        if current is None:
            current = self._positions[-1]
            return current[0], current[1], None
        else:
            distance, bearing = _distance_bearing(self._positions[0], (current[0], current[1]))
            if distance >= self._min_distance:
                self._positions.append(current)
            return current[0], current[1], None

    def _track(self, current):
        n_positions = len(self._positions)
        if n_positions < 2:
            return self._begin(current)
        if current is None:
            current = self._positions[-1]
            distance, bearing = _distance_bearing(self._positions[0], (current[0], current[1]))
            return current[0], current[1], bearing
        else:
            distance, bearing = _distance_bearing(self._positions[0], (current[0], current[1]))
            if distance >= self._min_distance:
                self._positions.append(current)
            return current[0], current[1], bearing

    def clear(self):
        self._positions.clear()

    def track(self, position):
        _key = position
        res = self._cache.get(_key)
        if res is None:
            res = self._track(position)
            self._cache[_key] = res
        return res
