class PropertyError(ValueError):
    def __init__(self, key, msg, suggestions=None):
        if suggestions is None:
            suggestions = list()
        self.key = key
        self.message = msg
        self.suggestions = suggestions


def _parse(key, fn_type=(lambda x: x), **kwargs):
    try:
        return fn_type(kwargs.get(key))
    except (ValueError, TypeError) as e:
        raise PropertyError(key, str(e))


def parse_option(key, fn_type=(lambda x: x), default_value=None, errors=None, **kwargs):
    try:
        return _parse(key, fn_type=fn_type, **kwargs)
    except PropertyError as pe:
        if errors is not None:
            errors.append(pe)
        return default_value


def hash_dict(**kwargs):
    return hash(''.join([str(k) + str(v) for k, v in kwargs.items()]))
