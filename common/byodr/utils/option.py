class PropertyError(ValueError):
    def __init__(self, key, msg, suggestions=None):
        if suggestions is None:
            suggestions = list()
        self.key = key
        self.message = msg
        self.suggestions = suggestions

    def __str__(self):
        return '{} - {}'.format(self.key, self.message)


def _parse(key, fn_type=(lambda x: x), **kwargs):
    try:
        return fn_type(kwargs[key])
    except (ValueError, TypeError) as e:
        raise PropertyError(key, str(e))


def parse_option(key, fn_type=(lambda x: x), default_value=None, errors=None, **kwargs):
    errors = [] if errors is None else errors
    try:
        return _parse(key, fn_type=fn_type, **kwargs)
    except KeyError:
        if default_value is None:
            errors.append(PropertyError(key, "The key is missing and no default value has been set"))
    except PropertyError as pe:
        errors.append(pe)
    return fn_type(default_value)


def hash_dict(**m):
    return hash(''.join(str(k) + str(m.get(k)) for k in sorted(m.keys())))
