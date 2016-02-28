import sys

if sys.version_info.major < 3:
    # note: for whatever reason, 2.7 is failing on circle in functools.wraps:
    #     setattr(wrapper, attr, getattr(wrapped, attr))
    #   AttributeError: 'method_descriptor' object has no attribute '__module__'
    def wraps(fcn):
        def wrapper(wrapping):
            wrapping.__doc__ = fcn.__doc__
            wrapping.__name__ = fcn.__name__
            try:
                wrapping.__dict__.update(fcn.__dict__)
            except AttributeError:
                pass
            return fcn
        return wrapper
else:
    from functools import wraps

cdef to_vec2(b2Vec2 vec):
    return Vec2(vec.x, vec.y)


cdef b2Vec2 to_b2vec2(iterable):
    return b2Vec2(iterable[0], iterable[1])


cdef long pointer_as_key(void *ptr):
    return <long>ptr


cdef safe_property(getter):
    def fget(self):
        if not self.valid:
            raise RuntimeError('Underlying C++ object has been deleted')
        return getter(self)

    return property(fget, doc=getter.__doc__)


cdef safe_rw_property(prop_wrap):
    # for whatever reason cython 'property' can't be customized, __get__ and
    # __set__ can't be wrapped, ... can't find a cleaner way to do this.
    def fget(self):
        if not self.valid:
            raise RuntimeError('Underlying C++ object has been deleted')
        return prop_wrap(self, None)

    def fset(self, value not None):
        if not self.valid:
            raise RuntimeError('Underlying C++ object has been deleted')
        prop_wrap(self, value)

    return property(fget, fset, doc=prop_wrap.__doc__)


cdef safe_method(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        if not self.valid:
            raise RuntimeError('Underlying C++ object has been deleted')
        return method(self, *args, **kwargs)

    return wrapped


class EnumBase(object):
    @classmethod
    def to_enum(cls, value):
        try:
            return cls._to_enum[value]
        except KeyError:
            raise ValueError('Invalid {}: {}'.format(cls.__name__, value))

    @classmethod
    def to_string(cls, value):
        try:
            return cls._to_string[value]
        except KeyError:
            raise ValueError('Invalid {}: {}'.format(cls.__name__, value))


cdef new_enum_type(name, type_map):
    to_string = dict(type_map)
    to_string.update({str_: str_
                      for enum_val, str_ in type_map.items()})

    to_enum = {str_: enum_val
               for enum_val, str_ in type_map.items()}
    to_enum.update({enum_val: enum_val
                    for enum_val, str_ in type_map.items()})
    return type(name, (EnumBase, ), dict(_to_string=to_string,
                                         _to_enum=to_enum))


BodyType = new_enum_type('BodyType', {b2_staticBody: 'static',
                                      b2_kinematicBody: 'kinematic',
                                      b2_dynamicBody: 'dynamic'
                                      }
                         )
