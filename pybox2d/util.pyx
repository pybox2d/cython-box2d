from functools import wraps

cdef to_vec2(b2Vec2 vec):
    return Vec2(vec.x, vec.y)


cdef b2Vec2 to_b2vec2(iterable):
    return b2Vec2(iterable[0], iterable[1])


cdef long pointer_as_key(void *ptr):
    return (<long>ptr)


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
