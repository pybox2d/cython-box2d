cdef class Base:
    cpdef _get_repr_info(self):
        raise NotImplementedError()

    def __repr__(self):
        repr_info = ('{}={!r}'.format(k, v) for k, v in self._get_repr_info())
        return '{0}({1})'.format(self.__class__.__name__, ', '.join(repr_info))

    def __getstate__(self):
        return dict(self._get_repr_info())

    def __setstate__(self):
        pass
