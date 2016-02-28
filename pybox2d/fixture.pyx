import collections

FilterTuple = collections.namedtuple('FilterTuple',
                                     'category_bits mask_bits group_index')
default_filter = FilterTuple(category_bits=0x0001, mask_bits=0xFFFF,
                             group_index=0)

cdef class FixtureDef(Base):
    cdef b2FixtureDef *thisptr
    cdef Shape _shape
    cdef public object data

    def __cinit__(self):
        self.thisptr = new b2FixtureDef()
    
    def __dealloc__(self):
        if self.thisptr != NULL:
            del self.thisptr

    def __init__(self, shape=None, friction=0.2, restitution=0.0,
                 density=0.0, sensor=False, data=None, filter_=None):
        self._shape = None
        self.friction = friction
        self.restitution = restitution
        self.density = density
        self.sensor = sensor
        self.data = data

        if filter_ is None:
            filter_ = default_filter

        self.filter_ = filter_

        if shape is not None:
            self.shape = shape

    property filter_:
        def __get__(self):
            return FilterTuple(self.thisptr.filter.categoryBits,
                               self.thisptr.filter.maskBits,
                               self.thisptr.filter.groupIndex)

        def __set__(self, filter_info):
            ft = FilterTuple(*filter_info)
            self.thisptr.filter.categoryBits = ft.category_bits
            self.thisptr.filter.maskBits = ft.mask_bits
            self.thisptr.filter.groupIndex = ft.group_index

    property shape:
        def __get__(self):
            return self._shape

        def __set__(self, Shape shape):
            self._shape = shape

            self.thisptr.shape = shape.shape
            # cdef b2Shape *b2shape = shape.shape
            # self.thisptr.shape = <b2Shape*>shape.upcasted_shape

    property density:
        def __get__(self):
            return self.thisptr.density

        def __set__(self, density):
            self.thisptr.density = density

    property friction:
        def __get__(self):
            return self.thisptr.friction

        def __set__(self, friction):
            self.thisptr.friction = friction

    property restitution:
        def __get__(self):
            return self.thisptr.restitution

        def __set__(self, restitution):
            self.thisptr.restitution = restitution

    property sensor:
        def __get__(self):
            return self.thisptr.isSensor

        def __set__(self, sensor):
            self.thisptr.isSensor = sensor

    def _get_repr_info(self):
        yield ('friction', self.friction)
        yield ('restitution', self.restitution)
        yield ('density', self.density)
        yield ('sensor', self.sensor)
        yield ('data', self.data)
        yield ('filter_', self.filter_)
        yield ('shape', self.shape)


cdef class Fixture(Base):
    cdef b2Fixture *thisptr
    cdef public object data

    def __hash__(self):
        return pointer_as_key(self.thisptr)

    @staticmethod
    cdef from_b2Fixture(b2Fixture *fixture):
        fx = Fixture()
        fx.thisptr = fixture
        return fx

    @property
    def valid(self):
        return (self.thisptr != NULL)

    def invalidate(self):
        # TODO shapes need some rethinking
        self.thisptr = NULL

    @safe_property
    def shape(self):
        shape = self.thisptr.GetShape()
        return Shape.upcast(shape)

    @safe_rw_property
    def density(self, density):
        if density is None:
            return self.thisptr.GetDensity()

        self.thisptr.SetDensity(density)

    @safe_rw_property
    def friction(self, friction):
        if friction is None:
            return self.thisptr.GetFriction()

        self.thisptr.SetFriction(friction)

    @safe_rw_property
    def restitution(self, restitution):
        if restitution is None:
            return self.thisptr.GetRestitution()

        self.thisptr.SetRestitution(restitution)

    @safe_rw_property
    def sensor(self, sensor):
        if sensor is None:
            return self.thisptr.IsSensor()

        self.thisptr.SetSensor(sensor)

    @property  # safe due to shape getter
    def mass_data(self):
        '''The mass data associated with the shape and this fixture's density

        See Shape.compute_mass for more information.
        '''
        shape = self.shape
        return shape.get_mass_data(self.density)

    def _get_repr_info(self):
        yield ('shape', self.shape)
        yield ('friction', self.friction)
        yield ('restitution', self.restitution)
        yield ('sensor', self.sensor)
        yield ('density', self.density)

        if self.data is not None:
            yield ('data', self.data)
