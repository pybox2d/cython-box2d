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
                 density=0.0, sensor=False, data=None, filter_=None,
                 category_bits=0x01, mask_bits=0xFFFF, group_index=0):
        self._shape = None
        self.friction = friction
        self.restitution = restitution
        self.density = density
        self.sensor = sensor
        self.data = data

        if filter_ is not None:
            self.filter_ = filter_
        else:
            self.thisptr.filter.categoryBits = category_bits
            self.thisptr.filter.maskBits = mask_bits
            self.thisptr.filter.groupIndex = group_index

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

    cpdef _get_repr_info(self):
        return [('friction', self.friction),
                ('restitution', self.restitution),
                ('density', self.density),
                ('sensor', self.sensor),
                ('data', self.data),
                ('filter_', self.filter_),
                ('shape', self.shape),
                ]


cdef class Fixture(Base):
    cdef b2Fixture *thisptr
    cdef public object data
    cdef readonly Shape shape

    def __hash__(self):
        return pointer_as_key(self.thisptr)

    @staticmethod
    cdef from_b2Fixture(b2Fixture *fixture):
        fx = Fixture()
        fx.thisptr = fixture
        fx.shape = Shape.upcast(fx.thisptr.GetShape())
        return fx

    @property
    def valid(self):
        return (self.thisptr != NULL)

    def invalidate(self):
        self.thisptr = NULL
        if self.shape is not None:
            self.shape.invalidate()
        self.shape = None

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

    cpdef _get_repr_info(self):
        repr_info = [('shape', self.shape),
                     ('friction', self.friction),
                     ('restitution', self.restitution),
                     ('sensor', self.sensor),
                     ('density', self.density),
                     ]

        if self.data is not None:
            repr_info.append(('data', self.data))
        return repr_info
