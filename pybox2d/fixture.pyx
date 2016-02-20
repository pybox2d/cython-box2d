cdef class FixtureDef:
    cdef b2FixtureDef *thisptr
    cdef Shape _shape
    cdef public object data

    def __cinit__(self):
        self.thisptr = new b2FixtureDef()

    def __dealloc__(self):
        del self.thisptr

    def __init__(self, shape=None, friction=0.2, restitution=0.0,
                 density=0.0, sensor=False, data=None):
        self._shape = None
        self.friction = friction
        self.restitution = restitution
        self.density = density
        self.sensor = sensor
        self.data = data

        if shape is not None:
            self.shape = shape

    # TODO filter, userdata
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


cdef class Fixture(Base):
    cdef b2Fixture *thisptr
    cdef public object data

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

    def _get_repr_info(self):
        yield ('shape', self.shape)
        yield ('friction', self.friction)
        yield ('restitution', self.restitution)
        yield ('sensor', self.sensor)
        yield ('density', self.density)

        if self.data is not None:
            yield ('data', self.data)
