include "shape.pyd"
include "fixture.pyd"


cdef class FixtureDef:
    cdef b2FixtureDef *thisptr
    cdef Shape _shape

    def __cinit__(self):
        self.thisptr = new b2FixtureDef()

    def __dealloc__(self):
        del self.thisptr

    def __init__(self, shape=None, friction=0.2, restitution=0.0,
                 density=0.0, sensor=False):
        self._shape = None
        self.friction = friction
        self.restitution = restitution
        self.density = density
        self.sensor = sensor

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


cdef class Fixture:
    cdef b2Fixture *thisptr

    property shape:
        def __get__(self):
            shape = self.thisptr.GetShape()
            return Shape.upcast(shape)

    property density:
        def __get__(self):
            return self.thisptr.GetDensity()

        def __set__(self, density):
            self.thisptr.SetDensity(density)

    property friction:
        def __get__(self):
            return self.thisptr.GetFriction()

        def __set__(self, friction):
            self.thisptr.SetFriction(friction)

    property restitution:
        def __get__(self):
            return self.thisptr.GetRestitution()

        def __set__(self, restitution):
            self.thisptr.SetRestitution(restitution)

    property sensor:
        def __get__(self):
            return self.thisptr.IsSensor()

        def __set__(self, sensor):
            self.thisptr.SetSensor(sensor)
