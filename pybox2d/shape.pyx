include "shape.pyd"

cdef class Shape:
    # cppclass inheritance not working so cleanly in Cython - or what am I
    # doing wrong?
    cdef b2Shape *pshape


cdef class CircleShape(Shape):
    cdef b2CircleShape shape

    def __cinit__(self):
        self.pshape = &self.shape

    def __init__(self, radius=0.0, center=None):
        if center is None:
            center = (0.0, 0.0)

        self.shape.m_radius = radius
        self.center = center

    property radius:
        def __get__(self):
            return self.shape.m_radius

        def __set__(self, value):
            self.shape.m_radius = value

    property center:
        def __get__(self):
            return Vec2(self.shape.m_p.x, self.shape.m_p.y)

        def __set__(self, center):
            cx, cy = center
            self.shape.m_p.x = cx
            self.shape.m_p.y = cy

    def __str__(self):
        return ('{0.__class__.__name__}(radius={0.radius}, center={0.center})'
                '' .format(self))

    __repr__ = __str__


cdef class PolygonShape(Shape):
    cdef b2PolygonShape shape

    def __cinit__(self):
        self.pshape = &self.shape


cdef class EdgeShape(Shape):
    cdef b2EdgeShape shape

    def __cinit__(self):
        self.pshape = &self.shape
