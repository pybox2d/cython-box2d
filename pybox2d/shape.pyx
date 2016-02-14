include "shape.pyd"


cdef class CircleShape:
    cdef b2CircleShape *thisptr
    def __cinit__(self):
        self.thisptr = new b2CircleShape()

    def __init__(self, radius=0.0, center=None):
        if center is None:
            center = (0.0, 0.0)

        self.thisptr.m_radius = radius
        self.center = center

    @property
    def radius(self):
        return self.thisptr.m_radius

    @radius.setter
    def radius(self, value):
        self.thisptr.m_radius = value

    @property
    def _center(self):
        return Vec2(self.thisptr.m_p.x, self.thisptr.m_p.y)

    @_center.setter
    def center(self, center):
        cx, cy = center
        self.thisptr.m_p.x = cx
        self.thisptr.m_p.y = cy

    def __str__(self):
        return ('{0.__class__.__name__}(radius={0.radius}, center={0.center})'
                '' .format(self))

    __repr__ = __str__
