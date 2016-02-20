include "shape.pyd"


cdef class Shape:
    # cppclass inheritance not working so cleanly in Cython - or what am I
    # doing wrong?
    cdef b2Shape *shape
    cdef bool owner

    def __cinit__(self):
        self.shape = NULL
        self.owner = False

    @staticmethod
    cdef upcast(const b2Shape *shape):
        # TODO copies for now
        if shape.GetType() == ShapeType_circle:
            sh = CircleShape()
            sh.from_existing(<b2CircleShape*>shape, owner=False)
            return sh
        elif shape.GetType() == ShapeType_polygon:
            return PolygonShape()
        elif shape.GetType() == ShapeType_edge:
            return EdgeShape()
        # elif shape.GetType() == ShapeType_chain:
        #     return ChainShape()
        raise RuntimeError('Unknown shape type')


cdef class CircleShape(Shape):
    def __cinit__(self):
        self.shape = <b2Shape*>new b2CircleShape()
        self.owner = True

    def __dealloc__(self):
        if self.owner:
            del self.shape

    cdef from_existing(self, b2CircleShape *shape, owner=True):
        if self.owner:
            del self.shape

        self.shape = shape
        self.owner = owner

    def __init__(self, radius=0.0, center=None):
        if center is None:
            center = (0.0, 0.0)

        (<b2CircleShape *>self.shape).m_radius = radius
        self.center = center

    property radius:
        def __get__(self):
            return (<b2CircleShape *>self.shape).m_radius

        def __set__(self, value):
            (<b2CircleShape *>self.shape).m_radius = value

    property center:
        def __get__(self):
            return Vec2((<b2CircleShape *>self.shape).m_p.x,
                        (<b2CircleShape *>self.shape).m_p.y)

        def __set__(self, center):
            cx, cy = center
            (<b2CircleShape *>self.shape).m_p.x = cx
            (<b2CircleShape *>self.shape).m_p.y = cy

    def __str__(self):
        return ('{0.__class__.__name__}(radius={0.radius}, center={0.center})'
                '' .format(self))

    __repr__ = __str__


cdef class PolygonShape(Shape):
    cdef b2PolygonShape polygon

    def __cinit__(self):
        self.shape = &self.polygon


cdef class EdgeShape(Shape):
    cdef b2EdgeShape edge

    def __cinit__(self):
        self.shape = &self.edge
