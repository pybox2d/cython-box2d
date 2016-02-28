cdef class CircleShape(Shape):
    def __cinit__(self):
        self.shape = <b2Shape*>new b2CircleShape()
        self.owner = True

    def __dealloc__(self):
        if self.owner:
            del self.shape

    def __init__(self, radius=0.0, center=None):
        if center is None:
            center = (0.0, 0.0)

        self.shape.m_radius = radius
        self.center = center

    property radius:
        '''The shape radius'''
        def __get__(self):
            return self.shape.m_radius

        def __set__(self, value):
            self.shape.m_radius = value

    property center:
        '''The center of the circle'''
        def __get__(self):
            return Vec2((<b2CircleShape *>self.shape).m_p.x,
                        (<b2CircleShape *>self.shape).m_p.y)

        def __set__(self, center):
            cx, cy = center
            (<b2CircleShape *>self.shape).m_p.x = cx
            (<b2CircleShape *>self.shape).m_p.y = cy

    def _get_repr_info(self):
        yield ('radius', self.radius)
        yield ('center', self.center)
