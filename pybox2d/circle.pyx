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

    @safe_rw_property
    def radius(self, radius):
        '''The circle radius'''
        if radius is None:
            return self.shape.m_radius

        self.shape.m_radius = radius

    @safe_rw_property
    def center(self, center):
        '''The center of the circle'''
        if center is None:
            return Vec2((<b2CircleShape *>self.shape).m_p.x,
                        (<b2CircleShape *>self.shape).m_p.y)

        cx, cy = center
        (<b2CircleShape *>self.shape).m_p.x = cx
        (<b2CircleShape *>self.shape).m_p.y = cy

    cpdef _get_repr_info(self):
        return [('radius', self.radius),
                ('center', self.center)]
