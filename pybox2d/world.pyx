include "world.pyd"


cdef to_vec2(b2Vec2 vec):
    return Vec2(vec.x, vec.y)


cdef b2Vec2 to_b2vec2(iterable):
    return b2Vec2(iterable[0], iterable[1])


cdef class World:
    cdef b2World *thisptr
    def __cinit__(self):
        self.thisptr = new b2World(b2Vec2(0.0, 0.0))

    def __init__(self, gravity=None):
        if gravity is None:
            gravity = (0, -10)

        self.gravity = gravity

    @property
    def _gravity(self):
        return to_vec2(self.thisptr.GetGravity())

    @_gravity.setter
    def set_gravity(self, gravity):
        self.thisptr.SetGravity(to_b2vec2(gravity))
