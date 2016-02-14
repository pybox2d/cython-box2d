include "world.pyd"
include "util.pyd"


cdef class World:
    cdef b2World *thisptr
    def __cinit__(self):
        self.thisptr = new b2World(b2Vec2(0.0, 0.0))

    def __init__(self, gravity=None):
        if gravity is None:
            gravity = (0, -10)

        self.gravity = gravity

    property gravity:
        def __get__(self):
            return to_vec2(self.thisptr.GetGravity())

        def __set__(self, value):
            self.thisptr.SetGravity(to_b2vec2(value))
