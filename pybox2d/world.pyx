include "world.pyd"


cdef class World:
    cdef b2World *thisptr
    def __cinit__(self):
        self.thisptr = new b2World(b2Vec2(0.0, 0.0))

    def __dealloc__(self):
        del self.thisptr

    def __init__(self, gravity=None):
        if gravity is None:
            gravity = (0, -10)

        self.gravity = gravity

    property gravity:
        def __get__(self):
            return to_vec2(self.thisptr.GetGravity())

        def __set__(self, value):
            self.thisptr.SetGravity(to_b2vec2(value))

    property bodies:
        def __get__(self):
            cdef b2Body *b2body
            b2body = self.thisptr.GetBodyList()

            while b2body:
                body = Body()
                body.thisptr = b2body
                yield body

                b2body = b2body.GetNext()

    def step(self, float time_step, int vel_iters, int pos_iters):
        self.thisptr.Step(time_step, vel_iters, pos_iters)

    def create_body(self, BodyDef body_defn):
        body = Body()
        body.thisptr = self.thisptr.CreateBody(body_defn.thisptr)
        return body
