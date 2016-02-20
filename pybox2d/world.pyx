cdef class World:
    cdef b2World *thisptr
    cdef dict _bodies

    def __cinit__(self):
        self.thisptr = new b2World(b2Vec2(0.0, 0.0))
        self._bodies = {}

    def __dealloc__(self):
        del self.thisptr

    def __init__(self, gravity=None):
        if gravity is None:
            gravity = (0, -10)

        self.gravity = gravity
        # self.thisptr.SetDestructionListener(self.object_registry)

    property gravity:
        def __get__(self):
            return to_vec2(self.thisptr.GetGravity())

        def __set__(self, value):
            self.thisptr.SetGravity(to_b2vec2(value))

    property bodies:
        def __get__(self):
            cdef b2Body *bptr
            bptr = self.thisptr.GetBodyList()

            while bptr:
                yield self._bodies[pointer_as_key(bptr)]
                bptr = bptr.GetNext()

    def step(self, float time_step, int vel_iters, int pos_iters):
        self.thisptr.Step(time_step, vel_iters, pos_iters)

    def create_body(self, BodyDef body_defn):
        bptr = self.thisptr.CreateBody(body_defn.thisptr)

        body = Body.from_b2Body(bptr)
        self._bodies[pointer_as_key(bptr)] = body
        return body
