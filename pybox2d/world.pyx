cdef class World:
    cdef b2World *world
    cdef dict _bodies

    def __cinit__(self):
        self.world = new b2World(b2Vec2(0.0, 0.0))
        self._bodies = {}

    def __dealloc__(self):
        del self.world

    def __init__(self, gravity=None):
        if gravity is None:
            gravity = (0, -10)

        self.gravity = gravity
        # self.world.SetDestructionListener(self.object_registry)

    property gravity:
        def __get__(self):
            return to_vec2(self.world.GetGravity())

        def __set__(self, value):
            self.world.SetGravity(to_b2vec2(value))

    property bodies:
        def __get__(self):
            cdef b2Body *bptr
            bptr = self.world.GetBodyList()

            while bptr:
                yield self._bodies[pointer_as_key(bptr)]
                bptr = bptr.GetNext()

    def step(self, float time_step, int vel_iters, int pos_iters):
        self.world.Step(time_step, vel_iters, pos_iters)

    def create_body(self, BodyDef body_defn not None):
        bptr = self.world.CreateBody(body_defn.thisptr)

        body = Body.from_b2Body(bptr)
        self._bodies[pointer_as_key(bptr)] = body
        return body

    def destroy_body(self, Body body not None):
        cdef b2Body *bptr = body.thisptr

        del self._bodies[pointer_as_key(bptr)]

        body.invalidate()
        del body
        self.world.DestroyBody(bptr)
