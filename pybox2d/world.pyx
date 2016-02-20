from defn.world cimport b2World


cdef class World:
    cdef b2World *world
    cdef dict _bodies
    cdef object _default_body_class

    def __cinit__(self):
        self.world = new b2World(b2Vec2(0.0, 0.0))
        self._bodies = {}

    def __dealloc__(self):
        del self.world

    def __init__(self, gravity=None, default_body_class=None):
        if gravity is None:
            gravity = (0, -10)

        if default_body_class is None:
            default_body_class = Body

        self.default_body_class = default_body_class
        self.gravity = gravity

    property default_body_class:
        def __get__(self):
            return self._default_body_class

        def __set__(self, cls not None):
            if not issubclass(cls, Body):
                raise TypeError('Class must be a subclass of Body')

            self._default_body_class = cls

    property gravity:
        def __get__(self):
            return to_vec2(self.world.GetGravity())

        def __set__(self, value):
            self.world.SetGravity(to_b2vec2(value))

    def _iter_bodies(self):
        '''Iterate over Bodies in the world

        Note: users may attempt to delete or add bodies during iteration, so
        the exposed property returns a full list.
        '''
        cdef b2Body *bptr
        bptr = self.world.GetBodyList()

        while bptr:
            yield self._bodies[pointer_as_key(bptr)]
            bptr = bptr.GetNext()

    property bodies:
        def __get__(self):
            return list(self._iter_bodies())

    def step(self, float time_step, int vel_iters, int pos_iters):
        self.world.Step(time_step, vel_iters, pos_iters)

    cpdef create_body(self, BodyDef body_defn, body_class=None):
        if body_defn is None:
            raise ValueError('Body definition must be specified')

        if body_class is None:
            body_class = self.default_body_class

        if not issubclass(body_class, Body):
            raise TypeError('body_class must be a subclass of Body')

        bptr = self.world.CreateBody(body_defn.thisptr)
        body = body_class()
        (<Body>body).thisptr = bptr

        self._bodies[pointer_as_key(bptr)] = body

        if body_defn.data is not None:
            body.data = body_defn.data

        return body

    def destroy_body(self, Body body not None):
        cdef b2Body *bptr = body.thisptr

        del self._bodies[pointer_as_key(bptr)]

        body.invalidate()
        del body
        self.world.DestroyBody(bptr)
