include "body.pyd"


cdef class Body:
    cdef b2Body *thisptr
    cdef public object data

    def __cinit__(self):
        self.data = None

    def __init__(self):
        pass

    def __hash__(self):
        return pointer_as_key(self.thisptr)

    cdef invalidate(self):
        # underlying b2Body has been destroyed
        self.thisptr = NULL

    @property
    def valid(self):
        return (self.thisptr != NULL)

    @staticmethod
    cdef from_b2Body(b2Body *b2body):
        body = Body()
        body.thisptr = b2body
        return body

    @safe_property
    def angle(self):
        return self.thisptr.GetAngle()

    @safe_property
    def world_center(self):
        return to_vec2(self.thisptr.GetWorldCenter())

    @safe_property
    def local_center(self):
        return to_vec2(self.thisptr.GetLocalCenter())

    @safe_rw_property
    def linear_velocity(self, Vec2 linear_velocity):
        if linear_velocity is None:
            return to_vec2(self.thisptr.GetLinearVelocity())

        self.thisptr.SetLinearVelocity(to_b2vec2(linear_velocity))

    @safe_rw_property
    def angular_velocity(self, float angular_velocity):
        if angular_velocity is None:
            return self.thisptr.GetAngularVelocity()

        self.thisptr.SetAngularVelocity(angular_velocity)

    @safe_method
    def create_fixture(self, FixtureDef userdef):
        df = userdef.thisptr
        fixture = Fixture()
        fixture.thisptr = self.thisptr.CreateFixture(df)
        return fixture

    @safe_property
    def fixtures(self):
        cdef b2Fixture *b2fixture
        b2fixture = self.thisptr.GetFixtureList()

        while b2fixture:
            fixture = Fixture()
            fixture.thisptr = b2fixture
            yield fixture

            b2fixture = b2fixture.GetNext()

    @safe_property
    def transform(self):
        return Transform.from_b2Transform(self.thisptr.GetTransform())

    property type:
        def __get__(self):
            return self.thisptr.GetType()
