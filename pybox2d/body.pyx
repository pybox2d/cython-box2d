include "body.pyd"


cdef class Body:
    cdef b2Body *thisptr

    def __init__(self):
        pass

    property angle:
        def __get__(self):
            return self.thisptr.GetAngle()

    property world_center:
        def __get__(self):
            return to_vec2(self.thisptr.GetWorldCenter())

    property local_center:
        def __get__(self):
            return to_vec2(self.thisptr.GetLocalCenter())

    property linear_velocity:
        def __get__(self):
            return to_vec2(self.thisptr.GetLinearVelocity())

        def __set__(self, linear_velocity):
            self.thisptr.SetLinearVelocity(to_b2vec2(linear_velocity))

    property angular_velocity:
        def __get__(self):
            return self.thisptr.GetAngularVelocity()

        def __set__(self, angular_velocity):
            self.thisptr.SetAngularVelocity(angular_velocity)

    def create_fixture(self, FixtureDef userdef):
        df = userdef.thisptr
        fixture = Fixture()
        fixture.thisptr = self.thisptr.CreateFixture(df)
        return fixture
