include "body.pyd"

cdef body_from_b2Body(b2Body *b2body):
    body = Body()
    body.thisptr = b2body
    return body


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

    property fixtures:
        def __get__(self):
            cdef b2Fixture *b2fixture
            b2fixture = self.thisptr.GetFixtureList()

            while b2fixture:
                fixture = Fixture()
                fixture.thisptr = b2fixture
                yield fixture

                b2fixture = b2fixture.GetNext()

    property transform:
        def __get__(self):
            ret = Transform()
            ret.from_b2Transform(self.thisptr.GetTransform())
            return ret
            # return Transform(position=to_vec2(transform.p),
            #                  rotation=Rotation(sine=transform.q.s,
            #                                    cosine=transform.q.c)
            #                  )

    property type:
        def __get__(self):
            return self.thisptr.GetType()
