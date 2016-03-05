from defn.math cimport *
from defn.fixture cimport b2Fixture


cdef extern from "b2WorldCallbacks.h":
    cdef cppclass b2DestructionListener:
        # virtual ~b2DestructionListener()
        # void SayGoodbye(b2Joint* joint)
        void SayGoodbye(b2Fixture* fixture)

    cdef cppclass b2RayCastCallback:
        # virtual ~b2DestructionListener()
        # void SayGoodbye(b2Joint* joint)
        float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                              const b2Vec2& normal, float32 fraction)
