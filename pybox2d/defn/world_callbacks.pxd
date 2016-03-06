from defn.math cimport *
from defn.fixture cimport b2Fixture
from defn.contact cimport b2Contact
from defn.collision cimport (b2ContactImpulse, b2Manifold)


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

    cdef cppclass b2ContactListener:
        void BeginContact(b2Contact* contact)
        void EndContact(b2Contact* contact)
        void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
        void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
