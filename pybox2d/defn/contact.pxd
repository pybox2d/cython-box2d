from defn.math cimport *
from defn.collision cimport (b2Manifold, b2WorldManifold)
from defn.body cimport b2Body
from defn.fixture cimport b2Fixture
from defn.broadphase cimport b2BroadPhase


cdef extern from "b2Contact.h":
    cdef cppclass b2ContactEdge:
        b2Body* other
        b2Contact* contact
        b2ContactEdge* prev
        b2ContactEdge* next

    cdef cppclass b2Contact:
        b2Manifold* GetManifold()
        const b2Manifold* GetManifold() const

        void GetWorldManifold(b2WorldManifold* worldManifold) const
        bool IsTouching() const
        void SetEnabled(bool flag)
        bool IsEnabled() const
        b2Contact* GetNext()
        const b2Contact* GetNext() const
        b2Fixture* GetFixtureA()
        const b2Fixture* GetFixtureA() const
        int32 GetChildIndexA() const
        b2Fixture* GetFixtureB()
        const b2Fixture* GetFixtureB() const
        int32 GetChildIndexB() const
        void SetFriction(float32 friction)
        float32 GetFriction() const
        void ResetFriction()
        void SetRestitution(float32 restitution)
        float32 GetRestitution() const
        void ResetRestitution()
        void SetTangentSpeed(float32 speed)
        float32 GetTangentSpeed() const
        void Evaluate(b2Manifold* manifold, const b2Transform& xfA,
                      const b2Transform& xfB)


    cdef cppclass b2ContactManager:
        b2ContactManager()
        # void AddPair(void* proxyUserDataA, void* proxyUserDataB)
        void FindNewContacts()
        void Destroy(b2Contact* c)
        void Collide()
        b2BroadPhase m_broadPhase
        b2Contact* m_contactList
        int32 m_contactCount
        # b2ContactFilter* m_contactFilter
        # b2ContactListener* m_contactListener
        # b2BlockAllocator* m_allocator
