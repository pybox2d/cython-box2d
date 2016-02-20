from defn.math cimport *
from defn.body cimport (b2BodyDef, b2Body)
from defn.world_callbacks cimport b2DestructionListener
from defn.shape cimport *


cdef extern from "b2World.h":
    cdef cppclass b2World:
        b2World(const b2Vec2& gravity)
        void SetDestructionListener(b2DestructionListener* listener)
        # void SetContactFilter(b2ContactFilter* filter)
        # void SetContactListener(b2ContactListener* listener)
        # void SetDebugDraw(b2Draw* debugDraw)
        b2Body* CreateBody(const b2BodyDef* defn)
        void DestroyBody(b2Body* body)
        # b2Joint* CreateJoint(const b2JointDef* def)
        # void DestroyJoint(b2Joint* joint)
        void Step(float32 timeStep, int32 velocityIterations,
                  int32 positionIterations)
        void ClearForces()
        void DrawDebugData()
        # void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
        # void RayCast(b2RayCastCallback* callback, const b2Vec2& point1,
        #              const b2Vec2& point2) const
        # b2Body* GetBodyList()
        const b2Body* GetBodyList() const
        # b2Joint* GetJointList()
        # const b2Joint* GetJointList() const
        # b2Contact* GetContactList()
        # const b2Contact* GetContactList() const
        void SetAllowSleeping(bool flag)
        bool GetAllowSleeping() const
        void SetWarmStarting(bool flag)
        bool GetWarmStarting() const
        void SetContinuousPhysics(bool flag)
        bool GetContinuousPhysics() const
        void SetSubStepping(bool flag)
        bool GetSubStepping() const
        int32 GetProxyCount() const
        int32 GetBodyCount() const
        int32 GetJointCount() const
        int32 GetContactCount() const
        int32 GetTreeHeight() const
        int32 GetTreeBalance() const
        float32 GetTreeQuality() const

        void SetGravity(const b2Vec2& gravity)
        b2Vec2 GetGravity() const

        bool IsLocked() const
        void SetAutoClearForces(bool flag)
        bool GetAutoClearForces() const
        void ShiftOrigin(const b2Vec2& newOrigin)
        # const b2ContactManager& GetContactManager() const
        # const b2Profile& GetProfile() const
        void Dump()
