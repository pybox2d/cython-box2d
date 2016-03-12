from defn.math cimport *
from defn.collision cimport (b2RayCastInput, b2AABB)

DEF e_nullProxy = -1

cdef extern from "b2BroadPhase.h":
    cdef struct b2Pair:
        int32 proxyIdA
        int32 proxyIdB

    cdef cppclass b2BroadPhase:
        b2BroadPhase()
        # ~b2BroadPhase()

        int32 CreateProxy(const b2AABB& aabb, void* userData)
        void DestroyProxy(int32 proxyId)
        void MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
        void TouchProxy(int32 proxyId)
        const b2AABB& GetFatAABB(int32 proxyId) const
        # void* GetUserData(int32 proxyId) const
        bool TestOverlap(int32 proxyIdA, int32 proxyIdB) const
        int32 GetProxyCount() const

        void UpdatePairs[T](T* callback)
        void Query[T](T* callback, const b2AABB& aabb) const
        void RayCast[T](T* callback, const b2RayCastInput& input) const

        int32 GetTreeHeight() const
        int32 GetTreeBalance() const
        float32 GetTreeQuality() const
        void ShiftOrigin(const b2Vec2& newOrigin)
