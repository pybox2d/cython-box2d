from defn.math cimport *
from defn.settings cimport *
from defn.shape cimport (b2Shape, b2CircleShape, b2PolygonShape, b2EdgeShape)

DEF b2_maxManifoldPoints = 2


cdef extern from "b2Collision.h":
    cdef enum b2ContactFeatureType:
        e_vertex = 0
        e_face = 1
    cdef struct b2ContactFeature:
        uint8 indexA
        uint8 indexB
        uint8 typeA
        uint8 typeB

    cdef union b2ContactID:
        b2ContactFeature cf
        uint32 key

    cdef struct b2ManifoldPoint:
        b2Vec2 localPoint
        float32 normalImpulse
        float32 tangentImpulse
        b2ContactID id

    cdef enum b2ManifoldType:
        e_circles
        e_faceA
        e_faceB

    cdef struct b2Manifold:
        b2ManifoldPoint points[b2_maxManifoldPoints]
        b2Vec2 localNormal
        b2Vec2 localPoint
        b2ManifoldType type
        int32 pointCount

    cdef struct b2WorldManifold:
        void Initialize(const b2Manifold* manifold,
                        const b2Transform& xfA, float32 radiusA,
                        const b2Transform& xfB, float32 radiusB)

        b2Vec2 normal
        b2Vec2 *points  # [b2_maxManifoldPoints]
        float32 *separations  # [b2_maxManifoldPoints]


    cdef enum b2PointState:
        b2_nullState,
        b2_addState,
        b2_persistState,
        b2_removeState


    cdef void b2GetPointStates(b2PointState *state1, # [b2_maxManifoldPoints],
                               b2PointState *state2, # [b2_maxManifoldPoints],
                               const b2Manifold* manifold1,
                               const b2Manifold* manifold2)


    cdef struct b2ClipVertex:
        b2Vec2 v
        b2ContactID id


    cdef struct b2RayCastInput:
        b2Vec2 p1, p2
        float32 maxFraction


    cdef struct b2RayCastOutput:
        b2Vec2 normal
        float32 fraction


    cdef cppclass b2AABB:
        bool IsValid() const
        b2Vec2 GetCenter() const

        b2Vec2 GetExtents() const
        float32 GetPerimeter() const
        void Combine(const b2AABB& aabb)
        void Combine(const b2AABB& aabb1, const b2AABB& aabb2)
        bool Contains(const b2AABB& aabb) const
        bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const
        b2Vec2 lowerBound
        b2Vec2 upperBound

    cdef void b2CollideCircles(b2Manifold* manifold,
                               const b2CircleShape* circleA, const b2Transform& xfA,
                               const b2CircleShape* circleB, const b2Transform& xfB)


    cdef void b2CollidePolygonAndCircle(b2Manifold* manifold, const b2PolygonShape*
                                        polygonA, const b2Transform& xfA, const
                                        b2CircleShape* circleB, const b2Transform&
                                        xfB)


    cdef void b2CollidePolygons(b2Manifold* manifold,
                                const b2PolygonShape* polygonA, const b2Transform& xfA,
                                const b2PolygonShape* polygonB, const b2Transform& xfB)


    cdef void b2CollideEdgeAndCircle(b2Manifold* manifold,
                                     const b2EdgeShape* polygonA, const b2Transform& xfA,
                                     const b2CircleShape* circleB, const b2Transform& xfB)


    cdef void b2CollideEdgeAndPolygon(b2Manifold* manifold,
                                      const b2EdgeShape* edgeA, const b2Transform& xfA,
                                      const b2PolygonShape* circleB, const b2Transform& xfB)


    cdef int32 b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
                                   const b2Vec2& normal, float32 offset, int32 vertexIndexA)



    cdef bool b2TestOverlap(const b2Shape* shapeA, int32 indexA,
                            const b2Shape* shapeB, int32 indexB,
                            const b2Transform& xfA, const b2Transform& xfB)


cdef extern from "b2WorldCallbacks.h":
    # TODO this belongs in world_callbacks.pxd, but i'm having trouble with
    #      DEF cimports
    cdef cppclass b2ContactImpulse:
        float32 normalImpulses[b2_maxManifoldPoints]
        float32 tangentImpulses[b2_maxManifoldPoints]
        int32 count
