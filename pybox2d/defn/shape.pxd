from defn.math cimport *
from defn.collision cimport (b2RayCastInput, b2RayCastOutput, b2AABB)


DEF b2_maxPolygonVertices = 8


cdef extern from "b2Shape.h":
    cdef struct b2MassData:
        float32 mass
        b2Vec2 center
        float32 I

    ctypedef enum b2ShapeType "b2Shape::Type":
        ShapeType_circle "b2Shape::e_circle"
        ShapeType_edge "b2Shape::e_edge"
        ShapeType_polygon "b2Shape::e_polygon"
        ShapeType_chain "b2Shape::e_chain"
        ShapeType_typeCount "b2Shape::e_typeCount"

    cdef cppclass b2Shape:
        # virtual ~b2Shape()
        # virtual b2Shape* Clone(b2BlockAllocator* allocator) const
        b2ShapeType GetType() const
        int32 GetChildCount() const
        bool TestPoint(const b2Transform& xf, const b2Vec2& p) const
        bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                     const b2Transform& transform, int32 childIndex)
        void ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex)
        void ComputeMass(b2MassData* massData, float32 density)
        b2ShapeType m_type
        float32 m_radius


cdef extern from "b2PolygonShape.h":
    cdef cppclass b2PolygonShape(b2Shape):
        b2PolygonShape()
        # b2Shape* Clone(b2BlockAllocator* allocator) const
        int32 GetChildCount() const
        void Set(const b2Vec2* points, int32 count)
        void SetAsBox(float32 hx, float32 hy)
        void SetAsBox(float32 hx, float32 hy, const b2Vec2& center, float32 angle)
        bool TestPoint(const b2Transform& transform, const b2Vec2& p) const
        bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                const b2Transform& transform, int32 childIndex) const
        void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const
        void ComputeMass(b2MassData* massData, float32 density) const
        int32 GetVertexCount() const
        const b2Vec2& GetVertex(int32 index) const
        bool Validate() const
        b2Vec2 m_centroid
        b2Vec2 m_vertices[b2_maxPolygonVertices]
        b2Vec2 m_normals[b2_maxPolygonVertices]
        int32 m_count


cdef extern from "b2CircleShape.h":
    cdef cppclass b2CircleShape(b2Shape):
        b2CircleShape()
        # b2Shape* Clone(b2BlockAllocator* allocator) const
        int32 GetChildCount() const
        bool TestPoint(const b2Transform& transform, const b2Vec2& p) const
        bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                    const b2Transform& transform, int32 childIndex) const
        void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const
        void ComputeMass(b2MassData* massData, float32 density) const
        int32 GetSupport(const b2Vec2& d) const
        const b2Vec2& GetSupportVertex(const b2Vec2& d) const
        int32 GetVertexCount() const
        const b2Vec2& GetVertex(int32 index) const
        b2Vec2 m_p


cdef extern from "b2EdgeShape.h":
    cdef cppclass b2EdgeShape(b2Shape):
        b2EdgeShape()
        void Set(const b2Vec2& v1, const b2Vec2& v2)
        # b2Shape* Clone(b2BlockAllocator* allocator) const
        int32 GetChildCount() const
        bool TestPoint(const b2Transform& transform, const b2Vec2& p) const
        bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                    const b2Transform& transform, int32 childIndex) const
        void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const
        void ComputeMass(b2MassData* massData, float32 density) const
        b2Vec2 m_vertex1, m_vertex2
        b2Vec2 m_vertex0, m_vertex3
        bool m_hasVertex0, m_hasVertex3
